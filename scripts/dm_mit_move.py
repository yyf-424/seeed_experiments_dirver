#!/usr/bin/env python3
"""dm_mit_move.py —— 真机 MIT 力控模式下的"限速摆动"工具（安全第一）。

    # 首次：小幅验证（只动 joint2，0.05 rad）
    ./scripts/dm_mit_move.py --set "joint2=-0.35"

    # 摆动几个姿态（每个目标都会按 slew 斜坡逼近，绝不阶跃）
    ./scripts/dm_mit_move.py --poses "joint1=0.3;joint1=0.3,joint2=-0.5;joint1=0"

为什么不能用"直接发目标"的方式（仿真已量化）：
    MuJoCo + 与真机相同的控制律（200 Hz、逐关节力矩限幅、纯 MIT 无重力补偿）实测：
      · 目标用 2 rad/s 斜坡 → 峰值角速度 2.0~2.9 rad/s（低于警告线 4.8）
      · 目标用**阶跃** → joint6 峰值 **25 rad/s**，会立即触发安全节点 6 rad/s 急停
    所以本脚本只做"斜坡逼近"，并且每一步都：
      ① 读当前 /joint_states 作为起点（不假设手臂在哪）
      ② 由控制器按统一目标斜率逼近
      ③ 全程盯 /b601_safety/status：检测到急停就停止发目标并退出

接口：
    发布  /b601_target_arbiter/target        b601_target_interfaces/TargetFrame
    订阅  /joint_states                       取当前位形
    订阅  /b601_safety/status                 安全状态（非 NORMAL 即中止）
"""

from __future__ import annotations

import argparse
import sys
import time
from math import isfinite

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from project_config import load_project_config
from b601_demos.target_client import TargetClient

PROJECT = load_project_config()
SETTINGS = PROJECT["tools"]["dm_mit_move"]

# 安全状态话题是 transient_local（锁存）：晚加入的订阅者也应立刻拿到"当前状态"。
# 用默认 volatile 订阅会收不到（状态不变时发布者不会重复发）—— 2026-11-21 实测踩过。
STATUS_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)


ARM = PROJECT["robot"]["arm_joints"]
GRIPPER = PROJECT["robot"]["gripper_joint"]
JOINTS = (*ARM, GRIPPER)


def load_limits():
    j = PROJECT["joints"]
    return ({n: float(j[n]["min_pos"]) for n in JOINTS},
            {n: float(j[n]["max_pos"]) for n in JOINTS})


class MitMover(Node):
    def __init__(self, rate_hz: float,
                 tolerance: float, debug_topic: str,
                 settle: float = SETTINGS["settle"], *, gravity_source=None, gravity_api=None):
        super().__init__("b601_mit_move")
        self.targets = TargetClient(
            self, "manual", gravity_source=gravity_source, gravity_api=gravity_api)
        self.current: dict[str, float] = {}
        self.current_vel: dict[str, float] = {}
        self.safety = "UNKNOWN"
        self.controller_seen = False
        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.create_subscription(
            DiagnosticStatus, "/b601_safety/status", self._on_status, STATUS_QOS)
        # MIT 控制器激活后才会以 ~10 Hz 发布 ~/debug。用它判断"控制器真的在跑"，
        # 否则目标发出去没人执行，而关节速度恒为 0 会被误判成"已到位"（2026-11-21 踩过）。
        self.create_subscription(
            Float64MultiArray, debug_topic, self._on_debug, 10)
        self.rate = rate_hz
        self.slew = PROJECT["target_slew"]["revolute"]
        self.tol = tolerance
        self.settle = settle
        self.lo, self.hi = load_limits()

    def _on_debug(self, _msg: Float64MultiArray) -> None:
        self.controller_seen = True

    def _on_state(self, msg: JointState) -> None:
        self.current = dict(zip(msg.name, msg.position))
        if msg.velocity and len(msg.velocity) == len(msg.name):
            self.current_vel = dict(zip(msg.name, msg.velocity))

    def _on_status(self, msg: DiagnosticStatus) -> None:
        for kv in msg.values:
            if kv.key == "state":
                self.safety = kv.value

    def wait_ready(self, timeout: float = SETTINGS["ready_timeout_sec"]) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=SETTINGS["poll_sec"])
            if (all(n in self.current for n in JOINTS) and self.safety != "UNKNOWN"
                    and self.controller_seen):
                return True
        if not self.controller_seen:
            self.get_logger().error(
                "等不到 MIT 控制器的心跳（~/debug 无数据）—— 控制器可能没激活，"
                "此时发目标不会被执行；请确认 controllers:=... b601_mit_controller 且状态 active")
        return False

    def goto(self, target: dict[str, float], label: str,
             velocities: dict[str, float], tff: dict[str, float]) -> bool:
        start = {n: self.current[n] for n in ARM}
        want = {n: target.get(n, start[n]) for n in ARM}
        if GRIPPER in target or GRIPPER in velocities or GRIPPER in tff:
            current_gripper = min(self.hi[GRIPPER], max(self.lo[GRIPPER], self.current[GRIPPER]))
            want[GRIPPER] = target.get(GRIPPER, current_gripper)

        # ---- 前置校验：不合法就根本不动 ----
        for n, v in want.items():
            if not (self.lo[n] - 1e-9 <= v <= self.hi[n] + 1e-9):
                self.get_logger().error(f"{label}：{n}={v:+.3f} 超出限位 [{self.lo[n]}, {self.hi[n]}]，拒发")
                return False
        if self.safety == "EMERGENCY_STOP":
            self.get_logger().error(f"{label}：当前是急停状态，拒发（先调用 clear_estop）")
            return False
        if self.safety == "WARNING":
            self.get_logger().warn(
                f"{label}：当前安全状态 WARNING —— 按设计警告不阻止驱动，继续（只记录）")

        move = max(abs(want[n] - start[n]) for n in ARM)
        self.get_logger().info(
            f"{label}：最大位移 {move:.3f} rad，斜坡 {self.slew:.2f} rad/s → 约 {move / max(self.slew, 1e-3):.1f}s")

        dt = 1.0 / self.rate
        t0 = time.monotonic()
        last_log = 0.0
        err0 = move
        self._quiet = 0.0
        self._frozen = 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.0)
            # 只有急停才中止；WARNING 按设计允许继续驱动（安全节点的语义就是"警告不停机"）
            if self.safety == "EMERGENCY_STOP":
                self.get_logger().error(f"{label}：急停触发 —— 立即停止发送目标")
                return False
            if self.safety == "WARNING" and not getattr(self, "_warned", False):
                self._warned = True
                self.get_logger().warn(f"{label}：安全状态 WARNING（继续，不停机）")
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(want)
            msg.position = list(want.values())
            msg.velocity = [velocities.get(n, 0.0) for n in msg.name] if velocities else []
            msg.effort = [tff.get(n, 0.0) for n in msg.name] if tff else []
            self.targets.publish(msg)

            elapsed = time.monotonic() - t0
            if elapsed - last_log > SETTINGS["log_interval_sec"]:
                last_log = elapsed
                errs = {n: abs(want[n] - self.current[n]) for n in ARM}
                worst = max(errs, key=errs.get)
                self.get_logger().info(
                    f"  {elapsed:5.1f}s  最大剩余误差 {errs[worst]:.3f} rad（{worst}）")
            err_now = max(abs(want[n] - self.current[n]) for n in ARM)
            vel_now = max((abs(self.current_vel.get(n, 0.0)) for n in ARM), default=0.0)
            # 至少等待按统一斜率走完全程的时间，再检查稳态误差。
            reached = elapsed >= move / self.slew
            # 冻结检测：误差很大却一动不动（控制器没接管 / 目标没被执行）——
            # 必须报错而不是报成功。纯 MIT 的重力下垂是"误差小且静止"，
            # 与"误差大且静止"是两回事（2026-11-21 在仿真里踩到过假成功）。
            # 冻结判据要看"有没有进展"：纯 MIT 到位后仍有 0.1 rad 左右的下垂，
            # 那不是冻结；只有"误差几乎没变过"才是目标没被执行。
            # ⚠ reached 之后误差不再变化是**正常的**（下垂），所以 reached 时不判冻结。
            if (not reached) and err_now > max(
                self.tol + SETTINGS["no_progress_margin_rad"],
                SETTINGS["no_progress_initial_error_fraction"] * err0,
            ) and vel_now < SETTINGS["no_progress_speed_rad_s"]:
                self._frozen = getattr(self, "_frozen", 0.0) + dt
            else:
                self._frozen = 0.0
            if getattr(self, "_frozen", 0.0) > SETTINGS["no_progress_timeout_sec"]:
                self.get_logger().error(
                    f"{label}：误差还有 {err_now:.3f} rad 但 1.2 s 内没有任何运动 —— "
                    "目标没被执行（控制器未接管？），判失败")
                return False
            if err_now <= self.tol:
                quiet = getattr(self, "_quiet", 0.0) + dt
            elif vel_now < SETTINGS["settled_speed_rad_s"]:
                # 纯 MIT（无重力补偿）稳态必然有下垂，"误差=0"不可能达到；
                # 因此"速度已经静止"也算到位，只是把稳态误差如实报出来。
                quiet = getattr(self, "_quiet", 0.0) + dt
            else:
                quiet = 0.0
            self._quiet = quiet
            # 到位判据必须两个条件同时成立：
            #   ① 目标命令已经走完全程（reached）—— 否则误把"斜坡中途"当成"稳态"报出去；
            #   ② 误差进入容差，或者"已经静止 settle 秒"（纯 MIT 稳态下垂，误差下不去）。
            # "静止即到位"还要求确实走过一大半路程，否则只读模式/未接管时会假成功。
            settled = quiet > self.settle and err_now <= max(self.tol, SETTINGS["settled_initial_error_fraction"] * err0)
            gripper_reached = (
                GRIPPER not in target or
                abs(self.current[GRIPPER] - target[GRIPPER])
                <= PROJECT["validation"]["mit_five_poses"]["criteria"]["gripper_max_error_m"]
            )
            done = reached and (err_now <= self.tol or settled) and gripper_reached
            if done or elapsed > SETTINGS["deadline_scale"] * max(
                move / max(self.slew, PROJECT["implementation"]["mit"]["slew_min"]),
                SETTINGS["deadline_floor_sec"],
            ) + SETTINGS["deadline_grace_sec"]:
                self.get_logger().info(
                    f"{label}：结束，稳态最大误差 {err_now:.3f} rad、最大速度 {vel_now:.3f} rad/s"
                    "（纯 MIT 无重力补偿时误差≈重力下垂量）")
                if not gripper_reached:
                    self.get_logger().error(f"{label}：夹爪未到位，实测 {self.current[GRIPPER]:.4f} m")
                    return False
                return True
            time.sleep(dt)


def parse_pose(spec: str) -> dict[str, float]:
    """'joint2=-0.35' → {'joint2': -0.35}；支持逗号分隔多个关节"""
    out = {}
    for part in spec.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise ValueError(f"无法解析 '{part}'（应为 joint2=-0.35）")
        k, v = part.split("=", 1)
        k = k.strip()
        if k not in JOINTS:
            raise ValueError(f"未知关节 '{k}'")
        value = float(v)
        if not isfinite(value):
            raise ValueError(f"{k} 的目标值必须为有限数")
        out[k] = value
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="MIT 力控模式的限速摆动工具")
    ap.add_argument("--controller", default="b601_mit_arm",
                    help="默认诊断话题所属的 MIT 控制器")
    ap.add_argument("--debug-topic",
                    help="MIT 控制器心跳（激活后 ~10 Hz 发布），用于确认控制器真在跑")
    ap.add_argument("--set", default="", help="目标姿态，例：joint2=-0.35,joint5=0.2")
    ap.add_argument("--poses", default="", help="多个姿态用 ; 分隔，依次执行")
    ap.add_argument("--vel", default="", help="目标速度，例如 joint1=0.05（rad/s，夹爪 m/s）")
    ap.add_argument("--tff", default="", help="任务前馈，例如 joint1=0.2（N·m，夹爪 N）")
    ap.add_argument("--gravity-source", choices=("internal", "external"), default="internal")
    ap.add_argument("--gravity-api", default="", help="外部 Python API 位置，格式 module:factory")
    ap.add_argument("--rate", type=float, default=SETTINGS["rate"], help="目标发布频率 Hz")
    ap.add_argument("--tolerance", type=float, default=SETTINGS["tolerance"],
                    help="到位判据 rad（只用于判到位，不会让目标跳步；纯 MIT 有下垂，别设太小）")
    ap.add_argument("--settle", type=float, default=SETTINGS["settle"],
                    help="目标命令走完后，实测『静止』多少秒才算一次姿态结束（默认 1.0 s）。"
                         "纯 MIT 稳态误差不会到 0，靠这个保证报出来的是稳态值而不是斜坡中途值")
    args = ap.parse_args()

    if not args.set and not args.poses:
        print("至少要给 --set 或 --poses", file=sys.stderr)
        return 2

    try:
        velocities = parse_pose(args.vel)
        tff = parse_pose(args.tff)
        specs = [p for p in args.poses.split(";") if p.strip()] if args.poses else [args.set]
        targets = [parse_pose(spec) for spec in specs]
    except ValueError as error:
        ap.error(str(error))

    rclpy.init()
    debug_topic = args.debug_topic or f"/{args.controller}/debug"
    try:
        node = MitMover(args.rate, args.tolerance, debug_topic, args.settle,
                        gravity_source=args.gravity_source, gravity_api=args.gravity_api)
    except ValueError as error:
        rclpy.try_shutdown()
        ap.error(str(error))
    try:
        if not node.wait_ready():
            node.get_logger().error("等不到 /joint_states 或 /b601_safety/status（bringup 起来了吗？）")
            return 1
        node.get_logger().info(
            "起始位形：" + ", ".join(f"{n}={node.current[n]:+.3f}" for n in ARM)
            + f"；安全状态 {node.safety}；斜坡 {node.slew:.2f} rad/s")

        node.targets.select(SETTINGS["ready_timeout_sec"])
        for i, (spec, target) in enumerate(zip(specs, targets), 1):
            if not node.goto(target, f"[{i}/{len(specs)}] {spec}", velocities, tff):
                node.get_logger().error("中止后续姿态")
                return 1
        node.get_logger().info("全部姿态完成")
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.targets.close(SETTINGS["ready_timeout_sec"])
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
