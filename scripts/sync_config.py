#!/usr/bin/env python3
"""sync_config.py —— 把「唯一真值」同步到全项目所有派生配置文件。

    ./scripts/sync_config.py            # 生成 / 更新全部派生文件
    ./scripts/sync_config.py --check    # 只检查是否已同步（verify.sh 用；不一致退出码 1）
    ./scripts/sync_config.py --diff     # 打印差异（不写盘）

真值：ros2_ws/src/b601_description/config/b601_project.yaml

生成控制器、安全监控、硬件、初始位置和 MoveIt 所需的参数文件，并同步 SRDF 命名姿态。
xacro、演示节点及命令行工具直接读取真值。MuJoCo 模型由 gen_mjcf.py 从 URDF 和场景配置生成。

为什么用「生成」而不是让每个包都去读同一个 yaml：
    controller_manager / MoveIt 只认标准的 yaml 参数文件格式，必须落盘；
    生成 + 校验（而不是手写）才能保证「改一处、全同步、且可验证」。
"""

from __future__ import annotations

import argparse
from decimal import Decimal
import difflib
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from project_config import expanded_position_range, safety_position_limits

REPO_ROOT = Path(__file__).resolve().parent.parent
WS_SRC = REPO_ROOT / "ros2_ws" / "src"
TRUTH = WS_SRC / "b601_description" / "config" / "b601_project.yaml"

GENERATED_NOTE = (
    "# ============================================================================\n"
    "# ⚠ 本文件由 scripts/sync_config.py 自动生成，请勿手工编辑。\n"
    "#    要改参数请改：ros2_ws/src/b601_description/config/b601_project.yaml\n"
    "#    改完执行：  ./scripts/sync_config.py\n"
    "# ============================================================================\n"
)


# --------------------------------------------------------------------------- 校验
def validate(cfg: dict) -> list[str]:
    """配置自身的一致性检查。返回错误列表（空 = 通过）。"""
    errors: list[str] = []
    joints = cfg.get("joints", {})
    robot = cfg.get("robot", {})
    safety = cfg.get("safety", {})

    if not joints:
        errors.append("joints 段为空")
        return errors

    arm = robot.get("arm_joints", [])
    if len(arm) != 6:
        errors.append(f"robot.arm_joints 应为 6 个，实际 {len(arm)}")
    for name in arm + [robot.get("gripper_joint", ""), robot.get("mimic_joint", "")]:
        if name and name not in joints:
            errors.append(f"robot 段引用了 joints 里不存在的关节 {name}")

    for name, j in joints.items():
        for key in ("min_pos", "max_pos", "max_velocity", "startup"):
            if key not in j:
                errors.append(f"关节 {name} 缺 {key}")
        if j.get("min_pos", 0) >= j.get("max_pos", 0):
            errors.append(f"关节 {name} 限位区间非法：[{j.get('min_pos')}, {j.get('max_pos')}]")
        if j.get("max_velocity", 0) <= 0:
            errors.append(f"关节 {name} 的 max_velocity 必须 > 0")
        torque_nm = j.get("max_motor_torque_nm")
        if type(torque_nm) not in (int, float) or not math.isfinite(torque_nm) or torque_nm <= 0:
            errors.append(f"关节 {name} 的 max_motor_torque_nm 必须为有限正数")
        start = j.get("startup")
        if start is not None and not (j["min_pos"] <= start <= j["max_pos"]):
            errors.append(f"关节 {name} 的启动姿态 {start} 不在限位内")
        jump = j.get("feedback_jump_limit")
        if jump is None:
            errors.append(f"关节 {name} 缺 feedback_jump_limit（防放宽角速度时守卫也跟着放松）")
        elif float(jump) <= 0:
            errors.append(f"关节 {name} 的 feedback_jump_limit 必须 > 0（当前 {jump}）")

        if j.get("control"):
            motor = j.get("motor")
            if not motor:
                errors.append(f"关节 {name} 有命令接口但没有 motor 段")
            else:
                if motor.get("scale", 0) == 0:
                    errors.append(f"关节 {name} 的 motor.scale 不能为 0")
                if motor.get("direction") not in (1.0, -1):
                    errors.append(f"关节 {name} 的 motor.direction 必须是 ±1")

    # 安全阈值自洽
    vel = safety.get("velocity", {})
    if not 0.0 <= vel.get("warn_ratio", 0) < 1.0:
        errors.append("safety.velocity.warn_ratio 必须在 [0,1) 内")
    if "gripper_estop_m_s" in vel:
        speed = vel["gripper_estop_m_s"]
        if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
            errors.append("safety.velocity.gripper_estop_m_s 必须为有限正数")
    angle = safety.get("angle", {})
    if angle.get("warn_margin", 0) <= 0:
        errors.append("safety.angle.warn_margin 必须 > 0")
    fraction = angle.get("position_expansion_fraction")
    fraction_valid = type(fraction) in (int, float) and math.isfinite(fraction) and fraction >= 0
    if not fraction_valid:
        errors.append("safety.angle.position_expansion_fraction 必须为有限非负数")
    measured = angle.get("measured_position_ranges", {})
    if set(measured) != {*arm, robot.get("gripper_joint")}:
        errors.append("safety.angle.measured_position_ranges 必须覆盖六轴与主动指")
    for name, bounds in measured.items():
        if (not isinstance(bounds, list) or len(bounds) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in bounds)
                or bounds[0] >= bounds[1]):
            errors.append(f"{name} 的实测位置区间无效")
            continue
        if name not in joints:
            continue
        joint = joints[name]
        lower, measured_upper = bounds
        command_upper = joint.get("calibrated_command_max_pos", measured_upper)
        if (type(command_upper) not in (int, float) or not math.isfinite(command_upper)
                or command_upper <= lower):
            errors.append(f"{name} 的指令位置校准上限无效")
            continue
        if fraction_valid and command_upper > expanded_position_range(bounds, fraction)[1]:
            errors.append(f"{name} 的指令位置校准上限超出急停边界")
        if joint.get("min_pos", lower) < lower or joint.get("max_pos", command_upper) > command_upper:
            errors.append(f"{name} 的指令位置区间超出实测或校准行程")
    low = safety.get("low_stiffness")
    if low is not None:
        speed = low.get("max_velocity")
        if type(speed) not in (int, float) or not math.isfinite(speed) or speed <= 0:
            errors.append("safety.low_stiffness.max_velocity 必须为有限正数")
    estop = safety.get("estop", {})
    hw_name = estop.get("hardware_component")
    if hw_name != robot.get("ros2_control_name"):
        errors.append(
            f"safety.estop.hardware_component={hw_name} 与 robot.ros2_control_name="
            f"{robot.get('ros2_control_name')} 不一致，急停会找不到硬件"
        )
    for cname in estop.get("deactivate_controllers", []) or []:
        if cname == "*":
            continue  # 通配：停掉当时所有 active 控制器
        known = {c.get("name") for c in cfg.get("controllers", {}).values() if isinstance(c, dict)}
        if cname not in known:
            errors.append(f"safety.estop.deactivate_controllers 里的 {cname} 不是已定义的控制器")

    errors.extend(validate_effort_scale(cfg))
    errors.extend(validate_mit(cfg))
    return errors


def validate_effort_scale(cfg: dict) -> list[str]:
    """effort_scale：关节 effort ↔ 电机力矩的换算系数（revolute=1，prismatic=等效齿轮半径）。"""
    errors: list[str] = []
    for name, j in cfg.get("joints", {}).items():
        if not j.get("control"):
            continue
        motor = j.get("motor") or {}
        es = motor.get("effort_scale")
        if es is None:
            errors.append(f"关节 {name} 的 motor 缺 effort_scale（revolute 给 1.0，prismatic 给等效齿轮半径）")
            continue
        if float(es) <= 0:
            errors.append(f"关节 {name} 的 effort_scale 必须 > 0（当前 {es}）")
        # 当前机械结构下 effort_scale 与 scale 数值相同；若不同，说明传动比模型变了，必须是刻意的
        if abs(float(es) - float(motor.get("scale", 1.0))) > 1e-12:
            errors.append(
                f"关节 {name} 的 effort_scale={es} 与 scale={motor.get('scale')} 不同 —— "
                f"这不是错误，但必须确认是刻意改的（力矩换算与位置换算的传动比不一致）"
            )
    return errors


def validate_mit(cfg: dict) -> list[str]:
    """MIT 实例表自检：每个可控关节必须且只能属于一个实例（覆盖门禁的真值侧）。"""
    errors: list[str] = []
    mit = cfg.get("controllers", {}).get("mit")
    if not mit:
        errors.append("缺 controllers.mit 段（MIT 力控实例表）")
        return errors
    joints_cfg = cfg.get("joints", {})
    profiles = mit.get("profiles") or {}
    instances = mit.get("instances") or {}
    if not instances:
        errors.append("control.mit.instances 为空")
        return errors

    # 覆盖：每个有命令接口的关节必须且只能属于一个实例；mimic 关节不允许出现
    controllable = [n for n, j in joints_cfg.items() if j.get("control")]
    owner: dict[str, list[str]] = {}
    for iname, inst in instances.items():
        for jname in inst.get("joints", []):
            owner.setdefault(jname, []).append(iname)
    for jname, owners in owner.items():
        if jname not in joints_cfg:
            errors.append(f"MIT 实例表引用了不存在的关节 {jname}")
        elif any(o for o in owners if len(owners) > 1):
            errors.append(f"关节 {jname} 被多个 MIT 实例声明：{owners}（命令接口会抢，运行期报 already claimed）")
    for jname in controllable:
        if jname not in owner:
            errors.append(f"关节 {jname} 没有归属任何 MIT 实例（裸关节 → 力矩恒 0）")
    for jname, j in joints_cfg.items():
        if not j.get("control") and jname in owner:
            errors.append(f"mimic 关节 {jname} 出现在 MIT 实例表里（它没有命令接口）")

    # 两条启动路径共用实例表，每种配置都应覆盖相同的关节。
    for iname, inst in instances.items():
        if not inst.get("plugin"):
            errors.append(f"MIT 实例 {iname} 缺 plugin")
        for key in ("profile", "gravity_profile"):
            pname = inst.get(key)
            if pname is None and key == "gravity_profile":
                continue
            if pname not in profiles:
                errors.append(f"MIT 实例 {iname} 引用了不存在的 {key} {pname}")
                continue
            profile = profiles[pname]
            if key == "gravity_profile":
                ratios = profile.get("hold_gain_ratios", {})
                for kind in ("kp", "kd"):
                    ratio = ratios.get(kind)
                    if type(ratio) not in (int, float) or not math.isfinite(ratio) or not 0 <= ratio <= 1:
                        errors.append(f"MIT 配置 {pname}.hold_gain_ratios.{kind} 缺失或无效")
            for joint in inst.get("joints", []):
                gains = profile["per_joint"].get(joint, {})
                required = ("kp_move", "kd_move")
                if key == "gravity_profile":
                    required += ("gravity_joint_scale",)
                for gain in required:
                    value = gains.get(gain)
                    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                        errors.append(f"MIT 配置 {pname}.{joint}.{gain} 缺失或无效")
    if not instances.get("b601_mit_arm", {}).get("gravity_profile"):
        errors.append("MIT 手臂缺少重力补偿配置")
    if not mit.get("shared", {}).get("gravity_params"):
        errors.append("controllers.mit.shared.gravity_params 为空")
    return errors


# --------------------------------------------------------------------- 渲染各文件
def dump_yaml(data: dict) -> str:
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True, default_flow_style=False)


def render_initial_positions(cfg: dict) -> str:
    positions = {name: float(j["startup"]) for name, j in cfg["joints"].items()}
    head = (
        GENERATED_NOTE
        + "# 上电/启动时的关节角（rad / m）。被 ros2_control.xacro 的 state_interface\n"
        "# initial_value 使用，决定\n"
        "#   - mock 后端启动后 /joint_states 的初值\n"
        "#   - 真机后端上电时认为关节在哪儿（必须与实测一致，否则第一次写命令会跳）\n"
        "#   - 安全节点判断「是否已离开启动位形」的基线\n"
    )
    return head + dump_yaml({"initial_positions": positions})


def _controller_block(name: str, ctrl: dict) -> tuple[dict, dict]:
    """返回 (controller_manager 里的声明, 控制器自身的 ros__parameters)。"""
    declaration = {"type": ctrl["type"]}
    params: dict = {"joints": list(ctrl.get("joints", []))}
    if ctrl.get("command_interfaces"):
        params["command_interfaces"] = list(ctrl["command_interfaces"])
    if ctrl.get("state_interfaces"):
        params["state_interfaces"] = list(ctrl["state_interfaces"])
    params.update(ctrl.get("params", {}))
    return declaration, params


def render_controllers(cfg: dict) -> str:
    ctrls = cfg["controllers"]
    cm_params: dict = {"update_rate": ctrls["update_rate"]}
    body: dict = {}

    broadcaster = ctrls["joint_state_broadcaster"]
    cm_params["joint_state_broadcaster"] = {"type": broadcaster["type"]}
    body["joint_state_broadcaster"] = {"ros__parameters": dict(broadcaster.get("params", {}))}

    for key in ("arm", "gripper"):
        ctrl = ctrls[key]
        declaration, params = _controller_block(ctrl["name"], ctrl)
        cm_params[ctrl["name"]] = declaration
        body[ctrl["name"]] = {"ros__parameters": params}

    head = (
        GENERATED_NOTE
        + "# 位置模式控制器（hardware:=mock|dm，command_mode=pos_vel 时用这套）\n"
        + "# 力控模式用 b601_mit_controllers.yaml\n"
    )
    data = {"controller_manager": {"ros__parameters": cm_params}}
    data.update(body)
    return head + dump_yaml(data)


def render_joint_limits(cfg: dict) -> str:
    moveit = cfg["moveit"]
    limits: dict = {}
    for name, j in cfg["joints"].items():
        if not j.get("control"):
            continue  # mimic 关节不参与规划
        limits[name] = {
            "has_velocity_limits": True,
            "max_velocity": min(float(j["max_velocity"]), float(cfg["target_slew"][j["type"]])),
            "has_acceleration_limits": False,
            "has_deceleration_limits": False,
        }
    data = {
        "default_velocity_scaling_factor": moveit["default_velocity_scaling_factor"],
        "default_acceleration_scaling_factor": moveit["default_acceleration_scaling_factor"],
        "joint_limits": limits,
    }
    head = (
        GENERATED_NOTE
        + "# MoveIt 规划限速：时间参数化的依据。速度上限 = URDF limit.velocity（同一个来源）。\n"
    )
    return head + dump_yaml(data)


def render_safety_params(cfg: dict) -> str:
    safety = cfg["safety"]
    joints = cfg["joints"]
    names = list(joints.keys())

    angle = safety["angle"]
    default_margin = float(angle["warn_margin"])
    active = cfg["robot"]["gripper_joint"]
    mimic = cfg["robot"]["mimic_joint"]
    gripper_speed = safety["velocity"].get("gripper_estop_m_s")
    velocities = {name: float(joints[name]["max_velocity"]) for name in names}
    if gripper_speed is not None:
        velocities[active] = velocities[mimic] = float(gripper_speed)
    bounds = safety_position_limits(cfg)
    params: dict = {
        "joint_names": names,
        "pos_min": [bounds[n][0] for n in names],
        "pos_max": [bounds[n][1] for n in names],
        "vel_max": list(velocities.values()),
        "startup_pose": [float(joints[n]["startup"]) for n in names],
        # 逐关节的软警告配置（夹爪关掉"接近限位/离开启动位形"，否则开合会一直挂警告）
        "warn_pos_margin": [float(joints[n].get("warn_pos_margin", default_margin))
                            for n in names],
        "warn_near_limit": [bool(joints[n].get("warn_near_limit", True)) for n in names],
        "warn_pose_deviation_joint": [bool(joints[n].get("warn_pose_deviation", True))
                                      for n in names],
        "evaluate_rate_hz": safety["evaluate_rate_hz"],
        "publish_rate_hz": safety["publish_rate_hz"],
        "feedback_timeout_sec": safety["feedback_timeout_sec"],
        "estop_on_feedback_timeout": safety["estop_on_feedback_timeout"],
        "startup_grace_sec": safety["startup_grace_sec"],
        "warn_vel_ratio": safety["velocity"]["warn_ratio"],
        "warn_on_pose_deviation": safety["pose"]["warn_on_deviation"],
        "warn_pose_threshold": safety["pose"]["warn_deviation"],
        "joint_states_topic": safety["joint_states_topic"],
        "joint_state_depth": safety["joint_state_depth"],
        "minimum_evaluate_rate_hz": safety["minimum_evaluate_rate_hz"],
        "minimum_publish_rate_hz": safety["minimum_publish_rate_hz"],
        "status_topic": safety["status_topic"],
        "clear_service": safety["clear_service"],
        "deactivate_controllers": list(safety["estop"].get("deactivate_controllers") or []),
        "deactivate_hardware": safety["estop"]["deactivate_hardware"],
        "hardware_component": safety["estop"]["hardware_component"],
        "service_timeout_sec": safety["estop"]["service_timeout_sec"],
        "service_discovery_timeout_sec": safety["estop"]["service_discovery_timeout_sec"],
        "latch": safety["estop"]["latch"],
    }
    if low := safety.get("low_stiffness"):
        arm_speed = float(low["max_velocity"])
        low_gripper_speed = (float(gripper_speed) if gripper_speed is not None
                             else arm_speed * abs(float(joints[active]["motor"]["scale"])))
        params["low_vel_max"] = [low_gripper_speed if n in (active, mimic) else arm_speed for n in names]
    data = {safety.get("node_name", "b601_safety_monitor"): {"ros__parameters": params}}
    head = (
        GENERATED_NOTE
        + "# 安全监控节点参数。两档共用实测外扩位置边界，速度上限按档位选择。\n"
        + "# 想永久调整阈值请改 b601_project.yaml，不要手改本文件。\n"
    )
    return head + dump_yaml(data)


# ---- MIT 力控（变刚度）：实例表 → 控制器 yaml ----
# 逐关节量必须按实例的 joints 顺序展开成数组；组级量（阈值/斜率/守卫）原样透传。
MIT_ARRAY_KEYS = (
    "kp_hold", "kd_hold", "kp_move", "kd_move", "friction",
    "kp_free", "kd_free", "free_enabled",
    "settle_band_min", "settle_band_max",
    "gravity_joint_scale", "gravity_joint_scale_free", "gravity_bias",
)
# 组级（标量）键透传白名单不需要维护：group 里除 state_interfaces 外都原样透传。


def _mit_params(cfg: dict, mit: dict, inst: dict, gravity_compensation: bool) -> dict:
    """把一个实例渲染成 ros__parameters。缺失的逐关节键**不输出**，交给控制器默认值。"""
    profiles = mit["profiles"]
    profile_name = inst["profile"]
    if gravity_compensation:
        profile_name = inst.get("gravity_profile", profile_name)
    prof = profiles[profile_name]
    per_joint = prof.get("per_joint") or {}
    group = dict(prof.get("group") or {})
    joints = list(inst["joints"])

    params: dict = {
        "joints": joints,
        "chain_joints": list(inst.get("chain_joints", joints)),
        "target_slew": [float(cfg["target_slew"][cfg["joints"][name]["type"]]) for name in joints],
        "state_interfaces": list(group.pop("state_interfaces", ["position", "velocity"])),
    }
    for key in MIT_ARRAY_KEYS:
        if key in group and not any(key in per_joint.get(jname, {}) for jname in joints):
            params[key] = group.pop(key)  # 保留兼容的组级标量
            continue
        values = []
        ok = True
        for jname in joints:
            v = per_joint.get(jname, {}).get(key, group.get(key))
            if v is None:
                # 组级给了标量就当全体同值；都没有就跳过这个键
                ok = False
                break
            values.append(float(v) if not isinstance(v, bool) else bool(v))
        group.pop(key, None)
        if ok and values:
            params[key] = values

    for key, val in group.items():
        params[key] = val

    ratios = prof.get("hold_gain_ratios")
    if ratios:
        for kind in ("kp", "kd"):
            factor = Decimal(str(ratios[kind]))
            params[f"{kind}_free"] = [float(Decimal(str(gain)) * factor)
                                      for gain in params[f"{kind}_move"]]
    params["gravity_params"] = [float(x) for x in mit["shared"]["gravity_params"]]
    return params


def mujoco_physical_gravity_params(cfg: dict) -> list[float]:
    model = ET.parse(WS_SRC / "b601_mujoco" / "models" / "b601_arm_physical.xml")
    # 与 generate_cpp_config.py 的 kGravityLinks 顺序一致。
    arm_count = len(cfg["robot"]["arm_joints"])
    links = [f"link{i}" for i in range(2, arm_count + 1)]
    links.extend(("end_link", "finger_left_link", "finger_right_link"))
    params = []
    for link in links:
        inertial = model.find(f".//body[@name='{link}']/inertial")
        mass = Decimal(inertial.attrib["mass"])
        center = [Decimal(value) for value in inertial.attrib["pos"].split()]
        params.extend([float(mass), *(float(mass * value) for value in center)])
    return params


def render_mit_controllers(cfg: dict, gravity_compensation: bool = False, *,
                           mujoco_physical: bool = False) -> str:
    """MIT 力控控制器 yaml（controller_manager 声明 + 每个实例的参数块）。"""
    mit = cfg["controllers"]["mit"]
    instances = mit["instances"]
    cm_params: dict = {"update_rate": mit.get("update_rate", 200)}
    cm_params["joint_state_broadcaster"] = {
        "type": "joint_state_broadcaster/JointStateBroadcaster"}
    body: dict = {"joint_state_broadcaster": {"ros__parameters": {"use_local_topics": False}}}
    for name, inst in instances.items():
        cm_params[name] = {"type": inst["plugin"]}
        body[name] = {"ros__parameters": _mit_params(cfg, mit, inst, gravity_compensation)}

    head = (
        GENERATED_NOTE
        + "# MIT 力控（变刚度）控制器：一个实例控一组关节，参数来自真值的 controllers.mit 段。\n"
        + "# 启动示例（电机侧 MIT，仿真）：\n"
        + "#   ros2 launch b601_bringup bringup.launch.py hardware:=mujoco \\\n"
        + "#       dm_command_mode:=mit_impedance \\\n"
        + "#       controllers_file:=<本文件> \\\n"
        + "#       controllers:=\"joint_state_broadcaster " + " ".join(instances) + "\"\n"
    )
    head += f"# bringup 参数：mit_gravity_compensation:={str(gravity_compensation).lower()}\n"
    data = {"controller_manager": {"ros__parameters": cm_params}}
    data.update(body)
    if mujoco_physical:
        params = mujoco_physical_gravity_params(cfg)
        for name in instances:
            body[name]["ros__parameters"]["gravity_params"] = params.copy()
        body["b601_mit_arm"]["ros__parameters"].update(
            use_gravity=True, gravity_scale=1.0, free_follow_feedback=False)
        head = GENERATED_NOTE + "# 内置 MuJoCo 物理场景。手臂使用 MJCF 重力前馈并保持高刚度。\n"
    return head + dump_yaml(data)


def render_moveit_controllers(cfg: dict, *, mit: bool = False) -> str:
    controllers = [cfg["controllers"][name] for name in ("arm", "gripper")]
    if mit:
        controllers = [{
            "name": cfg["moveit"]["mit_trajectory_controller"],
            "joints": [joint for controller in controllers for joint in controller["joints"]],
        }]
    simple = {"controller_names": [ctrl["name"] for ctrl in controllers]}
    for ctrl in controllers:
        simple[ctrl["name"]] = {
            "type": "FollowJointTrajectory", "action_ns": "follow_joint_trajectory",
            "default": True, "joints": ctrl["joints"],
        }
    return GENERATED_NOTE + dump_yaml({
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": False,
        "moveit_simple_controller_manager": simple,
        "trajectory_execution": cfg["moveit"]["trajectory_execution"],
    })


def render_hardware_dm(cfg: dict) -> str:
    dm = cfg["hardware"]["dm"]
    joints = [joint for joint in cfg["joints"].values() if joint["control"]]
    motors = [joint["motor"] for joint in joints]
    return GENERATED_NOTE + dump_yaml({"dm_hardware": {
        "transport": "serial", "device": dm["device"], "baudrate": dm["baudrate"],
        "motor_ids": [motor["id"] for motor in motors],
        "direction": [motor["direction"] for motor in motors],
        "zero_offset": [motor["zero_offset"] for motor in motors],
        "command_mode": dm["command_mode"],
        "kp": [motor["kp"] for motor in motors],
        "kd": [motor["kd"] for motor in motors],
        "motor_torque_limit_nm": [joint["max_motor_torque_nm"] for joint in joints],
        "enable_on_activate": dm["enable_motors"],
        "watchdog_timeout": dm["watchdog_timeout"],
        "max_velocity": [joint["max_velocity"] for joint in joints],
    }})


def render_srdf(cfg: dict, path: Path) -> str:
    """保持 SRDF 拓扑，替换从真值派生的命名位置。"""
    values = dict(cfg["moveit"]["named_arm_poses"])
    arm = cfg["robot"]["arm_joints"]
    finger = cfg["robot"]["gripper_joint"]
    values["home"] = {name: cfg["joints"][name]["startup"] for name in arm}
    presets = cfg["nodes"]["gripper_control"]["presets"]
    values["open"] = {finger: presets["open"]}
    values["closed"] = {finger: presets["close"]}

    def group(match):
        name = match.group("name")
        return match.group("open") + re.sub(
            r'(?P<field><joint name="(?P<joint>[^"]+)" value=")[^"]+(?P<end>"/>)',
            lambda joint: joint.group("field") + str(values[name][joint.group("joint")]) +
            joint.group("end"),
            match.group("joints"),
        ) + match.group("close")

    return re.sub(
        r'(?P<open><group_state name="(?P<name>[^"]+)"[^>]*>)(?P<joints>.*?)(?P<close></group_state>)',
        group, path.read_text(encoding="utf-8"), flags=re.DOTALL,
    )


def render_all(cfg: dict) -> dict[Path, str]:
    moveit = WS_SRC / "b601_moveit_config" / "config"
    srdf = moveit / "b601_dm.srdf"
    outputs = {
        WS_SRC / "b601_description" / "config" / "initial_positions.yaml":
            render_initial_positions(cfg),
        WS_SRC / "b601_description" / "config" / "safety_position_limits.yaml":
            GENERATED_NOTE + dump_yaml(safety_position_limits(cfg)),
        WS_SRC / "b601_bringup" / "config" / "b601_controllers.yaml":
            render_controllers(cfg),
        WS_SRC / "b601_moveit_config" / "config" / "joint_limits.yaml":
            render_joint_limits(cfg),
        WS_SRC / "b601_safety" / "config" / "safety_params.yaml":
            render_safety_params(cfg),
        WS_SRC / "b601_bringup" / "config" / "b601_mit_controllers.yaml":
            render_mit_controllers(cfg),
        WS_SRC / "b601_bringup" / "config" / "b601_mit_controllers_gravity.yaml":
            render_mit_controllers(cfg, gravity_compensation=True),
        WS_SRC / "b601_bringup" / "config" / "b601_mit_controllers_mujoco_physical.yaml":
            render_mit_controllers(cfg, mujoco_physical=True),
        WS_SRC / "b601_bringup" / "config" / "hardware_dm.yaml":
            render_hardware_dm(cfg),
        srdf: render_srdf(cfg, srdf),
        moveit / "moveit_controllers.yaml": render_moveit_controllers(cfg),
        moveit / "moveit_mit_controllers.yaml": render_moveit_controllers(cfg, mit=True),
        moveit / "kinematics.yaml": GENERATED_NOTE + dump_yaml(cfg["moveit"]["kinematics"]),
        moveit / "ompl_planning.yaml": GENERATED_NOTE + dump_yaml(cfg["moveit"]["ompl_planning"]),
    }
    return outputs


# ------------------------------------------------------------------------- 主流程
def main() -> int:
    parser = argparse.ArgumentParser(description="把 b601_project.yaml 同步到全部派生配置")
    parser.add_argument("--check", action="store_true", help="只检查是否已同步（不写盘）")
    parser.add_argument("--diff", action="store_true", help="打印差异（不写盘）")
    parser.add_argument("--quiet", action="store_true", help="成功时不打印文件清单")
    args = parser.parse_args()

    if not TRUTH.is_file():
        print(f"❌ 找不到真值配置：{TRUTH}")
        return 1
    with TRUTH.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    errors = validate(cfg)
    if errors:
        print(f"❌ 配置自身有 {len(errors)} 个问题：")
        for e in errors:
            print(f"   · {e}")
        return 1

    rendered = render_all(cfg)
    drifted: list[Path] = []
    mit_dir = WS_SRC / "b601_bringup" / "config"
    obsolete = sorted(path for path in mit_dir.glob("b601_mit_controllers*.yaml")
                      if path not in rendered)
    if obsolete:
        for path in obsolete:
            print(f"❌ 未登记的 MIT 控制器文件：{path.relative_to(REPO_ROOT)}")
        return 1

    for path, content in rendered.items():
        old = path.read_text(encoding="utf-8") if path.is_file() else ""
        if old == content:
            continue
        drifted.append(path)
        if args.check or args.diff:
            if args.diff:
                diff = difflib.unified_diff(
                    old.splitlines(keepends=True), content.splitlines(keepends=True),
                    fromfile=f"{path}（磁盘）", tofile=f"{path}（应按真值生成）",
                )
                sys.stdout.writelines(diff)
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    rel = [str(p.relative_to(REPO_ROOT)) for p in drifted]

    if args.check:
        if drifted:
            print(f"❌ 有 {len(drifted)} 个派生文件与真值不同步：")
            for r in rel:
                print(f"   · {r}")
            print("   → 跑 ./scripts/sync_config.py 同步")
            return 1
        print("✅ 所有派生配置与 b601_project.yaml 同步")
        return 0

    if args.diff:
        return 1 if drifted else 0

    if drifted:
        print(f"✅ 已生成 {len(drifted)} 个派生文件：")
        for r in rel:
            print(f"   · {r}")
    else:
        print("✅ 派生配置本来就是最新的（无需改动）")
    if not args.quiet:
        print(f"\n真值：{TRUTH.relative_to(REPO_ROOT)}")
        print("下一步：./scripts/verify.sh   （一致性检查会验证派生文件与真值匹配）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
