"""Plan base-frame translations and replay them through the MIT target arbiter."""
import argparse
import json
from time import monotonic

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from b601_description.project_config import load_project_config

from .mit_target import measured_positions
from .target_arbiter import ARBITER
from .target_client import TargetClient
from .tcp_motion import BASE, TCP, TcpPlanner, primitive, xyz


def parse_primitives(tokens):
    if not tokens or len(tokens) % 2:
        raise ValueError("按「方向 距离_mm」输入一组或多组原语")
    return [primitive(direction, raw) for direction, raw in zip(tokens[::2], tokens[1::2])]


def report(**values):
    print(json.dumps(values, ensure_ascii=False), flush=True)


class TcpPrimitives(Node):
    def __init__(self, project):
        super().__init__("b601_tcp_primitives")
        self.gripper = project["robot"]["gripper_joint"]
        validation = project["validation"]["mit_five_poses"]
        self.planning_timeout = validation["moveit_timeout_sec"]
        self.service_timeout = validation["state_check_timeout_sec"]
        self.feedback_timeout = project["safety"]["feedback_timeout_sec"]
        self.settle = project["tools"]["dm_mit_move"]["settle"]
        self.target = TargetClient(self, "tcp_primitives")
        self.planner = TcpPlanner(self, project, self.target.call)
        self.arbiter_status = {}
        self.safety = None
        self.started_dispatch = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ARBITER}/status", self.on_arbiter_status, latched)
        self.create_subscription(
            DiagnosticStatus, project["safety"]["status_topic"], self.on_safety, latched)

    def on_arbiter_status(self, msg):
        self.arbiter_status = json.loads(msg.data)

    def on_safety(self, msg):
        self.safety = msg.level

    def sample(self):
        return measured_positions(
            self.target.joints, (self.target.feedback, self.target.feedback_at), self.feedback_timeout)

    def wait_until(self, condition, timeout, failure, *, executing=False):
        deadline = monotonic() + timeout
        while rclpy.ok():
            if executing:
                self.check_execution()
            if condition():
                return
            remaining = deadline - monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(self, timeout_sec=min(remaining, self.feedback_timeout))
        raise RuntimeError(failure)

    def check_execution(self):
        if self.safety not in (DiagnosticStatus.OK, DiagnosticStatus.WARN):
            raise RuntimeError("未收到正常的整机安全状态")
        if self.arbiter_status.get("source") != self.target.source:
            raise RuntimeError("整机目标会话已失效")
        if self.sample() is None:
            raise RuntimeError("整机关节反馈已断流")

    def wait_motion(self, duration):
        end = monotonic() + duration
        self.wait_until(lambda: monotonic() >= end, duration, "等待轨迹采样点超时", executing=True)

    def execute(self, segment, gripper):
        dispatch = None
        for delay, positions in self.planner.targets(segment, gripper):
            self.wait_motion(delay)
            self.started_dispatch = True
            dispatch = self.target.set_joints(positions, self.service_timeout)
        self.wait_until(lambda: self.arbiter_status.get("last_applied_id") == str(dispatch),
                        self.service_timeout, f"两组 MIT 控制器未共同采纳目标 {dispatch}", executing=True)
        self.wait_motion(self.settle)
        self.check_execution()
        return dispatch, xyz(self.planner.fk_pose(self.sample()))

    def run(self, primitives, execute):
        self.wait_until(
            self.planner.ready,
            self.planning_timeout, "MoveIt 运动学和笛卡尔路径服务未就绪",
        )
        if execute:
            self.wait_until(lambda: self.safety is not None, self.planning_timeout, "未收到整机安全状态")
            # Resolve DDS endpoints before the short authority lease starts.
            self.wait_until(
                lambda: all(client.service_is_ready() for client in
                            (self.target.selector, self.target.sender, self.target.releaser))
                and self.target.heartbeats.get_subscription_count() > 0,
                self.planning_timeout, "整机目标仲裁端点未就绪",
            )
            self.target.select(self.planning_timeout)
            self.wait_until(lambda: self.arbiter_status.get("source") == self.target.source,
                            self.service_timeout, "未收到整机目标会话状态")
        self.wait_until(
            lambda: self.sample() is not None,
            self.planning_timeout, "未收到完整且新鲜的关节位置反馈",
        )
        initial = self.sample()
        origin, segments = self.planner.plan(primitives, initial)
        report(event="start", frame=BASE, tcp=TCP, measured_xyz_mm=[value * 1000 for value in xyz(origin)])
        for index, segment in enumerate(segments, 1):
            report(event="planned", segment=index, direction=segment.direction, distance_mm=segment.distance_mm,
                   target_xyz_mm=[value * 1000 for value in segment.goal],
                   points=len(segment.trajectory.points))
        if execute:
            self.check_execution()
            gripper = initial[self.gripper]
            before = xyz(origin)
            for index, segment in enumerate(segments, 1):
                dispatch, after = self.execute(segment, gripper)
                report(event="executed", segment=index, direction=segment.direction, dispatch_id=str(dispatch),
                       measured_xyz_mm=[value * 1000 for value in after],
                       delta_xyz_mm=[(end - start) * 1000 for start, end in zip(before, after)])
                before = after


def main():
    parser = argparse.ArgumentParser(description="规划基座坐标系内的 TCP 平移原语组合")
    parser.add_argument("--execute", action="store_true", help="通过已运行的 MIT 目标仲裁执行")
    parser.add_argument("primitives", nargs="+", metavar="方向 距离_mm")
    args = parser.parse_args()
    try:
        primitives = parse_primitives(args.primitives)
    except ValueError as error:
        parser.error(str(error))

    rclpy.init()
    node = TcpPrimitives(load_project_config())
    succeeded = False
    try:
        node.run(primitives, args.execute)
        succeeded = True
    except (RuntimeError, ValueError, KeyboardInterrupt, ExternalShutdownException) as error:
        node.get_logger().error(str(error) or "TCP 原语组合已中断")
    finally:
        # If a command may have reached the controller, let the arbiter's
        # watchdog request measured hold upon expiration of the failed session.
        if succeeded or not node.started_dispatch:
            node.target.close(node.service_timeout)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if succeeded else 1
