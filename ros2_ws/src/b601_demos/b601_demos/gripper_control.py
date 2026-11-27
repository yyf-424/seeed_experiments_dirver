#!/usr/bin/env python3
"""gripper_control.py —— 节点 b601_gripper

角色：夹爪开合。用“一条两点的短轨迹”实现，而不是自定义服务，
因为控制器的接口本来就只需要轨迹 —— 少一个自定义接口，少一层坑。

接口：
    动作客户端  /b601_gripper_controller/follow_joint_trajectory
    订阅        /joint_states（取夹爪当前位置）
    参数        preset (str, 默认 open)：open / close / 数值（米，0~0.05）
                duration (double, 默认 1.0 秒)
                controller (str, 默认 b601_gripper_controller)

用法：
    ros2 run b601_demos gripper                      # 张开
    ros2 run b601_demos gripper --ros-args -p preset:=close
    ros2 run b601_demos gripper --ros-args -p preset:=0.02

夹爪的量程：finger_left 0（闭合）→ 0.05（最大张开），单位 m。
从动指 finger_right 由 URDF 的 mimic 自动跟随（multiplier=-1），不需要命令。
"""

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from b601_demos.trajectory_utils import (
    GRIPPER_JOINT,
    MIT_TRAJECTORY_CONTROLLER,
    make_trajectory,
    send_goal_with_retry,
    spin_until,
)

from b601_description.project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["nodes"]["gripper_control"]

PRESETS = SETTINGS["presets"]


class Gripper(Node):
    def __init__(self):
        super().__init__("b601_gripper")

        # preset 允许两种类型：字符串（open/close）或数值（米，如 0.02）。
        # 必须开 dynamic_typing —— 否则 `-p preset:=0.02` 被 ROS 解析成 double，
        # 与声明的 string 类型冲突（2026-11-21 实测：InvalidParameterType）。
        self.declare_parameter("preset", "open", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("duration", SETTINGS["duration"])
        self.declare_parameter("controller", MIT_TRAJECTORY_CONTROLLER)

        raw_preset = self.get_parameter("preset").value
        self.duration = float(self.get_parameter("duration").value)
        self.controller = self.get_parameter("controller").value

        if isinstance(raw_preset, str) and raw_preset in PRESETS:
            self.target = PRESETS[raw_preset]
        else:
            self.target = float(raw_preset)
        limits = PROJECT["joints"][GRIPPER_JOINT]
        self.target = min(max(self.target, limits["min_pos"]), limits["max_pos"])

        self.current = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory, f"/{self.controller}/follow_joint_trajectory"
        )
        self.get_logger().info(
            f"b601_gripper：目标开度 {self.target * 1000:.0f} mm，用时 {self.duration:.1f}s"
        )

    def _on_state(self, msg: JointState):
        if self.current is not None:
            return
        values = dict(zip(msg.name, msg.position))
        if GRIPPER_JOINT in values:
            self.current = values[GRIPPER_JOINT]

    def run(self) -> int:
        if not self.client.wait_for_server(timeout_sec=SETTINGS["server_timeout_sec"]):
            self.get_logger().error("夹爪 controller 的 action server 不可用")
            return 1

        if self.current is not None:
            waypoints = [[self.current], [self.target]]
            durations = [0.0, self.duration]
        else:
            waypoints = [[self.target]]
            durations = [self.duration]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = make_trajectory([GRIPPER_JOINT], waypoints, durations)
        goal.goal_time_tolerance.sec = 1

        handle = send_goal_with_retry(self, self.client, goal, "发送夹爪轨迹")
        if handle is None:
            return 1
        if not handle.accepted:
            self.get_logger().error("夹爪轨迹被拒绝")
            return 1

        result = spin_until(self, handle.get_result_async(), self.duration + SETTINGS["result_grace_sec"],
                            "等待夹爪结果")
        if result is None:
            return 1
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"✅ 夹爪动作完成，开度 {self.target * 1000:.0f} mm")
            return 0
        self.get_logger().error(f"❌ 失败 error_code={result.result.error_code}")
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = Gripper()
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
