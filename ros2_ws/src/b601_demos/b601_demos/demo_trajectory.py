#!/usr/bin/env python3
"""demo_trajectory.py —— 节点 b601_demo_trajectory

角色：一次发一条**多点**轨迹（对比 move_to_joints 的两点轨迹）。
多点轨迹是理解“轨迹 = 时间 + 关节角序列”的最好材料：
控制器会在相邻点之间自己插值，所以要控制的是“每个点什么时候到”。

接口：
    动作客户端  /b601_arm_controller/follow_joint_trajectory
    订阅        /joint_states（取起点）
    参数        controller (str, 默认 b601_arm_controller)
                hold       (double, 默认 1.5 秒，每段的过渡时间)

用法：
    ros2 run b601_demos demo_trajectory
    ros2 run b601_demos demo_trajectory --ros-args -p hold:=3.0

动作序列（都是小角度，mock 后端看不出风险，真机上仍然务必先试小幅度）：
    ① 原地 → 微微前倾      （joint2/joint3 各 -0.2）
    ② 底座右转            （joint1 +0.4）
    ③ 底座左转            （joint1 -0.4）
    ④ 腕部转一下          （joint5 +0.5, joint6 +0.5）
    ⑤ 回零位
"""

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from b601_demos.trajectory_utils import (
    ARM_JOINTS,
    MIT_TRAJECTORY_CONTROLLER,
    check_limits,
    describe,
    make_trajectory,
    send_goal_with_retry,
    spin_until,
)

from b601_description.project_config import load_project_config

PROJECT = load_project_config()
SETTINGS = PROJECT["nodes"]["demo_trajectory"]

# 每一段的关节目标（joint1..joint6，rad）
SEQUENCE = SETTINGS["sequence"]


class DemoTrajectory(Node):
    def __init__(self):
        super().__init__("b601_demo_trajectory")

        self.declare_parameter("controller", MIT_TRAJECTORY_CONTROLLER)
        self.declare_parameter("hold", SETTINGS["hold"])
        self.controller = self.get_parameter("controller").value
        self.hold = float(self.get_parameter("hold").value)

        self.start_positions = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory, f"/{self.controller}/follow_joint_trajectory"
        )

    def _on_state(self, msg: JointState):
        if self.start_positions is not None:
            return
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in ARM_JOINTS):
            self.start_positions = [values[name] for name in ARM_JOINTS]

    def run(self) -> int:
        if not self.client.wait_for_server(timeout_sec=SETTINGS["server_timeout_sec"]):
            self.get_logger().error("action server 不可用，先确认 bringup 与 spawner 正常")
            return 1

        # 把所有段的耗时累加，得到每个点的绝对到达时刻
        waypoints, durations = [], []
        t = 0.0
        if self.start_positions is not None:
            waypoints.append(self.start_positions)
            durations.append(t)
        for _, target in SEQUENCE:
            problems = check_limits(ARM_JOINTS, target)
            if problems:
                self.get_logger().error("目标越界：" + "; ".join(problems))
                return 1
            t += self.hold
            waypoints.append(list(target))
            durations.append(t)

        trajectory = make_trajectory(ARM_JOINTS, waypoints, durations)
        self.get_logger().info(f"一条轨迹走完 {len(SEQUENCE)} 段：" + describe(trajectory))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance.sec = SETTINGS["goal_time_tolerance_sec"]

        goal_handle = send_goal_with_retry(self, self.client, goal, "发送轨迹")
        if goal_handle is None:
            return 1
        if not goal_handle.accepted:
            self.get_logger().error("轨迹被拒绝：检查 joint 名、点数、限位是否与 controller 一致")
            return 1

        for label, target in SEQUENCE:
            self.get_logger().info(f"  {label}: {target}")

        total = float(trajectory.points[-1].time_from_start.sec
                      + trajectory.points[-1].time_from_start.nanosec / 1e9)
        result = spin_until(self, goal_handle.get_result_async(), total + SETTINGS["result_grace_sec"], "等待执行结果")
        if result is None:
            return 1
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅ 整条轨迹执行完成")
            return 0

        self.get_logger().error(
            f"❌ 失败 error_code={result.result.error_code} "
            f"error_string={result.result.error_string!r}"
        )
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = DemoTrajectory()
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
