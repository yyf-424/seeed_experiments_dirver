#!/usr/bin/env python3
"""move_to_joints.py —— 节点 b601_move_to_joints

角色：绕过 MoveIt，直接给 ros2_control 的 joint_trajectory_controller 发一条轨迹。
这是“最小可用控制闭环”，用来验证：
    controller 活着吗？command interface 通吗？硬件后端真的动了吗？

接口：
    动作客户端  /<controller>/follow_joint_trajectory
                control_msgs/action/FollowJointTrajectory
    订阅        /joint_states（只为了把当前位置作为轨迹第 0 点）
    参数        controller (str,   默认 b601_arm_controller)
                target     (double[6], 默认 [0.0, -0.3, -0.6, 0.0, 0.0, 0.0])
                duration   (double, 默认 3.0 秒，从起点到终点的时间)

用法：
    ros2 run b601_demos move_to_joints
    ros2 run b601_demos move_to_joints --ros-args -p target:="[0.5,-0.6,-0.8,0.3,0.2,0.0]"
    ros2 run b601_demos move_to_joints --ros-args -p duration:=6.0

为什么先订阅 /joint_states 再发轨迹：
    控制器的轨迹点如果只有终点，期望速度会是一条跳变；把当前点作为 t=0 的第 0 点，
    控制器才会按 duration 做平滑插值。这一步是“手写轨迹”和“MoveIt 规划”最大的区别。
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
SETTINGS = PROJECT["nodes"]["move_to_joints"]


class MoveToJoints(Node):
    """单目标点（点对点）轨迹发送器。"""

    def __init__(self):
        super().__init__("b601_move_to_joints")

        self.declare_parameter("controller", MIT_TRAJECTORY_CONTROLLER)
        self.declare_parameter("target", SETTINGS["target"])
        self.declare_parameter("duration", SETTINGS["duration"])

        self.controller = self.get_parameter("controller").value
        self.target = [float(v) for v in self.get_parameter("target").value]
        self.duration = float(self.get_parameter("duration").value)

        if len(self.target) != len(ARM_JOINTS):
            raise ValueError(f"target 需要 {len(ARM_JOINTS)} 个值，收到 {len(self.target)}")

        # 越界的轨迹宁可不发：真机上撞限位要人工复位
        problems = check_limits(ARM_JOINTS, self.target)
        if problems:
            raise ValueError("目标超出关节限位：" + "; ".join(problems))

        self.start_positions = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

        self.client = ActionClient(
            self, FollowJointTrajectory, f"/{self.controller}/follow_joint_trajectory"
        )

        self.get_logger().info(
            f"b601_move_to_joints 就绪：controller={self.controller}, "
            f"target={self.target}, duration={self.duration:.1f}s"
        )

    def _on_state(self, msg: JointState):
        """只记录一次起点（第一帧），避免运行中起点被自己刷新。"""
        if self.start_positions is not None:
            return
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in ARM_JOINTS):
            self.start_positions = [values[name] for name in ARM_JOINTS]

    def send(self):
        self.get_logger().info(f"等待 action server /{self.controller}/follow_joint_trajectory ...")
        if not self.client.wait_for_server(timeout_sec=SETTINGS["server_timeout_sec"]):
            self.get_logger().error(
                "等不到 action server。检查："
                "① spawner 是否成功加载了 controller  "
                "② ros2 control list_controllers 是否 active"
            )
            return 1

        # 等第一帧 /joint_states（最多 3 秒），这样轨迹起点一定是真实姿态。
        # 不等待就直接发单点轨迹时，控制器只能拿"当前"自己插值，起手会顿一下。
        deadline = self.get_clock().now().nanoseconds + 3_000_000_000
        while self.start_positions is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=SETTINGS["state_poll_sec"])

        # 起点：有真实状态就从当前姿态出发，否则退化成单点轨迹
        if self.start_positions is not None:
            waypoints = [self.start_positions, self.target]
            durations = [0.0, self.duration]
        else:
            self.get_logger().warn(
                "3 秒内没收到 /joint_states，直接发单点轨迹（控制器自行从当前位置插值）"
            )
            waypoints = [self.target]
            durations = [self.duration]

        trajectory = make_trajectory(ARM_JOINTS, waypoints, durations)
        self.get_logger().info("发送轨迹：" + describe(trajectory))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        # 允许的时间误差（秒）。真机首次调试别设太小。
        goal.goal_time_tolerance.sec = 1

        goal_handle = send_goal_with_retry(
            self, self.client, goal, "发送轨迹", feedback_callback=self._on_feedback
        )
        if goal_handle is None:
            return 1
        if not goal_handle.accepted:
            self.get_logger().error("轨迹被拒绝：检查 joint 名、点数、限位是否与 controller 一致")
            return 1

        self.get_logger().info("轨迹已接受，等待执行完成 ...")
        result = spin_until(
            self, goal_handle.get_result_async(), float(self.duration) + SETTINGS["result_grace_sec"], "等待执行结果"
        )
        if result is None:
            return 1

        error_code = result.result.error_code
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✅ 执行成功，已到达目标")
            return 0

        self.get_logger().error(
            f"❌ 执行失败 error_code={error_code}（{_error_name(error_code)}）；"
            f"error_string={result.result.error_string!r}"
        )
        return 1

    def _on_feedback(self, feedback_msg):
        """控制器会周期回报 desired / actual / error 三个轨迹点，这里打最大跟踪误差。

        注意 control_msgs 的 FollowJointTrajectory **Feedback 没有 trajectory 字段**，
        只有 header、desired、actual、error 四个（实测 Humble）。
        mock 后端下 error 恒为 0；真机上这个值就是"命令与实际差多少"。
        """
        feedback = feedback_msg.feedback
        error = list(getattr(feedback, "error", None).positions) if getattr(feedback, "error", None) else []
        if error:
            worst = max(abs(e) for e in error)
            self.get_logger().info(
                f"执行中 ... 最大跟踪误差 {worst:.4f} rad",
                throttle_duration_sec=1.0,
            )


def _error_name(code: int) -> str:
    names = {
        FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
        FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
        FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
    }
    return names.get(code, "UNKNOWN")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToJoints()
    try:
        exit_code = node.send()
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
