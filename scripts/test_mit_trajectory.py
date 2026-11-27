#!/usr/bin/env python3
"""Exercise the standard MIT trajectory action against a local MuJoCo demo."""

import json
from time import monotonic

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from b601_demos.target_arbiter import ARBITER
from b601_demos.target_client import TargetClient
from b601_demos.trajectory_utils import ARM_JOINTS, GRIPPER_JOINT, MIT_TRAJECTORY_CONTROLLER, make_trajectory
from b601_target_interfaces.msg import CompensationSources
from b601_target_interfaces.srv import RevokeSource


class Fixture(Node):
    def __init__(self):
        super().__init__("mit_trajectory_integration")
        self.positions = {}
        self.status = {}
        self.gravity = None
        self.feedback = None
        self.description = ""
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ARBITER}/status", self.read_status, qos)
        self.create_subscription(String, "/robot_description", self.read_description, qos)
        self.create_subscription(JointState, "/joint_states", self.read_joints, qos_profile_sensor_data)
        self.create_subscription(CompensationSources, "/b601_mit_arm/compensation_sources",
                                 self.read_gravity, qos_profile_sensor_data)
        self.action = ActionClient(self, FollowJointTrajectory,
                                   f"/{MIT_TRAJECTORY_CONTROLLER}/follow_joint_trajectory")
        self.moveit = ActionClient(self, MoveGroup, "/move_action")

    def read_status(self, message):
        self.status = json.loads(message.data)

    def read_joints(self, message):
        self.positions = dict(zip(message.name, message.position))

    def read_description(self, message):
        self.description = message.data

    def read_gravity(self, message):
        self.gravity = message.gravity_enabled

    def read_feedback(self, message):
        self.feedback = message.feedback

    def until(self, predicate, timeout=15):
        end = monotonic() + timeout
        while not predicate():
            assert monotonic() < end, f"timeout: {self.status}, joints={self.positions}"
            rclpy.spin_once(self, timeout_sec=0.05)

    def answer(self, future):
        self.until(future.done)
        return future.result()

    def follow(self, names, points, times):
        goal = FollowJointTrajectory.Goal(trajectory=make_trajectory(names, points, times))
        handle = self.answer(self.action.send_goal_async(goal, feedback_callback=self.read_feedback))
        assert handle.accepted
        return handle

    def succeeds(self, handle):
        response = self.answer(handle.get_result_async())
        assert response.status == GoalStatus.STATUS_SUCCEEDED, response
        assert response.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, response

    def verify_fixture(self):
        self.until(lambda: self.description)
        assert "b601_hardware/MujocoSystemHardware" in self.description, "expected an isolated MuJoCo robot"
        controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        assert controllers.wait_for_service(timeout_sec=15)
        expected = {"joint_state_broadcaster", "b601_mit_arm", "b601_mit_gripper"}

        def controllers_ready():
            loaded = self.answer(controllers.call_async(ListControllers.Request())).controller
            return {item.name for item in loaded} == expected and all(item.state == "active" for item in loaded)

        self.until(controllers_ready)
        assert self.action.wait_for_server(timeout_sec=15)
        assert self.moveit.wait_for_server(timeout_sec=15)
        self.until(lambda: self.status.get("feedback_ready") and self.gravity is True)
        print("PASS MuJoCo MIT controllers, physical gravity compensation and standard action", flush=True)

    def test_execution(self):
        names = [ARM_JOINTS[0], GRIPPER_JOINT]
        start_arm, start_gripper = (self.positions[name] for name in names)
        finger_delta = 0.002 if start_gripper <= 0.025 else -0.002
        points = [[start_arm + 0.04, start_gripper + finger_delta / 2],
                  [start_arm + 0.06, start_gripper + finger_delta]]
        self.succeeds(self.follow(names, points, [0.75, 1.5]))
        assert self.feedback.joint_names == names and len(self.feedback.error.positions) == len(names)
        assert abs(self.positions[names[0]] - points[-1][0]) <= 0.01
        assert abs(self.positions[names[1]] - points[-1][1]) <= 0.0005
        self.until(lambda: self.status.get("source") is None)
        print("PASS timed arm and gripper waypoints in one MIT session, feedback and result", flush=True)

    def test_cancel(self):
        start = self.positions["joint1"]
        handle = self.follow(["joint1"], [[start + 0.18]], [4.0])
        self.until(lambda: self.positions["joint1"] > start + 0.015
                   and self.status.get("source") == "trajectory")
        competitor = TargetClient(self, "trajectory_test_competitor")
        try:
            competitor.select(2.0)
        except RuntimeError as error:
            assert "control held" in str(error), error
        else:
            competitor.release(2.0)
            raise AssertionError("another source obtained authority during trajectory")
        assert self.answer(handle.cancel_goal_async()).goals_canceling
        response = self.answer(handle.get_result_async())
        assert response.status == GoalStatus.STATUS_CANCELED, response
        self.until(lambda: self.status.get("holding_all") and self.status.get("source") is None)
        assert self.positions["joint1"] < start + 0.09
        print("PASS exclusive trajectory authority, action cancellation and both MIT holds", flush=True)

    def test_failed_trajectory(self):
        names = ["joint1"]
        target = self.positions["joint1"] + 0.1
        goal = FollowJointTrajectory.Goal(
            trajectory=make_trajectory(names, [[target], [target]], [0.0, 1.0]),
            path_tolerance=[JointTolerance(name=names[0], position=0.001)])
        handle = self.answer(self.action.send_goal_async(goal))
        assert handle.accepted
        response = self.answer(handle.get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED, response
        assert response.result.error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, response
        self.until(lambda: self.status.get("source") is None)
        print("PASS requested path tolerance reports violation and releases authority", flush=True)

    def test_revoke(self):
        start = self.positions["joint1"]
        handle = self.follow(["joint1"], [[start + 0.18]], [4.0])
        self.until(lambda: self.positions["joint1"] > start + 0.015
                   and self.status.get("source") == "trajectory")
        revoke = self.create_client(RevokeSource, f"{ARBITER}/revoke_source")
        assert revoke.wait_for_service(timeout_sec=5)
        assert self.answer(revoke.call_async(RevokeSource.Request(session=self.status["session"]))).accepted
        response = self.answer(handle.get_result_async())
        assert response.status == GoalStatus.STATUS_ABORTED, response
        self.until(lambda: self.status.get("holding_all") and self.status.get("source") is None)
        print("PASS operator revocation aborts the trajectory and both MIT groups hold", flush=True)

    def test_moveit(self, execution_mode="MIT"):
        self.until(lambda: all(name in self.positions for name in ARM_JOINTS))
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.10
        goal.request.max_acceleration_scaling_factor = 0.10
        goal.request.start_state.is_diff = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        targets = dict(zip(ARM_JOINTS, (self.positions[name] for name in ARM_JOINTS)))
        targets["joint1"] += 0.05
        goal.request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=value, tolerance_above=0.001,
                            tolerance_below=0.001, weight=1.0)
            for name, value in targets.items()])]
        handle = self.answer(self.moveit.send_goal_async(goal))
        assert handle.accepted
        response = self.answer(handle.get_result_async())
        assert response.result.error_code.val == MoveItErrorCodes.SUCCESS, response
        points = response.result.planned_trajectory.joint_trajectory.points
        assert points and abs(self.positions["joint1"] - points[-1].positions[0]) <= 0.01
        print(f"PASS MoveIt Plan & Execute via {execution_mode} ({len(points)} points)", flush=True)


def main():
    rclpy.init()
    node = Fixture()
    try:
        node.verify_fixture()
        node.test_execution()
        node.test_cancel()
        node.test_failed_trajectory()
        node.test_revoke()
        node.test_moveit()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
