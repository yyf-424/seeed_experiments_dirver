"""Shared planning and timed joint references for base-frame TCP primitives."""
from copy import deepcopy
from dataclasses import dataclass
from math import isfinite

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


BASE = "base_link"
TCP = "end_link"
DIRECTIONS = {
    "forward": ("x", 1),
    "backward": ("x", -1),
    "left": ("y", 1),
    "right": ("y", -1),
    "up": ("z", 1),
    "down": ("z", -1),
}


@dataclass
class Segment:
    direction: str
    distance_mm: float
    goal_pose: Pose
    trajectory: JointTrajectory

    @property
    def goal(self):
        return xyz(self.goal_pose)


MIN_DISTANCE_MM = 0.1


def primitive(direction, raw):
    if direction not in DIRECTIONS:
        raise ValueError(f"未知方向 {direction}，可用方向为 {'、'.join(DIRECTIONS)}")
    distance = float(raw)
    if isinstance(raw, bool) or not isfinite(distance) or distance < MIN_DISTANCE_MM:
        raise ValueError(f"距离需要是至少 {MIN_DISTANCE_MM:g} mm 的有限数")
    return direction, distance


def xyz(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


class TcpPlanner:
    def __init__(self, node, project, call):
        self.call = call
        self.arm = project["robot"]["arm_joints"]
        self.gripper = project["robot"]["gripper_joint"]
        self.mimic = project["robot"]["mimic_joint"]
        self.joints = (*self.arm, self.gripper)
        self.gripper_limits = project["joints"][self.gripper]
        self.planning_timeout = project["validation"]["mit_five_poses"]["moveit_timeout_sec"]
        self.slew = project["target_slew"]["revolute"]
        self.fk = node.create_client(GetPositionFK, "/compute_fk")
        self.ik = node.create_client(GetPositionIK, "/compute_ik")
        self.cartesian = node.create_client(GetCartesianPath, "/compute_cartesian_path")

    def ready(self):
        return all(client.service_is_ready() for client in (self.fk, self.ik, self.cartesian))

    def targets(self, segment, gripper):
        trajectory = segment.trajectory
        bounds = self.gripper_limits
        gripper = min(bounds["max_pos"], max(bounds["min_pos"], gripper))
        for previous, point in zip(trajectory.points, trajectory.points[1:]):
            delta_sec = ((point.time_from_start.sec - previous.time_from_start.sec)
                         + (point.time_from_start.nanosec - previous.time_from_start.nanosec) * 1e-9)
            joint_travel = max(abs(end - start) for start, end in zip(previous.positions, point.positions))
            position = dict(zip(trajectory.joint_names, point.positions))
            position[self.gripper] = gripper
            yield max(delta_sec, joint_travel / self.slew), position

    def robot_state(self, positions):
        names = [*self.joints, self.mimic]
        return RobotState(joint_state=JointState(
            name=names,
            position=[*(positions[name] for name in self.joints), -positions[self.gripper]],
        ))

    def fk_pose(self, positions):
        request = GetPositionFK.Request(
            header=Header(frame_id=BASE), fk_link_names=[TCP],
            robot_state=self.robot_state(positions),
        )
        answer = self.call(self.fk, request, self.planning_timeout)
        if answer.error_code.val != MoveItErrorCodes.SUCCESS or len(answer.pose_stamped) != 1:
            raise RuntimeError(f"MoveIt FK 失败，错误码 {answer.error_code.val}")
        return answer.pose_stamped[0].pose

    def short_path(self, state, goal, positions, number):
        # Humble's time parameterizer can reduce a valid short Cartesian path
        # to its first waypoint. Recover the collision-checked endpoint and
        # let the existing joint slew determine the duration.
        request = GetPositionIK.Request(ik_request=PositionIKRequest(
            group_name="arm", robot_state=state, ik_link_name=TCP,
            pose_stamped=PoseStamped(header=Header(frame_id=BASE), pose=goal),
            avoid_collisions=True, timeout=Duration(sec=1),
        ))
        answer = self.call(self.ik, request, self.planning_timeout)
        joint_state = answer.solution.joint_state
        end = dict(zip(joint_state.name, joint_state.position))
        if answer.error_code.val != MoveItErrorCodes.SUCCESS or not set(self.arm) <= end.keys():
            raise RuntimeError(f"第 {number} 段短步终点逆解失败，错误码 {answer.error_code.val}")
        return JointTrajectory(joint_names=self.arm, points=[
            JointTrajectoryPoint(positions=[positions[name] for name in self.arm]),
            JointTrajectoryPoint(positions=[end[name] for name in self.arm]),
        ])

    def plan(self, primitives, initial, *, reference=None):
        positions = initial.copy()
        origin = self.fk_pose(positions)
        goal = deepcopy(origin if reference is None else reference)
        segments = []
        for index, (direction, distance) in enumerate(primitives, 1):
            axis, sign = DIRECTIONS[direction]
            setattr(goal.position, axis, getattr(goal.position, axis) + sign * distance / 1000)
            state = self.robot_state(positions)
            request = GetCartesianPath.Request(
                header=Header(frame_id=BASE), start_state=state,
                group_name="arm", link_name=TCP, waypoints=[goal],
                max_step=0.005, avoid_collisions=True,
                max_velocity_scaling_factor=1.0, max_acceleration_scaling_factor=1.0,
            )
            answer = self.call(self.cartesian, request, self.planning_timeout)
            trajectory = answer.solution.joint_trajectory
            if (answer.error_code.val != MoveItErrorCodes.SUCCESS or answer.fraction != 1.0
                    or set(trajectory.joint_names) != set(self.arm) or not trajectory.points):
                raise RuntimeError(
                    f"第 {index} 段 {direction} {distance:g} mm 规划未完成"
                    f"（fraction={answer.fraction:.3f}，错误码={answer.error_code.val}）")
            if len(trajectory.points) == 1:
                trajectory = self.short_path(state, goal, positions, index)
            end = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
            if all(end[name] == positions[name] for name in self.arm):
                raise RuntimeError(f"第 {index} 段 {direction} 的规划终点未产生关节位移")
            positions.update(end)
            segments.append(Segment(direction, distance, deepcopy(goal), trajectory))
        return origin, segments
