"""Shared planning and timed joint references for base-frame TCP primitives."""
from copy import deepcopy
from dataclasses import dataclass
from math import atan2, ceil, cos, degrees, fmod, hypot, isfinite, radians, sin

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
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
ROTATION_AXES = {"rotate_x": 0, "rotate_y": 1, "rotate_z": 2}


@dataclass
class Segment:
    direction: str
    amount: float
    goal_pose: Pose
    trajectory: JointTrajectory

    @property
    def goal(self):
        return xyz(self.goal_pose)

    @property
    def motion(self):
        return {"direction": self.direction, amount_key(self.direction): self.amount}


MIN_DISTANCE_MM = 0.1


def primitive(direction, raw):
    if direction not in DIRECTIONS and direction not in ROTATION_AXES:
        raise ValueError(f"未知原语 {direction}，可用原语为 {'、'.join((*DIRECTIONS, *ROTATION_AXES))}")
    amount = float(raw)
    finite = not isinstance(raw, bool) and isfinite(amount)
    if direction in ROTATION_AXES:
        if not finite or not fmod(amount, 360):
            raise ValueError("旋转角度需要是有限数，且目标姿态必须改变")
    elif not finite or amount < MIN_DISTANCE_MM:
        raise ValueError(f"距离需要是至少 {MIN_DISTANCE_MM:g} mm 的有限数")
    return direction, amount


def amount_key(direction):
    return "angle_deg" if direction in ROTATION_AXES else "distance_mm"


def xyz(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


def orientation(pose):
    q = pose.orientation
    return [q.x, q.y, q.z, q.w]


def angular_error_deg(goal, measured):
    reference, actual = orientation(goal), orientation(measured)
    if sum(a * b for a, b in zip(reference, actual)) < 0:
        actual = [-value for value in actual]
    difference = hypot(*(a - b for a, b in zip(reference, actual)))
    total = hypot(*(a + b for a, b in zip(reference, actual)))
    return degrees(4 * atan2(difference, total))


def rotation_waypoints(reference, axis, angle):
    # Keep each interpolation below 180° so MoveIt follows the requested sign.
    angle = fmod(angle, 360)
    steps = max(1, ceil(abs(angle) / 120))
    x, y, z, w = orientation(reference)
    vector = [x, y, z]
    j, k = (axis + 1) % 3, (axis + 2) % 3
    waypoints = []
    for index in range(1, steps + 1):
        half_angle = radians(angle * index / steps) / 2
        s, c = sin(half_angle), cos(half_angle)
        # q_delta * q_reference expresses the rotation in the base frame.
        rotated = vector.copy()
        rotated[axis] = c * vector[axis] + s * w
        rotated[j] = c * vector[j] - s * vector[k]
        rotated[k] = c * vector[k] + s * vector[j]
        scalar = c * w - s * vector[axis]
        norm = hypot(*rotated, scalar)
        waypoint = deepcopy(reference)
        qx, qy, qz = (value / norm for value in rotated)
        waypoint.orientation = Quaternion(x=qx, y=qy, z=qz, w=scalar / norm)
        waypoints.append(waypoint)
    return waypoints


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
        for index, (direction, amount) in enumerate(primitives, 1):
            if direction in ROTATION_AXES:
                waypoints = rotation_waypoints(goal, ROTATION_AXES[direction], amount)
                goal = waypoints[-1]
            else:
                axis, sign = DIRECTIONS[direction]
                setattr(goal.position, axis, getattr(goal.position, axis) + sign * amount / 1000)
                waypoints = [goal]
            state = self.robot_state(positions)
            request = GetCartesianPath.Request(
                header=Header(frame_id=BASE), start_state=state,
                group_name="arm", link_name=TCP, waypoints=waypoints,
                max_step=0.005, avoid_collisions=True,
                max_velocity_scaling_factor=1.0, max_acceleration_scaling_factor=1.0,
            )
            answer = self.call(self.cartesian, request, self.planning_timeout)
            trajectory = answer.solution.joint_trajectory
            if (answer.error_code.val != MoveItErrorCodes.SUCCESS or answer.fraction != 1.0
                    or set(trajectory.joint_names) != set(self.arm) or not trajectory.points
                    or (len(waypoints) > 1 and len(trajectory.points) == 1)):
                unit = "°" if direction in ROTATION_AXES else "mm"
                raise RuntimeError(
                    f"第 {index} 段 {direction} {amount:g} {unit} 规划未完成"
                    f"（fraction={answer.fraction:.3f}，错误码={answer.error_code.val}）")
            if len(trajectory.points) == 1:
                trajectory = self.short_path(state, goal, positions, index)
            end = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
            if all(end[name] == positions[name] for name in self.arm):
                raise RuntimeError(f"第 {index} 段 {direction} 的规划终点未产生关节位移")
            positions.update(end)
            segments.append(Segment(direction, amount, deepcopy(goal), trajectory))
        return origin, segments
