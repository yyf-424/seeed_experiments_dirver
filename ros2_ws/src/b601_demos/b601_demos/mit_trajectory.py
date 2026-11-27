"""Execute standard joint trajectories through one B601 MIT target session."""

from bisect import bisect_right
import json
from math import isfinite
from threading import Lock
from time import monotonic, sleep

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from b601_description.project_config import load_project_config

from .target_arbiter import ARBITER, TRAJECTORY_SOURCE
from .target_client import TargetClient
from .trajectory_utils import MIT_TRAJECTORY_CONTROLLER, duration

Result = FollowJointTrajectory.Result
MOTION_FIELDS = ("positions", "velocities", "accelerations")
TOLERANCE_FIELDS = ("position", "velocity", "acceleration")


def seconds(message):
    return message.sec + message.nanosec * 1e-9


class TrajectoryError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def interpolate(left, right, elapsed, span):
    """Use linear, cubic or quintic interpolation for the supplied derivatives."""
    if left.velocities and right.velocities:
        if left.accelerations and right.accelerations:
            order = 5
        else:
            order = 3
    else:
        order = 1
    ratio = elapsed / span
    position, velocity, acceleration = [], [], []
    for index, (start, end) in enumerate(zip(left.positions, right.positions)):
        displacement = end - start
        if order == 1:
            coefficients = (start, displacement, 0.0, 0.0, 0.0, 0.0)
        else:
            v0 = span * left.velocities[index]
            v1 = span * right.velocities[index]
            if order == 3:
                coefficients = (start, v0, 3 * displacement - 2 * v0 - v1,
                                -2 * displacement + v0 + v1, 0.0, 0.0)
            else:
                a0 = span**2 * left.accelerations[index]
                a1 = span**2 * right.accelerations[index]
                coefficients = (start, v0, a0 / 2,
                                10 * displacement - 6 * v0 - 4 * v1 - 1.5 * a0 + a1 / 2,
                                -15 * displacement + 8 * v0 + 7 * v1 + 1.5 * a0 - a1,
                                6 * displacement - 3 * v0 - 3 * v1 + (a1 - a0) / 2)
        c0, c1, c2, c3, c4, c5 = coefficients
        position.append(c0 + ratio * (c1 + ratio * (c2 + ratio * (c3 + ratio * (c4 + ratio * c5)))))
        velocity.append((c1 + ratio * (2*c2 + ratio * (3*c3 + ratio * (4*c4 + ratio * 5*c5)))) / span)
        acceleration.append((2*c2 + ratio * (6*c3 + ratio * (12*c4 + ratio * 20*c5))) / span**2)
    effort = []
    if left.effort or right.effort:
        zero = [0.0] * len(position)
        effort = [start + ratio * (end - start)
                  for start, end in zip(left.effort or zero, right.effort or zero)]
    return JointTrajectoryPoint(positions=position, velocities=velocity, accelerations=acceleration,
                                effort=effort)


def compare(actual, desired, names, tolerance):
    for field, key in zip(MOTION_FIELDS, TOLERANCE_FIELDS):
        reference = getattr(desired, field)
        measured = getattr(actual, field)
        for i, name in enumerate(names):
            limit = tolerance.get((name, key), 0.0)
            if limit > 0.0 and (not measured or not reference
                                or abs(reference[i] - measured[i]) > limit):
                return f"{name} {key} outside tolerance {limit:g} or unavailable"
    return ""


class MitTrajectory(Node):
    def __init__(self):
        super().__init__(MIT_TRAJECTORY_CONTROLLER)
        config = load_project_config()
        self.period = 1.0 / config["tools"]["dm_mit_move"]["rate"]
        self.service_timeout = config["safety"]["estop"]["service_timeout_sec"]
        self.feedback_timeout = config["safety"]["feedback_timeout_sec"]
        self.execution = config["moveit"]["trajectory_execution"]
        mit = config["controllers"]["mit"]
        self.goal_defaults = {}
        for instance in mit["instances"].values():
            profile = mit["profiles"][instance.get("gravity_profile", instance["profile"])]
            for name in instance["joints"]:
                self.goal_defaults[name, "position"] = profile["per_joint"][name].get(
                    "settle_band_min", profile["group"]["settle_band_min"])
        self.busy = Lock()
        self.latest_feedback = None
        self.status = {}
        self.estopped = False
        self.callbacks = ReentrantCallbackGroup()
        self.client = TargetClient(self, TRAJECTORY_SOURCE, callback_group=self.callbacks)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ARBITER}/status", self.read_status, status_qos)
        self.create_subscription(DiagnosticStatus, config["safety"]["status_topic"],
                                 self.read_safety, status_qos)
        self.create_subscription(JointState, "/joint_states", self.read_feedback, qos_profile_sensor_data)
        self.server = ActionServer(
            self, FollowJointTrajectory, "~/follow_joint_trajectory", self.execute,
            goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.callbacks)

    def read_status(self, message):
        self.status = json.loads(message.data)

    def read_safety(self, message):
        self.estopped = any(item.key == "state" and item.value == "EMERGENCY_STOP"
                           for item in message.values)

    def read_feedback(self, message):
        stamp = seconds(message.header.stamp)
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        accelerations = {}
        previous = self.latest_feedback
        if previous and stamp > previous[1]:
            accelerations = {name: (value - previous[3][name]) / (stamp - previous[1])
                             for name, value in velocities.items() if name in previous[3]}
        self.latest_feedback = monotonic(), stamp, positions, velocities, accelerations

    def measured(self, names):
        feedback = self.latest_feedback
        if feedback is None or monotonic() - feedback[0] > self.feedback_timeout:
            raise TrajectoryError(Result.INVALID_GOAL, "joint feedback unavailable or stale")
        fields = {}
        for field, samples in zip(MOTION_FIELDS, feedback[2:]):
            values = [samples.get(name, float("nan")) for name in names]
            if all(isfinite(value) for value in values):
                fields[field] = values
        if "positions" not in fields:
            raise TrajectoryError(Result.INVALID_GOAL, "joint positions unavailable")
        return JointTrajectoryPoint(**fields)

    def feedback_ready(self):
        feedback = self.latest_feedback
        return (feedback is not None and monotonic() - feedback[0] <= self.feedback_timeout
                and all(isfinite(feedback[2].get(name, float("nan"))) for name in self.client.joints))

    def accept(self, _request):
        return GoalResponse.REJECT if self.busy.locked() else GoalResponse.ACCEPT

    def validate(self, request):
        trajectory = request.trajectory
        names = list(trajectory.joint_names)
        if not names or len(names) != len(set(names)) or not set(names) <= set(self.client.joints):
            raise TrajectoryError(Result.INVALID_JOINTS, "trajectory must name distinct B601 joints")
        if (not trajectory.points or request.multi_dof_trajectory.points
                or request.component_path_tolerance or request.component_goal_tolerance):
            raise TrajectoryError(Result.INVALID_GOAL, "expected a B601 joint trajectory with joint tolerances")
        times = [seconds(point.time_from_start) for point in trajectory.points]
        if times[0] < 0.0 or any(a >= b for a, b in zip(times, times[1:])):
            raise TrajectoryError(Result.INVALID_GOAL, "waypoint times must be nonnegative and strictly increasing")
        for point in trajectory.points:
            if (len(point.positions) != len(names)
                    or any(values and (len(values) != len(names) or not all(map(isfinite, values)))
                           for values in (point.positions, point.velocities, point.accelerations, point.effort))):
                raise TrajectoryError(Result.INVALID_GOAL, "invalid waypoint vectors")
            self.check_positions(names, point.positions)
        if seconds(request.goal_time_tolerance) < 0.0:
            raise TrajectoryError(Result.INVALID_GOAL, "negative goal time tolerance")
        start = seconds(trajectory.header.stamp)
        if start and start + times[-1] <= self.now():
            raise TrajectoryError(Result.OLD_HEADER_TIMESTAMP, "trajectory ends in the past")
        return names, times, list(trajectory.points), start

    def check_positions(self, names, positions):
        for name, position in zip(names, positions):
            bounds = self.client.limits[name]
            if not bounds["min_pos"] <= position <= bounds["max_pos"]:
                raise TrajectoryError(Result.INVALID_GOAL, f"{name} outside configured position bounds")

    def tolerances(self, requested, names, defaults):
        values = defaults.copy()
        for tolerance in requested:
            if tolerance.name not in names:
                raise TrajectoryError(Result.INVALID_JOINTS, f"unexpected tolerance joint {tolerance.name}")
            for field in TOLERANCE_FIELDS:
                value = getattr(tolerance, field)
                if not isfinite(value) or (value < 0 and value != -1):
                    raise TrajectoryError(Result.INVALID_GOAL, f"invalid {field} tolerance")
                if value != 0:
                    values[tolerance.name, field] = value
        return values

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_active(self, handle, session):
        if handle.is_cancel_requested:
            raise TrajectoryError(Result.SUCCESSFUL, "trajectory canceled")
        if self.estopped:
            raise TrajectoryError(Result.INVALID_GOAL, "emergency stop")
        if session and self.status.get("session") != session:
            raise TrajectoryError(Result.INVALID_GOAL, "trajectory lost MIT control authority")

    def wait_until(self, condition, guard, timeout):
        end = monotonic() + timeout
        while rclpy.ok():
            guard()
            if condition():
                return True
            if monotonic() >= end:
                break
            sleep(self.period)
        return False

    def run(self, handle, session, names, times, points, start, path, goal):
        request = handle.request
        fixed = {}
        for name, value in zip(self.client.joints, self.measured(self.client.joints).positions):
            if name not in names:
                bounds = self.client.limits[name]
                fixed[name] = min(bounds["max_pos"], max(bounds["min_pos"], value))
        joint_names = [*names, *fixed]
        start = start or self.now()
        if times[0] > 0.0:
            initial = self.measured(names)
            for i, name in enumerate(names):
                bounds = self.client.limits[name]
                initial.positions[i] = min(bounds["max_pos"], max(bounds["min_pos"], initial.positions[i]))
            points.insert(0, initial)
            times.insert(0, 0.0)
        final_time = times[-1]
        goal_time = seconds(request.goal_time_tolerance)
        if goal_time == 0.0:
            goal_time = (final_time * (self.execution["allowed_execution_duration_scaling"] - 1)
                         + self.execution["allowed_goal_duration_margin"])
        final_dispatch = None
        while rclpy.ok():
            self.check_active(handle, session)
            elapsed = self.now() - start
            actual = self.measured(names)
            if 0.0 <= elapsed < final_time:
                i = bisect_right(times, elapsed) - 1
                desired = interpolate(points[i], points[i + 1], elapsed - times[i], times[i + 1] - times[i])
                self.check_positions(names, desired.positions)
                error = compare(actual, desired, names, path)
                if error:
                    raise TrajectoryError(Result.PATH_TOLERANCE_VIOLATED, error)
                zeros = [0.0] * len(fixed)
                self.client.publish(JointState(
                    name=joint_names, position=[*desired.positions, *fixed.values()],
                    velocity=[*desired.velocities, *zeros],
                    effort=[*desired.effort, *zeros] if desired.effort else []))
                self.publish_feedback(handle, names, desired, actual, elapsed)
            elif elapsed >= final_time:
                end = points[-1]
                desired = JointTrajectoryPoint(positions=end.positions,
                                               velocities=end.velocities or [0.0] * len(names),
                                               accelerations=end.accelerations or [0.0] * len(names),
                                               effort=end.effort)
                if final_dispatch is None:
                    final_dispatch = self.client.set_joints(
                        dict(zip(names, desired.positions)) | fixed, self.service_timeout,
                        velocities=dict(zip(names, desired.velocities)),
                        tff=dict(zip(names, desired.effort)), spinning=True)
                self.publish_feedback(handle, names, desired, actual, final_time)
                error = compare(actual, desired, names, goal)
                applied = self.status.get("last_applied_id") == str(final_dispatch)
                if applied and not error:
                    return
                if elapsed >= final_time + goal_time:
                    raise TrajectoryError(Result.GOAL_TOLERANCE_VIOLATED,
                                          error or "both MIT routes must acknowledge the final target")
            sleep(self.period)
        raise TrajectoryError(Result.INVALID_GOAL, "trajectory executor shutting down")

    def publish_feedback(self, handle, names, desired, actual, elapsed):
        error = JointTrajectoryPoint()
        for field in MOTION_FIELDS:
            setattr(error, field, [reference - measured for reference, measured
                                   in zip(getattr(desired, field), getattr(actual, field))])
        for point in (actual, desired, error):
            point.time_from_start = duration(elapsed)
        message = FollowJointTrajectory.Feedback(joint_names=names, desired=desired, actual=actual, error=error)
        message.header.stamp = self.get_clock().now().to_msg()
        handle.publish_feedback(message)

    def execute(self, handle):
        if not self.busy.acquire(blocking=False):
            handle.abort()
            return Result(error_code=Result.INVALID_GOAL, error_string="another trajectory owns the executor")
        session = None
        try:
            names, times, points, start = self.validate(handle.request)
            path = self.tolerances(handle.request.path_tolerance, names, {})
            goal = self.tolerances(handle.request.goal_tolerance, names, self.goal_defaults)
            if not self.wait_until(self.feedback_ready, lambda: self.check_active(handle, None),
                                   self.service_timeout):
                raise TrajectoryError(Result.INVALID_GOAL, "joint feedback unavailable or stale")
            session = self.client.select(self.service_timeout, spinning=True)
            if not self.wait_until(lambda: self.status.get("session") == session,
                                   lambda: self.check_active(handle, None), self.service_timeout):
                raise TrajectoryError(Result.INVALID_GOAL, "MIT authority was not confirmed")
            self.run(handle, session, names, times, points, start, path, goal)
            self.check_active(handle, session)
            self.client.release(self.service_timeout, spinning=True)
            handle.succeed()
            return Result()
        except (TrajectoryError, RuntimeError, ValueError) as error:
            message = str(error)
            code = error.code if isinstance(error, TrajectoryError) else Result.INVALID_GOAL
            stopped = session is None
            if session is not None and self.client.session_id and rclpy.ok():
                try:
                    self.client.release(self.service_timeout, spinning=True, hold=True)
                    stopped = self.wait_until(
                        lambda: self.status.get("holding_all") or self.estopped,
                        lambda: None, self.service_timeout)
                    if not stopped:
                        message += "; MIT hold was not confirmed"
                except RuntimeError as stop_error:
                    message += f"; {stop_error}"
            if handle.is_cancel_requested and stopped:
                handle.canceled()
                code = Result.SUCCESSFUL
            else:
                handle.abort()
                if code == Result.SUCCESSFUL:
                    code = Result.INVALID_GOAL
            return Result(error_code=code, error_string=message)
        finally:
            self.busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = MitTrajectory()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        rclpy.try_shutdown()
        executor.shutdown()
        node.server.destroy()
        node.destroy_node()
