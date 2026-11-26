"""Client for selecting a source and supplying sequenced MIT joint targets."""
from math import isfinite
from time import monotonic
from threading import Event, Lock

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from b601_description.project_config import load_project_config

from b601_target_interfaces.msg import MitTargetFrame, TargetFrame
from b601_target_interfaces.srv import ReleaseSource, RenewSource, SelectSource, SetMitTarget, SetTarget

from .gravity_provider import GravityProvider
from .mit_target import make_mit_target, positions_from_deltas
from .target_arbiter import ARBITER


class TargetClient:
    def __init__(self, node, source, *, callback_group=None, auto_renew=True,
                 gravity_source=None, gravity_api=None):
        self.node = node
        self.source = source
        self.session_id = ""
        self.sequence = 0
        self.lock = Lock()

        project = load_project_config()
        robot = project["robot"]
        self.joints = (*robot["arm_joints"], robot["gripper_joint"])
        self.limits = project["joints"]
        self.feedback_timeout = project["safety"]["feedback_timeout_sec"]
        self.gravity = GravityProvider(
            node, self.joints, self.feedback_timeout, source=gravity_source, api=gravity_api,
            callback_group=callback_group)
        self.feedback = {}
        self.feedback_at = -float("inf")
        self.positions = {}
        node.create_subscription(
            JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data,
            callback_group=callback_group)

        if auto_renew:
            timeout = project["tools"]["target_arbitration"]["authority_timeout_sec"]
            self.heartbeats = node.create_publisher(String, f"{ARBITER}/source_heartbeat", 10)
            self.heartbeat_timer = node.create_timer(
                timeout / 5, self.heartbeat, callback_group=callback_group,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publisher = node.create_publisher(TargetFrame, f"{ARBITER}/target", 10)
        self.mit_publisher = node.create_publisher(MitTargetFrame, f"{ARBITER}/mit_target", 10)
        self.selector = node.create_client(
            SelectSource, f"{ARBITER}/select_source", callback_group=callback_group)
        self.releaser = node.create_client(
            ReleaseSource, f"{ARBITER}/release_source", callback_group=callback_group)
        self.renewer = node.create_client(
            RenewSource, f"{ARBITER}/renew_source", callback_group=callback_group)
        self.sender = node.create_client(
            SetTarget, f"{ARBITER}/set_target", callback_group=callback_group)
        self.mit_sender = node.create_client(
            SetMitTarget, f"{ARBITER}/set_mit_target", callback_group=callback_group)

    def call(self, client, request, timeout, *, spinning=False):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"target arbitration service unavailable: {client.srv_name}")
        future = client.call_async(request)
        if spinning:
            completed = Event()
            future.add_done_callback(lambda _: completed.set())
            completed.wait(timeout)
        else:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            client.remove_pending_request(future)
            raise RuntimeError(f"target arbitration service timed out: {client.srv_name}")
        return future.result()

    def heartbeat(self):
        with self.lock:
            session_id = self.session_id
        if session_id:
            self.heartbeats.publish(String(data=session_id))

    def select(self, timeout, *, spinning=False):
        with self.lock:
            if self.session_id:
                raise RuntimeError("release the previous target session before selecting again")
        request = SelectSource.Request(source=self.source)
        reply = self.call(self.selector, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)
        with self.lock:
            self.session_id = reply.session_id
            self.sequence = 0
            self.positions = {}

    def renew(self, timeout, *, spinning=False):
        with self.lock:
            request = RenewSource.Request(session_id=self.session_id)
        reply = self.call(self.renewer, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)

    def release(self, timeout, *, spinning=False):
        with self.lock:
            session_id = self.session_id
        if not session_id:
            return
        request = ReleaseSource.Request(session_id=session_id)
        reply = self.call(self.releaser, request, timeout, spinning=spinning)
        with self.lock:
            self.session_id = ""
        if not reply.accepted:
            raise RuntimeError(reply.message)

    def frame(self, target, *, gravity_ff=None, friction_ff=None):
        command = make_mit_target(target, gravity_ff, friction_ff)
        with self.lock:
            if not self.session_id:
                raise RuntimeError("select the target source before publishing")
            self.sequence += 1
            if command.gravity_ff or command.friction_ff:
                return MitTargetFrame(session_id=self.session_id, sequence=self.sequence, command=command)
            return TargetFrame(session_id=self.session_id, sequence=self.sequence, target=target)

    def on_feedback(self, message):
        with self.lock:
            self.feedback = dict(zip(message.name, message.position))
            self.feedback_at = monotonic()
            self.gravity.feedback = (self.feedback, self.feedback_at)

    def set_gravity_source(self, source, api=None):
        self.gravity.select_source(source, api)

    def complete_target(self, message):
        """Expand local joint edits into one full joint target for the ROS interface."""
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        tff = dict(zip(message.name, message.effort))
        if (not positions or len(positions) != len(message.name)
                or len(message.position) != len(message.name)
                or (message.velocity and len(message.velocity) != len(message.name))
                or (message.effort and len(message.effort) != len(message.name))
                or not positions.keys() <= set(self.joints)):
            raise ValueError("invalid joint target")
        with self.lock:
            values = []
            for name in self.joints:
                bounds = self.limits[name]
                if name in positions:
                    value = positions[name]
                elif name in self.positions:
                    value = self.positions[name]
                else:
                    value = self.feedback.get(name, float("nan"))
                    if monotonic() - self.feedback_at > self.feedback_timeout or not isfinite(value):
                        raise RuntimeError(f"waiting for joint feedback for {name}")
                    value = min(bounds["max_pos"], max(bounds["min_pos"], value))
                values.append(value)
        return JointState(
            header=message.header, name=list(self.joints), position=values,
            velocity=[velocities.get(name, 0.0) for name in self.joints] if velocities else [],
            effort=[tff.get(name, 0.0) for name in self.joints] if tff else [],
        )

    def remember_target(self, frame):
        target = frame.command.target if isinstance(frame, MitTargetFrame) else frame.target
        with self.lock:
            if frame.session_id == self.session_id and frame.sequence == self.sequence:
                self.positions = dict(zip(target.name, target.position))

    def publish(self, message, *, gravity_ff=None, friction_ff=None):
        target = self.complete_target(message)
        frame = self.frame(target, gravity_ff=gravity_ff, friction_ff=friction_ff)
        publisher = self.mit_publisher if isinstance(frame, MitTargetFrame) else self.publisher
        publisher.publish(frame)
        if (all(self.limits[name]["min_pos"] <= value <= self.limits[name]["max_pos"]
                for name, value in zip(target.name, target.position))
                and all(isfinite(value) for value in [*target.velocity, *target.effort])):
            self.remember_target(frame)

    def set_joints(self, positions, timeout, *, velocities=None, tff=None,
                   gravity_ff=None, friction_ff=None, spinning=False):
        message = JointState(name=list(positions), position=list(positions.values()))
        target = self.complete_target(message)
        for field, values in (("velocity", velocities), ("effort", tff)):
            if values:
                if not values.keys() <= set(self.joints):
                    raise ValueError(f"unknown joint in {field}")
                setattr(target, field, [float(values.get(name, 0.0)) for name in self.joints])
        frame = self.frame(target, gravity_ff=gravity_ff, friction_ff=friction_ff)
        if isinstance(frame, MitTargetFrame):
            sender, request = self.mit_sender, SetMitTarget.Request(frame=frame)
        else:
            sender, request = self.sender, SetTarget.Request(frame=frame)
        reply = self.call(sender, request, timeout, spinning=spinning)
        if not reply.accepted:
            raise RuntimeError(reply.message)
        self.remember_target(frame)
        return reply.dispatch_id

    def set_joint(self, joint, position, timeout, *, velocity=None, tff=None, spinning=False):
        return self.set_joints(
            {joint: position}, timeout, spinning=spinning,
            velocities={joint: velocity} if velocity is not None else None,
            tff={joint: tff} if tff is not None else None,
        )

    def publish_joint_deltas(self, deltas):
        """Resolve increments locally and stream a full absolute target."""
        positions = self._delta_positions(deltas)
        self.publish(JointState(name=list(positions), position=list(positions.values())))

    def set_joint_deltas(self, deltas, timeout, *, spinning=False):
        """Resolve increments locally, then dispatch a full absolute target."""
        return self.set_joints(self._delta_positions(deltas), timeout, spinning=spinning)

    def _delta_positions(self, deltas):
        with self.lock:
            sample = self.feedback, self.feedback_at
        return positions_from_deltas(self.joints, deltas, sample, self.feedback_timeout)

    def close(self, timeout):
        if rclpy.ok():
            try:
                self.release(timeout)
            except RuntimeError as error:
                self.node.get_logger().warning(str(error))
