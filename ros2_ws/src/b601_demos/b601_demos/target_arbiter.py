"""Exclusive MIT joint-target sessions and routing."""
import json
from dataclasses import dataclass
from math import isfinite
from time import monotonic_ns
from uuid import uuid4

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import SetBool

from b601_target_interfaces.msg import AuthorizedTarget, MitTarget, MitTargetFrame, TargetFrame
from b601_target_interfaces.srv import (
    ReleaseSource, RenewSource, RevokeSource, SelectSource, SetAuthority, SetMitTarget, SetTarget,
)

from b601_description.project_config import load_project_config

from .mit_target import feedforward_by_joint, make_mit_target
from .trajectory_utils import RECENTER_COMPLETED, mit_recenter_bounds

ARBITER = "/b601_target_arbiter"


@dataclass
class Session:
    source: str
    id: str
    expires_at: int
    status_id: str
    sequence: int = 0


class TargetArbiter(Node):
    def __init__(self):
        super().__init__("b601_target_arbiter")
        project_path = self.declare_parameter("project_config_file", "").value
        project = load_project_config(project_path or None)
        routes = self.declare_parameter("routes", "{}").value
        self.routes = {name: tuple(joints) for name, joints in json.loads(routes).items()}
        self.limits = project["joints"]
        robot = project["robot"]
        self.arm_joints = tuple(robot["arm_joints"])
        self.joints = (*self.arm_joints, robot["gripper_joint"])
        self.home_bounds = mit_recenter_bounds(project)
        self.completed_recenter = 0
        self.disable_ready = False
        self.disabling = False
        if not self.routes:
            raise ValueError("No MIT target controller routes")
        if any(not names or any(j not in self.joints for j in names)
               for names in self.routes.values()):
            raise ValueError("Invalid MIT joint route")
        self.feedback = {}
        self.session = None
        self.authority = None
        self.dispatched = None
        self.applied = {}
        self.feedback_stamp = None
        self.feedback_ready = False
        self.feedback_timeout_ns = project["safety"]["feedback_timeout_sec"] * 1e9
        authority_timeout = project["tools"]["target_arbitration"]["authority_timeout_sec"]
        self.authority_timeout_ns = int(authority_timeout * 1e9)
        self.holding = {}
        self.epochs = {name: self.new_epoch() for name in self.routes}
        self.last_heartbeat_ns = monotonic_ns()
        self.dispatch_id = monotonic_ns()
        self.hold_revisions = dict.fromkeys(self.routes, 0)
        self.outputs = {}
        for controller in self.routes:
            self.outputs[controller] = self.create_publisher(
                AuthorizedTarget, f"/{controller}/authorized_target", 10)
            self.create_subscription(
                UInt64, f"/{controller}/target_applied",
                lambda msg, controller=controller: self.on_applied(controller, msg), 10,
            )
            self.create_subscription(
                Bool, f"/{controller}/target_holding",
                lambda msg, controller=controller: self.on_holding(controller, msg), 10,
            )
            self.create_subscription(
                UInt64, f"/{controller}/target_epoch_requested",
                lambda msg, controller=controller: self.on_epoch_requested(controller, msg), 10,
            )
        self.status = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(TargetFrame, "~/target", self.on_target, 10)
        self.create_subscription(MitTargetFrame, "~/mit_target", self.on_mit_target, 10)
        self.create_subscription(String, "~/source_heartbeat", self.source_heartbeat, 10)
        self.create_subscription(
            UInt64, RECENTER_COMPLETED, self.on_recenter_completed,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetBool, "~/reserve_disable", self.reserve_disable)
        self.create_service(SelectSource, "~/select_source", self.select_source)
        self.create_service(SetAuthority, "~/set_authority", self.set_authority)
        self.create_service(ReleaseSource, "~/release_source", self.release_source)
        self.create_service(ReleaseSource, "~/release_web_for_recenter", self.release_web_for_recenter)
        self.create_service(RevokeSource, "~/revoke_source", self.revoke_source)
        self.create_service(RenewSource, "~/renew_source", self.renew_source)
        self.create_timer(
            authority_timeout / 5, self.heartbeat,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_service(SetTarget, "~/set_target", self.set_target)
        self.create_service(SetMitTarget, "~/set_mit_target", self.set_mit_target)
        self.last_event = {"event": "ready"}
        self.publish_status()

    def on_feedback(self, message):
        self.feedback = {key: value for key, value in zip(message.name, message.position)
                         if isfinite(value)}
        self.feedback_stamp = monotonic_ns()

    def select_source(self, request, response):
        now = monotonic_ns()
        self.expire_sessions(now)
        source = request.source.strip()
        if self.disabling:
            response.message = "整机正在失能"
        elif not source:
            response.message = "source is required"
        elif self.authority is not None and source != self.authority:
            response.message = f"authority designated to {self.authority}"
        elif self.session:
            response.message = f"control held by {self.session.source}"
        else:
            self.session = Session(
                source, uuid4().hex, now + self.authority_timeout_ns, uuid4().hex)
            response.accepted = True
            response.session_id = self.session.id
            response.session = self.session.status_id
            response.message = "granted"
        self.event("select", source, response.message)
        return response

    def set_authority(self, request, response):
        if self.disabling:
            response.message = "整机正在失能"
            return response
        self.expire_sessions(monotonic_ns())
        source = request.source.strip() or None
        if self.session and source != self.session.source:
            self.session = None
        self.authority = source
        response.accepted = True
        response.message = "authority designated" if source else "authority released"
        self.event("authority", source or "", response.message)
        return response

    @staticmethod
    def new_epoch():
        return int(uuid4().hex[:16], 16) or 1

    def publish_heartbeat(self, controller):
        self.outputs[controller].publish(AuthorizedTarget(
            kind=AuthorizedTarget.HEARTBEAT, arbiter_id=self.epochs[controller],
            hold_revision=self.hold_revisions[controller]))

    def request_hold(self, controller):
        self.hold_revisions[controller] += 1
        self.publish_heartbeat(controller)

    def heartbeat(self):
        now = monotonic_ns()
        self.expire_sessions(now)
        self.announce_heartbeat(now)
        available = self.feedback_is_fresh(now) and all(name in self.feedback for name in self.joints)
        if available != self.feedback_ready or (not self.disabling and self.parked(now)) != self.disable_ready:
            self.feedback_ready = available
            self.publish_status()

    def on_recenter_completed(self, message):
        self.completed_recenter = message.data
        self.publish_status()

    def parked(self, now):
        dispatch = str(self.completed_recenter)
        return (self.session is None and self.completed_recenter != 0
                and self.dispatched is not None and self.dispatched["source"] == "recenter"
                and self.dispatched["dispatch_id"] == dispatch
                and all(self.applied.get(route) == self.completed_recenter for route in self.routes)
                and self.feedback_is_fresh(now)
                and all(name in self.feedback and low <= self.feedback[name] <= high
                        for name, (low, high) in zip(self.arm_joints, self.home_bounds)))

    def reserve_disable(self, request, response):
        self.expire_sessions(monotonic_ns())
        if request.data and (self.disabling or not self.parked(monotonic_ns())):
            response.message = "需要先完成归中并确认六轴实测到位"
        else:
            self.disabling = request.data
            self.publish_status()
            response.success = True
        return response

    def feedback_is_fresh(self, now):
        return self.feedback_stamp is not None and now - self.feedback_stamp <= self.feedback_timeout_ns

    def announce_heartbeat(self, now):
        self.last_heartbeat_ns = now
        for controller in self.outputs:
            self.publish_heartbeat(controller)

    def expire_sessions(self, now):
        if now - self.last_heartbeat_ns >= self.authority_timeout_ns:
            self.session = None
            self.epochs = {name: self.new_epoch() for name in self.routes}
            self.announce_heartbeat(now)
            self.event("arbiter_timeout", "", "all prior sessions revoked after arbiter interruption")
            return
        if self.session and self.session.expires_at <= now:
            session, self.session = self.session, None
            for controller in self.routes:
                self.request_hold(controller)
            self.event("expire", session.source, "source heartbeat timed out; hold requested")

    def release_source(self, request, response):
        self.expire_sessions(monotonic_ns())
        session = self.session
        if session and session.id == request.session_id:
            self.session = None
            response.accepted = True
        response.message = "released" if response.accepted else "unknown or released session"
        self.event("release", session.source if response.accepted else "", response.message)
        return response

    def release_web_for_recenter(self, request, response):
        self.expire_sessions(monotonic_ns())
        session = self.session
        if session and session.source == "web" and session.id == request.session_id:
            self.session = None
            if self.authority == session.source:
                self.authority = None
            response.accepted = True
            response.message = "网页已交还控制权"
            self.event("release", session.source, response.message)
        else:
            response.message = "网页会话已变化，请检查当前控制权"
        return response

    def revoke_source(self, request, response):
        self.expire_sessions(monotonic_ns())
        session = self.session
        if session is None or session.status_id != request.session:
            response.message = "no matching active session"
        else:
            self.session = None
            response.accepted = True
            response.message = "revoked"
        self.event("revoke", session.source if response.accepted else "", response.message)
        return response

    def keep_alive(self, session_id):
        now = monotonic_ns()
        self.expire_sessions(now)
        if self.session is None or self.session.id != session_id:
            return False
        self.session.expires_at = now + self.authority_timeout_ns
        return True

    def source_heartbeat(self, message):
        self.keep_alive(message.data)

    def renew_source(self, request, response):
        response.accepted = self.keep_alive(request.session_id)
        response.message = "renewed" if response.accepted else "unknown or expired session"
        return response

    def on_target(self, frame):
        return self.dispatch(frame.session_id, frame.sequence, MitTarget(target=frame.target))

    def on_mit_target(self, frame):
        return self.dispatch(frame.session_id, frame.sequence, frame.command)

    def set_target(self, request, response):
        response.accepted, response.message, response.dispatch_id = self.on_target(request.frame)
        return response

    def set_mit_target(self, request, response):
        response.accepted, response.message, response.dispatch_id = self.on_mit_target(request.frame)
        return response

    def dispatch(self, session_id, sequence, command):
        now = monotonic_ns()
        self.expire_sessions(now)
        session = self.session if self.session and self.session.id == session_id else None
        source = session.source if session else ""
        try:
            positions, velocities, tff, gravity_ff, friction_ff = self.prepare(command, sequence, session)
        except ValueError as error:
            self.event("reject", source, str(error))
            return False, str(error), 0

        self.dispatch_id += 1
        session.sequence = sequence
        session.expires_at = now + self.authority_timeout_ns
        self.dispatched = {
            "source": source, "session": session.status_id,
            "sequence": sequence, "dispatch_id": str(self.dispatch_id),
            "position": {name: positions[name] for name in self.joints},
            "velocity": {name: velocities.get(name, 0.0) for name in self.joints},
            "tff": {name: tff.get(name, 0.0) for name in self.joints},
            "gravity_ff": gravity_ff,
            "friction_ff": friction_ff,
        }
        for controller, joints in self.routes.items():
            target = JointState(
                name=list(joints), position=[positions[name] for name in joints],
                velocity=[velocities[name] for name in joints] if velocities else [],
                effort=[tff.get(name, 0.0) for name in joints],
            )
            routed_command = make_mit_target(
                target, {name: gravity_ff[name] for name in joints if name in gravity_ff},
                {name: friction_ff[name] for name in joints if name in friction_ff},
            )
            self.outputs[controller].publish(
                AuthorizedTarget(kind=AuthorizedTarget.TARGET, arbiter_id=self.epochs[controller],
                                 hold_revision=self.hold_revisions[controller],
                                 dispatch_id=self.dispatch_id, command=routed_command))
        self.event("dispatch", source, "accepted")
        return True, "accepted for dispatch", self.dispatch_id

    def prepare(self, command, sequence, session):
        if session is None:
            raise ValueError("unknown or released session")
        if sequence <= session.sequence:
            raise ValueError("sequence must increase")
        msg = command.target
        names = msg.name
        if (len(names) != len(self.joints) or set(names) != set(self.joints)
                or len(msg.position) != len(names)
                or (msg.velocity and len(msg.velocity) != len(names))
                or (msg.effort and len(msg.effort) != len(names))):
            raise ValueError("send all seven configured joints exactly once with positions and optional velocities and tff")
        velocities = dict(zip(names, msg.velocity))
        positions = dict(zip(names, msg.position))
        tff = dict(zip(names, msg.effort))
        if not all(isfinite(value) for value in [*positions.values(), *velocities.values(), *tff.values()]):
            raise ValueError("non-finite target")
        gravity_ff = feedforward_by_joint(command.gravity_ff, positions, "gravity_ff")
        friction_ff = feedforward_by_joint(command.friction_ff, positions, "friction_ff")
        for name in gravity_ff.keys() | friction_ff.keys():
            if not isfinite(gravity_ff.get(name, 0.0) + friction_ff.get(name, 0.0) + tff.get(name, 0.0)):
                raise ValueError(f"non-finite combined feedforward for {name}")
        for name, value in positions.items():
            bounds = self.limits[name]
            if not bounds["min_pos"] <= value <= bounds["max_pos"]:
                raise ValueError(f"{name} outside configured position bounds")
        return positions, velocities, tff, gravity_ff, friction_ff

    def on_applied(self, controller, message):
        previous = self.applied.get(controller, 0)
        if previous == message.data:
            return
        if message.data:
            self.applied[controller] = message.data
        else:
            self.applied.pop(controller, None)
        self.publish_status()

    def on_epoch_requested(self, controller, message):
        if message.data != self.epochs[controller]:
            return
        session, self.session = self.session, None
        self.epochs[controller] = self.new_epoch()
        self.publish_heartbeat(controller)
        if session:
            for name in self.routes:
                if name != controller:
                    self.request_hold(name)
        self.event("controller_timeout", session.source if session else "",
                   f"{controller} target bridge lost arbiter heartbeat; previous authority revoked")

    def on_holding(self, controller, message):
        if self.holding.get(controller) != message.data:
            self.holding[controller] = message.data
            self.publish_status()

    def event(self, event, source, message):
        self.last_event = {"event": event, "source": source, "message": message}
        self.publish_status()

    def publish_status(self):
        acknowledgments = {self.applied.get(controller) for controller in self.routes}
        applied = next(iter(acknowledgments)) if len(acknowledgments) == 1 else None
        self.disable_ready = not self.disabling and self.parked(monotonic_ns())
        state = {
            "joints": self.joints,
            "disable_ready": self.disable_ready,
            "feedback_ready": self.feedback_ready,
            "authority": self.authority,
            "source": self.session.source if self.session else None,
            "session": self.session.status_id if self.session else None,
            "holding_after_loss": any(self.holding.values()),
            "last_forwarded": self.dispatched,
            "last_applied_id": str(applied) if applied else None,
            "last_event": self.last_event,
        }
        self.status.publish(String(data=json.dumps(state, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = TargetArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
