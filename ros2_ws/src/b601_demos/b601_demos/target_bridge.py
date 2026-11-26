"""Forward authorized position, velocity and feedforward to the MIT controller."""
import json
from collections import deque
from math import isfinite
from time import monotonic_ns

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, UInt64

from b601_description.project_config import load_project_config
from b601_target_interfaces.msg import AuthorizedTarget, MitTarget


class ControllerRoute:
    def __init__(self, node, name, joints, depth):
        self.node = node
        self.joints = tuple(joints)
        self.command = node.create_publisher(MitTarget, f"/{name}/joint_target", depth)
        self.applied = node.create_publisher(UInt64, f"/{name}/target_applied", depth)
        self.holding = node.create_publisher(Bool, f"/{name}/target_holding", depth)
        self.epoch_request = node.create_publisher(UInt64, f"/{name}/target_epoch_requested", depth)
        node.create_subscription(AuthorizedTarget, f"/{name}/authorized_target", self.on_frame, depth)
        node.create_subscription(Float64MultiArray, f"/{name}/state", self.on_state, depth)

        self.epoch = 0
        self.revision = 0
        self.heartbeat_ns = None
        self.blocked = False
        self.queued = None
        self.hold_requested = False
        self.hold_confirmed = False
        self.dispatch_id = 0
        self.state_ns = None
        self.sequence = 0
        self.sent_sequence = 0
        self.pending = deque(maxlen=depth)

    def enter_hold(self):
        self.queued = None
        self.hold_requested = True
        self.dispatch_id = 0
        self.pending.clear()

    def check_heartbeat(self, now):
        if (self.heartbeat_ns is not None and not self.blocked
                and now - self.heartbeat_ns >= self.node.authority_timeout_ns):
            self.blocked = True
            self.enter_hold()

    def on_frame(self, frame):
        now = monotonic_ns()
        self.check_heartbeat(now)
        heartbeat = frame.kind == AuthorizedTarget.HEARTBEAT
        if frame.arbiter_id == 0 or (not heartbeat and frame.kind != AuthorizedTarget.TARGET):
            return
        if frame.arbiter_id != self.epoch:
            if not heartbeat:
                return
            if self.epoch:
                self.enter_hold()
            self.epoch = frame.arbiter_id
            self.revision = frame.hold_revision
            self.blocked = False
        elif frame.hold_revision < self.revision:
            return
        elif frame.hold_revision > self.revision:
            self.revision = frame.hold_revision
            self.enter_hold()
        if heartbeat:
            self.heartbeat_ns = now
        elif not self.blocked and frame.dispatch_id:
            self.queued = frame
        self.flush(now)

    def state_is_fresh(self, now):
        return self.state_ns is not None and now - self.state_ns <= self.node.feedback_timeout_ns

    def on_state(self, message):
        count = len(self.joints)
        if len(message.data) != 8 * count + 4:
            return
        value = message.data[8 * count + 1]
        if not isfinite(value) or value < 0 or not value.is_integer():
            return
        now = monotonic_ns()
        sequence = int(value)
        if not self.state_is_fresh(now) or not self.sequence <= sequence <= self.sent_sequence:
            self.pending.clear()
            self.sent_sequence = sequence
            self.dispatch_id = 0
        self.state_ns = now
        self.sequence = sequence
        # The stock controller reports its last consumed sequence, including skips
        # when multiple commands arrive between updates.
        while self.pending and self.pending[0][0] <= sequence:
            expected, dispatch = self.pending.popleft()
            if expected == sequence:
                self.dispatch_id = dispatch
                self.hold_confirmed = dispatch == 0
                self.publish_status()
        self.check_heartbeat(now)
        self.flush(now)

    def send(self, target, dispatch):
        # joint_target has one normal writer. Each valid message increments
        # the stock controller's sequence once.
        self.command.publish(target)
        self.sent_sequence += 1
        self.pending.append((self.sent_sequence, dispatch))

    def flush(self, now):
        if not self.state_is_fresh(now) or not self.command.get_subscription_count():
            return
        if self.hold_requested:
            positions = self.node.measured_positions(self.joints, now)
            if positions is None:
                return
            self.send(MitTarget(target=JointState(name=list(self.joints), position=positions)), 0)
            self.hold_requested = False
        if self.queued is not None:
            self.send(self.queued.command, self.queued.dispatch_id)
            self.queued = None

    def publish_status(self):
        self.applied.publish(UInt64(data=self.dispatch_id))
        self.holding.publish(Bool(data=self.hold_confirmed))

    def tick(self, now):
        self.check_heartbeat(now)
        self.flush(now)
        if self.blocked:
            self.epoch_request.publish(UInt64(data=self.epoch))
        self.publish_status()


class TargetBridge(Node):
    def __init__(self):
        super().__init__("b601_target_bridge")
        source = self.declare_parameter("project_config_file", "").value
        config = load_project_config(source or None)
        routes = json.loads(self.declare_parameter("routes", "{}").value)
        timeout = config["tools"]["target_arbitration"]["authority_timeout_sec"]
        self.authority_timeout_ns = int(timeout * 1e9)
        self.feedback_timeout_ns = int(config["safety"]["feedback_timeout_sec"] * 1e9)
        self.feedback = {}
        self.feedback_ns = None
        depth = config["implementation"]["mit"]["topic_depth"]
        self.routes = [ControllerRoute(self, name, joints, depth) for name, joints in routes.items()]
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_timer(timeout / 5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_feedback(self, state):
        self.feedback = {name: value for name, value in zip(state.name, state.position) if isfinite(value)}
        self.feedback_ns = monotonic_ns()

    def measured_positions(self, joints, now):
        if self.feedback_ns is None or now - self.feedback_ns > self.feedback_timeout_ns:
            return None
        if all(name in self.feedback for name in joints):
            return [self.feedback[name] for name in joints]
        return None

    def tick(self):
        now = monotonic_ns()
        for route in self.routes:
            route.tick(now)


def main(args=None):
    rclpy.init(args=args)
    node = TargetBridge()
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
