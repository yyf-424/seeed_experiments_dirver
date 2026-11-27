"""Select and refresh external gravity independently of motion targets."""

from collections.abc import Mapping
from importlib import import_module
import json
from math import isfinite
from threading import Lock
from time import monotonic
from uuid import uuid4

from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from b601_description.project_config import load_project_config
from b601_target_interfaces.msg import GravityCompensation, JointFeedforward
from b601_target_interfaces.srv import SetGravitySource

from .mit_target import measured_positions
from .target_bridge import BRIDGE, LATCHED_QOS


class GravityProvider:
    def __init__(self, node, joints, feedback_timeout, *, source=None, api=None, callback_group=None):
        explicit_internal = source == "internal"
        source = self.parameter(node, "gravity_source", "internal") if source is None else source
        api = self.parameter(node, "gravity_api", "") if api is None else api
        self.joints = joints
        self.feedback_timeout = feedback_timeout
        self.logger = node.get_logger()
        self.lock = Lock()
        self.id = uuid4().hex
        self.bridge = None
        self.bridge_received_at = None
        self.feedback = ({}, None)
        self.sent_feedback_at = None
        self.future = None
        self.response_deadline = None
        self.failure = None
        self.source = "internal"
        self.api = api
        self.evaluate = None
        self.pending = False
        self.select_source(source or "internal", api)
        self.pending = source == "external" or explicit_internal
        self.selector = node.create_client(
            SetGravitySource, f"{BRIDGE}/set_gravity_source", callback_group=callback_group)
        update_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            lifespan=Duration(nanoseconds=int(feedback_timeout * 1e9)))
        self.updates = node.create_publisher(GravityCompensation, f"{BRIDGE}/gravity_update", update_qos)
        node.create_subscription(
            String, f"{BRIDGE}/gravity_status", self.on_status, LATCHED_QOS, callback_group=callback_group)
        timeout = load_project_config()["tools"]["target_arbitration"]["authority_timeout_sec"]
        node.create_timer(
            timeout / 5, self.tick, callback_group=callback_group, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def parameter(node, name, default):
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        return node.get_parameter(name).value

    @staticmethod
    def load_factory(api):
        module_name, separator, factory_name = api.partition(":")
        if not separator or not module_name or not factory_name:
            raise ValueError("external gravity requires gravity_api=module:factory")
        try:
            factory = import_module(module_name)
            for attribute in factory_name.split("."):
                factory = getattr(factory, attribute)
            evaluate = factory().gravity_ff
            if not callable(evaluate):
                raise TypeError("the provider must implement gravity_ff(measured_positions)")
        except (Exception, SystemExit) as error:
            raise ValueError(f"cannot load gravity_api {api}: {error}") from error
        return evaluate

    def select_source(self, source, api=None, *, apply=True):
        """Prepare a gravity source and optionally request selection on the bridge."""
        if source not in ("internal", "external"):
            raise ValueError("gravity_source must be internal or external")
        with self.lock:
            api = self.api if api is None else api
            evaluate = None
            if source == "external":
                evaluate = self.evaluate if api == self.api else None
                evaluate = evaluate or self.load_factory(api)
            self.source, self.api, self.evaluate = source, api, evaluate
            self.pending = apply
            self.failure = None

    def on_status(self, message):
        status = json.loads(message.data)
        with self.lock:
            if self.bridge and self.bridge["epoch"] != status["epoch"]:
                if self.future is not None:
                    self.selector.remove_pending_request(self.future)
                    self.future = None
                    self.pending = True
                if self.bridge["producer_id"] == self.id and self.source == "external":
                    self.pending = True
            self.bridge = status
            self.bridge_received_at = monotonic()

    def snapshot(self):
        with self.lock:
            bridge = self.bridge or {}
            bridge_live = (self.bridge_received_at is not None
                           and monotonic() - self.bridge_received_at < self.feedback_timeout)
            return {
                "requested": self.source, "api": self.api,
                "pending": self.pending or self.future is not None,
                "selected": bridge.get("source"),
                "effective": bridge.get("effective_source") if bridge_live else None,
                "stale": bridge.get("source") == "external" and (not bridge_live or bridge.get("stale", False)),
                "error": self.failure,
            }

    def entries(self, sample):
        positions = measured_positions(self.joints, sample, self.feedback_timeout)
        if positions is None:
            return None
        try:
            values = self.evaluate(positions)
            if not isinstance(values, Mapping) or not values or not values.keys() <= positions.keys():
                raise ValueError("gravity_ff must contain efforts keyed by controlled joint name")
            entries = [JointFeedforward(joint=name, effort=float(value)) for name, value in values.items()]
            if not all(isfinite(entry.effort) for entry in entries):
                raise ValueError("gravity_ff must contain finite effort values")
        except Exception as error:
            reason = str(error)
            if reason != self.failure:
                self.logger.warning(f"external gravity API failed: {reason}")
            self.failure = reason
            return None
        self.failure = None
        return entries

    def tick(self):
        with self.lock:
            if self.future is not None:
                if self.future.done():
                    try:
                        reply = self.future.result()
                        if not reply.accepted:
                            self.failure = reply.message
                    except Exception as error:
                        self.failure = str(error)
                elif monotonic() < self.response_deadline:
                    return
                else:
                    self.selector.remove_pending_request(self.future)
                    self.failure = "未收到重力来源切换回执"
                self.future = None
            sample = self.feedback
            if self.pending:
                if not self.selector.service_is_ready():
                    return
                entries = self.entries(sample) if self.evaluate is not None else []
                if entries is None:
                    return
                command = GravityCompensation(producer_id=self.id, efforts=entries)
                self.future = self.selector.call_async(SetGravitySource.Request(command=command))
                self.response_deadline = monotonic() + 3.0
                self.pending = False
                self.sent_feedback_at = sample[1]
            elif (self.evaluate is not None and self.bridge and self.bridge["producer_id"] == self.id
                  and sample[1] != self.sent_feedback_at):
                entries = self.entries(sample)
                if entries is not None:
                    self.updates.publish(GravityCompensation(producer_id=self.id, efforts=entries))
                    self.sent_feedback_at = sample[1]
