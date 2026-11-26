"""Publish external load estimates independently from the Web and control stack."""

from importlib import import_module
from time import monotonic

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from b601_description.project_config import load_project_config
from b601_target_interfaces.msg import LoadEstimates

from .load_api import DEFAULT_LOAD_API, LOAD_SITES, LOAD_TOPIC, load_estimates, parse_load_positions


class LoadNode(Node):
    def __init__(self):
        super().__init__("b601_load")
        positions = parse_load_positions(self.declare_parameter("load_positions", "gripper").value)
        self.sites = {name: LOAD_SITES[name] for name in positions}
        api = self.declare_parameter("load_api", DEFAULT_LOAD_API).value
        config_path = self.declare_parameter("project_config_file", "").value
        self.timeout = load_project_config(config_path or None)["safety"]["feedback_timeout_sec"]
        self.provider = None
        self.failure = None
        try:
            module, separator, factory = api.partition(":")
            if not separator:
                raise ValueError("load_api must be module:factory")
            factory_object = import_module(module)
            for attribute in factory.split("."):
                factory_object = getattr(factory_object, attribute)
            provider = factory_object(self.sites.copy())
            if not callable(provider.estimate):
                raise TypeError("load provider must implement estimate(feedback, robot_description)")
            self.provider = provider
        except (Exception, SystemExit) as error:
            self.report_error(error)

        self.robot_description = None
        self.latest = None
        self.last_stamp = None
        self.received_at = None
        self.publisher = self.create_publisher(LoadEstimates, LOAD_TOPIC, qos_profile_sensor_data)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(
            String, "/robot_description", self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "~/tare", self.tare)
        self.create_timer(0.025, self.publish_estimates)

    def report_error(self, error):
        reason = str(error)
        if reason != self.failure:
            self.get_logger().warning(f"load API unavailable: {reason}")
        self.failure = reason

    def on_description(self, message):
        if self.robot_description != message.data:
            self.robot_description = message.data
            self.latest = None
            self.last_stamp = None

    def on_feedback(self, message):
        if self.provider is None:
            return
        stamp = message.header.stamp
        if not (stamp.sec or stamp.nanosec) or stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        self.received_at = monotonic()
        try:
            readings = self.provider.estimate(message, self.robot_description)
            self.latest = load_estimates(self.sites, readings, "unavailable", stamp)
        except Exception as error:
            self.report_error(error)
            self.latest = load_estimates(self.sites, {}, "provider_error", stamp)
        else:
            self.failure = None

    def live(self):
        return self.latest is not None and monotonic() - self.received_at <= self.timeout

    def tare(self, _request, response):
        reset = getattr(self.provider, "tare", None)
        if self.live() and self.failure is None and callable(reset):
            try:
                response.message = reset()
            except Exception as error:
                response.message = str(error)
            else:
                response.success = True
                self.latest = load_estimates(self.sites, {}, "taring", self.last_stamp)
        else:
            response.message = "当前未收到反馈，或负载 API 不提供清零"
        return response

    def publish_estimates(self):
        status = "provider_error" if self.provider is None else "waiting_feedback"
        message = self.latest if self.live() else load_estimates(self.sites, {}, status)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LoadNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
