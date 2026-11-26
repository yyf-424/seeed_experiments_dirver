"""Load and invoke a node's optional external gravity model."""
from collections.abc import Mapping
from importlib import import_module
from math import isfinite
from threading import Lock

from .mit_target import measured_positions


class GravityProvider:
    def __init__(self, node, joints, feedback_timeout, *, source=None, api=None):
        if source is None:
            source = self.parameter(node, "gravity_source", "internal")
        if api is None:
            api = self.parameter(node, "gravity_api", "")
        self.joints = joints
        self.feedback_timeout = feedback_timeout
        self.logger = node.get_logger()
        self.source = source or "internal"
        self.api = api if self.source == "external" else ""
        self.evaluate = None
        self.failure = None
        self.lock = Lock()
        if source in ("", "internal"):
            return
        if source != "external":
            raise ValueError("gravity_source must be internal or external")
        module_name, separator, factory_name = api.partition(":")
        if not separator or not module_name or not factory_name:
            raise ValueError("external gravity requires gravity_api=module:factory")
        try:
            factory = import_module(module_name)
            for attribute in factory_name.split("."):
                factory = getattr(factory, attribute)
            self.evaluate = factory().gravity_ff
            if not callable(self.evaluate):
                raise TypeError("the provider must implement gravity_ff(measured_positions)")
        except (Exception, SystemExit) as error:
            raise ValueError(f"cannot load gravity_api {api}: {error}") from error

    @staticmethod
    def parameter(node, name, default):
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        return node.get_parameter(name).value

    def resolve(self, explicit, feedback_sample):
        if explicit is not None or self.evaluate is None:
            return explicit
        positions = measured_positions(self.joints, feedback_sample, self.feedback_timeout)
        if positions is None:
            return None
        with self.lock:
            try:
                values = self.evaluate(positions)
                if not isinstance(values, Mapping) or not values.keys() <= positions.keys():
                    raise ValueError("gravity_ff must be keyed by controlled joint name")
                efforts = {name: float(value) for name, value in values.items()}
                if not all(isfinite(effort) for effort in efforts.values()):
                    raise ValueError("gravity_ff must contain finite effort values")
            except Exception as error:
                reason = str(error)
                if reason != self.failure:
                    self.logger.warning(f"external gravity API failed: {reason}")
                self.failure = reason
                return None
            self.failure = None
            return efforts
