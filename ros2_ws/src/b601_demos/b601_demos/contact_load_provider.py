"""Adapter for the separately maintained load_identification observer."""

from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from b601_load_id.contact.dynamics import ContactDynamics
from b601_load_id.contact.estimator import ContactEstimator
from b601_load_id.contact.node import read_feedback
from b601_load_id.model import Point, read_gravity
import numpy as np

from .load_api import LoadReading


class ContactLoadProvider:
    def __init__(self, sites):
        self.sites = tuple(sites)
        self.points = [Point(name, site.link, np.array(site.xyz_m)) for name, site in sites.items()]
        self.description = None
        self.dynamics = None
        self.observer = None
        self.profile = None

    def configure(self, description):
        self.dynamics = None
        self.observer = None
        self.profile = None
        hardware = ElementTree.fromstring(description).find("./ros2_control/hardware")
        if hardware is None:
            return
        plugin = hardware.findtext("plugin")
        gravity = None
        zero_gravity = False
        if plugin == "b601_hardware/DmSystemHardware":
            models = Path(get_package_share_directory("b601_mujoco")) / "models"
            path = models / "b601_arm.xml"
            project = Path(get_package_share_directory("b601_description")) / "config/b601_project.yaml"
            gravity, _ = read_gravity(project)
            effort_sign = -1  # Same as the existing B601 physical observer's measured DM convention.
        elif plugin == "b601_hardware/MujocoSystemHardware":
            path = Path(hardware.findtext("param[@name='model_path']"))
            zero_gravity = path.name == "b601_arm_free.xml"
            if zero_gravity:
                path = path.with_name("b601_arm.xml")
            effort_sign = 1
        else:
            return
        self.dynamics = ContactDynamics(path, gravity, effort_sign)
        if zero_gravity:
            self.dynamics.model.opt.gravity[:] = 0.0
        # Existing load_identification defaults, including the reference/noise policy.
        self.observer = ContactEstimator(self.sites, None, 0.060, 0.015, 1.0, 0.02, 3.0)

    def estimate(self, feedback, robot_description):
        if not robot_description:
            return dict.fromkeys(self.sites, LoadReading("waiting_description"))
        if robot_description != self.description:
            self.configure(robot_description)
            self.description = robot_description
        if self.observer is None:
            return dict.fromkeys(self.sites, LoadReading("unsupported_backend"))
        try:
            stamp, profile, states = read_feedback(feedback)
        except (ValueError, KeyError):
            self.profile = None
            return dict.fromkeys(self.sites, LoadReading("invalid_feedback"))
        values = self.dynamics.terms(*states, self.points)
        result = self.observer.update(stamp * 1e-9, profile, *values)
        self.profile = profile
        readings = dict.fromkeys(self.sites, LoadReading(result.status))
        if result.status == "no_contact":
            return dict.fromkeys(self.sites, LoadReading("no_contact", (0.0, 0.0, 0.0)))
        if result.index is not None:
            # The observer produces MJCF world axes. Express the force in the
            # fixed model base axes before consumers apply the URDF world transform.
            body = self.dynamics.model.body("base_link").id
            rotation = self.dynamics.data.xmat[body].reshape(3, 3)
            force = rotation.T @ result.force_n
            readings = dict.fromkeys(self.sites, LoadReading("unresolved"))
            readings[self.sites[result.index]] = LoadReading("estimated", tuple(force))
        return readings

    def tare(self):
        if self.profile is None:
            raise RuntimeError("尚未收到完整的负载辨识反馈")
        self.observer.start_tare(self.profile)
        return f"正在清零 {self.profile}，请保持无外力 {self.observer.tare_s:g} 秒"


def create_provider(sites):
    return ContactLoadProvider(sites)
