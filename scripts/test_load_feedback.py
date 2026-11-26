#!/usr/bin/env python3
"""Verify the load node and console with synthetic MuJoCo force feedback."""

import argparse
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
from socket import socket
import subprocess
import time
from urllib.request import urlopen

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState, InterfaceValue
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
import xacro

from b601_demos.load_api import DEFAULT_LOAD_API, LOAD_FRAME, LOAD_SITES, LOAD_TOPIC, LoadReading, load_snapshot
from b601_target_interfaces.msg import LoadEstimates

ROOT = Path(__file__).resolve().parents[1]
ARM = [f"joint{index}" for index in range(1, 7)]
JOINTS = ARM + ["finger_left", "finger_right"]


class MultiSiteProvider:
    def __init__(self, sites):
        self.readings = {
            name: LoadReading("estimated", (float(index), 0.0, 0.0))
            for index, name in enumerate(sites, start=1)
        }

    def estimate(self, _feedback, _robot_description):
        return self.readings


def create_multi_provider(sites):
    return MultiSiteProvider(sites)


class Fixture(Node):
    def __init__(self):
        super().__init__("b601_load_feedback_test")
        self.latest = None
        self.description = self.create_publisher(
            String, "/robot_description", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher = self.create_publisher(DynamicJointState, "/dynamic_joint_states", qos_profile_sensor_data)
        self.create_subscription(LoadEstimates, LOAD_TOPIC, self.receive, qos_profile_sensor_data)
        self.tare = self.create_client(Trigger, "/b601_load/tare")

        source = Path(get_package_share_directory("b601_description")) / "urdf/b601_dm.urdf.xacro"
        path = Path(get_package_share_directory("b601_mujoco")) / "models/b601_arm_physical.xml"
        self.urdf = xacro.process_file(
            str(source), mappings={"hardware": "mujoco", "mujoco_model": str(path)}).toxml()
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.positions = [0.32, -0.76, -0.98, 0.41, 0.33, 0.25, 0.02, -0.02]
        for name, value in zip(JOINTS, self.positions):
            self.data.joint(name).qpos[0] = value
        mujoco.mj_forward(self.model, self.data)
        self.dofs = [int(self.model.joint(name).dofadr[0]) for name in JOINTS]
        self.load = np.zeros(3)
        self.send_feedback = True
        self.create_timer(0.005, self.publish_feedback)

    def receive(self, message):
        self.latest = load_snapshot(message)

    def publish_feedback(self):
        if not self.send_feedback:
            return
        site = LOAD_SITES["gripper"]
        body = self.model.body(site.link).id
        xyz = self.data.xpos[body] + self.data.xmat[body].reshape(3, 3) @ site.xyz_m
        generalized_force = np.zeros(self.model.nv)
        mujoco.mj_applyFT(self.model, self.data, self.load, np.zeros(3), xyz, body, generalized_force)
        effort = self.data.qfrc_bias - self.data.qfrc_passive - generalized_force
        values = [InterfaceValue(interface_names=["position", "velocity", "effort"],
                                 values=[position, 0.0, float(effort[dof])])
                  for position, dof in zip(self.positions, self.dofs)]
        message = DynamicJointState(joint_names=JOINTS, interface_values=values)
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)

    def wait_for(self, condition):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if condition():
                return
            rclpy.spin_once(self, timeout_sec=0.02)
        raise AssertionError(f"timeout waiting for {condition.__name__}; last estimate: {self.latest}")

    def status_is(self, site, status):
        return self.latest and self.latest["estimates"].get(site, {}).get("status") == status


@contextmanager
def run_console(*options):
    with socket() as port_socket:
        port_socket.bind(("127.0.0.1", 0))
        port = port_socket.getsockname()[1]
    log_path = ROOT / ".verify" / f"load-web-{port}.log"
    log_path.parent.mkdir(exist_ok=True)
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(ROOT / "scripts"), env.get("PYTHONPATH")]))
    with log_path.open("w") as log:
        console = subprocess.Popen(
            [str(ROOT / "scripts/start_target_web.sh"), f"port:={port}", *options],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env,
        )
        try:
            yield f"http://127.0.0.1:{port}"
        except Exception:
            print(f"console log: {log_path}", flush=True)
            raise
        finally:
            if console.poll() is None:
                console.send_signal(signal.SIGINT)
            try:
                console.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(console.pid, signal.SIGKILL)
                console.wait()


def web_read(address, path):
    try:
        with urlopen(address + path, timeout=0.5) as response:
            return json.load(response)
    except OSError:
        return {}


def check(node):
    node.description.publish(String(data=node.urdf))
    for options in ((), ("load_positions:=wrist",)):
        with run_console(*options) as server:
            node.wait_for(lambda: "load" in web_read(server, "/api/state"))
            assert web_read(server, "/api/state")["load"] is None
            assert web_read(server, "/api/load") is None
            assert not node.tare.service_is_ready()
            assert not node.get_publishers_info_by_topic(LOAD_TOPIC)
            assert all(info.node_name != "b601_target_web"
                       for info in node.get_subscriptions_info_by_topic(LOAD_TOPIC))
    print("load feedback disabled without an explicit API", flush=True)

    with run_console(f"load_api:={DEFAULT_LOAD_API}") as server:
        node.wait_for(lambda: node.status_is("gripper", "uncalibrated") and node.tare.service_is_ready())
        result = node.tare.call_async(Trigger.Request())
        node.wait_for(result.done)
        assert result.result().success, result.result().message
        node.wait_for(lambda: node.status_is("gripper", "no_contact"))
        assert node.latest["estimates"]["gripper"]["magnitude_n"] == 0
        assert node.latest["estimates"]["gripper"]["direction"] is None

        node.load[:] = [4.0, -6.0, -8.0]
        node.wait_for(lambda: node.status_is("gripper", "estimated"))

        def web_estimate_ready():
            state = web_read(server, "/api/load")
            reading = state.get("estimates", {}).get("gripper", {})
            return reading.get("status") == "estimated" and np.allclose(reading["force_n"], node.load, atol=0.01)

        node.wait_for(web_estimate_ready)
        load = web_read(server, "/api/state")["load"]
        assert load["positions"] == ["gripper"] and load["frame_id"] == LOAD_FRAME and load["stamp_ns"]
        reading = load["estimates"]["gripper"]
        np.testing.assert_allclose(reading["force_n"], node.load, atol=0.05)
        np.testing.assert_allclose(reading["direction"], node.load / np.linalg.norm(node.load), atol=0.01)
        np.testing.assert_allclose(reading["magnitude_n"], np.linalg.norm(node.load), atol=0.05)
        assert reading["link"] == "end_link" and reading["point_xyz_m"] == list(LOAD_SITES["gripper"].xyz_m)
        print(f"gripper estimate with explicit contact provider: {reading}", flush=True)

        node.send_feedback = False
        node.wait_for(lambda: node.status_is("gripper", "waiting_feedback"))
        node.wait_for(lambda: web_read(server, "/api/load").get("stamp_ns", "missing") is None)
        assert web_read(server, "/api/load")["estimates"]["gripper"]["force_n"] is None

    node.latest = None
    node.send_feedback = True
    api = "test_load_feedback:create_multi_provider"
    with run_console("load_positions:=upper_arm,forearm,wrist,gripper", f"load_api:={api}") as server:
        def all_sites_ready():
            load = web_read(server, "/api/load")
            readings = load.get("estimates", {})
            return load.get("api") == api and set(readings) == set(LOAD_SITES) and all(
                item["status"] == "estimated" for item in readings.values())

        node.wait_for(all_sites_ready)
        load = web_read(server, "/api/load")
        assert load["positions"] == list(LOAD_SITES)
        for index, (name, site) in enumerate(LOAD_SITES.items(), start=1):
            reading = load["estimates"][name]
            assert reading["link"] == site.link
            assert reading["point_xyz_m"] == list(site.xyz_m)
            assert reading["force_n"] == [float(index), 0.0, 0.0]
            assert reading["direction"] == [1.0, 0.0, 0.0]
        print("external API and four independent location readings verified", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", default="147", help="ROS domain reserved for this offline fixture")
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=args.domain, ROS_LOCALHOST_ONLY="1")
    rclpy.init(args=[])
    node = Fixture()
    try:
        check(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
