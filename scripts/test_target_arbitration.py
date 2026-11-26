#!/usr/bin/env python3
"""Start an isolated MuJoCo MIT fixture for integration checks or web preview."""
import argparse
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from urllib.request import Request, urlopen

import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from b601_description.project_config import load_project_config
from b601_demos.target_arbiter import ARBITER
from b601_demos.target_client import TargetClient
from b601_target_interfaces.msg import JointFeedforward, TargetFrame
from b601_target_interfaces.srv import RevokeSource, SelectSource, SetAuthority, SetMitTarget, SetTarget

ARM = "b601_mit_arm"
GRIPPER = "b601_mit_gripper"
TIMEOUT = 8.0


class TestNode(Node):
    def __init__(self):
        super().__init__("target_arbitration_integration")
        self.feedback = {}
        self.status = {}
        self.safety = None
        self.holds = {}
        self.ff = {}
        self.controller_state = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ARBITER}/status", self.on_status, latched)
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(DiagnosticStatus, "/b601_safety/status", self.on_safety, latched)
        for controller in (ARM, GRIPPER):
            self.create_subscription(
                Bool, f"/{controller}/target_holding",
                lambda msg, controller=controller: self.on_hold(controller, msg), 10)
            self.create_subscription(
                Float64MultiArray, f"/{controller}/ff",
                lambda msg, controller=controller: self.ff.update({controller: list(msg.data)}), 10)
            self.create_subscription(
                Float64MultiArray, f"/{controller}/state",
                lambda msg, controller=controller: self.controller_state.update(
                    {controller: list(msg.data)}), 10)

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_feedback(self, message):
        self.feedback = dict(zip(message.name, message.position))

    def on_hold(self, controller, message):
        self.holds[controller] = message.data

    @property
    def watchdog_hold(self):
        return self.holds.get(ARM, False)

    def on_safety(self, message):
        self.safety = message.level

    def until(self, condition, label, timeout=TIMEOUT):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not condition():
            raise AssertionError(f"{label}; status={self.status}; feedback={self.feedback}; safety={self.safety}")

    def applied(self, dispatch_id):
        self.until(lambda: self.status.get("last_applied_id") == str(dispatch_id),
                   f"all routes must acknowledge dispatch {dispatch_id}")

    def check_tff(self, arm, gripper, label):
        self.until(lambda: self.ff.get(ARM, [])[12:18] == arm
                   and self.ff.get(GRIPPER, [])[2:] == [gripper], label, timeout=2)

    def check_compensation(self, label, *, gravity=None, friction=None, finger_friction=None):
        def matches():
            arm = self.ff.get(ARM, [])
            finger = self.ff.get(GRIPPER, [])
            return (len(arm) == 18 and len(finger) == 3
                    and all(arm[6 + index] == value for index, value in (gravity or {}).items())
                    and all(arm[index] == value for index, value in (friction or {}).items())
                    and (finger_friction is None or finger[0] == finger_friction))

        self.until(matches, label, timeout=2)

    def reaches(self, values, tolerance=0.004):
        self.until(
            lambda: all(joint in self.feedback and abs(self.feedback[joint] - goal) < tolerance
                        for joint, goal in values.items()),
            f"MuJoCo must reach {values}",
        )


def test_tff(node, client, positions=None):
    names = node.status["joints"]
    if positions is None:
        positions = {name: node.feedback[name] for name in names}
    zeros = [0.0] * 6
    arm = [0.25, *zeros[1:]]
    client.select(TIMEOUT)
    try:
        for effort in ([0.25], [float("nan"), *zeros]):
            malformed = client.frame(JointState(
                name=names, position=list(positions.values()), effort=effort))
            assert not client.call(client.sender, SetTarget.Request(frame=malformed), TIMEOUT).accepted

        dispatch = client.set_joints(
            positions, TIMEOUT, velocities={"joint1": 0.01},
            tff={"joint1": arm[0], "finger_left": 0.1})
        node.applied(dispatch)
        node.check_tff(arm, 0.1, "SDK point target must apply both task feedforward groups")
        assert node.status["last_forwarded"]["velocity"]["joint1"] == 0.01
        dispatch = client.set_joints(positions, TIMEOUT)
        node.applied(dispatch)
        node.check_tff(zeros, 0.0, "omitting tff must clear both groups on the next target")
        assert not any(node.status["last_forwarded"]["tff"].values())

        client.publisher.publish(client.frame(JointState(
            name=list(reversed(names)), position=list(reversed(positions.values())),
            effort=list(reversed([*arm, 0.1])))))
        node.check_tff(arm, 0.1, "raw stream must match feedforward by joint name")
        node.check_tff(zeros, 0.0, "stale task feedforward must clear with the session still active")
        assert node.status["source"] == client.source
    finally:
        client.release(TIMEOUT)
    print("PASS SDK and raw ROS 2 task feedforward, invalid frames, omitted zero and timeout")


def test_compensation(node, client, positions=None):
    has_internal_gravity = positions is not None
    if positions is None:
        positions = {name: node.feedback[name] for name in node.status["joints"]}

    def internal_gravity_restored():
        gravity = node.ff[ARM][7]
        internal = -node.controller_state[ARM][7]
        return abs(gravity - internal) < 0.02 and (not has_internal_gravity or abs(gravity) > 0.2)

    client.select(TIMEOUT)
    try:
        target = JointState(name=list(reversed(positions)), position=list(reversed(positions.values())))
        frame = client.frame(target, gravity_ff={"joint2": 0.0})
        for invalid in (
            JointFeedforward(joint="unknown", effort=0.1),
            JointFeedforward(joint="joint2", effort=0.1),
            JointFeedforward(joint="joint3", effort=float("nan")),
        ):
            frame.command.gravity_ff = [JointFeedforward(joint="joint2", effort=0.0), invalid]
            assert not client.call(client.mit_sender, SetMitTarget.Request(frame=frame), TIMEOUT).accepted

        dispatch = client.set_joints(
            positions, TIMEOUT, gravity_ff={"joint2": 0.0},
            friction_ff={"joint1": 0.2, "finger_left": -0.1}, tff={"joint1": 0.25})
        node.applied(dispatch)
        node.check_compensation("must apply sparse overrides including zero and both groups",
                                gravity={1: 0.0}, friction={0: 0.2}, finger_friction=-0.1)
        node.check_tff([0.25, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, "independent task feedforward")
        node.until(lambda: abs(node.controller_state[ARM][0] - 0.45) < 1e-8,
                   "controller must sum friction and task feedforward")
        assert node.status["last_forwarded"]["gravity_ff"] == {"joint2": 0.0}
        if has_internal_gravity:
            assert abs(node.ff[ARM][8]) > 0.2, "unspecified J3 must still use internal gravity"

        client.publish(target, gravity_ff={"joint2": 0.15})
        node.check_compensation("new gravity-only stream frame must clear friction overrides",
                                gravity={1: 0.15}, friction={0: 0.0}, finger_friction=0.0)
        node.until(internal_gravity_restored, "timed out external gravity must restore internal gravity")
        assert node.status["source"] == client.source

        client.publish(target, gravity_ff={"joint2": 0.2})
        node.check_compensation("new stream frame must apply external gravity", gravity={1: 0.2})
        node.applied(client.set_joints(positions, TIMEOUT))
        node.until(internal_gravity_restored, "legacy target must restore internal gravity")
        assert node.status["last_forwarded"]["gravity_ff"] == {}
    finally:
        client.release(TIMEOUT)
    print("PASS external gravity and friction routing, explicit zero, timeout and internal fallback")


def test_delta_target(node, client):
    names = node.status["joints"]
    node.until(lambda: all(name in client.feedback for name in names), "SDK must receive a complete feedback sample")
    arbiter_services = dict(node.get_service_names_and_types_by_node("b601_target_arbiter", "/"))
    assert f"{ARBITER}/set_delta_target" not in arbiter_services
    assert not node.get_subscriptions_info_by_topic(f"{ARBITER}/delta_target")
    client.select(TIMEOUT)
    try:
        for deltas, expected in (({}, "controlled joint"), ({"finger_right": 0.01}, "controlled joint"),
                                 ({"joint1": float("nan")}, "finite"), ({"joint1": 10.0}, "position bounds")):
            try:
                client.set_joint_deltas(deltas, TIMEOUT)
            except (ValueError, RuntimeError) as error:
                assert expected in str(error), error
            else:
                raise AssertionError(f"invalid delta accepted: {deltas}")

        measured = {name: node.feedback[name] for name in names}
        previous = {**measured, "joint1": measured["joint1"] + 0.25}
        client.set_joints(previous, TIMEOUT)
        measured = dict(node.feedback)
        increments = {"finger_left": 0.0004, "joint1": 0.035}
        dispatch = client.set_joint_deltas(increments, TIMEOUT)
        node.applied(dispatch)
        resolved = node.status["last_forwarded"]["position"]
        assert client.positions == resolved
        assert abs(resolved["joint1"] - measured["joint1"] - increments["joint1"]) < 0.05
        assert abs(resolved["joint1"] - previous["joint1"]) > 0.1
        assert abs(resolved["finger_left"] - measured["finger_left"] - increments["finger_left"]) < 0.00015
        assert abs(resolved["joint4"] - measured["joint4"]) < 0.005
        node.reaches({"joint1": resolved["joint1"]})

        replay = client.frame(JointState(name=list(resolved), position=list(resolved.values())))
        replay.sequence -= 1
        result = client.call(client.sender, SetTarget.Request(frame=replay), TIMEOUT)
        assert not result.accepted and "sequence" in result.message, result
        client.publish_joint_deltas({"joint1": -0.008})
        node.until(lambda: node.status["last_forwarded"]["sequence"] == client.sequence,
                   "streaming delta target must share the absolute target sequence")
        node.applied(node.status["last_forwarded"]["dispatch_id"])
        assert client.positions == node.status["last_forwarded"]["position"]
    finally:
        client.release(TIMEOUT)
    print("PASS SDK converts measured-relative targets before sending full ROS targets to the arbiter")


def test_http_and_cli(node, port, positions=None):
    def request(path, body=None):
        payload = json.dumps(body).encode() if body is not None else None
        req = Request(
            f"http://127.0.0.1:{port}/api/{path}", data=payload,
            headers={"Content-Type": "application/json"})
        with urlopen(req, timeout=3) as reply:
            return json.load(reply)

    if positions is None:
        positions = {name: node.feedback[name] for name in node.status["joints"]}
    internal = dict.fromkeys(positions, "internal")
    internal_sources = {"gravity": internal, "friction": internal}
    node.until(lambda: request("state")["compensation_sources"] == internal_sources,
               "web must read internal sources from both controllers")
    session = request("select", {})["session_id"]
    frame = {"session_id": session, "sequence": 1, "positions": positions}
    arm = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0]
    zeros = [0.0] * 6
    try:
        reply = request("target", {**frame, "velocities": {"joint1": 0.01},
                                   "tff": {"joint1": 0.25, "finger_left": 0.1},
                                   "gravity_ff": {"joint1": 0.15},
                                   "friction_ff": {"joint1": -0.1, "finger_left": 0.0}})
        node.applied(reply["dispatch_id"])
        node.check_tff(arm, 0.1, "HTTP must apply both task feedforward groups")
        node.check_compensation("HTTP must apply independent compensation overrides",
                                gravity={0: 0.15}, friction={0: -0.1})
        mixed_sources = {"gravity": {**internal, "joint1": "external"},
                         "friction": {**internal, "joint1": "external", "finger_left": "external"}}
        node.until(lambda: request("state")["compensation_sources"] == mixed_sources,
                   "web must report mixed sources and external zero on the gripper")
        assert request("state")["status"]["last_forwarded"]["velocity"]["joint1"] == 0.01
        reply = request("target", {**frame, "sequence": 2})
        node.applied(reply["dispatch_id"])
        node.check_tff(zeros, 0.0, "HTTP position-only target must clear both groups")
        node.check_compensation("HTTP position-only target must clear friction override", friction={0: 0.0})
        node.until(lambda: request("state")["compensation_sources"] == internal_sources,
                   "web must report internal sources on the replacement frame")
        reply = request("delta_target", {"session_id": session, "sequence": 3, "deltas": {"joint1": 0.004}})
        node.applied(reply["dispatch_id"])
        assert reply["positions"] == node.status["last_forwarded"]["position"]
    finally:
        request("release", {"session_id": session})
    print("PASS HTTP velocity, task feedforward, mixed compensation sources and internal fallback")
    print("PASS HTTP delta target returns the resolved absolute positions")

    mover = Path(__file__).with_name("dm_mit_move.py")
    with subprocess.Popen([
        "python3", str(mover), "--set", "joint1=0.055,finger_left=0.015",
        "--vel", "joint1=0.01", "--tff", "joint1=0.25,finger_left=0.1",
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as process:
        try:
            node.check_tff(arm, 0.1, "CLI must apply both task feedforward groups")
            node.until(lambda: process.poll() is not None, "CLI must complete")
        finally:
            if process.poll() is None:
                process.terminate()
        output, _ = process.communicate(timeout=5)
        assert process.returncode == 0, output
    assert node.status["last_forwarded"]["velocity"]["joint1"] == 0.01
    print("PASS CLI position, velocity and task feedforward")


def test(node, launch_log, http_port=None):
    external = TargetClient(node, "external")
    policy = TargetClient(node, "policy")
    authority = node.create_client(SetAuthority, f"{ARBITER}/set_authority")
    node.until(lambda: node.status.get("feedback_ready") and len(node.feedback) == 8
               and node.safety is not None and len(node.holds) == 2,
               "MIT controllers, live feedback and safety must be present")
    assert len(node.status["joints"]) == 7 and "controllers" not in node.status
    assert node.status["source"] is None and node.status["authority"] is None
    for controller in (ARM, GRIPPER):
        pubs = node.get_publishers_info_by_topic(f"/{controller}/authorized_target")
        assert [info.node_name for info in pubs] == ["b601_target_arbiter"], pubs
        inputs = node.get_publishers_info_by_topic(f"/{controller}/joint_target")
        assert [info.node_name for info in inputs] == ["b601_target_bridge"], inputs
        subscribers = node.get_subscriptions_info_by_topic(f"/{controller}/joint_target")
        assert [info.node_name for info in subscribers] == [controller], subscribers
    print("PASS idle startup and one publisher per controller")
    test_delta_target(node, external)
    test_tff(node, external)
    test_compensation(node, external)

    first_frame = TargetFrame(session_id="no-session", sequence=1,
                              target=JointState(name=["joint1"], position=[-0.1]))
    assert not external.call(external.sender, SetTarget.Request(frame=first_frame), TIMEOUT).accepted
    external.select(TIMEOUT)
    try:
        busy = policy.call(policy.selector, SelectSource.Request(source="policy"), TIMEOUT)
        assert not busy.accepted and "external" in busy.message
        partial = external.frame(JointState(name=["finger_left"], position=[0.02]))
        rejected = external.call(external.sender, SetTarget.Request(frame=partial), TIMEOUT)
        assert not rejected.accepted and "all seven" in rejected.message

        positions = {name: node.feedback[name] for name in node.status["joints"]}
        goal = {"joint1": 0.12, "finger_left": positions["finger_left"] + 0.0003}
        positions.update(goal)
        old = external.frame(JointState(name=list(positions), position=list(positions.values())))
        response = external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT)
        assert response.accepted, response.message
        node.applied(response.dispatch_id)
        node.reaches({"joint1": goal["joint1"]})
        node.reaches({"finger_left": goal["finger_left"]}, tolerance=0.00015)
        assert not external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT).accepted
        assert node.status["last_forwarded"]["position"] == positions
        print("PASS exclusive owner, full seven-joint wire target, replay rejection and both routes")

        reply = external.call(authority, SetAuthority.Request(source="policy"), TIMEOUT)
        assert reply.accepted
        node.until(lambda: node.status.get("authority") == "policy" and node.status.get("source") is None,
                   "designating a new input must revoke the prior holder")
        denied = external.call(external.selector, SelectSource.Request(source="external"), TIMEOUT)
        assert not denied.accepted and "designated to policy" in denied.message
        policy.select(TIMEOUT)
        assert external.call(authority, SetAuthority.Request(source="policy"), TIMEOUT).accepted
        old.sequence += 1000
        old.target.position[0] = -0.14
        external.publisher.publish(old)
        assert not external.call(external.sender, SetTarget.Request(frame=old), TIMEOUT).accepted
        node.until(lambda: node.status["source"] == "policy", "policy is the designated owner")
        assert node.status["last_forwarded"]["dispatch_id"] == str(response.dispatch_id)
        new_goal = JointState(name=["joint1", "joint4"], position=[0.02, 0.03], velocity=[0.0, 0.0])
        policy.publish(new_goal)
        node.until(lambda: node.status["last_forwarded"]["source"] == "policy",
                   "policy stream must reach the arbiter")
        node.applied(node.status["last_forwarded"]["dispatch_id"])
        node.reaches(dict(zip(new_goal.name, new_goal.position)))
        assert list(node.status["last_forwarded"]["position"]) == node.status["joints"]
        assert node.safety == DiagnosticStatus.OK, node.safety

        policy.release(TIMEOUT)
        denied = external.call(external.selector, SelectSource.Request(source="external"), TIMEOUT)
        assert not denied.accepted and "designated to policy" in denied.message
        policy.select(TIMEOUT)
        assert external.call(authority, SetAuthority.Request(source=""), TIMEOUT).accepted
        node.until(lambda: node.status["source"] is None and node.status["authority"] is None,
                   "operator release must clear the designation and the active holder")
        print("PASS operator handover, previous-session rejection, policy stream and release")
    finally:
        external.close(TIMEOUT)
        policy.close(TIMEOUT)

    if http_port:
        test_http_and_cli(node, http_port)

    reserve_disable = node.create_client(SetBool, f"{ARBITER}/reserve_disable")
    assert not external.call(reserve_disable, SetBool.Request(data=True), TIMEOUT).success

    recenter = node.create_client(Trigger, "/b601_mit_recenter/recenter")
    assert recenter.wait_for_service(timeout_sec=TIMEOUT)
    result = recenter.call_async(Trigger.Request())
    node.until(lambda: node.status.get("source") == "recenter", "recenter must acquire ownership")
    contention = policy.call(
        policy.selector, SelectSource.Request(source="policy"), TIMEOUT)
    assert not contention.accepted and "recenter" in contention.message
    node.until(result.done, "recenter action must finish", timeout=45)
    assert result.result().success, result.result().message
    node.until(lambda: node.status["source"] is None, "recenter must release ownership")
    assert node.status["last_forwarded"]["source"] == "recenter"
    project = load_project_config()
    for joint, value in project["recenter"]["joints"].items():
        if joint in node.status["joints"]:
            assert abs(node.feedback[joint] - value) < 0.03, (joint, node.feedback[joint], value)
    assert node.safety in (DiagnosticStatus.OK, DiagnosticStatus.WARN), node.safety
    node.until(lambda: node.status.get("disable_ready"), "successful recenter must unlock disable")
    assert external.call(reserve_disable, SetBool.Request(data=True), TIMEOUT).success
    try:
        assert not external.call(policy.selector, SelectSource.Request(source="policy"), TIMEOUT).accepted
        assert not external.call(authority, SetAuthority.Request(source="policy"), TIMEOUT).accepted
    finally:
        assert external.call(reserve_disable, SetBool.Request(data=False), TIMEOUT).success
    print("PASS MuJoCo recenter and shutdown gate block competing control sources")

    revoke = node.create_client(RevokeSource, f"{ARBITER}/revoke_source")
    abandoned = external.call(external.selector, SelectSource.Request(source="abandoned"), TIMEOUT)
    assert abandoned.accepted
    node.until(lambda: node.status.get("source") == "abandoned", "operator must see orphaned ownership")
    public_id = node.status["session"]
    wrong = RevokeSource.Request(session="old-selection")
    assert not external.call(revoke, wrong, TIMEOUT).accepted
    current = RevokeSource.Request(session=public_id)
    assert external.call(revoke, current, TIMEOUT).accepted
    print("PASS operator revocation matches only the selected session")

    lost = TargetClient(node, "lost_source", auto_renew=False)
    lost.select(TIMEOUT)
    lost.renew(TIMEOUT)
    started = time.monotonic()
    first = lost.set_joint("joint1", 0.8, TIMEOUT)
    node.applied(first)
    previous = lost.frame(JointState(name=["joint1"], position=[0.7]))
    node.until(lambda: node.status.get("source") is None and node.watchdog_hold and node.holds.get(GRIPPER),
               "lost source must release authority and put the MIT controller into measured hold", timeout=2)
    source_hold_time = time.monotonic() - started
    assert 0.45 <= source_hold_time <= 0.8, source_hold_time
    hold_position = node.feedback["joint1"]
    assert hold_position < 0.4, hold_position
    assert not external.call(external.sender, SetTarget.Request(frame=previous), TIMEOUT).accepted
    node.until(lambda: node.status["last_applied_id"] is None, "hold clears prior receipt")
    print(f"PASS source disconnect, hold after {source_hold_time:.3f}s, prior frame rejected")

    policy.select(TIMEOUT)
    try:
        dispatch = policy.set_joint("joint1", 0.8, TIMEOUT)
        node.applied(dispatch)
        node.until(lambda: not node.watchdog_hold, "fresh selection must leave measured hold")
        log = launch_log.read_text()
        match = re.search(r"target_arbiter-\d+\]: process started with pid \[([0-9]+)\]", log)
        assert match, "arbiter PID not present in this MuJoCo launch"
        arbiter_pid = int(match.group(1))
        started = time.monotonic()
        os.kill(arbiter_pid, signal.SIGSTOP)
        try:
            node.until(lambda: node.watchdog_hold and node.holds.get(GRIPPER),
                       "the bridge must forward a measured hold to both controllers",
                       timeout=2)
            arbiter_hold_time = time.monotonic() - started
            assert 0.35 <= arbiter_hold_time <= 0.8, arbiter_hold_time
            assert node.feedback["joint1"] < 0.7
        finally:
            os.kill(arbiter_pid, signal.SIGCONT)

        old_frame = policy.frame(JointState(name=["joint1"], position=[0.8]))
        assert not external.call(external.sender, SetTarget.Request(frame=old_frame), TIMEOUT).accepted
        try:
            policy.release(TIMEOUT)
        except RuntimeError as error:
            assert "released session" in str(error)
        node.until(lambda: node.status.get("source") is None,
                   "resumed arbiter must leave the old owner revoked")
        policy.select(TIMEOUT)
        recovered = policy.set_joint("joint1", 0.07, TIMEOUT)
        node.applied(recovered)
        node.reaches({"joint1": 0.07})
        node.until(lambda: not node.watchdog_hold, "fresh selection must leave measured hold")
        print(f"PASS arbiter interrupt, hold after {arbiter_hold_time:.3f}s, and new explicit selection")
    finally:
        policy.close(TIMEOUT)
    print(json.dumps({"feedback": node.feedback, "status": node.status}, ensure_ascii=False))


def test_physical_recenter(node, http_port=None):
    project = load_project_config()
    arm = project["robot"]["arm_joints"]
    home = {name: project["recenter"]["joints"][name] for name in arm}
    recenter = node.create_client(Trigger, "/b601_mit_recenter/recenter")
    reserve = node.create_client(SetBool, f"{ARBITER}/reserve_disable")
    target = TargetClient(node, "external")
    node.until(lambda: node.status.get("feedback_ready") and recenter.service_is_ready(),
               "physical MuJoCo must provide feedback and recenter service")
    for goal in (
        dict(zip(arm, (-0.28, -0.18, -0.38, 0.24, 0.23, 0.0))),
        dict(zip(arm, (0.0, -0.03, -0.16, -0.18, 0.0, 0.0))),
    ):
        target.select(TIMEOUT)
        try:
            node.applied(target.set_joints(goal, TIMEOUT))
            node.reaches(goal)
        finally:
            target.release(TIMEOUT)
        assert not node.status["disable_ready"], node.status
        start = {name: node.feedback[name] for name in arm}
        result = recenter.call_async(Trigger.Request())
        node.until(result.done, "physical MuJoCo recenter must finish", timeout=45)
        assert result.result().success, (start, result.result().message)
        node.reaches(home)
        node.until(lambda: node.status.get("disable_ready"),
                   "physical MuJoCo parking feedback must unlock disable")
        assert target.call(reserve, SetBool.Request(data=True), TIMEOUT).success
        assert target.call(reserve, SetBool.Request(data=False), TIMEOUT).success
        print(f"PASS physical MuJoCo recenter and disable gate from {start}", flush=True)
    if http_port:
        # Exercise the generated physical controller configuration from a joint goal
        # well inside the limits after the existing recenter checks.
        goal = dict(zip(arm, (-0.28, -0.18, -0.38, 0.24, 0.23, 0.0)))
        goal[project["robot"]["gripper_joint"]] = 0.015
        target.select(TIMEOUT)
        try:
            node.applied(target.set_joints(goal, TIMEOUT))
            node.reaches(goal)
        finally:
            target.release(TIMEOUT)
        test_tff(node, target, positions=goal)
        test_compensation(node, target, positions=goal)
        test_http_and_cli(node, http_port, positions=goal)


def test_stiffness_switching(node, enabled, port=None):
    node.until(lambda: node.status.get("feedback_ready") and ARM in node.controller_state,
               "MIT controller and feedback must be ready")
    client = TargetClient(node, "external")
    joint = "joint1"
    project = load_project_config()
    profile = project["controllers"]["mit"]["profiles"]
    high = profile["arm_pure_pd"]["per_joint"][joint]["kp_move"]
    low = high * profile["arm_gravity_pd"]["hold_gain_ratios"]["kp"]
    assert node.controller_state[ARM][12] == high
    client.select(TIMEOUT)
    try:
        position = node.feedback[joint] + 0.04
        node.applied(client.set_joint(joint, position, TIMEOUT))
        node.until(lambda: node.controller_state[ARM][48] == 1.0
                   and node.controller_state[ARM][12] == (low if enabled else high),
                   "settled MIT target must use the selected stiffness")
        if port:
            def web_reports_setting():
                with urlopen(f"http://127.0.0.1:{port}/api/state", timeout=3) as response:
                    snapshot = json.load(response)
                return (snapshot["backend"]["stiffness_switching"] == str(enabled).lower()
                        and snapshot["stiffness"] == ("low" if enabled else "high"))

            node.until(web_reports_setting, "web must report the selected switch and current stiffness")
        if enabled:
            node.applied(client.set_joint(joint, position + 0.04, TIMEOUT))
            node.until(lambda: node.controller_state[ARM][12] == high,
                       "new MIT target must switch back to high stiffness")
    finally:
        client.release(TIMEOUT)
    print(f"PASS MIT startup switch {enabled}: joint1 Kp {high} → {low if enabled else high}")


def start_fixture(output, web, physical=False, *, port=8765, stiffness_switching="auto"):
    if physical:
        with (output / "launch.log").open("w") as stream:
            return subprocess.Popen([
                "ros2", "launch", "b601_bringup", "bringup.launch.py",
                "hardware:=mujoco", "mujoco_scene:=physical",
                "dm_command_mode:=mit_impedance", "use_rviz:=false",
                f"mit_stiffness_switching:={stiffness_switching}",
                f"use_target_web:={str(web).lower()}", f"target_web_port:={port}",
            ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)

    root = Path(__file__).resolve().parent.parent
    source = root / "ros2_ws/src"

    initial = {
        "initial_positions": {"joint1": 0.0, "joint2": -0.3, "joint3": -0.6,
                              "joint4": 0.0, "joint5": 0.0, "joint6": 0.0,
                              "finger_left": 0.015, "finger_right": -0.015},
    }
    initial_path = output / "initial.yaml"
    initial_path.write_text(yaml.safe_dump(initial))
    config = yaml.safe_load((source / "b601_bringup/config/b601_mit_controllers.yaml").read_text())
    # The committed DM gains overdrive this MuJoCo finger model. Tame only this test fixture.
    finger = config[GRIPPER]["ros__parameters"]
    for key, value in (("kp_hold", 500.0), ("kp_move", 500.0),
                       ("kd_hold", 5.0), ("kd_move", 5.0)):
        finger[key] = [value]
    controllers = output / "controllers.yaml"
    controllers.write_text(yaml.safe_dump(config))
    with (output / "launch.log").open("w") as stream:
        return subprocess.Popen([
            "ros2", "launch", "b601_bringup", "bringup.launch.py",
            "hardware:=mujoco", "dm_command_mode:=mit_impedance", "use_rviz:=false",
            f"mit_stiffness_switching:={stiffness_switching}",
            "mit_recenter_check_collisions:=false",
            f"initial_positions_file:={initial_path}", f"controllers_file:={controllers}",
            f"use_target_web:={str(web).lower()}", f"target_web_port:={port}",
        ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--web", action="store_true", help="启动同一 MuJoCo 夹具并预览本地网页")
    mode.add_argument("--physical", action="store_true", help="验证内置物理场景的归中及失能门控")
    mode.add_argument("--stiffness-switching", choices=("true", "false"),
                      help="验证显式软硬刚度启动选项及实际增益")
    parser.add_argument("--http-port", type=int, help="在指定本地端口启动网页并验证 HTTP 和 CLI")
    args = parser.parse_args()
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    domain = os.environ.get("B601_TEST_ROS_DOMAIN_ID", "227")
    if not domain.isdecimal() or not 0 <= int(domain) < 233:
        raise SystemExit("B601_TEST_ROS_DOMAIN_ID must be a ROS domain number.")
    os.environ["ROS_DOMAIN_ID"] = domain
    output = Path(__file__).resolve().parent.parent / ".verify" / f"target-arbitration-{domain}"
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(output / "roslog")
    rclpy.init()
    node = TestNode()
    process = None
    try:
        wait_until = time.monotonic() + 1.0
        while time.monotonic() < wait_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.count_publishers("/joint_states") or "controller_manager" in node.get_node_names():
            raise RuntimeError("Test domain already contains a robot, choose B601_TEST_ROS_DOMAIN_ID.")
        process = start_fixture(
            output, args.web or args.http_port is not None, args.physical,
            port=args.http_port or 8765, stiffness_switching=args.stiffness_switching or "auto",
        )
        if args.web:
            node.until(lambda: node.status.get("feedback_ready"),
                       "MuJoCo preview must become ready")
            print(f"MuJoCo：访问 http://127.0.0.1:{args.http_port or 8765}/ ，Ctrl-C 停止。", flush=True)
            while rclpy.ok() and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() not in (None, 0):
                raise RuntimeError(f"MuJoCo launch failed, see {output / 'launch.log'}")
        elif args.physical:
            test_physical_recenter(node, args.http_port)
        elif args.stiffness_switching:
            test_stiffness_switching(node, args.stiffness_switching == "true", args.http_port)
        else:
            test(node, output / "launch.log", args.http_port)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not args.web:
            raise
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
