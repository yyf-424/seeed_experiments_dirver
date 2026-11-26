"""Local HTTP and ROS bridge for the B601 target authorization console."""
import json
import os
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from math import dist
from pathlib import Path
from tempfile import mkdtemp
from threading import Event, Lock, Thread
from urllib.parse import parse_qs, urlsplit
from uuid import uuid4
from xml.etree.ElementTree import ParseError

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from b601_description.project_config import load_project_config
from b601_target_interfaces.msg import CompensationSources, LoadEstimates, MitTargetFrame, TargetFrame
from b601_target_interfaces.srv import (
    ReleaseSource, RenewSource, RevokeSource, SelectSource, SetAuthority, SetMitTarget, SetTarget,
)

from .load_api import LOAD_SITES, LOAD_TOPIC, load_estimates, load_snapshot, parse_load_positions
from .mit_recenter import RECENTER_SERVICE
from .mit_recenter_client import recenter_result_timeout
from .mit_target import make_mit_target, measured_positions, positions_from_deltas
from .gravity_provider import GravityProvider
from .target_arbiter import ARBITER
from .tcp_motion import BASE, MIN_DISTANCE_MM, TCP, TcpPlanner, primitive, xyz
from .web_backend import WebBackend
from .web_recording import WebRecording
from .web_robot_model import build_model, find_vendor_model


class TargetWeb(Node):
    def __init__(self):
        super().__init__("b601_target_web")
        self.port = self.declare_parameter("port", 8765).value
        config_path = self.declare_parameter("project_config_file", "").value
        config = load_project_config(config_path or None)
        attached = self.declare_parameter("attached_backend", "").value
        attached_scene = self.declare_parameter("attached_mujoco_scene", "").value
        attached_stiffness = self.declare_parameter("attached_stiffness_switching", "").value
        default_stiffness = self.declare_parameter("default_stiffness_switching", "false").value
        directory = os.environ.get("B601_WEB_RECORD_DIR")
        if directory:
            recording_dir = Path(directory).expanduser().resolve()
        else:
            root = Path(os.environ.get("B601_WEB_LOG_ROOT", ".verify/web-recordings")).expanduser().resolve()
            root.mkdir(parents=True, exist_ok=True)
            recording_dir = Path(mkdtemp(prefix=f"{datetime.now():%Y%m%d-%H%M%S}-", dir=root))
        self.backend = WebBackend(
            self, config, config_path, attached, attached_scene, attached_stiffness,
            default_stiffness, recording_dir=recording_dir)
        self.feedback_timeout = config["safety"]["feedback_timeout_sec"]
        self.heartbeat_period_ms = round(
            config["tools"]["target_arbitration"]["authority_timeout_sec"] * 200)
        self.limits = {joint: {"min": bounds["min_pos"], "max": bounds["max_pos"]}
                       for joint, bounds in config["joints"].items()}
        robot = config["robot"]
        self.joints = (*robot["arm_joints"], robot["gripper_joint"])
        self.gravity = GravityProvider(self, self.joints, self.feedback_timeout)
        self.load_api = self.declare_parameter("load_api", "").value
        load_positions = self.declare_parameter("load_positions", "gripper").value
        self.load_positions = parse_load_positions(load_positions) if self.load_api else ()
        self.load_sample = None
        self.tcp = TcpPlanner(self, config, self.call_planner)
        self.tcp_lock = Lock()
        self.tcp_reference = None
        self.tcp_settle = config["tools"]["dm_mit_move"]["settle"]
        self.tcp_poll = config["validation"]["mit_five_poses"]["callback_poll_sec"]
        self.tcp_ack_timeout = config["validation"]["mit_five_poses"]["state_check_timeout_sec"]
        self.tcp_safety = None
        self.compensation_samples = {}
        self.status = None
        self.feedback_sample = ({}, None)
        self.stiffness_sample = None
        self.sim_stiffness_sample = None
        self.robot_model = None
        self.preview_model = None
        self.model_revision = 0
        self.vendor_model = find_vendor_model()
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, description_qos)
        self.preview_pose = None
        if self.declare_parameter("preview_model", False).value:
            self.preview_pose = {joint: bounds["startup"] for joint, bounds in config["joints"].items()}
            self.create_subscription(
                String, "/b601_preview/robot_description",
                lambda message: self.on_description(message, preview=True), description_qos)
        self.create_subscription(
            String, f"{ARBITER}/status", self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(
            DiagnosticStatus, config["safety"]["status_topic"], self.on_safety, description_qos)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.on_stiffness_feedback, qos_profile_sensor_data)
        self.create_subscription(
            Bool, "/b601_mit_arm/soft_stiffness", self.on_sim_stiffness, qos_profile_sensor_data)
        if self.load_api:
            self.create_subscription(LoadEstimates, LOAD_TOPIC, self.on_load, qos_profile_sensor_data)
        for controller in config["controllers"]["mit"]["instances"]:
            self.create_subscription(
                CompensationSources, f"/{controller}/compensation_sources",
                lambda message, name=controller: self.on_compensation_sources(name, message),
                qos_profile_sensor_data)
        self.select = self.create_client(SelectSource, f"{ARBITER}/select_source")
        self.authority = self.create_client(SetAuthority, f"{ARBITER}/set_authority")
        self.renew = self.create_client(RenewSource, f"{ARBITER}/renew_source")
        self.release = self.create_client(ReleaseSource, f"{ARBITER}/release_source")
        self.revoke = self.create_client(RevokeSource, f"{ARBITER}/revoke_source")
        self.set_target = self.create_client(SetTarget, f"{ARBITER}/set_target")
        self.set_mit_target = self.create_client(SetMitTarget, f"{ARBITER}/set_mit_target")
        self.recenter = self.create_client(Trigger, RECENTER_SERVICE)
        self.release_for_recenter = self.create_client(ReleaseSource, f"{ARBITER}/release_web_for_recenter")
        self.recenter_timeout = recenter_result_timeout(config)
        self.recording = WebRecording(recording_dir, self.get_logger().error)
        gravity = self.gravity.snapshot()
        self.recording.add("console_started", port=self.port, domain_id=self.context.get_domain_id(),
                           attached_backend=attached, frame=BASE, tcp=TCP,
                           gravity_source=gravity["requested"], gravity_api=gravity["api"],
                           load_api=self.load_api, load_positions=self.load_positions)

    def on_status(self, message):
        previous = self.status or {}
        self.status = current = json.loads(message.data)
        target = current.get("last_forwarded")
        if target and target != previous.get("last_forwarded"):
            self.recording.add("target_forwarded", **target)
        applied = current.get("last_applied_id")
        if applied and applied != previous.get("last_applied_id"):
            self.recording.add("target_applied", dispatch_id=applied)

    def on_feedback(self, message):
        positions = dict(zip(message.name, message.position))
        self.feedback_sample = (positions, time.monotonic())
        self.gravity.feedback = self.feedback_sample
        stamp = message.header.stamp
        self.recording.add(
            "joint_feedback", backend=self.backend.attached or self.backend.backend,
            ros_stamp_ns=stamp.sec * 10**9 + stamp.nanosec, positions=positions,
            velocities=dict(zip(message.name, message.velocity)),
            efforts=dict(zip(message.name, message.effort)),
        )

    def on_safety(self, message):
        self.tcp_safety = message.level

    def on_stiffness_feedback(self, message):
        joint = next((values for name, values in zip(message.joint_names, message.interface_values)
                      if name == "joint1"), None)
        flag = dict(zip(joint.interface_names, joint.values)).get("low_stiffness") if joint else None
        self.stiffness_sample = (flag, time.monotonic()) if flag in (0.0, 1.0) else None

    def on_sim_stiffness(self, message):
        self.sim_stiffness_sample = (message.data, time.monotonic())

    def on_load(self, message):
        reading = load_snapshot(message)
        self.load_sample = (time.monotonic(), reading)
        self.recording.add("load_estimate", **reading)

    def load_state(self, now):
        if not self.load_api:
            return None
        sample = self.load_sample
        if sample is not None and now - sample[0] <= self.feedback_timeout:
            data = sample[1]
        else:
            sites = {name: LOAD_SITES[name] for name in self.load_positions}
            data = load_snapshot(load_estimates(sites, {}, "waiting_node"))
        estimates = data["estimates"]
        return {
            **data, "api": self.load_api, "positions": list(self.load_positions),
            "estimates": {name: estimates[name] for name in self.load_positions if name in estimates},
        }

    def on_compensation_sources(self, controller, message):
        if len(message.joints) == len(message.gravity_external) == len(message.friction_external):
            self.compensation_samples[controller] = (time.monotonic(), message)
            self.recording.add("compensation_sources", controller=controller, joints=list(message.joints),
                               gravity_enabled=message.gravity_enabled,
                               gravity_external=list(message.gravity_external),
                               friction_external=list(message.friction_external))

    def compensation_sources(self, now):
        gravity, friction = {}, {}
        for received_at, message in self.compensation_samples.copy().values():
            if now - received_at <= self.feedback_timeout:
                for joint, gravity_external, friction_external in zip(
                        message.joints, message.gravity_external, message.friction_external):
                    gravity[joint] = ("external" if gravity_external else "internal") \
                        if message.gravity_enabled else "disabled"
                    friction[joint] = "external" if friction_external else "internal"
        if gravity.keys() != set(self.joints):
            return None
        return {"gravity": gravity, "friction": friction}

    def on_description(self, message, *, preview=False):
        mesh_prefix = "/robot-mesh/preview" if preview else "/robot-mesh"
        try:
            model, assets = build_model(message.data, self.vendor_model, mesh_prefix)
        except (KeyError, ValueError, ParseError) as error:
            self.get_logger().warning(f"网页机械臂模型读取失败：{error}")
            return
        self.model_revision += 1
        model["revision"] = self.model_revision
        if preview:
            self.preview_model = (model, assets)
        else:
            self.robot_model = (model, assets)

    def snapshot(self):
        feedback, feedback_at = self.feedback_sample
        now = time.monotonic()
        age = now - feedback_at if feedback_at is not None else None
        status = self.status if self.select.service_is_ready() else None
        fresh = age is not None and age <= self.feedback_timeout
        backend = self.backend.snapshot(
            fresh and bool(status and status["feedback_ready"]),
            fresh and bool(status and status.get("disable_ready")))
        preview = self.preview_model
        show_preview = preview is not None and backend["phase"] == "idle"
        model = preview if show_preview else self.robot_model or preview
        sample = self.sim_stiffness_sample if backend["name"] == "mujoco" else self.stiffness_sample
        stiffness = None
        connected = backend["phase"] == "connected" and self.backend.active
        if connected and sample is not None and now - sample[1] <= self.feedback_timeout:
            stiffness = "low" if sample[0] == 1.0 else "high"
        gravity = self.gravity.snapshot()
        recording_path = str(self.recording.path)
        return {
            "status": status,
            "backend": backend,
            "stiffness": stiffness,
            "gravity_source": gravity["requested"],
            "gravity_api": gravity["api"] if gravity["requested"] == "external" else "",
            "gravity_selection": gravity,
            "load": self.load_state(now),
            "recording_path": recording_path,
            "compensation_sources": self.compensation_sources(now) if connected else None,
            "recenter_ready": self.recenter.service_is_ready(),
            "tcp_jog": {
                "frame": BASE,
                "min_distance_mm": MIN_DISTANCE_MM,
                "planner_ready": self.tcp.ready(),
                "recording_path": recording_path,
                "ready": connected and fresh and self.tcp_ready(),
            },
            "feedback": {} if show_preview else feedback,
            "preview_enabled": self.preview_pose is not None,
            "preview_pose": self.preview_pose if show_preview else None,
            "model_revision": model[0]["revision"] if model else None,
            "limits": self.limits,
            "heartbeat_period_ms": self.heartbeat_period_ms,
            "feedback_age_ms": round(age * 1000) if age is not None else None,
        }

    def request(self, path, data, operation_id=None, *, tcp_waypoint=False):
        if path == "/api/gravity_source":
            self.gravity.select_source(data["source"])
            return HTTPStatus.ACCEPTED, {"accepted": True, "message": "已请求切换重力补偿"}
        if path == "/api/connect":
            code, result = self.backend.connect(
                data["backend"], data.get("mujoco_scene", "free"), data.get("stiffness_switching"))
            if code == HTTPStatus.ACCEPTED:
                self.recording.add(
                    "backend_started", operation_id=operation_id, backend=data["backend"],
                    dm_log_csv=str(self.backend.dm_log_csv) if self.backend.dm_log_csv else None,
                )
            return code, result
        if path == "/api/disable":
            return self.backend.disable(self.call_service)
        if path == "/api/tcp_jog":
            return self.request_tcp_jog(data, operation_id)
        if path == "/api/tcp_view":
            return HTTPStatus.OK, {"accepted": True}
        if path == "/api/recenter":
            return self.request_recenter(data.get("session_id"))
        if path == "/api/select":
            request = SelectSource.Request(source="web")
            client = self.select
        elif path == "/api/authority":
            request = SetAuthority.Request(source=data["source"])
            client = self.authority
        elif path == "/api/heartbeat":
            request = RenewSource.Request(session_id=data["session_id"])
            client = self.renew
        elif path == "/api/release":
            request = ReleaseSource.Request(session_id=data["session_id"])
            client = self.release
        elif path == "/api/revoke":
            request = RevokeSource.Request(session=data["session"])
            client = self.revoke
        elif path in ("/api/target", "/api/delta_target"):
            feedback = self.feedback_sample
            if path == "/api/delta_target":
                try:
                    positions = positions_from_deltas(
                        self.joints, data["deltas"], feedback, self.feedback_timeout)
                except RuntimeError as error:
                    return HTTPStatus.SERVICE_UNAVAILABLE, {"message": str(error)}
            else:
                positions = data["positions"]
                if not isinstance(positions, dict):
                    raise ValueError("positions must be an object keyed by joint name")
            target = JointState(name=list(positions), position=[float(v) for v in positions.values()])
            for key, field in (("velocities", "velocity"), ("tff", "effort")):
                values = data.get(key, {}) if path == "/api/target" else {}
                if not isinstance(values, dict) or not values.keys() <= positions.keys():
                    raise ValueError(f"{key} must be an object keyed by joint name")
                if values:
                    setattr(target, field, [float(values.get(name, 0.0)) for name in target.name])
            gravity = data.get("gravity_ff") if path == "/api/target" else None
            friction = data.get("friction_ff") if path == "/api/target" else None
            command = make_mit_target(target, gravity, friction)
            frame = {"session_id": data["session_id"], "sequence": data["sequence"]}
            if command.gravity_ff or command.friction_ff:
                request = SetMitTarget.Request(frame=MitTargetFrame(command=command, **frame))
                client = self.set_mit_target
            else:
                request = SetTarget.Request(frame=TargetFrame(target=target, **frame))
                client = self.set_target
        else:
            raise KeyError("unknown endpoint")

        status, reply = self.call_service(client, request)
        if status != HTTPStatus.OK:
            return status, reply
        result = {"accepted": reply.accepted, "message": reply.message}
        if isinstance(reply, SelectSource.Response) and reply.accepted:
            result["session_id"] = reply.session_id
            result["session"] = reply.session
        if isinstance(reply, (SetTarget.Response, SetMitTarget.Response)) and reply.accepted:
            result["dispatch_id"] = str(reply.dispatch_id)
            if not tcp_waypoint:
                self.tcp_reference = None
        if path == "/api/delta_target" and reply.accepted:
            result["positions"] = positions
        return HTTPStatus.OK if reply.accepted else HTTPStatus.CONFLICT, result

    def tcp_ready(self):
        return (self.tcp.ready()
                and self.tcp_safety in (DiagnosticStatus.OK, DiagnosticStatus.WARN))

    def call_planner(self, client, request, timeout):
        code, result = self.call_service(client, request, timeout)
        if code != HTTPStatus.OK:
            raise RuntimeError(f"{client.srv_name}：{result['message']}")
        return result

    def tcp_feedback(self, public_id):
        state = self.status or {}
        if state.get("source") != "web" or state.get("session") != public_id:
            raise RuntimeError("网页整机控制权已失效")
        if not self.backend.active or not self.tcp_ready():
            raise RuntimeError("TCP 点动需要就绪的 MoveIt、MIT 控制器与整机安全状态")
        sample = measured_positions(self.joints, self.feedback_sample, self.feedback_timeout)
        if sample is None:
            raise RuntimeError("缺少完整且新鲜的关节反馈")
        return sample

    def tcp_wait(self, public_id, seconds, dispatch=None):
        deadline = time.monotonic() + seconds
        while True:
            positions = self.tcp_feedback(public_id)
            if dispatch and self.status.get("last_applied_id") == dispatch:
                return positions
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if dispatch:
                    raise RuntimeError(f"两组 MIT 控制器未共同采纳目标 {dispatch}")
                return positions
            time.sleep(min(remaining, self.tcp_poll))

    def request_tcp_jog(self, data, operation_id):
        move = primitive(data["direction"], data["distance_mm"])
        public_id = data["session"]
        session_id = data["session_id"]
        sequence = data["sequence"]
        if type(sequence) is not int or sequence < 1:
            raise ValueError("目标序号必须是正整数")
        if not self.tcp_lock.acquire(blocking=False):
            return HTTPStatus.CONFLICT, {"message": "上一条 TCP 点动仍在执行"}

        try:
            initial = self.tcp_feedback(public_id)
            code, answer = self.call_service(self.renew, RenewSource.Request(session_id=session_id))
            if code != HTTPStatus.OK:
                return code, answer
            if not answer.accepted:
                return HTTPStatus.CONFLICT, {"message": answer.message}

            forwarded = self.status.get("last_forwarded") or {}
            previous = self.tcp_reference
            use_previous = previous is not None and previous[:2] == (public_id, forwarded.get("dispatch_id"))
            reference = previous[2] if use_previous else None
            origin, (segment,) = self.tcp.plan([move], initial, reference=reference)
            origin_xyz = xyz(origin)
            reference_xyz_mm = [value * 1000 for value in xyz(reference if reference is not None else origin)]
            reference_source = "previous_tcp_goal" if use_previous else "feedback"
            goal_xyz_mm = [value * 1000 for value in segment.goal]
            self.recording.add(
                "tcp_plan", operation_id=operation_id, frame=BASE, tcp=TCP,
                direction=move[0], distance_mm=move[1],
                origin_xyz_mm=[value * 1000 for value in origin_xyz], goal_xyz_mm=goal_xyz_mm,
                reference_xyz_mm=reference_xyz_mm, reference_source=reference_source,
            )
            dispatch = None
            for delay, positions in self.tcp.targets(segment, initial[self.tcp.gripper]):
                self.tcp_wait(public_id, delay)
                code, answer = self.request("/api/target", {
                    "session_id": session_id, "sequence": sequence, "positions": positions,
                }, tcp_waypoint=True)
                if code != HTTPStatus.OK:
                    return code, {**answer, "sequence": sequence}
                if dispatch is None:
                    self.tcp_reference = None
                dispatch = answer["dispatch_id"]
                self.recording.add("tcp_dispatch", operation_id=operation_id, dispatch_id=dispatch,
                                   sequence=sequence, positions=positions)
                sequence += 1

            self.tcp_wait(public_id, self.tcp_ack_timeout, dispatch)
            self.tcp_reference = (public_id, dispatch, segment.goal_pose)
            feedback = self.tcp_wait(public_id, self.tcp_settle)
            actual = xyz(self.tcp.fk_pose(feedback))
            terminal_error_mm = dist(actual, segment.goal) * 1000
            return HTTPStatus.OK, {
                "accepted": True, "sequence": sequence - 1, "dispatch_id": dispatch,
                "frame": BASE, "tcp": TCP, "direction": move[0], "distance_mm": move[1],
                "delta_xyz_mm": [(end - start) * 1000 for start, end in zip(origin_xyz, actual)],
                "goal_xyz_mm": goal_xyz_mm, "measured_xyz_mm": [value * 1000 for value in actual],
                "reference_xyz_mm": reference_xyz_mm, "reference_source": reference_source,
                "terminal_error_mm": terminal_error_mm, "terminal_squared_error_mm2": terminal_error_mm**2,
            }
        except RuntimeError as error:
            return HTTPStatus.CONFLICT, {"message": str(error), "sequence": sequence - 1}
        finally:
            self.tcp_lock.release()

    def request_recenter(self, session_id):
        if not self.recenter.service_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"message": "归中服务未就绪"}
        if session_id:
            request = ReleaseSource.Request(session_id=session_id)
            status, reply = self.call_service(self.release_for_recenter, request)
            if status != HTTPStatus.OK:
                return status, reply
            if not reply.accepted:
                return HTTPStatus.CONFLICT, {"message": reply.message}
        status, reply = self.call_service(self.recenter, Trigger.Request(), self.recenter_timeout)
        if status == HTTPStatus.GATEWAY_TIMEOUT:
            return status, {"message": "未收到归中结果，请查看当前机器人状态"}
        if status != HTTPStatus.OK:
            return status, reply
        return (HTTPStatus.OK if reply.success else HTTPStatus.CONFLICT,
                {"accepted": reply.success, "message": reply.message})

    @staticmethod
    def call_service(client, request, timeout=3):
        if not client.service_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"message": "ROS 服务未就绪"}
        future = client.call_async(request)
        complete = Event()
        future.add_done_callback(lambda _: complete.set())
        if not complete.wait(timeout):
            client.remove_pending_request(future)
            return HTTPStatus.GATEWAY_TIMEOUT, {"message": "ROS 服务响应超时"}
        return HTTPStatus.OK, future.result()


class TargetHandler(BaseHTTPRequestHandler):
    asset_files = {
        "/": ("index.html", "text/html"),
        "/app.js": ("app.js", "text/javascript"),
        "/session-heartbeat.js": ("session-heartbeat.js", "text/javascript"),
        "/style.css": ("style.css", "text/css"),
        "/robot-view.js": ("robot-view.js", "text/javascript"),
        "/tcp-ik.js": ("tcp-ik.js", "text/javascript"),
        "/vendor/three.module.min.js": ("vendor/three.module.min.js", "text/javascript"),
        "/vendor/OrbitControls.js": ("vendor/OrbitControls.js", "text/javascript"),
        "/vendor/STLLoader.js": ("vendor/STLLoader.js", "text/javascript"),
        "/vendor/THREE-LICENSE.txt": ("vendor/THREE-LICENSE.txt", "text/plain"),
    }

    def do_GET(self):
        url = urlsplit(self.path)
        path = url.path
        if path == "/api/state":
            self.send_json(HTTPStatus.OK, self.server.bridge.snapshot())
            return
        bridge = self.server.bridge
        if path == "/api/load":
            self.send_json(HTTPStatus.OK, bridge.load_state(time.monotonic()))
            return
        models = (bridge.robot_model, bridge.preview_model)
        if path == "/api/model":
            revision = parse_qs(url.query).get("revision", [""])[0]
            robot = next((model for model in models if model and
                          (not revision or str(model[0]["revision"]) == revision)), None)
            if robot is None:
                self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"message": "等待机械臂模型"})
            else:
                self.send_json(HTTPStatus.OK, robot[0])
            return
        mesh = next((model[1][path] for model in models if model and path in model[1]), None)
        if mesh is not None:
            try:
                data = mesh.read_bytes()
            except OSError:
                self.send_error(HTTPStatus.NOT_FOUND)
            else:
                self.send_content(HTTPStatus.OK, "model/stl", data, cache="public, max-age=3600")
            return
        asset = self.asset_files.get(path)
        if asset is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        filename, content_type = asset
        self.send_content(HTTPStatus.OK, content_type, (self.server.assets / filename).read_bytes())

    def do_POST(self):
        if self.headers.get("Host") != f"127.0.0.1:{self.server.server_port}":
            self.send_json(HTTPStatus.FORBIDDEN, {"message": "使用页面的本地地址"})
            return
        origin = self.headers.get("Origin")
        if origin and origin != f"http://127.0.0.1:{self.server.server_port}":
            self.send_json(HTTPStatus.FORBIDDEN, {"message": "请求来源与页面不一致"})
            return
        if self.headers.get_content_type() != "application/json":
            self.send_json(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, {"message": "需要 JSON"})
            return
        path = urlsplit(self.path).path
        recorder = self.server.bridge.recording if path != "/api/heartbeat" else None
        operation_id = uuid4().hex if recorder else None
        try:
            length = int(self.headers.get("Content-Length", 0))
            if not 0 < length <= 65536:
                raise ValueError("invalid request size")
            data = json.loads(self.rfile.read(length))
            if operation_id:
                recorder.add("web_request", operation_id=operation_id, path=path, payload=data)
            code, body = self.server.bridge.request(path, data, operation_id)
        except (ValueError, TypeError, KeyError, AssertionError, OverflowError) as error:
            code, body = HTTPStatus.BAD_REQUEST, {"message": str(error)}
        if operation_id:
            recorder.add("web_result", operation_id=operation_id, path=path, status=int(code), payload=body)
        self.send_json(code, body)

    def log_message(self, _format, *_args):
        pass

    def send_json(self, status, data):
        self.send_content(status, "application/json", json.dumps(data, ensure_ascii=False).encode())

    def send_content(self, status, content_type, data, *, cache="no-store"):
        self.send_response(status)
        if content_type.startswith("text/") or content_type == "application/json":
            content_type += "; charset=utf-8"
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", cache)
        self.end_headers()
        self.wfile.write(data)


def main(args=None):
    rclpy.init(args=args)
    node = TargetWeb()
    try:
        with ThreadingHTTPServer(("127.0.0.1", node.port), TargetHandler) as server:
            server.bridge = node
            server.assets = Path(get_package_share_directory("b601_demos")) / "web"
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            node.get_logger().info(f"Target console: http://127.0.0.1:{server.server_port}")
            node.get_logger().info(f"网页会话日志：{node.recording.path}")
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            except RuntimeError:
                if rclpy.ok():
                    raise
            finally:
                server.shutdown()
                thread.join()
    finally:
        node.recording.add("console_stopped")
        node.recording.close()
        node.destroy_node()
        rclpy.try_shutdown()
