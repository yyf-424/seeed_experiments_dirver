"""Launch ROS bringup from the standalone local console and report its lifecycle."""
import os
import signal
import subprocess
from contextlib import ExitStack
from fcntl import LOCK_EX, LOCK_NB, flock
from http import HTTPStatus
from pathlib import Path
from tempfile import NamedTemporaryFile
from threading import Lock
from time import monotonic, sleep

from controller_manager_msgs.srv import ListControllers, SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
from rclpy.duration import Duration
from std_srvs.srv import SetBool

from .target_arbiter import ARBITER


BACKEND_NAMES = {"mock": "Mock", "mujoco": "MuJoCo", "dm": "实机"}
MUJOCO_SCENES = ("free", "physical")
STIFFNESS_SETTINGS = ("auto", "true", "false")


def bringup_command(backend, project_file, mujoco_scene="free", stiffness_switching="auto"):
    args = [
        "ros2", "launch", "b601_bringup", "bringup.launch.py",
        f"hardware:={backend}", "dm_command_mode:=mit_impedance",
        "use_rviz:=false", "use_safety:=true", "use_target_web:=false",
        "mit_recenter_on_startup:=false",
        f"mit_stiffness_switching:={stiffness_switching}",
    ]
    if backend == "dm":
        args.extend(("dm_enable_motors:=true", "mit_gravity_compensation:=true"))
    elif backend == "mujoco":
        args.append(f"mujoco_scene:={mujoco_scene}")
    if project_file:
        args.append(f"project_config_file:={project_file}")
    return args


class WebBackend:
    def __init__(self, node, project, project_file, attached, attached_mujoco_scene,
                 attached_stiffness_switching, default_stiffness_switching):
        self.node = node
        self.project_file = project_file
        self.attached = attached
        self.mujoco_scene = attached_mujoco_scene or None
        self.stiffness_switching = attached_stiffness_switching or None
        self.default_stiffness_switching = default_stiffness_switching
        self.device = Path(project["hardware"]["dm"]["device"])
        self.startup_timeout = project["tools"]["sim_mit_ros"]["startup_timeout"]
        self.controllers = {"joint_state_broadcaster", *project["controllers"]["mit"]["instances"]}
        self.controller_list = node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.controller_request = None
        self.switch = node.create_client(SwitchController, "/controller_manager/switch_controller")
        self.hardware_state = node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state")
        self.reserve = node.create_client(SetBool, f"{ARBITER}/reserve_disable")
        self.hardware_component = project["safety"]["estop"]["hardware_component"]
        self.service_timeout = project["safety"]["estop"]["service_timeout_sec"]
        self.active = False
        self.lock = Lock()
        self.process = None
        self.backend = None
        self.started_at = None
        self.stopping = False
        self.disabling = False
        self.disabled = False
        self.error = None
        self.log_path = None
        node.create_timer(0.5, self.update_controllers)

    def update_controllers(self):
        if not self.controller_list.service_is_ready():
            self.active = False
            if self.controller_request is not None:
                self.controller_list.remove_pending_request(self.controller_request)
                self.controller_request = None
            return
        if self.controller_request is None:
            self.controller_request = self.controller_list.call_async(ListControllers.Request())
        elif self.controller_request.done():
            result = self.controller_request.result()
            self.controller_request = None
            active = {c.name for c in result.controller if c.state == "active"} if result else set()
            self.active = self.controllers <= active

    def graph_in_use(self):
        return (self.controller_list.service_is_ready()
                or bool(self.node.count_publishers("/joint_states"))
                or "controller_manager" in self.node.get_node_names())

    def reap(self):
        if self.process is not None and (result := self.process.poll()) is not None:
            self.error = None if self.stopping else f"{BACKEND_NAMES[self.backend]} 后端已退出（{result}）"
            if self.backend != "dm" or self.disabled:
                self.backend = None
            self.process = None
            self.stopping = False
            self.active = False

    def snapshot(self, feedback_ready, parked):
        with self.lock:
            self.reap()
            active_backend = self.attached or self.backend
            attached = bool(self.attached) or (self.process is None and self.graph_in_use())
            if self.disabling or self.stopping:
                phase, message = "stopping", "正在失能并退出后端…"
            elif self.backend and self.process is None:
                phase, message = "error", self.error or "实机失能状态未确认"
            elif self.backend:
                label = BACKEND_NAMES[self.backend]
                if self.active and self.controller_list.service_is_ready() and feedback_ready:
                    detail = "电机已使能" if self.backend == "dm" else "MIT 控制器已激活"
                    phase, message = "connected", f"{label} 已连接 · {detail}"
                elif monotonic() - self.started_at >= self.startup_timeout:
                    phase, message = "error", f"{label} 控制器或反馈未就绪，请查看启动日志"
                else:
                    phase, message = "starting", f"正在连接 {label}…"
            elif attached:
                phase = "connected" if feedback_ready else "starting"
                label = BACKEND_NAMES.get(self.attached, "现有后端")
                message = f"{label} 已连接" if feedback_ready else f"等待 {label} 反馈…"
            elif self.error:
                phase, message = "error", self.error
            elif self.disabled:
                phase, message = "idle", "整机已失能，可切换后端"
            else:
                phase, message = "idle", "等待连接"
            guard_ready = self.reserve.service_is_ready()
            can_disable = (self.process is not None and parked and not self.disabling
                           and not self.stopping and guard_ready)
            disable_reason = None
            if phase == "connected":
                if not guard_ready:
                    disable_reason = "当前后端缺少失能确认接口，需结束本次运行后启动新版后端。"
                elif self.process is None:
                    disable_reason = "当前后端由独立进程管理，请使用原停机入口。"
                elif not parked:
                    disable_reason = "归中到位后可点击 Disable。"
            return {
                "name": active_backend,
                "mujoco_scene": self.mujoco_scene if active_backend == "mujoco" else None,
                "stiffness_switching": self.stiffness_switching if active_backend else None,
                "default_stiffness_switching": self.default_stiffness_switching,
                "phase": phase,
                "message": message,
                "can_connect": not attached and self.backend is None and self.process is None,
                "can_disable": can_disable,
                "disable_reason": disable_reason,
                "log": str(self.log_path) if phase == "error" and self.log_path else None,
            }

    def connect(self, backend, mujoco_scene="free", stiffness_switching=None):
        if not isinstance(backend, str) or backend not in BACKEND_NAMES:
            return HTTPStatus.BAD_REQUEST, {"message": "请选择 Mock、MuJoCo 或实机"}
        if backend == "mujoco" and mujoco_scene not in MUJOCO_SCENES:
            return HTTPStatus.BAD_REQUEST, {"message": "请选择同时开启或关闭重力与碰撞"}
        if stiffness_switching is None:
            stiffness_switching = self.default_stiffness_switching
        if stiffness_switching not in STIFFNESS_SETTINGS:
            return HTTPStatus.BAD_REQUEST, {"message": "请选择自动、开启或关闭软硬刚度切换"}
        with self.lock:
            self.reap()
            if self.backend or self.process is not None or self.attached or self.graph_in_use():
                return HTTPStatus.CONFLICT, {"message": "整机失能并退出后才能切换后端"}
            if backend == "mock":
                return HTTPStatus.OK, {"message": "Mock 已选择，未启动后端"}
            try:
                self.process = self.launch(backend, mujoco_scene, stiffness_switching)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                return HTTPStatus.SERVICE_UNAVAILABLE, {"message": str(error)}
            self.backend = backend
            self.mujoco_scene = mujoco_scene if backend == "mujoco" else None
            if stiffness_switching == "auto":
                self.stiffness_switching = "true" if backend == "dm" else "false"
            else:
                self.stiffness_switching = stiffness_switching
            self.started_at = monotonic()
            self.disabled = False
            self.stopping = False
            self.error = None
            self.active = False
            self.node.get_logger().info(
                f"{BACKEND_NAMES[backend]} bringup PID {self.process.pid}，日志 {self.log_path}")
        return HTTPStatus.ACCEPTED, {"message": f"正在连接 {BACKEND_NAMES[backend]}…"}

    def launch(self, backend, mujoco_scene, stiffness_switching):
        directory = Path(os.environ.get("ROS_LOG_DIR") or Path.home() / ".ros" / "log")
        directory.mkdir(parents=True, exist_ok=True)
        locks = Path(os.environ.get("XDG_RUNTIME_DIR") or Path.home() / ".ros")
        locks.mkdir(parents=True, exist_ok=True)
        domain = self.node.context.get_domain_id()
        names = [f"b601-web-domain-{domain}"]
        if backend == "dm":
            if not self.device.is_char_device():
                raise RuntimeError(f"找不到真机串口 {self.device}")
            names.append(f"b601-web-serial-{self.device.stat().st_rdev}")
        with ExitStack() as stack:
            descriptors = []
            for name in names:
                descriptor = stack.enter_context((locks / f"{name}.lock").open("a")).fileno()
                try:
                    flock(descriptor, LOCK_EX | LOCK_NB)
                except BlockingIOError as error:
                    raise RuntimeError("已有网页启动的后端占用当前 ROS 域或串口") from error
                descriptors.append(descriptor)
            if backend == "dm":
                scan = subprocess.run(["fuser", str(self.device)], capture_output=True, timeout=3)
                if scan.returncode != 1:
                    raise RuntimeError(f"未确认真机串口空闲 {self.device}")
            output = stack.enter_context(NamedTemporaryFile(
                prefix=f"b601-web-{backend}-", suffix=".log", dir=directory, delete=False))
            self.log_path = Path(output.name)
            return subprocess.Popen(
                bringup_command(backend, self.project_file, mujoco_scene, stiffness_switching),
                start_new_session=True,
                env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
                stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                pass_fds=tuple(descriptors))

    def disable(self, call_service):
        with self.lock:
            self.reap()
            if self.process is None or self.disabling or self.stopping:
                return HTTPStatus.CONFLICT, {"message": "没有可失能的网页后端"}
            process = self.process
            backend = self.backend
            self.disabling = True

        def call(client, request):
            status, reply = call_service(client, request, self.service_timeout)
            if status != HTTPStatus.OK:
                raise RuntimeError(f"{client.srv_name}：{reply['message']}")
            return reply

        reserved = False
        try:
            active = call(self.controller_list, ListControllers.Request())
            reply = call(self.reserve, SetBool.Request(data=True))
            if not reply.success:
                return HTTPStatus.CONFLICT, {"message": reply.message}
            reserved = True
            controllers = [c.name for c in active.controller
                           if c.state == "active" and c.name != "joint_state_broadcaster"]
            if controllers:
                request = SwitchController.Request(
                    deactivate_controllers=controllers,
                    strictness=SwitchController.Request.STRICT,
                    timeout=Duration(seconds=self.service_timeout).to_msg())
                if not call(self.switch, request).ok:
                    raise RuntimeError("停用控制器失败")
                remaining = call(self.controller_list, ListControllers.Request())
                if any(c.state == "active" for c in remaining.controller
                       if c.name != "joint_state_broadcaster"):
                    raise RuntimeError("仍有命令控制器运行")
            log_offset = self.log_path.stat().st_size if backend == "dm" else 0
            request = SetHardwareComponentState.Request(
                name=self.hardware_component, target_state=State(id=State.PRIMARY_STATE_INACTIVE, label="inactive"))
            reply = call(self.hardware_state, request)
            if not reply.ok or reply.state.id != State.PRIMARY_STATE_INACTIVE:
                raise RuntimeError(f"整机未进入 inactive，当前状态 {reply.state.label}")
            if backend == "dm":
                self.confirm_dm_disabled(log_offset)
            self.node.get_logger().info(f"整机 {self.hardware_component} 已确认 inactive")
            with self.lock:
                self.disabled = True
                self.stopping = True
                process.send_signal(signal.SIGINT)
            return HTTPStatus.ACCEPTED, {"message": "整机已失能，正在退出后端"}
        except (OSError, RuntimeError) as error:
            return HTTPStatus.SERVICE_UNAVAILABLE, {"message": str(error)}
        finally:
            if reserved and not self.disabled:
                call_service(self.reserve, SetBool.Request(data=False), self.service_timeout)
            with self.lock:
                self.disabling = False

    def confirm_dm_disabled(self, offset):
        deadline = monotonic() + self.service_timeout
        while monotonic() < deadline:
            with self.log_path.open("rb") as log:
                log.seek(offset)
                output = log.read().decode(errors="replace")
            if "[DmSystemHardware]: 失能电机失败" in output:
                raise RuntimeError("真机驱动报告失能失败")
            if "[DmSystemHardware]: 已失能电机" in output:
                return
            sleep(0.05)
        raise RuntimeError("未收到真机驱动的失能成功记录")
