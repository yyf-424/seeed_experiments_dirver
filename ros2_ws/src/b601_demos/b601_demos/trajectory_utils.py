"""trajectory_utils.py —— 拼轨迹的公共工具（不是节点，import 用）。

为什么单独抽出来：
    “怎么把 [关节角列表 + 时间] 拼成一条 FollowJointTrajectory” 这件事，
    在三个演示节点里重复了三次，而且最容易写错（时间戳、起点、单位）。

约定：
    - 关节顺序固定为 ARM_JOINTS（joint1..joint6）
    - 角度单位 rad、时间单位 s、夹爪位移单位 m
    - 限位来自 config/b601_project.yaml（唯一真值）；拼轨迹前先验证，越界直接拒绝
"""

import math

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from b601_description.project_config import load_project_config

_PROJECT = load_project_config()
ARM_JOINTS = tuple(_PROJECT["robot"]["arm_joints"])
GRIPPER_JOINT = _PROJECT["robot"]["gripper_joint"]
MIT_TRAJECTORY_CONTROLLER = _PROJECT["moveit"]["mit_trajectory_controller"]
RECENTER_POSITIONS = tuple(_PROJECT["recenter"]["joints"][name] for name in ARM_JOINTS)
RECENTER_COMPLETED = "/b601_mit_recenter/completed_dispatch"
RECENTER_TOLERANCE = _PROJECT["recenter"]["position_tolerance_rad"]
_JOINTS = _PROJECT["joints"]

# 限位直接来自统一配置，不再在本文件抄一份（历史上抄过 ±2.8，和 URDF 的 ±2.60 打架）
JOINT_LOWER = {name: float(j["min_pos"]) for name, j in _JOINTS.items()}
JOINT_UPPER = {name: float(j["max_pos"]) for name, j in _JOINTS.items()}
JOINT_VEL_MAX = {name: float(j["max_velocity"]) for name, j in _JOINTS.items()}


def mit_recenter_tolerances(project):
    park = project["recenter"]
    arm = project["robot"]["arm_joints"]
    overrides = park.get("mit", {}).get("position_tolerance_rad_by_joint", {})
    if set(overrides) - set(arm):
        raise ValueError("MIT 归中容差包含未知手臂关节")
    tolerances = tuple(float(overrides.get(name, park["position_tolerance_rad"])) for name in arm)
    if not all(math.isfinite(value) and value >= 0.0 for value in tolerances):
        raise ValueError("MIT 归中容差必须是有限非负值")
    return tolerances


def recenter_position_bounds(home, tolerances, bounds):
    return tuple((max(low, target - tolerance), min(high, target + tolerance))
                 for target, tolerance, (low, high) in zip(home, tolerances, bounds))


def mit_recenter_bounds(project):
    arm = project["robot"]["arm_joints"]
    home = (project["recenter"]["joints"][name] for name in arm)
    limits = ((project["joints"][name]["min_pos"], project["joints"][name]["max_pos"])
              for name in arm)
    return recenter_position_bounds(home, mit_recenter_tolerances(project), limits)


def at_recenter(positions, *, mit=False):
    """检查六轴反馈是否同时满足停放位容差和关节限位。"""
    tolerances = mit_recenter_tolerances(_PROJECT) if mit else (RECENTER_TOLERANCE,) * len(ARM_JOINTS)
    bounds = recenter_position_bounds(
        RECENTER_POSITIONS, tolerances, ((JOINT_LOWER[name], JOINT_UPPER[name]) for name in ARM_JOINTS)
    )
    return len(positions) == len(bounds) and all(
        low <= actual <= high for actual, (low, high) in zip(positions, bounds))


def check_limits(joint_names, positions):
    """返回越界描述列表；空列表表示全部合法。"""
    problems = []
    for name, value in zip(joint_names, positions):
        low = JOINT_LOWER[name]
        high = JOINT_UPPER[name]
        if not (low <= value <= high):
            problems.append(f"{name}={value:.3f} 超出 [{low:.3f}, {high:.3f}]")
    return problems


def duration(seconds):
    """float 秒 → builtin_interfaces/Duration。"""
    sec, nanosec = divmod(round(seconds * 1e9), 1_000_000_000)
    return Duration(sec=sec, nanosec=nanosec)


def make_point(joint_names, positions, time_from_start):
    """一个轨迹点 = 时间 + 与 joint_names 同序的位置。"""
    point = JointTrajectoryPoint()
    point.positions = [float(p) for p in positions]
    point.time_from_start = duration(time_from_start)
    return point


def make_trajectory(joint_names, waypoints, durations):
    """把若干 (关节角元组, 到达时刻) 拼成 JointTrajectory。

    joint_names: 关节名（决定 positions 的顺序）
    waypoints:   [positions, positions, ...]，每个 positions 长度同 joint_names
    durations:   每个 waypoint 的 time_from_start（秒，单调递增）

    注意 time_from_start 是**相对轨迹起点**的绝对时刻，不是相邻两点的时间差，
    这是新手最常写错的地方。
    """
    if len(waypoints) != len(durations):
        raise ValueError("waypoints 与 durations 数量必须一致")

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    for positions, t in zip(waypoints, durations):
        trajectory.points.append(make_point(joint_names, positions, t))
    return trajectory


def describe(trajectory):
    """把轨迹打成人能读的一行摘要，日志里用。"""
    last = trajectory.points[-1]
    return (
        f"{len(trajectory.points)} 点 / 总时长 "
        f"{last.time_from_start.sec + last.time_from_start.nanosec * 1e-9:.2f}s / 终点 "
        + ", ".join(f"{n}={p:+.3f}" for n, p in zip(trajectory.joint_names, last.positions))
    )

def spin_until(node, future, timeout_sec: float, what: str):
    """带超时地等待一个 future，永不无限阻塞。

    为什么必须有超时（2026-11-21 实测）：
        action 的 goal 响应可能被 DDS 丢掉，服务端 rclcpp 会打印
        "Failed to send goal response (timeout): client will not receive response"。
        此时 rclpy 的 `spin_until_future_complete(future)` 没有超时会**永久阻塞**，
        节点看起来像"卡死"，实际上是在等一个永远不会到的响应。
        教学节点必须报错退出，而不是挂在那里。

    返回：future 的结果；超时/异常返回 None（调用方负责打日志与返回码）。
    """
    import rclpy

    try:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    except Exception as exc:  # noqa: BLE001 —— 上下文失效等
        node.get_logger().error(f"{what}：等待时出错（{exc}）")
        return None
    if not future.done():
        node.get_logger().error(
            f"{what}：{timeout_sec:.0f}s 内没有收到响应 —— 多半是 DDS 把响应丢了；"
            "可重试一次，或检查 RMW（本机沙箱偶发）。"
        )
        return None
    try:
        return future.result()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"{what}：拿结果失败（{exc}）")
        return None


def send_goal_with_retry(node, client, goal, what: str, timeout_sec: float = _PROJECT["nodes"]["trajectory_utils"]["goal_request_timeout_sec"],
                         attempts: int = _PROJECT["nodes"]["trajectory_utils"]["goal_request_attempts"], feedback_callback=None):
    """发 goal 并等响应，失败自动重试一次（DDS 偶发丢响应）。

    返回 goal_handle；彻底失败返回 None。
    """
    for attempt in range(1, attempts + 1):
        kwargs = {"feedback_callback": feedback_callback} if feedback_callback else {}
        future = client.send_goal_async(goal, **kwargs)
        handle = spin_until(node, future, timeout_sec, f"{what}（第 {attempt} 次）")
        if handle is not None:
            return handle
        if attempt < attempts:
            node.get_logger().warn(f"{what}：第 {attempt} 次没拿到 goal 响应，重试一次")
    return None
