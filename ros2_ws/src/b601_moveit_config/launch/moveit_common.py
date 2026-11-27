"""moveit_common.py —— demo.launch.py 与 move_group.launch.py 共用的装配逻辑。

要点（Humble 起 MoveIt 2 的标准做法）：
    MoveItConfigsBuilder 把散落在多个 yaml 里的参数收成一个对象，
    .to_dict() 给 move_group 用，.robot_description* 那几项给 rviz 用。

运动学、惯量和限位沿用 b601_description。MoveIt 和 RViz 的规划模型使用随包安装的
Seeed B601-DM 碰撞网格。
"""

import os
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def use_vendor_collisions(description, vendor_model):
    robot = ET.fromstring(description)
    vendor = ET.parse(vendor_model / "urdf" / "ReBot_Arm_DM.urdf")
    vendor_links = {link.attrib["name"]: link for link in vendor.findall("link")}
    prefix = "package://rebotarm_bringup/description/DM/"
    for link in robot.findall("link"):
        existing = link.findall("collision")
        if not existing:
            continue
        name = link.attrib["name"]
        collisions = vendor_links[name].findall("collision")
        if not collisions:
            raise ValueError(f"Seeed B601-DM is missing collision geometry for {name}")
        for collision in existing:
            link.remove(collision)
        for collision in collisions:
            mesh = collision.find("./geometry/mesh")
            uri = mesh.attrib["filename"]
            if not uri.startswith(prefix):
                raise ValueError(f"Unexpected Seeed B601-DM collision mesh URI: {uri}")
            path = vendor_model / uri.removeprefix(prefix)
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", path.as_uri())
            link.append(collision)
    return ET.tostring(robot, encoding="unicode")


def build_moveit_configs(hardware: str = "mock", mode: str = "pos_vel", *, collision_description=None):
    """按指定硬件后端装配 MoveIt 参数集。"""
    description_share = get_package_share_directory("b601_description")

    config = (
        MoveItConfigsBuilder("b601_dm", package_name="b601_moveit_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "b601_dm.urdf.xacro"),
            mappings={
                "hardware": hardware,
                "dm_command_mode": mode,
                "initial_positions_file": os.path.join(
                    description_share, "config", "initial_positions.yaml"
                ),
                # 唯一真值：关节限位/电机参数都从这里来，MoveIt 看到的模型与控制器一致
                "project_config_file": os.path.join(
                    description_share, "config", "b601_project.yaml"
                ),
            },
        )
        .robot_description_semantic(file_path="config/b601_dm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path=("config/moveit_mit_controllers.yaml"
                                         if mode == "mit_impedance" else "config/moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    vendor_model = Path(collision_description or Path(description_share) / "assets" / "b601_dm").resolve()
    config.robot_description["robot_description"] = use_vendor_collisions(
        config.robot_description["robot_description"], vendor_model
    )
    return config


def trajectory_mode(context):
    mode = LaunchConfiguration("trajectory_mode").perform(context)
    if mode != "auto":
        return mode
    return "pos_vel" if LaunchConfiguration("hardware").perform(context) == "mock" else "mit_impedance"


def launch_moveit(context):
    hardware = LaunchConfiguration("hardware").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    mode = trajectory_mode(context)
    moveit_config = build_moveit_configs(hardware, mode)
    allow_execution = LaunchConfiguration(
        "allow_trajectory_execution", default="true").perform(context) == "true"
    clock = {"use_sim_time": use_sim_time}
    actions = [
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit_config.to_dict(), clock, {"allow_trajectory_execution": allow_execution}]),
    ]
    start_mit = LaunchConfiguration("start_mit_trajectory").perform(context) == "true"
    if allow_execution and mode == "mit_impedance" and start_mit:
        actions.insert(0, Node(package="b601_demos", executable="mit_trajectory", output="screen",
                               parameters=[clock]))
    if LaunchConfiguration("use_rviz").perform(context) == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", moveit_rviz_config_file()],
                            parameters=rviz_parameters(moveit_config, use_sim_time)))
    return actions


def moveit_rviz_config_file() -> str:
    return os.path.join(get_package_share_directory("b601_moveit_config"), "rviz", "moveit.rviz")


def rviz_parameters(moveit_config, use_sim_time):
    """RViz 的 MotionPlanning 插件需要的参数（比 move_group 少很多）。"""
    return [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"use_sim_time": use_sim_time},
    ]
