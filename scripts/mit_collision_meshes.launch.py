from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from b601_demos.web_robot_model import find_vendor_model
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, str(Path(get_package_share_directory("b601_moveit_config")) / "launch"))
from moveit_common import build_moveit_configs


def generate_launch_description():
    vendor_model = find_vendor_model()
    if vendor_model is None:
        raise RuntimeError("找不到 B601-DM 模型。请构建当前工作区，或设置 B601_VENDOR_DESCRIPTION。")
    config = build_moveit_configs(hardware="dm", collision_description=vendor_model).to_dict()
    return LaunchDescription([
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[config, {"allow_trajectory_execution": False}]),
    ])
