#!/usr/bin/env python3
"""Start the existing B601 bringup, the trajectory endpoint and MoveIt."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_common import launch_moveit, trajectory_mode  # noqa: E402


def launch_bringup(context):
    hardware = LaunchConfiguration("hardware").perform(context)
    mode = trajectory_mode(context)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("b601_bringup"), "launch", "bringup.launch.py",
        ])),
        launch_arguments={
            "hardware": hardware,
            "dm_command_mode": mode,
            "dm_enable_motors": LaunchConfiguration("dm_enable_motors"),
            "mit_gravity_compensation": str(hardware == "dm" and mode == "mit_impedance").lower(),
            "mujoco_scene": LaunchConfiguration("mujoco_scene"),
            "use_rviz": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mujoco", choices=["mock", "dm", "mujoco"]),
        DeclareLaunchArgument("trajectory_mode", default_value="auto",
                              choices=["auto", "mit_impedance", "pos_vel"]),
        DeclareLaunchArgument("mujoco_scene", default_value="physical", choices=["free", "physical"]),
        DeclareLaunchArgument("dm_enable_motors", default_value="false", choices=["false", "true"]),
        DeclareLaunchArgument("start_mit_trajectory", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_bringup),
        OpaqueFunction(function=launch_moveit),
    ])
