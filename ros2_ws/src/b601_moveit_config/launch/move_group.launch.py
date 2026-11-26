#!/usr/bin/env python3
"""Attach MoveIt and the MIT trajectory executor to an existing B601 bringup."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_common import launch_moveit  # noqa: E402


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mujoco", choices=["mock", "dm", "mujoco"]),
        DeclareLaunchArgument("trajectory_mode", default_value="auto",
                              choices=["auto", "mit_impedance", "pos_vel"]),
        DeclareLaunchArgument("start_mit_trajectory", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_moveit),
    ])
