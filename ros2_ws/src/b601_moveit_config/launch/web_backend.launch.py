"""Group the Web-managed MIT bringup and its planning-only MoveIt instance."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("b601_bringup")
    moveit = FindPackageShare("b601_moveit_config")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "bringup.launch.py"]))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([moveit, "launch", "move_group.launch.py"])),
            launch_arguments={
                "use_rviz": "false",
                "trajectory_mode": "mit_impedance",
                "allow_trajectory_execution": "false",
            }.items(),
        ),
    ])
