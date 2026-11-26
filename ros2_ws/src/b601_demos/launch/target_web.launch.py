"""Start the web console and an independent URDF publisher."""

from http.client import HTTPConnection, HTTPException
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def preview_already_running(port, gravity_source, gravity_api):
    if port == 0:
        return False
    connection = HTTPConnection("127.0.0.1", port, timeout=2)
    try:
        connection.request("GET", "/api/state")
        with connection.getresponse() as response:
            if response.status == 200:
                state = json.load(response)
                if isinstance(state, dict) and (
                    state.get("preview_enabled") is True or isinstance(state.get("preview_pose"), dict)
                ):
                    active = state.get("gravity_source", "internal"), state.get("gravity_api", "")
                    requested = gravity_source, gravity_api if gravity_source == "external" else ""
                    if active != requested:
                        raise RuntimeError(
                            f"端口 {port} 的控制台已用其他重力配置运行，请先结束该控制台或使用其他端口。")
                    return True
    except ConnectionRefusedError:
        return False
    except (HTTPException, OSError, ValueError):
        pass
    finally:
        connection.close()
    raise RuntimeError(f"端口 {port} 已被其他服务占用，请使用 port:=<空闲端口>。")


def launch_console(context):
    port = int(LaunchConfiguration("port").perform(context))
    gravity_source = LaunchConfiguration("gravity_source").perform(context)
    gravity_api = LaunchConfiguration("gravity_api").perform(context)
    if preview_already_running(port, gravity_source, gravity_api):
        return [LogInfo(msg=f"B601 控制台已运行，请打开 http://127.0.0.1:{port}/")]

    description = Path(get_package_share_directory("b601_description"))
    urdf = xacro.process_file(
        str(description / "urdf" / "b601_dm.urdf.xacro"),
        mappings={"include_ros2_control": "false"},
    ).toxml()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="/b601_preview",
            output="screen",
            parameters=[{"robot_description": urdf}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            on_exit=[Shutdown()],
        ),
        Node(
            package="b601_demos",
            executable="target_web",
            output="screen",
            parameters=[{
                "port": port, "preview_model": True,
                "gravity_source": gravity_source,
                "gravity_api": ParameterValue(gravity_api, value_type=str),
                "default_stiffness_switching": ParameterValue(
                    LaunchConfiguration("mit_stiffness_switching"), value_type=str),
            }],
            on_exit=[Shutdown()],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8765", description="Local console HTTP port"),
        DeclareLaunchArgument("gravity_source", default_value="internal", choices=["internal", "external"]),
        DeclareLaunchArgument("gravity_api", default_value="", description="External Python module:factory"),
        DeclareLaunchArgument(
            "mit_stiffness_switching", default_value="false", choices=["auto", "true", "false"],
            description="网页连接 MIT 后端时的默认软硬刚度切换选项",
        ),
        OpaqueFunction(function=launch_console),
    ])
