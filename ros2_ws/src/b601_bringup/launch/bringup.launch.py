#!/usr/bin/env python3
"""bringup.launch.py —— 一个入口拉起“机器人 + ros2_control + 控制器 + 安全监控”。

    # 1) 标准做法（先跑这个）
    ros2 launch b601_bringup bringup.launch.py hardware:=mock

    # 2) 真机（第一次一定先 enable_motors:=false 的只读模式）
    ros2 launch b601_bringup bringup.launch.py hardware:=dm

    # 3) 不想被安全节点盯着（不推荐，只用于排查安全节点本身）
    ros2 launch b601_bringup bringup.launch.py use_safety:=false

启动的节点（顺序即依赖顺序）：

  ┌────────────────────────┐   /robot_description(std)   ┌──────────────────────┐
  │ robot_state_publisher  │ ──────────────────────────► │  rviz2 (可选)        │
  │  xacro → URDF → TF     │ ◄──────── /joint_states ─── │  RobotModel + TF     │
  └────────────────────────┘                             └──────────────────────┘
              ▲ /joint_states
  ┌───────────┴────────────┐   ┌──────────────────────┐
  │ joint_state_broadcaster│ ◄ │ controller_manager   │ ── 加载 b601_controllers.yaml
  └────────────────────────┘   │  (ros2_control_node) │    ├─ b601_arm_controller
                               └──────────────────────┘    └─ b601_gripper_controller
                                          ▲
                                          │ 硬件插件（hardware:=mock|dm|mujoco）
                                     [ 物理世界 ]

  ┌────────────────────────┐
  │ b601_safety_monitor    │  订阅 /joint_states
  │  正常/警告/急停 三档     │  发布 /b601_safety/status
  │  急停 → 停控制器 + 失能  │  服务 /b601_safety/clear_estop
  └────────────────────────┘

默认值全部来自 config/b601_project.yaml（唯一真值），本文件不写死任何数字；
launch 参数只是“临时覆盖”，要永久修改请改配置再跑 scripts/sync_config.py。
"""

import json
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import AndSubstitution, Command, LaunchConfiguration, NotEqualsSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_project_config():
    """读取唯一真值配置（launch 描述阶段就需要它的值做默认值）。"""
    description_share = get_package_share_directory("b601_description")
    with open(f"{description_share}/config/b601_project.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_project_config()
    dm_cfg = cfg["hardware"]["dm"]
    ctrls = cfg["controllers"]
    safety_cfg = cfg["safety"]

    description_share = FindPackageShare("b601_description")
    bringup_share = FindPackageShare("b601_bringup")
    safety_share = FindPackageShare("b601_safety")

    hardware = LaunchConfiguration("hardware")
    dm_device = LaunchConfiguration("dm_device")
    dm_enable_motors = LaunchConfiguration("dm_enable_motors")
    dm_command_mode = LaunchConfiguration("dm_command_mode")
    dm_log_csv = LaunchConfiguration("dm_log_csv")
    mujoco_model = LaunchConfiguration("mujoco_model")
    mujoco_scene = LaunchConfiguration("mujoco_scene")
    use_target_web = LaunchConfiguration("use_target_web")
    target_web_port = LaunchConfiguration("target_web_port")
    target_web_load_api = LaunchConfiguration("target_web_load_api")
    target_web_load_params = {
        "load_api": ParameterValue(target_web_load_api, value_type=str),
        "load_positions": ParameterValue(LaunchConfiguration("target_web_load_positions"), value_type=str),
    }
    use_rviz = LaunchConfiguration("use_rviz")
    use_safety = LaunchConfiguration("use_safety")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_file = LaunchConfiguration("controllers_file")
    mit_gravity_compensation = LaunchConfiguration("mit_gravity_compensation")
    mit_stiffness_switching = LaunchConfiguration("mit_stiffness_switching")
    resolved_controller_file = LaunchConfiguration("resolved_controller_file")
    active_stiffness_switching = LaunchConfiguration("active_stiffness_switching", default="")
    mit_recenter_check_collisions = LaunchConfiguration("mit_recenter_check_collisions")
    mit_recenter_on_startup = LaunchConfiguration("mit_recenter_on_startup")
    safety_params_file = PathJoinSubstitution([safety_share, "config", "safety_params.yaml"])
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    project_config_file = LaunchConfiguration("project_config_file")
    physical_model = PathJoinSubstitution([FindPackageShare("b601_mujoco"), "models", "b601_arm_physical.xml"])

    def configure_mujoco_scene(context):
        if hardware.perform(context) == "mujoco" and not mujoco_model.perform(context):
            scene = mujoco_scene.perform(context)
            actions = [SetLaunchConfiguration("attached_mujoco_scene", scene)]
            if scene == "physical":
                actions.append(SetLaunchConfiguration("mujoco_model", physical_model))
            return actions
        return []

    # robot_description 是 xacro 现算出来的字符串，不是磁盘上的 URDF 文件。
    # 这样 hardware 参数一变，硬件插件那一段就跟着变，不需要维护两份 URDF。
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([description_share, "urdf", "b601_dm.urdf.xacro"]),
                " hardware:=",
                hardware,
                " dm_device:=",
                dm_device,
                " dm_enable_motors:=",
                dm_enable_motors,
                " dm_command_mode:=",
                dm_command_mode,
                " dm_log_csv:=",
                dm_log_csv,
                " mujoco_model:=",
                mujoco_model,
                " initial_positions_file:=",
                initial_positions_file,
                " project_config_file:=",
                project_config_file,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    # controller_manager：唯一“知道硬件存在”的节点。
    # 它按 URDF 里的 <ros2_control> 加载硬件插件，按 controllers yaml 加载控制器。
    def controller_file(context):
        is_mit = dm_command_mode.perform(context) == "mit_impedance"
        if "mit_preset" in context.launch_configurations:
            raise RuntimeError("实验预选已删除，请使用 mit_gravity_compensation:=true 选择重力补偿")
        gravity = IfCondition(mit_gravity_compensation).evaluate(context)
        config_override = controllers_file.perform(context)
        if gravity and (not is_mit or hardware.perform(context) not in ("dm", "mujoco")):
            raise RuntimeError("MIT 重力补偿需要 dm 或 mujoco 后端及 dm_command_mode:=mit_impedance")
        if gravity and config_override:
            raise RuntimeError("mit_gravity_compensation 与 controllers_file 只能选择一个")
        if not is_mit:
            default_file = "b601_controllers.yaml"
        elif (hardware.perform(context) == "mujoco"
              and LaunchConfiguration("attached_mujoco_scene", default="").perform(context) == "physical"):
            default_file = "b601_mit_controllers_mujoco_physical.yaml"
        elif gravity:
            default_file = "b601_mit_controllers_gravity.yaml"
        else:
            default_file = "b601_mit_controllers.yaml"
        return config_override or PathJoinSubstitution(
            [bringup_share, "config", default_file]
        ).perform(context)

    def launch_manager(context):
        return [Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": use_sim_time},
                resolved_controller_file,
            ],
        )]

    # 安全监控：独立节点，只读 /joint_states；急停时停控制器并把硬件置 INACTIVE。
    def launch_safety(context):
        dynamic = (hardware.perform(context) in ("dm", "mujoco")
                   and dm_command_mode.perform(context) == "mit_impedance")
        return [Node(
            package="b601_safety",
            executable="safety_monitor",
            name=safety_cfg.get("node_name", "b601_safety_monitor"),
            output="screen",
            parameters=[safety_params_file, {"use_sim_time": use_sim_time,
                                            "use_dynamic_joint_states": dynamic}],
            condition=IfCondition(use_safety),
        )]

    # 控制器清单做成参数：位置模式用 JTC，MIT 模式换成 b601_mit_controller。
    # 一个 spawner 进程按顺序加载所有控制器。
    controllers_arg = LaunchConfiguration("controllers")

    def controller_names(context):
        is_mit = dm_command_mode.perform(context) == "mit_impedance"
        names = controllers_arg.perform(context).split()
        if not names:
            if is_mit:
                names = ["joint_state_broadcaster", *ctrls["mit"]["instances"]]
            else:
                names = ["joint_state_broadcaster", ctrls["arm"]["name"], ctrls["gripper"]["name"]]
        return names

    def configure_mit_stiffness(context):
        config_path = controller_file(context)
        actions = []
        if dm_command_mode.perform(context) != "mit_impedance":
            return [SetLaunchConfiguration("resolved_controller_file", config_path)]

        config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        types = config["controller_manager"]["ros__parameters"]
        arm_joints = set(cfg["robot"]["arm_joints"])
        mode = mit_stiffness_switching.perform(context)
        arm_instance = ctrls["mit"]["instances"]["b601_mit_arm"]
        profiles = ctrls["mit"]["profiles"]
        hold_ratios = profiles[arm_instance["gravity_profile"]]["hold_gain_ratios"]
        default_gains = profiles[arm_instance["profile"]]["per_joint"]
        enabled = False
        for name in controller_names(context):
            if types.get(name, {}).get("type") != "b601_mit_controller/B601MitController":
                continue
            parameters = config[name]["ros__parameters"]
            joints = set(parameters.get("joints", ()))
            if not joints or not joints <= arm_joints:
                continue
            if mode == "true":
                parameters.update(free_enabled=True, free_follow_feedback=True, low_after_first_target=True)
                for gain, ratio in hold_ratios.items():
                    if f"{gain}_free" not in parameters and f"{gain}_hold" not in parameters:
                        move_key = f"{gain}_move"
                        move = parameters.get(move_key)
                        if move is None:
                            move = [default_gains[joint][move_key] for joint in parameters["joints"]]
                        parameters[f"{gain}_free"] = (
                            [value * ratio for value in move] if isinstance(move, list) else move * ratio)
            elif mode == "false":
                parameters["free_enabled"] = False
            free = parameters.get("free_enabled", True)
            enabled |= any(free) if isinstance(free, list) else free

        if mode != "auto":
            temporary = TemporaryDirectory(prefix="b601-mit-controllers-")
            output = Path(temporary.name) / "controllers.yaml"
            output.write_text(yaml.safe_dump(config), encoding="utf-8")
            config_path = str(output)
            actions.append(RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: temporary.cleanup())))

        actual = "true" if enabled else "false"
        return [
            *actions,
            SetLaunchConfiguration("resolved_controller_file", config_path),
            SetLaunchConfiguration("active_stiffness_switching", actual),
            LogInfo(msg=f"MIT 手臂软硬刚度切换{'已开启' if enabled else '已关闭'}（启动选项 {mode}）"),
        ]

    def launch_target_arbiter(context):
        if dm_command_mode.perform(context) != "mit_impedance":
            return []
        with open(resolved_controller_file.perform(context), encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        types = config["controller_manager"]["ros__parameters"]
        routes = {
            name: config[name]["ros__parameters"]["joints"]
            for name in controller_names(context)
            if types.get(name, {}).get("type") == "b601_mit_controller/B601MitController"
        }
        if not routes:
            return []
        route_params = {"project_config_file": project_config_file, "routes": json.dumps(routes)}
        return [
            Node(
                package="b601_demos",
                executable="target_arbiter",
                output="screen",
                parameters=[route_params],
            ),
            Node(
                package="b601_demos",
                executable="target_bridge",
                output="screen",
                parameters=[route_params],
            ),
            Node(
                package="b601_demos",
                executable="load_node",
                output="screen",
                condition=IfCondition(AndSubstitution(use_target_web, NotEqualsSubstitution(target_web_load_api, ""))),
                parameters=[{"project_config_file": project_config_file, **target_web_load_params}],
            ),
            Node(
                package="b601_demos",
                executable="target_web",
                output="screen",
                condition=IfCondition(use_target_web),
                parameters=[{
                    **target_web_load_params,
                    "project_config_file": project_config_file,
                    "port": ParameterValue(target_web_port, value_type=int),
                    "gravity_source": ParameterValue(
                        LaunchConfiguration("target_web_gravity_source"), value_type=str),
                    "gravity_api": ParameterValue(
                        LaunchConfiguration("target_web_gravity_api"), value_type=str),
                    "attached_backend": ParameterValue(hardware, value_type=str),
                    "attached_mujoco_scene": ParameterValue(
                        LaunchConfiguration("attached_mujoco_scene", default=""), value_type=str),
                    "attached_stiffness_switching": ParameterValue(active_stiffness_switching, value_type=str),
                }],
            ),
        ]

    def launch_spawners(context):
        is_mit = dm_command_mode.perform(context) == "mit_impedance"
        is_dm = hardware.perform(context) == "dm"
        names = controller_names(context)
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*names, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        if not is_mit or (is_dm and not IfCondition(dm_enable_motors).evaluate(context)):
            return [spawner]

        recenter_on_startup = (
            is_dm and IfCondition(mit_recenter_on_startup).evaluate(context)
            and {"joint_state_broadcaster", "b601_mit_arm"}.issubset(names)
        )
        parameters = {"project_config_file": project_config_file,
                      "recenter_on_startup": recenter_on_startup,
                      "gravity_source": ParameterValue(
                          LaunchConfiguration("mit_recenter_gravity_source"), value_type=str),
                      "gravity_api": ParameterValue(
                          LaunchConfiguration("mit_recenter_gravity_api"), value_type=str)}
        collision_override = mit_recenter_check_collisions.perform(context)
        if collision_override:
            parameters["check_collisions"] = IfCondition(mit_recenter_check_collisions).evaluate(context)
        recenter = Node(
            package="b601_demos",
            executable="mit_recenter",
            output="screen",
            parameters=[parameters],
        )
        if not recenter_on_startup:
            return [spawner, recenter]

        def after_spawner(event, launch_context):
            if event.returncode == 0 and not launch_context.is_shutdown:
                return [recenter]
            return []

        return [RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_spawner)),
                spawner]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([description_share, "rviz", "b601_display.rviz"])],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_target_web", default_value="false",
                description="启动仅监听 127.0.0.1 的 MIT 浏览器目标入口",
            ),
            DeclareLaunchArgument(
                "target_web_port", default_value="8765",
                description="浏览器目标入口的本地 HTTP 端口",
            ),
            DeclareLaunchArgument(
                "target_web_gravity_source", default_value="internal", choices=["internal", "external"],
            ),
            DeclareLaunchArgument("target_web_gravity_api", default_value=""),
            DeclareLaunchArgument(
                "target_web_load_api", default_value="",
                description="Load Python module:factory (empty disables load feedback)",
            ),
            DeclareLaunchArgument("target_web_load_positions", default_value="gripper"),
            DeclareLaunchArgument(
                "mit_recenter_gravity_source", default_value="internal", choices=["internal", "external"],
            ),
            DeclareLaunchArgument("mit_recenter_gravity_api", default_value=""),
            DeclareLaunchArgument(
                "hardware", default_value=str(cfg["hardware"].get("default_backend", "mock")),
                description="硬件后端：mock（默认，无实物）| dm（真机）| mujoco（动力学仿真）",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="是否启动 rviz2",
            ),
            DeclareLaunchArgument(
                "use_safety", default_value="true" if safety_cfg.get("enabled", True) else "false",
                description="是否启动 b601_safety_monitor（默认开；关掉只用于排查安全节点本身）",
            ),
            DeclareLaunchArgument(
                "dm_device", default_value=str(dm_cfg["device"]),
                description="真机串口（hardware:=dm 时有效；换 USB 口后编号可能变化）",
            ),
            DeclareLaunchArgument(
                "dm_enable_motors", default_value="true" if dm_cfg["enable_motors"] else "false",
                description="真机是否真的使能电机。false = 只读模式（首调必须 false）",
            ),
            DeclareLaunchArgument(
                "dm_command_mode", default_value=str(dm_cfg["command_mode"]),
                description="控制模式：pos_vel / mit / mit_impedance（电机侧 PD）/ mit_torque（原始力矩接口）",
            ),
            DeclareLaunchArgument(
                "dm_log_csv", default_value=str(dm_cfg.get("log_csv", "")),
                description="真机 CSV 记录路径（速度/MIT 数据）；留空 = 关闭。"
                            "例：dm_log_csv:=\"$PWD/.verify/run.csv\"",
            ),
            DeclareLaunchArgument(
                "mujoco_scene", default_value="free", choices=["free", "physical"],
                description="MuJoCo 物理模式：free 关闭重力与碰撞，physical 同时开启",
            ),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value="",
                description="显式覆盖 MuJoCo 模型路径（留空=按 mujoco_scene 选择）；"
                            "仿真回归可指向零重力夹具等变体",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false",
                description="仿真后端（mujoco/gazebo）里设为 true",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description="控制器配置文件，留空按控制模式和 MuJoCo 场景自动选择",
            ),
            DeclareLaunchArgument(
                "mit_gravity_compensation", default_value="false", choices=["false", "true"],
                description="选择电机侧 MIT 重力补偿配置；内置 MuJoCo 物理场景使用专用高刚度配置",
            ),
            DeclareLaunchArgument(
                "mit_stiffness_switching", default_value="auto", choices=["auto", "true", "false"],
                description="MIT 手臂软硬刚度切换：自动按预设、显式开启或保持高刚度",
            ),
            DeclareLaunchArgument(
                "mit_recenter_on_startup", default_value="true", choices=["false", "true"],
                description="已使能的真机 MIT 控制器启动后自动归中",
            ),
            DeclareLaunchArgument(
                "mit_recenter_check_collisions", default_value="",
                description="覆盖统一配置中的 MIT 归中 MoveIt 状态有效性检查开关",
            ),
            DeclareLaunchArgument(
                "controllers", default_value="",
                description="控制器名称（空格分隔），留空按 dm_command_mode 自动选择",
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=PathJoinSubstitution(
                    [description_share, "config", "initial_positions.yaml"]),
                description="上电初值：mock 决定显示姿态，真机决定“以为关节在哪儿”",
            ),
            DeclareLaunchArgument(
                "project_config_file",
                default_value=PathJoinSubstitution(
                    [description_share, "config", "b601_project.yaml"]),
                description="全项目唯一参数真值（关节限位/电机参数/控制模式）",
            ),

            OpaqueFunction(function=configure_mujoco_scene),
            OpaqueFunction(function=configure_mit_stiffness),
            robot_state_publisher,
            OpaqueFunction(function=launch_manager),
            OpaqueFunction(function=launch_safety),
            rviz,

            OpaqueFunction(function=launch_target_arbiter),
            OpaqueFunction(function=launch_spawners),
        ]
    )
