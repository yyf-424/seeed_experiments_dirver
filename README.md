# B601-DM ROS 2 实验工作区 v1.1.1

这套框架用 ROS 2 Humble 和 ros2_control 驱动 B601-DM 六轴机械臂与夹爪。
上层提供本地网页、命令行、Python `TargetClient` 和原生 ROS 2 接口。
MoveIt 2 提供标准位置轨迹规划，MuJoCo 提供动力学仿真。

**控制器目前可用，增益、重力补偿和切换表现仍需细调。**
已完成高刚度跟踪、两档重力补偿、短时低刚度保持和归中失能的基础验收。负载变化、全工作区精度和长期低刚度表现仍待系统评估。
v1.1.1 支持同帧发送位置、速度、任务前馈以及按关节覆盖重力和摩擦补偿。网页分别显示两类补偿的实时来源。网页与重力补偿启动脚本默认保持高刚度，可在启动时显式开启切换。SDK、HTTP 网关和七轴增量脚本可在上层用实测位置计算绝对目标。见 [扩展验收](validation/target-web.md#b601_experiments-扩展验收) 和 [发布记录](CHANGELOG.md)。

## 演示

框架支持电机侧 MIT 重力补偿与变刚度控制。网页可渲染 B601-DM 官方三维模型、连接 MuJoCo 或实机并发送关节目标。
常用安全防护包括关节限位、超速、过温与反馈异常监测，以及归中确认后的整机失能。

![B601-DM 框架演示](./assets/demo-preview.gif)

## 选择使用方式

| 场景 | 入口 |
|---|---|
| 只看网页与官方三维模型 | `./scripts/start_target_web.sh`，打开 [本地控制台](http://127.0.0.1:8765/) |
| 演练网页发令与归中 | 在网页手动连接 MuJoCo，发送目标后归中、Disable |
| 跑标准位置与 MoveIt 轨迹 | 加载当前工作区后运行 `ros2 launch b601_moveit_config demo.launch.py hardware:=mock` |
| 控制真机 | 现场确认后，在网页连接「实机」或在终端运行 [MIT 重力补偿脚本](#快速启动重力补偿)，按所选入口归中并失能 |
| 接入程序或策略 | 用 [TargetClient 或原生 ROS 2](#ros-2-接口) 申请整机控制权并发送六轴与夹爪目标 |

## 安装与运行前置条件

本版在 Ubuntu 22.04（x86_64）、ROS 2 Humble 和系统 Python 3.10 上验证。先配置 Humble 的 apt 软件源。见 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

| 用途 | 所需组件 |
|---|---|
| 从源码完整构建 | ROS 2 Humble、C++17 编译器、CMake、colcon、Python 3.10、PyYAML、xacro、ros2_control、MoveIt 2、`motorbridge` 提供的 `libmotor_abi.so` |
| 网页与官方模型预览 | 当前工作区已构建，浏览器支持 WebGL。在本机可用 HTTP 端口上运行，默认 `127.0.0.1:8765` |
| Mock / MoveIt 2 | ros2_control、轨迹控制器和 MoveIt 2。使用 RViz 时需要图形会话 |
| MuJoCo 后端及仿真回归 | 构建前安装 Python `mujoco`，由 CMake 查找其 C++ 头文件和动态库 |
| B601-DM 实机 | B601-DM、24 V 供电、达妙 USB-CAN 串口桥、对串口有读写权限的用户。探针和 C++ 驱动都使用 `motorbridge` |

在已配置 ROS 软件源的 Ubuntu 22.04 上安装构建和常用运行依赖：

```bash
sudo apt update
sudo apt install \
  ros-humble-desktop ros-humble-xacro \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-ros2controlcli ros-humble-control-msgs \
  ros-humble-moveit ros-humble-moveit-configs-utils \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  build-essential cmake python3-colcon-common-extensions \
  python3-pip python3-yaml python3-numpy liburdfdom-tools psmisc util-linux

python3 -m pip install --user 'motorbridge==0.4.9'
```

上面给出本版实测的 motorbridge 版本。做 MuJoCo 仿真时，在首次构建前用同一 Python 解释器安装本版验证过的版本。见 [motorbridge](https://pypi.org/project/motorbridge/0.4.9/) 和 [MuJoCo](https://pypi.org/project/mujoco/3.17.0/)。

```bash
python3 -m pip install --user 'mujoco==3.17.0'
```

重力辨识和重新生成物理场景的碰撞凸包需要 `python3-scipy`，绘图工具需要 `python3-matplotlib`。
`./scripts/install_deps.sh core control moveit` 可检查各组 ROS 软件包。

### 构建和首次运行

在检出目录执行：

```bash
./scripts/build.sh
./scripts/verify.sh
./scripts/start_target_web.sh
```

打开本地控制台即可看模型。需要 ROS CLI、Python SDK 或离线回归时，在同一检出目录的新终端加载环境。见 [本地控制台](http://127.0.0.1:8765/)。

```bash
source scripts/ros_env.sh && b601_source_workspace
./scripts/preflight.sh
```

脚本优先查找 `B601_ROS_SETUP`，其次使用已加载的 Humble 环境，或从系统 ROS 软件包找到 `setup.bash`。相对路径从检出目录解析。移动检出后，在新位置重新构建。

### 实机串口和波特率

在工程配置的 `hardware.dm` 下修改 `device` 和 `baudrate`。默认设置如下。见 [统一配置](ros2_ws/src/b601_description/config/b601_project.yaml#L250)。

```yaml
hardware:
  dm:
    device: /dev/ttyACM0
    baudrate: 921600
```

用 `ls -l /dev/serial/by-id/` 查看当前 USB-CAN 设备。有对应条目时，可把 `device` 设置为完整的 `/dev/serial/by-id/...` 路径，以免换 USB 接口后 `ttyACM*` 编号变动。`baudrate` 是电脑到 USB-CAN 桥的串口波特率，需与桥接器设置一致。确认当前用户可读写设备。若设备归属 `dialout` 且当前账号不在该组，可运行 `sudo usermod -aG dialout "$USER"` 后重新登录。

```bash
./scripts/sync_config.py
./scripts/build.sh
./scripts/dm_probe.py
```

探针默认只读。核对反馈后再启动网页实机连接或 MIT 快速启动。两种入口统一读取这个配置，归中停机工具也用该串口。直接调用 `bringup.launch.py` 时，`dm_device:=...` 只覆盖本次的串口设备；波特率仍从配置读取。网页的 `port:=8766` 用于更改 HTTP 监听端口。

真机测试前固定机械臂，清空扫掠范围和夹爪，确认现场可立即急停。在归中位置开始。使能前完成探针回读、只读检查和方向核对。同一串口只运行一个控制程序。
真机夹爪启动位置允许在行程边界外 1 mm 内。边界处理期间 ROS 位置反馈显示相应的端点，进入行程后继续报告实测位置。
常规收工先核对归中再失能。触发急停时由安全节点立即停止。

## 统一配置

工程参数集中在 `ros2_ws/src/b601_description/config/b601_project.yaml`。该文件管理关节与电机参数、
控制和安全门槛、URDF 形状与惯量、MoveIt 规划、仿真场景、演示节点和测试工具。ROS 运行时参数和
脚本命令行选项可覆盖对应默认值。

固定增益、逐轴重力前馈及辨识模型来自 `controllers.mit`。停放位、J4 预置、分段等待、
碰撞检查默认值及归中容差来自 `recenter`。控制器 YAML 和安全配置由该文件生成。

整机运动约束由 `joints.min_pos/max_pos`、`joints.max_velocity`、`joints.max_motor_torque_nm` 和
`target_slew` 提供。手臂速度上限为 6.0 rad/s，夹爪为 0.05 m/s。位置目标的斜率分别为
0.5 rad/s 和 0.5 m/s。MIT 的总输出包括 PD 和前馈。关节端 MIT 在下发前用最新反馈估算
总力矩，电机仍按内部高速闭环计算。温度、总线故障和反馈失效继续触发停机保护。
力矩上限统一填写电机侧 N·m。URDF 的直线关节 `effort` 由 xacro 按传动比换算。

```bash
./scripts/sync_config.py           # 同步生成的 ROS 参数文件和 SRDF 命名姿态
./scripts/sync_config.py --check   # 检查派生文件同步状态
```

修改模型或仿真场景后，先构建并加载工作空间，再生成所需的模型。单独生成物理场景需要 `python3-scipy`。

```bash
./scripts/build.sh
source scripts/ros_env.sh && b601_source_workspace
model="$(mktemp)"
if python3 scripts/gen_mjcf.py > "$model"; then
  cp "$model" ros2_ws/src/b601_mujoco/models/b601_arm.xml
fi
if python3 scripts/gen_mjcf.py --physical-collisions > "$model"; then
  cp "$model" ros2_ws/src/b601_mujoco/models/b601_arm_physical.xml
fi
rm "$model"
./scripts/build.sh
```

位置回路使用 `./scripts/dm_park.sh` 核对归中并失能，`--keep-enabled` 用于只归中核对。
正常停机在实测归中通过后依次停用活动控制器、确认硬件进入 `inactive`、退出 `controller_manager`，最后回读电机状态。
位置回路和关节端 MIT 都使用 `recenter.joints` 的校准停放位。MIT 归中使用逐轴容差与整机关节限位的交集。J2 的实际归中范围为 −0.02 至 0 rad，J3 容差为 ±0.03 rad，J4 为 ±0.02 rad，J1、J5 和 J6 为 ±0.01 rad。

### 电机侧 MIT 归中

实机以 `hardware:=dm dm_enable_motors:=true dm_command_mode:=mit_impedance` 启动时，关节状态广播器和手臂控制器激活后，`b601_mit_recenter` 会等待实测反馈和安全状态，然后自动归中一次。使能前须确认现场可运动。上层在日志出现 `启动归中成功` 后才发送第一个目标。只读启动不执行归中。
默认归中使用关节反馈和控制配置。显式启用 `mit_recenter_check_collisions:=true` 时，应用层归中节点会请求 MoveIt 进行碰撞检查。

归中时，节点先根据反馈判断 J4。
J4 实测高于 −0.10 rad 时，先向负方向移动至少 0.10 rad，并到达 −0.10 rad 或更负；已经达到预置角时保持原角度。
随后 J1、J2、J3、J5、J6 归中，最后 J4 回落。六轴实测到位后返回成功。归中前，先让上一位目标发布者结束发令。

从终端调用时，`ROS_DOMAIN_ID` 和 `ROS_LOCALHOST_ONLY` 要与运行中的后端一致。`start_mit_gravity.sh` 默认使用域 211。网页启动后端时沿用网页终端的 ROS 域。网页托管的实机可在页面先「一键归中」，到位后点「Disable」完成停机并恢复预览。

```bash
# 只归中并保持电机使能
./scripts/dm_park.sh --mit --keep-enabled

# 归中成功后失能
./scripts/dm_park.sh --mit

# 从其他 ROS 2 节点或命令行单独触发归中
ros2 service call /b601_mit_recenter/recenter std_srvs/srv/Trigger '{}'
```

## 电机侧 MIT

手臂只有三套现行增益，统一维护在 `b601_project.yaml` 的 `controllers.mit`。夹爪共用一套纯 PD。

| 使用阶段 | J1–J6 Kp | J1–J6 Kd |
|---|---|---|
| 无重力补偿纯 PD | 115 / 265 / 190 / 200 / 36 / 28 | 5 / 5 / 5 / 3.4 / 1.2 / 0.82 |
| 重力补偿高刚度跟踪 | 115 / 120.025 / 120.025 / 30 / 18 / 28 | 5 / 5 / 5 / 2.8 / 1.6 / 0.65 |
| 重力补偿低刚度保持 | 57.5 / 60.0125 / 60.0125 / 15 / 9 / 14 | 1 / 1 / 1 / 0.56 / 0.32 / 0.13 |

`mit_gravity_compensation` 默认关闭，使用纯 PD。启用软硬刚度切换时从高刚度起步，首个目标完成且六轴连续到位 3 秒后降档，收到新目标再升档。每轴按相邻目标位移的 5% 计算到位误差带，上限为 0.05 rad。重力补偿档的下限为 J3 0.03 rad、J4 和 J5 0.02 rad、其余轴 0.01 rad。纯 PD 的六轴下限均为 0.01 rad。低档逐周期采用实测位置作为位置参考。重力补偿配置的两档共用实机辨识重力模型，高档沿用 J2–J5 的前馈比例，低档保留全轴重力比例。低档默认使用高档的 50% Kp 和 20% Kd。

每次启动可用 `mit_stiffness_switching:=true` 开启软硬刚度切换，或用 `mit_stiffness_switching:=false` 保持高刚度。`auto` 沿用所选控制器配置。启动日志显示实际选择；重力补偿仍由后端和 `mit_gravity_compensation` 决定。

### 快速启动重力补偿

先核对 [实机串口和波特率](#实机串口和波特率)。ROS 域和控制选项在 [MIT 启动脚本](scripts/start_mit_gravity.sh) 顶部配置。默认使用真机电机侧 MIT、重力补偿和 `ROS_DOMAIN_ID=211`。默认会使能电机并自动归中，现场确认后再执行。只读检查时，将 `DM_ENABLE_MOTORS` 改为 `false`。

脚本顶部的 `MIT_STIFFNESS_SWITCHING` 默认 `false`，重力补偿运行于高刚度档。

```bash
./scripts/start_mit_gravity.sh
```

保持启动终端运行。看到 `启动归中成功` 后，在第二个终端进入同一检出目录，加载工作区环境，按脚本顶部的值设置 ROS 域。下面向手臂发送一次已实测姿态，`position` 按 J1–J6 顺序填写绝对角度，单位为 rad。

```bash
source scripts/ros_env.sh && b601_source_workspace
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=211

python3 scripts/dm_mit_move.py --set \
  'joint1=0.36171661,joint2=-0.36209808,joint3=-0.37163483,joint4=-0.38040863,joint5=0.37049042,joint6=0.36133514'
ros2 topic echo /b601_safety/status --qos-durability transient_local
```

开启软硬刚度切换时，可在 `limit_profile` 变为 `low_stiffness` 后开始记录低档保持。收工时先退出状态查看，再从同一 ROS 域的第二个终端运行 `./scripts/dm_park.sh --mit`，完成实测归中、失能和回读。停放脚本成功后，在启动终端结束剩余节点。

### 安全档位与只读检查

驱动激活和两档急停共用 `safety.angle` 的位置边界，各端从实测端点向外扩展全行程的 5%。夹爪从动指的边界由主动指镜像。手臂的实测超速急停阈值为常规档 `6 rad/s`、低刚度档 `10 rad/s`。两档的夹爪急停阈值由 `safety.velocity.gripper_estop_m_s` 指定，主动指和从动指均为 `0.25 m/s`，速度警告从 `0.20 m/s` 开始。常规档超过急停阈值触发，低刚度档达到阈值即触发。夹爪的命令速度上限为 `0.05 m/s`。`/b601_safety/status` 的 `limit_profile` 显示当前速度档。

既有回读记录的 4310 `VMAX` 为 `30 rad/s`，按夹爪比例 `0.01 m/rad` 换算，反馈满量程约为 `0.30 m/s`。

以下命令只读取真机反馈。使能和运动需要现场确认。

```bash
ros2 launch b601_bringup bringup.launch.py hardware:=dm \
  dm_command_mode:=mit_impedance mit_gravity_compensation:=true \
  dm_enable_motors:=false use_rviz:=false
```

仿真回归使用相同高低增益，夹具的前馈系数由 MuJoCo 模型质量和质心计算。检查 5% 跟踪误差带、自动降档和低档 5 秒的关节变化。JSON、CSV 和启动日志存放在 `.verify/`。

```bash
source scripts/ros_env.sh && b601_source_workspace
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=214 python3 scripts/sim_mit_ros.py \
  --gravity-varstiff-switch-test --tag varstiff_switch
```

修改统一配置后运行 `./scripts/sync_config.py` 并重新构建。早期实验参数可从历史标签查看。

## 6+1 MIT 真机复测

碰撞复测默认使用随 `b601_description` 安装的 Seeed B601-DM URDF 和碰撞网格。历史实测使用同一上游提交。可通过 `B601_VENDOR_DESCRIPTION` 指定另一份厂商 `rebotarm_bringup` 目录。见 [模型来源](ros2_ws/src/b601_description/assets/b601_dm/README.md)。

```bash
./scripts/hw_mit_five_poses.sh --simulate
./scripts/hw_mit_five_poses.sh --probe
./scripts/hw_mit_five_poses.sh --readonly
```

仿真结果只用于评估控制器稳定性。真机复测以现场确认、探针和只读回读为准。
确认探针回读的七台电机处于 MIT 模式，方向已核对，机械臂与夹爪均处于配置的停放范围。
现场确认后执行。入口先检查厂商网格碰撞，再从小幅动作开始。

```bash
./scripts/hw_mit_five_poses.sh --execute
```

真机复测使用整机的关节速度限幅、总力矩限幅和目标斜率。
日志、CSV 和判定写入 `.verify/mit_hardware_five_poses/`。需要在试验中途停止发新目标并尝试安全归中时，
在输出目录创建 `HOLD_HARDWARE`；遇到危险立即使用现场急停。参数集中在
`ros2_ws/src/b601_description/config/b601_project.yaml`，改后执行 `./scripts/sync_config.py` 和构建。

v0.2.1 的纯 PD 真机五姿态记录可离线重算。

```bash
mkdir -p .verify
git show v0.2.1:validation/results.json > .verify/mit-v0.2.1-results.json
source scripts/ros_env.sh && b601_source_workspace
python3 scripts/mit_five_poses.py --score-hardware .verify/mit-v0.2.1-results.json
```

历史位置回路的复测入口见 [pos_vel 流程](validation/pos-vel.md)。

## MIT 目标仲裁与网页

上层架构、MuJoCo 双场景复测、实机覆盖和复现命令见 [上层框架验收](validation/target-web.md)。

MIT bringup 启动 `b601_target_arbiter` 和 `b601_target_bridge`，
根据实际加载的控制器配置建立内部路由。手动工具、分段归中、仿真与真机五姿态脚本、浏览器
和外部策略共用整机仲裁入口。同一时刻只授予一个整机会话，默认启动时没有来源持有控制权。
仲裁向各组发布 `/<控制器>/authorized_target`，转发节点是 `/<控制器>/joint_target`
的唯一正常发布者。转发节点从
`/<控制器>/state` 读取主线控制器的目标序号，在 `/<控制器>/target_applied` 回报采纳编号。
标准位置模式仍使用自己的 `FollowJointTrajectory` 接口。

### 在网页选择运行后端

```bash
./scripts/build.sh
./scripts/start_target_web.sh
```

打开 [本地目标控制台](http://127.0.0.1:8765/)，页面会默认加载随包安装的 B601-DM 官方 STL，显示默认姿态。脚本只启动网页和预览模型发布器，URDF 发布到 `/b601_preview/robot_description`，预览姿态来自项目配置。需要另一个端口时运行 `./scripts/start_target_web.sh port:=8766`。

在网页选择 MuJoCo 或实机后，「软硬刚度切换」默认关闭。连接前可选开启或按后端默认。连接后状态栏显示实际使用的设置及当前刚度。可用 `./scripts/start_target_web.sh mit_stiffness_switching:=true` 将网页首次选择预设为开启。

后端区域分别显示重力和摩擦补偿的实时来源。部分关节使用外部数据时显示「混合」并列出这些关节。外部值过期后，控制器回退内部计算，网页随回读更新。显式传入的零值仍标记为外部来源。

重复运行会显示已有控制台的地址并返回终端。要退出控制台，在最初启动它的终端按 `Ctrl+C`。端口被其他服务占用时，启动命令会提示选择空闲端口。

需要关节反馈时，在「运行后端」选择模式，再点击「连接」。启动终端与目标客户端须使用相同的 ROS 域。网页收到后端的模型和关节反馈后切换到实测姿态。页面连接实机后，收工先按下文完成归中和 Disable，再结束启动脚本。

- **Mock** 是网页的空操作，页面保持空闲。
- **MuJoCo** 启动 MIT 控制器，默认关闭重力和物理碰撞。需要模拟重力时，连接前在
  「MuJoCo 物理模式」选择「开启重力与碰撞」，同时启用地面接触和机械臂自碰撞。
  内置物理场景使用专用 MIT 配置，手臂按模型质量和质心做重力前馈，默认保持高刚度跟踪。
- **实机** 使用项目串口与 MIT 重力补偿变刚度。点击连接会使能电机，请按项目规程先确认现场。
  连接只启动底层控制并以进入时的实测位置作为初始参考。上层目标和自动归中需要独立操作。
  关闭浏览器不会停止 bringup。

实机或 MuJoCo 连接期间，后端选择锁定。点击「一键归中」，等六轴实测到位后点击「Disable」。
页面确认整机已失能并退出当前 bringup 后，可以手动选择下一个后端。
网页右上角显示驱动回读的刚度档位。低档确认前显示高刚度，控制器未激活或反馈中断时显示刚度待回读。

从命令行启动 `hardware:=mujoco` 也默认同时关闭重力和物理碰撞，传入
`mujoco_scene:=physical` 可同时开启。回归夹具仍可通过 `mujoco_model` 指向自定义模型。
物理场景独立加载 `b601_arm_physical.xml`，其中 `link1`、`link3`、`link5` 的接触几何使用随包的 Seeed 碰撞 STL 凸包。
物理场景的一键归中使用 `recenter.joints` 的标定停放位。六轴实测到位后才允许 Disable。

连接就绪后，可手动指定权威来源及发送目标。也可通过 `use_target_web:=true`
随既有 MIT bringup 启动同一页面，此时页面显示该后端。

### MuJoCo 网页回归夹具

```bash
source scripts/ros_env.sh && b601_source_workspace
python3 scripts/test_target_arbitration.py --web
```

运行 `python3 scripts/test_target_arbitration.py` 执行隔离仲裁回归。加 `--physical` 验证物理场景两个起始姿态的归中与失能门控。详见 [复测步骤](validation/target-web.md#复测)。

浏览器打开 [本地目标控制台](http://127.0.0.1:8765/)，可以指定网页、外部策略或其他来源名。
指定 `web` 后，可用滑块或数值框编辑同一列表中的七个关节，点击「发送七关节目标」提交。
页面同时显示指定来源、当前会话持有者、实时反馈和整机回执。
三维视图使用运行中的 URDF 和关节反馈。实测模型保留 B601 的外观配色，青色模型显示
最近下发的目标或待发送预览，视图下方显示两者的末端 XYZ 坐标。拖动旋转，滚轮缩放，
「复位视角」回到初始视角。网页默认使用工作区内的官方 STL，关节树和姿态来自项目 URDF 与关节反馈。可用 `B601_VENDOR_DESCRIPTION` 选择另一份厂商目录。见 [随包模型资源](ros2_ws/src/b601_description/assets/b601_dm/README.md)。
宽窗口并排展示模型与关节控制，窄窗口滚动关节列表时，模型固定在屏幕上方。
在三维视图点选 TCP，按住 Ctrl 沿当前视角平面拖动。页面按 URDF 逆解末端位置并填入同一列表，
保留第七个关节的目标值。青色模型显示待发送预览，点击同一个「发送七关节目标」按钮执行。
指定外部来源后由相应客户端申请和发送目标。点击「释放控制权」清除指定来源及现有会话。
用上述 MuJoCo 夹具时，在网页指定 `manual`，再从第二个终端发送 J1 目标。其他关节由客户端用当前反馈补齐。

```bash
source scripts/ros_env.sh && b601_source_workspace
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1
python3 scripts/dm_mit_move.py --set 'joint1=0.035'
```
点击「一键归中」会调用现有 ROS 归中服务并显示执行结果。当前网页持有控制权时，它会先交还会话。
若当前由外部来源持有或指定给外部来源，先在控制台释放该来源，再点击归中。
归中沿用现有的 J4 上抬、其余轴归中和 J4 回落流程。三维目标模型跟随各段实际下发的目标，
执行结束后恢复未发送的草稿预览。列表中的第七关节显示为 `gripper`。
退出前释放控制权。这个入口启动本地零重力 MuJoCo 夹具，夹爪使用仿真测试增益，整机安全节点仍启用。
服务只在本机回环地址监听。常规 MIT bringup 可通过 `use_target_web:=true` 启动同一页面。

### ROS 2 接口

| 端点（前缀 `/b601_target_arbiter`） | 类型 | 用途 |
|---|---|---|
| `/set_authority` | `b601_target_interfaces/srv/SetAuthority` | 指定整机权威的来源名。切换时撤销旧会话，传空字符串清除指定并释放当前会话 |
| `/select_source` | `b601_target_interfaces/srv/SelectSource` | 以来源名申请整机会话，成功返回私有 `session_id` 和公开 `session` |
| `/release_source` | `b601_target_interfaces/srv/ReleaseSource` | 持有者释放整机会话，已指定的来源可以再次申请 |
| `/release_web_for_recenter` | `b601_target_interfaces/srv/ReleaseSource` | 网页凭当前私有会话令牌交还自身会话和网页来源指定 |
| `/revoke_source` | `b601_target_interfaces/srv/RevokeSource` | 操作者按公开会话编号撤销指定会话 |
| `/renew_source` | `b601_target_interfaces/srv/RenewSource` | 维持现有会话，接受新目标也会续期 |
| `/set_target` | `b601_target_interfaces/srv/SetTarget` | 接收完整七关节目标，返回仲裁结果和转发编号 |
| `/set_mit_target` | `b601_target_interfaces/srv/SetMitTarget` | 接收同帧的目标及可选外部重力、摩擦前馈 |
| `/source_heartbeat` | `std_msgs/msg/String` | 低开销续期，`data` 为私有 `session_id` |
| `/target` | `b601_target_interfaces/msg/TargetFrame` | 流式提交七关节目标，按会话内序号递增 |
| `/mit_target` | `b601_target_interfaces/msg/MitTargetFrame` | 流式提交带外部补偿的目标，与 `/target` 共用序列号 |
| `/status` | `std_msgs/msg/String` | JSON，包含指定来源、当前来源、公开会话号、最近七关节目标、整机采纳编号和掉线持位状态 |

未指定来源时，第一个申请成功的客户端持有整机控制权。手动指定后，只有名称匹配的客户端可以申请。
每帧包含七个关节名和位置，可同时提交七个速度和七个任务前馈。原生 ROS 2 使用
`TargetFrame.target` 中的 `name`、`position`、`velocity` 和 `effort`，其中 `effort` 承载 `tff`。

| 目标量 | `joint1` 至 `joint6` | `finger_left` |
|---|---|---|
| 位置 | rad | m |
| 速度 | rad/s | m/s |
| `tff` | N·m | N |
| `gravity_ff`、`friction_ff` | N·m | N |

每帧省略 `tff` 时，任务前馈立即归零。MIT 前馈等于重力补偿、摩擦补偿和任务前馈之和。
原 `~/task_torque` 的发送方可迁移到同一目标帧的 `JointState.effort`，由整机会话与序号统一仲裁。
非零任务前馈在最近一次目标采纳后默认有效 0.5 秒。只发会话心跳时，该项仍按时归零。
SDK 支持局部编辑，首次用实测反馈补齐七关节，后续沿用同一会话已发出的其它位置。
`set_joints(..., velocities={...}, tff={...})` 按名称传入可选量，省略的关节取零。
会话令牌用于命令和释放，状态话题只公布独立的公开编号。`last_forwarded` 包含已转发的 `position`、`velocity`、`tff`、`gravity_ff` 和 `friction_ff`，可用 `last_applied_id` 对照两组控制器的采纳编号。ROS 域内的应用应部署为可信节点，
若需隔离未经授权的参与者，由部署端设置 DDS/SROS2 发布与服务权限。

### 相对关节目标

上层 SDK 或 HTTP 网关读取各自最新且完整的 `/joint_states`，按关节计算 `q_target = q_measured + Δq`。未指定关节的增量按零处理。`joint1` 到 `joint6` 用 rad，`finger_left` 用 m。仲裁只接收七关节绝对目标，判定会话权威、校验并转发。

网页已连接后端且控制权空闲时，可按 J1–J6、主动夹爪的顺序在终端输入七个增量。下面发送一次 J1 `+0.03 rad`，脚本会输出合成后的目标和转发编号，并释放会话。网页使用其它端口时可传 `--port`。

```bash
./scripts/send_joint_deltas.py 0.03 0 0 0 0 0 0
```

已经取得整机会话的 Python `TargetClient` 可以按名称点发或流式发送增量。

```python
dispatch_id = client.set_joint_deltas({"joint2": -0.02, "joint4": 0.03}, timeout=3.0)
```

流式客户端可调用 `client.publish_joint_deltas(deltas)`。原生 ROS 2 客户端先订阅 `/joint_states` 并自行换算，再使用上表的绝对目标接口。每个新帧都以当时的实测位置起算。连续发送同一非零增量会继续生成新的目标。SDK 和 HTTP 的增量快捷入口使用默认速度参考与内部补偿配置。

HTTP `POST /api/delta_target` 接收 `session_id`、递增的 `sequence` 和按关节名索引的 `deltas`。成功响应包含 `dispatch_id` 和合成后的 `positions`。反馈不齐或过期时上层拒绝计算。绝对目标沿用既有位置范围校验。

### 外部重力与摩擦前馈

`gravity_ff` 和 `friction_ff` 各自接受按关节名索引的最终补偿量，手臂用 N·m，主动夹爪用 N。每个关节分别选择外部重力、摩擦或对应的内部项，再加 `tff`。显式传 `0.0` 会把该项设为零。省略字段或关节时立即使用内部项。停止发送目标后，覆盖与 `tff` 共用现有的 `task_torque_timeout_sec`，默认 0.5 秒。持续计算时应按新鲜实测位姿更新前馈，随目标帧发送。缩放、偏置及内部摩擦模型参数作用于回退使用的内部项。

SDK 的 `TargetClient.set_joints()` 和 `publish()` 都接受 `gravity_ff={...}`、`friction_ff={...}`。HTTP `POST /api/target` 同样接收两个可选的关节字典。原生 ROS 2 使用 `MitTargetFrame.command` 的 `target`（`JointState`）和两个 `JointFeedforward[]` 字段。每个 `JointFeedforward` 条目包含 `joint` 和 `effort`。

同级目录 `gravity_identification/` 的传统模型返回 J2–J5 的完整保持力矩。当前共享神经网络的 `predict()` 返回 `FULL_JOINTS` 顺序的七路完整保持量，包含 URDF 标称项、负载项及 J2–J5 的学习修正。`predict_compensation()` 只返回学习修正与偏置。传入 `gravity_ff` 时应使用完整保持量。只想覆盖经过辨识的关节时，只发送 J2–J5，其余关节沿用内部计算。动态摩擦由外部模型按速度求出，再传入对应的 `friction_ff`。

```python
from gravity_basis import INPUT_JOINTS, OUTPUT_JOINTS

# model 是已加载的传统模型，client 已取得整机会话。
q = [measured_positions[name] for name in INPUT_JOINTS]
gravity_ff = dict(zip(OUTPUT_JOINTS, model.predict(q)[0].tolist()))
dispatch_id = client.set_joints(positions, 3.0, gravity_ff=gravity_ff)
```

`/b601_mit_arm/ff` 和 `/b601_mit_gripper/ff` 按摩擦、重力、任务三段回读实际采用的值。两条原生 ROS 2 旧入口 `/target` 与 `/set_target` 仍支持位置、速度和任务前馈。

各控制器的 `~/compensation_sources` 按关节回读实际选用的来源。HTTP `/api/state` 的 `compensation_sources` 提供 `gravity` 和 `friction` 两个关节字典，取值为 `internal` 或 `external`。等待可用回读时返回 `null`。

### 客户端示例

下面的调用可在上述 MuJoCo 预览的第二个终端运行。默认预览使用 `ROS_DOMAIN_ID=227`。
会话在约 0.5 秒内需要持续收到心跳或新目标，建议心跳间隔 0.1 秒。CLI 调用若分开手动输入，
间隔可能超时，下面示例从程序取得令牌并用 SDK 定时续期。相同服务也可用任意 ROS 2 客户端调用。

```bash
source scripts/ros_env.sh && b601_source_workspace
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1

python3 - <<'PYTHON'
import rclpy
from b601_demos.target_client import TargetClient

rclpy.init()
node = rclpy.create_node("external_target_example")
client = TargetClient(node, "external")
try:
    client.select(timeout=3.0)
    target = dict(zip(client.joints, [0.04, -0.3, -0.6, 0.03, 0.0, 0.0, 0.015]))
    print(client.set_joints(target, timeout=3.0))
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.02)
finally:
    client.close(timeout=3.0)
    node.destroy_node()
    rclpy.try_shutdown()
PYTHON

ros2 topic echo /b601_target_arbiter/status --qos-durability transient_local --once
```

流式策略用 `TargetClient(node, "policy")` 申请控制权，按策略频率调用
`client.publish(JointState(name=[...], position=[...], velocity=[...], effort=[...]))`，并保持节点的 ROS executor
运行以发送定时心跳。ROS 2 原生客户端可订阅状态，调用 `select_source`，
发布带私有会话号和递增序号的 `TargetFrame`，也可用 `set_target` 取得转发编号。
`select_source` 的状态公开编号可与 `revoke_source` 一起使用。

按 `tools.target_arbitration.authority_timeout_sec=0.5`，当前来源失联或仲裁链路断流时，
转发节点把新鲜的实测位置作为普通目标送入 MIT 控制器。控制器按主线的目标斜坡和刚度策略执行。
恢复发送前需重新申请，先前会话的目标即使迟到也会被拒绝。
完成任务后主动释放时，控制器继续执行已接受的目标。关闭还持有控制权的网页会触发断流持位。
转发节点通过各组的 `target_holding` 和 `target_applied` 报告状态。断流持位依赖转发进程、
ROS 通信和新鲜反馈。使用主线自带的刚度切换时，持位目标仍可按正常策略进入低刚度。

### HTTP 与命令行

本地网页可逐关节填写速度和任务前馈。HTTP `POST /api/target` 接收以下 JSON，
`session_id` 由 `POST /api/select` 返回。未列出的速度和任务前馈取零。

```json
{
  "session_id": "<session_id>",
  "sequence": 1,
  "positions": {
    "joint1": 0.04, "joint2": -0.3, "joint3": -0.6, "joint4": 0.03,
    "joint5": 0.0, "joint6": 0.0, "finger_left": 0.015
  },
  "velocities": {"joint1": 0.01},
  "tff": {"joint1": 0.25, "finger_left": 0.1}
}
```

在 MuJoCo 中用命令行发同类目标：

```bash
python3 scripts/dm_mit_move.py --set 'joint1=0.04,finger_left=0.015' \
  --vel 'joint1=0.01' --tff 'joint1=0.25,finger_left=0.1'
```

### 复测

```bash
source scripts/ros_env.sh && b601_source_workspace
python3 scripts/test_target_arbitration.py --http-port 8876
python3 scripts/test_target_arbitration.py --physical --http-port 8876
```

这个脚本会自动启动和关闭隔离 MuJoCo，验证转发节点的唯一输出、整机独占会话、七关节目标、
单关节和多关节编辑后的完整下发、控制器采纳回执、旧会话拒绝、策略流、归中交接、撤销、来源失联持位以及仲裁进程断流后的重新接管。
日志存于 `.verify/target-arbitration-227/`。完成网页预览后再运行自动验收，
或用 `B601_TEST_ROS_DOMAIN_ID` 选另一空闲 ROS 域。

## 文档与运行产物

仓库保留本使用说明、发布记录与可复测的验收入口。设计、试验日志和历史原始回读在本地维护。
构建和网页日志默认位于 `ros2_ws/log/`，回归输出位于 `.verify/`。旧版原始验收文件可由相应 Git 标签读取。

## 许可证

框架原创代码及文档以 MIT 许可证发布。见 [MIT 许可证](LICENSE)。

- 随包提供的 Seeed B601-DM 官方 URDF 和 STL 使用上游声明的 Apache-2.0。见 [模型来源与许可](ros2_ws/src/b601_description/assets/b601_dm/README.md)。
- 网页使用的 three.js 及其控件和加载器按 MIT 许可分发。见 [three.js 原版权声明与许可证](ros2_ws/src/b601_demos/web/vendor/THREE-LICENSE.txt)。
