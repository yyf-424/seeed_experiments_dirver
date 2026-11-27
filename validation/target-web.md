# 上层框架验收

本页记录 `B601_experiments` 的隔离 MuJoCo 回归、实机日志复算和各版本的发布复测。原始日志留在执行测试时的本机检出目录，链接指向本地 `.verify/`。安装和使用流程见 [README](../README.md)。

## TCP 点动与 MIT 定时轨迹联调

`codex/tcp-primitives` 以实验主工作区 `7a0c209` 为基线。独立 MoveIt 支持 MIT 轨迹执行，网页托管的 MoveIt 以规划模式服务 TCP 原语。两种入口均经过同一整机目标仲裁。

- MuJoCo 通过 CLI 与网页的基座六向平移、三轴旋转、连续参考、采纳回执及外部重力切换。 [TCP 回归](../.verify/rebase-main-7a0c209/tcp.log)
- 默认 MIT 的 MoveIt Plan & Execute、手臂与夹爪联合轨迹、路径容差、取消及撤销后持位通过。Mock 的显式 JTC 位置模式也通过 MoveIt 执行。 [MIT 回归](../.verify/rebase-main-7a0c209/mit.log) · [JTC 回归](../.verify/rebase-main-7a0c209/jtc.log)
- 同一条网页托管 MuJoCo 链路在 J3 Ki=38 时跑通点动和积分回读，释放控制权后积分归零，随后接续标准 MIT action、归中及失能。仿真使用物理场景的 Kp、Kd。 [联调日志](../.verify/rebase-main-7a0c209/managed.log) · [实机参数](experiments/2026-11-27-tcp-ki38.yaml)
- 10 包构建成功，静态自检 34/34、离线回归 21/21，标准仲裁和夹爪负载估计回归通过。 [离线回归](../.verify/rebase-main-7a0c209/preflight.log) · [仲裁回归](../.verify/rebase-main-7a0c209/arbiter.log) · [负载复测](../.verify/rebase-main-7a0c209/load.log)

首次定时轨迹检查在控制器管理器查询服务上超时。复测等待控制器激活并核对实测反馈后完成轨迹测试。MoveIt 退出时复现主工作区已有的段错误。 [首次启动记录](../.verify/rebase-main-7a0c209/launch-mit-startup-race.log) · [复测启动与退出](../.verify/rebase-main-7a0c209/launch-mit.log)

### MIT 任务积分与 J3 Ki=38 实机记录

六轴积分启用后，12 向实机复测中，66 个有效关节与动作组合有 23 个在末 3 秒每个样本均达到 5%。J2 在绕 Y 正转时达到积分幅值后仍有偏差，两个小关节增量受反馈编码档距限制。 [六轴实测](../.verify/mit-integral-six-axes/report.md)

固定 J3 Kp 为 132.32756，Ki 从 33.082 调到 38。相同四组动作各重复两次，J3 末 3 秒 RMSE 从 0.121° 降至 0.111°。向上 5 mm 两次的相对误差从 8.95% 和 12.82% 降至 3.14% 和 3.14%。绕 X 正转最大过冲从 0.096° 增至 0.205°。 [参数配置](experiments/2026-11-27-tcp-ki38.yaml) · [对照报告](../.verify/mit-j3-pi-refinement/report.md)

另一轮 Ki=38 手动测试记录 24 次点动，含 16 次平移和 8 次旋转。TCP 回执的三维绝对误差 RMSE 为 4.38 mm，最大 7.79 mm。按基座 X/Y/Z 的最大绝对分量为 1.85/0.75/7.78 mm。相邻命令间隔约 0.04–1.89 秒，这些数据描述回执时的误差。 [逐次结果](../.verify/mit-integral-six-axes/manual-ki38-20261127-184525/comparison.json)

## MIT 定时轨迹复测

本节对应 `B601_experiments/main` 的功能提交 `ca73e33`。可从 [启动与停机说明](../README.md#mit-定时轨迹) 进入复测。实机原始 CSV、轨迹 JSON 和曲线保存在本机工作区的 `.verify/`。

在隔离 ROS 域以 `ros2 launch b601_moveit_config demo.launch.py use_rviz:=false` 启动默认 MuJoCo 物理场景，在同一环境运行 `python3 scripts/test_mit_trajectory.py`。检查仅加载 MIT 控制器和反馈广播器，手臂控制器报告重力补偿已开启。

- 通过手臂与夹爪同目标的两点定时轨迹、实测反馈和末帧采纳。
- 通过竞争来源拒绝、执行中取消、操作者撤销控制权和双组持位确认。带显式路径容差的偏差目标返回 `PATH_TOLERANCE_VIOLATED`。
- MoveIt Plan & Execute 通过 12 点 MIT 轨迹。显式 mock 位置模式通过 12 点 JTC 轨迹。
- 原有 MuJoCo MIT 后端就绪后，单独启动 `move_group.launch.py hardware:=mujoco use_rviz:=false`，重复上述 MIT 轨迹检查并通过。首次并行启动时下层控制器管理器出现 DDS 服务响应失败，按顺序启动后通过。

仿真退出时 `move_group` 复现既有段错误，MIT 轨迹执行器正常退出。

### B601-DM 实机双轴往返

2026-11-27 在主工作区使用 `hardware:=dm dm_command_mode:=mit_impedance mit_gravity_compensation:=true mit_stiffness_switching:=true`。现场确认后，从 `FollowJointTrajectory` 发送 4 秒四点轨迹，J1 和 J3 相对实测起点的最大参考位移为 +0.040 和 −0.035 rad。手臂回读内置重力开启，两路末帧采纳一致，action 返回 `SUCCESSFUL`，共收到 194 帧反馈。

| 关节 | 实测最大位移幅值 | 跟踪 RMSE | 沿途峰值误差 | 终点绝对误差 |
|---|---:|---:|---:|---:|
| J1 | 0.03586 rad | 0.00467 rad | 0.00717 rad | 0.00458 rad |
| J3 | 0.01411 rad | 0.01504 rad | 0.02538 rad | 0.01106 rad |

驱动 CSV 在两轴都记录到完整参考幅值。J3 实测最大位移约为参考的 40%。本次未给 action 指定沿途误差容差。试验期间仅有偏离教学启动位形告警，收尾通过六轴实测归中，硬件确认进入 `inactive`，串口已释放。 [原始 action 反馈](../.verify/mit-trajectory-real/real-action-feedback.csv) · [驱动参考与反馈](../.verify/mit-trajectory-real/driver-feedback.csv) · [归中停机](../.verify/mit-trajectory-real/main-park.log)

### B601-DM 实机六轴往返

2026-11-27 沿用主工作区内置重力补偿与变刚度。以启动归中后的实测关节位置为起点，六轴在 0、1.5、3.0、5.0 秒分别走零位移、半幅、全幅和零位移。J1–J6 全幅依次为 `[+40, −30, −35, −30, +30, +35]` mrad，夹爪保持起始开度。各轨迹点速度和加速度设为零，段间使用五次插值。

| 关节 | 实测峰值位移（mrad） | 沿途 RMSE（mrad） | 沿途峰值误差（mrad） | 终点绝对误差（mrad） |
|---|---:|---:|---:|---:|
| J1 | +35.10 | 4.57 | 7.12 | 4.20 |
| J2 | −26.70 | 6.13 | 12.97 | 12.97 |
| J3 | −13.35 | 14.85 | 24.98 | 11.06 |
| J4 | −20.98 | 7.13 | 13.86 | 7.63 |
| J5 | +19.84 | 7.31 | 12.51 | 4.20 |
| J6 | +27.08 | 4.94 | 8.53 | 0.38 |

收到 333 帧 action 反馈，驱动记录显示六轴参考均达到设定幅值，夹爪驱动参考保持恒定，实测漂移最大 0.0038 mm。末段等待 2 秒后，J2 终点误差仍超过配置容差 10 mrad，action 返回 `GOAL_TOLERANCE_VIOLATED`。J3 实测幅值约为目标的 38%。全程未触发急停。随后六轴按归中服务到位，硬件确认 `inactive`，串口已释放。 [实际轨迹](../.verify/mit-trajectory-real-six/real-goal.json) · [跟踪曲线](../.verify/mit-trajectory-real-six/trajectory-tracking.png) · [驱动参考与反馈](../.verify/mit-trajectory-real-six/driver-feedback.csv) · [归中停机](../.verify/mit-trajectory-real-six/main-park.log)

## 节点和数据流

| 接口 | 职责 |
|---|---|
| `scripts/start_target_web.sh` | 启动本地网页及命名空间 `/b601_preview` 下的模型发布器。独立启动时展示默认姿态，可指定重力来源。显式指定 `load_api` 时启动负载节点。同配置复用已有控制台 |
| `scripts/send_joint_deltas.py` | 按 J1–J6、主动夹爪的顺序点发七轴增量，通过网页 HTTP 换算绝对目标并回读转发编号 |
| `b601_target_web` | 展示 URDF、实测和目标姿态，提供 TCP 平移与原地旋转、控制权、后端和刚度选择、补偿来源回读、归中与 Disable |
| `b601_target_arbiter` | 整机只授予一个有效会话。确认会话、递增序号、有限值及关节限位后，向手臂和夹爪分别发布授权目标 |
| `b601_target_bridge` | 调用现有 MIT 控制器的 `joint_target` 入口，按控制器消费序号匹配回执。来源失联或仲裁心跳中断时，以新鲜实测位置提交持位目标 |
| `b601_tcp_primitives` | 规划基座坐标系 TCP 平移与旋转序列，显式执行时经同一 MIT 仲裁会话逐段发送并回读 |
| `b601_mit_trajectory` | 接收标准 `FollowJointTrajectory`，按时序通过整机仲裁发给两组 MIT 控制器，核对末帧采纳与实测终点 |
| `b601_mit_recenter` | 通过整机仲裁接管，按 J4 上抬、其余五轴归中、J4 回落执行，依据六轴实测及控制器回执完成归中 |

客户端通过同一整机会话提交目标。`TargetFrame` 接受七关节位置、可选速度和任务前馈。`MitTargetFrame` 另可携带外部重力和摩擦补偿。控制器的采纳回执用于确认目标已经消费，位置误差和到位窗口用于确认运动完成。网页 Disable 需要最近一次归中回执及六轴仍在停放范围。执行期间仲裁会冻结新的会话申请，网页确认硬件进入 `inactive` 后退出后端。

## B601_experiments 扩展验收

### 重力与 TCP 同会话复测

`codex/tcp-primitives` 基于主工作区提交 `70daca2`。网页依次显示重力补偿、控制权、`base_link` 六向 TCP 点动与日志、逐关节指令。使用独立 DDS 域和本地 HTTP 端口运行以下回归。

```bash
source scripts/ros_env.sh && b601_source_workspace
B601_TEST_ROS_DOMAIN_ID=169 python3 scripts/test_target_arbitration.py --tcp-primitives --http-port 8892
B601_TEST_ROS_DOMAIN_ID=170 python3 scripts/test_target_arbitration.py --http-port 8893
B601_TEST_ROS_DOMAIN_ID=172 python3 scripts/test_target_arbitration.py --physical --http-port 8895
python3 scripts/test_load_feedback.py --domain 171
```

同一网页会话中，切换外部重力再点动，控制器的 J1 补偿仍回读为外部。切回内部后，七关节目标由两组控制器采纳，下一次 TCP 点动重新读取实测参考。日志入口显示当次 JSONL 路径，点动回执可在该文件内按转发编号查到。

Chromium 通过网页连接物理 MuJoCo，由网页启动 MIT 控制器和 MoveIt。实际点击重力切换、向左点动 3 mm、读取夹爪外力并发送七关节目标，随后归中、失能、回到预览。 [连接后的网页](../.verify/merge-review/live-browser-complete/connected.png) · [浏览器结果](../.verify/merge-review/live-browser-complete/browser-result.json)

默认与物理 MuJoCo 的仲裁测试分别通过 16 项和 8 项。夹爪负载专项核对默认关闭、显式启用和四位置回读。构建 10 个包，自检 34/34。测试仅使用仿真和反馈夹具。MoveIt 在原语回归结束后退出时复现已知段错误。

[TCP 复测](../.verify/merge-review/tcp.log) · [默认场景](../.verify/merge-review/gravity.log) · [物理场景](../.verify/merge-review/physical.log) · [负载复测](../.verify/merge-review/load.log)

### v1.1.2 发布复测

| 检查 | 覆盖 |
|---|---|
| 构建与版本 | 构建 10 个 ROS 包，核对 10 个已安装 ROS 包和 3 个已安装 Python 包均为 `1.1.2` |
| 离线回归 | 自检 34/34，安全判定 81/81，完整离线回归 21/21 |
| 持续重力补偿 | 默认 MuJoCo 16/16，物理场景 8/8。覆盖显式选择、断流回退、恢复更新、手动切换和 MIT 总开关。 [选源复测](#持续重力补偿与手动切换) |
| 网页负载估计 | 独立 Web 与 MIT bringup 复测默认关闭和显式启用，Chromium 核对面板状态。 [负载估计复测](#网页负载估计复测) |

复测使用 mock、隔离 MuJoCo、ROS 2 反馈夹具与 Chromium。现场测力精度待单独核对。

### MIT 重力补偿总开关

在零重力 MuJoCo 用实际高斯 API 选外部。Chromium 显示当前实际来源为「已关闭」，控制器回读六轴重力均为零，随后归中成功。隔离 SDK 测试通过 ROS 参数打开与关闭同一个开关，核对开启时新鲜外部力矩能生效，关闭时回到零。

### 持续重力补偿与手动切换

```bash
source scripts/ros_env.sh && b601_source_workspace
python3 scripts/test_target_arbitration.py --http-port 18867
```

隔离 MuJoCo 验证单条目标发出 0.65 秒后，重力来源继续使用外部，桥接器仍接收按实测姿态计算的新力矩。位置目标保持原转发编号。手动切内部后逐轴回读恢复。模拟 SDK 提供者停更时，控制器回到内部。恢复该提供者的更新后，无须重新选择就回到外部。隔离暂停桥接器也可触发控制器回退。摩擦与任务前馈的时效沿用原回归。

独立网页此前已验证首组外部补偿可在发送运动目标前生效。本次用 Chromium 加入桥接器暂停与恢复，验证网页显示的逐轴实际来源和用户选择分别变化。实际高斯与夹爪 API 已同环境加载，实机持续切换待现场验证。

### 网页六向 TCP 点动

在隔离 MuJoCo 中，通过网页 Connect 自动启动 MoveIt 和 MIT 控制器。
Chromium 核对 `base_link` 标记、六向轴符号、网页控制权和在途按钮状态。单击 5 mm 原语后，
以关节回读正解核对 ±X、±Y、±Z 的方向。不可达目标未更新采纳编号，随后仍可提交普通七关节目标。
桌面与窄屏均检查了按钮和位移反馈布局。离开页面后会话超时进入实测持位，随后隔离后端归中并 Disable。
MoveIt 在仿真后端归中失能后退出时复现已知段错误。网页已回到空闲预览。

[网页操作](../README.md#网页-tcp-点动) · [浏览器回读](../.verify/tcp-web-browser/results.json)

### 网页连续 TCP 目标参考

`B601_TEST_ROS_DOMAIN_ID=215 python3 scripts/test_target_arbitration.py --tcp-primitives --http-port 8892`
通过 CLI 组合及网页同会话连续点动、不可达后保留基准、普通关节目标后重取实测基准。

以 01:00:04.921 的真机关节反馈离线回放「前 20 mm」。起点 Z 为 277.57 mm，
上一条目标 Z 为 291.36 mm，修正后规划终点 Z 为 291.36 mm。 [回放数值](../.verify/tcp-reference-replay/result.json)

### TCP 原地旋转复测

构建后，在隔离 MuJoCo MIT 和 MoveIt 中复测三轴旋转、六向平移和 Web 同会话组合。

```bash
source scripts/ros_env.sh && b601_source_workspace
B601_TEST_ROS_DOMAIN_ID=181 python3 scripts/test_target_arbitration.py --tcp-primitives --http-port 8916
```

按 `rotate_x 5 rotate_y -5 rotate_z 5` 顺序执行。目标 XYZ 完全相同。以下实测位置漂移由仿真关节反馈正解得到，单位为 mm。

| 段 | 中途实测最大漂移 | 段末实测漂移 | 段末角度误差 |
|---|---:|---:|---:|
| 绕 X 正转 5° | 0.590 | 0.00051 | 0.000072° |
| 绕 Y 反转 5° | 0.885 | 0.00021 | 0.000026° |
| 绕 Z 正转 5° | 0.664 | 0.00007 | 0.000008° |

目标轨迹采样点经正解的最大位置偏差为 0.0274 mm。Chromium 在物理 MuJoCo 通过网页完成左移、绕 Y 反转 3°、重力切换、夹爪负载回读、七关节目标、归中与失能。Web API 同一会话的旋转后平移保持旋转后的姿态，失败规划与普通关节目标的参考语义通过回归。MoveIt 在仿真停机时复现已知退出段错误。 [轨迹数值](../.verify/tcp-rotation/path-metrics.json) · [网页](../.verify/tcp-rotation/browser-retry/connected.png) · [原语回归](../.verify/tcp-rotation/tcp.log)

用实际高斯重力和夹爪负载工厂的一键入口复测物理 MuJoCo。左移 3 mm 后绕 Y 反转 3°，固定同一个目标位置。旋转前后实测位置相差 0.358 mm，段末姿态误差为 0.100°，距目标位置 1.675 mm。后续归中、失能并落盘。默认和物理 MuJoCo 仲裁分别通过 16 项和 8 项，负载专项通过。 [一键入口](../.verify/tcp-rotation/external-smoke.log) · [默认回归](../.verify/tcp-rotation/standard-final.log) · [物理回归](../.verify/tcp-rotation/physical-final.log)

最终源码构建 10 包，自检 34/34，完整离线回归 21/21，六向平移、三轴旋转和 Web 同会话组合再次通过。 [最终原语回归](../.verify/tcp-rotation/tcp-final.log) · [离线回归](../.verify/tcp-rotation/preflight.log)

### TCP 原语上层入口复测

使用隔离 MuJoCo。网页一键入口在有重力与碰撞的场景连接实际高斯重力和夹爪负载 API，执行 3 mm 平移与 ±3° 旋转。其余程序接口用保持夹爪开度的 MuJoCo 夹具执行。每段核对控制器联合采纳。平移保持目标姿态，旋转保持目标 XYZ。

| 入口 | 通过项 |
|---|---|
| 网页 | 一键入口实际点击全部 12 个平移和旋转按钮，核对重力、负载和归中失能。普通独立网页、挂载式网页分别通过混合点动 |
| CLI `tcp_primitives` | 12 段混合序列预览与执行。预览无仲裁目标，执行后释放会话 |
| HTTP `/api/tcp_jog` | 独立客户端执行同样的 12 段，核对连续参考、日志、序号、回执及零角拒绝 |
| Python SDK | 分别通过 `set_joints()` 与 `publish()` 执行三轴平移和旋转的规划采样点 |
| 原生 ROS 2 | 分别通过 `SetTarget` 服务与 `TargetFrame` 话题执行三轴平移和旋转的同一规划结果 |

日常回归可用原地旋转小节的启动命令复跑。完整入口矩阵的记录保留于本工作树。MoveIt 在仿真退出时复现已知段错误，测试进程已退出。 [浏览器](../.verify/upper-primitives/browser.log) · [普通网页](../.verify/upper-primitives/standard-web.log) · [挂载网页](../.verify/upper-primitives/attached.log) · [CLI、HTTP 和服务](../.verify/upper-primitives/matrix-results.json) · [流式结果](../.verify/upper-primitives/stream-results.json)

### TCP 平移与旋转实机复测

2026-11-27 15:54–15:56 的 B601-DM 日志含 32 次成功点动，其中 19 次平移、13 次旋转。样本使用内部与外部重力补偿，分段姿态和动作顺序不同。位置与姿态由关节反馈正解计算。测试以归中和 Disable 结束。

| 动作 | 次数 | 三维终点位置 RMSE | 最大终点位置误差 |
|---|---:|---:|---:|
| 全部 | 32 | 7.20 mm | 15.46 mm |
| 平移 | 19 | 6.90 mm | 13.18 mm |
| 旋转 | 13 | 7.61 mm | 15.46 mm |

旋转姿态 RMSE 为 0.845°。旋转目标 XYZ 均保持上一参考位置，实测 TCP 在各次旋转开始与结束之间的位置变化 RMSE 为 11.91 mm，最大 18.52 mm。全轮 X/Y/Z 的终点 RMSE 为 1.45/0.65/7.02 mm。最终关节目标正解相对规划目标的三维位置 RMSE 约 0.00001 mm。

关节稳态统计使用下一操作前的 0.30 秒保持窗，首条动作因窗内仍在漂移而排除。31 条有效样本中，只有绝对关节指令增量至少为 1° 的样本参与相对误差计算。分母为本次最终关节指令与动作开始前驱动参考的差值。

| 关节 | 最大绝对稳态误差 | 最大相对误差 | 仅旋转最大相对误差 |
|---|---:|---:|---:|
| J1 | 0.294° | 16.4% | 6.4% |
| J2 | 0.721° | 49.3% | 36.6% |
| J3 | 1.990° | 85.4% | 67.6% |
| J4 | 0.595° | 17.7% | 17.7% |
| J5 | 0.708° | 69.6% | 69.6% |
| J6 | 0.328° | 9.5% | 9.5% |

[TCP 逐次数据](../.verify/web-recordings/20261127-154905-g4BYFH/analysis/tcp-jogs.csv) · [关节逐次数据](../.verify/web-recordings/20261127-154905-g4BYFH/analysis/joint-steady-relative.csv)

### TCP 平移原语组合

`codex/tcp-primitives` 使用隔离的默认 MuJoCo MIT 场景和 MoveIt，初始 J5 为 0.3 rad。在 DDS 域 219 和 220 连续复测通过。
[复测命令](../README.md#tcp-平移原语组合)

| 检查 | 结果 |
|---|---|
| 单步输入与预览 | `forward 0.09` 参数校验失败，`forward 0.1` 生成两个轨迹点，预览未发目标 |
| 方形组合 | 四段各 20 mm，反馈 FK 的主方向依次为 +X、+Y、−X、−Y。段末均有两组 MIT 回执，最终回到起点附近 |
| 同向组合 | `forward 8 forward 12` 的两段反馈均沿 +X，完成后释放会话 |
| 不可达的后续段 | `forward 5 left 1200` 的第二段仅完成约 10% 的规划，未转发任一段，且释放会话 |

仿真结果用于核对规划和执行顺序。本轮未做实机运动或定位精度验收。
MoveIt 在组合结束并保存结果后复现已知的 `move_group` 退出段错误，MuJoCo 后端与控制节点正常退出。
[JSON 回读](../.verify/target-arbitration-219/tcp-primitives.json) · [MoveIt 日志](../.verify/target-arbitration-219/moveit.log)

### 上层重力 API 复测

以下记录对应 2026-11-26 随目标帧提交的实现。现行复测命令和时效见上节。

| 检查 | 结果 |
|---|---|
| 默认兼容性 | 默认 MuJoCo 仲裁、SDK、HTTP 和原生 ROS 2 回归 13/13 |
| 实际高斯 API | SDK 和独立 Web 以不同于目标位置的实测姿态求值。Web 的绝对与增量入口均提交 J2–J5 完整保持力矩 |
| 物理 MuJoCo | bringup 启动 Web 时选择外部高斯。手臂和夹爪均确认同一转发编号，手臂控制器的重力回读与 J2–J5 仲裁值逐轴一致 |
| 回退与手工覆盖 | 核对缺少或过期反馈、非有限输出、显式零值及静态自检 34/34、完整离线回归 21/21。外部模式缺少 API 时报告配置错误 |
| 其他入口 | 归中节点用自己的 ROS 参数加载高斯 API。手动 MIT 工具确认新参数可用。两条入口各自启用外部模型后的完整运动流程待复测 |

旧版复测在 MuJoCo 物理场景分别启动内部模式和外部高斯模式，读入实测姿态并发一次目标，再从 `/api/state` 核对以下字段。

- `gravity_source` 和 `gravity_api` 表示 Web 启动配置。
- `status.last_forwarded.gravity_ff` 表示最近转发的外部补偿。默认模式的普通网页目标为空，高斯模式为 J2–J5 四项。
- 在下一帧覆盖或前馈过期前，`compensation_sources.gravity` 显示控制器逐轴实际来源。停止更新目标帧后，高斯补偿按默认 **0.5 秒**超时回退。可通过 `/b601_mit_arm/ff` 的重力段核对数值。

这组结果的覆盖范围为 SDK、Web 和隔离仿真。

### v1.1.1 发布复测

| 检查 | 结果 |
|---|---|
| 构建与版本 | 10 个 ROS 包构建通过，10 份 ROS 清单与 3 个已安装 Python 包均为 `1.1.1` |
| 自检与离线回归 | 静态自检 34/34，安全判定 81/81，离线回归 21/21 |
| 默认与物理 MuJoCo | 分别通过 13/13 和 7/7。覆盖 SDK、原生 ROS 2、HTTP、命令行、外部补偿与内部回退、归中及失能门控 |
| 刚度选择 | 开启时，J1 Kp 到位后从 115 降至 57.5，新目标使其回到 115。关闭时，到位后保持 115 |
| 七轴增量点发 | J1、J6 和夹爪的非零增量映射正确，手臂与夹爪均回执采纳，J1 到位，会话已释放 |

本轮自动验证使用 mock 与隔离 MuJoCo。现场验收范围见 [实机覆盖](#实机覆盖)。

### 相对关节目标复测

用确定反馈核对 HTTP 网关自行计算 J1 `0.100 + 0.050 = 0.150 rad` 和夹爪 `0.015 − 0.002 = 0.013 m`，其余轴采用同帧实测值。反馈缺失、缺轴或过期时上层不发出目标。MuJoCo 中核对 SDK 点发和流式发送均进入原有全量绝对目标接口，旧帧仍受原序号规则约束。HTTP 返回发送的绝对位置与转发编号。

增量 CLI 在隔离 MuJoCo 中验证 J1 `+0.03 rad`、J6 `−1e-3 rad` 和夹爪 `+0.0004 m` 的顺序映射、双路采纳、J1 到位和会话释放。参数缺位或非有限值会在发出请求前拒绝。

### 外部补偿与刚度

早期实验快照包括两种 MuJoCo 场景、离线回归、刚度、七路前馈、浏览器状态和重力模型算例。模型算例使用本工作区 URDF 及同级辨识目录中的模型。数据文件记录模型与采样的 SHA-256。 [2026-11-26 实验数据](experiments/2026-11-26.json)

| 检查 | 结果 |
|---|---|
| 外部补偿 | 默认、物理 MuJoCo 分别通过 11 项、6 项。覆盖 SDK、原生 ROS 2 服务和话题、HTTP 的局部覆盖、显式零、非法条目、过期及恢复内部值 |
| 内部补偿 | 物理场景确认 `gravity_ff` 省略或过期后恢复非零的内部重力前馈。原三段前馈回归读到 0.1 N·m 内部摩擦峰值 |
| 刚度选择 | `true` 时 J1 的 Kp 在到位后从 115 降至 57.5，新目标使其回到 115。`false` 时到位后仍为 115。网页状态接口回读当前设置及档位 |
| 数据格式 | 传统模型输出 J2–J5 完整保持力矩。当前共享神经网络输出七路完整保持量，只有 J2–J5 有学习修正 |
| 默认启动与七路覆盖 | 独立网页无开关参数启动并连接默认 MuJoCo，报告切换关闭。运动与到位共 33 帧均回读六轴高档 Kp。六轴和主动夹爪分别提交非零重力、摩擦值，全部在控制器 `ff` 回读中匹配 |
| 网页补偿来源 | Chromium 验证待回读、内部数据、外部传入、局部混合和断连。混合例为重力 J2、摩擦 J4 和夹爪，显式零均识别为外部。只续期会话时，两项在目标前馈过期后回到内部数据 |

传统与当前共享重力模型的 J2–J5 留组 MSE 分别为 `0.17772` 和 `0.16899 (N·m)²`。评估使用同一批 25 个历史姿态及 19 个留组。共享结构也是用这批姿态选定的，独立姿态验证仍待完成。

早期实验快照中，默认 MuJoCo 的 `safety_monitor` 在结束时出现一次 ROS 上下文已关闭异常。该组 11 项运行期断言全部通过，仿真子进程也已退出。

## v1.1.0 MuJoCo 验收结果

| 检查 | 结果 |
|---|---|
| 默认场景 | 10/10。HTTP、CLI、SDK、原生 ROS 2 点发与流式目标通过。覆盖整机独占、双路采纳、旧会话拒绝、归中、失能门控、撤销和断流持位。来源及仲裁断流进入持位分别为 0.669 s 和 0.499 s |
| 物理场景 | 5/5。两个起始姿态归中与失能门控通过，验证 SDK、原生 ROS 2、HTTP 和 CLI 的位置、速度与任务前馈 |
| 浏览器 | 从独立预览连接 MuJoCo。只发位置时速度和任务前馈均为零。同帧模式回读 J1 速度 `0.01 rad/s`、J1 任务前馈 `0.25 N·m` 与夹爪任务前馈 `0.1 N`。两次均确认采纳，随后归中、Disable 并恢复预览 |
| 任务前馈专项 | 固定七关节位置与全零速度，J1 接收 `+0.8 N·m`、`−0.8 N·m` 时，稳态分别偏移 `+0.00696 rad`、`−0.00696 rad`。省略后归零并回位。并在双场景回归中核对了超时归零与畸形前馈拒绝 |

## v1.0.1 历史 MuJoCo 验收结果

| 场景 | 验收结果 |
|---|---|
| 独立网页 | Chromium 加载 11 个链节和 34 个官方 STL。重复启动复用控制台，普通 HTTP 服务占用端口时报告冲突并保留原服务 |
| 双场景发令 | `free` 和 `physical` 均通过网页七关节目标、`manual` 命令行、外部 `TargetClient`、原生 ROS 2 服务点发和 `TargetFrame` 话题流。检查了手臂和夹爪两路采纳及实测到位 |
| 双场景仲裁 | 两个场景各通过七组检查，覆盖发布者唯一性、控制权争用与交接、重复帧和旧会话拒绝、归中及失能预约、撤销、来源断流和仲裁中断后的持位与重新申请 |
| 目标校验 | 物理场景拒绝重复关节、错误速度长度、非有限位置、非有限速度、位置超限和带 `effort` 的位置帧。七关节按关节名匹配，乱序完整帧和完整速度数组可采纳 |
| 浏览器操作 | 验证实测读入、关节草稿、Ctrl 拖动 TCP、保留夹爪草稿、目标叠加与手动发送。页面离开后约 `0.638 s` 进入持位，重新打开后取得新会话 |
| 归中与后端切换 | 网页在原标定停放位确认归中，Disable 前要求实测到位及双路回执。Disable 后退出后端并恢复预览，随后可选择另一场景 |
| MoveIt 2 | 独立 `pos_vel` 链路通过 34 点规划和 33 点规划执行。标准轨迹由 `FollowJointTrajectory` 发送给位置控制器 |

两种场景的来源断流与仲裁中断复测中，进入持位的实测时间为 `0.564–0.656 s`。网页会话释放和撤销均通过，公开状态只报告公开会话编号。

### 复跑与边界

- 并行启动的首轮物理场景归中有一次报告 MIT 目标转发端点未就绪，隔离复跑的两个起始姿态均通过。首轮浏览器自动手势等待超时，之后两轮实际指针拖动通过。
- J2 处在 MuJoCo 软限位时，测得约 `+0.001 rad` 的超调会触发原归中逻辑的目标校验。页面会显示失败阶段「J4 上抬」，同一阶段保持的 J2 目标触发越界。碰撞模型和归中限位各自由原入口处理。

夹爪两指在常规档和低刚度档的实测超速急停阈值为 `0.25 m/s`，警告线为 `0.20 m/s`。常规档超过阈值触发，低刚度档达到阈值即触发。隔离 MuJoCo 的安全节点已回读两档新值。夹爪命令速度上限为 `0.05 m/s`。

## 实机覆盖

以下覆盖按操作者在 2026-11-25 提供的实机验收情况登记。自动回归复测使用隔离 MuJoCo。实机轨迹、次数和测量值沿用已有现场记录。

| 流程 | 操作者确认的覆盖 |
|---|---|
| 网页实机发令 | 网页提交目标并回读采纳与实测 |
| 命令行 `manual` | 使用 `dm_mit_move.py` 经整机仲裁发目标 |
| 外部 `TargetClient` | 外部客户端经整机仲裁发目标 |
| 原生 ROS 2 客户端 | 使用仲裁服务或话题发送目标 |
| 整机控制权交接 | 网页与其他来源之间交接整机会话 |
| 网页归中、Disable、恢复 | 完成归中失能并恢复网页预览 |

## 复测

从工作区根目录构建并运行静态校验。默认回归覆盖目标、仲裁和断流持位。`--physical` 复测物理场景的两个归中起点与失能门控。两种模式均可加 `--http-port` 检查 HTTP 与命令行发令。

```bash
./scripts/build.sh
./scripts/verify.sh
source scripts/ros_env.sh && b601_source_workspace
python3 scripts/test_target_arbitration.py --http-port 8876
python3 scripts/test_target_arbitration.py --physical --http-port 8876
```

实验工作区的刚度选择可用相同夹具单独复测。

```bash
python3 scripts/test_target_arbitration.py --stiffness-switching true --http-port 8876
python3 scripts/test_target_arbitration.py --stiffness-switching false --http-port 8876
```

回归夹具默认使用 ROS 域 `227`。通过 `B601_TEST_ROS_DOMAIN_ID` 可指定另一个空闲域。独立网页和同场景的 ROS 客户端使用同一个空闲域，例如 `226`。

```bash
export ROS_DOMAIN_ID=226 ROS_LOCALHOST_ONLY=1
./scripts/start_target_web.sh port:=8766
```

1. 打开 [隔离控制台](http://127.0.0.1:8766/)，检查默认模型，连接 MuJoCo。分别选择关闭和开启重力与碰撞，重复以下流程。
2. 指定 `web`，读入实测，只修改并发送小幅 J1 位置。检查「已采纳」并核对 `tff` 为零。然后同帧填写位置、速度和任务前馈，核对 `/api/state` 的 `status.last_forwarded` 与 `last_applied_id`。
3. 将指定来源切换为 `manual`，另开终端加载同一工作区和 ROS 域，执行 `python3 scripts/dm_mit_move.py --set 'joint1=0.065'`。再依次测试外部 `TargetClient` 与原生 ROS 2 点发和话题流。示例见 [ROS 2 接口](../README.md#ros-2-接口)。
4. 清除外部来源，回到网页点击一键归中。实测到位后点击 Disable，等预览恢复再切换场景。

任务前馈运动复测使用默认 MuJoCo。通过 SDK 固定七关节位置与零速度，在每一阶段以约 20 Hz 重发目标。J1 依次省略 `tff`、传 `+0.8 N·m`、省略、传 `−0.8 N·m`、省略。比对 `/joint_states` 中 J1 的稳态位置和 `/b601_mit_arm/ff` 的任务前馈段。

独立位置轨迹和 MoveIt 复测使用另一个空闲 ROS 域。见 [位置回路复测](pos-vel.md#在-mujoco-复测-moveit-2)。

## 网页负载估计复测

```bash
source scripts/ros_env.sh && b601_source_workspace
OPENBLAS_NUM_THREADS=1 python3 scripts/test_load_feedback.py
```

试验以独立 ROS 域启动网页。覆盖普通启动和只指定位置的启动，两种情况下负载 HTTP 数据均为 `null`，且没有负载节点。显式指定物理观察器后，MuJoCo 将夹爪点上的 `[4, -6, -8] N` 换算为反馈力矩，节点从中恢复同向的三维外力，HTTP 三轴误差小于 `0.01 N`。空载读数为零且没有方向，断流后力值和时间戳清空。换用测试工厂后验证大臂、小臂、腕部和夹爪四个位置各有独立读数。

同一姿态下对比 URDF 与物理 MJCF 的四个作用点，基座系下最大位置差为 `2.07e-7 m`。Chromium 验证 `10.77 N` 和带符号的 XYZ 读数，并确认夹爪箭头随反馈中断消失。实机外力估计的精度待现场测力确认。

挂载式 Web 使用隔离 MuJoCo 复测。只启用外部重力时，`/api/state` 中的 `load` 为 `null`，`b601_load` 未启动，Chromium 中的外力面板隐藏。显式传入 `target_web_load_api:=b601_load_id.gripper_load_api:create_provider` 后，节点启动并提供夹爪读数，Chromium 显示面板。

## 非 Web 上层接口复测

2026-11-27 在 `v1.1.2` 的隔离 MuJoCo 物理场景中验证。重力提供者使用实际高斯 API `neural_network.feedforward:GaussianFeedforward.load`，负载节点独立运行 `b601_load_id.gripper_load_api:create_provider`，各入口均从 ROS 反馈核对结果。 [启动独立估计节点](../README.md#不开网页时读取夹爪估计) · [重力选源](../README.md#在上层启动时选择重力来源)

| 入口 | 观测结果 |
|---|---|
| Python `TargetClient` | 点发和流式发送均采纳，J2–J5 回读为外部，夹爪估计持续发布。释放运动会话后，只要提供者节点继续处理 ROS 回调，外部重力仍持续更新 |
| 手动 MIT 命令行 | 使用外部 API 完成目标。进程退出后重力回退内部，夹爪估计继续发布 |
| 原生 ROS 2 | 用重力选择服务与更新话题提供高斯样本，用目标服务发令。控制器采纳，提供者停更后回退内部，夹爪估计继续发布 |
| MIT 归中 | 使用外部高斯 API 完成六轴归中，负载节点持续输出，停稳后归零 |
| MoveIt `pos_vel` | 规划并执行末端上移 20 mm，夹爪估计持续发布。该位置回路未加载 MIT 重力桥接器 |

在夹爪估计点固定 0.8 kg 模拟载荷，理论载荷为 7.85 N，方向沿 `base_link` 的 −Z。SDK、命令行和原生 ROS 2 的核对样本为 7.32–7.55 N，方向距 −Z 约 2°。在空载归中复测中，250 个样本有 18 个为非零，最大 77.60 N，同姿态回放未见碰撞接触，停稳后归零。夹爪网络从关节力矩变化估计等效负载，加减速产生的惯性力矩也会计入读数。验收确认了接口可用性及已知载荷的采样结果。

### 非 Web 上层与 MIT 仲裁联合复测

2026-11-27 在 TCP 工作树 `833ccc1` 加本地改动上，以隔离物理 MuJoCo 场景复测。点动与轨迹使用场景原有 MIT 参数，在夹爪估计点固定 0.8 kg 载荷。独立 `b601_load` 运行 `b601_load_id.gripper_load_api:create_provider`。通过两组控制器回读核对内置重力和实际高斯 API 的选择。外部模式下 J2–J5 取高斯模型的前馈，其余轴沿用内部。SDK、ROS 2、命令行与轨迹均核对了负载估计的持续更新。

| 上层入口 | 内置重力 | 外部高斯重力 | 核对项 |
|---|---|---|---|
| `tcp_primitives` CLI | 12/12 | 12/12 | ±X/Y/Z 平移 3 mm 和 ±X/Y/Z 旋转 3°，逐段核对两组采纳，预览未发目标 |
| Python SDK 点发／流式 | 各 12/12 | 各 12/12 | 共用 `TcpPlanner`，核对平移姿态与旋转 XYZ 的目标约束 |
| 原生 ROS 2 服务／话题 | 各 12/12 | 各 12/12 | 共用规划结果，使用整机仲裁下发和实际回读 |
| 手动 MIT CLI | 通过 | 通过 | 两个关节目标先后到达，释放控制权 |
| 标准 `FollowJointTrajectory` | 通过 | 通过 | 手臂与夹爪同轨迹、反馈和结果、竞争拒绝、取消持位、撤销持位与容差失败 |
| MoveIt 及轨迹示例 | 通过 | 通过 | Plan & Execute、位姿执行、关节点到点、多点轨迹、夹爪与规划文件回放 |
| MIT 归中 | 通过 | 通过 | 空载专项，负载节点持续发布，六轴到位后允许预留失能 |

两种重力模式合计验证 120 段 TCP 原语。轨迹运行时，控制器管理器显示 `b601_mit_arm` 与 `b601_mit_gripper` 均活跃并占用各轴的 position、velocity、effort、Kp、Kd 接口。MoveIt 的执行端点为 `/b601_mit_trajectory/follow_joint_trajectory`。仲裁状态记录 `trajectory` 控制会话和两组控制器对末帧的共同采纳。

仲裁独立回归 13 项通过。覆盖独占控制权、交接、重放拒绝、两路回执、掉线持位、重力停更回退与恢复、归中与失能门控。另验证 TCP CLI 在已有控制者时被拒绝，现有重力来源和已转发目标保持原值。CLI 在取得控制权前加载外部模型，取得控制权并确认来源生效后才发目标。

在带载场景中，CLI 的旋转起止反馈最大位移为内部 0.30 mm、外部 0.49 mm。相对各段规划目标，首段下沉使 CLI 最大绝对位置误差约为内部 11.2 mm、外部 12.3 mm。此次验收覆盖上层接口、目标约束和实际回执。MoveIt 退出期间复现已知段错误，轨迹结果此前已收齐。

从当前检出目录构建并加载环境后，可用以下两条命令分别复测非 Web 仲裁和 TCP 组合。每条命令会自行启动隔离 MuJoCo。

```bash
source scripts/ros_env.sh && b601_source_workspace
B601_TEST_ROS_DOMAIN_ID=212 python3 scripts/test_target_arbitration.py
B601_TEST_ROS_DOMAIN_ID=216 python3 scripts/test_target_arbitration.py --tcp-primitives
```

标准 action 和 MoveIt 的同框架复测见 [MIT 定时轨迹复测](#mit-定时轨迹复测)。使用外部高斯重力时，先开物理 MuJoCo MIT 后端，再按操作说明启动轨迹执行器并挂接 MoveIt。 [重力和负载启动](../README.md#在轨迹模式中选择外部-api)

[逐段结果](../.verify/non-web-mit-20261127/results.json) · [内置轨迹](../.verify/non-web-mit-20261127/internal-trajectories.log) · [外部轨迹](../.verify/non-web-mit-20261127/external-trajectories.log) · [仲裁](../.verify/non-web-mit-20261127/arbiter-final.log) · [CLI 竞争复测](../.verify/non-web-mit-20261127/cli-guard.log) · [复测脚本](../.verify/non-web-mit-20261127/matrix.py)

## 外部 API 一键启动与日志复测

从实验工作区根目录用空闲 ROS 域和 HTTP 端口启动。

```bash
ROS_DOMAIN_ID=158 ./scripts/start_target_web_external.sh port:=18878
```

在 Web 中连接 MuJoCo「开启重力与碰撞」，核对 J2–J5 采用外部高斯重力，夹爪节点输出估计。发送 J1 小位移后核对两路控制器采纳，再依次归中、Disable 和退出。重复启动同配置时复用了原预览及日志。 [启动与文件说明](../README.md#外部-api-一键启动)

本轮写入 2,072 条 JSONL 事件，覆盖网页操作、采纳、反馈、负载和补偿来源，并正常结束记录。MuJoCo 运行日志写入同一会话的 `ros/`。实机驱动 CSV 路径通过 xacro 参数核对，含空格和引号的路径也可正确传入。

### TCP 工作区同步 `2eb5f80` 后复测

普通入口与一键入口分别使用独立 ROS 域和端口启动。两种入口重复启动均复用原预览和日志路径。普通入口保留默认记录，一键入口在自己分配的目录内写入 JSONL 和 `ros/`。使用实际高斯重力与夹爪 API 在物理 MuJoCo 点动 3 mm，核对 J2–J5 外部重力、负载读数、点动回执与同一 JSONL 中的规划事件。随后归中、失能并正常结束记录。

构建 10 包、自检 34/34，TCP 原语及网页同会话回归通过。独立原语测试结束时复现已知的 MoveIt 退出段错误。未连接实机。 [一键入口复测](../.verify/base-2eb5f80/smoke-retry.log) · [TCP 回归](../.verify/base-2eb5f80/tcp.log)
