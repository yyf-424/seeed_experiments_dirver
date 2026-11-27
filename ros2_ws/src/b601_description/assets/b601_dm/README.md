# Seeed B601-DM 官方模型资源

URDF 和 STL 来自 Seeed Projects 的 ROS 2 包 `rebotarm_bringup`，复制自上游提交
`87780c6a5c2fb43b55ddebda5eac375ceb9c5a40` 的 `src/rebotarm_bringup/description/DM`。
原始 URDF 和网格文件保持原样。见 [上游模型](https://github.com/Seeed-Projects/reBotArmController_ROS2/tree/87780c6a5c2fb43b55ddebda5eac375ceb9c5a40/src/rebotarm_bringup/description/DM)。

上游包声明 Apache-2.0。见 [包元数据](https://github.com/Seeed-Projects/reBotArmController_ROS2/blob/87780c6a5c2fb43b55ddebda5eac375ceb9c5a40/src/rebotarm_bringup/package.xml) 和 [许可证文本](LICENSE)。

`b601_description` 安装此目录。网页、默认 MoveIt 规划和碰撞复测工具将上游 URDF 中的
`package://rebotarm_bringup/description/DM/` 网格引用解析到这里。

MuJoCo 的「开启重力与碰撞」场景使用 `link1`、`link3`、`link5` 的官方碰撞网格凸包。生成器从 `meshes/collision/*.STL` 提取凸包顶点并嵌入专用 MJCF。见 [生成器](../../../../../scripts/gen_mjcf.py) 和 [物理场景模型](../../../b601_mujoco/models/b601_arm_physical.xml)。
