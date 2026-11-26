#!/usr/bin/env bash
set -eo pipefail

# 修改这里的启动参数。发目标、查看状态和停机的终端须使用同一 ROS_DOMAIN_ID。
ROS_DOMAIN_ID=211
ROS_LOCALHOST_ONLY=1
HARDWARE=dm                     # dm 或 mujoco。重力补偿支持这两个后端。
DM_COMMAND_MODE=mit_impedance
MIT_GRAVITY_COMPENSATION=true
MIT_STIFFNESS_SWITCHING=false  # true 开启软硬切换，false 保持高刚度，auto 沿用预设。
DM_ENABLE_MOTORS=true          # 真机 true 会使能并自动归中。现场确认后再启动。
USE_SAFETY=true
USE_RVIZ=false

if (( $# > 0 )); then
  printf '%s\n' '在脚本顶部修改参数，然后直接运行 ./scripts/start_mit_gravity.sh。' >&2
  exit 2
fi

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/ros_env.sh
b601_source_workspace
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY

exec ros2 launch b601_bringup bringup.launch.py \
  "hardware:=$HARDWARE" \
  "dm_command_mode:=$DM_COMMAND_MODE" \
  "mit_gravity_compensation:=$MIT_GRAVITY_COMPENSATION" \
  "mit_stiffness_switching:=$MIT_STIFFNESS_SWITCHING" \
  "dm_enable_motors:=$DM_ENABLE_MOTORS" \
  "use_safety:=$USE_SAFETY" \
  "use_rviz:=$USE_RVIZ"
