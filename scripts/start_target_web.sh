#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/ros_env.sh
b601_source_workspace
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/ros2_ws/log/ros}"

# Keep the default observer external to the ROS workspace. A linked worktree
# can sit at a different depth from the main checkout.
load_root="${B601_LOAD_ROOT:-}"
if [[ -z "$load_root" ]]; then
    ancestor="$PWD"
    while [[ "$ancestor" != / ]]; do
        if [[ -d "$ancestor/load_identification/b601_load_id" ]]; then
            load_root="$ancestor/load_identification"
            break
        fi
        ancestor="$(dirname "$ancestor")"
    done
fi
if [[ -n "$load_root" ]]; then
    export PYTHONPATH="$load_root${PYTHONPATH:+:$PYTHONPATH}"
fi

load_positions=gripper
options=()
for option in "$@"; do
    case "$option" in
        load_positions:=*) load_positions="${option#*=}" ;;
        *) options+=("$option") ;;
    esac
done
exec ros2 launch b601_demos target_web.launch.py "load_positions:=$load_positions" "${options[@]}"
