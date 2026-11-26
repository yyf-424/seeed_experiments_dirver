#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

gravity_root="${B601_GRAVITY_ROOT:-}"
if [[ -z "$gravity_root" ]]; then
    ancestor="$PWD"
    while [[ "$ancestor" != / ]]; do
        if [[ -f "$ancestor/gravity_identification/neural_network/feedforward.py" ]]; then
            gravity_root="$ancestor/gravity_identification"
            break
        fi
        ancestor="$(dirname "$ancestor")"
    done
fi
if [[ -z "$gravity_root" || ! -f "$gravity_root/neural_network/feedforward.py" ]]; then
    printf '%s\n' '未找到高斯重力 API，请将 B601_GRAVITY_ROOT 指向 gravity_identification。' >&2
    exit 1
fi

log_root="${B601_WEB_LOG_ROOT:-$PWD/.verify/web-recordings}"
mkdir -p "$log_root"
log_root="$(cd "$log_root" && pwd -P)"
export B601_WEB_RECORD_DIR
B601_WEB_RECORD_DIR="$(mktemp -d "$log_root/$(date '+%Y%m%d-%H%M%S')-XXXXXX")"
export ROS_LOG_DIR="$B601_WEB_RECORD_DIR/ros"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-226}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export PYTHONPATH="$gravity_root${PYTHONPATH:+:$PYTHONPATH}"

exec ./scripts/start_target_web.sh \
    gravity_source:=external \
    gravity_api:=neural_network.feedforward:GaussianFeedforward.load \
    load_positions:=gripper \
    load_api:=b601_load_id.gripper_load_api:create_provider \
    "$@"
