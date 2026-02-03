#!/usr/bin/env bash
# soki_robo ROS2 開発環境セットアップ
#
# 使い方:
#   source ros2_ws/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# ワークスペース (ビルド済みの場合)
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
    source "${SCRIPT_DIR}/install/setup.bash"
fi

# エイリアス
alias cb="cd ${SCRIPT_DIR} && colcon build --symlink-install"
