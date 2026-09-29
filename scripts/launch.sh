#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TASK_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=42
cd "$TASK_ROOT"
exec ros2 launch ur3_llm_control llm_robot.launch.py "$@"
