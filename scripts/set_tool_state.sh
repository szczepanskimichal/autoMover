#!/usr/bin/env bash

set -euo pipefail

# Publish generic tool commands for either blade or auger profiles.

container_name="${CONTAINER_NAME:-automower_ros}"
profile="${1:-auger}"
enabled="${2:-false}"
power="${3:-0.0}"
angle="${4:-0.0}"

case "$profile" in
  blade|auger)
    ;;
  *)
    echo "unsupported profile: $profile" >&2
    echo "usage: ./scripts/set_tool_state.sh [blade|auger] [true|false] [power 0..1] [angle -1..1]" >&2
    exit 1
    ;;
esac

case "$enabled" in
  true|false)
    ;;
  *)
    echo "enabled must be true or false" >&2
    exit 1
    ;;
esac

docker exec "$container_name" /bin/bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 topic pub --once /tool_profile std_msgs/msg/String '{data: \"${profile}\"}'
"

docker exec "$container_name" /bin/bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 topic pub --once /tool_enabled std_msgs/msg/Bool '{data: ${enabled}}'
"

docker exec "$container_name" /bin/bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 topic pub --once /tool_power std_msgs/msg/Float32 '{data: ${power}}'
"

docker exec "$container_name" /bin/bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 topic pub --once /tool_angle std_msgs/msg/Float32 '{data: ${angle}}'
"