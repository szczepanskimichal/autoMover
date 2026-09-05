#!/usr/bin/env bash

set -euo pipefail

# Publish a work-mode command into the running ROS graph.

container_name="${CONTAINER_NAME:-automower_ros}"
mode="${1:-manual_drive}"

case "$mode" in
  manual_drive|mowing|snow_clearing|emergency_stop)
    ;;
  *)
    echo "unsupported mode: $mode" >&2
    echo "usage: ./scripts/set_work_mode.sh [manual_drive|mowing|snow_clearing|emergency_stop]" >&2
    exit 1
    ;;
esac

docker exec "$container_name" /bin/bash -lc "
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 topic pub --once /work_mode std_msgs/msg/String '{data: \"${mode}\"}'
"

if [[ "$mode" == "emergency_stop" ]]; then
  docker exec "$container_name" /bin/bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /automower/ros2_ws/install/setup.bash
    ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
  "
else
  docker exec "$container_name" /bin/bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /automower/ros2_ws/install/setup.bash
    ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'
  "
fi