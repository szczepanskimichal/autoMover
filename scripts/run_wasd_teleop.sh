#!/usr/bin/env bash

set -euo pipefail

# Run the ROS-native teleop inside the container for smoother /cmd_vel output.

container_name="${1:-automower_ros}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not available on PATH" >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container_name"; then
  echo "container '$container_name' is not running" >&2
  echo "start it with ./scripts/run_full_session.sh" >&2
  exit 1
fi

exec docker exec -it "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  exec ros2 run automower_control automower_wasd_teleop
'