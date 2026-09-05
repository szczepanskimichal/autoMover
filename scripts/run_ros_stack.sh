#!/usr/bin/env bash

set -euo pipefail

# Launch the robot description and control node in the active container.

container_name="${1:-automower_ros}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not available on PATH" >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container_name"; then
  echo "container '$container_name' is not running" >&2
  echo "start it with ./scripts/start_automower_container.sh" >&2
  exit 1
fi

docker exec "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  if ! command -v xacro >/dev/null 2>&1; then
    apt-get update >/dev/null
    apt-get install -y ros-jazzy-xacro >/dev/null
  fi
'

docker exec -it "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 launch automower_description display.launch.py use_rviz:=false
'