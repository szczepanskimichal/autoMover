#!/usr/bin/env bash

set -euo pipefail

# Build the ROS 2 packages inside the running container without allocating an
# interactive TTY, which keeps scripted runs from hanging.

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
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-xacro >/dev/null
  fi
'

docker exec "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /automower/ros2_ws
  colcon build --packages-select automower_control automower_description
'