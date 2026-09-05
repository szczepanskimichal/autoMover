#!/usr/bin/env bash

set -euo pipefail

# Start joy_node and the joystick-to-cmd_vel mapper inside the running container.

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

docker exec "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  if ! ros2 pkg prefix joy >/dev/null 2>&1; then
    apt-get update >/dev/null
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-joy >/dev/null
  fi
'

cleanup() {
  stty sane >/dev/null 2>&1 || true
}

trap cleanup EXIT INT TERM

docker exec -it "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  trap "kill 0" EXIT INT TERM
  ros2 run joy joy_node &
  echo "started joy_node and automower_joystick_teleop"
  echo "note: on macOS Docker Desktop, direct gamepad passthrough may still require a host-side bridge"
  exec ros2 run automower_control automower_joystick_teleop
'