#!/usr/bin/env bash

set -euo pipefail

# Publish short /cmd_vel commands from host keyboard input.

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

publish_twist() {
  local linear_x="$1"
  local angular_z="$2"

  docker exec "$container_name" /bin/bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /automower/ros2_ws/install/setup.bash
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: ${linear_x}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${angular_z}}}'
  " >/dev/null
}

cleanup() {
  stty sane
}

trap cleanup EXIT INT TERM

stty -echo -icanon time 0 min 0

cat <<'EOF'
WASD teleop
w: forward
s: backward
a: turn left
d: turn right
space: stop
q: quit

Keep the focus in this terminal while driving.
EOF

while true; do
  key=""
  if ! IFS= read -r -s -n 1 key; then
    continue
  fi

  case "$key" in
    [wW])
      publish_twist 0.6 0.0
      printf '\rforward   '
      ;;
    [sS])
      publish_twist -0.6 0.0
      printf '\rbackward  '
      ;;
    [aA])
      publish_twist 0.0 0.8
      printf '\rleft      '
      ;;
    [dD])
      publish_twist 0.0 -0.8
      printf '\rright     '
      ;;
    ' ')
      publish_twist 0.0 0.0
      printf '\rstop      '
      ;;
    [qQ])
      publish_twist 0.0 0.0
      printf '\rquit      \n'
      exit 0
      ;;
  esac
done