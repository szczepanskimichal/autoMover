#!/usr/bin/env bash

set -euo pipefail

# One-shot workflow: container, build, ROS stack, and Foxglove Bridge.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

container_name="${CONTAINER_NAME:-automower_ros}"
foxglove_port="${FOXGLOVE_PORT:-8765}"
bridge_port="8765"
rosbridge_port="${ROSBRIDGE_PORT:-9090}"
recreate="false"

for arg in "$@"; do
  case "$arg" in
    --recreate)
      recreate="true"
      ;;
    *)
      echo "unknown argument: $arg" >&2
      echo "usage: ./scripts/run_full_session.sh [--recreate]" >&2
      exit 1
      ;;
  esac
done

cd "$repo_root"

if [[ "$recreate" == "true" ]]; then
  CONTAINER_NAME="$container_name" FOXGLOVE_PORT="$foxglove_port" ROSBRIDGE_PORT="$rosbridge_port" \
    "$script_dir/start_automower_container.sh" --recreate
else
  CONTAINER_NAME="$container_name" FOXGLOVE_PORT="$foxglove_port" ROSBRIDGE_PORT="$rosbridge_port" \
    "$script_dir/start_automower_container.sh"
fi

"$script_dir/build_ros_workspace.sh" "$container_name"

docker exec "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    apt-get update >/dev/null
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-foxglove-bridge >/dev/null
  fi
  if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    apt-get update >/dev/null
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-rosbridge-server >/dev/null
  fi
'

docker exec -e BRIDGE_PORT="$bridge_port" -e ROSBRIDGE_PORT="$rosbridge_port" "$container_name" /bin/bash -lc '
  pkill -f "[d]isplay.launch.py" || true
  pkill -f "[a]utomower_drive_node" || true
  pkill -f "[f]oxglove_bridge_launch.xml" || true
  pkill -f "[/]opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge" || true
  pkill -f "[r]osbridge_websocket" || true
  mkdir -p /config/automower_logs
  : >/config/automower_logs/ros_stack.log
  : >/config/automower_logs/foxglove_bridge.log
  : >/config/automower_logs/rosbridge.log
'

docker exec -d "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  exec ros2 launch automower_description display.launch.py use_rviz:=false >/config/automower_logs/ros_stack.log 2>&1
'

docker exec -d -e BRIDGE_PORT="$bridge_port" "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=${BRIDGE_PORT} address:=0.0.0.0 >/config/automower_logs/foxglove_bridge.log 2>&1
'

docker exec -d -e ROSBRIDGE_PORT="$rosbridge_port" "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${ROSBRIDGE_PORT} address:=0.0.0.0 >/config/automower_logs/rosbridge.log 2>&1
'

cat <<EOF
session is up

container: $container_name
foxglove: ws://localhost:$foxglove_port
rosbridge: ws://localhost:$rosbridge_port

logs:
  docker exec -it $container_name /bin/bash -lc 'tail -f /config/automower_logs/ros_stack.log'
  docker exec -it $container_name /bin/bash -lc 'tail -f /config/automower_logs/foxglove_bridge.log'
  docker exec -it $container_name /bin/bash -lc 'tail -f /config/automower_logs/rosbridge.log'

teleop:
  ./scripts/run_wasd_teleop.sh

mobile:
  ./scripts/run_mobile_control.sh
EOF