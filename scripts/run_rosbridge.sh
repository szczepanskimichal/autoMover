#!/usr/bin/env bash

set -euo pipefail

container_name="${1:-automower_ros}"
port="${ROSBRIDGE_PORT:-9090}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not available on PATH" >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container_name"; then
  echo "container '$container_name' is not running" >&2
  echo "start it with ./scripts/run_full_session.sh" >&2
  exit 1
fi

published_ports="$(docker inspect "$container_name" --format '{{json .HostConfig.PortBindings}}')"
if [[ "$published_ports" != *'9090/tcp'* ]]; then
  cat >&2 <<EOF
container '$container_name' does not publish port 9090 to the host.

Recreate it once with:
  ./scripts/start_automower_container.sh --recreate
EOF
  exit 1
fi

docker exec "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    apt-get update >/dev/null
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-rosbridge-server >/dev/null
  fi
  pkill -f "[r]osbridge_websocket" || true
  mkdir -p /config/automower_logs
  : >/config/automower_logs/rosbridge.log
'

echo "rosbridge will listen on ws://localhost:${port}"

docker exec -it -e ROSBRIDGE_PORT="$port" "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=${ROSBRIDGE_PORT} address:=0.0.0.0 | tee /config/automower_logs/rosbridge.log
'