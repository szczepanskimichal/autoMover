#!/usr/bin/env bash

set -euo pipefail

# Start Foxglove Bridge in the container and expose it through the host port.

container_name="${1:-automower_ros}"
port="${FOXGLOVE_PORT:-8765}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not available on PATH" >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container_name"; then
  echo "container '$container_name' is not running" >&2
  echo "start it with ./scripts/start_automower_container.sh" >&2
  exit 1
fi

published_ports="$(docker inspect "$container_name" --format '{{json .HostConfig.PortBindings}}')"
if [[ "$published_ports" != *'8765/tcp'* ]]; then
  cat >&2 <<EOF
container '$container_name' does not publish port 8765 to the host.

Recreate it once with:
  ./scripts/start_automower_container.sh --recreate
EOF
  exit 1
fi

docker exec "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    apt-get update >/dev/null
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-foxglove-bridge >/dev/null
  fi
'

echo "Foxglove bridge will listen on ws://localhost:${port}"

docker exec -it "$container_name" /bin/bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /automower/ros2_ws/install/setup.bash
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=0.0.0.0
'