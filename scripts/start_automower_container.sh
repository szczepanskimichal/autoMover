#!/usr/bin/env bash

set -euo pipefail

# Create or reuse the ROS workflow container with the expected bind mounts and
# host port mappings for Foxglove and mobile-control rosbridge.

container_name="${CONTAINER_NAME:-automower_ros}"
image_name="${IMAGE_NAME:-env-ros2}"
config_volume="${CONFIG_VOLUME:-automower_ros_config}"
host_port="${FOXGLOVE_PORT:-8765}"
rosbridge_port="${ROSBRIDGE_PORT:-9090}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
recreate="false"

for arg in "$@"; do
  case "$arg" in
    --recreate)
      recreate="true"
      ;;
    *)
      echo "unknown argument: $arg" >&2
      echo "usage: ./scripts/start_automower_container.sh [--recreate]" >&2
      exit 1
      ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not available on PATH" >&2
  exit 1
fi

container_exists="false"
if docker container inspect "$container_name" >/dev/null 2>&1; then
  container_exists="true"
fi

if [[ "$container_exists" == "true" ]]; then
  published_ports="$(docker inspect "$container_name" --format '{{json .HostConfig.PortBindings}}')"

  if [[ "$published_ports" != *'8765/tcp'* || "$published_ports" != *'9090/tcp'* ]] && [[ "$recreate" != "true" ]]; then
    cat >&2 <<EOF
container '$container_name' already exists but does not publish the required host ports.

Run this once to rebuild the workflow container correctly:
  ./scripts/start_automower_container.sh --recreate
EOF
    exit 1
  fi

  if [[ "$recreate" == "true" ]]; then
    docker rm -f "$container_name" >/dev/null
    container_exists="false"
  fi
fi

if [[ "$container_exists" == "false" ]]; then
  docker run -dit \
    --name "$container_name" \
    --hostname "$container_name" \
    -v "$repo_root:/automower" \
    -v "$config_volume:/config" \
    -w /automower/ros2_ws \
    -p "$host_port:8765" \
    -p "$rosbridge_port:9090" \
    "$image_name" >/dev/null
else
  docker start "$container_name" >/dev/null
fi

echo "container '$container_name' is ready"
echo "foxglove bridge host port: $host_port"
echo "rosbridge host port: $rosbridge_port"
echo "next: ./scripts/build_ros_workspace.sh"