#!/usr/bin/env bash

set -euo pipefail

# Experimental fallback path for RViz on macOS through XQuartz.

container_name="${1:-automower_ros}"
display_target="${DISPLAY_TARGET:-host.docker.internal:0}"

print_xquartz_help() {
  cat >&2 <<'EOF'
XQuartz is not ready for Docker GUI forwarding.

Do this on macOS:
  1. open -a XQuartz
  2. In XQuartz Preferences -> Security, enable "Allow connections from network clients"
  3. Restart XQuartz
  4. defaults write org.xquartz.X11 nolisten_tcp -bool false
  5. open -a XQuartz

After that, retry:
  ./scripts/run_rviz_xquartz.sh
EOF
}

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not available on PATH" >&2
  exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is not available. Install and start XQuartz first." >&2
  exit 1
fi

if ! pgrep -f 'XQuartz|X11.bin|Xquartz' >/dev/null 2>&1; then
  print_xquartz_help
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$container_name"; then
  echo "container '$container_name' is not running" >&2
  exit 1
fi

xhost +localhost >/dev/null

if ! nc -z localhost 6000 >/dev/null 2>&1; then
  print_xquartz_help
  exit 1
fi

docker exec \
  -e DISPLAY="$display_target" \
  -e QT_X11_NO_MITSHM=1 \
  -it "$container_name" /bin/bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /automower/ros2_ws/install/setup.bash
    rviz2 -d /automower/ros2_ws/install/automower_description/share/automower_description/rviz/automower.rviz
  '