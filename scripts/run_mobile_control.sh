#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
mobile_dir="$repo_root/mobile"
port="${MOBILE_UI_PORT:-8080}"
foxglove_port="${FOXGLOVE_PORT:-8765}"
rosbridge_port="${ROSBRIDGE_PORT:-9090}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is not available on PATH" >&2
  exit 1
fi

if [[ ! -d "$mobile_dir" ]]; then
  echo "mobile UI directory is missing: $mobile_dir" >&2
  exit 1
fi

candidate_ips=()
for interface in en0 en1; do
  if ip="$(ipconfig getifaddr "$interface" 2>/dev/null || true)"; then
    if [[ -n "$ip" ]]; then
      candidate_ips+=("$ip")
    fi
  fi
done

echo "mobile control UI"
echo ""
echo "open on this machine:"
echo "  http://localhost:${port}"

if [[ ${#candidate_ips[@]} -gt 0 ]]; then
  echo ""
  echo "open on phone in the same Wi-Fi network:"
  for ip in "${candidate_ips[@]}"; do
    echo "  http://${ip}:${port}"
  done
  echo ""
  echo "Foxglove bridge from phone:"
  for ip in "${candidate_ips[@]}"; do
    echo "  ws://${ip}:${foxglove_port}"
  done
  echo ""
  echo "Mobile UI rosbridge target:"
  for ip in "${candidate_ips[@]}"; do
    echo "  ws://${ip}:${rosbridge_port}"
  done
fi

echo ""
echo "press Ctrl-C to stop the mobile UI server"

exec python3 -m http.server "$port" --bind 0.0.0.0 --directory "$mobile_dir"