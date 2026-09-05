# Mobile Control

The mower stack now supports phone-based control through a simple web UI.

## What runs where

- Foxglove Bridge runs in the Docker container on `ws://HOST_IP:8765`
- `rosbridge_server` runs in the Docker container on `ws://HOST_IP:9090`
- the mobile page is served from the macOS host on `http://HOST_IP:8080`

## Fast start

Start the full ROS session:

```bash
./scripts/run_full_session.sh --recreate
```

The first `--recreate` after this feature is important because the container
must expose both `8765` and `9090` to the host.

Then start the phone UI server on the host:

```bash
./scripts/run_mobile_control.sh
```

The script prints:

- the local URL for the same Mac
- the Wi-Fi URL to open on a phone
- the Foxglove websocket URL
- the rosbridge websocket URL

## What the phone UI controls

The page publishes these topics:

- `/cmd_vel`
- `/work_mode`
- `/tool_profile`
- `/tool_enabled`
- `/tool_power`
- `/tool_angle`
- `/engine_enabled`
- `/simulate_lidar_obstacle`
- `/emergency_stop`

The page also subscribes to:

- `/mower/status_text`
- `/mower/blade_rpm`
- `/mower/cut_height`
- `/mower/lidar_obstacle`
- `/mower/safety_stop`

## Recommended device split

- phone 1: mobile control page on `http://HOST_IP:8080`
- phone 2, tablet, or laptop: Foxglove connected to `ws://HOST_IP:8765`

This keeps steering and visualization separate, which is easier during tests.

## Safety notes

- the page keeps republishing operator state so the mower stays armed only while the UI is alive and connected
- if operator commands stop, the mower controller drops to `operator_command_timeout`
- `E-Stop` on the page publishes `/emergency_stop = true`
- `Obstacle` on the page simulates a lidar stop in front of the mower
