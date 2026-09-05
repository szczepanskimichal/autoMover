# autoMower

`autoMower` is a small robotics learning project that combines a simple C++ drive model with a ROS 2 visualization and teleoperation stack running in Docker.

Current development focus:

- the mower learning path is the active track again
- the hybrid snowblower work stays in the repository as a documented reference, but it is currently parked until the budget and hardware plan are clearer

The repository currently has two layers:

- a root C++ prototype that contains the reusable drive logic
- a ROS 2 workspace that publishes the mower state, transforms, markers, and teleop commands for visualization in Foxglove

## Project Goals

- learn how differential-style steering maps into left and right wheel commands
- publish a minimal robot state into ROS 2
- visualize the mower in Foxglove through `/tf`, `/odom`, `/joint_states`, and `/visualization_marker_array`
- keep the macOS workflow reliable by running ROS 2 inside Docker

## Repository Structure

```text
.
|- include/                  Shared C++ interfaces and controller abstractions
|- src/                      Root C++ prototype and local terminal control example
|- ros2_ws/
|  |- src/automower_control/ ROS 2 control node and ROS teleop executable
|  `- src/automower_description/ URDF, launch files, and RViz config
|- scripts/                  Host-side workflow scripts for Docker and ROS
|- docs/                     Learning-oriented notes and workflow documentation
`- simulation/               Webots-related assets and controller files
```

## How The System Works

### 1. Drive command mapping

The shared `DriveController` converts a normalized throttle and steering pair into four wheel commands.

- left wheels = `throttle + steering`
- right wheels = `throttle - steering`
- each value is clamped to the range `[-1.0, 1.0]`

This logic lives in the shared C++ layer so both the prototype code and ROS 2 node can use the same steering model.

### 2. ROS 2 state publishing

The `automower_drive_node` in `ros2_ws/src/automower_control` is responsible for:

- subscribing to `/cmd_vel`
- converting commands into wheel speeds with `DriveController`
- updating wheel angles and a simple odometry estimate on a fixed-rate timer
- publishing `/joint_states`
- publishing `/odom`
- broadcasting `odom -> base_footprint`
- publishing a marker-based mower body to `/visualization_marker_array`
- respecting `/work_mode` and `/emergency_stop` before applying drive commands

The node keeps its motion state on a fixed update loop and falls back to zero wheel speed when teleop input times out. That keeps the `odom` frame alive even when no fresh command is being sent, which matters for Foxglove because the 3D panel needs a stable fixed frame.

### 3. Robot description

The `automower_description` package provides:

- the Xacro/URDF model
- the launch file that starts `robot_state_publisher` and the drive node
- the RViz config used by the experimental XQuartz path

### 4. Mower operating layer

The active control path is mower-specific.

The current control-side topics are:

- `/work_mode` with `manual_drive` and `mowing`
- `/emergency_stop` as a hard stop signal
- `/tool_profile` with `blade` as the active profile
- `/tool_enabled` to request blade activation
- `/tool_power` as a normalized `0.0..1.0` blade power setpoint
- `/tool_angle` as a normalized `-1.0..1.0` cut-height bias input
- `/engine_enabled` for blade-drive state
- `/simulate_lidar_obstacle` for front obstacle testing

The `automower_tool_controller` node turns those inputs into mower-specific
runtime outputs such as `/mower/blade_enabled`, `/mower/blade_rpm`,
`/mower/cut_height`, `/mower/safety_stop`, `/mower/status_text`, and `/scan`.
It also disarms blade drive when operator commands time out, so a lost teleop
session drops back to a safe idle state instead of leaving the cutter armed.

### 5. Visualization workflow

On macOS, the main workflow is:

1. start the Docker container
2. build the ROS 2 workspace inside the container
3. launch the ROS stack
4. launch Foxglove Bridge
5. connect Foxglove to `ws://localhost:8765`
6. drive with the WASD teleop script or the phone UI

The wrapper script `./scripts/run_full_session.sh` automates steps 1 to 4 and
also starts `rosbridge` on `ws://localhost:9090` for the mobile control page.

## Main Workflows

### Start everything

```bash
./scripts/run_full_session.sh
```

If the container was created before Foxglove port publishing was added:

```bash
./scripts/run_full_session.sh --recreate
```

Use `--recreate` at least once after the mobile-control update so Docker also
publishes `9090` for `rosbridge`.

### Drive the mower

```bash
./scripts/run_wasd_teleop.sh
```

The teleop is incremental rather than hold-based:

- `w` and `s` increase or decrease forward speed
- `a` and `d` increase or decrease turn rate
- `1` switches to `manual_drive`
- `2` switches to `mowing`
- the mower path now starts in `mowing` with the `blade` profile as the default operator setup
- `b` forces the `blade` profile
- `i` toggles the blade drive state
- `t` toggles blade enable
- `r` and `f` increase or decrease blade power demand
- `g` and `h` lower or raise the cut-height input
- `o` simulates a lidar obstacle in front of the mower
- `e` toggles emergency stop
- `x` or `space` stops the mower
- `q` quits teleop

In `mowing`, the control layer now also exposes a mower deck, blade state, cut
height estimate, and lidar obstacle state so the mower branch has its own
runtime model instead of looking like a generic box robot.

The mower tool controller now also applies an operator-command timeout. If the
teleop stops publishing, blade drive falls back to a safe idle state and
`/mower/status_text` switches to `operator_command_timeout`.

### Drive from a phone

```bash
./scripts/run_mobile_control.sh
```

This starts a small host-side web server on port `8080` and prints the local
Wi-Fi URL to open on a phone. The page publishes the same motion and mower
control topics as the keyboard teleop, but through `rosbridge` on port `9090`.

Typical operator flow:

1. run `./scripts/run_full_session.sh --recreate` once
2. run `./scripts/run_mobile_control.sh`
3. open the printed `http://MAC_IP:8080` on the phone
4. connect Foxglove on the phone or another device to `ws://MAC_IP:8765`

Experimental joystick path:

```bash
./scripts/run_joystick_teleop.sh
```

This starts `joy_node` and maps `/joy` to `/cmd_vel`. On macOS with Docker
Desktop, direct USB or Bluetooth gamepad access inside the container may still
need a host-side bridge, so treat this as the software integration path first.

### Manual step-by-step workflow

```bash
./scripts/start_automower_container.sh
./scripts/build_ros_workspace.sh
./scripts/run_ros_stack.sh
./scripts/run_foxglove_bridge.sh
./scripts/run_wasd_teleop.sh
```

### Test work modes and tool state

```bash
./scripts/set_work_mode.sh mowing
./scripts/set_tool_state.sh blade true 0.5 -0.3
```

## Key Topics And Frames

The most important runtime interfaces are:

- `/cmd_vel` for motion commands
- `/joint_states` for wheel motion
- `/odom` for odometry
- `/tf` and `/tf_static` for transforms
- `/visualization_marker_array` for the simple mower body visualization
- `/robot_description` for the URDF model
- `/scan` for the synthetic lidar scan anchored at `lidar_link`
- `/mower/blade_enabled`, `/mower/blade_rpm`, `/mower/cut_height`, `/mower/lidar_obstacle`, `/mower/safety_stop`, and `/mower/status_text` for mower-specific runtime state

In Foxglove, the marker view now also shows a forward mower safety zone and a
simple mowing trail in `odom` while the blade is active and the mower is moving.

The main frame chain is:

```text
odom -> base_footprint -> base_link -> wheel links
```

## Development Notes

- `scripts/` is the fastest place to understand how the host machine, Docker container, ROS launch files, and Foxglove are connected.
- `include/` and `src/` contain the simplest version of the drive model and are a good starting point if you want to learn the core logic first.
- `ros2_ws/src/automower_control/src/automower_drive_node.cpp` is the best file to read when you want to understand the full runtime data flow.
- `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp` is the best file to read when you want to understand how keyboard input becomes a stable `/cmd_vel` stream.
- `ros2_ws/src/automower_control/src/automower_tool_controller.cpp` is the best place to evolve the repo from a mower-specific prototype into a platform with swappable tools.

## Additional Documentation

- [docs/project-structure.md](docs/project-structure.md)
- [docs/diagrams.md](docs/diagrams.md)
- [docs/learning-guide.md](docs/learning-guide.md)
- [docs/mobile-control.md](docs/mobile-control.md)
- [docs/hybrid-snowblower-plan.md](docs/hybrid-snowblower-plan.md)
- [docs/visualization.md](docs/visualization.md)

If you continue active implementation work now, treat the mower path as the
primary scope and the hybrid snowblower notes as parked design material.
