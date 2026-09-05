# autoMower

`autoMower` is a small robotics learning project that combines a simple C++ drive model with a ROS 2 visualization and teleoperation stack running in Docker.

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

### 4. Tool and mode scaffold

The repository now includes a generic tool-control scaffold so the same drive
platform can later host either a mower blade or a snowblower auger.

The current control-side topics are:

- `/work_mode` with values such as `manual_drive`, `mowing`, `snow_clearing`, or `emergency_stop`
- `/emergency_stop` as a hard stop signal
- `/tool_profile` with `blade` or `auger`
- `/tool_enabled` to arm or disarm the tool
- `/tool_power` as a normalized `0.0..1.0` setpoint
- `/tool_angle` as a normalized `-1.0..1.0` setpoint for blade angle or future auger-related steering

The `automower_tool_controller` node is currently a stub that validates these
signals and reports the effective tool state. That keeps the repo ready for the
snowblower path before the real hardware driver exists.

### 5. Visualization workflow

On macOS, the main workflow is:

1. start the Docker container
2. build the ROS 2 workspace inside the container
3. launch the ROS stack
4. launch Foxglove Bridge
5. connect Foxglove to `ws://localhost:8765`
6. drive with the WASD teleop script

The wrapper script `./scripts/run_full_session.sh` automates steps 1 to 4.

## Main Workflows

### Start everything

```bash
./scripts/run_full_session.sh
```

If the container was created before Foxglove port publishing was added:

```bash
./scripts/run_full_session.sh --recreate
```

### Drive the mower

```bash
./scripts/run_wasd_teleop.sh
```

The teleop is incremental rather than hold-based:

- `w` and `s` increase or decrease forward speed
- `a` and `d` increase or decrease turn rate
- `1`, `2`, `3` switch between `manual_drive`, `mowing`, and `snow_clearing`
- `b` and `n` switch between `blade` and `auger`
- `i` toggles the engine state
- `t` toggles auger engagement
- `r` and `f` increase or decrease engine throttle demand
- `g` and `h` rotate the chute left and right
- `y` and `u` adjust the deflector down and up
- `v` simulates auger overload for testing
- `j` requests auger fault reset after the auger has been disengaged and the throttle has been reduced
- `e` toggles emergency stop
- `x` or `space` stops the mower
- `q` quits teleop

In `snow_clearing`, the drive node automatically applies a slower safety
profile so the same keyboard inputs produce gentler motion and lower turn
aggression than in summer-oriented modes.

The teleop still publishes the generic tool topics for compatibility, but the
current winter-oriented control path now also publishes hybrid-specific
commands for engine state, auger engagement, chute position, deflector position,
and overload simulation.

The auger path now also exposes explicit interlock state, a latched fault, and a
reset-required signal so the mechanical snowblower workflow can be debugged in
software before hardware wiring exists.

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
./scripts/set_work_mode.sh snow_clearing
./scripts/set_tool_state.sh auger true 0.7 0.0
```

You can switch back to mower-oriented behavior later with:

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
- `/engine_enabled`, `/engine_throttle`, and `/auger_engaged` for the hybrid winter attachment model
- `/hybrid/engine_rpm`, `/hybrid/battery_voltage`, `/hybrid/battery_soc`, and `/hybrid/dc_bus_current` for simulated power-train telemetry
- `/hybrid/auger_rpm`, `/hybrid/auger_overload`, `/hybrid/chute_position`, and `/hybrid/deflector_position` for mechanical auger state
- `/hybrid/auger_interlock_ok`, `/hybrid/auger_fault_latched`, `/hybrid/auger_reset_required`, and `/hybrid/auger_status_text` for auger safety and recovery state

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
- [docs/hybrid-snowblower-plan.md](docs/hybrid-snowblower-plan.md)
- [docs/visualization.md](docs/visualization.md)
