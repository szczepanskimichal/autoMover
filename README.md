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

The node keeps its motion state on a fixed update loop and falls back to zero wheel speed when teleop input times out. That keeps the `odom` frame alive even when no fresh command is being sent, which matters for Foxglove because the 3D panel needs a stable fixed frame.

### 3. Robot description

The `automower_description` package provides:

- the Xacro/URDF model
- the launch file that starts `robot_state_publisher` and the drive node
- the RViz config used by the experimental XQuartz path

### 4. Visualization workflow

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
- `x` or `space` stops the mower
- `q` quits teleop

### Manual step-by-step workflow

```bash
./scripts/start_automower_container.sh
./scripts/build_ros_workspace.sh
./scripts/run_ros_stack.sh
./scripts/run_foxglove_bridge.sh
./scripts/run_wasd_teleop.sh
```

## Key Topics And Frames

The most important runtime interfaces are:

- `/cmd_vel` for motion commands
- `/joint_states` for wheel motion
- `/odom` for odometry
- `/tf` and `/tf_static` for transforms
- `/visualization_marker_array` for the simple mower body visualization
- `/robot_description` for the URDF model

The main frame chain is:

```text
odom -> base_footprint -> base_link -> wheel links
```

## Development Notes

- `scripts/` is the fastest place to understand how the host machine, Docker container, ROS launch files, and Foxglove are connected.
- `include/` and `src/` contain the simplest version of the drive model and are a good starting point if you want to learn the core logic first.
- `ros2_ws/src/automower_control/src/automower_drive_node.cpp` is the best file to read when you want to understand the full runtime data flow.
- `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp` is the best file to read when you want to understand how keyboard input becomes a stable `/cmd_vel` stream.

## Additional Documentation

- [docs/project-structure.md](docs/project-structure.md)
- [docs/visualization.md](docs/visualization.md)
