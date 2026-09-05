# Project Structure Guide

This document is a learning-oriented walkthrough of the repository. It is meant to answer the question: "which file is responsible for what?"

## Root C++ Layer

The root-level C++ code is the smallest version of the mower logic.

### `include/DriveController.h`

Defines `WheelSpeeds` and the `DriveController` interface.

This is the place where the normalized motion model starts:

- throttle represents forward and backward intent
- steering represents turning intent
- the controller converts those two values into four wheel commands

### `src/DriveController.cpp`

Implements the mixing formula for the wheel speeds.

If you want to understand why left and right wheels get different values during turns, start here.

### `include/IRobotDriver.h`

Defines a very small abstraction for "something that can receive wheel commands".

That abstraction keeps the controller separate from the output mechanism.

### `include/SimulationDriver.h` and `src/SimulationDriver.cpp`

Provide a basic console-backed driver.

Right now the driver prints wheel commands, which makes it useful for learning and for early testing without ROS 2.

### `src/main.cpp`

Contains the standalone terminal demo.

It reads keyboard input directly, computes wheel speeds, and forwards them to the simulation driver.

## ROS 2 Layer

The ROS 2 workspace is where the project becomes observable in Foxglove.

### `ros2_ws/src/automower_control`

This package contains the control-side runtime pieces.

#### `src/automower_drive_node.cpp`

This is the most important runtime file in the repo.

It:

- subscribes to `/cmd_vel`
- computes wheel commands from incoming velocity requests
- runs a fixed-rate state update loop
- updates wheel joint positions
- integrates a simple odometry estimate
- publishes `/joint_states`
- publishes `/odom`
- broadcasts `odom -> base_footprint`
- publishes visualization markers

There are two useful mental models when reading this file:

1. command path: `/cmd_vel` -> wheel speeds -> odometry update
2. visualization path: internal state -> `/joint_states`, `/odom`, `/tf`, markers

#### `src/automower_wasd_teleop.cpp`

This is the ROS-native teleop executable.

It runs a small keyboard loop and continuously publishes `geometry_msgs/msg/Twist` on `/cmd_vel`.

The current teleop model is incremental:

- `w` and `s` adjust forward speed
- `a` and `d` adjust turn rate
- `1`, `2`, `3` switch work mode between manual, mowing, and snow clearing
- `b` and `n` select the tool profile
- `i`, `t`, `r`, `f`, `g`, `h`, `y`, `u`, and `v` act as a hybrid operator console for engine, auger, chute, deflector, and overload testing
- `e` toggles emergency stop
- `x` or `space` resets motion to zero

That makes the host-side wrapper simpler and avoids the jerky one-shot command pattern from the earlier shell version.

This file now also acts as the keyboard-side operator console for the current
winter workflow, not just a raw `/cmd_vel` sender.

### `ros2_ws/src/automower_description`

This package contains the robot model and launch assets.

#### `urdf/automower.urdf.xacro`

Defines the robot body, base frames, and wheel joints.

#### `launch/display.launch.py`

Starts:

- `robot_state_publisher`
- `automower_drive_node`
- optionally `rviz2`

#### `rviz/automower.rviz`

Contains the RViz layout used for the older desktop visualization path.

## Script Layer

The host-side scripts are glue code between macOS, Docker, and ROS 2.

### `scripts/start_automower_container.sh`

Creates or starts the Docker container with the right bind mounts and Foxglove port publishing.

### `scripts/build_ros_workspace.sh`

Builds the ROS 2 packages inside the container.

### `scripts/run_ros_stack.sh`

Runs the robot description and drive node stack.

### `scripts/run_foxglove_bridge.sh`

Starts Foxglove Bridge inside the container and exposes it to the host.

### `scripts/run_full_session.sh`

Wraps the full development workflow into one command.

### `scripts/run_wasd_teleop.sh`

Starts the ROS-native teleop executable inside the running container.

## Suggested Reading Order

If you want to learn the project from simple to complex, this order works well:

1. `include/DriveController.h`
2. `src/DriveController.cpp`
3. `src/main.cpp`
4. `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp`
5. `ros2_ws/src/automower_control/src/automower_drive_node.cpp`
6. `ros2_ws/src/automower_description/urdf/automower.urdf.xacro`
7. `scripts/run_full_session.sh`
8. `docs/visualization.md`
