# Learning Guide

This document is for the person who wants to understand the code well enough to change the mower behavior on purpose.

It is different from `README.md`:

- `README.md` explains the project to other people
- this file explains how to read the code and where to make changes safely

If you prefer a visual map first, open `docs/diagrams.md` next to this file.

## The Right Mental Model

Do not read this repository as a flat list of files.

Read it as a flow of decisions:

1. the operator sends a movement request
2. the request becomes `/cmd_vel`
3. the control node converts that into wheel speeds
4. the control node updates position and orientation over time
5. the control node publishes `/joint_states`, `/odom`, and `/tf`
6. Foxglove only visualizes the result

The most important consequence is this:

Foxglove is not the source of behavior.
It only shows what the ROS node already computed.

## The Three Layers

The repository has three practical layers.

### 1. Shared C++ motion logic

This layer is the easiest entry point.

Read these files first:

1. `include/DriveController.h`
2. `src/DriveController.cpp`
3. `include/IRobotDriver.h`
4. `include/SimulationDriver.h`
5. `src/SimulationDriver.cpp`
6. `src/main.cpp`

This layer teaches the smallest version of the problem:

- read keyboard intent
- convert it into left and right wheel behavior
- send it to some output

If ROS feels too big, come back to this layer first.

### 2. ROS 2 runtime logic

This layer is where the mower becomes a live system.

The main files are:

1. `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp`
2. `ros2_ws/src/automower_control/src/automower_drive_node.cpp`
3. `ros2_ws/src/automower_control/src/automower_tool_controller.cpp`

This layer adds:

- topics
- timers
- odometry
- transforms
- visualization data

### 3. Runtime workflow and tooling

This layer starts the environment but usually does not define robot behavior.

The main files are:

1. `scripts/run_full_session.sh`
2. `scripts/run_wasd_teleop.sh`
3. `scripts/build_ros_workspace.sh`
4. `scripts/run_ros_stack.sh`
5. `scripts/run_foxglove_bridge.sh`

These files matter because they decide how the app is started, but they are not the first place to change driving behavior.

## The Smallest Useful Reading Order

If you want to learn quickly without drowning in details, use this order:

1. `include/DriveController.h`
2. `src/DriveController.cpp`
3. `src/main.cpp`
4. `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp`
5. `ros2_ws/src/automower_control/src/automower_drive_node.cpp`
6. `ros2_ws/src/automower_description/urdf/automower.urdf.xacro`
7. `ros2_ws/src/automower_description/launch/display.launch.py`
8. `docs/visualization.md`

That order goes from easiest logic to full system behavior.

## What Each Important File Controls

### `src/DriveController.cpp`

This file controls how throttle and steering are mixed.

Today the rule is simple:

- left wheels = `throttle + steering`
- right wheels = `throttle - steering`

If you want to change how strongly the mower turns, this is one of the first places to inspect.

### `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp`

This file controls how keyboard input becomes a stream of `geometry_msgs/msg/Twist` messages.

This is where you change:

- speed step size
- turn step size
- max forward speed
- max turn rate
- key mapping
- stop behavior

If the mower feels too aggressive or too sluggish, this file is often the first lever.

### `ros2_ws/src/automower_control/src/automower_drive_node.cpp`

This is the core runtime file.

This file controls:

- how `/cmd_vel` is interpreted
- how `/work_mode` and `/emergency_stop` gate drive motion
- how wheel speeds are stored
- how often state is updated
- how wheel positions are integrated
- how odometry is computed
- how `/joint_states` is published
- how `/odom` and `odom -> base_footprint` are published
- how the simple marker model is shown in Foxglove

If you want to affect physical behavior over time, this is the most important file in the repo.

One recent example is the winter safety profile: the node now scales motion down
when `work_mode` is `snow_clearing`, so safety lives in the runtime controller
instead of depending only on careful keyboard use.

### `ros2_ws/src/automower_control/src/automower_tool_controller.cpp`

This file is the current placeholder for the working tool.

It does not drive real hardware yet, but it already defines the contract for:

- `blade` profile
- `auger` profile
- tool enable state
- tool power
- tool angle
- interaction with work mode and emergency stop

This is where the mower-to-snowblower transition starts to become a real
software boundary instead of just a future idea.

### `ros2_ws/src/automower_description/urdf/automower.urdf.xacro`

This file controls the robot geometry and frame tree.

This is where you change:

- body dimensions
- wheel placement
- frame relationships
- visual layout

If something looks wrong in 3D but the math seems right, this is a likely place to inspect.

## How To Read `automower_drive_node.cpp`

Do not read it as one long file.
Read it as four roles.

### 1. Input role

Function: `cmdVelCallback`

Question to ask:
What new command did the outside world request?

This function does not move the robot directly.
It updates the requested wheel-speed target.

### 2. State update role

Functions:

- `publishState`
- `updateWheelPositions`
- `updateOdometry`

Question to ask:
How does the internal state evolve over time?

This is the place where motion becomes continuous instead of event-based.

### 3. Output role

Functions:

- `publishJointStates`
- `publishOdometry`
- `publishVisualization`

Question to ask:
How does the outside world learn about the state?

These functions do not decide behavior.
They expose already computed behavior.

### 4. Configuration role

Functions:

- `workModeCallback`
- `emergencyStopCallback`
- `driveAllowedInCurrentMode`
- `applyDriveSafetyProfile`

Question to ask:
What higher-level operating rules are shaping the motion before the odometry
math even runs?

This is where seasonal behavior, safety constraints, and future task-specific
profiles belong.

Look at the constants near the top of the class.

These constants decide things like:

- wheel radius
- track width
- body size
- command timeout
- max wheel linear speed

This is where small numeric changes can have big visible effects.

## The Most Important Topics And Frames

You should understand these before making larger changes.

### `/cmd_vel`

This is the requested movement command.
It is an input, not a measured result.

### `/odom`

This is the node's estimate of mower position and velocity.
If odometry looks wrong, the issue is usually in the control node, not in Foxglove.

### `/joint_states`

This carries wheel motion for the robot model.
If the robot body exists but wheel motion looks wrong, inspect this path.

### `/tf` and `/tf_static`

These describe frame relationships.
If Foxglove or RViz says a transform is missing, the problem is usually not visual styling, but missing or inconsistent frame publication.

## How To Debug Without Getting Lost

When something looks wrong, follow this order:

1. Did teleop publish the intended command?
2. Did the control node receive and store it?
3. Did the fixed-rate update loop run?
4. Did odometry and TF get published?
5. Is Foxglove using the correct fixed frame?

This order matters because it prevents random guessing.

## Where You Have Real Physical Influence

If you want to change what the mower actually does, these are the highest-value places.

### Change operator feeling

Edit `ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp`.

Examples:

- slower acceleration per key press
- sharper steering increments
- lower top speed
- different stop behavior

### Change steering model

Edit `src/DriveController.cpp`.

Examples:

- softer turns
- stronger differential effect
- different left/right mixing logic

### Change motion integration

Edit `ros2_ws/src/automower_control/src/automower_drive_node.cpp`.

Examples:

- different update frequency
- different odometry math
- acceleration smoothing
- slower decay to zero after input stops

### Change robot geometry

Edit `ros2_ws/src/automower_description/urdf/automower.urdf.xacro`.

Examples:

- wider wheelbase
- longer body
- different wheel positions
- different frame offsets

## Safe Exercises To Learn The System

These are good first changes because they are visible and easy to reason about.

1. Change the teleop speed step in `automower_wasd_teleop.cpp` and test the driving feel.
2. Change `maxLinearSpeed` in teleop and compare the difference in control.
3. Change `TRACK_WIDTH` in `automower_drive_node.cpp` and observe the turning behavior.
4. Change `MAX_WHEEL_LINEAR_SPEED` in `automower_drive_node.cpp` and inspect the odometry effect.
5. Change wheel positions in `automower.urdf.xacro` and verify them in Foxglove.

## What To Ignore At First

Do not start by studying everything.

At the beginning, you can safely treat these as secondary:

- Docker startup details
- XQuartz fallback path
- old RViz workflow
- Webots controller scaffold

They matter later, but they are not the shortest path to understanding the current control loop.

## Best Next Step

If you want to become independent in this repo, the best next deep read is:

`ros2_ws/src/automower_control/src/automower_drive_node.cpp`

Read it function by function and always ask:

1. Is this function receiving input?
2. Is it updating state?
3. Is it publishing output?
4. Which variables does it truly control?

Once you can answer those questions confidently, you will be able to modify behavior intentionally instead of by trial and error.
