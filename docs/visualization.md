# Visualization Workflow

This project keeps ROS 2 execution inside Docker.

Use this document together with the repository overview in `README.md` when you
want the operational workflow rather than the code structure.

The recommended workflow on macOS is:

- ROS graph in Docker
- visualization through Foxglove Bridge on port `8765`
- Foxglove app or browser on the host

This avoids the `rviz2` + XQuartz + OpenGL failure mode.

## Recommended Workflow

## Fast Path

If you do not want to manage multiple terminals, use the single wrapper:

```bash
./scripts/run_full_session.sh
```

For an older container without the Foxglove port mapping:

```bash
./scripts/run_full_session.sh --recreate
```

This wrapper does all of the following:

- starts or recreates the container
- ensures runtime dependencies exist
- builds `automower_control` and `automower_description`
- stops stale mower and bridge processes
- starts the ROS stack in the background
- starts Foxglove Bridge in the background
- prints the Foxglove URL and log locations

Then open Foxglove and connect to:

```text
ws://localhost:8765
```

Use the long-form workflow below only if you want each step separated.

### 1. Start the container with the correct port mapping

From the repository root on macOS:

```bash
./scripts/start_automower_container.sh
```

If you already have an older `automower_ros` container without port `8765`, recreate it once:

```bash
./scripts/start_automower_container.sh --recreate
```

This starts the container with:

- the repository mounted at `/automower`
- the working directory set to `/automower/ros2_ws`
- Foxglove Bridge port `8765` published to the host

### 2. Build the ROS workspace

In terminal 1 on macOS:

```bash
./scripts/build_ros_workspace.sh
```

This ensures:

- `xacro` exists in the container
- `automower_control` is rebuilt
- `automower_description` is rebuilt

### 3. Start the ROS stack

In terminal 1 on macOS:

```bash
./scripts/run_ros_stack.sh
```

This starts:

- `robot_state_publisher`
- `automower_drive_node`

### 4. Start Foxglove Bridge

In terminal 2 on macOS:

```bash
./scripts/run_foxglove_bridge.sh
```

The script installs `ros-jazzy-foxglove-bridge` inside the container if needed and starts the bridge on:

```text
ws://localhost:8765
```

### 5. Open Foxglove on the host

Use Foxglove Desktop or the Foxglove web app on your Mac and connect to:

```text
ws://localhost:8765
```

### 5a. Minimal Foxglove setup for this repository

After connecting, create a `3D` panel and set:

- `Fixed frame` = `odom`
- `Display frame` = `base_link` or `base_footprint`
- `Follow mode` = `Position` or `Heading`

In the `Topics` section of the 3D panel, enable:

- `/tf`
- `/tf_static`
- `/visualization_marker_array`

This repository already publishes a simple mower body as markers, so enabling
`/visualization_marker_array` is the fastest way to see the full prototype.

If you also want the robot from URDF, add a `URDF` custom layer and set:

- `Source` = `Topic`
- `Topic` = `/robot_description`
- `Control mode` = `Joint states`
- `Joint states` = `/joint_states`
- `Display mode` = `Visual`

Recommended helper panels for learning the scene:

- `Topics`
- `Raw Messages`
- `Transform Tree`

### 6. Drive the mower

Use your existing teleop flow in another shell inside the container:

```bash
./scripts/run_wasd_teleop.sh
```

Key mapping in the host teleop:

- `w` forward
- `s` backward
- `a` turn left
- `d` turn right
- `space` stop
- `q` quit

Keep the keyboard focus in the terminal running the teleop. Foxglove uses many
of the same keys for camera control, so driving from the terminal is more
reliable than driving from inside the 3D panel.

## What you should see

Once connected in Foxglove, you should be able to inspect:

- `/odom`
- `/tf`
- `/tf_static`
- `/joint_states`
- `/robot_description`

Use these views first:

- 3D panel
- Raw Messages
- Topics
- Transform Tree

In the 3D panel, the two most useful displays for this project are:

- `/visualization_marker_array` for the simple box-and-wheels prototype
- `URDF` custom layer sourced from `/robot_description` for the robot model

The robot stack already publishes:

- wheel rotation through `/joint_states`
- body motion through `odom -> base_footprint`
- odometry through `/odom`

## Stable Debug Workflow

If visualization seems wrong, keep this order:

1. `./scripts/start_automower_container.sh`
2. `./scripts/build_ros_workspace.sh`
3. `./scripts/run_ros_stack.sh`
4. `./scripts/run_foxglove_bridge.sh`
5. connect Foxglove to `ws://localhost:8765`
6. start teleop

Useful checks from the host:

```bash
docker exec -it automower_ros /bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /automower/ros2_ws/install/setup.bash && ros2 topic list -t'
```

```bash
docker exec -it automower_ros /bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /automower/ros2_ws/install/setup.bash && ros2 topic echo /odom'
```

```bash
docker exec -it automower_ros /bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source /automower/ros2_ws/install/setup.bash && ros2 run tf2_ros tf2_echo odom base_footprint'
```

## Foxglove Troubleshooting

If you can see `odom` and TF axes, but not the mower body:

1. In the 3D panel, confirm `Fixed frame` is `odom`.
2. Confirm `/tf` and `/tf_static` are enabled.
3. Enable `/visualization_marker_array`.
4. If you use the URDF view, add a `URDF` custom layer from topic `/robot_description` and set joint states to `/joint_states`.
5. Press `1` in the 3D panel to recenter the camera on the selected display frame.
6. Toggle `3D` mode if you are accidentally in the flat `2D` view.

For this repository, seeing only wheels or only axes usually means the topic or
custom layer was not enabled in the 3D panel, not that the robot model is missing.

## XQuartz Status

The XQuartz `rviz2` path is experimental on this Mac setup.

We confirmed that:

- ROS launch works
- X11 forwarding can start
- `rviz2` still fails later on GLX/OpenGL context creation

That makes XQuartz unsuitable as the primary workflow for this project.
