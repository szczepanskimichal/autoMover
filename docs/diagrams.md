# System Diagrams

This file collects visual diagrams for the current `autoMower` architecture.

Use it together with `docs/learning-guide.md` when you want a quick visual map before reading source code.

## 1. End-to-End Runtime Flow

```mermaid
flowchart LR
    User[Operator keyboard input]
    Wrapper[run_wasd_teleop.sh]
    Teleop[automower_wasd_teleop.cpp]
    CmdVel[/cmd_vel]
    Mode[/work_mode and /emergency_stop]
    DriveNode[automower_drive_node.cpp]
    Controller[DriveController]
    State[Internal motion state\ncurrentSpeeds x y yaw wheel positions]
    ToolCmd[/tool_profile /tool_enabled\n/tool_power /tool_angle]
    ToolNode[automower_tool_controller.cpp]
    JointStates[/joint_states]
    Odom[/odom]
    TF[/tf: odom -> base_footprint]
    Markers[/visualization_marker_array]
    Foxglove[Foxglove 3D view]

    User --> Wrapper
    Wrapper --> Teleop
    Teleop --> CmdVel
    Mode --> DriveNode
    CmdVel --> DriveNode
    DriveNode --> Controller
    Controller --> State
    ToolCmd --> ToolNode
    State --> JointStates
    State --> Odom
    State --> TF
    State --> Markers
    JointStates --> Foxglove
    Odom --> Foxglove
    TF --> Foxglove
    Markers --> Foxglove
```

## 2. What The Control Node Does

```mermaid
flowchart TD
    A[cmdVelCallback] --> B[Store last command time]
    B --> C[Convert Twist into WheelSpeeds]
    C --> D[Update currentSpeeds_]

    T[stateUpdateTimer every 50 ms] --> E[publishState]
    E --> F{Recent command available?}
    F -- yes --> G[Use currentSpeeds_]
    F -- no --> H[Use zero wheel speeds]
    G --> I[Compute dt]
    H --> I
    I --> J[updateWheelPositions]
    J --> K[updateOdometry]
    K --> L[publishJointStates]
    L --> M[publishOdometry]
    M --> N[Broadcast TF]
```

## 3. File Responsibility Map

```mermaid
flowchart TD
    subgraph Shared_Cpp[Shared C++ logic]
        DC_H[include/DriveController.h]
        DC_CPP[src/DriveController.cpp]
        IDRV[include/IRobotDriver.h]
        SIM_H[include/SimulationDriver.h]
        SIM_CPP[src/SimulationDriver.cpp]
        MAIN[src/main.cpp]
    end

    subgraph ROS_2[ROS 2 runtime]
        TELEOP[ros2_ws/src/automower_control/src/automower_wasd_teleop.cpp]
        TOOL[ros2_ws/src/automower_control/src/automower_tool_controller.cpp]
        NODE[ros2_ws/src/automower_control/src/automower_drive_node.cpp]
        URDF[ros2_ws/src/automower_description/urdf/automower.urdf.xacro]
        LAUNCH[ros2_ws/src/automower_description/launch/display.launch.py]
    end

    subgraph Host_Scripts[Host workflow]
        FULL[scripts/run_full_session.sh]
        TELEOP_SH[scripts/run_wasd_teleop.sh]
        BUILD[scripts/build_ros_workspace.sh]
        STACK[scripts/run_ros_stack.sh]
        BRIDGE[scripts/run_foxglove_bridge.sh]
    end

    DC_H --> DC_CPP
    IDRV --> SIM_H
    SIM_H --> SIM_CPP
    DC_CPP --> NODE
    DC_CPP --> MAIN
    TELEOP --> NODE
    TOOL --> LAUNCH
    URDF --> LAUNCH
    NODE --> LAUNCH
    FULL --> BUILD
    FULL --> STACK
    FULL --> BRIDGE
    TELEOP_SH --> TELEOP
```

## 4. Topic And Frame View

```mermaid
flowchart LR
    Teleop[Teleop node] -->|publishes| CmdVel[/cmd_vel]
    ModeTools[Mode and tool scripts] -->|publish| WorkMode[/work_mode]
    ModeTools -->|publish| EStop[/emergency_stop]
    ModeTools -->|publish| ToolProfile[/tool_profile]
    ModeTools -->|publish| ToolEnabled[/tool_enabled]
    ModeTools -->|publish| ToolPower[/tool_power]
    ModeTools -->|publish| ToolAngle[/tool_angle]
    Drive[Drive node] -->|publishes| Joint[/joint_states]
    Drive -->|publishes| Odom[/odom]
    Drive -->|broadcasts| TF[/tf]
    ToolController[Tool controller] -->|interprets| ToolStatus[Tool state]
    RobotStatePublisher[robot_state_publisher] -->|broadcasts| TFStatic[/tf_static]
    URDF[/robot_description/] --> RobotStatePublisher
    WorkMode --> Drive
    EStop --> Drive
    WorkMode --> ToolController
    EStop --> ToolController
    ToolProfile --> ToolController
    ToolEnabled --> ToolController
    ToolPower --> ToolController
    ToolAngle --> ToolController
    Joint --> Foxglove[Foxglove]
    Odom --> Foxglove
    TF --> Foxglove
    TFStatic --> Foxglove
```

```mermaid
flowchart TD
    OdomFrame[odom]
    BaseFootprint[base_footprint]
    BaseLink[base_link]
    FL[front_left_wheel]
    FR[front_right_wheel]
    RL[rear_left_wheel]
    RR[rear_right_wheel]

    OdomFrame --> BaseFootprint
    BaseFootprint --> BaseLink
    BaseLink --> FL
    BaseLink --> FR
    BaseLink --> RL
    BaseLink --> RR
```

## 5. Development Workflow

```mermaid
sequenceDiagram
    participant Dev as Developer
    participant Script as run_full_session.sh
    participant Docker as Docker container
    participant ROS as ROS stack
    participant Bridge as Foxglove Bridge
    participant UI as Foxglove

    Dev->>Script: run full session
    Script->>Docker: start or recreate container
    Script->>Docker: build ROS workspace
    Script->>ROS: launch display.launch.py
    Script->>Bridge: launch foxglove_bridge
    Bridge-->>UI: expose ws://localhost:8765
    Dev->>UI: connect to bridge
    Dev->>Docker: start teleop through wrapper
    Docker->>ROS: publish /cmd_vel
    ROS-->>UI: publish /odom, /tf, /joint_states, markers
```

## How To Use These Diagrams

- Start with the end-to-end runtime flow when you want to understand the big picture.
- Use the control-node diagram when reading `automower_drive_node.cpp`.
- Use the file responsibility map when deciding where to make a code change.
- Use the topic and frame view when Foxglove or TF looks wrong.
