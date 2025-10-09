# robot_controller Architecture

## Overview

This package contains controller configurations for the robot_xl platform, supporting both mecanum and differential drive types.

## Package Structure

```mermaid
graph TB
    subgraph "robot_controller Package"
        direction TB
        
        subgraph "Launch Files"
            CL[controller.launch.py]
        end
        
        subgraph "Config Files"
            direction LR
            subgraph "robot_xl"
                MC[mecanum_drive_controller.yaml]
                DC[diff_drive_controller.yaml]
            end
        end
        
        subgraph "Controllers"
            MDC[Mecanum Drive Controller]
            JSB[Joint State Broadcaster]
            IB[IMU Broadcaster]
        end
        
        CL --> MC
        CL --> DC
        MC --> MDC
        MC --> JSB
        MC --> IB
        DC --> JSB
        DC --> IB
    end
    
    style CL fill:#4CAF50
    style MC fill:#2196F3
    style DC fill:#FF9800
```

## Controller Architecture

```mermaid
graph TB
    subgraph "Controller Manager"
        CM[controller_manager<br/>Lifecycle Management]
    end
    
    subgraph "Controllers"
        MDC[mecanum_drive_controller<br/>Velocity Control]
        JSB[joint_state_broadcaster<br/>State Publishing]
        IB[imu_sensor_broadcaster<br/>IMU Publishing]
    end
    
    subgraph "Hardware Interface"
        HWI[RobotSystem<br/>Command & State Interfaces]
        IMU_HW[RobotImuSensor<br/>IMU State Interfaces]
    end
    
    subgraph "ROS2 Topics"
        CMD[/cmd_vel<br/>Input]
        ODOM[/odometry/wheels<br/>Output]
        JS[/joint_states<br/>Output]
        IMU[/imu/data<br/>Output]
    end
    
    CM --> MDC
    CM --> JSB
    CM --> IB
    
    CMD --> MDC
    MDC -->|velocity commands| HWI
    HWI -->|state interfaces| MDC
    MDC --> ODOM
    
    HWI -->|state interfaces| JSB
    JSB --> JS
    
    IMU_HW -->|state interfaces| IB
    IB --> IMU
    
    style CM fill:#9C27B0
    style MDC fill:#4CAF50
    style JSB fill:#2196F3
    style IB fill:#FF9800
```

## Data Flow Diagram

```mermaid
flowchart LR
    subgraph "Input"
        TELEOP[Teleop/Nav2]
    end
    
    subgraph "Controller Layer"
        MDC[Mecanum Drive<br/>Controller]
        
        subgraph "Control Logic"
            IK[Inverse Kinematics<br/>twist → wheel vel]
            PID[PID Control<br/>Optional]
            ODOM_CALC[Odometry<br/>Calculation]
        end
    end
    
    subgraph "Hardware Interface"
        CMD_IF[Command Interfaces<br/>4x velocity]
        STATE_IF[State Interfaces<br/>4x position<br/>4x velocity]
    end
    
    subgraph "Output"
        ODOM_OUT[Odometry]
        JS_OUT[Joint States]
    end
    
    TELEOP -->|/cmd_vel| MDC
    MDC --> IK
    IK --> CMD_IF
    
    STATE_IF --> ODOM_CALC
    ODOM_CALC --> ODOM_OUT
    
    STATE_IF --> JS_OUT
    
    style MDC fill:#4CAF50
    style IK fill:#2196F3
```

## Controller Configuration Structure

```mermaid
graph TB
    subgraph "YAML Configuration"
        direction TB
        
        ROOT[controller_manager]
        
        subgraph "Controller Definitions"
            MDC_DEF[mecanum_drive_controller<br/>type: mecanum_drive_controller/MecanumDriveController]
            JSB_DEF[joint_state_broadcaster<br/>type: joint_state_broadcaster/JointStateBroadcaster]
            IB_DEF[imu_sensor_broadcaster<br/>type: imu_sensor_broadcaster/IMUSensorBroadcaster]
        end
        
        subgraph "Controller Parameters"
            direction TB
            
            subgraph "Mecanum Parameters"
                JOINTS[wheel_names<br/>4 wheel joints]
                GEOM[Geometry<br/>wheel_separation_x/y<br/>wheel_radius]
                LIMITS[Velocity Limits<br/>linear/angular]
                ODOM[Odometry<br/>frame_id<br/>pose_covariance]
            end
            
            subgraph "IMU Parameters"
                IMU_NAME[sensor_name: imu_sensor]
                IMU_FRAME[frame_id: imu_link]
            end
        end
        
        ROOT --> MDC_DEF
        ROOT --> JSB_DEF
        ROOT --> IB_DEF
        
        MDC_DEF --> JOINTS
        MDC_DEF --> GEOM
        MDC_DEF --> LIMITS
        MDC_DEF --> ODOM
        
        IB_DEF --> IMU_NAME
        IB_DEF --> IMU_FRAME
    end
    
    style ROOT fill:#9C27B0
    style MDC_DEF fill:#4CAF50
    style JSB_DEF fill:#2196F3
    style IB_DEF fill:#FF9800
```

## Sequence Diagram: Controller Lifecycle

```mermaid
sequenceDiagram
    participant CM as Controller Manager
    participant MDC as Mecanum Drive Controller
    participant HWI as Hardware Interface
    participant FW as Firmware
    
    CM->>MDC: configure()
    MDC->>MDC: Load parameters<br/>(wheel names, geometry, limits)
    MDC->>HWI: Claim command interfaces
    MDC->>HWI: Claim state interfaces
    MDC-->>CM: SUCCESS
    
    CM->>MDC: activate()
    MDC->>MDC: Initialize odometry
    MDC->>MDC: Reset integrators
    MDC-->>CM: SUCCESS
    
    Note over CM,FW: Control Loop Active
    
    loop Control Loop (100 Hz)
        MDC->>HWI: Read state interfaces
        HWI->>FW: Get latest /joint_states
        FW-->>HWI: Joint positions & velocities
        HWI-->>MDC: State data
        
        MDC->>MDC: Calculate odometry
        MDC->>MDC: Publish odometry
        
        Note over MDC: Receive /cmd_vel
        MDC->>MDC: Apply inverse kinematics
        MDC->>MDC: Apply velocity limits
        MDC->>HWI: Write command interfaces
        HWI->>FW: Publish /cmd_vel (Twist)
    end
    
    CM->>MDC: deactivate()
    MDC->>HWI: Write zero velocities
    MDC-->>CM: SUCCESS
```

## Mecanum Drive Kinematics

```mermaid
graph TB
    subgraph "Inverse Kinematics (Controller)"
        direction TB
        TWIST[Twist Input<br/>vx, vy, ω]
        
        IK_CALC[Inverse Kinematics<br/>vfl = vx - vy - ω*L<br/>vfr = vx + vy + ω*L<br/>vrl = vx + vy - ω*L<br/>vrr = vx - vy + ω*L]
        
        WHEEL_VEL[Wheel Velocities<br/>4 wheels]
        
        TWIST --> IK_CALC
        IK_CALC --> WHEEL_VEL
    end
    
    subgraph "Forward Kinematics (Odometry)"
        direction TB
        WHEEL_STATE[Wheel States<br/>4 positions & velocities]
        
        FK_CALC[Forward Kinematics<br/>vx = Σ wheel_vel / 4<br/>vy = weighted sum<br/>ω = weighted sum]
        
        ODOM[Odometry Output<br/>pose & twist]
        
        WHEEL_STATE --> FK_CALC
        FK_CALC --> ODOM
    end
    
    WHEEL_VEL -.->|Command Interfaces| HWI[Hardware Interface]
    HWI -.->|State Interfaces| WHEEL_STATE
    
    style IK_CALC fill:#4CAF50
    style FK_CALC fill:#2196F3
```

## Topic Remapping

```mermaid
graph LR
    subgraph "Controller Topics"
        direction TB
        CMD_IN[drive_controller/cmd_vel_unstamped]
        ODOM_OUT[drive_controller/odom]
        IMU_IN[imu_sensor_node/imu]
        IMU_OUT[imu_sensor_broadcaster/imu]
    end
    
    subgraph "Remapped Topics"
        direction TB
        CMD[/cmd_vel]
        ODOM[/odometry/wheels]
        IMU_RAW[/imu/data_raw]
        IMU_DATA[/imu/data]
    end
    
    CMD --> CMD_IN
    ODOM_OUT --> ODOM
    IMU_RAW --> IMU_IN
    IMU_OUT --> IMU_DATA
    
    style CMD fill:#4CAF50
    style ODOM fill:#2196F3
    style IMU_RAW fill:#FF9800
    style IMU_DATA fill:#9C27B0
```

## Configuration Parameters

### Mecanum Drive Controller

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel_names` | string[] | - | Names of 4 wheel joints (FL, FR, RL, RR) |
| `wheel_separation_x` | double | 0.270 | Distance between front and rear wheels (m) |
| `wheel_separation_y` | double | 0.170 | Distance between left and right wheels (m) |
| `wheel_radius` | double | 0.047 | Wheel radius (m) |
| `linear.x.max_velocity` | double | 0.5 | Max linear velocity in x (m/s) |
| `linear.y.max_velocity` | double | 0.5 | Max linear velocity in y (m/s) |
| `angular.z.max_velocity` | double | 1.0 | Max angular velocity (rad/s) |
| `odom_frame_id` | string | odom | Odometry frame name |
| `base_frame_id` | string | base_link | Base frame name |
| `pose_covariance_diagonal` | double[] | [0.001, ...] | Pose covariance (6 values) |
| `twist_covariance_diagonal` | double[] | [0.001, ...] | Twist covariance (6 values) |

### Joint State Broadcaster

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joints` | string[] | - | Joint names to broadcast |
| `interfaces` | string[] | [position, velocity] | State interfaces to publish |

### IMU Sensor Broadcaster

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sensor_name` | string | imu_sensor | Name of IMU sensor in URDF |
| `frame_id` | string | imu_link | IMU frame name |

## State Machine: Controller States

```mermaid
stateDiagram-v2
    [*] --> Unconfigured: Load Plugin
    
    Unconfigured --> Inactive: configure()<br/>Load parameters<br/>Claim interfaces
    Unconfigured --> Finalized: cleanup()
    
    Inactive --> Active: activate()<br/>Initialize odometry<br/>Reset state
    Inactive --> Unconfigured: cleanup()
    
    Active --> Inactive: deactivate()<br/>Write zero velocities
    
    Finalized --> [*]
    
    note right of Active
        update() called at 100 Hz:
        1. Read state interfaces
        2. Calculate odometry
        3. Process cmd_vel
        4. Apply inverse kinematics
        5. Write command interfaces
    end note
```

## Performance Characteristics

- **Update Rate**: 100 Hz (10 ms period)
- **Odometry Calculation**: < 1 ms
- **Inverse Kinematics**: < 0.1 ms
- **Command Latency**: < 10 ms (end-to-end)
- **Odometry Accuracy**: ±2% (wheel slip dependent)

## Error Handling

| Error Condition | Detection | Action | Recovery |
|----------------|-----------|--------|----------|
| Missing wheel joint | Configuration | Fail to configure | Fix URDF/config |
| Invalid geometry | Parameter validation | Fail to configure | Fix parameters |
| Command timeout | No cmd_vel received | Continue with last command | Automatic on new command |
| State read failure | Hardware interface error | Log error, use last state | Automatic on next cycle |
| Velocity limit exceeded | Input validation | Clamp to limits | Automatic |

## Launch File Structure

### controller.launch.py

**Purpose**: Load and activate all controllers

**Key Features**:
- Load controller manager configuration
- Spawn controllers in correct order
- Configure topic remappings
- Handle controller lifecycle

**Spawned Controllers**:
1. `joint_state_broadcaster` (first - provides joint states)
2. `imu_sensor_broadcaster` (IMU data)
3. `mecanum_drive_controller` (last - depends on joint states)

## Dependencies

- **controller_manager**: Controller lifecycle management
- **mecanum_drive_controller**: Custom mecanum controller (from robot_controllers package)
- **joint_state_broadcaster**: Standard ros2_controllers
- **imu_sensor_broadcaster**: Standard ros2_controllers
- **robot_hardware_interfaces**: Hardware interface plugin

## Usage Examples

### Launch Controllers
```bash
ros2 launch robot_controller controller.launch.py
```

### List Controllers
```bash
ros2 control list_controllers
```

### Check Controller Status
```bash
ros2 control list_hardware_interfaces
```

### Send Velocity Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Monitor Odometry
```bash
ros2 topic echo /odometry/wheels
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Controller fails to load | Wrong controller type | Check controller_manager config |
| No odometry published | Controller not active | Check controller state |
| Robot doesn't move | No cmd_vel received | Check topic remapping |
| Incorrect motion | Wrong kinematics parameters | Verify wheel_separation and wheel_radius |
| Jerky motion | Velocity limits too low | Increase max_velocity parameters |
| Drift in odometry | Wheel slip or wrong parameters | Calibrate wheel radius, check for slip |
