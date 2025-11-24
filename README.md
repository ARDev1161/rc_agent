# rc_agent

## Contents

- [Introduction](#introduction)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [API](#api)

## Introduction

This project is a ROS 2-based robot control and navigation system. It integrates multiple modules, including network communication, mapping, base control, and navigation using the Nav2 stack. The system manages communication between the robot and a remote control station, processes sensor data, and executes navigation commands.

## Features

- **Network Communication**: Easily add support for new types of BMS and communication buses.
  - Uses gRPC for exchanging control and sensor data.
  - Provides an ARP service for discovering control machines in LAN.
- **Battery State Monitoring**: Subscribes to battery state topics and converts ROS messages to protobuf format.
- **Map Module**: Receives map data from topic and converts it into a protobuf map structure.
- **Base Control**: Manages robot base movement commands and monitors odometry for real-time feedback.
- **Navigation**: Uses Nav2 action clients for tasks such as moving to a pose, following waypoints, and spinning.

## Requirements

- **Operating System**: Linux
- **ROS2**: Installed and configured (ROS2 Humble or later is recommended)
- **CMake**: Version 3.5 or higher
- **Compiler**: Support for C++17 or higher
- **Libraries**:
  - **rclcpp**: For developing ROS2 nodes
  - **sensor_msgs**: For `BatteryState` messages

## Installation

### 1. Install Dependencies

```
cd <your ros2 workspace>
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Make workspace if need

If you don't have a ROS2 workspace yet, create one:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. Clone this repository in workspace

Navigate to the src directory of your workspace and clone the project:

```
cd ~/ros2_ws/src
git clone https://github.com/ARDev1161/rc_agent.git
```

### 4. Build the Project

```
cd ~/ros2_ws
colcon build --packages-select rc_agent
```

### 5. Set Up the Environment

```
source install/setup.bash
```

## Usage

### Run the Node

To run the rc_agent node, execute the following command, specifying your serial port name:

```
ros2 run rc_agent rc_agent
```

Note: For simulation you should run this:

```
ros2 run rc_agent rc_agent --ros-args -p use_sim_time:=true
```


### Launch Parameters
- `BMS Type`: By default, M365BMSReader is used. Support for other BMS types can be added and selected via parameters.
- `Publishing Frequency`: The frequency at which battery data is published to the /battery_state topic (default is 10 Hz).

## Project Structure

```
.
├── CMakeLists.txt              # Build instructions for the project
├── package.xml                 # ROS 2 package metadata and dependencies
├── src/                        # Source code directory
│   ├── main.cpp                # Main entry point of the application
│   ├── network/                # Network communication module (gRPC, arping service)
│   ├── modules/                # Different functional modules
│   │   ├── battery/            # Battery state monitoring module
│   │   ├── map/                # Map processing module
│   │   ├── basecontrol/        # Base movement control module
│   │   ├── nav2/               # Navigation and path planning control module
│   ├── system/                 # System-related utilities and helpers
│   ├── grpc/                   # gRPC services implementation
│   ├── arper/                  # Broadcast based robot discovery service
└── README.md                   # This documentation file
```

## API

### Overview
This document describes the gRPC API used in the robot system. The API is structured into several services, handling different aspects of robot control, such as movement, navigation, sensor data, and power management.

### Proto Files Structure

The API consists of multiple `.proto` files:
- `robot.proto` - Main file that imports all modules and defines core messages.
- `base.proto` - Defines base control messages.
- `drives.proto` - Defines drive-related messages.
- `imu.proto` - Defines IMU sensor messages.
- `power.proto` - Defines power and battery-related messages.
- `map.proto` - Defines mapping and occupancy grid messages.
- `navigation.proto` - Defines navigation-related messages.
- `credentials.proto` - Defines authentication and security messages.

### Messages

#### Controls

```proto
message Controls {
    message Light {
        uint32 ledLeftPower = 1; // 0 to 255, LED intensity for left side
        uint32 ledRightPower = 2; // 0 to 255, LED intensity for right side
    }

    message System {
        bool haltFlag = 1; // Flag for shutdown
        bool restartFlag = 2; // Flag for reboot
        string onTime = 3; // Uptime string
    }

    Drives.CameraServos cameraServos = 1;
    Drives.WheelMotors wheelMotors = 2;
    Drives.HandMotors handMotors = 3;
    Base.BaseControl baseControl = 4;
    Navigation.NavCommandRequest navControl = 5;

    Light light = 6;
    System system = 7;
}
```

#### Sensors

```proto
message Sensors {
    Drives.WheelEncoders wheelEncoders = 1;
    Drives.HandEncoders handEncoders = 2;
    Base.Odometry odometry = 3;
    Imu.ImuData imuData = 4;
    Power.BatteryState batteryState = 5;
    Navigation.NavCommandResponse navControlStatus = 6;
}
```

#### Drives

```proto
message WheelMotors {
    float leftMotorSpeed = 1;
    float rightMotorSpeed = 2;
}

message WheelEncoders {
    int32 leftEncoder = 1;
    int32 rightEncoder = 2;
}

message CameraServos {
    float pan = 1;
    float tilt = 2;
}

message HandMotors {
    float gripStrength = 1;
}

message HandEncoders {
    float gripPosition = 1;
}
```

#### Base

```proto
message BaseControl {
    float linearVelocity = 1;
    float angularVelocity = 2;
}

message Odometry {
    float positionX = 1;
    float positionY = 2;
    float orientation = 3;
}
```

#### IMU

```proto
message ImuData {
    float orientationX = 1;
    float orientationY = 2;
    float orientationZ = 3;
    float orientationW = 4;
    float angularVelocityX = 5;
    float angularVelocityY = 6;
    float angularVelocityZ = 7;
    float linearAccelerationX = 8;
    float linearAccelerationY = 9;
    float linearAccelerationZ = 10;
}
```

#### Power

```proto
message BatteryState {
    float voltage = 1;
    float current = 2;
    float percentage = 3;
    bool charging = 4;
}
```

#### Navigation

```proto
message NavCommandRequest {
    message GoToPose {
        float x = 1;
        float y = 2;
        float orientation = 3;
    }
    GoToPose goToPose = 1;
}

message NavCommandResponse {
    string status = 1;
}
```

### Services

#### Robot Control Service

```proto
service ServerOnRobot {
    rpc DataExchange (Controls) returns (Sensors);
    rpc DataStreamExchange (stream Controls) returns (stream Sensors);
}
```

- **SendNavigationCommand(NavCommandRequest) -> NavCommandResponse** - Sends navigation goals to the robot.

#### Navigation Service

```proto
service NavigationService {
    rpc SendNavigationCommand (Navigation.NavCommandRequest) returns (Navigation.NavCommandResponse);
}
```

- **SendNavigationCommand(NavCommandRequest) -> NavCommandResponse** - Sends navigation goals to the robot.

#### Mapping Service

```proto
service MapService {
    rpc GetMap (map_service.GetMapRequest) returns (map_service.GetMapResponse);
}
```

- **GetMap(GetMapRequest) -> GetMapResponse** - Retrieves the current map of the environment.
