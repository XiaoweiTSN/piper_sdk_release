# Piper Robot Arm C++ SDK (Pre-compiled)

[中文](README.md)

Piper robot arm SDK implemented in modern C++17. This is a pre-compiled binary version with header files and static/dynamic libraries.

**C++ Standard:** ![C++17](https://img.shields.io/badge/C++-17-blue.svg)

## Contents

- [Building Examples](#building-examples)
- [CAN Module Configuration](#can-module-configuration)
- [Quick Start](#quick-start)
- [Linking to Your Project](#linking-to-your-project)
- [API Overview](#api-overview)
- [Example Programs](#example-programs)

## Building Examples

CMakeLists.txt automatically detects system architecture (x86_64/aarch64) and builds all example programs:

```bash
mkdir build && cd build
cmake ..
make

# Run examples
./examples/piper_ctrl_enable
./examples/piper_read_joint_state
```

## CAN Module Configuration

### Automatic Configuration (Recommended)

The SDK provides an automatic configuration script `install_udev_rules.sh` to set up CAN interface udev rules.

**Features:**
- Auto-detect Jetson mttcan interface and rename to can99 (avoid occupying can0)
- Auto-configure gs_usb devices (like Piper-compatible CAN modules) as can0
- Auto-set bitrate to 1000000 (1 Mbps)
- Auto-start interface when plugging/unplugging USB CAN module

**Usage:**

```bash
sudo bash install_udev_rules.sh
```

The script will guide you through the configuration.

### Manual Configuration

If manual configuration is needed:

```bash
sudo apt update && sudo apt install can-utils
sudo ip link set can0 up type can bitrate 1000000
```

**Note:** Piper robot arm bitrate must be **1000000** (1 Mbps).

Verify interface status:
```bash
ip link show can0
```

## Quick Start

Minimal example to enable the robot arm and read joint states:

```cpp
#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

int main() {
    // Create SocketCAN transport object
    auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");
    
    // Create Piper interface object
    piper_sdk::PiperConfig config;
    piper_sdk::PiperInterface piper(transport, config);
    
    // Connect port
    piper.connect_port(10ms);
    std::this_thread::sleep_for(100ms);
    
    // Enable robot arm
    while (!piper.enable_piper()) {
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "Enabled successfully!" << std::endl;
    
    // Read joint states
    for (int i = 0; i < 100; i++) {
        auto joint_state = piper.get_arm_joint_messages();
        if (joint_state) {
            std::cout << "Joint angles (0.001 deg): ";
            for (auto pos : joint_state->state.position_mdeg) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(5ms);
    }
    
    // Disable robot arm
    piper.disable_piper();
    
    return 0;
}
```

## Linking to Your Project

### Using CMake

Add to your project's `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_piper_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set SDK path
set(PIPER_SDK_DIR "/opt/piper_sdk")

# Add include directories
include_directories(${PIPER_SDK_DIR}/include)

# Detect system architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(ARCH_DIR "x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Add library directories
link_directories(${PIPER_SDK_DIR}/lib/${ARCH_DIR})

# Create executable
add_executable(my_app main.cpp)

# Link SDK library (dynamic linking)
target_link_libraries(my_app piper_sdk)

# Or use static linking
# target_link_libraries(my_app ${PIPER_SDK_DIR}/lib/${ARCH_DIR}/libpiper_sdk.a)
```

### Using g++ Command Line

```bash
# Dynamic linking
g++ -std=c++17 -I./include -L./lib/x86_64 -o my_app main.cpp -lpiper_sdk

# Static linking
g++ -std=c++17 -I./include -o my_app main.cpp ./lib/x86_64/libpiper_sdk.a

# Set runtime library path
export LD_LIBRARY_PATH=./lib/x86_64:$LD_LIBRARY_PATH
```

## API Overview

All types are in the `piper_sdk` namespace.

#### Initialization

```cpp
// Create transport layer
auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");

// Configure parameters
piper_sdk::PiperConfig config;
config.dh_is_offset = true;              // Use modified DH parameters
config.enable_fk_calculation = true;     // Enable forward kinematics
config.logger_level = piper_sdk::LogLevel::kInfo;

// Create interface
piper_sdk::PiperInterface piper(transport, config);

// Connect
piper.connect_port(10ms);
```

#### Basic Control

```cpp
// Enable/Disable
bool success = piper.enable_piper();   // Enable all motors
piper.disable_piper();                 // Disable all motors
piper.enable_arm(motor_num);           // Enable specific motor (1-7)
piper.disable_arm(motor_num);          // Disable specific motor

// Other controls
piper.reset_arm();                     // Reset arm
piper.emergency_stop();                // Emergency stop
piper.emergency_resume();              // Resume
```

#### Motion Control

```cpp
// Joint control (unit: 0.001 degree)
std::vector<int32_t> joints = {0, 0, 0, 0, 0, 0};
piper.move_joint(joints);

// Cartesian control (position: 0.001mm, orientation: 0.001 degree)
piper.move_cartesian(x, y, z, rx, ry, rz);

// Set motion mode
piper.motion_control_2(
    0x01,  // ctrl_mode: 0x01=CAN control
    0x01,  // move_mode: 0x01=MoveJ
    50     // speed_rate: 0-100
);
```

#### Gripper Control

```cpp
piper_sdk::GripperCommand cmd;
cmd.position_um = 50000;     // Position: 50mm
cmd.effort_mNm = 1000;       // Torque: 1N·m
cmd.gripper_code = 0x01;     // 0x01=Enable
cmd.set_zero = 0x00;         // 0xAE=Set zero point

piper.control_gripper(cmd);
```

#### Status Reading

All getters return `std::optional<>` type:

```cpp
// Read joint state
auto joint_state = piper.get_arm_joint_messages();
if (joint_state) {
    // Use joint_state->state.position_mdeg
}

// Read end pose
auto pose = piper.get_arm_end_pose_messages();
if (pose) {
    // Use pose->state [x, y, z, rx, ry, rz]
}
```

For detailed API documentation, refer to the header file `include/piper_sdk/piper_interface.hpp`.

## Example Programs

The SDK provides comprehensive example programs covering all functionalities:

### Basic Control

| Example | Description |
|---------|-------------|
| `piper_ctrl_enable.cpp` | Enable robot arm |
| `piper_ctrl_disable.cpp` | Disable robot arm |
| `piper_ctrl_reset.cpp` | Reset robot arm |
| `piper_ctrl_stop.cpp` | Emergency stop |
| `piper_ctrl_go_zero.cpp` | Return to zero position |

### Motion Control

| Example | Description |
|---------|-------------|
| `piper_ctrl_gripper.cpp` | Control gripper |
| `piper_ctrl_joint.cpp` | Control joint angles |
| `piper_ctrl_end_pose.cpp` | Control end effector pose |
| `piper_ctrl_moveL.cpp` | Linear motion |
| `piper_ctrl_moveC.cpp` | Circular motion |

### Status Reading

| Example | Description |
|---------|-------------|
| `piper_read_status.cpp` | Read arm status |
| `piper_read_joint_state.cpp` | Read joint states |
| `piper_read_end_pose.cpp` | Read end pose |
| `piper_read_all_fps.cpp` | Read message frequencies |

For more examples, see `examples/README.md`.

