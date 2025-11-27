# Piper 机械臂 C++ SDK（预编译版）

[English](README_EN.md)

基于现代C++17标准实现的Piper机械臂SDK，这是预编译的二进制版本，包含头文件和静态/动态库。

**C++ 标准:** ![C++17](https://img.shields.io/badge/C++-17-blue.svg)

## 目录

- [编译示例程序](#编译示例程序)
- [CAN模块配置](#can模块配置)
- [快速开始](#快速开始)
- [链接到你的项目](#链接到你的项目)
- [API概览](#api概览)
- [示例程序](#示例程序)

## 编译示例程序

CMakeLists.txt 会自动检测系统架构（x86_64/aarch64）并编译所有示例程序：

```bash
mkdir build && cd build
cmake ..
make

# 运行示例
./examples/piper_ctrl_enable
./examples/piper_read_joint_state
```

## CAN模块配置

### 自动配置（推荐）

SDK提供了自动配置脚本`install_udev_rules.sh`，可以自动设置CAN接口的udev规则。

**主要功能：**
- 自动检测Jetson平台的mttcan接口并重命名为can99（避免占用can0）
- 自动配置gs_usb设备（如Piper兼容的CAN模块）为can0
- 自动设置波特率为1000000（1 Mbps）
- 插拔USB CAN模块时自动启动接口

**使用方法：**

```bash
sudo bash install_udev_rules.sh
```

脚本会引导你完成配置，配置后拔插USB CAN模块即可自动生效。

### 手动配置

如果需要手动配置CAN接口：

```bash
sudo apt update && sudo apt install can-utils
sudo ip link set can0 up type can bitrate 1000000
```

**注意:** Piper机械臂的波特率必须为**1000000**（1 Mbps）。

验证接口状态：
```bash
ip link show can0
```

## 快速开始

最小示例，演示如何使能机械臂并读取关节状态：

```cpp
#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

int main() {
    // 创建 SocketCAN 传输对象
    auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");
    
    // 创建 Piper 接口对象
    piper_sdk::PiperConfig config;
    piper_sdk::PiperInterface piper(transport, config);
    
    // 连接端口
    piper.connect_port(10ms);
    std::this_thread::sleep_for(100ms);
    
    // 使能机械臂
    while (!piper.enable_piper()) {
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "使能成功!" << std::endl;
    
    // 读取关节状态
    for (int i = 0; i < 100; i++) {
        auto joint_state = piper.get_arm_joint_messages();
        if (joint_state) {
            std::cout << "关节角度 (0.001度): ";
            for (auto pos : joint_state->state.position_mdeg) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(5ms);
    }
    
    // 失能机械臂
    piper.disable_piper();
    
    return 0;
}
```

## 链接到你的项目

### 使用 CMake

在你的项目的 `CMakeLists.txt` 中添加：

```cmake
cmake_minimum_required(VERSION 3.10)
project(my_piper_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 SDK 路径
set(PIPER_SDK_DIR "/opt/piper_sdk")

# 添加头文件路径
include_directories(${PIPER_SDK_DIR}/include)

# 检测系统架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(ARCH_DIR "x86_64")
else()
    message(FATAL_ERROR "不支持的架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 添加库文件路径
link_directories(${PIPER_SDK_DIR}/lib/${ARCH_DIR})

# 创建可执行文件
add_executable(my_app main.cpp)

# 链接 SDK 库（动态链接）
target_link_libraries(my_app piper_sdk)

# 或者使用静态链接
# target_link_libraries(my_app ${PIPER_SDK_DIR}/lib/${ARCH_DIR}/libpiper_sdk.a)
```

### 使用 g++ 命令行

```bash
# 动态链接
g++ -std=c++17 -I./include -L./lib/x86_64 -o my_app main.cpp -lpiper_sdk

# 静态链接
g++ -std=c++17 -I./include -o my_app main.cpp ./lib/x86_64/libpiper_sdk.a

# 运行时设置动态库路径
export LD_LIBRARY_PATH=./lib/x86_64:$LD_LIBRARY_PATH
```

## API概览

所有类型都在 `piper_sdk` 命名空间下。

#### 初始化

```cpp
// 创建传输层
auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");

// 配置参数
piper_sdk::PiperConfig config;
config.dh_is_offset = true;              // 使用修改的 DH 参数
config.enable_fk_calculation = true;     // 启用正运动学计算
config.logger_level = piper_sdk::LogLevel::kInfo;

// 创建接口
piper_sdk::PiperInterface piper(transport, config);

// 连接
piper.connect_port(10ms);
```

#### 基本控制

```cpp
// 使能/失能
bool success = piper.enable_piper();   // 使能所有电机
piper.disable_piper();                 // 失能所有电机
piper.enable_arm(motor_num);           // 使能指定电机（1-7）
piper.disable_arm(motor_num);          // 失能指定电机

// 其他控制
piper.reset_arm();                     // 重置机械臂
piper.emergency_stop();                // 紧急停止
piper.emergency_resume();              // 恢复
```

#### 运动控制

```cpp
// 关节控制（单位：0.001度）
std::vector<int32_t> joints = {0, 0, 0, 0, 0, 0};
piper.move_joint(joints);

// 笛卡尔控制（位置单位：0.001mm，姿态单位：0.001度）
piper.move_cartesian(x, y, z, rx, ry, rz);

// 设置运动模式
piper.motion_control_2(
    0x01,  // ctrl_mode: 0x01=CAN控制
    0x01,  // move_mode: 0x01=MoveJ
    50     // speed_rate: 0-100
);
```

#### 夹爪控制

```cpp
piper_sdk::GripperCommand cmd;
cmd.position_um = 50000;     // 位置：50mm
cmd.effort_mNm = 1000;       // 力矩：1N·m
cmd.gripper_code = 0x01;     // 0x01=使能
cmd.set_zero = 0x00;         // 0xAE=设置零点

piper.control_gripper(cmd);
```

#### 状态读取

所有 getter 返回 `std::optional<>` 类型：

```cpp
// 读取关节状态
auto joint_state = piper.get_arm_joint_messages();
if (joint_state) {
    // 使用 joint_state->state.position_mdeg
}

// 读取末端位姿
auto pose = piper.get_arm_end_pose_messages();
if (pose) {
    // 使用 pose->state [x, y, z, rx, ry, rz]
}
```

详细的 API 文档请参考头文件 `include/piper_sdk/piper_interface.hpp`。

## 示例程序

SDK 提供了丰富的示例程序，涵盖所有功能：

### 基础控制类

| 示例 | 说明 |
|------|------|
| `piper_ctrl_enable.cpp` | 使能机械臂 |
| `piper_ctrl_disable.cpp` | 失能机械臂 |
| `piper_ctrl_reset.cpp` | 重置机械臂 |
| `piper_ctrl_stop.cpp` | 紧急停止 |
| `piper_ctrl_go_zero.cpp` | 回到零位 |

### 运动控制类

| 示例 | 说明 |
|------|------|
| `piper_ctrl_gripper.cpp` | 控制夹爪开合 |
| `piper_ctrl_joint.cpp` | 控制关节角度 |
| `piper_ctrl_end_pose.cpp` | 控制末端位姿 |
| `piper_ctrl_moveL.cpp` | 直线运动 |
| `piper_ctrl_moveC.cpp` | 圆弧运动 |

### 状态读取类

| 示例 | 说明 |
|------|------|
| `piper_read_status.cpp` | 读取机械臂状态 |
| `piper_read_joint_state.cpp` | 读取关节状态 |
| `piper_read_end_pose.cpp` | 读取末端位姿 |
| `piper_read_all_fps.cpp` | 读取消息频率 |

更多示例请查看 `examples/README.md`。

