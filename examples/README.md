# Piper 机械臂 C++ SDK 示例程序

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![C++](https://img.shields.io/badge/C++-17-blue.svg)

本目录包含 Piper 机械臂 C++ SDK 的各种示例程序，展示了 SDK 的完整功能。

## 编译说明

### 使用 SDK 提供的 CMakeLists.txt

```bash
cd piper_sdk_release
mkdir build && cd build
cmake ..
make
```

编译后的可执行文件将位于 `build/examples/` 目录下。

### 单独编译某个示例

如果您只想编译某个示例程序，可以使用以下命令：

**动态链接：**
```bash
g++ -std=c++17 -I../include \
    -L../lib/x86_64 \
    -o piper_ctrl_enable piper_ctrl_enable.cpp \
    -lpiper_sdk
```

**静态链接：**
```bash
g++ -std=c++17 -I../include \
    -o piper_ctrl_enable piper_ctrl_enable.cpp \
    ../lib/x86_64/libpiper_sdk.a
```

（ARM64 平台请将 `x86_64` 替换为 `aarch64`）

## 示例程序列表

### 基础控制类

| 文件名 | 说明 |
|---|---|
| [`piper_ctrl_enable.cpp`](./piper_ctrl_enable.cpp) | 使能机械臂 |
| [`piper_ctrl_disable.cpp`](./piper_ctrl_disable.cpp) | 失能机械臂 |
| [`piper_ctrl_reset.cpp`](./piper_ctrl_reset.cpp) | 重置机械臂，需要在 MIT 或示教模式切换为位置速度控制模式时执行 |
| [`piper_ctrl_stop.cpp`](./piper_ctrl_stop.cpp) | 停止机械臂，使用后需要 reset 并重新使能两次 |
| [`piper_ctrl_go_zero.cpp`](./piper_ctrl_go_zero.cpp) | 控制机械臂回到零位 |

### 运动控制类

| 文件名 | 说明 |
|---|---|
| [`piper_ctrl_gripper.cpp`](./piper_ctrl_gripper.cpp) | 控制机械臂夹爪开合 |
| [`piper_ctrl_joint.cpp`](./piper_ctrl_joint.cpp) | 控制机械臂关节运动 |
| [`piper_ctrl_end_pose.cpp`](./piper_ctrl_end_pose.cpp) | 控制机械臂末端位姿 |
| [`piper_ctrl_line.cpp`](./piper_ctrl_line.cpp) | 控制机械臂直线运动模式 |
| [`piper_ctrl_moveJ.cpp`](./piper_ctrl_moveJ.cpp) | 控制机械臂关节运动（MoveJ 模式） |
| [`piper_ctrl_moveL.cpp`](./piper_ctrl_moveL.cpp) | 控制机械臂在 XOY 平面上画正方形（MoveL 模式） |
| [`piper_ctrl_moveC.cpp`](./piper_ctrl_moveC.cpp) | 控制机械臂圆弧运动（MoveC 模式） |
| [`piper_ctrl_moveP.cpp`](./piper_ctrl_moveP.cpp) | 控制机械臂末端执行器运动（MoveP 模式） |

### 状态读取类

| 文件名 | 说明 |
|---|---|
| [`piper_read_status.cpp`](./piper_read_status.cpp) | 读取机械臂状态信息 |
| [`piper_read_joint_state.cpp`](./piper_read_joint_state.cpp) | 读取关节和夹爪状态 |
| [`piper_read_end_pose.cpp`](./piper_read_end_pose.cpp) | 读取末端位姿 |
| [`piper_read_gripper_status.cpp`](./piper_read_gripper_status.cpp) | 读取夹爪状态 |
| [`piper_read_firmware.cpp`](./piper_read_firmware.cpp) | 读取固件版本 |
| [`piper_read_version.cpp`](./piper_read_version.cpp) | 读取 SDK 版本信息 |
| [`piper_read_fk.cpp`](./piper_read_fk.cpp) | 读取正向运动学计算结果 |
| [`piper_read_high_msg.cpp`](./piper_read_high_msg.cpp) | 读取高速消息（电机状态） |
| [`piper_read_low_msg.cpp`](./piper_read_low_msg.cpp) | 读取低速消息（驱动器状态） |
| [`piper_read_joint_ctrl.cpp`](./piper_read_joint_ctrl.cpp) | 读取关节控制消息 |
| [`piper_read_mode_ctrl.cpp`](./piper_read_mode_ctrl.cpp) | 读取模式控制消息 |
| [`piper_read_mode_ctrl_canid_151.cpp`](./piper_read_mode_ctrl_canid_151.cpp) | 读取 CAN ID 151 的模式控制消息 |
| [`piper_read_resp_instruction.cpp`](./piper_read_resp_instruction.cpp) | 读取指令应答消息 |
| [`piper_read_all_fps.cpp`](./piper_read_all_fps.cpp) | 读取各项数据的频率 |
| [`piper_read_arm_motor_max_angle_spd.cpp`](./piper_read_arm_motor_max_angle_spd.cpp) | 读取所有电机的最大角度和速度限制 |
| [`piper_read_arm_motor_max_acc_limit.cpp`](./piper_read_arm_motor_max_acc_limit.cpp) | 读取所有电机的最大加速度限制 |
| [`V2_piper_read_gripper_param_feedback.cpp`](./V2_piper_read_gripper_param_feedback.cpp) | 读取夹爪/示教器参数反馈 |

### 参数配置类

| 文件名 | 说明 |
|---|---|
| [`piper_set_load.cpp`](./piper_set_load.cpp) | 设置机械臂末端负载参数 |
| [`piper_set_joint_zero.cpp`](./piper_set_joint_zero.cpp) | 设置关节电机零点位置 |
| [`piper_set_gripper_zero.cpp`](./piper_set_gripper_zero.cpp) | 设置夹爪零点 |
| [`piper_set_log_level.cpp`](./piper_set_log_level.cpp) | 设置 SDK 日志级别 |
| [`piper_set_motor_angle_limit.cpp`](./piper_set_motor_angle_limit.cpp) | 设置电机角度限位 |
| [`piper_set_motor_max_acc_limit.cpp`](./piper_set_motor_max_acc_limit.cpp) | 设置电机最大加速度限制 |
| [`V2_piper_ctrl_motor_max_spd.cpp`](./V2_piper_ctrl_motor_max_spd.cpp) | 设置电机最大速度限制 |
| [`piper_set_sdk_param.cpp`](./piper_set_sdk_param.cpp) | 设置 SDK 关节和夹爪软件限位参数 |
| [`piper_set_init_default.cpp`](./piper_set_init_default.cpp) | 恢复所有参数为默认值 |
| [`V2_piper_set_gripper_param.cpp`](./V2_piper_set_gripper_param.cpp) | 设置夹爪/示教器参数 |
| [`V2_piper_set_installation_pos.cpp`](./V2_piper_set_installation_pos.cpp) | 设置机械臂安装位置 |

### 高级功能类

| 文件名 | 说明 |
|---|---|
| [`piper_set_master.cpp`](./piper_set_master.cpp) | 设置机械臂为主动臂 |
| [`piper_set_slave.cpp`](./piper_set_slave.cpp) | 设置机械臂为从动臂 |
| [`piper_set_mit.cpp`](./piper_set_mit.cpp) | 设置机械臂为 MIT 控制模式 |
| [`V2_piper_ctrl_joint_mit.cpp`](./V2_piper_ctrl_joint_mit.cpp) | 单独控制某个关节的 MIT 模式 |

### 测试类

| 文件名 | 说明 |
|---|---|
| [`piper_test_interface_disconnect.cpp`](./piper_test_interface_disconnect.cpp) | 测试接口断开连接功能 |
| [`piper_test_multi_interface_instance.cpp`](./piper_test_multi_interface_instance.cpp) | 测试多实例（多机械臂） |

## 使用说明

### 前置条件

#### 1. CAN 接口配置

确保 CAN 接口已正确启用（通常为 `can0`）：

```bash
# 查看 CAN 接口状态
ip link show can0

# 如果未启用，执行以下命令启用 CAN 接口
sudo ip link set can0 up type can bitrate 1000000
```

**重要：** Piper 机械臂的 CAN 波特率必须为 **1000000** (1 Mbps)。

#### 2. 硬件连接

确保机械臂已正确连接到 CAN 总线并已上电。

### 运行示例

#### 方法 1：从 build 目录运行

```bash
cd build/examples

# 运行使能机械臂示例
./piper_ctrl_enable

# 运行读取状态示例
./piper_read_status

# 运行关节控制示例
./piper_ctrl_joint
```

#### 方法 2：设置动态库路径后运行

如果使用动态链接，需要设置 `LD_LIBRARY_PATH`：

```bash
export LD_LIBRARY_PATH=/path/to/piper_sdk_release/lib/x86_64:$LD_LIBRARY_PATH
./piper_ctrl_enable
```

## 注意事项

### ⚠️ 安全警告

1. **运行任何控制类示例前，请确保机械臂周围没有障碍物和人员，以防意外伤害。**
2. **首次运行时，建议从小幅度、慢速度的运动开始，逐步熟悉机械臂行为。**

## 典型使用流程

### 基本控制流程

```
1. piper_ctrl_enable        # 使能机械臂
2. piper_read_joint_state   # 读取当前状态
3. piper_ctrl_joint         # 执行运动控制
4. piper_ctrl_disable       # 失能机械臂
```

### 运动控制流程

```
1. piper_ctrl_enable        # 使能机械臂
2. piper_ctrl_moveJ         # 设置运动模式（MoveJ/MoveL/MoveC）
3. piper_ctrl_joint         # 发送目标位置
4. piper_read_joint_state   # 监控运动状态
5. piper_ctrl_disable       # 完成后失能
```

### 模式切换流程

```
# 从位置模式切换到 MIT 模式
1. piper_ctrl_enable        # 使能机械臂
2. piper_set_mit            # 切换到 MIT 模式
3. V2_piper_ctrl_joint_mit  # MIT 控制
4. piper_ctrl_reset         # 切换回位置模式前必须 reset
5. piper_ctrl_enable        # 重新使能（需要两次）
6. piper_ctrl_enable        # 第二次使能
```

## 示例程序目录结构

```
examples/
├── README.md                           # 本文件
├── piper_ctrl_*.cpp                    # 控制类示例
├── piper_read_*.cpp                    # 读取类示例
├── piper_set_*.cpp                     # 设置类示例
├── piper_test_*.cpp                    # 测试类示例
└── V2_*.cpp                            # V2 版本特有示例
```

## 技术支持

如有问题，请参考：
- [SDK 主文档（中文）](../README.md)
- [SDK 主文档（英文）](../README_EN.md)
- 头文件中的详细注释：`../include/piper_sdk/piper_interface.hpp`

---

**免责声明：** 请仔细阅读 LICENSE 文件。使用本 SDK 时请注意安全，作者不对使用本软件造成的任何损失负责。

