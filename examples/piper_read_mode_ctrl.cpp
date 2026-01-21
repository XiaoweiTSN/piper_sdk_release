// 注意demo需要实际连接机械臂才能运行
// 读取机械臂控制模式并打印
// 控制模式来自 0x2A1 (ARM_STATUS_FEEDBACK) 的反馈
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

// 控制模式名称映射
const char* get_ctrl_mode_name(uint8_t mode) {
  switch (mode) {
    case 0x00: return "待机模式";
    case 0x01: return "CAN指令控制模式";
    case 0x02: return "示教模式";
    case 0x03: return "以太网控制模式";
    case 0x04: return "WiFi控制模式";
    case 0x05: return "遥控器控制模式";
    case 0x07: return "离线轨迹模式";
    default: return "未知模式";
  }
}

// 运动模式名称映射
const char* get_move_mode_name(uint8_t mode) {
  switch (mode) {
    case 0x00: return "MOVE P (点到点)";
    case 0x01: return "MOVE J (关节空间)";
    case 0x02: return "MOVE L (直线)";
    case 0x03: return "MOVE C (圆弧)";
    case 0x04: return "MOVE M (MIT)";
    case 0x05: return "MOVE CPV";
    default: return "未知";
  }
}

int main() {
  // 创建SocketCAN传输对象，连接到can0接口
  auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");

  // 创建Piper接口对象
  piper_sdk::PiperConfig config;
  piper_sdk::PiperInterface piper(transport, config);

  // 连接端口
  piper.connect_port(10ms);

  std::cout << "=== 机械臂控制模式读取 ===" << std::endl;
  std::cout << "数据来源: CAN ID 0x2A1 (ARM_STATUS_FEEDBACK)" << std::endl;
  std::cout << std::endl;

  // 循环读取并打印控制模式
  while (true) {
    auto status = piper.get_arm_status();
    if (status) {
      std::cout << "机械臂状态反馈 (0x2A1):" << std::endl;
      std::cout << "  控制模式: 0x" << std::hex << static_cast<int>(status->control_mode)
                << std::dec << " (" << get_ctrl_mode_name(status->control_mode) << ")" << std::endl;
      std::cout << "  机械臂状态: 0x" << std::hex << static_cast<int>(status->arm_status) << std::dec << std::endl;
      std::cout << "  运动模式: 0x" << std::hex << static_cast<int>(status->mode_feedback)
                << std::dec << " (" << get_move_mode_name(status->mode_feedback) << ")" << std::endl;
      std::cout << "  示教状态: 0x" << std::hex << static_cast<int>(status->teach_status) << std::dec << std::endl;
      std::cout << "  运动状态: 0x" << std::hex << static_cast<int>(status->motion_status) << std::dec << std::endl;
      std::cout << "  轨迹编号: " << static_cast<int>(status->trajectory_num) << std::endl;
      std::cout << "  错误代码: " << status->error_code << std::endl;
      std::cout << "  使能状态: " << (status->enabled ? "已使能" : "未使能") << std::endl;
    } else {
      std::cout << "暂无机械臂状态反馈" << std::endl;
    }

    // 同时显示 SDK 发送的最后控制参数（本地缓存）
    auto last_params = piper.get_last_motion_ctrl_code151();
    std::cout << "SDK发送的最后控制参数 (0x151 本地缓存):" << std::endl;
    std::cout << "  控制模式: 0x" << std::hex << static_cast<int>(last_params.ctrl_mode)
              << std::dec << " (" << get_ctrl_mode_name(last_params.ctrl_mode) << ")" << std::endl;
    std::cout << "  运动模式: 0x" << std::hex << static_cast<int>(last_params.move_mode)
              << std::dec << " (" << get_move_mode_name(last_params.move_mode) << ")" << std::endl;
    std::cout << "  速度百分比: " << static_cast<int>(last_params.speed_percent) << "%" << std::endl;

    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(500ms);
  }

  return 0;
}
