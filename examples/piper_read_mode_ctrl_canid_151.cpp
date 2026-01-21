// 注意demo需要实际连接机械臂才能运行
// 读取主臂发送的模式控制消息（CAN ID 0x151）并打印
//
// 重要说明：
// - 此示例仅适用于【主从模式】，用于从臂读取主臂发送的 0x151 控制命令
// - 单机模式下机械臂不会发送 0x151 消息，此示例将无法获取数据
// - 如果要读取机械臂从臂模式下的控制模式，请使用 piper_read_mode_ctrl.cpp 示例
//   （通过 get_arm_status() 从 0x2A1 反馈中读取）
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

int main() {
  // 创建SocketCAN传输对象，连接到can0接口
  auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");
  
  // 创建Piper接口对象
  piper_sdk::PiperConfig config;
  piper_sdk::PiperInterface piper(transport, config);
  
  // 连接端口
  piper.connect_port(10ms);
  
  // 循环读取并打印CAN ID 151的模式控制消息
  while (true) {
    auto mode_ctrl = piper.get_arm_ctrl_code151();
    if (mode_ctrl) {
      std::cout << "CAN ID 151 模式控制消息:" << std::endl;
      std::cout << "  控制模式: 0x" << std::hex << static_cast<int>(mode_ctrl->params.ctrl_mode) << std::dec << std::endl;
      std::cout << "  运动模式: 0x" << std::hex << static_cast<int>(mode_ctrl->params.move_mode) << std::dec << std::endl;
      std::cout << "  速度百分比: " << static_cast<int>(mode_ctrl->params.speed_percent) << "%" << std::endl;
      std::cout << "  MIT模式: 0x" << std::hex << static_cast<int>(mode_ctrl->params.mit_mode) << std::dec << std::endl;
      std::cout << "  停留时间: " << static_cast<int>(mode_ctrl->params.residence_time) << std::endl;
      std::cout << "  安装位置: 0x" << std::hex << static_cast<int>(mode_ctrl->params.installation_pos) << std::dec << std::endl;
      std::cout << "  频率: " << mode_ctrl->hz << " Hz" << std::endl;
    } else {
      std::cout << "暂无CAN ID 151模式控制消息" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

