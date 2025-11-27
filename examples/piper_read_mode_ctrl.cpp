// 注意demo需要实际连接机械臂才能运行
// 读取模式控制消息并打印
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
  
  // 循环读取并打印模式控制消息
  while (true) {
    auto mode_ctrl = piper.get_arm_mode_ctrl();
    if (mode_ctrl) {
      std::cout << "模式控制消息 (CAN ID 0x151):" << std::endl;
      std::cout << "  控制模式: 0x" << std::hex << static_cast<int>(mode_ctrl->params.ctrl_mode) << std::dec << std::endl;
      std::cout << "  运动模式: 0x" << std::hex << static_cast<int>(mode_ctrl->params.move_mode) << std::dec << std::endl;
      std::cout << "  速度百分比: " << static_cast<int>(mode_ctrl->params.speed_percent) << "%" << std::endl;
      std::cout << "  MIT模式: 0x" << std::hex << static_cast<int>(mode_ctrl->params.mit_mode) << std::dec << std::endl;
      std::cout << "  停留时间: " << static_cast<int>(mode_ctrl->params.residence_time) << std::endl;
      std::cout << "  安装位置: 0x" << std::hex << static_cast<int>(mode_ctrl->params.installation_pos) << std::dec << std::endl;
      std::cout << "  频率: " << mode_ctrl->hz << " Hz" << std::endl;
    } else {
      std::cout << "暂无模式控制消息" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

