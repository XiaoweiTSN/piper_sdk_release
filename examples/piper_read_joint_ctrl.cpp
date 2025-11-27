// 注意demo需要实际连接机械臂才能运行
// 读取关节控制消息并打印
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
  
  // 循环读取并打印关节控制消息
  while (true) {
    auto joint_ctrl = piper.get_arm_joint_ctrl();
    if (joint_ctrl) {
      std::cout << "关节控制消息:" << std::endl;
      std::cout << "  关节位置 (0.001度): [";
      for (size_t i = 0; i < joint_ctrl->ctrl.joints_mdeg.size(); i++) {
        std::cout << joint_ctrl->ctrl.joints_mdeg[i];
        if (i < joint_ctrl->ctrl.joints_mdeg.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      std::cout << "  频率: " << joint_ctrl->hz << " Hz" << std::endl;
    } else {
      std::cout << "暂无关节控制消息" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

