// 注意demo需要实际连接机械臂才能运行
// 读取机械臂消息并打印
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
  
  // 循环读取并打印机械臂状态
  while (true) {
    auto status = piper.get_arm_status();
    if (status) {
      std::cout << "机械臂状态:" << std::endl;
      std::cout << "  关节位置 (0.001度): [";
      for (size_t i = 0; i < status->joints.position_mdeg.size(); i++) {
        std::cout << status->joints.position_mdeg[i];
        if (i < status->joints.position_mdeg.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      
      std::cout << "  关节力矩 (0.001 N·m): [";
      for (size_t i = 0; i < status->joints.effort_mNm.size(); i++) {
        std::cout << status->joints.effort_mNm[i];
        if (i < status->joints.effort_mNm.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      
      if (status->gripper_raw) {
        std::cout << "  夹爪位置: " << (*status->gripper_raw)[0] << " (0.001mm)" << std::endl;
        std::cout << "  夹爪力矩: " << (*status->gripper_raw)[1] << " (0.001 N·m)" << std::endl;
      }
    }
    
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

