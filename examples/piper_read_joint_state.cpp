// 注意demo需要实际连接机械臂才能运行
// 读取关节和夹爪状态并打印
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
  
  // 循环读取并打印关节和夹爪状态
  while (true) {
    // 读取关节状态
    auto joint_state = piper.get_arm_joint_messages();
    if (joint_state) {
      std::cout << "关节状态:" << std::endl;
      std::cout << "  位置 (0.001度): [";
      for (size_t i = 0; i < joint_state->state.position_mdeg.size(); i++) {
        std::cout << joint_state->state.position_mdeg[i];
        if (i < joint_state->state.position_mdeg.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      
      std::cout << "  力矩 (0.001 N·m): [";
      for (size_t i = 0; i < joint_state->state.effort_mNm.size(); i++) {
        std::cout << joint_state->state.effort_mNm[i];
        if (i < joint_state->state.effort_mNm.size() - 1) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
      
      std::cout << "  频率: " << joint_state->hz << " Hz" << std::endl;
    }
    
    // 读取夹爪状态
    auto gripper_state = piper.get_arm_gripper_messages();
    if (gripper_state) {
      std::cout << "夹爪状态:" << std::endl;
      std::cout << "  位置: " << gripper_state->state[0] << " (0.001mm)" << std::endl;
      std::cout << "  力矩: " << gripper_state->state[1] << " (0.001 N·m)" << std::endl;
      std::cout << "  频率: " << gripper_state->hz << " Hz" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

