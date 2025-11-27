// 注意demo需要实际连接机械臂才能运行
// 读取机械臂各项数据的频率
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
  
  // 连接端口，启用线程
  piper.connect_port(10ms, true);
  
  // 循环读取并打印各项数据的频率
  while (true) {
    std::cout << "\n频率统计:" << std::endl;
    std::cout << "  连接状态: " << (piper.is_ok() ? "正常" : "异常") << std::endl;
    std::cout << "  总体CAN帧频率: " << piper.get_can_fps() << " Hz" << std::endl;
    
    auto end_pose = piper.get_arm_end_pose_messages();
    if (end_pose) {
      std::cout << "  末端位姿: " << end_pose->hz << " Hz" << std::endl;
    }
    
    auto joint_states = piper.get_arm_joint_messages();
    if (joint_states) {
      std::cout << "  关节状态: " << joint_states->hz << " Hz" << std::endl;
    }
    
    auto gripper_msg = piper.get_arm_gripper_messages();
    if (gripper_msg) {
      std::cout << "  夹爪消息: " << gripper_msg->hz << " Hz" << std::endl;
    }
    
    auto high_spd = piper.get_motor_states();
    if (high_spd) {
      std::cout << "  高速消息: " << high_spd->hz << " Hz" << std::endl;
    }
    
    auto low_spd = piper.get_driver_states();
    if (low_spd) {
      std::cout << "  低速消息: " << low_spd->hz << " Hz" << std::endl;
    }
    
    auto joint_ctrl = piper.get_arm_joint_ctrl();
    if (joint_ctrl) {
      std::cout << "  关节控制: " << joint_ctrl->hz << " Hz" << std::endl;
    }
    
    auto gripper_ctrl = piper.get_arm_gripper_ctrl();
    if (gripper_ctrl) {
      std::cout << "  夹爪控制: " << gripper_ctrl->hz << " Hz" << std::endl;
    }
    
    auto mode_ctrl = piper.get_arm_mode_ctrl();
    if (mode_ctrl) {
      std::cout << "  模式控制: " << mode_ctrl->hz << " Hz" << std::endl;
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

