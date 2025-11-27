// 注意demo需要实际连接机械臂才能运行
// 测试接口断开连接功能
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
  piper.connect_port(10ms, true);
  
  int count = 0;
  
  // 循环测试连接/断开
  while (true) {
    count++;
    std::cout << "--------------- " << count << " ---------------" << std::endl;
    
    // 在计数200-400之间断开连接
    if (count > 200 && count < 400) {
      piper.disconnect_port();
      std::cout << "已断开连接" << std::endl;
    } 
    // 在计数400之后重新连接
    else if (count > 400) {
      piper.connect_port(10ms, true);
      std::cout << "已重新连接" << std::endl;
    }
    
    // 打印连接状态和各项数据频率
    std::cout << "\n连接状态: " << (piper.is_ok() ? "正常" : "异常") << std::endl;
    std::cout << "CAN帧频率: " << piper.get_can_fps() << " Hz" << std::endl;
    
    auto status = piper.get_arm_status();
    std::cout << "机械臂状态: " << (status ? std::to_string(0) : "0") << " Hz" << std::endl;
    
    auto end_pose = piper.get_arm_end_pose_messages();
    std::cout << "末端位姿: " << (end_pose ? std::to_string(end_pose->hz) : "0") << " Hz" << std::endl;
    
    auto joint_states = piper.get_arm_joint_messages();
    std::cout << "关节状态: " << (joint_states ? std::to_string(joint_states->hz) : "0") << " Hz" << std::endl;
    
    auto gripper_msg = piper.get_arm_gripper_messages();
    std::cout << "夹爪消息: " << (gripper_msg ? std::to_string(gripper_msg->hz) : "0") << " Hz" << std::endl;
    
    auto high_spd = piper.get_motor_states();
    std::cout << "高速消息: " << (high_spd ? std::to_string(high_spd->hz) : "0") << " Hz" << std::endl;
    
    auto low_spd = piper.get_driver_states();
    std::cout << "低速消息: " << (low_spd ? std::to_string(low_spd->hz) : "0") << " Hz" << std::endl;
    
    auto joint_ctrl = piper.get_arm_joint_ctrl();
    std::cout << "关节控制: " << (joint_ctrl ? std::to_string(joint_ctrl->hz) : "0") << " Hz" << std::endl;
    
    auto gripper_ctrl = piper.get_arm_gripper_ctrl();
    std::cout << "夹爪控制: " << (gripper_ctrl ? std::to_string(gripper_ctrl->hz) : "0") << " Hz" << std::endl;
    
    auto ctrl_151 = piper.get_arm_ctrl_code151();
    std::cout << "控制151: " << (ctrl_151 ? std::to_string(ctrl_151->hz) : "0") << " Hz" << std::endl;
    
    std::cout << std::endl;
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

