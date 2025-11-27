// 注意demo需要实际连接机械臂才能运行
// 控制机械臂关节运动
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

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
  
  // 使能机械臂
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  // 初始化夹爪
  piper_sdk::GripperCommand gripper_cmd;
  gripper_cmd.position_um = 0;
  gripper_cmd.effort_mNm = 1000;
  gripper_cmd.gripper_code = 0x01;
  gripper_cmd.set_zero = 0;
  piper.control_gripper(gripper_cmd);
  
  // 转换因子：57295.7795 = 1000*180/π
  constexpr double kFactor = 57295.7795;
  
  // 初始位置
  std::vector<double> position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int count = 0;
  
  // 循环控制关节运动
  while (true) {
    count++;
    
    // 根据计数改变目标位置
    if (count == 0) {
      std::cout << "1-----------" << std::endl;
      position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    } else if (count == 300) {
      std::cout << "2-----------" << std::endl;
      position = {0.2, 0.2, -0.2, 0.3, -0.2, 0.5, 0.08};
    } else if (count == 600) {
      std::cout << "1-----------" << std::endl;
      position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      count = 0;
    }
    
    // 转换为0.001度单位(millidegrees)
    std::vector<int32_t> joints(6);
    for (int i = 0; i < 6; i++) {
      joints[i] = static_cast<int32_t>(std::round(position[i] * kFactor));
    }
    int32_t joint_6 = static_cast<int32_t>(std::round(position[6] * 1000.0 * 1000.0)); // 夹爪位置，单位：0.001mm
    
    // 设置运动模式控制
    piper.motion_control_2(0x01, 0x01, 100);
    
    // 发送关节控制命令
    piper.move_joint(joints);
    
    // 控制夹爪
    gripper_cmd.position_um = std::abs(joint_6);
    piper.control_gripper(gripper_cmd);
    
    // 打印状态
    auto status = piper.get_arm_status();
    if (status) {
      std::cout << "机械臂状态: " << std::endl;
    }
    std::cout << "目标位置: [";
    for (size_t i = 0; i < position.size(); i++) {
      std::cout << position[i];
      if (i < position.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

