// 注意demo需要实际连接机械臂才能运行
// 控制机械臂回到零位
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
  
  // 转换因子：57295.7795 = 1000*180/π
  constexpr double kFactor = 57295.7795;
  
  // 零位位置
  std::vector<double> position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // 转换为0.001度单位(millidegreese)
  std::vector<int32_t> joints(6);
  for (int i = 0; i < 6; i++) {
    joints[i] = static_cast<int32_t>(std::round(position[i] * kFactor));
  }
  int32_t joint_6 = static_cast<int32_t>(std::round(position[6] * 1000.0 * 1000.0)); // 夹爪位置，单位：0.001mm
  
  // 设置运动模式控制
  piper.motion_control_2(0x01, 0x01, 30);
  
  // 发送关节控制命令
  piper.move_joint(joints);
  
  // 控制夹爪
  piper_sdk::GripperCommand gripper_cmd;
  gripper_cmd.position_um = std::abs(joint_6);
  gripper_cmd.effort_mNm = 1000;
  gripper_cmd.gripper_code = 0x01;
  gripper_cmd.set_zero = 0;
  piper.control_gripper(gripper_cmd);
  
  return 0;
}

