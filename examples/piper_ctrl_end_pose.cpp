// 注意demo需要实际连接机械臂才能运行
// 控制机械臂末端位姿
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
  
  // 转换因子
  constexpr double kFactor = 1000.0; // mm转换为0.001mm
  
  // 初始位置 [X, Y, Z, RX, RY, RZ, 夹爪]
  std::vector<double> position = {57.0, 0.0, 215.0, 0.0, 85.0, 0.0, 0.0};
  int count = 0;
  
  // 循环控制末端位姿
  while (true) {
    // 获取并打印末端位姿
    auto end_pose = piper.get_arm_end_pose_messages();
    if (end_pose) {
      std::cout << "末端位姿 X=" << end_pose->pose.position_um[0] 
                << " Y=" << end_pose->pose.position_um[1] 
                << " Z=" << end_pose->pose.position_um[2] 
                << " (单位：0.001mm)" << std::endl;
    }
    
    count++;
    
    // 根据计数改变目标位置
    if (count == 0) {
      std::cout << "1-----------" << std::endl;
      position = {57.0, 0.0, 215.0, 0.0, 85.0, 0.0, 0.0};
    } else if (count == 200) {
      std::cout << "2-----------" << std::endl;
      position = {57.0, 0.0, 260.0, 0.0, 85.0, 0.0, 0.0};
    } else if (count == 400) {
      std::cout << "1-----------" << std::endl;
      position = {57.0, 0.0, 215.0, 0.0, 85.0, 0.0, 0.0};
      count = 0;
    }
    
    // 转换为0.001mm和0.001度单位
    int32_t X = static_cast<int32_t>(std::round(position[0] * kFactor));
    int32_t Y = static_cast<int32_t>(std::round(position[1] * kFactor));
    int32_t Z = static_cast<int32_t>(std::round(position[2] * kFactor));
    int32_t RX = static_cast<int32_t>(std::round(position[3] * kFactor));
    int32_t RY = static_cast<int32_t>(std::round(position[4] * kFactor));
    int32_t RZ = static_cast<int32_t>(std::round(position[5] * kFactor));
    int32_t joint_6 = static_cast<int32_t>(std::round(position[6] * kFactor));
    
    std::cout << "目标位姿: X=" << X << " Y=" << Y << " Z=" << Z 
              << " RX=" << RX << " RY=" << RY << " RZ=" << RZ << std::endl;
    
    // 设置运动模式控制 (MOVEP模式)
    piper.motion_control_2(0x01, 0x00, 100);
    
    // 发送末端位姿控制命令
    piper_sdk::CartesianPose pose;
    pose.position_um[0] = X;
    pose.position_um[1] = Y;
    pose.position_um[2] = Z;
    pose.orientation_mdeg[0] = RX;
    pose.orientation_mdeg[1] = RY;
    pose.orientation_mdeg[2] = RZ;
    piper.move_cartesian(pose);
    
    // 控制夹爪
    gripper_cmd.position_um = std::abs(joint_6);
    piper.control_gripper(gripper_cmd);
    
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

