// 注意demo需要实际连接机械臂才能运行
// 读取正向运动学计算结果
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
  
  // 创建Piper接口对象，启用FK计算
  piper_sdk::PiperConfig config;
  config.dh_is_offset = true;
  config.enable_fk_calculation = true;
  piper_sdk::PiperInterface piper(transport, config);
  
  // 连接端口
  piper.connect_port(10ms);
  
  // 使能FK计算
  // 注意：由于计算在单一线程中十分耗费资源，打开后会导致CPU占用率上升接近一倍
  piper.enable_fk_cal();
  
  // 循环读取并打印FK结果
  while (true) {
    // 获取基于反馈数据的FK
    auto fk_feedback = piper.get_forward_kinematics(piper_sdk::FKMode::kFeedback);
    if (fk_feedback) {
      std::cout << "FK (feedback) - Joint 6 位姿:" << std::endl;
      const auto& joint6_pose = (*fk_feedback)[5]; // 第6个关节（索引5）
      std::cout << "  X: " << joint6_pose.position_mm[0] << " mm" << std::endl;
      std::cout << "  Y: " << joint6_pose.position_mm[1] << " mm" << std::endl;
      std::cout << "  Z: " << joint6_pose.position_mm[2] << " mm" << std::endl;
      std::cout << "  RX: " << joint6_pose.orientation_deg[0] << " 度" << std::endl;
      std::cout << "  RY: " << joint6_pose.orientation_deg[1] << " 度" << std::endl;
      std::cout << "  RZ: " << joint6_pose.orientation_deg[2] << " 度" << std::endl;
    }
    
    // 获取基于控制命令的FK
    auto fk_control = piper.get_forward_kinematics(piper_sdk::FKMode::kControl);
    if (fk_control) {
      std::cout << "\nFK (control) - 所有关节:" << std::endl;
      for (size_t i = 0; i < fk_control->size(); i++) {
        const auto& pose = (*fk_control)[i];
        std::cout << "  Joint " << (i + 1) << ": [" 
                  << pose.position_mm[0] << ", " << pose.position_mm[1] << ", " << pose.position_mm[2] << ", "
                  << pose.orientation_deg[0] << ", " << pose.orientation_deg[1] << ", " << pose.orientation_deg[2] << "]" << std::endl;
      }
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

