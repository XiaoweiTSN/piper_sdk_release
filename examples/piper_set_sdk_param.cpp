// 注意demo需要实际连接机械臂才能运行
// 设置SDK关节和夹爪软件限位参数
#include <chrono>
#include <iostream>
#include <memory>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

int main() {
  // 创建SocketCAN传输对象，连接到can0接口
  auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");
  
  // 创建Piper接口对象，启用SDK软件限位
  piper_sdk::PiperConfig config;
  config.can_name = "can0";
  
  // 设置关节软件限位 (6个关节，单位：0.001度)
  config.software_limits.joint_min_mdeg = {-120000, -120000, -120000, -120000, -120000, -120000};
  config.software_limits.joint_max_mdeg = {120000, 120000, 120000, 120000, 120000, 120000};
  config.software_limits.enable_joint_limits = true;
  
  // 设置夹爪软件限位 (单位：0.001mm)
  config.software_limits.gripper_min_um = 0;      // 0mm
  config.software_limits.gripper_max_um = 50000;  // 50mm
  config.software_limits.enable_gripper_limits = true;
  
  piper_sdk::PiperInterface piper(transport, config);
  
  // 连接端口
  piper.connect_port(10ms);
  
  std::cout << "SDK软件限位参数已设置:" << std::endl;
  std::cout << "  关节限位: 已启用，范围 -120° ~ 120°" << std::endl;
  std::cout << "  夹爪限位: 已启用，范围 0 ~ 50mm" << std::endl;
  
  return 0;
}
