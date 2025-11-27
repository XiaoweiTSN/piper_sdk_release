// 注意demo需要实际连接机械臂才能运行
// 读取末端位姿并打印
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
  
  // 循环读取并打印末端位姿
  while (true) {
    auto end_pose = piper.get_arm_end_pose_messages();
    if (end_pose) {
      std::cout << "末端位姿:" << std::endl;
      std::cout << "  X: " << end_pose->pose.position_um[0] << " (0.001mm)" << std::endl;
      std::cout << "  Y: " << end_pose->pose.position_um[1] << " (0.001mm)" << std::endl;
      std::cout << "  Z: " << end_pose->pose.position_um[2] << " (0.001mm)" << std::endl;
      std::cout << "  RX: " << end_pose->pose.orientation_mdeg[0] << " (0.001度)" << std::endl;
      std::cout << "  RY: " << end_pose->pose.orientation_mdeg[1] << " (0.001度)" << std::endl;
      std::cout << "  RZ: " << end_pose->pose.orientation_mdeg[2] << " (0.001度)" << std::endl;
      std::cout << "  频率: " << end_pose->hz << " Hz" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

