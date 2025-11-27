// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 单独控制某个电机的MIT模式
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
  
  // 使能机械臂
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  // 循环控制第6号电机的MIT模式
  while (true) {
    // 设置MIT模式
    // 设置MIT单关节模式
    piper.motion_control_2(0x01, 0x04, 0, 0xAD);
    
    // 位置 -0.5 rad
    piper.control_joint_mit(6, -0.5, 0.0, 10.0, 0.8, 0.0);
    std::cout << "1 - 位置: -0.5 rad" << std::endl;
    std::this_thread::sleep_for(1s);
    
    // 位置 0 rad
    piper.motion_control_2(0x01, 0x04, 0, 0xAD);
    piper.control_joint_mit(6, 0.0, 0.0, 10.0, 0.8, 0.0);
    std::cout << "2 - 位置: 0.0 rad" << std::endl;
    std::this_thread::sleep_for(1s);
    
    // 位置 0.5 rad
    piper.motion_control_2(0x01, 0x04, 0, 0xAD);
    piper.control_joint_mit(6, 0.5, 0.0, 10.0, 0.8, 0.0);
    std::cout << "3 - 位置: 0.5 rad" << std::endl;
    std::this_thread::sleep_for(1s);
  }
  
  return 0;
}

