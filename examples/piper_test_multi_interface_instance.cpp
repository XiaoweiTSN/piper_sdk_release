// 注意demo需要实际连接机械臂才能运行
// 当有多个机械臂的时候，可以创建多实例
// 注意需要多个CAN模块
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

int main() {
  // 创建第一个机械臂实例（连接到can0）
  auto transport0 = std::make_shared<piper_sdk::SocketCanTransport>("can0");
  piper_sdk::PiperConfig config0;
  config0.can_name = "can0";
  piper_sdk::PiperInterface piper0(transport0, config0);
  piper0.connect_port(10ms, true);
  
  std::cout << "机械臂1已连接到can0" << std::endl;
  
  // 创建第二个机械臂实例（连接到can1）
  auto transport1 = std::make_shared<piper_sdk::SocketCanTransport>("can1");
  piper_sdk::PiperConfig config1;
  config1.can_name = "can1";
  piper_sdk::PiperInterface piper1(transport1, config1);
  piper1.connect_port(10ms, true);
  
  std::cout << "机械臂2已连接到can1" << std::endl;
  
  // 循环打印两个机械臂的CAN帧频率
  while (true) {
    std::cout << "\n=== 多实例状态 ===" << std::endl;
    std::cout << "机械臂1 (can0) CAN帧频率: " << piper0.get_can_fps() << " Hz" << std::endl;
    std::cout << "机械臂1 连接状态: " << (piper0.is_ok() ? "正常" : "异常") << std::endl;
    
    std::cout << "机械臂2 (can1) CAN帧频率: " << piper1.get_can_fps() << " Hz" << std::endl;
    std::cout << "机械臂2 连接状态: " << (piper1.is_ok() ? "正常" : "异常") << std::endl;
    
    std::this_thread::sleep_for(1s);
  }
  
  return 0;
}

