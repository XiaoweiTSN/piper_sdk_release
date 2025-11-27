// 注意demo需要实际连接机械臂才能运行
// 使能机械臂
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
  std::this_thread::sleep_for(100ms);
  
  // 使能机械臂，循环直到成功
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  std::cout << "使能成功!!!!" << std::endl;
  
  return 0;
}

