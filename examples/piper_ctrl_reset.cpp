// 注意demo需要实际连接机械臂才能运行
// 设置机械臂重置，需要在mit或者示教模式切换为位置速度控制模式时执行
#include <chrono>
#include <iostream>
#include <memory>

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
  
  // 恢复(0x02)
  piper.motion_control_1(0x02, 0, 0);
  
  return 0;
}

