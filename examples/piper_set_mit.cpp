// 注意demo需要实际连接机械臂才能运行
// 设置机械臂为MIT控制模式，这个模式下机械臂响应最快
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
  
  // 循环设置MIT模式
  while (true) {
    // 设置运动模式控制 (MIT模式)
    auto result = piper.motion_control_2(0x01, 0x01, 0, 0xAD);
    
    if (result.ok) {
      std::cout << "MIT模式设置成功" << std::endl;
    } else {
      std::cerr << "MIT模式设置失败: " << result.message << std::endl;
    }
    
    std::this_thread::sleep_for(1s);
  }
  
  return 0;
}

