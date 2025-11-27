// 注意demo需要实际连接机械臂才能运行
// 读取机械臂固件版本并打印
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
  
  // 需要时间去读取固件反馈帧
  std::this_thread::sleep_for(30ms);
  
  // 获取并打印固件版本
  auto firmware_version = piper.get_cached_firmware_version();
  if (firmware_version) {
    std::cout << "固件版本: " << *firmware_version << std::endl;
  } else {
    std::cout << "无法读取固件版本" << std::endl;
  }
  
  return 0;
}

