// 注意demo需要实际连接机械臂才能运行
// 读取SDK各模块软件版本
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
  
  // 连接端口（启用初始化和线程）
  piper.connect_port(10ms, true, true);
  
  // 需要时间去读取固件反馈帧
  std::this_thread::sleep_for(100ms);
  
  // 注意：C++ SDK没有像Python SDK那样的GetCurrentInterfaceVersion等接口
  // 这里我们打印SDK相关的版本信息
  
  std::cout << "=====>> Piper C++ SDK 版本信息 <<=====" << std::endl;
  
  // 获取固件版本
  auto firmware_version = piper.get_cached_firmware_version();
  if (firmware_version) {
    std::cout << "Piper 固件版本: " << *firmware_version << std::endl;
  } else {
    std::cout << "Piper 固件版本: 读取失败（错误码：-0x4AF）" << std::endl;
    std::cout << "提示：确保 connect_port(piper_init=true, start_thread=true) 并等待足够时间" << std::endl;
  }
  
  std::cout << "\n注意：C++ SDK的版本信息可以在CMakeLists.txt或README.md中查看" << std::endl;
  
  return 0;
}

