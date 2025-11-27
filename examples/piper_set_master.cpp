// 注意demo需要实际连接机械臂才能运行
// 设置机械臂为主动臂，直接设置即可
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
  
  // 配置为主动臂
  auto result = piper.configure_master_slave(
    0xFA,  // linkage_config: 主动臂模式
    0x00,  // feedback_offset
    0x00,  // ctrl_offset
    0x00   // linkage_offset
);

  if (result.ok) {
    std::cout << "成功设置为主动臂模式！" << std::endl;
  } else {
    std::cerr << "设置主动臂模式失败: " << result.message << std::endl;
  }
  
  return 0;
}

