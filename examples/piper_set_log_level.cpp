// 注意demo需要实际连接机械臂才能运行
// 设置SDK日志级别并记录到文件
#include <chrono>
#include <iostream>
#include <memory>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

int main() {
  // 创建SocketCAN传输对象，连接到can0接口
  auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");
  
  // 创建Piper接口对象，设置日志级别和日志文件
  piper_sdk::PiperConfig config;
  config.can_name = "can0";
  config.logger_level = piper_sdk::LogLevel::kDebug; // 设置为DEBUG级别
  config.log_to_file = true; // 启用日志文件
  config.log_file_path = "/tmp/piper_sdk.log"; // 设置日志文件路径
  
  piper_sdk::PiperInterface piper(transport, config);
  
  // 连接端口
  piper.connect_port(10ms);
  
  std::cout << "日志级别已设置为 DEBUG" << std::endl;
  std::cout << "日志文件路径: " << config.log_file_path << std::endl;
  std::cout << "注意：日志将同时输出到控制台和文件" << std::endl;
  
  return 0;
}

