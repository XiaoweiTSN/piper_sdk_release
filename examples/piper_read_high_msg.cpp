// 注意demo需要实际连接机械臂才能运行
// 读取机械臂高速消息并打印
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
  
  // 循环读取并打印高速消息
  while (true) {
    // 获取电机状态（高速消息）
    auto motor_states = piper.get_motor_states();
    if (motor_states) {
      std::cout << "电机高速消息:" << std::endl;
      for (size_t i = 0; i < motor_states->motor_speed_mrad_per_s.size(); i++) {
        std::cout << "  电机 " << (i + 1) << ":" << std::endl;
        std::cout << "    位置: " << motor_states->motor_position_rad[i] << " rad" << std::endl;
        std::cout << "    速度: " << motor_states->motor_speed_mrad_per_s[i] << " (0.001 rad/s)" << std::endl;
        std::cout << "    电流: " << motor_states->motor_current_ma[i] << " mA" << std::endl;
        std::cout << "    力矩: " << motor_states->motor_effort_mNm[i] << " (0.001 N·m)" << std::endl;
      }
      std::cout << "  频率: " << motor_states->hz << " Hz" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(100ms);
  }
  
  return 0;
}

