// 注意demo需要实际连接机械臂才能运行
// 读取机械臂所有电机的最大角度和速度限制
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
  
  // 循环查询并打印所有电机的最大角度和速度限制
  while (true) {
    // 查询所有电机的角度和速度限制
    piper.get_arm_motor_max_angle_speed();
    
    // 获取并打印结果
    auto limits = piper.get_all_motor_limits();
    if (limits) {
      std::cout << "所有电机的角度和速度限制:" << std::endl;
      for (size_t i = 0; i < limits->motors.size(); i++) {
        const auto& motor = limits->motors[i];
        if (motor) {
          std::cout << "  电机 " << (i + 1) << ":" << std::endl;
          std::cout << "    最小角度: " << motor->min_angle_limit << " (0.1度)" << std::endl;
          std::cout << "    最大角度: " << motor->max_angle_limit << " (0.1度)" << std::endl;
          std::cout << "    最大速度: " << motor->max_joint_speed << " (0.001 rad/s)" << std::endl;
        }
      }
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

