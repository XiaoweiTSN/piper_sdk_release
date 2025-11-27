// 注意demo需要实际连接机械臂才能运行
// 读取机械臂所有电机的最大加速度限制
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
  
  // 循环查询并打印所有电机的最大加速度限制
  while (true) {
    // 查询所有电机的加速度限制
    piper.get_arm_motor_max_acc_limit();
    
    // 获取并打印结果
    auto acc_limits = piper.get_all_motor_acceleration_limits();
    if (acc_limits) {
      std::cout << "所有电机的最大加速度限制:" << std::endl;
      for (size_t i = 0; i < acc_limits->motors.size(); i++) {
        const auto& motor = acc_limits->motors[i];
        if (motor) {
          std::cout << "  电机 " << (i + 1) << ":" << std::endl;
          if (motor->max_joint_accel) {
            std::cout << "    最大加速度: " << *motor->max_joint_accel << " (0.01 rad/s²)" << std::endl;
          }
        }
      }
    }
    
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

