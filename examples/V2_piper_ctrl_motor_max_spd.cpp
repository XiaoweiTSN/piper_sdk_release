// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 单独设定某个电机的最大速度
// 注意：这个指令是通过协议直接写入到驱动flash中，不可实时更新
// 如果需要动态调整速度，请使用位置速度模式中的速度百分比
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
  
  // 使能机械臂
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  // 设置所有电机的最大速度为 3 rad/s (3000 * 0.001 rad/s)
  for (int i = 1; i <= 6; i++) {
    piper.set_motor_speed_limit(i, 3000);
    std::cout << "电机" << i << " 最大速度设置为: 3 rad/s" << std::endl;
    std::this_thread::sleep_for(100ms);
  }
  
  std::cout << "\n速度限制设置完成！" << std::endl;
  std::cout << "注意：这些设置已写入驱动器flash，重启后仍然有效" << std::endl;
  
  // 循环读取并打印所有电机的角度和速度限制
  while (true) {
    piper.get_arm_motor_max_angle_speed();
    auto limits = piper.get_all_motor_limits();
    if (limits) {
      std::cout << "\n当前所有电机速度限制:" << std::endl;
      for (size_t i = 0; i < limits->motors.size(); i++) {
        const auto& motor = limits->motors[i];
        if (motor) {
          std::cout << "  电机 " << (i + 1) << ": " 
                    << motor->max_joint_speed / 1000.0 << " rad/s" << std::endl;
        }
      }
    }
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

