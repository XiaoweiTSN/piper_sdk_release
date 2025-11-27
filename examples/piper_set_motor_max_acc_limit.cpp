// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 单独设定某个电机的最大加速度
// 注意：这个指令是通过协议直接写入到驱动flash中，不可实时更新
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
  
  // 使能所有电机
  piper.enable_arm(7);
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  // 设置所有电机的最大加速度为 5 rad/s² (500 * 0.01 rad/s²)
  for (int i = 1; i <= 6; i++) {
    piper.set_motor_acceleration(i, 500);
    std::cout << "电机" << i << " 最大加速度设置为: 5 rad/s²" << std::endl;
    std::this_thread::sleep_for(500ms); // 数据的写入需要时间，发送完上一帧设定指令需要延时
  }
  
  std::cout << "\n加速度限制设置完成！" << std::endl;
  std::cout << "注意：这些设置已写入驱动器flash，重启后仍然有效" << std::endl;
  
  // 循环读取并打印所有电机的最大加速度限制
  while (true) {
    piper.get_arm_motor_max_acc_limit();
    auto acc_limits = piper.get_all_motor_acceleration_limits();
    if (acc_limits) {
      std::cout << "\n当前所有电机最大加速度限制:" << std::endl;
      for (size_t i = 0; i < acc_limits->motors.size(); i++) {
        const auto& motor = acc_limits->motors[i];
        if (motor && motor->max_joint_accel) {
          std::cout << "  电机 " << (i + 1) << ": " 
                    << *motor->max_joint_accel / 100.0 << " rad/s²" << std::endl;
        }
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  
  return 0;
}

