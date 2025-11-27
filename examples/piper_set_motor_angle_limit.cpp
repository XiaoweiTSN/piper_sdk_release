// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 单独设定某个电机的关节限位
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
  
  // 设置各电机的角度限位（单位：0.1度）
  // 电机1: ±150.0度
  piper.set_joint_angle_limits(1, -1500, 1500);
  std::cout << "电机1角度限位: -150.0° ~ 150.0°" << std::endl;
  
  // 电机2: 0° ~ 180.0度
  piper.set_joint_angle_limits(2, 0, 1800);
  std::cout << "电机2角度限位: 0° ~ 180.0°" << std::endl;
  
  // 电机3: -170.0度 ~ 0°
  piper.set_joint_angle_limits(3, -1700, 0);
  std::cout << "电机3角度限位: -170.0° ~ 0°" << std::endl;
  
  // 电机4: ±100.0度
  piper.set_joint_angle_limits(4, -1000, 1000);
  std::cout << "电机4角度限位: -100.0° ~ 100.0°" << std::endl;
  
  // 电机5: ±70.0度
  piper.set_joint_angle_limits(5, -700, 700);
  std::cout << "电机5角度限位: -70.0° ~ 70.0°" << std::endl;
  
  // 电机6: ±170.0度
  piper.set_joint_angle_limits(6, -1700, 1700);
  std::cout << "电机6角度限位: -170.0° ~ 170.0°" << std::endl;
  
  std::cout << "\n角度限位设置完成！" << std::endl;
  std::cout << "注意：这些设置已写入驱动器flash，重启后仍然有效" << std::endl;
  
  // 循环读取并打印所有电机的角度限位
  while (true) {
    piper.get_arm_motor_max_angle_speed();
    auto limits = piper.get_all_motor_limits();
    if (limits) {
      std::cout << "\n当前所有电机角度限位:" << std::endl;
      for (size_t i = 0; i < limits->motors.size(); i++) {
        const auto& motor = limits->motors[i];
        if (motor) {
          std::cout << "  电机 " << (i + 1) << ": " 
                    << motor->min_angle_limit / 10.0 << "° ~ " 
                    << motor->max_angle_limit / 10.0 << "°" << std::endl;
        }
      }
    }
    std::this_thread::sleep_for(100ms);
  }
  
  return 0;
}

