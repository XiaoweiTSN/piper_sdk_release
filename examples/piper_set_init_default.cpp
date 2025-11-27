// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 设置全部关节限位、关节最大速度、关节加速度为默认值
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
  
  // 恢复所有参数为默认值
  // 参数: param_enquiry=0x01, param_setting=0x02, data_feedback=0, end_load_effective=0, set_end_load=0x02
  auto result = piper.arm_param_enquiry_and_config(0x01, 0x02, 0, 0, 0x02);
  
  if (result.ok) {
    std::cout << "所有关节参数已恢复为默认值！" << std::endl;
  } else {
    std::cerr << "恢复默认值失败: " << result.message << std::endl;
  }
  
  // 循环读取并打印所有电机的角度和速度限制
  while (true) {
    piper.get_arm_motor_max_angle_speed();
    auto limits = piper.get_all_motor_limits();
    if (limits) {
      std::cout << "\n当前所有电机参数（已恢复默认）:" << std::endl;
      for (size_t i = 0; i < limits->motors.size(); i++) {
        const auto& motor = limits->motors[i];
        if (motor) {
          std::cout << "  电机 " << (i + 1) << ":" << std::endl;
          std::cout << "    角度范围: " << motor->min_angle_limit / 10.0
                    << "° ~ " << motor->max_angle_limit / 10.0 << "°" << std::endl;
          std::cout << "    最大速度: " << motor->max_joint_speed / 1000.0 << " rad/s" << std::endl;
        }
      }
    }
    std::this_thread::sleep_for(10ms);
  }
  
  return 0;
}

