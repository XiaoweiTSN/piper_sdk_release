// 注意demo需要实际连接机械臂才能运行
// 设置机械臂末端负载参数
#include <array>
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
  
  // 设置负载参数
  // mass_kg: 末端负载质量 (kg)
  //   - 质量 <= 0.0 kg: 负载等级 0
  //   - 质量 <= 0.5 kg: 负载等级 1
  //   - 质量 > 0.5 kg: 负载等级 2
  double mass_kg = 1.0;  // 设置为1.0kg，对应负载等级2
  std::array<double, 3> center_of_gravity_m = {0.0, 0.0, 0.0};  // 重心坐标 (m)
  
  auto result = piper.set_load_parameters(mass_kg, center_of_gravity_m);
  
  if (result.ok) {
    std::cout << "负载参数设置成功！质量: " << mass_kg << " kg" << std::endl;
  } else {
    std::cerr << "负载参数设置失败: " << result.message << std::endl;
  }
  
  return 0;
}

