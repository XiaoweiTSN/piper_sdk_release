// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 设定安装位置
// 0x01: 水平正装
// 0x02: 侧装左
// 0x03: 侧装右
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
  
  // 设定安装位置为水平正装
  // 注意：设置安装位置使用 motion_control_2() 方法的 installation_pos 参数
  // 0x01: 水平正装, 0x02: 侧装左, 0x03: 侧装右
  
  auto result = piper.motion_control_2(0x01, 0x01, 0, 0x00, 0, 0x01);
  
  if (result.ok) {
    std::cout << "安装位置设置成功！" << std::endl;
    std::cout << "当前安装位置: 水平正装 (0x01)" << std::endl;
    std::cout << "\n其他选项:" << std::endl;
    std::cout << "  侧装左: 将 installation_pos 设置为 0x02" << std::endl;
    std::cout << "  侧装右: 将 installation_pos 设置为 0x03" << std::endl;
  } else {
    std::cerr << "安装位置设置失败: " << result.message << std::endl;
  }
  
  return 0;
}

