// 注意demo需要实际连接机械臂才能运行
// 读取机械臂低速消息并打印
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
  
  // 循环读取并打印低速消息
  while (true) {
    // 获取驱动器状态（低速消息）
    auto driver_states = piper.get_driver_states();
    if (driver_states) {
      std::cout << "驱动器低速消息:" << std::endl;
      for (size_t i = 0; i < driver_states->drivers.size(); i++) {
        const auto& driver = driver_states->drivers[i];
        std::cout << "  驱动器 " << (i + 1) << ":" << std::endl;
        std::cout << "    总线电压: " << driver.bus_voltage_decivolts << " (0.1 V)" << std::endl;
        std::cout << "    FOC温度: " << driver.foc_temperature_c << " °C" << std::endl;
        std::cout << "    电机温度: " << static_cast<int>(driver.motor_temperature_c) << " °C" << std::endl;
        std::cout << "    状态码: 0x" << std::hex << static_cast<int>(driver.status_code) << std::dec << std::endl;
        std::cout << "    总线电流: " << driver.bus_current_ma << " mA" << std::endl;
        std::cout << "    使能状态: " << (driver.flags.driver_enable_status ? "已使能" : "未使能") << std::endl;
      }
      std::cout << "  频率: " << driver_states->hz << " Hz" << std::endl;
    }
    
    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

