// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 读取夹爪/示教器参数反馈
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
  
  // 循环查询并打印夹爪/示教器参数反馈
  while (true) {
    // 查询参数
    piper.arm_param_enquiry_and_config(4); // param_enquiry=4
    
    // 获取并打印反馈
    auto feedback = piper.get_gripper_teaching_pendant_param();
    if (feedback) {
      std::cout << "夹爪/示教器参数反馈:" << std::endl;
      std::cout << "  电流限制: " << static_cast<int>(feedback->raw[0]) << std::endl;
      std::cout << "  最大范围: " << static_cast<int>(feedback->raw[1]) << std::endl;
      std::cout << "  使能标志: " << static_cast<int>(feedback->raw[2]) << std::endl;
      std::cout << "-------------------" << std::endl;
    } else {
      std::cout << "等待参数反馈..." << std::endl;
    }
    
    std::this_thread::sleep_for(50ms);
  }
  
  return 0;
}

