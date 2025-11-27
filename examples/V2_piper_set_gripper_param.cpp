// 注意demo需要实际连接机械臂才能运行
// V2版本SDK - 夹爪/示教器参数设置指令
// 第一次使用夹爪或者示教器时，需要设定一下这两个末端执行器参数
// 否则会出现数据没有反馈并且执行器无法控制的情况
// 一般情况下 max_range_config 是 70
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
  
  // 配置夹爪/示教器参数
  uint8_t teaching_range_per = 100;  // 示教器行程系数 [100~200]
  uint8_t max_range_config = 70;     // 最大控制行程 (0,70,100)
  uint8_t teaching_friction = 1;     // 摩擦系数 [1~10]
  
  auto result = piper.configure_gripper_teaching_pendant(teaching_range_per, 
                                                         max_range_config, 
                                                         teaching_friction);
  
  if (result.ok) {
    std::cout << "夹爪/示教器参数配置成功！" << std::endl;
    std::cout << "  示教器行程系数: " << static_cast<int>(teaching_range_per) << std::endl;
    std::cout << "  最大控制行程: " << static_cast<int>(max_range_config) << std::endl;
    std::cout << "  摩擦系数: " << static_cast<int>(teaching_friction) << std::endl;
  } else {
    std::cerr << "参数配置失败: " << result.message << std::endl;
  }
  
  // 查询参数
  piper.arm_param_enquiry_and_config(4); // param_enquiry=4
  
  // 循环读取并打印夹爪参数反馈
  while (true) {
    auto feedback = piper.get_gripper_teaching_pendant_param();
    if (feedback) {
      std::cout << "\n夹爪/示教器参数反馈:" << std::endl;
      std::cout << "  示教器行程系数: " << static_cast<int>(feedback->raw[0]) << std::endl;
      std::cout << "  最大控制行程: " << static_cast<int>(feedback->raw[1]) << std::endl;
      std::cout << "  摩擦系数: " << static_cast<int>(feedback->raw[2]) << std::endl;
    } else {
      std::cout << "等待参数反馈..." << std::endl;
    }
    
    std::this_thread::sleep_for(50ms);
  }
  
  return 0;
}

