// 注意demo需要实际连接机械臂才能运行
// 控制机械臂夹爪
#include <chrono>
#include <cmath>
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
  
  // 初始化夹爪（先松开再使能）
  piper_sdk::GripperCommand gripper_cmd;
  gripper_cmd.position_um = 0;
  gripper_cmd.effort_mNm = 1000;
  gripper_cmd.gripper_code = 0x02; // 松开
  gripper_cmd.set_zero = 0;
  piper.control_gripper(gripper_cmd);
  
  gripper_cmd.gripper_code = 0x01; // 使能
  piper.control_gripper(gripper_cmd);
  
  int32_t range = 0;
  int count = 0;
  
  // 循环控制夹爪开合
  while (true) {
    // 获取并打印夹爪状态
    auto gripper_state = piper.get_arm_gripper_messages();
    if (gripper_state) {
      std::cout << "夹爪位置: " << gripper_state->state[0] << " (0.001mm)" << std::endl;
    }
    
    count++;
    
    // 根据计数改变夹爪位置
    if (count == 0) {
      std::cout << "1-----------" << std::endl;
      range = 0;
    } else if (count == 300) {
      std::cout << "2-----------" << std::endl;
      range = static_cast<int32_t>(0.05 * 1000.0 * 1000.0); // 0.05m = 50mm，转换为0.001mm单位
    } else if (count == 600) {
      std::cout << "3-----------" << std::endl;
      range = 0;
      count = 0;
    }
    
    // 发送夹爪控制命令
    gripper_cmd.position_um = std::abs(range);
    gripper_cmd.effort_mNm = 1000;
    gripper_cmd.gripper_code = 0x01;
    gripper_cmd.set_zero = 0;
    piper.control_gripper(gripper_cmd);
    
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

