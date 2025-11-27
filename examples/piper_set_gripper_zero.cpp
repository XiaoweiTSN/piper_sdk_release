// 注意demo需要实际连接机械臂才能运行
// 夹爪设定零点demo
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
  
  // 第一步：松开夹爪
  piper_sdk::GripperCommand gripper_cmd;
  gripper_cmd.position_um = 0;
  gripper_cmd.effort_mNm = 1000;
  gripper_cmd.gripper_code = 0x00; // 松开模式
  gripper_cmd.set_zero = 0;
  piper.control_gripper(gripper_cmd);
  
  std::this_thread::sleep_for(1500ms);
  
  // 第二步：设置零点并清除错误
  piper.set_gripper_zero();
  
  std::cout << "夹爪零点设置完成！" << std::endl;
  
  return 0;
}

