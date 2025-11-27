// 注意demo需要实际连接机械臂才能运行
// piper机械臂直线模式demo
// 注意机械臂工作空间内不要有障碍
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
  
  // 使能机械臂
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  // 在XOY平面上画正方形
  
  // 切换至MOVEP模式，移动到初始位置
  piper.motion_control_2(0x01, 0x00, 100);
  
  piper_sdk::CartesianPose pose;
  pose.position_um[0] = 150000;
  pose.position_um[1] = -50000;
  pose.position_um[2] = 150000;
  pose.orientation_mdeg[0] = -179900;
  pose.orientation_mdeg[1] = 0;
  pose.orientation_mdeg[2] = -179900;
  piper.move_cartesian(pose);
  std::this_thread::sleep_for(2s);
  
  // 切换至MOVEL模式
  piper.motion_control_2(0x01, 0x02, 100);
  
  // 第一条边
  pose.position_um[0] = 150000;
  pose.position_um[1] = 50000;
  pose.position_um[2] = 150000;
  piper.move_cartesian(pose);
  std::this_thread::sleep_for(2s);
  
  // 第二条边
  pose.position_um[0] = 250000;
  pose.position_um[1] = 50000;
  pose.position_um[2] = 150000;
  piper.move_cartesian(pose);
  std::this_thread::sleep_for(2s);
  
  // 第三条边
  pose.position_um[0] = 250000;
  pose.position_um[1] = -50000;
  pose.position_um[2] = 150000;
  piper.move_cartesian(pose);
  std::this_thread::sleep_for(2s);
  
  // 第四条边
  pose.position_um[0] = 150000;
  pose.position_um[1] = -50000;
  pose.position_um[2] = 150000;
  piper.move_cartesian(pose);
  
  std::cout << "正方形绘制完成！" << std::endl;
  
  return 0;
}

