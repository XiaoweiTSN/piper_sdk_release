// 注意demo需要实际连接机械臂才能运行
// piper机械臂圆弧模式demo
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
  
  // 初始化夹爪
  piper_sdk::GripperCommand gripper_cmd;
  gripper_cmd.position_um = 0;
  gripper_cmd.effort_mNm = 1000;
  gripper_cmd.gripper_code = 0x01;
  gripper_cmd.set_zero = 0;
  piper.control_gripper(gripper_cmd);
  
  // 切换至MOVEC模式（圆弧运动模式）
  piper.motion_control_2(0x01, 0x03, 30);
  
  // 第一个点
  piper_sdk::CartesianPose pose;
  pose.position_um[0] = 135481;
  pose.position_um[1] = 9349;
  pose.position_um[2] = 161129;
  pose.orientation_mdeg[0] = 178756;
  pose.orientation_mdeg[1] = 6035;
  pose.orientation_mdeg[2] = -178440;
  piper.move_cartesian(pose);
  piper.move_circular(0x01); // 更新圆弧轨迹点1
  std::this_thread::sleep_for(1ms);
  
  // 第二个点
  pose.position_um[0] = 222158;
  pose.position_um[1] = 128758;
  pose.position_um[2] = 142126;
  pose.orientation_mdeg[0] = 175152;
  pose.orientation_mdeg[1] = -1259;
  pose.orientation_mdeg[2] = -157235;
  piper.move_cartesian(pose);
  piper.move_circular(0x02); // 更新圆弧轨迹点2
  std::this_thread::sleep_for(1ms);
  
  // 第三个点
  pose.position_um[0] = 359079;
  pose.position_um[1] = 3221;
  pose.position_um[2] = 153470;
  pose.orientation_mdeg[0] = 179038;
  pose.orientation_mdeg[1] = 1105;
  pose.orientation_mdeg[2] = 179035;
  piper.move_cartesian(pose);
  piper.move_circular(0x03); // 更新圆弧轨迹点3
  std::this_thread::sleep_for(1ms);
  
  // 再次发送运动模式控制，执行圆弧运动
  piper.motion_control_2(0x01, 0x03, 30);
  
  std::cout << "圆弧运动完成！" << std::endl;
  
  return 0;
}

