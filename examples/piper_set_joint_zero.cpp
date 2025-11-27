// 注意demo需要实际连接机械臂才能运行
// 设置关节电机零点位置
// 警告：设置零点前会对指定的电机失能，请保护好机械臂！
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
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
  std::this_thread::sleep_for(100ms);
  
  std::cout << "设置过程输入'q'可以退出程序" << std::endl;
  std::cout << "During setup, enter 'q' to exit the program" << std::endl;
  std::cout << "设置零点前会对指定的电机失能，请保护好机械臂" << std::endl;
  std::cout << "Before setting zero position, the specified motor will be disabled. Please protect the robotic arm." << std::endl;
  
  // 使能机械臂
  while (!piper.enable_piper()) {
    std::this_thread::sleep_for(10ms);
  }
  
  // 设置初始位置
  piper.motion_control_2(0x01, 0x01, 30);
  
  std::vector<int32_t> zero_joints = {0, 0, 0, 0, 0, 0};
  piper.move_joint(zero_joints);
  
  piper_sdk::GripperCommand gripper_cmd;
  gripper_cmd.position_um = 0;
  gripper_cmd.effort_mNm = 1000;
  gripper_cmd.gripper_code = 0x01;
  gripper_cmd.set_zero = 0;
  piper.control_gripper(gripper_cmd);
  
  int mode = -1;
  while (true) {
    // 模式选择
    if (mode == -1) {
      std::cout << "\nStep 1: 请选择设置模式(0: 指定电机; 1: 顺序设置): " << std::endl;
      std::cout << "Step 1: Select setting mode (0: Single motor; 1: Sequential setting): " << std::endl;
      std::string input;
      std::getline(std::cin, input);
      if (input == "0") {
        mode = 0;
      } else if (input == "1") {
        mode = 1;
      } else if (input == "q") {
        break;
      }
    }
    
    // 单电机设置
    else if (mode == 0) {
      std::cout << "\nStep 2: 输入需要设置零点的电机序号(1~7), 7代表所有电机: " << std::endl;
      std::cout << "Step 2: Enter motor number to set zero (1~7), 7 represents all motors: " << std::endl;
      std::string input;
      std::getline(std::cin, input);
      if (input == "q") {
        mode = -1;
        continue;
      }
      
      try {
        int motor_num = std::stoi(input);
        if (motor_num < 1 || motor_num > 7) {
          std::cout << "Tip: 输入超出范围" << std::endl;
          continue;
        }
        
        piper.disable_arm(motor_num);
        std::cout << "\nInfo: 第" << motor_num << "号电机失能成功，请手动纠正电机的零点位置" << std::endl;
        std::cout << "Info: Motor " << motor_num << " disabled successfully. Please manually adjust to zero position" << std::endl;
        
        std::cout << "\nStep 3: 回车设置第" << motor_num << "号电机零点: " << std::endl;
        std::cout << "Step 3: Press Enter to set zero for motor " << motor_num << ": " << std::endl;
        std::string confirm;
        std::getline(std::cin, confirm);
        if (confirm == "q") {
          mode = -1;
          continue;
        }
        
          piper.set_joint_zero(motor_num); // 设置零点
          piper.enable_arm(motor_num);
        std::cout << "\nInfo: 第" << motor_num << "号电机零点设置成功" << std::endl;
        std::cout << "Info: Motor " << motor_num << " zero position set successfully" << std::endl;
        
      } catch (...) {
        std::cout << "Tip: 请输入整数" << std::endl;
      }
    }
    
    // 顺序设置
    else if (mode == 1) {
      std::cout << "\nStep 2: 输入从第几号电机开始设置(1~6): " << std::endl;
      std::cout << "Step 2: Enter starting motor number (1~6): " << std::endl;
      std::string input;
      std::getline(std::cin, input);
      if (input == "q") {
        mode = -1;
        continue;
      }
      
      try {
        int motor_num = std::stoi(input);
        if (motor_num < 1 || motor_num > 6) {
          std::cout << "Tip: 输入超出范围" << std::endl;
          continue;
        }
        
        for (int i = motor_num; i <= 6; i++) {
          piper.disable_arm(i);
          std::cout << "\nInfo: 第" << i << "号电机失能成功，请手动纠正电机的零点位置" << std::endl;
          std::cout << "Info: Motor " << i << " disabled successfully. Please manually adjust to zero position" << std::endl;
          
          std::cout << "\nStep 3: 回车设置第" << i << "号电机零点: " << std::endl;
          std::cout << "Step 3: Press Enter to set zero for motor " << i << ": " << std::endl;
          std::string confirm;
          std::getline(std::cin, confirm);
          if (confirm == "q") {
            mode = -1;
            break;
          }
          
          piper.set_joint_zero(i); // 设置零点
          piper.enable_arm(i);
          std::cout << "\nInfo: 第" << i << "号电机零点设置成功" << std::endl;
          std::cout << "Info: Motor " << i << " zero position set successfully" << std::endl;
        }
        
      } catch (...) {
        std::cout << "Tip: 请输入整数" << std::endl;
      }
    }
  }
  
  return 0;
}

