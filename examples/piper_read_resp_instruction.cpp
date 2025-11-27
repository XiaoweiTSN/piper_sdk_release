// 注意demo需要实际连接机械臂才能运行
// 读取指令应答消息并打印
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
  
  std::cout << "1-----------" << std::endl;
  auto resp = piper.get_resp_instruction();
  if (resp) {
    std::cout << "初始应答: 指令码=0x" << std::hex << static_cast<int>(resp->code) 
              << ", 参数=0x" << static_cast<int>(resp->payload[0]) << std::dec << std::endl;
  }
  std::cout << "1-----------" << std::endl;
  
  // 使能机械臂
  piper.enable_arm();
  
  // 循环读取应答指令
  while (true) {
    std::cout << "------------" << std::endl;
    resp = piper.get_resp_instruction();
    if (resp) {
      std::cout << "应答指令:" << std::endl;
      std::cout << "  指令码: 0x" << std::hex << static_cast<int>(resp->code) << std::dec << std::endl;
      std::cout << "  参数1: 0x" << std::hex << static_cast<int>(resp->payload[0]) << std::dec << std::endl;
      std::cout << "  参数2: 0x" << std::hex << static_cast<int>(resp->payload[1]) << std::dec << std::endl;
      std::cout << "  参数3: 0x" << std::hex << static_cast<int>(resp->payload[2]) << std::dec << std::endl;
      
      // 捕获到设置指令0x471的应答时（使能机械臂发送的指令id为471）
      if (resp->code == 0x71) {
        std::cout << "捕获到使能指令应答（0x71）" << std::endl;
        // 等待3s后清除SDK保存的应答信息
        std::this_thread::sleep_for(3s);
        std::cout << "3-----------" << std::endl;
        piper.clear_resp_instruction();
        
        resp = piper.get_resp_instruction();
        if (resp) {
          std::cout << "清除后的应答: 0x" << std::hex << static_cast<int>(resp->code) << std::dec << std::endl;
        } else {
          std::cout << "应答已清除" << std::endl;
        }
        break;
      }
    } else {
      std::cout << "暂无应答指令" << std::endl;
    }
    std::cout << "------------" << std::endl;
    
    std::this_thread::sleep_for(5ms);
  }
  
  return 0;
}

