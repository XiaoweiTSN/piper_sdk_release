// 注意demo需要实际连接灵巧手才能运行
// 控制灵巧手单个手指
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
  // 注意：SDK启动后约需50-100ms才能收到机械臂状态反馈(0x2A1)
  // 在此期间调用灵巧手函数会触发一次控制模式切换命令(0x151)
  // 对于长期运行的应用，这只会在启动时发生一次，不影响正常使用
  piper.connect_port(10ms);

  // 选择要控制的手指（可修改为其他手指）
  // kThumbTip=拇指尖, kThumbRoot=拇指根, kIndex=食指, kMiddle=中指, kRing=无名指, kPinky=小指
  auto finger = piper_sdk::DexFingerIndex::kRing;

  uint8_t position = 0;
  int count = 0;

  // 循环控制手指开合
  while (true) {
    count++;

    if (count == 300) {
      std::cout << "弯曲手指" << std::endl;
      position = 100;
    } else if (count == 600) {
      std::cout << "张开手指" << std::endl;
      position = 0;
      count = 0;
    }

    piper.move_dex_finger(finger, position);

    std::this_thread::sleep_for(5ms);
  }

  return 0;
}
