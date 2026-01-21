// 注意demo需要实际连接灵巧手才能运行
// 读取灵巧手状态（左右手标识、电机状态）
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

const char* get_status_name(piper_sdk::DexFingerStatus status) {
  switch (status) {
    case piper_sdk::DexFingerStatus::kIdle:
      return "空闲";
    case piper_sdk::DexFingerStatus::kRunning:
      return "运行";
    case piper_sdk::DexFingerStatus::kStalled:
      return "堵转";
    default:
      return "未知";
  }
}

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

  // 循环读取并打印状态
  const char* finger_names[] = {"拇指尖", "拇指根", "食指", "中指", "无名指", "小指"};

  while (true) {
    auto status = piper.get_dex_hand_status();
    if (status) {
      // 左右手标识
      std::cout << "左右手: ";
      if (status->hand_side == piper_sdk::DexHandSide::kLeft) {
        std::cout << "左手";
      } else if (status->hand_side == piper_sdk::DexHandSide::kRight) {
        std::cout << "右手";
      } else {
        std::cout << "未知";
      }
      std::cout << std::endl;

      // 电机状态
      std::cout << "电机状态: [";
      for (size_t i = 0; i < 6; ++i) {
        std::cout << finger_names[i] << ":" << get_status_name(status->finger_status[i]);
        if (i < 5) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
    }

    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
