// 注意demo需要实际连接灵巧手才能运行
// 读取灵巧手左右手标识
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

  // 等待状态反馈
  std::cout << "等待识别灵巧手..." << std::endl;

  // 带超时尝试获取左右手标识（5秒超时）
  int attempts = 0;
  const int max_attempts = 50;

  while (attempts < max_attempts) {
    auto status = piper.get_dex_hand_status();

    if (status && status->hand_side != piper_sdk::DexHandSide::kUnknown) {
      if (status->hand_side == piper_sdk::DexHandSide::kLeft) {
        std::cout << R"(
   _      ______ ______ _______
  | |    |  ____|  ____|__   __|
  | |    | |__  | |__     | |
  | |    |  __| |  __|    | |
  | |____| |____| |       | |
  |______|______|_|       |_|

  检测到左手
)" << std::endl;
      } else {
        std::cout << R"(
   _____  _____ _____ _    _ _______
  |  __ \|_   _/ ____| |  | |__   __|
  | |__) | | || |  __| |__| |  | |
  |  _  /  | || | |_ |  __  |  | |
  | | \ \ _| || |__| | |  | |  | |
  |_|  \_\_____\_____|_|  |_|  |_|

  检测到右手
)" << std::endl;
      }
      return 0;
    }

    std::cout << "." << std::flush;
    std::this_thread::sleep_for(100ms);
    attempts++;
  }

  std::cout << "\n未能识别左右手（超时）" << std::endl;
  return 1;
}
