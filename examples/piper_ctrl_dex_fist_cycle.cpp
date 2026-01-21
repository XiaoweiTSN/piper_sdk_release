// 注意demo需要实际连接灵巧手才能运行
// 控制灵巧手四指握拳循环（拇指保持张开）
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

  // 手指顺序: [拇指尖, 拇指根, 食指, 中指, 无名指, 小指]
  piper_sdk::DexHandPositionCommand cmd;

  bool fist_closed = false;
  int count = 0;

  // 循环控制四指握拳/张开
  while (true) {
    count++;

    // 每3秒切换一次（300 * 10ms = 3s）
    if (count == 300) {
      std::cout << "四指握拳" << std::endl;
      // 拇指保持自然(40)，四指握拳(100)
      cmd.finger_positions_pct = {40, 40, 100, 100, 100, 100};
      piper.move_dex_hand_position(cmd);
    } else if (count == 600) {
      std::cout << "四指张开" << std::endl;
      // 全部张开
      cmd.finger_positions_pct = {40, 40, 0, 0, 0, 0};
      piper.move_dex_hand_position(cmd);
      count = 0;
    }

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}
