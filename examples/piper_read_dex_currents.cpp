// 注意demo需要实际连接灵巧手才能运行
// 读取灵巧手电流反馈 (-100% ~ +100%)
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

  // 循环读取并打印电流数据
  const char* finger_names[] = {"拇指尖", "拇指根", "食指", "中指", "无名指", "小指"};

  while (true) {
    auto currents = piper.get_dex_hand_currents();
    if (currents) {
      std::cout << "灵巧手电流 (-100%~+100%): [";
      for (size_t i = 0; i < 6; ++i) {
        std::cout << finger_names[i] << ":" << static_cast<int>(currents->finger_currents_pct[i]) << "%";
        if (i < 5) std::cout << ", ";
      }
      std::cout << "]" << std::endl;
    }

    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
