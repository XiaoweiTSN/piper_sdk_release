// 注意demo需要实际连接灵巧手才能运行
// 读取灵巧手完整反馈数据（位置、速度、电流、状态、频率）
#include <chrono>
#include <iomanip>
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

  // 循环读取并打印完整反馈数据
  const char* finger_names[] = {"拇指尖", "拇指根", "食指", "中指", "无名指", "小指"};

  while (true) {
    auto feedback = piper.get_dex_hand_feedback();
    if (feedback) {
      std::cout << "=== 灵巧手完整反馈 ===" << std::endl;

      // 左右手标识
      std::cout << "左右手: ";
      if (feedback->feedback.status.hand_side == piper_sdk::DexHandSide::kLeft) {
        std::cout << "左手";
      } else if (feedback->feedback.status.hand_side == piper_sdk::DexHandSide::kRight) {
        std::cout << "右手";
      } else {
        std::cout << "未知";
      }
      std::cout << std::endl;

      // 位置反馈 (0-100%)
      std::cout << "位置 (%): [";
      for (size_t i = 0; i < 6; ++i) {
        std::cout << static_cast<int>(feedback->feedback.positions.finger_positions_pct[i]);
        if (i < 5) std::cout << ", ";
      }
      std::cout << "]" << std::endl;

      // 速度反馈 (-100% ~ +100%)
      std::cout << "速度 (%): [";
      for (size_t i = 0; i < 6; ++i) {
        std::cout << static_cast<int>(feedback->feedback.velocities.finger_velocities_pct[i]);
        if (i < 5) std::cout << ", ";
      }
      std::cout << "]" << std::endl;

      // 电流反馈 (-100% ~ +100%)
      std::cout << "电流 (%): [";
      for (size_t i = 0; i < 6; ++i) {
        std::cout << static_cast<int>(feedback->feedback.currents.finger_currents_pct[i]);
        if (i < 5) std::cout << ", ";
      }
      std::cout << "]" << std::endl;

      // 电机状态
      std::cout << "状态: [";
      for (size_t i = 0; i < 6; ++i) {
        std::cout << get_status_name(feedback->feedback.status.finger_status[i]);
        if (i < 5) std::cout << ", ";
      }
      std::cout << "]" << std::endl;

      // 反馈频率
      std::cout << "频率: 位置=" << std::fixed << std::setprecision(1) << feedback->position_hz
                << "Hz (期望100Hz), 状态=" << feedback->status_hz << "Hz (期望20Hz)" << std::endl;
    }

    std::cout << "-------------------" << std::endl;
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}
