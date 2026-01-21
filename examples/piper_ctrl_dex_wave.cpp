// 注意demo需要实际连接灵巧手才能运行
// 灵巧手四指波浪动画演示（大拇指保持自然位置）
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

int main() {
  std::cout << "灵巧手四指波浪动画演示" << std::endl;

  // 创建SocketCAN传输对象，连接到can0接口
  auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");

  // 创建Piper接口对象
  piper_sdk::PiperConfig config;
  piper_sdk::PiperInterface piper(transport, config);

  // 连接端口
  piper.connect_port(10ms);
  std::this_thread::sleep_for(100ms);

  // 手指顺序: [拇指尖, 拇指根, 食指, 中指, 无名指, 小指]
  piper_sdk::DexHandPositionCommand cmd;

  // 初始化：回到自然位置
  piper.move_dex_hand_home();
  std::this_thread::sleep_for(1s);

  cmd.finger_positions_pct = {40, 40, 0, 0, 0, 0};

  // =====================================================
  // 波浪1：顺序波浪（手指逐个弯曲再逐个张开）
  // =====================================================
  std::cout << "波浪1：顺序波浪" << std::endl;

  for (int round = 0; round < 3; ++round) {
    // 四指顺序: 食指(2) -> 中指(3) -> 无名指(4) -> 小指(5)
    int indices[] = {2, 3, 4, 5};

    // 波浪进入：逐个弯曲
    for (int idx : indices) {
      cmd.finger_positions_pct[idx] = 100;
      piper.move_dex_hand_position(cmd);
      std::this_thread::sleep_for(150ms);
    }

    std::this_thread::sleep_for(200ms);

    // 波浪退出：逐个张开
    for (int idx : indices) {
      cmd.finger_positions_pct[idx] = 0;
      piper.move_dex_hand_position(cmd);
      std::this_thread::sleep_for(150ms);
    }

    std::this_thread::sleep_for(300ms);
  }

  // =====================================================
  // 波浪2：正弦波（平滑振荡）
  // =====================================================
  std::cout << "波浪2：正弦波" << std::endl;

  const double frequency = 1.0;          // Hz
  const double phase_offset = M_PI / 3;  // 手指间相位偏移
  auto start_time = std::chrono::steady_clock::now();

  for (int frame = 0; frame < 150; ++frame) {
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - start_time).count();

    // 大拇指保持自然位置
    cmd.finger_positions_pct[0] = 40;
    cmd.finger_positions_pct[1] = 40;

    // 四指按相位偏移的正弦波运动
    for (int i = 2; i < 6; ++i) {
      double phase = (i - 2) * phase_offset;
      double value = (std::sin(2 * M_PI * frequency * t - phase) + 1.0) / 2.0;
      cmd.finger_positions_pct[i] = static_cast<uint8_t>(value * 100);
    }

    piper.move_dex_hand_position(cmd);
    std::this_thread::sleep_for(20ms);
  }

  // =====================================================
  // 波浪3：弹钢琴模拟
  // =====================================================
  std::cout << "波浪3：弹钢琴" << std::endl;

  // 恢复张开
  cmd.finger_positions_pct = {0, 0, 0, 0, 0, 0};
  piper.move_dex_hand_position(cmd);
  std::this_thread::sleep_for(500ms);

  // 四指弹钢琴
  int piano_fingers[] = {2, 3, 4, 5};  // 食指, 中指, 无名指, 小指

  for (int tap = 0; tap < 16; ++tap) {
    int finger_idx = piano_fingers[tap % 4];

    // 按下
    cmd.finger_positions_pct = {0, 0, 0, 0, 0, 0};
    cmd.finger_positions_pct[finger_idx] = 80;
    piper.move_dex_hand_position(cmd);
    std::this_thread::sleep_for(150ms);

    // 释放
    cmd.finger_positions_pct[finger_idx] = 0;
    piper.move_dex_hand_position(cmd);
    std::this_thread::sleep_for(100ms);
  }

  // 恢复张开
  std::cout << "演示完成" << std::endl;
  piper.move_dex_hand_home();

  return 0;
}
