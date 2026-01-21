// 注意demo需要实际连接灵巧手才能运行
// 对比灵巧手不同控制方式：位置控制 vs 位置+时间控制 vs 速度控制
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "piper_sdk/piper_interface.hpp"
#include "piper_sdk/socketcan_transport.hpp"

using namespace std::chrono_literals;

int main() {
  std::cout << "灵巧手控制方式对比演示" << std::endl;

  // 创建SocketCAN传输对象，连接到can0接口
  auto transport = std::make_shared<piper_sdk::SocketCanTransport>("can0");

  // 创建Piper接口对象
  piper_sdk::PiperConfig config;
  piper_sdk::PiperInterface piper(transport, config);

  // 连接端口
  piper.connect_port(10ms);
  std::this_thread::sleep_for(100ms);

  // 初始化：回到自然位置
  piper.move_dex_hand_home();
  std::this_thread::sleep_for(1s);

  piper_sdk::DexHandPositionCommand pos_cmd;

  // =====================================================
  // 模式1：位置控制（瞬时到位）
  // =====================================================
  std::cout << "\n模式1：位置控制 - 尽快到达目标位置" << std::endl;

  pos_cmd.finger_positions_pct = {40, 40, 100, 100, 100, 100};
  piper.move_dex_hand_position(pos_cmd);
  std::this_thread::sleep_for(2s);

  // =====================================================
  // 模式2：位置+时间控制（平滑过渡）
  // =====================================================
  std::cout << "模式2：位置+时间控制 - 2秒内平滑到达" << std::endl;

  piper_sdk::DexHandPositionTimedCommand timed_cmd;
  timed_cmd.finger_positions_pct = {40, 40, 0, 0, 0, 0};
  // 时间单位为10ms：200 = 2000ms = 2秒
  timed_cmd.finger_times_10ms = {200, 200, 200, 200, 200, 200};
  piper.move_dex_hand_position_timed(timed_cmd);
  std::this_thread::sleep_for(3s);

  // =====================================================
  // 模式3：速度控制
  // =====================================================
  std::cout << "模式3：速度控制 - 以30%速度闭合" << std::endl;

  piper_sdk::DexHandVelocityCommand vel_cmd;
  vel_cmd.finger_velocities_pct = {30, 30, 30, 30, 30, 30};
  piper.move_dex_hand_velocity(vel_cmd);
  std::this_thread::sleep_for(2s);

  // 停止速度控制
  vel_cmd.finger_velocities_pct = {0, 0, 0, 0, 0, 0};
  piper.move_dex_hand_velocity(vel_cmd);

  // 恢复张开
  std::cout << "\n演示完成" << std::endl;
  piper.move_dex_hand_home();

  return 0;
}
