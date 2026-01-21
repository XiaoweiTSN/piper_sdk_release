// 注意demo需要实际连接灵巧手才能运行
// 控制灵巧手到特定手势
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
  // 位置范围: 0(张开) ~ 100(弯曲)
  piper_sdk::DexHandPositionCommand cmd;

  // 可以修改下面的数组来切换不同手势
  cmd.finger_positions_pct = {40, 40, 0, 0, 0, 0};

  // 其他可选手势（修改上面的数组即可）:
  // OK手势:    {60, 55, 40, 0, 0, 0}
  // 张开:       {0, 0, 0, 0, 0, 0}
  // 竖大拇指:   {0, 0, 100, 100, 100, 100}
  // 指向:       {100, 100, 0, 100, 100, 100}
  // 胜利/剪刀:  {100, 100, 0, 0, 100, 100}
  // 比三:       {100, 100, 0, 0, 0, 100}
  // 比四:       {100, 100, 0, 0, 0, 0}
  // 打电话:     {0, 100, 100, 100, 100, 0}
  // 自然状态:    {40, 40, 0, 0, 0, 0}

  std::cout << "执行手势完成" << std::endl;
  piper.move_dex_hand_position(cmd);

  return 0;
}
