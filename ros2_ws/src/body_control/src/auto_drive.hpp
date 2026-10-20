#pragma once

#include <cstdint>

class AutoDriveGate
{
public:
  bool set_mode(std::int32_t mode)
  {
    if (mode == mode_) {
      return false;
    }
    mode_ = mode;
    started_ = false;  // 每次模式切换后都需要重新启动。
    return true;
  }

  void set_start_command(bool start)
  {
    // 自动模式之前收到的启动命令不缓存为之后的启动授权。
    started_ = (mode_ == 1) && start;
  }

  bool enabled() const { return mode_ == 1 && started_; }

private:
  std::int32_t mode_{0};
  bool started_{false};
};
