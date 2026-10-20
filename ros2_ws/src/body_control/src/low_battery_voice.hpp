#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 仅保存内部任务状态，没有新增文档外业务信号或 ROS 接口。
class LowBatteryVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    // 经用户确认：将自动模式下的有效启动近似映射为远程端点击发车。
    const bool started = state.veh_mode_st == 1 && state.auto_drive_enabled;
    if (!started) {
      close_window();
    } else if (!previous_started_) {
      window_open_ = true;
      triggered_ = false;
      remaining_ = 0;
      next_due_.reset();
    }
    previous_started_ = started;
    if (!window_open_) {
      return;
    }
    if ((state.running_state_received && state.vehicle_running) ||
      (triggered_ && !state.battery_low_power_st))
    {
      close_window(); // 开始行驶或任务开始后低电量解除，本次发车不再重试。
      return;
    }
    if (triggered_) {
      return; // 已建立任务，不因重复反馈重新计数或计时。
    }
    if (!state.running_state_received || !state.battery_low_power_st) {
      // 启动时尚无静止或低电量反馈，只等待，不关闭发车窗口。
      // 后续回调补齐条件后可触发；已收到行驶状态 true 的情况在上方关闭。
      return;
    }
    triggered_ = true; // 此处已确认静止且低电量，开始本次两条播报请求。
    remaining_ = 2;
    next_due_ = now;
  }

  bool active() const { return window_open_ && triggered_ && remaining_ > 0; }

  // 只有现有 P0 无效时由调度器调用；返回值表示一次发送请求，不是播放完成。
  std::optional<std::int32_t> take_due(TimePoint now)
  {
    if (!active() || !next_due_ || now < *next_due_) {
      return std::nullopt;
    }
    --remaining_;
    if (remaining_ == 0) {
      close_window();
    } else {
      // 以第一次真正产生发送请求的时刻计时，避免阻塞后集中补发。
      next_due_ = now + std::chrono::seconds(10);
    }
    return 17;
  }

private:
  void close_window()
  {
    window_open_ = false;
    remaining_ = 0;
    next_due_.reset();
  }

  bool previous_started_{false};
  bool window_open_{false};
  bool triggered_{false};
  unsigned int remaining_{0};
  std::optional<TimePoint> next_due_;
};
