#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

inline bool forward_voice_enabled(const VoiceSceneState & state)
{
  return state.auto_drive_enabled && state.running_state_received &&
         state.vehicle_running && state.veh_gear_st == 3;
}

// 全部成员均为内部调度状态，不新增业务信号或 ROS 接口。
// 由调用方传入单调时钟时间；一次检查最多产生一条播报请求。
class VoiceScheduler
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  std::optional<std::int32_t> update(const VoiceSceneState & state, TimePoint now)
  {
    const auto p0_id = select_voice_id(state);
    if (p0_id != active_p0_id_) {
      active_p0_id_ = p0_id;
      next_p0_due_ = p0_id != 0 ? std::optional<TimePoint>(now) : std::nullopt;
    }

    const bool forward_enabled = forward_voice_enabled(state);
    if (!forward_enabled) {
      next_forward_due_.reset();
      pending_forward_due_.reset();
    } else if (!forward_active_) {
      next_forward_due_ = now;  // 新进入前进场景，立即开始第一轮。
    }
    forward_active_ = forward_enabled;

    if (next_forward_due_ && now >= *next_forward_due_) {
      // 跳过历史轮次，仅保留最近一轮；周期以原到期时间为基准。
      const auto skipped = (now - *next_forward_due_) / kForwardPeriod;
      pending_forward_due_ = *next_forward_due_ + kForwardPeriod * skipped;
      next_forward_due_ = *pending_forward_due_ + kForwardPeriod;
    }
    if (pending_forward_due_ && now - *pending_forward_due_ >= kForwardTimeout) {
      pending_forward_due_.reset();  // 达到 10 秒即丢弃，不能在边界补播。
    }

    // 场景有效即视为 P0 占用，不在其 5 秒请求间隙穿插 P1。
    if (active_p0_id_ != 0) {
      if (now >= *next_p0_due_) {
        const auto skipped = (now - *next_p0_due_) / kP0Period;
        *next_p0_due_ += kP0Period * (skipped + 1);
        return active_p0_id_;
      }
      return std::nullopt;
    }
    if (pending_forward_due_) {
      pending_forward_due_.reset();
      return 7;
    }
    return std::nullopt;
  }

private:
  static constexpr std::chrono::seconds kP0Period{5};
  static constexpr std::chrono::seconds kForwardPeriod{120};
  static constexpr std::chrono::seconds kForwardTimeout{10};

  std::int32_t active_p0_id_{0};
  bool forward_active_{false};
  std::optional<TimePoint> next_p0_due_;
  std::optional<TimePoint> next_forward_due_;
  std::optional<TimePoint> pending_forward_due_;
};
