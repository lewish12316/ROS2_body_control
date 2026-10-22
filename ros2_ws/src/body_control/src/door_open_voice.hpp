#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 仅保存内部周期及待播时间，不新增业务信号或 ROS 接口。
class DoorOpenVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    // 自动模式有效启动近似 APP/小程序发车，已确认未锁近似箱门未关闭。
    // 未锁不等于门未关闭，现有接口无法确认箱门实际开闭或 APP 点击来源。
    const bool enabled = automatic_voice_enabled(state) &&
      state.trunk_lock_state_received && !state.trunk_lock_st;
    if (!enabled) {
      next_due_.reset();
      pending_due_.reset();
      return;
    }
    if (!next_due_) {
      next_due_ = now + kPeriod; // 条件完整后先等 10 秒，同次授权再未锁也重新等待。
    }
    if (now >= *next_due_) {
      const auto skipped = (now - *next_due_) / kPeriod;
      pending_due_ = *next_due_ + kPeriod * skipped;
      next_due_ = *pending_due_ + kPeriod; // 固定时间轴，跳过历史轮次而不集中补发。
    }
    expire(now); // 即使被 P0 阻塞，也处理本轮超时。
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_due_) {
      return std::nullopt;
    }
    pending_due_.reset();
    return 1;
  }

private:
  void expire(TimePoint now)
  {
    if (pending_due_ && now - *pending_due_ >= kTimeout) {
      pending_due_.reset(); // 达到 3 秒丢弃，不重置下一轮。
    }
  }

  static constexpr std::chrono::seconds kPeriod{10};
  static constexpr std::chrono::seconds kTimeout{3};
  std::optional<TimePoint> next_due_;
  std::optional<TimePoint> pending_due_;
};
