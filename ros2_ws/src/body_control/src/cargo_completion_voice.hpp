#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 内部三轮计时任务，复用 ID 5 的开锁事件，不新增业务信号或 ROS 接口。
class CargoCompletionVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now, bool unlocked)
  {
    if (state.trunk_lock_state_received && state.trunk_lock_st) {
      unlock_time_.reset();
      pending_since_.reset();
      next_round_ = 1;
      return; // 用户确认：重新上锁取消本组全部剩余提醒。
    }
    if (unlocked) {
      // 以锁状态变化近似 APP 开锁，不能识别点击来源；与 ID 5 共用同一事件时刻。
      unlock_time_ = now;
      pending_since_.reset();
      next_round_ = 1;
    }
    if (unlock_time_) {
      // 最多循环三轮；迟到检查仅保留最近已到期轮次，不补发历史轮次。
      while (next_round_ <= 3) {
        const auto due = *unlock_time_ + std::chrono::seconds(60 * next_round_);
        if (now < due) {
          break;
        }
        pending_since_ = due; // 必须使用原定到期时间，而非检查或实际发送时刻。
        ++next_round_; // 超时轮次也计入三轮，不增加补偿轮次。
      }
    }
    expire(now);
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_since_) {
      return std::nullopt;
    }
    pending_since_.reset();
    return 2;
  }

private:
  void expire(TimePoint now)
  {
    if (pending_since_ && now - *pending_since_ >= std::chrono::seconds(10)) {
      pending_since_.reset();
    }
    if (next_round_ > 3 && !pending_since_) {
      unlock_time_.reset(); // 三个触发轮次已结束，等待下一次真正开锁。
    }
  }

  std::optional<TimePoint> unlock_time_;
  std::optional<TimePoint> pending_since_;
  unsigned int next_round_{1};
};
