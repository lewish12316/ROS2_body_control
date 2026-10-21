#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 仅保存内部状态，不增加 APP 指令或其他文档外业务信号。
class UnlockVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    if (state.trunk_lock_state_received) {
      if (state.trunk_lock_st) {
        pending_since_.reset(); // 重新上锁，不能继续请求过时的“开锁成功”。
      } else if (previous_lock_ && *previous_lock_) {
        // 以已知锁状态 true->false 近似表示开锁成功。
        // 不能识别 APP 点击来源；首次收到 false 仅初始化，不产生事件。
        pending_since_ = now;
      }
      previous_lock_ = state.trunk_lock_st;
    }
    expire(now); // 阻塞期间也更新前值、取消事件和原来的 3 秒期限。
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_since_) {
      return std::nullopt;
    }
    pending_since_.reset();
    return 5;
  }

private:
  void expire(TimePoint now)
  {
    if (pending_since_ && now - *pending_since_ >= std::chrono::seconds(3)) {
      pending_since_.reset();
    }
  }

  std::optional<bool> previous_lock_;
  std::optional<TimePoint> pending_since_;
};
