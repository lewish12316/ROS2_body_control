#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 每次到站最多产生一个 ID 3 请求；此处成员均为内部调度状态。
class CargoVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    if (!state.veh_arrive_st) {
      used_this_arrival_ = false; // 仅明确收到未到站后，才允许下一次到站触发。
      pending_since_.reset();
      return;
    }
    if (state.veh_gear_st != 2 || !state.veh_epb_st) {
      pending_since_.reset(); // 已生成过请求则消费状态保留，条件恢复也不重试。
      return;
    }
    if (!used_this_arrival_) {
      used_this_arrival_ = true; // 建立请求即消费，不以最终能否播出为准。
      pending_since_ = now;
    }
    expire(now); // 高优先级阻塞期间仍按原时刻超时。
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_since_) {
      return std::nullopt;
    }
    pending_since_.reset();
    return 3;
  }

private:
  void expire(TimePoint now)
  {
    if (pending_since_ && now - *pending_since_ >= std::chrono::seconds(10)) {
      pending_since_.reset();
    }
  }

  bool used_this_arrival_{false};
  std::optional<TimePoint> pending_since_;
};
