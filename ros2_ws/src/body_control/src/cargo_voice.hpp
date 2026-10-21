#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 每次到站按顺序各产生最多一个 ID 3、4 请求；成员均为内部调度状态。
class CargoVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    if (!state.veh_arrive_st) {
      used_this_arrival_ = false; // 仅明确收到未到站后，才允许下一次到站触发。
      clear_pending();
      return;
    }
    if (state.veh_gear_st != 2 || !state.veh_epb_st) {
      clear_pending(); // 已生成过请求则消费状态保留，条件恢复也不重试。
      return;
    }
    if (!used_this_arrival_) {
      used_this_arrival_ = true; // 建立请求即消费，不以最终能否播出为准。
      pending_since_ = now;
      pending_id3_ = true;
      pending_id4_ = true;
    }
    expire(now); // 高优先级阻塞期间仍按原时刻超时。
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_since_) {
      return std::nullopt;
    }
    if (pending_id3_) {
      pending_id3_ = false;
      return 3; // 保留原触发时间，不给 ID 4 重新计算 10 秒期限。
    }
    if (pending_id4_) {
      clear_pending();
      return 4;
    }
    return std::nullopt;
  }

private:
  void clear_pending()
  {
    pending_since_.reset();
    pending_id3_ = false;
    pending_id4_ = false;
  }

  void expire(TimePoint now)
  {
    if (pending_since_ && now - *pending_since_ >= std::chrono::seconds(10)) {
      clear_pending();
    }
  }

  bool used_this_arrival_{false};
  bool pending_id3_{false};
  bool pending_id4_{false};
  std::optional<TimePoint> pending_since_;
};
