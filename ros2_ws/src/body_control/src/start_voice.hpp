#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// 每个已确认的静止->行驶边沿最多一次；成员仅为内部事件和调度状态。
class StartVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    // 沿用已确认的近似映射：自动模式有效启动表示远程发车，无法识别 APP 点击来源。
    const bool enabled = automatic_voice_enabled(state);
    if (!enabled || !state.running_state_received) {
      was_stationary_ = false;
      low_speed_ = false;
      pending_since_.reset(); // 授权外事件不缓存，首次 true 不构造起步。
      return;
    }
    if (!state.vehicle_running) {
      was_stationary_ = true;
      low_speed_ = false;
      pending_since_.reset(); // 停车取消旧请求，并允许下一次真正起步。
      return;
    }
    if (was_stationary_) {
      pending_since_ = now; // 期限从起步边沿开始，不能从车速补齐时开始。
      speed_seq_at_start_ = state.speed_feedback_seq;
      low_speed_ = false;
    }
    was_stationary_ = false;
    // 仅起步边沿之后新接收的车速可用于本次判断；相同数值重发也是新反馈。
    // 接收序号无法识别上游真实采集时间，不新增话题、消息字段或全局有效期。
    const bool valid_speed = state.speed_state_received &&
      state.speed_feedback_seq != speed_seq_at_start_ &&
      std::isfinite(state.veh_speed_st) && state.veh_speed_st >= 0.0F;
    low_speed_ = valid_speed && state.veh_speed_st < 5.0F;
    if (valid_speed && state.veh_speed_st >= 5.0F) {
      pending_since_.reset(); // 同次行驶中再次减速也不重试。
    }
    expire(now); // 即使 P0 持续有效，也观察车速、取消和超时。
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_since_ || !low_speed_) {
      return std::nullopt; // 等待车速不阻塞其他已就绪的请求。
    }
    pending_since_.reset();
    return 6;
  }

private:
  void expire(TimePoint now)
  {
    if (pending_since_ && now - *pending_since_ >= std::chrono::seconds(3)) {
      pending_since_.reset();
    }
  }

  bool was_stationary_{false};
  bool low_speed_{false};
  std::uint64_t speed_seq_at_start_{0};
  std::optional<TimePoint> pending_since_;
};
