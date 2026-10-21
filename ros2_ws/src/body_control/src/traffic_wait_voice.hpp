#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

// ID 13 的等待任务；以下标记和时间均为内部调度状态。
class TrafficWaitVoiceTask
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  void observe(const VoiceSceneState & state, TimePoint now)
  {
    if (state.ambient_light_received && std::isfinite(state.ambient_light_level_st) &&
      state.ambient_light_level_st >= 0.0)
    {
      night_ = state.ambient_light_level_st <= kNightThreshold;
    } // 非法光照忽略，保留此前有效昼夜；无有效值则保持未知。
    const bool enabled = state.veh_mode_st == 1 && state.auto_drive_enabled &&
      state.traffic_light_wait_st && state.running_state_received && !state.vehicle_running;
    if (!enabled) {
      phase_.reset();
      next_day_due_.reset();
      pending_due_.reset();
      sent_this_wait_ = false;
      night_used_ = false;
      return; // 结束本次等待，但保留最后有效光照判断。
    }
    if (!night_) {
      return; // 光照未知时不产生首轮。
    }
    expire(now);
    if (phase_ != night_) {
      phase_ = night_;
      if (*night_) {
        next_day_due_.reset();
        if (sent_this_wait_ || night_used_) {
          pending_due_.reset();
        } else {
          night_used_ = true; // 建立请求即消耗夜间机会，超时/昼夜反复也不重开。
          if (!pending_due_) {
            pending_due_ = now;
          } // 未发的白天请求仍有效时沿用原到期时刻，不延长 3 秒期限。
        }
      } else {
        pending_due_ = now; // 恢复白天立即首轮，替换旧阶段尚未发送的请求。
        next_day_due_ = now + kDayPeriod;
      }
    } else if (!*night_ && next_day_due_ && now >= *next_day_due_) {
      const auto skipped = (now - *next_day_due_) / kDayPeriod;
      pending_due_ = *next_day_due_ + kDayPeriod * skipped;
      next_day_due_ = *pending_due_ + kDayPeriod;
    }
    expire(now); // 阻塞及迟到检查也处理固定期限，不补发历史轮次。
  }

  std::optional<std::int32_t> take_due(TimePoint now)
  {
    expire(now);
    if (!pending_due_) {
      return std::nullopt;
    }
    pending_due_.reset();
    sent_this_wait_ = true; // 仅表示节点已发出请求，不表示音频播放完成。
    return 13;
  }

private:
  void expire(TimePoint now)
  {
    if (pending_due_ && now - *pending_due_ >= std::chrono::seconds(3)) {
      pending_due_.reset();
    }
  }

  // 阈值由用户暂定，待后续文档同步；文档未定义光照单位。
  static constexpr double kNightThreshold = 5.0; // <=5 夜间，>5 白天，不使用 3/7 回差。
  static constexpr std::chrono::seconds kDayPeriod{10};
  std::optional<bool> night_; // 跨等待任务保留最后有效光照；未初始化表示未知。
  std::optional<bool> phase_; // 当前等待任务已处理的昼夜阶段。
  bool sent_this_wait_{false};
  bool night_used_{false};
  std::optional<TimePoint> pending_due_;
  std::optional<TimePoint> next_day_due_;
};
