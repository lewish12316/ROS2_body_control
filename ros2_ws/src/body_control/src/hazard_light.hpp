#pragma once

#include <chrono>
#include <optional>

#include "voice_scene.hpp"

class HazardLightControl
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  bool update(
    const VoiceSceneState & state, bool emergency_stop_st,
    bool mode_transition_abnormal_st, TimePoint now)
  {
    const bool stopped = state.running_state_received && !state.vehicle_running;
    // 用户确认的近似：无法证明停车原因、靠边位置或实际驻车动作。
    // 即使其他双闪条件成立，也必须更新遇障计时，避免短路造成漏计或未清零。
    if (state.obs_detect_st && stopped) {
      if (!obstacle_wait_since_) {
        obstacle_wait_since_ = now;
      }
    } else {
      obstacle_wait_since_.reset();
    }
    const bool obstacle_wait = obstacle_wait_since_ &&
      now - *obstacle_wait_since_ >= kObstacleWait;
    return emergency_stop_st || mode_transition_abnormal_st ||
           (state.alarm_diag_st > 0 && stopped) || obstacle_wait ||
           state.veh_mode_st == 2 || (state.battery_low_power_st && stopped);
  }

private:
  // 文档未规定，经用户确认暂定30秒；不是新增业务信号或配置参数。
  static constexpr auto kObstacleWait = std::chrono::seconds(30);
  // 内部计时起点；optional是否有值同时表示计时有效，不作为业务接口发布。
  std::optional<TimePoint> obstacle_wait_since_;
};
