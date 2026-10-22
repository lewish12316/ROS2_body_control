#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

class AuxLightControl
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  std::int32_t update(const VoiceSceneState & state, bool low_speed_parking_st, TimePoint now)
  {
    if (state.ambient_light_received && std::isfinite(state.ambient_light_level_st) &&
      state.ambient_light_level_st >= 0.0)
    {
      dark_ = state.ambient_light_level_st <= kDarkThreshold;
    } // 非法值保留最后有效判断；从未有效则未知，不触发环境补光。

    if (state.turn_state == 1 || state.turn_state == 2) {
      // 用户选择：直接切换方向，或延时期间重新转向，立即按新方向控制。
      turn_side_ = state.turn_state;
      turn_off_at_.reset();
    } else if (turn_side_ != 0) {
      if (!turn_off_at_) {
        turn_off_at_ = now + std::chrono::seconds(1);
      }
      if (now >= *turn_off_at_) {
        turn_side_ = 0;
        turn_off_at_.reset();
      }
    }

    const bool ambient = dark_.value_or(false) &&
      state.running_state_received && state.vehicle_running;
    // 先更新各来源再合并；撤销一个来源不覆盖其他来源的请求。
    return low_speed_parking_st || ambient ? 3 : turn_side_;
  }

private:
  // 阈值由用户暂定为5；文档未定义光照单位，沿用现有光照判定。
  static constexpr double kDarkThreshold = 5.0;
  // 以下均为内部实现状态，不作为新增业务接口发布。
  std::int32_t turn_side_{0}; // 当前转向或延时保持的侧，0-无，1-左，2-右。
  std::optional<TimePoint> turn_off_at_; // 首次退出转向后的固定期限。
  std::optional<bool> dark_; // 最后有效光照判断，未初始化表示未知。
};
