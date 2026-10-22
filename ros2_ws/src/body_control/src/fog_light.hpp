#pragma once

#include <cmath>
#include <optional>

#include "voice_scene.hpp"

class FogLightControl
{
public:
  bool update(
    const VoiceSceneState & state, bool veh_power_st, bool low_visibility_st,
    std::optional<bool> fog_light_sw_cmd = std::nullopt)
  {
    if (state.ambient_light_received && std::isfinite(state.ambient_light_level_st) &&
      state.ambient_light_level_st >= 0.0)
    {
      dark_ = state.ambient_light_level_st <= kDarkThreshold;
    } // 无效值保留最后有效判断；首次未知不触发光照来源。

    if (!veh_power_st || state.veh_mode_st != 2) {
      manual_on_ = false; // 非有效时段的指令不缓存；下电或退出遥控即清除。
    } else if (fog_light_sw_cmd) {
      manual_on_ = *fog_light_sw_cmd;
    }

    if (!veh_power_st) {
      return false;
    }
    if (state.veh_mode_st == 1) {
      // 自动模式忽略手动指令，任一环境原因成立即开启，不依赖启动或行驶。
      return low_visibility_st || dark_.value_or(false);
    }
    // 遥控仅按本次有效手动请求；环境变化不覆盖。待机/未知模式关闭。
    return state.veh_mode_st == 2 && manual_on_;
  }

private:
  // 用户暂定阈值5；文档未定义光照单位，沿用现有光照判定。
  static constexpr double kDarkThreshold = 5.0;
  std::optional<bool> dark_; // 内部最后有效光照判断，跨模式/电源保留；空表示未知。
  bool manual_on_{false}; // 内部本次遥控请求，不是新增业务信号，初始无开启请求。
};
