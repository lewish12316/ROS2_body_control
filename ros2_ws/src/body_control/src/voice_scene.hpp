#pragma once

#include <cstdint>

struct VoiceSceneState
{
  bool auto_drive_enabled{false};//自动驾驶模式状态
  std::int32_t turn_state{0};//转弯状态（0-直行，1-左转，2-右转）
  std::int32_t veh_gear_st{0};//整车挡位信号：0-P，1-R，2-N，3-D。
  bool obs_detect_st{false};//障碍物检测状态：false-无，true-有。
  std::int32_t alarm_diag_st{0};// 故障告警信号：0 正常、正数故障，负数不确认故障。
  bool running_state_received{false};//是否接收到过车辆行驶状态
  bool vehicle_running{true};//车辆行驶状态（0-静止，1-行驶中）
  bool pedestrian_crossing{false};//行人横穿检测状态（0-未横穿，1-有）
};

inline std::int32_t select_voice_id(const VoiceSceneState & state)
{
  // 故障提示不受自动模式或启动按钮限制，必须已收到车辆行驶状态。
  // 以“故障且静止”近似判断靠边停车完成。
  // 无法确认实际靠边位置；启动时已静止且有故障也允许触发 ID 16。
  if (state.alarm_diag_st > 0 && state.running_state_received) {
    return state.vehicle_running ? 15 : 16;
  }
  if (!state.auto_drive_enabled) {
    return 0;
  }
  // 均为 P0，文档未规定同级顺序；约定故障 > 倒车 > 障碍物 > 礼让 > 转弯。
  if (state.veh_gear_st == 1) {
    return 12;
  }
  // 以检测到障碍物代替实际绕行状态。
  // true 触发、false 结束；该信号不能确认正在绕行或绕行已完成。
  if (state.obs_detect_st) {
    return 8;
  }
  // 现有信号只能确认有行人且车辆静止，无法单独确认斑马线位置。
  if (state.pedestrian_crossing &&
    state.running_state_received && !state.vehicle_running)
  {
    return 11;
  }
  if (state.turn_state == 1) {
    return 10;
  }
  if (state.turn_state == 2) {
    return 9;
  }
  return 0;  // 内部表示无播报任务，不发布此值。
}
