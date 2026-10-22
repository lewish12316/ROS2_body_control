#pragma once

#include <cstdint>

struct VoiceSceneState
{
  std::int32_t veh_mode_st{0};// 整车模式状态：0-待机，1-自动，2-远程遥控。
  bool auto_drive_enabled{false};// 内部有效启动授权，仅由节点从 AutoDriveGate::enabled() 更新。
  std::int32_t turn_state{0};//转弯状态（0-直行，1-左转，2-右转）
  std::int32_t veh_gear_st{0};//整车挡位信号：0-P，1-R，2-N，3-D。
  bool veh_arrive_st{false};// 到站停靠状态：false-未到站，true-停靠完成。
  // 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。
  // 临时 EPB 接口：std_msgs/msg/Bool，false-未拉起，true-已拉起。
  bool veh_epb_st{false};
  bool trunk_lock_st{false};// 货箱门锁状态：false-未锁，true-上锁。
  bool trunk_lock_state_received{false};// 内部接收标记，不是新增业务信号。
  bool obs_detect_st{false};//障碍物检测状态：false-无，true-有。
  bool battery_low_power_st{false};// 电池电量预警；由上游按 SOC<=20% 提供，节点不计算 SOC。
  std::int32_t alarm_diag_st{0};// 故障告警信号：0 正常、正数故障，负数不确认故障。
  bool running_state_received{false};//是否接收到过车辆行驶状态
  bool vehicle_running{true};//车辆行驶状态（0-静止，1-行驶中）
  // 文档未定义，经用户授权暂定，名称、类型及单位待后续确认。
  // 临时车速接口：std_msgs/msg/Float32，单位 km/h；初值不作为已接收车速。
  float veh_speed_st{0.0F};
  bool speed_state_received{false};// 内部接收标记，不是新增业务信号。
  std::uint64_t speed_feedback_seq{0};// 内部接收序号，每条车速消息递增，不是业务接口。
  bool pedestrian_crossing{false};//行人横穿检测状态（0-未横穿，1-有）
  // 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。
  bool traffic_light_wait_st{false};// 临时 Bool 接口：true-正在等红绿灯。
  double ambient_light_level_st{0.0};// 文档已有环境光照数值，单位未定义。
  bool ambient_light_received{false};// 内部接收标记，未知光照不按夜间处理。
};

// 自动驾驶相关语音的唯一公共准入条件；故障、遥控和装卸货等仍有独立规则。
inline bool automatic_voice_enabled(const VoiceSceneState & state)
{
  return state.veh_mode_st == 1 && state.auto_drive_enabled;
}

inline std::int32_t select_voice_id(const VoiceSceneState & state)
{
  // 故障提示不受自动模式或启动按钮限制，必须已收到车辆行驶状态。
  // 以“故障且静止”近似判断靠边停车完成。
  // 无法确认实际靠边位置；启动时已静止且有故障也允许触发 ID 16。
  if (state.alarm_diag_st > 0 && state.running_state_received) {
    return state.vehicle_running ? 15 : 16;
  }
  if (!automatic_voice_enabled(state)) {
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
