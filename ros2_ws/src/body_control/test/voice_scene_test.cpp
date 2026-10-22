#include <cassert>
#include <iostream>

#include "../src/voice_scene.hpp"
#include "auto_drive_test_support.hpp"

int main()
{
  VoiceSceneState state;
  AutoDriveGate drive;
  confirm_safe_feedback(drive);
  state.turn_state = 1;
  assert(select_voice_id(state) == 0);  // 启动前禁用语音。
  state.veh_gear_st = 1;
  assert(select_voice_id(state) == 0);  // 未启动，即使挂 R 挡也不播报。
  state.veh_gear_st = 0;
  drive.set_start_command(true);
  assert(!drive.enabled());            // 非自动模式下启动无效。
  assert(drive.set_mode(1));
  state.veh_mode_st = 1;
  assert(!drive.enabled());            // 先前启动命令不生效。
  drive.set_start_command(true);
  assert(drive.enabled());
  assert(!drive.set_mode(1));
  assert(drive.enabled());             // 重复自动模式反馈不撤销启动。
  state.auto_drive_enabled = drive.enabled();
  state.turn_state = 0;
  assert(select_voice_id(state) == 0);
  // R 挡即触发，不附加行驶状态或行人条件。
  state.veh_gear_st = 1;
  assert(select_voice_id(state) == 12);
  state.turn_state = 2;
  assert(select_voice_id(state) == 12);  // 倒车覆盖转弯提示。
  state.pedestrian_crossing = true;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(select_voice_id(state) == 12);  // 倒车覆盖礼让提示。
  state.veh_gear_st = 3;
  assert(select_voice_id(state) == 11);  // 退出 R 挡后恢复其他场景。
  state.pedestrian_crossing = false;
  assert(select_voice_id(state) == 9);
  state.turn_state = 0;
  for (const auto gear : {0, 2, 3, -1, 99}) {
    state.veh_gear_st = gear;
    assert(select_voice_id(state) == 0);  // 非 R 挡和无效挡位不触发倒车。
  }
  state.veh_gear_st = 0;
  state.running_state_received = false;
  state.vehicle_running = true;
  state.turn_state = 1;
  assert(select_voice_id(state) == 10);
  state.turn_state = 2;
  assert(select_voice_id(state) == 9);
  state.turn_state = 0;
  state.pedestrian_crossing = true;
  state.vehicle_running = false;
  assert(select_voice_id(state) == 0);  // 没收到行驶状态，不能确认停车。
  state.running_state_received = true;
  assert(select_voice_id(state) == 11);
  state.turn_state = 2;
  assert(select_voice_id(state) == 11);  // 礼让覆盖转弯。
  state.pedestrian_crossing = false;
  assert(select_voice_id(state) == 9);   // 行人离开，恢复转弯。
  state.turn_state = 0;
  assert(select_voice_id(state) == 0);
  state.pedestrian_crossing = true;
  state.vehicle_running = true;
  assert(select_voice_id(state) == 0);   // 尚未停车。
  state.vehicle_running = false;
  assert(select_voice_id(state) == 11);
  drive.set_start_command(false);
  state.auto_drive_enabled = drive.enabled();
  assert(select_voice_id(state) == 0);   // 停止命令关闭语音。
  drive.set_start_command(true);
  state.auto_drive_enabled = drive.enabled();
  assert(select_voice_id(state) == 11);  // 再启动，恢复当前场景。
  drive.set_mode(2);
  state.veh_mode_st = 2;
  state.auto_drive_enabled = drive.enabled();
  assert(select_voice_id(state) == 0);   // 遥控接管关闭语音。
  drive.set_start_command(true);
  assert(!drive.enabled());
  drive.set_mode(0); // 遥控接管后必须先返回待机。
  drive.set_mode(1);
  state.veh_mode_st = 1;
  assert(!drive.enabled());            // 再次进入自动模式仍需新启动命令。
  drive.set_start_command(true);
  assert(drive.enabled());
  drive.set_mode(99);
  state.veh_mode_st = 99;
  assert(!drive.enabled());            // 未知模式也不允许启动。
  state.veh_gear_st = 1;
  state.auto_drive_enabled = drive.enabled();
  assert(select_voice_id(state) == 0);  // 退出自动模式禁止倒车提示。
  drive.set_mode(1);
  state.veh_mode_st = 1;
  drive.set_start_command(true);
  state.auto_drive_enabled = drive.enabled();
  assert(select_voice_id(state) == 12);
  drive.set_start_command(false);
  state.auto_drive_enabled = drive.enabled();
  assert(select_voice_id(state) == 0);  // R 挡期间按停止键也会关闭提示。

  VoiceSceneState obstacle_state;
  obstacle_state.obs_detect_st = true;
  assert(select_voice_id(obstacle_state) == 0);  // 未启动不提示障碍物。
  obstacle_state.veh_mode_st = 1;
  obstacle_state.auto_drive_enabled = true;
  assert(select_voice_id(obstacle_state) == 8);
  obstacle_state.obs_detect_st = false;
  assert(select_voice_id(obstacle_state) == 0);  // 检测清除后结束。
  obstacle_state.turn_state = 1;
  obstacle_state.obs_detect_st = true;
  assert(select_voice_id(obstacle_state) == 8);  // 障碍物覆盖转弯。
  obstacle_state.pedestrian_crossing = true;
  obstacle_state.running_state_received = true;
  obstacle_state.vehicle_running = false;
  assert(select_voice_id(obstacle_state) == 8);  // 不要求车辆正在运动。
  obstacle_state.veh_gear_st = 1;
  assert(select_voice_id(obstacle_state) == 12); // 保留倒车优先规则。
  obstacle_state.veh_gear_st = 3;
  assert(select_voice_id(obstacle_state) == 8);  // 退出 R 挡后恢复障碍物提示。
  obstacle_state.obs_detect_st = false;
  assert(select_voice_id(obstacle_state) == 11); // 清除后恢复礼让。
  obstacle_state.pedestrian_crossing = false;
  assert(select_voice_id(obstacle_state) == 10); // 再恢复左转。
  obstacle_state.obs_detect_st = true;
  obstacle_state.auto_drive_enabled = false;
  assert(select_voice_id(obstacle_state) == 0);  // 停止或退出模式后不播报。
  obstacle_state.veh_mode_st = 1;
  obstacle_state.auto_drive_enabled = true;
  assert(select_voice_id(obstacle_state) == 8);  // 再启动按当前检测状态恢复。

  VoiceSceneState fault_state;
  fault_state.alarm_diag_st = 1;
  assert(select_voice_id(fault_state) == 0); // 默认行驶值不能代替已收到的状态。
  fault_state.running_state_received = true;
  fault_state.vehicle_running = false;
  assert(select_voice_id(fault_state) == 16); // 启动即静止故障也触发。
  fault_state.vehicle_running = true;
  assert(select_voice_id(fault_state) == 15); // 无自动启动授权也有效。
  fault_state.veh_mode_st = 1;
  fault_state.auto_drive_enabled = true;
  fault_state.veh_gear_st = 1;
  fault_state.obs_detect_st = true;
  fault_state.turn_state = 1;
  assert(select_voice_id(fault_state) == 15);
  for (const int alarm : {0, -1, -99}) {
    fault_state.alarm_diag_st = alarm;
    assert(select_voice_id(fault_state) == 12); // 正常或未确认故障恢复原场景。
  }
  fault_state.alarm_diag_st = 2;
  fault_state.veh_gear_st = 3;
  fault_state.obs_detect_st = false;
  fault_state.pedestrian_crossing = true;
  fault_state.vehicle_running = false;
  assert(select_voice_id(fault_state) == 16); // 停车后故障提示仍优先于礼让。
  fault_state.alarm_diag_st = 0;
  assert(select_voice_id(fault_state) == 11); // 清除故障后才恢复礼让。
  std::cout << "Voice scene checks passed\n";
}
