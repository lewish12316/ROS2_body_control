#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "auto_drive_test_support.hpp"
#include "../src/voice_scheduler.hpp"

using namespace std::chrono_literals;

VoiceScheduler::TimePoint at(std::int64_t milliseconds)
{
  return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

VoiceSceneState forward_state()
{
  VoiceSceneState state;
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  state.running_state_received = true;
  state.vehicle_running = true;
  state.veh_gear_st = 3;
  return state;
}

void test_forward_conditions_and_period()
{
  auto state = forward_state();
  assert(forward_voice_enabled(state));
  state.auto_drive_enabled = false;
  assert(!forward_voice_enabled(state));
  state = forward_state();
  state.running_state_received = false;
  assert(!forward_voice_enabled(state));
  state = forward_state();
  state.vehicle_running = false;
  assert(!forward_voice_enabled(state));
  for (const int gear : {0, 1, 2, -1, 99}) {
    state = forward_state();
    state.veh_gear_st = gear;
    assert(!forward_voice_enabled(state));
  }

  VoiceScheduler scheduler;
  state = forward_state();
  assert(scheduler.update(state, at(0)) == 7);
  for (const int t : {0, 50, 1000, 5000, 60000, 119999}) {
    assert(!scheduler.update(state, at(t)));
  }
  assert(scheduler.update(state, at(120000)) == 7);
  assert(!scheduler.update(state, at(120000)));
  assert(!scheduler.update(state, at(239999)));
  assert(scheduler.update(state, at(240000)) == 7);
}

void test_p0_periods_and_switching()
{
  for (const int id : {12, 8, 11, 10, 9}) {
    VoiceSceneState state;
    state.veh_mode_st = 1;
    state.auto_drive_enabled = true;
    if (id == 12) {state.veh_gear_st = 1;}
    if (id == 8) {state.obs_detect_st = true;}
    if (id == 11) {
      state.pedestrian_crossing = true;
      state.running_state_received = true;
      state.vehicle_running = false;
    }
    if (id == 10) {state.turn_state = 1;}
    if (id == 9) {state.turn_state = 2;}
    VoiceScheduler scheduler;
    assert(scheduler.update(state, at(0)) == id);
    assert(!scheduler.update(state, at(1000)));
    assert(!scheduler.update(state, at(4999)));
    assert(scheduler.update(state, at(5000)) == id);
    assert(scheduler.update(state, at(10000)) == id);
    assert(scheduler.update(state, at(27000)) == id); // 不补发遗漏的多轮。
    assert(!scheduler.update(state, at(27000)));
    state.auto_drive_enabled = false;
    assert(!scheduler.update(state, at(30000)));
  }

  VoiceScheduler scheduler;
  VoiceSceneState state;
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  state.turn_state = 1;
  assert(scheduler.update(state, at(0)) == 10);
  state.turn_state = 2;
  assert(scheduler.update(state, at(1000)) == 9);
  assert(!scheduler.update(state, at(5000)));
  assert(scheduler.update(state, at(6000)) == 9);
  state.obs_detect_st = true;
  assert(scheduler.update(state, at(7000)) == 8);
  state.veh_gear_st = 1;
  assert(scheduler.update(state, at(8000)) == 12);
  state.veh_gear_st = 3;
  assert(scheduler.update(state, at(9000)) == 8);
}

void test_pending_and_fixed_cadence()
{
  auto state = forward_state();
  state.obs_detect_st = true;
  VoiceScheduler scheduler;
  assert(scheduler.update(state, at(0)) == 8);
  assert(!scheduler.update(state, at(50))); // P0 间隔内也不能发送 P1。
  assert(scheduler.update(state, at(5000)) == 8);
  state.obs_detect_st = false;
  assert(scheduler.update(state, at(9999)) == 7);
  assert(!scheduler.update(state, at(119999)));
  assert(scheduler.update(state, at(120000)) == 7); // 非 129999。

  // 后续轮次同样允许等待，不改变下一轮原定时间。
  state.turn_state = 1;
  assert(scheduler.update(state, at(239000)) == 10);
  assert(!scheduler.update(state, at(240000)));
  state.turn_state = 0;
  assert(scheduler.update(state, at(249999)) == 7);
  assert(scheduler.update(state, at(360000)) == 7);
}

void test_timeout_boundaries_and_no_unscheduled_replay()
{
  for (const int clear_time : {10000, 10001, 20000}) {
    VoiceScheduler scheduler;
    auto state = forward_state();
    state.obs_detect_st = true;
    assert(scheduler.update(state, at(0)) == 8);
    state.obs_detect_st = false;
    assert(!scheduler.update(state, at(clear_time))); // >=10 秒丢弃。
    assert(!scheduler.update(state, at(119999)));
    assert(scheduler.update(state, at(120000)) == 7);
  }
  VoiceScheduler scheduler;
  auto state = forward_state();
  assert(scheduler.update(state, at(0)) == 7);
  state.obs_detect_st = true;
  assert(scheduler.update(state, at(1000)) == 8);
  state.obs_detect_st = false;
  assert(!scheduler.update(state, at(2000))); // 没有待播请求，不补播。
  assert(scheduler.update(state, at(120000)) == 7);

  // 长时间阻塞后不回放历史请求，仍保持 120 秒时间轴。
  assert(!scheduler.update(state, at(375000))); // 最近一轮 360s 已超时。
  assert(!scheduler.update(state, at(479999)));
  assert(scheduler.update(state, at(480000)) == 7);
  assert(scheduler.update(state, at(721000)) == 7); // 仅最近一轮仍有效。
  assert(!scheduler.update(state, at(721000)));
}

void test_cancel_and_reentry()
{
  // 所有退出条件均清除周期和被 P0 阻塞的待播请求。
  for (int condition = 0; condition != 4; ++condition) {
    VoiceScheduler scheduler;
    auto state = forward_state();
    state.turn_state = 1;
    assert(scheduler.update(state, at(0)) == 10);
    if (condition == 0) {state.vehicle_running = false;}
    if (condition == 1) {state.veh_gear_st = 2;}
    if (condition >= 2) {
      AutoDriveGate gate;
      confirm_safe_feedback(gate);
      gate.set_mode(1);
      state.veh_mode_st = 1;
      gate.set_start_command(true);
      if (condition == 2) {gate.set_start_command(false);}
      if (condition == 3) {gate.set_mode(2); state.veh_mode_st = 2;}
      state.auto_drive_enabled = gate.enabled();
    }
    scheduler.update(state, at(1000));
    state.turn_state = 0;
    assert(!scheduler.update(state, at(2000)));
    state = forward_state();
    assert(scheduler.update(state, at(3000)) == 7);
    assert(!scheduler.update(state, at(120000)));
    assert(scheduler.update(state, at(123000)) == 7);
  }
}

void test_fault_input_order_and_invalid_values()
{
  for (const bool fault_first : {false, true}) {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    if (fault_first) {
      state.alarm_diag_st = 1;
    } else {
      state.running_state_received = true;
      state.vehicle_running = true;
    }
    assert(!scheduler.update(state, at(0)));
    state.alarm_diag_st = 1;
    state.running_state_received = true;
    state.vehicle_running = true;
    assert(scheduler.update(state, at(1000)) == 15);
    assert(!scheduler.update(state, at(5999)));
    assert(scheduler.update(state, at(6000)) == 15);
  }
  for (const int alarm : {0, -1, -99}) {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    state.alarm_diag_st = alarm;
    state.running_state_received = true;
    state.vehicle_running = true;
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(5000)));
  }
}

void test_fault_period_independent_of_mode_and_start()
{
  VoiceScheduler scheduler;
  VoiceSceneState state;
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = true;
  AutoDriveGate gate; // 初始待机，未按启动。
  confirm_safe_feedback(gate);
  assert(scheduler.update(state, at(0)) == 15);
  assert(!scheduler.update(state, at(1000))); // 重复相同故障。
  state.alarm_diag_st = 3;
  assert(!scheduler.update(state, at(2000))); // 正数等级改变不重置周期。
  gate.set_mode(1);
  state.veh_mode_st = 1;
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(2500)));
  gate.set_start_command(true);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(3000)));
  gate.set_start_command(false);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(4000))); // 停止按钮不结束故障场景。
  assert(scheduler.update(state, at(5000)) == 15);
  gate.set_mode(2);
  state.veh_mode_st = 2;
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(6000)));
  assert(scheduler.update(state, at(10000)) == 15); // 遥控模式仍持续。
  state.vehicle_running = false;
  assert(scheduler.update(state, at(11000)) == 16);
  assert(!scheduler.update(state, at(15000))); // 从切换时刻重新计时。
  state.vehicle_running = true;
  assert(scheduler.update(state, at(16000)) == 15);
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(17000)) == 14); // 故障解除后恢复有效遥控行驶请求。
  assert(!scheduler.update(state, at(21000)));
  state.alarm_diag_st = 2;
  assert(scheduler.update(state, at(22000)) == 15); // 再次满足条件立即发出。
  state.alarm_diag_st = -1;
  assert(!scheduler.update(state, at(23000))); // 负数也结束已确认故障提示。
}

void test_fault_preemption_and_recovery()
{
  for (const int id : {12, 8, 10, 9}) {
    VoiceScheduler scheduler;
    auto state = forward_state();
    if (id == 12) {state.veh_gear_st = 1;}
    if (id == 8) {state.obs_detect_st = true;}
    if (id == 10) {state.turn_state = 1;}
    if (id == 9) {state.turn_state = 2;}
    assert(scheduler.update(state, at(0)) == id);
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(1000)) == 15);
    assert(!scheduler.update(state, at(5000))); // 原场景到期不能抢占故障。
    assert(scheduler.update(state, at(6000)) == 15);
    state.alarm_diag_st = 0;
    assert(scheduler.update(state, at(7000)) == id);
    assert(scheduler.update(state, at(12000)) == id);
  }
  // 停车后切换故障 ID 16；清除故障后才恢复礼让。
  VoiceScheduler scheduler;
  auto state = forward_state();
  state.alarm_diag_st = 1;
  state.pedestrian_crossing = true;
  assert(scheduler.update(state, at(0)) == 15);
  state.vehicle_running = false;
  assert(scheduler.update(state, at(1000)) == 16);
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(2000)) == 11);
}

void test_fault_blocks_p1_without_changing_deadlines()
{
  for (const int clear_time : {9999, 10000, 10001}) {
    VoiceScheduler scheduler;
    auto state = forward_state();
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(0)) == 15);
    assert(!scheduler.update(state, at(1000))); // 不能穿插 P1。
    assert(scheduler.update(state, at(5000)) == 15);
    state.alarm_diag_st = 0;
    const auto request = scheduler.update(state, at(clear_time));
    if (clear_time < 10000) {
      assert(request == 7);
    } else {
      assert(!request);
    }
    assert(scheduler.update(state, at(120000)) == 7);
  }
  VoiceScheduler scheduler;
  auto state = forward_state();
  assert(scheduler.update(state, at(0)) == 7);
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(119000)) == 15);
  assert(!scheduler.update(state, at(120000))); // 第二轮 P1 待播。
  assert(scheduler.update(state, at(124000)) == 15);
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(129999)) == 7);
  assert(scheduler.update(state, at(240000)) == 7);
}

void test_stopped_fault_input_order_and_invalid_values()
{
  for (const bool fault_first : {false, true}) {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    state.vehicle_running = false;
    if (fault_first) {
      state.alarm_diag_st = 1;
    } else {
      state.running_state_received = true;
    }
    assert(!scheduler.update(state, at(0))); // 必须收到完整条件。
    state.alarm_diag_st = 1;
    state.running_state_received = true;
    assert(scheduler.update(state, at(1000)) == 16); // 无需此前发生行驶故障。
    assert(!scheduler.update(state, at(5999)));
    assert(scheduler.update(state, at(6000)) == 16);
  }
  for (const int alarm : {0, -1, -99}) {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    state.alarm_diag_st = alarm;
    state.running_state_received = true;
    state.vehicle_running = false;
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(5000)));
  }
}

void test_stopped_fault_period_and_mode_independence()
{
  VoiceScheduler scheduler;
  VoiceSceneState state;
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  AutoDriveGate gate; // 待机且未启动。
  confirm_safe_feedback(gate);
  assert(scheduler.update(state, at(0)) == 16);
  assert(!scheduler.update(state, at(1000))); // 重复停车、故障状态不重置。
  state.alarm_diag_st = 3;
  assert(!scheduler.update(state, at(2000)));
  gate.set_mode(1);
  state.veh_mode_st = 1;
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(2500)));
  gate.set_start_command(true);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(3000)));
  gate.set_start_command(false);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(4000)));
  assert(scheduler.update(state, at(5000)) == 16);
  gate.set_mode(2);
  state.veh_mode_st = 2;
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(6000)));
  assert(scheduler.update(state, at(10000)) == 16); // 遥控模式不阻断。
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(11000)) == 14); // 遥控切入请求仍在原10秒期限内。
  assert(!scheduler.update(state, at(15000)));
  state.alarm_diag_st = 2;
  assert(scheduler.update(state, at(16000)) == 16);
  state.alarm_diag_st = -1;
  assert(!scheduler.update(state, at(17000)));
  assert(!scheduler.update(state, at(21000)));
}

void test_fault_motion_transitions_reset_period()
{
  VoiceScheduler scheduler;
  VoiceSceneState state;
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  assert(scheduler.update(state, at(0)) == 15);
  state.vehicle_running = false;
  assert(scheduler.update(state, at(1234)) == 16);
  assert(!scheduler.update(state, at(5000))); // 原 ID 15 的到期时间不再有效。
  assert(!scheduler.update(state, at(6233)));
  assert(scheduler.update(state, at(6234)) == 16);
  state.vehicle_running = true;
  assert(scheduler.update(state, at(7234)) == 15);
  assert(!scheduler.update(state, at(11234)));
  assert(!scheduler.update(state, at(12233)));
  assert(scheduler.update(state, at(12234)) == 15);
}

void test_stopped_fault_preemption_and_recovery()
{
  for (const int id : {12, 8, 11, 10, 9}) {
    VoiceScheduler scheduler;
    auto state = forward_state();
    state.vehicle_running = false;
    if (id == 12) {state.veh_gear_st = 1;}
    if (id == 8) {state.obs_detect_st = true;}
    if (id == 11) {state.pedestrian_crossing = true;}
    if (id == 10) {state.turn_state = 1;}
    if (id == 9) {state.turn_state = 2;}
    assert(scheduler.update(state, at(0)) == id);
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(1000)) == 16);
    assert(!scheduler.update(state, at(5000)));
    assert(scheduler.update(state, at(6000)) == 16);
    state.alarm_diag_st = 0;
    assert(scheduler.update(state, at(7000)) == id);
    assert(scheduler.update(state, at(12000)) == id);
  }
}

void test_fault_stop_cancels_forward_pending()
{
  VoiceScheduler scheduler;
  auto state = forward_state();
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(0)) == 15); // ID 7 的首轮被故障阻塞。
  state.vehicle_running = false;
  assert(scheduler.update(state, at(1000)) == 16); // 停车取消 ID 7 待播。
  state.alarm_diag_st = 0;
  assert(!scheduler.update(state, at(2000))); // 即使不足 10 秒也不能补播。
  state.vehicle_running = true;
  assert(scheduler.update(state, at(3000)) == 7); // 再次前进重新开始周期。
  assert(!scheduler.update(state, at(120000)));
  assert(scheduler.update(state, at(123000)) == 7);
}

int main()
{
  test_forward_conditions_and_period();
  test_p0_periods_and_switching();
  test_pending_and_fixed_cadence();
  test_timeout_boundaries_and_no_unscheduled_replay();
  test_cancel_and_reentry();
  test_fault_input_order_and_invalid_values();
  test_fault_period_independent_of_mode_and_start();
  test_fault_preemption_and_recovery();
  test_fault_blocks_p1_without_changing_deadlines();
  test_stopped_fault_input_order_and_invalid_values();
  test_stopped_fault_period_and_mode_independence();
  test_fault_motion_transitions_reset_period();
  test_stopped_fault_preemption_and_recovery();
  test_fault_stop_cancels_forward_pending();
  std::cout << "Voice scheduler checks passed\n";
}
