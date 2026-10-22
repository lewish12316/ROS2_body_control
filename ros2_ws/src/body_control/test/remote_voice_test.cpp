#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "auto_drive_test_support.hpp"
#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t milliseconds)
{
  return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

VoiceSceneState remote_state(bool moving)
{
  VoiceSceneState state;
  state.veh_mode_st = 2;
  state.running_state_received = true;
  state.vehicle_running = moving;
  return state;
}

void test_entry_without_movement_and_unknown_running_state()
{
  for (const bool received : {false, true}) {
    VoiceScheduler scheduler;
    auto state = remote_state(false);
    state.running_state_received = received;
    assert(scheduler.update(state, at(0)) == 14);
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(1000)));
    assert(!scheduler.update(state, at(60000))); // 静止或未知状态不启动循环。
    state.running_state_received = true;
    state.vehicle_running = true;
    assert(scheduler.update(state, at(61000)) == 14);
    assert(!scheduler.update(state, at(120999)));
    assert(scheduler.update(state, at(121000)) == 14);
  }
  VoiceScheduler scheduler;
  VoiceSceneState state;
  state.veh_mode_st = 2; // 默认 vehicle_running=true 也不能代替收到行驶状态。
  assert(scheduler.update(state, at(0)) == 14);
  assert(!scheduler.update(state, at(60000)));
}

void test_moving_entry_merge_and_period()
{
  for (const int gear : {0, 1, 2, 3, -1, 99}) {
    VoiceScheduler scheduler;
    auto state = remote_state(true);
    state.veh_gear_st = gear;
    assert(scheduler.update(state, at(0)) == 14); // 不要求 D 挡。
    for (const int t : {0, 50, 1000, 5000, 59999}) {
      assert(!scheduler.update(state, at(t))); // 合并后不重复，重复状态不重置。
    }
    assert(scheduler.update(state, at(60000)) == 14);
    assert(!scheduler.update(state, at(60000)));
    assert(!scheduler.update(state, at(119999)));
    assert(scheduler.update(state, at(120000)) == 14);
  }
}

void test_stopping_restarting_and_mode_exit()
{
  VoiceScheduler scheduler;
  auto state = remote_state(false);
  assert(scheduler.update(state, at(0)) == 14);
  state.vehicle_running = true;
  assert(scheduler.update(state, at(1000)) == 14); // 开始行驶是新的触发。
  state.vehicle_running = false;
  assert(!scheduler.update(state, at(10000)));
  state.vehicle_running = true;
  assert(scheduler.update(state, at(11000)) == 14);
  state.veh_mode_st = 0;
  assert(!scheduler.update(state, at(12000)));
  state.veh_mode_st = 2;
  assert(scheduler.update(state, at(13000)) == 14);
  assert(!scheduler.update(state, at(61000))); // 已取消旧周期。
  assert(!scheduler.update(state, at(71000)));
  assert(scheduler.update(state, at(73000)) == 14);
  state.veh_mode_st = 99;
  assert(!scheduler.update(state, at(74000)));
  assert(!scheduler.update(state, at(133000)));
}

void test_button_independence_and_existing_p0_scope()
{
  AutoDriveGate gate;
  confirm_safe_feedback(gate);
  gate.set_mode(2);
  auto state = remote_state(true);
  // 已有自动驾驶专用 P0 不因新增远程提示而启用。
  state.veh_gear_st = 1;
  state.obs_detect_st = true;
  state.turn_state = 1;
  VoiceScheduler scheduler;
  assert(scheduler.update(state, at(0)) == 14);
  for (const bool button : {true, false, true}) {
    gate.set_start_command(button);
    assert(!gate.enabled()); // 远程模式下按钮不能生成自动驾驶 true 请求。
    state.auto_drive_enabled = gate.enabled();
    assert(!scheduler.update(state, at(1000)));
  }
  assert(scheduler.update(state, at(60000)) == 14);
}

void test_p0_timeout_and_fixed_period()
{
  for (const int clear_time : {9999, 10000, 10001}) {
    auto state = remote_state(true);
    state.alarm_diag_st = 1;
    VoiceScheduler scheduler;
    assert(scheduler.update(state, at(0)) == 15);
    assert(!scheduler.update(state, at(50)));
    assert(scheduler.update(state, at(5000)) == 15);
    state.alarm_diag_st = 0;
    const auto request = scheduler.update(state, at(clear_time));
    if (clear_time < 10000) {assert(request == 14);} else {assert(!request);}
    assert(!scheduler.update(state, at(clear_time))); // 切入与首轮都已清除。
    assert(!scheduler.update(state, at(59999)));
    assert(scheduler.update(state, at(60000)) == 14);
  }
  VoiceScheduler scheduler;
  auto state = remote_state(true);
  assert(scheduler.update(state, at(0)) == 14);
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(59000)) == 15);
  assert(!scheduler.update(state, at(60000)));
  assert(scheduler.update(state, at(64000)) == 15);
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(69999)) == 14);
  assert(scheduler.update(state, at(120000)) == 14); // 不因延迟改为 129999。
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(121000)) == 15);
  state.alarm_diag_st = 0;
  assert(!scheduler.update(state, at(122000))); // 无待播请求，不补播。
}

void test_independent_entry_and_cycle_deadlines()
{
  // 切入请求已过期，但较晚产生的行驶首轮仍有效。
  VoiceScheduler scheduler;
  auto state = remote_state(false);
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(0)) == 16);
  state.vehicle_running = true;
  assert(scheduler.update(state, at(5000)) == 15);
  assert(scheduler.update(state, at(10000)) == 15); // 切入超时不影响行驶首轮。
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(14000)) == 14);
  assert(!scheduler.update(state, at(14000)));
  assert(scheduler.update(state, at(65000)) == 14);

  // 两种期限均未超时，只发一次并清除两种请求。
  VoiceScheduler merged;
  state = remote_state(false);
  state.alarm_diag_st = 1;
  assert(merged.update(state, at(0)) == 16);
  state.vehicle_running = true;
  assert(merged.update(state, at(3000)) == 15);
  state.alarm_diag_st = 0;
  assert(merged.update(state, at(9000)) == 14);
  assert(!merged.update(state, at(11000)));
  assert(merged.update(state, at(63000)) == 14);

  // 不允许把切入的期限延长到行驶首轮的期限。
  VoiceScheduler expired;
  state = remote_state(false);
  state.alarm_diag_st = 1;
  assert(expired.update(state, at(0)) == 16);
  state.vehicle_running = true;
  assert(expired.update(state, at(3000)) == 15);
  state.alarm_diag_st = 0;
  assert(!expired.update(state, at(13000))); // 两个请求均达到各自 10 秒期限。
  assert(expired.update(state, at(63000)) == 14);
}

void test_stop_and_exit_cancel_only_relevant_requests()
{
  // 停车取消循环，但保留未超时切入提示。
  VoiceScheduler scheduler;
  auto state = remote_state(true);
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(0)) == 15);
  state.vehicle_running = false;
  assert(scheduler.update(state, at(1000)) == 16);
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(2000)) == 14);
  assert(!scheduler.update(state, at(60000)));

  // 后续循环被阻塞时停车，没有切入待播请求可保留。
  VoiceScheduler cycle;
  state = remote_state(true);
  assert(cycle.update(state, at(0)) == 14);
  state.alarm_diag_st = 1;
  assert(cycle.update(state, at(59000)) == 15);
  assert(!cycle.update(state, at(60000)));
  state.vehicle_running = false;
  assert(cycle.update(state, at(61000)) == 16);
  state.alarm_diag_st = 0;
  assert(!cycle.update(state, at(62000)));

  // 退出模式取消所有待播，故障依然有效，重进模式产生全新请求。
  VoiceScheduler exit;
  state = remote_state(true);
  state.alarm_diag_st = 1;
  assert(exit.update(state, at(0)) == 15);
  state.veh_mode_st = 0;
  assert(!exit.update(state, at(1000)));
  state.alarm_diag_st = 0;
  assert(!exit.update(state, at(2000)));
  state.veh_mode_st = 2;
  assert(exit.update(state, at(3000)) == 14);
  assert(!exit.update(state, at(60000)));
  assert(exit.update(state, at(63000)) == 14);
}

void test_delayed_checks_and_auto_remote_transition()
{
  VoiceScheduler scheduler;
  auto state = remote_state(true);
  assert(scheduler.update(state, at(0)) == 14);
  assert(!scheduler.update(state, at(195000))); // 最近一轮 180s 已超时。
  assert(scheduler.update(state, at(240000)) == 14);
  assert(scheduler.update(state, at(361000)) == 14); // 仅最近一轮仍有效。
  assert(!scheduler.update(state, at(361000)));

  // 自动切远程取消 ID 7，切回自动必须重新启动。
  VoiceScheduler transition;
  AutoDriveGate gate;
  confirm_safe_feedback(gate);
  gate.set_mode(1);
  gate.set_start_command(true);
  state = remote_state(true);
  state.veh_mode_st = 1;
  state.veh_gear_st = 3;
  state.auto_drive_enabled = gate.enabled();
  assert(transition.update(state, at(0)) == 7);
  gate.set_mode(2);
  state.veh_mode_st = 2;
  state.auto_drive_enabled = gate.enabled();
  assert(transition.update(state, at(1000)) == 14);
  assert(transition.update(state, at(61000)) == 14);
  assert(!transition.update(state, at(120000))); // 原 ID 7 的到期无效。
  gate.set_mode(0); // 遥控接管后先回待机，再接收自动模式反馈。
  gate.set_mode(1);
  state.veh_mode_st = 1;
  state.auto_drive_enabled = gate.enabled();
  assert(!transition.update(state, at(120500)));
  gate.set_start_command(true);
  state.auto_drive_enabled = gate.enabled();
  assert(transition.update(state, at(121000)) == 7);
  assert(!transition.update(state, at(181000))); // 原 ID 14 周期已取消。
  assert(transition.update(state, at(241000)) == 7);
}

int main()
{
  test_entry_without_movement_and_unknown_running_state();
  test_moving_entry_merge_and_period();
  test_stopping_restarting_and_mode_exit();
  test_button_independence_and_existing_p0_scope();
  test_p0_timeout_and_fixed_period();
  test_independent_entry_and_cycle_deadlines();
  test_stop_and_exit_cancel_only_relevant_requests();
  test_delayed_checks_and_auto_remote_transition();
  std::cout << "Remote voice checks passed\n";
}
