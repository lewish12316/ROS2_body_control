#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t milliseconds)
{
  return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

VoiceSceneState locked_state()
{
  VoiceSceneState state;
  state.trunk_lock_state_received = true;
  state.trunk_lock_st = true;
  return state;
}

void test_first_feedback_and_exactly_three_rounds()
{
  for (const bool first_locked : {false, true}) {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    assert(!scheduler.update(state, at(0)));
    state.trunk_lock_state_received = true;
    state.trunk_lock_st = first_locked;
    assert(!scheduler.update(state, at(1000)));
    assert(!scheduler.update(state, at(181000))); // 首次状态不启动三轮。
    state.trunk_lock_st = true;
    assert(!scheduler.update(state, at(182000)));
    state.trunk_lock_st = false;
    assert(scheduler.update(state, at(183000)) == 5);
    assert(!scheduler.update(state, at(183000))); // ID 2 不立即发送。
    assert(!scheduler.update(state, at(200000))); // 重复未锁不重置起点。
    for (const int due : {243000, 303000, 363000}) {
      assert(!scheduler.update(state, at(due - 1)));
      assert(scheduler.update(state, at(due)) == 2);
      assert(!scheduler.update(state, at(due)));
    }
    assert(!scheduler.update(state, at(423000))); // 不产生第四轮。
    assert(!scheduler.update(state, at(1000000)));
  }
}

void test_id5_timeout_does_not_change_reminder_origin()
{
  VoiceScheduler scheduler;
  auto state = locked_state();
  state.running_state_received = true;
  state.vehicle_running = false;
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(0)) == 16);
  state.trunk_lock_st = false;
  assert(!scheduler.update(state, at(1000)));
  assert(!scheduler.update(state, at(4000))); // ID 5 已超时，但开锁事件仍在 1000ms。
  state.alarm_diag_st = 0;
  assert(!scheduler.update(state, at(5000)));
  assert(!scheduler.update(state, at(60999)));
  assert(scheduler.update(state, at(61000)) == 2);
  assert(scheduler.update(state, at(121000)) == 2);
  assert(scheduler.update(state, at(181000)) == 2);
  assert(!scheduler.update(state, at(241000)));
}

void test_fault_timeout_boundaries_and_fixed_rounds()
{
  for (const int clear_time : {70999, 71000, 71001}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    state.running_state_received = true;
    state.vehicle_running = false;
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(0)) == 16);
    state.trunk_lock_st = false;
    assert(!scheduler.update(state, at(1000)));
    assert(scheduler.update(state, at(61000)) == 16);
    assert(!scheduler.update(state, at(61050))); // P0 两次请求之间也不插播。
    assert(scheduler.update(state, at(65000)) == 16);
    state.alarm_diag_st = 0;
    const auto result = scheduler.update(state, at(clear_time));
    if (clear_time < 71000) {assert(result == 2);} else {assert(!result);}
    assert(!scheduler.update(state, at(120999)));
    assert(scheduler.update(state, at(121000)) == 2); // 非实际发送时刻再加60秒。
    assert(scheduler.update(state, at(181000)) == 2);
    assert(!scheduler.update(state, at(241000))); // 丢弃一轮也不补偿第四轮。
  }
}

void test_all_existing_p0_scenes_take_priority()
{
  for (const int id : {15, 16, 12, 8, 11, 10, 9}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    state.veh_mode_st = 1;
    state.auto_drive_enabled = true;
    assert(!scheduler.update(state, at(0)));
    state.trunk_lock_st = false;
    assert(scheduler.update(state, at(1000)) == 5);
    if (id == 15 || id == 16) {
      state.alarm_diag_st = 1;
      state.running_state_received = true;
      state.vehicle_running = id == 15;
    }
    if (id == 12) {state.veh_gear_st = 1;}
    if (id == 8) {state.obs_detect_st = true;}
    if (id == 11) {
      state.running_state_received = true;
      state.vehicle_running = false;
      state.pedestrian_crossing = true;
    }
    if (id == 10) {state.turn_state = 1;}
    if (id == 9) {state.turn_state = 2;}
    assert(scheduler.update(state, at(61000)) == id);
    assert(!scheduler.update(state, at(61050)));
    state.alarm_diag_st = 0;
    state.veh_gear_st = 0;
    state.obs_detect_st = false;
    state.pedestrian_crossing = false;
    state.turn_state = 0;
    assert(scheduler.update(state, at(70000)) == 2);
  }
}

void test_low_battery_blocking_and_timeout()
{
  for (const int low_start : {60000, 61000}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    state.veh_mode_st = 1;
    assert(!scheduler.update(state, at(0)));
    state.trunk_lock_st = false;
    assert(scheduler.update(state, at(1000)) == 5);
    state.auto_drive_enabled = true;
    state.running_state_received = true;
    state.vehicle_running = false;
    state.battery_low_power_st = true;
    assert(scheduler.update(state, at(low_start)) == 17);
    assert(!scheduler.update(state, at(low_start + 5000)));
    assert(scheduler.update(state, at(low_start + 10000)) == 17);
    // 新增 ID 1 已到期，先于 ID 2，但不得延长 ID 2 的原期限。
    assert(scheduler.update(state, at(low_start + 10050)) == 1);
    const auto result = scheduler.update(state, at(low_start + 10100));
    if (low_start + 10050 < 71000) {assert(result == 2);} else {assert(!result);}
    assert(scheduler.update(state, at(121000)) == 1);
    assert(scheduler.update(state, at(121050)) == 2);
  }
}

void test_relock_cancels_and_new_unlock_restarts()
{
  VoiceScheduler scheduler;
  auto state = locked_state();
  assert(!scheduler.update(state, at(0)));
  state.trunk_lock_st = false;
  assert(scheduler.update(state, at(1000)) == 5);
  assert(scheduler.update(state, at(61000)) == 2);
  state.trunk_lock_st = true;
  assert(!scheduler.update(state, at(62000)));
  assert(!scheduler.update(state, at(121000)));
  assert(!scheduler.update(state, at(181000)));
  state.trunk_lock_st = false;
  assert(scheduler.update(state, at(200000)) == 5);
  assert(!scheduler.update(state, at(259999)));
  assert(scheduler.update(state, at(260000)) == 2);

  VoiceScheduler pending;
  state = locked_state();
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(pending.update(state, at(0)) == 16);
  state.trunk_lock_st = false;
  assert(!pending.update(state, at(1000)));
  assert(pending.update(state, at(61000)) == 16); // 首轮提醒待播。
  state.trunk_lock_st = true;
  assert(pending.update(state, at(65000)) == 16);
  state.alarm_diag_st = 0;
  assert(!pending.update(state, at(66000)));
  assert(!pending.update(state, at(121000)));
  state.trunk_lock_st = false;
  assert(pending.update(state, at(122000)) == 5);
  assert(!pending.update(state, at(181999)));
  assert(pending.update(state, at(182000)) == 2);

  VoiceScheduler early_cancel;
  state = locked_state();
  assert(!early_cancel.update(state, at(0)));
  state.trunk_lock_st = false;
  assert(early_cancel.update(state, at(1000)) == 5);
  state.trunk_lock_st = true;
  assert(!early_cancel.update(state, at(2000))); // 首轮尚未开始时也可取消。
  assert(!early_cancel.update(state, at(61000)));
}

void test_modes_and_other_vehicle_state_do_not_cancel()
{
  VoiceScheduler scheduler;
  auto state = locked_state();
  assert(!scheduler.update(state, at(0)));
  state.trunk_lock_st = false;
  assert(scheduler.update(state, at(1000)) == 5);
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  state.running_state_received = true;
  state.vehicle_running = true;
  state.veh_gear_st = 3;
  assert(scheduler.update(state, at(30000)) == 7);
  state.veh_mode_st = 2;
  state.auto_drive_enabled = false;
  assert(scheduler.update(state, at(40000)) == 14);
  state.veh_mode_st = 0;
  assert(!scheduler.update(state, at(50000)));
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  assert(scheduler.update(state, at(51000)) == 3);
  assert(scheduler.update(state, at(51050)) == 4);
  state.veh_arrive_st = false;
  state.veh_epb_st = false;
  state.veh_gear_st = 0;
  assert(!scheduler.update(state, at(52000)));
  assert(scheduler.update(state, at(61000)) == 2);
  assert(scheduler.update(state, at(121000)) == 2);
  assert(scheduler.update(state, at(181000)) == 2);
}

void test_p1_priority_and_original_deadlines()
{
  VoiceScheduler remote;
  auto state = locked_state();
  assert(!remote.update(state, at(0)));
  state.veh_mode_st = 2;
  state.running_state_received = true;
  state.vehicle_running = true;
  state.trunk_lock_st = false;
  assert(remote.update(state, at(1000)) == 14);
  assert(remote.update(state, at(1050)) == 5);
  assert(remote.update(state, at(61000)) == 2);
  assert(remote.update(state, at(61050)) == 14);
  assert(remote.update(state, at(121000)) == 2);
  assert(remote.update(state, at(121050)) == 14);

  // 同级被延迟时仍沿用原来的 10 秒期限。
  VoiceScheduler near_timeout;
  state = locked_state();
  assert(!near_timeout.update(state, at(0)));
  state.veh_mode_st = 2;
  state.running_state_received = true;
  state.vehicle_running = true;
  state.trunk_lock_st = false;
  assert(near_timeout.update(state, at(1000)) == 14);
  assert(near_timeout.update(state, at(1050)) == 5);
  state.alarm_diag_st = 1;
  assert(near_timeout.update(state, at(60000)) == 15);
  assert(!near_timeout.update(state, at(61000)));
  state.alarm_diag_st = 0;
  assert(near_timeout.update(state, at(70999)) == 2);
  assert(!near_timeout.update(state, at(71000))); // ID 14 在原 deadline 被丢弃。
  assert(near_timeout.update(state, at(121000)) == 2);
  assert(near_timeout.update(state, at(121050)) == 14);

  VoiceScheduler forward;
  state = locked_state();
  state.veh_mode_st = 1;
  state.running_state_received = true;
  state.vehicle_running = true;
  state.veh_gear_st = 3;
  assert(!forward.update(state, at(0)));
  state.auto_drive_enabled = true;
  state.trunk_lock_st = false;
  assert(forward.update(state, at(1000)) == 7);
  assert(forward.update(state, at(1050)) == 5);
  assert(forward.update(state, at(61000)) == 1);
  assert(forward.update(state, at(61050)) == 2);
  assert(forward.update(state, at(121000)) == 1);
  assert(forward.update(state, at(121050)) == 2);
  assert(forward.update(state, at(121100)) == 7);
  assert(forward.update(state, at(181000)) == 1);
  assert(forward.update(state, at(181050)) == 2);
  assert(forward.update(state, at(241000)) == 1);
  assert(forward.update(state, at(241050)) == 7); // 没有第四轮 ID 2。
}

void test_delayed_checks_skip_expired_rounds()
{
  for (const int check_time : {181000, 190999, 191000, 500000}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    assert(!scheduler.update(state, at(0)));
    state.trunk_lock_st = false;
    assert(scheduler.update(state, at(1000)) == 5);
    const auto request = scheduler.update(state, at(check_time));
    if (check_time < 191000) {assert(request == 2);} else {assert(!request);}
    assert(!scheduler.update(state, at(check_time))); // 不集中补发此前轮次。
    assert(!scheduler.update(state, at(600000)));
  }
  VoiceScheduler scheduler;
  auto state = locked_state();
  assert(!scheduler.update(state, at(0)));
  state.trunk_lock_st = false;
  assert(scheduler.update(state, at(1000)) == 5);
  assert(!scheduler.update(state, at(150000))); // 前两轮均已超时。
  assert(scheduler.update(state, at(181000)) == 2);
  assert(!scheduler.update(state, at(241000)));
}

void test_p1_reminder_before_p2_arrival()
{
  VoiceScheduler scheduler;
  auto state = locked_state();
  assert(!scheduler.update(state, at(0)));
  state.trunk_lock_st = false;
  assert(scheduler.update(state, at(1000)) == 5);
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  assert(scheduler.update(state, at(61000)) == 2);
  assert(scheduler.update(state, at(61050)) == 3);
  assert(scheduler.update(state, at(61100)) == 4);
}

int main()
{
  test_first_feedback_and_exactly_three_rounds();
  test_id5_timeout_does_not_change_reminder_origin();
  test_fault_timeout_boundaries_and_fixed_rounds();
  test_all_existing_p0_scenes_take_priority();
  test_low_battery_blocking_and_timeout();
  test_relock_cancels_and_new_unlock_restarts();
  test_modes_and_other_vehicle_state_do_not_cancel();
  test_p1_priority_and_original_deadlines();
  test_delayed_checks_skip_expired_rounds();
  test_p1_reminder_before_p2_arrival();
  std::cout << "Cargo completion voice checks passed\n";
}
