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

void test_initial_feedback_edges_and_repeated_messages()
{
  for (const bool first_locked : {false, true}) {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    assert(!scheduler.update(state, at(0))); // 未收到状态，不使用默认值构造事件。
    state.trunk_lock_state_received = true;
    state.trunk_lock_st = first_locked;
    assert(!scheduler.update(state, at(1000))); // 首条消息无论 true/false 都不播报。
    assert(!scheduler.update(state, at(2000)));
    state.trunk_lock_st = true;
    assert(!scheduler.update(state, at(3000)));
    state.trunk_lock_st = false;
    assert(scheduler.update(state, at(4000)) == 5);
    for (const int t : {4000, 4050, 5000, 7000, 10000}) {
      assert(!scheduler.update(state, at(t))); // 同一次转换只请求一次。
    }
    state.trunk_lock_st = true;
    assert(!scheduler.update(state, at(11000)));
    state.trunk_lock_st = false;
    assert(scheduler.update(state, at(12000)) == 5);
  }
}

void test_modes_and_start_independence()
{
  for (const int mode : {0, 1, 2}) {
    for (const bool started : {false, true}) {
      VoiceScheduler scheduler;
      auto state = locked_state();
      state.veh_mode_st = mode;
      state.auto_drive_enabled = mode == 1 && started;
      const auto initial = scheduler.update(state, at(0));
      if (mode == 2) {assert(initial == 14);} else {assert(!initial);}
      state.trunk_lock_st = false;
      assert(scheduler.update(state, at(1000)) == 5);
      assert(!scheduler.update(state, at(2000)));
    }
  }
}

void test_p0_priority_and_three_second_deadline()
{
  for (const int id : {15, 16, 12, 8, 11, 10, 9}) {
    for (const int clear_time : {3999, 4000, 4001}) {
      VoiceScheduler scheduler;
      auto state = locked_state();
      state.veh_mode_st = 1;
      state.auto_drive_enabled = true;
      if (id == 15 || id == 16) {
        state.alarm_diag_st = 1;
        state.running_state_received = true;
        state.vehicle_running = id == 15;
      }
      if (id == 12) {state.veh_gear_st = 1;}
      if (id == 8) {state.obs_detect_st = true;}
      if (id == 11) {
        state.pedestrian_crossing = true;
        state.running_state_received = true;
        state.vehicle_running = false;
      }
      if (id == 10) {state.turn_state = 1;}
      if (id == 9) {state.turn_state = 2;}
      assert(scheduler.update(state, at(0)) == id);
      state.trunk_lock_st = false;
      assert(!scheduler.update(state, at(1000))); // 事件此时开始计时，P0 间隙也不插播。
      assert(!scheduler.update(state, at(2000))); // 重复 false 不延长期限。
      state.alarm_diag_st = 0;
      state.veh_gear_st = 0;
      state.obs_detect_st = false;
      state.pedestrian_crossing = false;
      state.turn_state = 0;
      const auto result = scheduler.update(state, at(clear_time));
      if (clear_time < 4000) {assert(result == 5);} else {assert(!result);}
      assert(!scheduler.update(state, at(5000))); // 超时后旧 false 不重试。
      state.trunk_lock_st = true;
      assert(!scheduler.update(state, at(6000)));
      state.trunk_lock_st = false;
      assert(scheduler.update(state, at(7000)) == 5); // 新转换才允许重开。
    }
  }
}

void test_relock_cancels_and_new_edge_gets_new_deadline()
{
  for (const int clear_time : {6499, 6500}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    state.alarm_diag_st = 1;
    state.running_state_received = true;
    state.vehicle_running = false;
    assert(scheduler.update(state, at(0)) == 16);
    state.trunk_lock_st = false;
    assert(!scheduler.update(state, at(1000)));
    state.trunk_lock_st = true;
    assert(!scheduler.update(state, at(2000))); // 取消第一次开锁请求。
    state.trunk_lock_st = false;
    assert(!scheduler.update(state, at(3500))); // 真实的新转换有新期限。
    assert(scheduler.update(state, at(5000)) == 16);
    state.alarm_diag_st = 0;
    const auto result = scheduler.update(state, at(clear_time));
    if (clear_time < 6500) {assert(result == 5);} else {assert(!result);}
    assert(!scheduler.update(state, at(7000)));
  }
  VoiceScheduler scheduler;
  auto state = locked_state();
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(scheduler.update(state, at(0)) == 16);
  state.trunk_lock_st = false;
  assert(!scheduler.update(state, at(1000)));
  state.trunk_lock_st = true;
  assert(!scheduler.update(state, at(2000)));
  state.alarm_diag_st = 0;
  assert(!scheduler.update(state, at(2500))); // 重新上锁后不能播报过时的成功结果。
  state.trunk_lock_st = false;
  assert(scheduler.update(state, at(3000)) == 5);
}

void test_other_state_changes_do_not_cancel()
{
  VoiceScheduler scheduler;
  auto state = locked_state();
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(scheduler.update(state, at(0)) == 16);
  state.trunk_lock_st = false;
  assert(!scheduler.update(state, at(1000)));
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  assert(!scheduler.update(state, at(2000)));
  state.auto_drive_enabled = false;
  assert(!scheduler.update(state, at(2200)));
  state.veh_arrive_st = true;
  state.veh_gear_st = 3;
  state.veh_epb_st = false;
  assert(!scheduler.update(state, at(2300)));
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  assert(!scheduler.update(state, at(2400)));
  state.veh_mode_st = 2;
  assert(!scheduler.update(state, at(2500)));
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(3000)) == 14); // 已到期 P1 仍优先。
  assert(scheduler.update(state, at(3050)) == 5);
  assert(scheduler.update(state, at(3100)) == 3);
  assert(scheduler.update(state, at(3150)) == 4);
  assert(!scheduler.update(state, at(4000)));
}

void test_p1_and_low_battery_priority()
{
  for (const int clear_time : {2000, 3999}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    state.veh_mode_st = 1;
    state.auto_drive_enabled = true;
    state.running_state_received = true;
    state.vehicle_running = true;
    state.veh_gear_st = 3;
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(0)) == 15);
    state.trunk_lock_st = false;
    assert(!scheduler.update(state, at(1000)));
    state.alarm_diag_st = 0;
    assert(scheduler.update(state, at(clear_time)) == 7);
    const auto result = scheduler.update(state, at(clear_time + 50));
    if (clear_time + 50 < 4000) {assert(result == 5);} else {assert(!result);}
  }
  for (const bool cancel_low : {false, true}) {
    VoiceScheduler scheduler;
    auto state = locked_state();
    state.veh_mode_st = 1;
    state.auto_drive_enabled = true;
    state.running_state_received = true;
    state.vehicle_running = false;
    state.battery_low_power_st = true;
    assert(scheduler.update(state, at(0)) == 17);
    state.trunk_lock_st = false;
    assert(!scheduler.update(state, at(1000)));
    if (cancel_low) {
      state.battery_low_power_st = false;
      assert(scheduler.update(state, at(2000)) == 5);
    } else {
      assert(!scheduler.update(state, at(4000))); // 等待低电量任务时也按原时刻超时。
      assert(scheduler.update(state, at(10000)) == 17);
      assert(!scheduler.update(state, at(10050)));
    }
  }
}

void test_p2_order_and_independent_deadlines()
{
  VoiceScheduler scheduler;
  auto state = locked_state();
  assert(!scheduler.update(state, at(0)));
  state.trunk_lock_st = false;
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  assert(scheduler.update(state, at(1000)) == 5);
  assert(scheduler.update(state, at(1050)) == 3);
  assert(scheduler.update(state, at(1100)) == 4);
  assert(!scheduler.update(state, at(2000)));

  // 较晚的开锁事件不延长原来两条到站请求的 10 秒期限。
  VoiceScheduler cargo_expired;
  state = locked_state();
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(cargo_expired.update(state, at(0)) == 16);
  state.trunk_lock_st = false;
  assert(cargo_expired.update(state, at(8000)) == 16);
  state.alarm_diag_st = 0;
  assert(cargo_expired.update(state, at(9999)) == 5);
  assert(!cargo_expired.update(state, at(10000)));
  assert(!cargo_expired.update(state, at(15000)));

  // ID 5 已超时不妨碍尚在 10 秒期限内的到站两条。
  VoiceScheduler unlock_expired;
  state = locked_state();
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(unlock_expired.update(state, at(0)) == 16);
  state.trunk_lock_st = false;
  assert(!unlock_expired.update(state, at(1000)));
  state.alarm_diag_st = 0;
  assert(unlock_expired.update(state, at(5000)) == 3);
  assert(unlock_expired.update(state, at(5050)) == 4);
  assert(!unlock_expired.update(state, at(6000)));
}

int main()
{
  test_initial_feedback_edges_and_repeated_messages();
  test_modes_and_start_independence();
  test_p0_priority_and_three_second_deadline();
  test_relock_cancels_and_new_edge_gets_new_deadline();
  test_other_state_changes_do_not_cancel();
  test_p1_and_low_battery_priority();
  test_p2_order_and_independent_deadlines();
  std::cout << "Unlock voice checks passed\n";
}
