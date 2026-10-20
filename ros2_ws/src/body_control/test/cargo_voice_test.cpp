#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t milliseconds)
{
  return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

VoiceSceneState cargo_state()
{
  VoiceSceneState state;
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.veh_epb_st = true;
  return state;
}

void test_input_order_and_missing_conditions()
{
  std::array<int, 3> order{0, 1, 2};
  do {
    VoiceScheduler scheduler;
    VoiceSceneState state;
    assert(!scheduler.update(state, at(0)));
    for (int step = 0; step != 3; ++step) {
      if (order[step] == 0) {state.veh_arrive_st = true;}
      if (order[step] == 1) {state.veh_gear_st = 2;}
      if (order[step] == 2) {state.veh_epb_st = true;}
      const auto request = scheduler.update(state, at((step + 1) * 100));
      if (step == 2) {assert(request == 3);} else {assert(!request);}
    }
    assert(!scheduler.update(state, at(10000)));
  } while (std::next_permutation(order.begin(), order.end()));

  for (int missing = 0; missing != 3; ++missing) {
    VoiceScheduler scheduler;
    auto state = cargo_state();
    if (missing == 0) {state.veh_arrive_st = false;}
    if (missing == 1) {state.veh_gear_st = 0;}
    if (missing == 2) {state.veh_epb_st = false;}
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(15000))); // 条件未齐时尚未开始 10 秒计时。
    state = cargo_state();
    assert(scheduler.update(state, at(20000)) == 3);
  }
}

void test_boot_modes_and_once_per_arrival()
{
  for (const int mode : {0, 1, 2}) {
    for (const bool started : {false, true}) {
      VoiceScheduler scheduler;
      auto state = cargo_state();
      state.veh_mode_st = mode;
      state.auto_drive_enabled = (mode == 1) && started;
      auto request = scheduler.update(state, at(0));
      if (mode == 2) {
        assert(request == 14); // P1 切入先发送，P2 下一次检查发送。
        request = scheduler.update(state, at(50));
      }
      assert(request == 3); // 无需先见到“未到站”或自动驾驶启动。
      assert(!scheduler.update(state, at(1000)));
      state.veh_gear_st = 3;
      assert(!scheduler.update(state, at(2000)));
      state.veh_gear_st = 2;
      assert(!scheduler.update(state, at(3000)));
      state.veh_epb_st = false;
      assert(!scheduler.update(state, at(4000)));
      state.veh_epb_st = true;
      assert(!scheduler.update(state, at(5000))); // 抖动不重开已消费机会。
      state.veh_arrive_st = false;
      assert(!scheduler.update(state, at(6000)));
      state.veh_arrive_st = true;
      assert(scheduler.update(state, at(7000)) == 3);
      assert(!scheduler.update(state, at(20000)));
    }
  }
}

void test_p0_priority_and_timeout_boundaries()
{
  for (const int id : {15, 16, 8, 11, 10, 9}) {
    for (const int clear_time : {9999, 10000, 10001}) {
      VoiceScheduler scheduler;
      auto state = cargo_state();
      state.veh_mode_st = 1;
      state.auto_drive_enabled = true;
      if (id == 15 || id == 16) {
        state.alarm_diag_st = 1;
        state.running_state_received = true;
        state.vehicle_running = id == 15;
      }
      if (id == 8) {state.obs_detect_st = true;}
      if (id == 11) {
        state.pedestrian_crossing = true;
        state.running_state_received = true;
        state.vehicle_running = false;
      }
      if (id == 10) {state.turn_state = 1;}
      if (id == 9) {state.turn_state = 2;}
      assert(scheduler.update(state, at(0)) == id);
      assert(!scheduler.update(state, at(1000))); // P0 间隙也阻塞 P2。
      assert(scheduler.update(state, at(5000)) == id);
      state.alarm_diag_st = 0;
      state.obs_detect_st = false;
      state.pedestrian_crossing = false;
      state.turn_state = 0;
      const auto result = scheduler.update(state, at(clear_time));
      if (clear_time < 10000) {assert(result == 3);} else {assert(!result);}
      assert(!scheduler.update(state, at(20000))); // 超时后也不能重新建立请求。
      state.veh_arrive_st = false;
      assert(!scheduler.update(state, at(21000)));
      state.veh_arrive_st = true;
      assert(scheduler.update(state, at(22000)) == 3);
    }
  }
}

void test_pending_cancel_and_no_retry_until_departure()
{
  for (const bool release_epb : {false, true}) {
    VoiceScheduler scheduler;
    auto state = cargo_state();
    state.running_state_received = true;
    state.vehicle_running = false;
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(0)) == 16);
    if (release_epb) {state.veh_epb_st = false;} else {state.veh_gear_st = 3;}
    assert(!scheduler.update(state, at(1000)));
    state.veh_epb_st = true;
    state.veh_gear_st = 2;
    assert(!scheduler.update(state, at(2000)));
    state.alarm_diag_st = 0;
    assert(!scheduler.update(state, at(3000))); // 条件恢复不重试。
    assert(!scheduler.update(state, at(11000)));
    state.veh_arrive_st = false;
    assert(!scheduler.update(state, at(12000)));
    state.veh_arrive_st = true;
    assert(scheduler.update(state, at(13000)) == 3);
  }
  VoiceScheduler scheduler;
  auto state = cargo_state();
  state.running_state_received = true;
  state.vehicle_running = false;
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(0)) == 16);
  state.veh_arrive_st = false;
  assert(!scheduler.update(state, at(1000)));
  state.alarm_diag_st = 0;
  assert(!scheduler.update(state, at(2000)));
  state.veh_arrive_st = true;
  assert(scheduler.update(state, at(3000)) == 3); // 离站后新到站可重新触发。
}

void test_mode_and_button_changes_do_not_cancel()
{
  VoiceScheduler scheduler;
  auto state = cargo_state();
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  state.alarm_diag_st = 1;
  state.running_state_received = true;
  state.vehicle_running = false;
  assert(scheduler.update(state, at(0)) == 16);
  state.auto_drive_enabled = false; // 模拟停止按钮。
  assert(!scheduler.update(state, at(1000)));
  state.veh_mode_st = 0;
  assert(!scheduler.update(state, at(2000)));
  state.veh_mode_st = 2;
  assert(!scheduler.update(state, at(3000)));
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(4000)) == 14);
  assert(scheduler.update(state, at(4050)) == 3); // 原 P2 请求仍有效。
}

void test_p1_dispatch_and_original_timeout()
{
  for (const int clear_time : {9000, 9999}) {
    VoiceScheduler scheduler;
    auto state = cargo_state();
    state.veh_mode_st = 2;
    state.running_state_received = true;
    state.vehicle_running = false;
    state.alarm_diag_st = 1;
    assert(scheduler.update(state, at(0)) == 16);
    state.alarm_diag_st = 0;
    assert(scheduler.update(state, at(clear_time)) == 14); // 待播 P1 先发。
    const int next_check = clear_time + 50;
    const auto result = scheduler.update(state, at(next_check));
    if (next_check < 10000) {assert(result == 3);} else {assert(!result);}
    assert(!scheduler.update(state, at(20000)));
  }

  // ID 7 与 N 挡条件互斥，退出 D 挡后取消其待播，不让旧 P1 阻塞 P2。
  VoiceScheduler scheduler;
  auto state = cargo_state();
  state.veh_arrive_st = false;
  state.veh_gear_st = 3;
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  state.running_state_received = true;
  state.vehicle_running = true;
  state.turn_state = 1;
  assert(scheduler.update(state, at(0)) == 10);
  state.veh_arrive_st = true;
  state.veh_gear_st = 2;
  state.vehicle_running = false;
  assert(!scheduler.update(state, at(1000)));
  state.turn_state = 0;
  assert(scheduler.update(state, at(2000)) == 3);
}

void test_low_battery_task_priority()
{
  for (const bool cancel_battery : {false, true}) {
    VoiceScheduler scheduler;
    auto state = cargo_state();
    state.veh_mode_st = 1;
    state.auto_drive_enabled = true;
    state.running_state_received = true;
    state.vehicle_running = false;
    state.battery_low_power_st = true;
    assert(scheduler.update(state, at(0)) == 17);
    assert(!scheduler.update(state, at(1000))); // 两次低电量请求间也阻塞 P2。
    if (cancel_battery) {
      state.battery_low_power_st = false;
      assert(scheduler.update(state, at(5000)) == 3);
    } else {
      assert(!scheduler.update(state, at(9999)));
      assert(scheduler.update(state, at(10000)) == 17);
      assert(!scheduler.update(state, at(10050))); // ID 3 已按原时刻超时。
      state.veh_arrive_st = false;
      assert(!scheduler.update(state, at(11000)));
      state.veh_arrive_st = true;
      assert(scheduler.update(state, at(12000)) == 3);
    }
  }
}

int main()
{
  test_input_order_and_missing_conditions();
  test_boot_modes_and_once_per_arrival();
  test_p0_priority_and_timeout_boundaries();
  test_pending_cancel_and_no_retry_until_departure();
  test_mode_and_button_changes_do_not_cancel();
  test_p1_dispatch_and_original_timeout();
  test_low_battery_task_priority();
  std::cout << "Cargo voice checks passed\n";
}
