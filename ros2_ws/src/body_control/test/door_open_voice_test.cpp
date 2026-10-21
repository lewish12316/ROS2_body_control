#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t ms)
{return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(ms);}
VoiceSceneState enabled()
{
  VoiceSceneState s;
  s.veh_mode_st = 1; s.auto_drive_enabled = true;
  s.trunk_lock_state_received = true; s.trunk_lock_st = false;
  return s;
}

void test_period_and_conditions()
{
  VoiceScheduler scheduler;
  auto s = enabled();
  assert(!scheduler.update(s, at(1000)));
  for (int t : {1001, 5000, 10999}) {assert(!scheduler.update(s, at(t)));}
  for (int due : {11000, 21000, 31000, 41000}) {
    assert(scheduler.update(s, at(due)) == 1);
    assert(!scheduler.update(s, at(due)));
    assert(!scheduler.update(s, at(due + 9999)));
  }
  for (int missing = 0; missing < 5; ++missing) {
    DoorOpenVoiceTask task;
    s = enabled();
    if (missing == 0) {s.veh_mode_st = 0;}
    if (missing == 1) {s.auto_drive_enabled = false;}
    if (missing == 2) {s.trunk_lock_state_received = false;}
    if (missing == 3) {s.trunk_lock_st = true;}
    if (missing == 4) {s.veh_mode_st = 2;}
    task.observe(s, at(0)); task.observe(s, at(50000));
    assert(!task.take_due(at(50000)));
    s = enabled(); task.observe(s, at(51000));
    task.observe(s, at(60999)); assert(!task.take_due(at(60999)));
    task.observe(s, at(61000)); assert(task.take_due(at(61000)) == 1);
  }
}

void test_cancel_and_restart()
{
  for (int reason = 0; reason < 3; ++reason) {
    VoiceScheduler scheduler;
    auto s = enabled();
    s.running_state_received = true; s.vehicle_running = false; s.alarm_diag_st = 1;
    assert(scheduler.update(s, at(0)) == 16);
    assert(scheduler.update(s, at(10000)) == 16);
    if (reason == 0) {s.trunk_lock_st = true;}
    if (reason == 1) {s.auto_drive_enabled = false;}
    if (reason == 2) {s.veh_mode_st = 0; s.auto_drive_enabled = false;}
    assert(!scheduler.update(s, at(11000)));
    s.alarm_diag_st = 0; assert(!scheduler.update(s, at(11500)));
    s = enabled();
    const auto result = scheduler.update(s, at(12000));
    if (reason == 0) {assert(result == 5);} else {assert(!result);}
    assert(!scheduler.update(s, at(21999)));
    assert(scheduler.update(s, at(22000)) == 1);
  }
}

void test_p0_timeout_and_fixed_timeline()
{
  for (int p0 : {16, 15, 12, 8, 11, 10, 9}) {
    for (int cleared : {12999, 13000}) {
      VoiceScheduler scheduler;
      auto s = enabled();
      s.running_state_received = true; s.vehicle_running = p0 == 15;
      if (p0 == 15 || p0 == 16) {s.alarm_diag_st = 1;}
      if (p0 == 12) {s.veh_gear_st = 1;}
      if (p0 == 8) {s.obs_detect_st = true;}
      if (p0 == 11) {s.pedestrian_crossing = true;}
      if (p0 == 10) {s.turn_state = 1;}
      if (p0 == 9) {s.turn_state = 2;}
      assert(scheduler.update(s, at(0)) == p0);
      assert(scheduler.update(s, at(10000)) == p0);
      assert(!scheduler.update(s, at(11000))); // P0 间隙也不穿插。
      s.alarm_diag_st = 0; s.veh_gear_st = 0; s.obs_detect_st = false;
      s.pedestrian_crossing = false; s.turn_state = 0;
      const auto result = scheduler.update(s, at(cleared));
      if (cleared == 12999) {assert(result == 1);} else {assert(!result);}
      assert(!scheduler.update(s, at(19999)));
      assert(scheduler.update(s, at(20000)) == 1);
    }
  }
}

void test_low_battery_and_unrelated_states()
{
  for (int start : {0, 1000}) {
    VoiceScheduler scheduler;
    auto s = enabled();
    assert(!scheduler.update(s, at(0)));
    s.running_state_received = true; s.vehicle_running = false;
    s.battery_low_power_st = true;
    assert(scheduler.update(s, at(start)) == 17);
    assert(!scheduler.update(s, at(9999)));
    if (start == 1000) {assert(!scheduler.update(s, at(10000)));}
    assert(scheduler.update(s, at(start + 10000)) == 17);
    assert(scheduler.update(s, at(start + 10001)) == 1);
    assert(scheduler.update(s, at(20000)) == 1);
  }
  {
    VoiceScheduler scheduler;
    auto s = enabled();
    assert(!scheduler.update(s, at(0)));
    s.running_state_received = true; s.vehicle_running = false;
    s.battery_low_power_st = true;
    assert(scheduler.update(s, at(6000)) == 17);
    assert(!scheduler.update(s, at(10000)));
    assert(!scheduler.update(s, at(13000))); // 低电量任务间隙也占用，本轮超时。
    assert(scheduler.update(s, at(16000)) == 17);
    assert(!scheduler.update(s, at(16001))); // 不补发已超时的 ID 1。
    assert(scheduler.update(s, at(20000)) == 1);
  }
  DoorOpenVoiceTask task;
  auto s = enabled(); task.observe(s, at(0));
  s.running_state_received = true; s.vehicle_running = true;
  s.veh_arrive_st = true; s.veh_gear_st = 2; s.veh_epb_st = true;
  task.observe(s, at(9000)); task.observe(s, at(10000));
  assert(task.take_due(at(10000)) == 1);
  s.vehicle_running = false; s.veh_arrive_st = false;
  s.veh_gear_st = 3; s.veh_epb_st = false;
  task.observe(s, at(19999)); assert(!task.take_due(at(19999)));
  task.observe(s, at(20000)); assert(task.take_due(at(20000)) == 1);
}

void test_late_checks()
{
  for (int late : {12000, 13000, 42000, 43000, 1000000}) {
    VoiceScheduler scheduler;
    auto s = enabled(); assert(!scheduler.update(s, at(0)));
    const auto result = scheduler.update(s, at(late));
    if (late % 10000 < 3000) {assert(result == 1);} else {assert(!result);}
    assert(!scheduler.update(s, at(late)));
    const int next = (late / 10000 + 1) * 10000;
    assert(!scheduler.update(s, at(next - 1)));
    assert(scheduler.update(s, at(next)) == 1);
  }
}

void test_priority_and_independent_deadlines()
{
  VoiceScheduler scheduler;
  auto s = enabled(); s.trunk_lock_st = true;
  s.running_state_received = true; s.vehicle_running = true; s.veh_gear_st = 3;
  assert(scheduler.update(s, at(0)) == 7);
  s.trunk_lock_st = false; assert(scheduler.update(s, at(0)) == 5);
  assert(scheduler.update(s, at(60000)) == 1);
  assert(scheduler.update(s, at(60000)) == 2);
  assert(!scheduler.update(s, at(60000)));
  assert(scheduler.update(s, at(120000)) == 1);
  assert(scheduler.update(s, at(120000)) == 2);
  assert(scheduler.update(s, at(120000)) == 7);
  s.alarm_diag_st = 1; assert(scheduler.update(s, at(179999)) == 15);
  assert(!scheduler.update(s, at(180000)));
  s.alarm_diag_st = 0;
  assert(scheduler.update(s, at(190000)) == 1);
  assert(!scheduler.update(s, at(190000))); // ID 2 原定 10 秒期限不延长。
  s.veh_arrive_st = true; s.veh_gear_st = 2; s.veh_epb_st = true;
  assert(scheduler.update(s, at(200000)) == 1);
  assert(scheduler.update(s, at(200000)) == 3);
  assert(scheduler.update(s, at(200000)) == 4);
}

int main()
{
  test_period_and_conditions(); test_cancel_and_restart();
  test_p0_timeout_and_fixed_timeline(); test_low_battery_and_unrelated_states();
  test_late_checks(); test_priority_and_independent_deadlines();
  std::cout << "door_open_voice_test passed\n";
}
