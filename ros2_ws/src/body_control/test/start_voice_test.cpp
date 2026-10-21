#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>

#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t ms)
{return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(ms);}

VoiceSceneState stationary()
{
  VoiceSceneState s;
  s.veh_mode_st = 1; s.auto_drive_enabled = true;
  s.running_state_received = true; s.vehicle_running = false;
  s.speed_state_received = true; s.veh_speed_st = 1.0F;
  return s;
}

void test_initial_feedback_and_repeat_starts()
{
  VoiceScheduler scheduler;
  auto s = stationary();
  s.running_state_received = false;
  assert(!scheduler.update(s, at(0)));
  s.running_state_received = true; s.vehicle_running = true;
  assert(!scheduler.update(s, at(1000))); // 首次 true 只初始化。
  assert(!scheduler.update(s, at(2000)));
  for (int t : {3000, 10000, 20000}) {
    s.vehicle_running = false; assert(!scheduler.update(s, at(t)));
    s.vehicle_running = true; assert(scheduler.update(s, at(t + 1)) == 6);
    assert(!scheduler.update(s, at(t + 2)));
    assert(!scheduler.update(s, at(t + 3001)));
  }
}

void test_authorization_and_modes()
{
  for (int mode : {0, 1, 2}) {
    VoiceScheduler scheduler;
    auto s = stationary(); s.veh_mode_st = mode; s.auto_drive_enabled = false;
    const auto first = scheduler.update(s, at(0));
    if (mode == 2) {assert(first == 14);} else {assert(!first);}
    s.vehicle_running = true;
    const auto moving = scheduler.update(s, at(100));
    if (mode == 2) {assert(moving == 14);} else {assert(!moving);}
    s.veh_mode_st = 1; s.auto_drive_enabled = true;
    assert(!scheduler.update(s, at(200))); // 授权外的起步不缓存。
    assert(!scheduler.update(s, at(1000)));
    s.vehicle_running = false; assert(!scheduler.update(s, at(1100)));
    s.vehicle_running = true; assert(scheduler.update(s, at(1200)) == 6);
  }
  VoiceScheduler scheduler;
  auto s = stationary(); s.auto_drive_enabled = false;
  assert(!scheduler.update(s, at(0)));
  s.auto_drive_enabled = true; assert(!scheduler.update(s, at(100)));
  s.vehicle_running = true; assert(scheduler.update(s, at(200)) == 6);
}

void test_speed_values_and_late_feedback()
{
  for (float speed : {0.0F, 4.999F, 5.0F, 6.0F}) {
    VoiceScheduler scheduler;
    auto s = stationary(); s.veh_speed_st = speed;
    assert(!scheduler.update(s, at(0)));
    s.vehicle_running = true;
    const auto result = scheduler.update(s, at(1000));
    if (speed < 5.0F) {assert(result == 6);} else {assert(!result);}
    s.veh_speed_st = 1.0F;
    assert(!scheduler.update(s, at(1100))); // 已发或超速取消，同次行驶不重试。
  }
  for (int received_at : {1000, 3999, 4000}) {
    VoiceScheduler scheduler;
    auto s = stationary(); s.speed_state_received = false; s.veh_speed_st = 0.0F;
    assert(!scheduler.update(s, at(0)));
    s.vehicle_running = true; assert(!scheduler.update(s, at(1000)));
    s.speed_state_received = true;
    const auto result = scheduler.update(s, at(received_at));
    if (received_at < 4000) {assert(result == 6);} else {assert(!result);}
    assert(!scheduler.update(s, at(5000)));
  }
  const float infinity = std::numeric_limits<float>::infinity();
  for (float invalid : {-1.0F, infinity, -infinity, std::numeric_limits<float>::quiet_NaN()}) {
    for (int repaired : {3999, 4000}) {
      VoiceScheduler scheduler;
      auto s = stationary(); s.veh_speed_st = invalid;
      assert(!scheduler.update(s, at(0)));
      s.vehicle_running = true; assert(!scheduler.update(s, at(1000)));
      assert(!scheduler.update(s, at(2000)));
      s.veh_speed_st = 0.0F;
      const auto result = scheduler.update(s, at(repaired));
      if (repaired < 4000) {assert(result == 6);} else {assert(!result);}
    }
  }
}

void test_p0_blocking_timeout_and_duplicates()
{
  for (int p0 : {15, 12, 8, 10, 9}) {
    for (int clear : {3999, 4000}) {
      VoiceScheduler scheduler;
      auto s = stationary(); assert(!scheduler.update(s, at(0)));
      s.vehicle_running = true;
      if (p0 == 15) {s.alarm_diag_st = 1;}
      if (p0 == 12) {s.veh_gear_st = 1;}
      if (p0 == 8) {s.obs_detect_st = true;}
      if (p0 == 10) {s.turn_state = 1;}
      if (p0 == 9) {s.turn_state = 2;}
      assert(scheduler.update(s, at(1000)) == p0);
      assert(!scheduler.update(s, at(2000))); // 重复输入与 P0 间隙不发送、不延长。
      assert(!scheduler.update(s, at(3998)));
      s.alarm_diag_st = 0; s.veh_gear_st = 0; s.obs_detect_st = false; s.turn_state = 0;
      const auto result = scheduler.update(s, at(clear));
      if (clear < 4000) {assert(result == 6);} else {assert(!result);}
      assert(!scheduler.update(s, at(5000)));
    }
  }
}

void test_cancellation_while_blocked()
{
  for (int reason = 0; reason < 5; ++reason) {
    VoiceScheduler scheduler;
    auto s = stationary(); assert(!scheduler.update(s, at(0)));
    s.alarm_diag_st = 1; s.vehicle_running = true;
    assert(scheduler.update(s, at(1000)) == 15);
    if (reason == 0) {s.vehicle_running = false;}
    if (reason == 1) {s.auto_drive_enabled = false;}
    if (reason == 2) {s.veh_mode_st = 0;}
    if (reason == 3) {s.veh_speed_st = 5.0F;}
    if (reason == 4) {s.veh_speed_st = 8.0F;}
    const auto canceled = scheduler.update(s, at(2000));
    if (reason == 0) {assert(canceled == 16);} else {assert(!canceled);}
    s.alarm_diag_st = 0;
    assert(!scheduler.update(s, at(2100)));
    s.auto_drive_enabled = true; s.veh_mode_st = 1; s.veh_speed_st = 1.0F;
    if (reason == 0) {
      s.vehicle_running = true; assert(scheduler.update(s, at(2200)) == 6);
    } else {
      assert(!scheduler.update(s, at(2200))); // 减速或重新授权不恢复已取消事件。
      s.vehicle_running = false; assert(!scheduler.update(s, at(2300)));
      s.vehicle_running = true; assert(scheduler.update(s, at(2400)) == 6);
    }
  }
  // 阻塞期间非法车速替换旧的低速值，必须等有效值补齐，期限不重置。
  VoiceScheduler scheduler;
  auto s = stationary(); assert(!scheduler.update(s, at(0)));
  s.vehicle_running = true; s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(1000)) == 15);
  s.veh_speed_st = -1.0F; assert(!scheduler.update(s, at(2000)));
  s.alarm_diag_st = 0; assert(!scheduler.update(s, at(3000)));
  s.veh_speed_st = 1.0F; assert(scheduler.update(s, at(3999)) == 6);
}

void test_priority_and_no_extra_gates()
{
  VoiceScheduler scheduler;
  auto s = stationary();
  s.trunk_lock_state_received = true; s.trunk_lock_st = true;
  assert(!scheduler.update(s, at(0)));
  s.trunk_lock_st = false; assert(scheduler.update(s, at(0)) == 5);
  // 所有已就绪的自动模式 P1：1 -> 6 -> 2 -> 7。
  s.vehicle_running = true; s.veh_gear_st = 3;
  assert(scheduler.update(s, at(60000)) == 1);
  assert(scheduler.update(s, at(60000)) == 6);
  assert(scheduler.update(s, at(60000)) == 2);
  assert(scheduler.update(s, at(60000)) == 7);
  assert(!scheduler.update(s, at(60000)));
  // 请求 ID 1 不能延长 ID 6 的 3 秒期限。
  s.vehicle_running = false; assert(!scheduler.update(s, at(69000)));
  s.vehicle_running = true; s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(70000)) == 15);
  s.alarm_diag_st = 0;
  assert(scheduler.update(s, at(72999)) == 1);
  assert(scheduler.update(s, at(73000)) == 7); // 起步过期，ID 7 仍在自身期限内。
  assert(!scheduler.update(s, at(73000)));
  // 到站/EPB/非D挡均不限制 ID 6。R挡本身的P0仍优先。
  for (int gear : {0, 2}) {
    VoiceScheduler other;
    auto state = stationary(); state.veh_gear_st = gear;
    state.veh_arrive_st = true; state.veh_epb_st = true;
    const auto first = other.update(state, at(0));
    if (gear == 2) {assert(first == 3);} else {assert(!first);}
    state.vehicle_running = true;
    assert(other.update(state, at(1000)) == 6);
    if (gear == 2) {assert(other.update(state, at(1050)) == 4);}
  }
}

void test_waiting_speed_does_not_block_and_low_battery_ends()
{
  VoiceScheduler scheduler;
  auto s = stationary(); s.speed_state_received = false;
  s.veh_gear_st = 3;
  assert(!scheduler.update(s, at(0)));
  s.vehicle_running = true;
  assert(scheduler.update(s, at(1000)) == 7); // 等车速不阻塞前进。
  s.speed_state_received = true; assert(scheduler.update(s, at(2000)) == 6);
  assert(!scheduler.update(s, at(2001)));
  VoiceScheduler low;
  s = stationary(); s.battery_low_power_st = true;
  assert(low.update(s, at(0)) == 17);
  s.vehicle_running = true;
  assert(low.update(s, at(1000)) == 6); // 行驶使低电量任务按原规则取消。
  assert(!low.update(s, at(10000)));
  // 只等待车速也不阻塞 P2。
  VoiceScheduler cargo;
  s = stationary(); s.speed_state_received = false;
  assert(!cargo.update(s, at(0)));
  s.vehicle_running = true; s.veh_arrive_st = true; s.veh_gear_st = 2; s.veh_epb_st = true;
  assert(cargo.update(s, at(1000)) == 3);
  assert(cargo.update(s, at(1050)) == 4);
  s.speed_state_received = true; assert(cargo.update(s, at(3999)) == 6);
}

int main()
{
  test_initial_feedback_and_repeat_starts(); test_authorization_and_modes();
  test_speed_values_and_late_feedback(); test_p0_blocking_timeout_and_duplicates();
  test_cancellation_while_blocked(); test_priority_and_no_extra_gates();
  test_waiting_speed_does_not_block_and_low_battery_ends();
  std::cout << "Start voice checks passed\n";
}
