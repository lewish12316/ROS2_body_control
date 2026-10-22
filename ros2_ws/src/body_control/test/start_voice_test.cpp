#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>

#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t ms)
{return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(ms);}
void feedback(VoiceSceneState & s, float speed)
{
  s.speed_state_received = true;
  s.veh_speed_st = speed;
  ++s.speed_feedback_seq; // 模拟订阅回调，每条反馈都递增，包括相同值和非法值。
}
VoiceSceneState stationary()
{
  VoiceSceneState s;
  s.veh_mode_st = 1; s.auto_drive_enabled = true;
  s.running_state_received = true; s.vehicle_running = false;
  feedback(s, 1.0F);
  return s;
}

void test_initial_feedback_and_repeated_starts()
{
  VoiceScheduler scheduler;
  auto s = stationary(); s.running_state_received = false;
  assert(!scheduler.update(s, at(0)));
  s.running_state_received = true; s.vehicle_running = true;
  assert(!scheduler.update(s, at(1000)));
  feedback(s, 1.0F); assert(!scheduler.update(s, at(1001))); // 首次true不构造事件。
  for (int t : {3000, 10000, 20000}) {
    s.vehicle_running = false; assert(!scheduler.update(s, at(t)));
    s.vehicle_running = true; assert(!scheduler.update(s, at(t + 1)));
    feedback(s, 1.0F); assert(scheduler.update(s, at(t + 2)) == 6);
    feedback(s, 1.0F); assert(!scheduler.update(s, at(t + 3)));
    assert(!scheduler.update(s, at(t + 3001)));
  }
}

void test_old_speed_and_fresh_boundaries()
{
  for (float old : {0.0F, 1.0F, 8.0F}) {
    for (int received : {100000, 102999, 103000}) {
      VoiceScheduler scheduler;
      auto s = stationary(); feedback(s, old);
      assert(!scheduler.update(s, at(0)));
      s.vehicle_running = true;
      assert(!scheduler.update(s, at(100000))); // 无论旧低速或旧高速都不能决定本次起步。
      feedback(s, 1.0F);
      const auto result = scheduler.update(s, at(received));
      if (received < 103000) {assert(result == 6);} else {assert(!result);}
      assert(!scheduler.update(s, at(104000)));
    }
  }
  VoiceScheduler no_feedback;
  auto s = stationary(); assert(!no_feedback.update(s, at(0)));
  s.vehicle_running = true; assert(!no_feedback.update(s, at(1000)));
  assert(!no_feedback.update(s, at(3999)));
  assert(!no_feedback.update(s, at(4000)));
  feedback(s, 1.0F); assert(!no_feedback.update(s, at(4001)));
  for (float speed : {0.0F, 4.999F, 5.0F, 6.0F}) {
    VoiceScheduler scheduler;
    s = stationary(); assert(!scheduler.update(s, at(0)));
    s.vehicle_running = true; assert(!scheduler.update(s, at(1000)));
    feedback(s, speed);
    const auto result = scheduler.update(s, at(1100));
    if (speed < 5.0F) {assert(result == 6);} else {assert(!result);}
    feedback(s, 1.0F); assert(!scheduler.update(s, at(1200))); // 已发/高速取消不重试。
  }
}

void test_invalid_new_speed_and_missing_feedback()
{
  const float inf = std::numeric_limits<float>::infinity();
  for (float invalid : {-1.0F, inf, -inf, std::numeric_limits<float>::quiet_NaN()}) {
    for (int repaired : {3999, 4000}) {
      VoiceScheduler scheduler;
      auto s = stationary(); s.speed_state_received = false;
      assert(!scheduler.update(s, at(0)));
      s.vehicle_running = true; assert(!scheduler.update(s, at(1000)));
      feedback(s, invalid); assert(!scheduler.update(s, at(2000)));
      feedback(s, 0.0F);
      const auto result = scheduler.update(s, at(repaired));
      if (repaired < 4000) {assert(result == 6);} else {assert(!result);}
    }
  }
  // 一次更新中边沿与车速已同时变化，也不把该缓存当成边沿之后的新消息。
  VoiceScheduler simultaneous;
  auto s = stationary(); assert(!simultaneous.update(s, at(0)));
  s.vehicle_running = true; feedback(s, 1.0F);
  assert(!simultaneous.update(s, at(1000)));
  feedback(s, 1.0F); assert(simultaneous.update(s, at(1100)) == 6);
  // 无需序号递增到更大，只需不同：无符号计数回绕仍识别下一条新消息。
  VoiceScheduler wrap;
  s = stationary(); s.speed_feedback_seq = std::numeric_limits<std::uint64_t>::max();
  assert(!wrap.update(s, at(0)));
  s.vehicle_running = true; assert(!wrap.update(s, at(1000)));
  feedback(s, 1.0F); assert(wrap.update(s, at(1001)) == 6);
}

void test_p0_blocking_and_original_deadline()
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
      feedback(s, 1.0F); assert(!scheduler.update(s, at(2000)));
      feedback(s, 1.0F); assert(!scheduler.update(s, at(3998))); // 不延长边沿期限。
      s.alarm_diag_st = 0; s.veh_gear_st = 0; s.obs_detect_st = false; s.turn_state = 0;
      const auto result = scheduler.update(s, at(clear));
      if (clear < 4000) {assert(result == 6);} else {assert(!result);}
      assert(!scheduler.update(s, at(5000)));
    }
  }
}

void test_cancellation_and_freshness_per_start()
{
  for (int reason = 0; reason < 5; ++reason) {
    VoiceScheduler scheduler;
    auto s = stationary(); assert(!scheduler.update(s, at(0)));
    s.alarm_diag_st = 1; s.vehicle_running = true;
    assert(scheduler.update(s, at(1000)) == 15);
    feedback(s, 1.0F); assert(!scheduler.update(s, at(1100)));
    if (reason == 0) {s.vehicle_running = false;}
    if (reason == 1) {s.auto_drive_enabled = false;}
    if (reason == 2) {s.veh_mode_st = 99;}
    if (reason == 3) {feedback(s, 5.0F);}
    if (reason == 4) {feedback(s, 8.0F);}
    const auto canceled = scheduler.update(s, at(2000));
    if (reason == 0) {assert(canceled == 16);} else {assert(!canceled);}
    s.alarm_diag_st = 0; assert(!scheduler.update(s, at(2100)));
    s.auto_drive_enabled = true; s.veh_mode_st = 1; feedback(s, 1.0F);
    assert(!scheduler.update(s, at(2200))); // 新低速也不能恢复已取消事件。
    if (s.vehicle_running) {s.vehicle_running = false; assert(!scheduler.update(s, at(2300)));}
    s.vehicle_running = true; assert(!scheduler.update(s, at(2400)));
    feedback(s, 1.0F); assert(scheduler.update(s, at(2500)) == 6);
  }
  VoiceScheduler scheduler;
  auto s = stationary(); assert(!scheduler.update(s, at(0)));
  s.vehicle_running = true; s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(1000)) == 15);
  feedback(s, 1.0F); assert(!scheduler.update(s, at(1100)));
  feedback(s, -1.0F); assert(!scheduler.update(s, at(2000)));
  s.alarm_diag_st = 0; assert(!scheduler.update(s, at(3000)));
  feedback(s, 1.0F); assert(scheduler.update(s, at(3999)) == 6);
}

void test_priority_and_waiting_does_not_block()
{
  VoiceScheduler scheduler;
  auto s = stationary(); s.trunk_lock_state_received = true; s.trunk_lock_st = true;
  assert(!scheduler.update(s, at(0)));
  s.trunk_lock_st = false; assert(scheduler.update(s, at(0)) == 5);
  // 先记录起步并用P0阻塞，让1/6/2/7同时就绪再验证原优先级。
  s.vehicle_running = true; s.veh_gear_st = 3; s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(59000)) == 15);
  feedback(s, 1.0F); assert(!scheduler.update(s, at(59500)));
  s.alarm_diag_st = 0;
  assert(scheduler.update(s, at(60000)) == 1);
  assert(scheduler.update(s, at(60000)) == 6);
  assert(scheduler.update(s, at(60000)) == 2);
  assert(scheduler.update(s, at(60000)) == 7);
  assert(!scheduler.update(s, at(60000)));
  s.vehicle_running = false; assert(!scheduler.update(s, at(69000)));
  s.vehicle_running = true; s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(70000)) == 15);
  feedback(s, 1.0F); assert(!scheduler.update(s, at(70100)));
  s.alarm_diag_st = 0;
  assert(scheduler.update(s, at(72999)) == 1);
  assert(scheduler.update(s, at(73000)) == 7); // ID1不延长ID6的3秒期限。
  VoiceScheduler forward;
  s = stationary(); s.veh_gear_st = 3;
  assert(!forward.update(s, at(0)));
  s.vehicle_running = true; assert(forward.update(s, at(1000)) == 7);
  feedback(s, 1.0F); assert(forward.update(s, at(2000)) == 6);
  VoiceScheduler cargo;
  s = stationary(); assert(!cargo.update(s, at(0)));
  s.vehicle_running = true; s.veh_arrive_st = true; s.veh_gear_st = 2; s.veh_epb_st = true;
  assert(cargo.update(s, at(1000)) == 3);
  assert(cargo.update(s, at(1050)) == 4);
  feedback(s, 1.0F); assert(cargo.update(s, at(3999)) == 6);
  VoiceScheduler low;
  s = stationary(); s.battery_low_power_st = true;
  assert(low.update(s, at(0)) == 17);
  s.vehicle_running = true; assert(!low.update(s, at(1000)));
  feedback(s, 1.0F); assert(low.update(s, at(1100)) == 6);
  assert(!low.update(s, at(10000)));
}

int main()
{
  test_initial_feedback_and_repeated_starts(); test_old_speed_and_fresh_boundaries();
  test_invalid_new_speed_and_missing_feedback(); test_p0_blocking_and_original_deadline();
  test_cancellation_and_freshness_per_start(); test_priority_and_waiting_does_not_block();
  std::cout << "Start voice checks passed\n";
}
