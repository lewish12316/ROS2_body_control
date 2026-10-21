#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>

#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t ms)
{return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(ms);}
VoiceSceneState waiting(double light = 6.0)
{
  VoiceSceneState s;
  s.veh_mode_st = 1; s.auto_drive_enabled = true;
  s.running_state_received = true; s.vehicle_running = false;
  s.traffic_light_wait_st = true;
  s.ambient_light_received = true; s.ambient_light_level_st = light;
  return s;
}

void test_condition_arrival_orders_and_scope()
{
  std::array<int, 5> order{0, 1, 2, 3, 4};
  do {
    VoiceScheduler scheduler;
    VoiceSceneState s;
    for (int i = 0; i < 5; ++i) {
      switch (order[i]) {
        case 0: s.veh_mode_st = 1; break;
        case 1: s.auto_drive_enabled = true; break;
        case 2: s.running_state_received = true; s.vehicle_running = false; break;
        case 3: s.traffic_light_wait_st = true; break;
        case 4: s.ambient_light_received = true; s.ambient_light_level_st = 6.0; break;
      }
      const auto id = scheduler.update(s, at(i * 100));
      if (i == 4) {assert(id == 13);} else {assert(!id);}
    }
  } while (std::next_permutation(order.begin(), order.end()));
  for (int reason = 0; reason < 6; ++reason) {
    TrafficWaitVoiceTask task;
    auto s = waiting();
    if (reason == 0) {s.veh_mode_st = 0;}
    if (reason == 1) {s.veh_mode_st = 2;}
    if (reason == 2) {s.auto_drive_enabled = false;}
    if (reason == 3) {s.running_state_received = false;}
    if (reason == 4) {s.vehicle_running = true;}
    if (reason == 5) {s.traffic_light_wait_st = false;}
    task.observe(s, at(0)); task.observe(s, at(10000));
    assert(!task.take_due(at(10000)));
  }
}

void test_light_boundary_invalid_and_retention()
{
  for (double value : {0.0, 4.999, 5.0, 5.001, 7.0}) {
    VoiceScheduler scheduler;
    auto s = waiting(value);
    assert(scheduler.update(s, at(0)) == 13);
    assert(!scheduler.update(s, at(9999)));
    const auto id = scheduler.update(s, at(10000));
    if (value > 5.0) {assert(id == 13);} else {assert(!id);}
  }
  const double inf = std::numeric_limits<double>::infinity();
  for (double invalid : {-1.0, inf, -inf, std::numeric_limits<double>::quiet_NaN()}) {
    VoiceScheduler scheduler;
    auto s = waiting(invalid);
    assert(!scheduler.update(s, at(0)));
    assert(!scheduler.update(s, at(10000)));
    s.ambient_light_level_st = 6.0;
    assert(scheduler.update(s, at(11000)) == 13);
    s.ambient_light_level_st = invalid;
    assert(!scheduler.update(s, at(20999)));
    assert(scheduler.update(s, at(21000)) == 13); // 保留白天，非法值不重置周期。
    s.ambient_light_level_st = 5.0;
    assert(!scheduler.update(s, at(22000))); // 本次已发过，进入夜间不再播。
    s.ambient_light_level_st = invalid;
    assert(!scheduler.update(s, at(31000))); // 保留夜间。
    s.traffic_light_wait_st = false; assert(!scheduler.update(s, at(32000)));
    s.traffic_light_wait_st = true;
    assert(scheduler.update(s, at(33000)) == 13); // 跨任务保留最后有效光照。
    assert(!scheduler.update(s, at(43000)));
  }
}

void test_day_period_duplicates_and_delayed_checks()
{
  VoiceScheduler scheduler;
  auto s = waiting();
  assert(scheduler.update(s, at(1000)) == 13);
  for (int t : {1000, 5000, 10999}) {assert(!scheduler.update(s, at(t)));}
  assert(scheduler.update(s, at(11000)) == 13);
  assert(!scheduler.update(s, at(20999)));
  assert(scheduler.update(s, at(21000)) == 13);
  for (int check : {12000, 13000, 42000, 43000}) {
    VoiceScheduler late;
    s = waiting(); assert(late.update(s, at(0)) == 13);
    const auto id = late.update(s, at(check));
    if (check % 10000 < 3000) {assert(id == 13);} else {assert(!id);}
    assert(!late.update(s, at(check))); // 不集中补发历史轮次。
    const int next = (check / 10000 + 1) * 10000;
    assert(late.update(s, at(next)) == 13);
  }
}

void test_p0_blocking_timeouts_and_night_once()
{
  for (double light : {5.0, 6.0}) {
    for (int p0 : {16, 12, 8, 11, 10, 9}) {
      for (int clear : {2999, 3000}) {
        VoiceScheduler scheduler;
        auto s = waiting(light);
        if (p0 == 16) {s.alarm_diag_st = 1;}
        if (p0 == 12) {s.veh_gear_st = 1;}
        if (p0 == 8) {s.obs_detect_st = true;}
        if (p0 == 11) {s.pedestrian_crossing = true;}
        if (p0 == 10) {s.turn_state = 1;}
        if (p0 == 9) {s.turn_state = 2;}
        assert(scheduler.update(s, at(0)) == p0);
        assert(!scheduler.update(s, at(1000)));
        s.alarm_diag_st = 0; s.veh_gear_st = 0; s.obs_detect_st = false;
        s.pedestrian_crossing = false; s.turn_state = 0;
        const auto id = scheduler.update(s, at(clear));
        if (clear < 3000) {assert(id == 13);} else {assert(!id);}
        assert(!scheduler.update(s, at(9999)));
        const auto next = scheduler.update(s, at(10000));
        if (light > 5.0) {assert(next == 13);} else {assert(!next);}
        assert(!scheduler.update(s, at(10001)));
      }
    }
  }
}

void test_phase_changes_and_original_deadline()
{
  for (int clear : {2999, 3000}) {
    VoiceScheduler scheduler;
    auto s = waiting(); s.alarm_diag_st = 1;
    assert(scheduler.update(s, at(0)) == 16);
    s.ambient_light_level_st = 5.0;
    assert(!scheduler.update(s, at(1000))); // 白天请求转夜间，仍沿用 t=0 的期限。
    s.alarm_diag_st = 0;
    const auto id = scheduler.update(s, at(clear));
    if (clear == 2999) {assert(id == 13);} else {assert(!id);}
    assert(!scheduler.update(s, at(10000)));
  }
  VoiceScheduler scheduler;
  auto s = waiting(5.0); s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(0)) == 16);
  assert(!scheduler.update(s, at(3000))); // 夜间首轮超时，机会已消费。
  s.ambient_light_level_st = 6.0;
  assert(!scheduler.update(s, at(4000))); // 白天新阶段首轮仍被P0阻塞。
  s.ambient_light_level_st = 5.0;
  assert(scheduler.update(s, at(5000)) == 16); // 再入夜不能重新使用夜间机会。
  s.alarm_diag_st = 0; assert(!scheduler.update(s, at(5100)));
  s.ambient_light_level_st = 6.0;
  assert(scheduler.update(s, at(6000)) == 13); // 恢复白天立即首轮。
  assert(!scheduler.update(s, at(15999)));
  assert(scheduler.update(s, at(16000)) == 13);
  s.ambient_light_level_st = 5.0; assert(!scheduler.update(s, at(17000)));
  assert(!scheduler.update(s, at(26000))); // 已发过时切入夜间不再提示。
  // 白天旧轮已经超时且未发过，切入夜间可建立唯一新请求。
  VoiceScheduler expired_day;
  s = waiting(); s.alarm_diag_st = 1;
  assert(expired_day.update(s, at(0)) == 16);
  assert(!expired_day.update(s, at(3000)));
  s.ambient_light_level_st = 5.0; assert(!expired_day.update(s, at(4000)));
  s.alarm_diag_st = 0; assert(expired_day.update(s, at(6999)) == 13);
}

void test_night_to_day_replaces_pending_and_cancel_rearm()
{
  VoiceScheduler scheduler;
  auto s = waiting(5.0); s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(0)) == 16);
  s.ambient_light_level_st = 6.0;
  assert(!scheduler.update(s, at(2000)));
  s.alarm_diag_st = 0;
  assert(scheduler.update(s, at(4999)) == 13); // 使用白天阶段的新期限，而非夜间旧请求。
  assert(scheduler.update(s, at(12000)) == 13);
  for (int reason = 0; reason < 4; ++reason) {
    VoiceScheduler task;
    s = waiting(5.0); s.alarm_diag_st = 1;
    assert(task.update(s, at(0)) == 16);
    if (reason == 0) {s.auto_drive_enabled = false;}
    if (reason == 1) {s.veh_mode_st = 0;}
    if (reason == 2) {s.traffic_light_wait_st = false;}
    if (reason == 3) {s.vehicle_running = true;}
    const auto blocked = task.update(s, at(1000));
    if (reason == 3) {assert(blocked == 15);} else {assert(!blocked);}
    s.alarm_diag_st = 0; assert(!task.update(s, at(2000)));
    s = waiting(5.0); assert(task.update(s, at(2100)) == 13);
    assert(!task.update(s, at(12100)));
  }
}

void test_low_battery_and_other_priorities()
{
  for (double light : {5.0, 6.0}) {
    VoiceScheduler scheduler;
    auto s = waiting(light); s.battery_low_power_st = true;
    assert(scheduler.update(s, at(0)) == 17);
    assert(!scheduler.update(s, at(2000)));
    assert(!scheduler.update(s, at(3000)));
    assert(scheduler.update(s, at(10000)) == 17);
    const auto id = scheduler.update(s, at(10050));
    if (light > 5.0) {assert(id == 13);} else {assert(!id);}
  }
  VoiceScheduler scheduler;
  auto s = waiting(); s.traffic_light_wait_st = false;
  s.trunk_lock_state_received = true; s.trunk_lock_st = true;
  assert(!scheduler.update(s, at(0)));
  s.trunk_lock_st = false; assert(scheduler.update(s, at(0)) == 5);
  s.traffic_light_wait_st = true;
  assert(scheduler.update(s, at(60000)) == 1);
  assert(scheduler.update(s, at(60000)) == 13);
  assert(scheduler.update(s, at(60000)) == 2);
  assert(!scheduler.update(s, at(60000)));
  s.veh_arrive_st = true; s.veh_gear_st = 2; s.veh_epb_st = true;
  assert(scheduler.update(s, at(70000)) == 1);
  assert(scheduler.update(s, at(70000)) == 13);
  assert(scheduler.update(s, at(70000)) == 3);
  assert(scheduler.update(s, at(70000)) == 4);
  // ID 1 不能延长 ID 13 的原期限；ID 2 也保持自己的10秒期限。
  s.alarm_diag_st = 1;
  assert(scheduler.update(s, at(120000)) == 16);
  s.alarm_diag_st = 0;
  assert(scheduler.update(s, at(122999)) == 1);
  assert(scheduler.update(s, at(123000)) == 2); // ID13恰好到3秒，不再发送。
  assert(!scheduler.update(s, at(123000)));
}

int main()
{
  test_condition_arrival_orders_and_scope(); test_light_boundary_invalid_and_retention();
  test_day_period_duplicates_and_delayed_checks(); test_p0_blocking_timeouts_and_night_once();
  test_phase_changes_and_original_deadline(); test_night_to_day_replaces_pending_and_cancel_rearm();
  test_low_battery_and_other_priorities();
  std::cout << "Traffic wait voice checks passed\n";
}
