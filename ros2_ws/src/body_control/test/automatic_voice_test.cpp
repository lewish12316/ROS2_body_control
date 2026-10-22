#include <cassert>
#include <chrono>
#include <iostream>
#include "../src/voice_scheduler.hpp"

auto at(int ms)
{return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(ms);}
VoiceSceneState state_for(int mode, bool authorized)
{
  VoiceSceneState s;
  s.veh_mode_st = mode; s.auto_drive_enabled = authorized;
  s.running_state_received = true; s.vehicle_running = false;
  return s;
}
void test_mode_authorization_matrix()
{
  for (int mode : {0, 1, 2, -1, 99}) {
    for (bool authorized : {false, true}) {
      const bool allowed = mode == 1 && authorized;
      auto s = state_for(mode, authorized);
      assert(automatic_voice_enabled(s) == allowed);
      for (int id : {12, 8, 11, 10, 9}) {
        auto scene = s;
        if (id == 12) {scene.veh_gear_st = 1;}
        if (id == 8) {scene.obs_detect_st = true;}
        if (id == 11) {scene.pedestrian_crossing = true;}
        if (id == 10) {scene.turn_state = 1;}
        if (id == 9) {scene.turn_state = 2;}
        assert(select_voice_id(scene) == (allowed ? id : 0));
      }
      s.vehicle_running = true; s.veh_gear_st = 3;
      assert(forward_voice_enabled(s) == allowed);
      s.vehicle_running = false;
      DoorOpenVoiceTask door;
      s.trunk_lock_state_received = true; s.trunk_lock_st = false;
      door.observe(s, at(0)); door.observe(s, at(10000));
      assert(door.take_due(at(10000)) == (allowed ? std::optional<std::int32_t>(1) : std::nullopt));
      StartVoiceTask start;
      start.observe(s, at(0)); s.vehicle_running = true; start.observe(s, at(1000));
      s.speed_state_received = true; s.veh_speed_st = 1.0F; ++s.speed_feedback_seq;
      start.observe(s, at(1100));
      assert(start.take_due(at(1100)) == (allowed ? std::optional<std::int32_t>(6) : std::nullopt));
      s.vehicle_running = false;
      TrafficWaitVoiceTask traffic;
      s.traffic_light_wait_st = true; s.ambient_light_received = true; s.ambient_light_level_st = 6.0;
      traffic.observe(s, at(0));
      assert(traffic.take_due(at(0)) == (allowed ? std::optional<std::int32_t>(13) : std::nullopt));
      LowBatteryVoiceTask low;
      s.battery_low_power_st = true; low.observe(s, at(0));
      assert(low.take_due(at(0)) == (allowed ? std::optional<std::int32_t>(17) : std::nullopt));
    }
  }
}
void test_independent_voices_in_all_modes()
{
  for (int mode : {0, 1, 2, -1, 99}) {
    for (bool authorized : {false, true}) {
      auto s = state_for(mode, authorized);
      s.alarm_diag_st = 1;
      VoiceScheduler fault;
      assert(fault.update(s, at(0)) == 16);
      s.vehicle_running = true; assert(fault.update(s, at(1000)) == 15);
      assert(fault.update(s, at(6000)) == 15);
      s = state_for(mode, authorized);
      VoiceScheduler cargo;
      // 首次遥控模式提示优先，随后到站任务不依赖自动准入。
      const auto entry = cargo.update(s, at(0));
      if (mode == 2) {assert(entry == 14);} else {assert(!entry);}
      s.veh_arrive_st = true; s.veh_gear_st = 2; s.veh_epb_st = true;
      assert(cargo.update(s, at(1000)) == 3);
      assert(cargo.update(s, at(1000)) == 4);
      // 不注入锁状态给其他自动任务，独立验证同一事件仍建立5/2。
      UnlockVoiceTask unlock;
      CargoCompletionVoiceTask completion;
      s.trunk_lock_state_received = true; s.trunk_lock_st = true;
      bool event = unlock.observe(s, at(0)); completion.observe(s, at(0), event);
      assert(!event);
      s.trunk_lock_st = false;
      event = unlock.observe(s, at(1000)); completion.observe(s, at(1000), event);
      assert(event); assert(unlock.take_due(at(1000)) == 5);
      completion.observe(s, at(61000), false); assert(completion.take_due(at(61000)) == 2);
      completion.observe(s, at(121000), false); assert(completion.take_due(at(121000)) == 2);
    }
  }
}
void test_mode_loss_cancels_pending_tasks_and_reentry()
{
  for (bool lose_mode : {false, true}) {
    auto s = state_for(1, true);
    s.trunk_lock_state_received = true; s.trunk_lock_st = false;
    s.battery_low_power_st = true; s.traffic_light_wait_st = true;
    s.ambient_light_received = true; s.ambient_light_level_st = 6.0;
    DoorOpenVoiceTask door; TrafficWaitVoiceTask traffic; LowBatteryVoiceTask low;
    door.observe(s, at(0)); traffic.observe(s, at(0)); low.observe(s, at(0));
    door.observe(s, at(10000)); // 让 ID1 已待播，其他任务仍未取出。
    if (lose_mode) {s.veh_mode_st = 99;} else {s.auto_drive_enabled = false;}
    door.observe(s, at(10001)); traffic.observe(s, at(10001)); low.observe(s, at(10001));
    assert(!door.take_due(at(10001))); assert(!traffic.take_due(at(10001)));
    assert(!low.take_due(at(10001))); assert(!low.active());
    s.veh_mode_st = 1; s.auto_drive_enabled = true;
    door.observe(s, at(11000)); traffic.observe(s, at(11000)); low.observe(s, at(11000));
    assert(!door.take_due(at(11000)));
    assert(traffic.take_due(at(11000)) == 13); assert(low.take_due(at(11000)) == 17);
    door.observe(s, at(20999)); assert(!door.take_due(at(20999)));
    door.observe(s, at(21000)); assert(door.take_due(at(21000)) == 1);
    // 只丢失模式但授权仍为true，也必须清除前进待播，再次准入重新首轮。
    VoiceScheduler forward;
    s = state_for(1, true); s.vehicle_running = true; s.veh_gear_st = 3; s.obs_detect_st = true;
    assert(forward.update(s, at(0)) == 8);
    if (lose_mode) {s.veh_mode_st = 99;} else {s.auto_drive_enabled = false;}
    assert(!forward.update(s, at(1000)));
    s.obs_detect_st = false; assert(!forward.update(s, at(2000)));
    s.veh_mode_st = 1; s.auto_drive_enabled = true;
    assert(forward.update(s, at(3000)) == 7);
    assert(!forward.update(s, at(120000)));
    assert(forward.update(s, at(123000)) == 7);
  }
}
int main()
{
  test_mode_authorization_matrix(); test_independent_voices_in_all_modes();
  test_mode_loss_cancels_pending_tasks_and_reentry();
  std::cout << "Automatic voice checks passed\n";
}
