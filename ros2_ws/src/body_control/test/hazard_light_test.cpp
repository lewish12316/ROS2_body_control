#include <cassert>
#include <iostream>

#include "../src/hazard_light.hpp"

auto at(int ms)
{
  return HazardLightControl::TimePoint{} + std::chrono::milliseconds(ms);
}

void test_trigger_combinations()
{
  // All 64 subsets of the six independent causes. Clear each cause in turn;
  // the lamp must remain active until the last cause has been removed.
  for (unsigned mask = 0; mask < 64; ++mask) {
    HazardLightControl control;
    VoiceSceneState s;
    s.running_state_received = true;
    s.vehicle_running = false;
    bool emergency = mask & 1U;
    bool abnormal = mask & 2U;
    s.alarm_diag_st = (mask & 4U) ? 1 : 0;
    s.obs_detect_st = mask & 8U;
    s.veh_mode_st = (mask & 16U) ? 2 : 0;
    s.battery_low_power_st = mask & 32U;
    assert(control.update(s, emergency, abnormal, at(0)) == ((mask & ~8U) != 0));
    assert(control.update(s, emergency, abnormal, at(30000)) == (mask != 0));
    unsigned remaining = mask;
    for (unsigned bit = 1; bit <= 32; bit <<= 1) {
      remaining &= ~bit;
      if (bit == 1) {emergency = false;}
      if (bit == 2) {abnormal = false;}
      if (bit == 4) {s.alarm_diag_st = 0;}
      if (bit == 8) {s.obs_detect_st = false;}
      if (bit == 16) {s.veh_mode_st = 0;}
      if (bit == 32) {s.battery_low_power_st = false;}
      assert(control.update(s, emergency, abnormal, at(30001)) == (remaining != 0));
    }
  }
}

void test_obstacle_timing()
{
  HazardLightControl control;
  VoiceSceneState s;
  s.obs_detect_st = true;
  assert(!control.update(s, false, false, at(0))); // Unknown running state.
  s.running_state_received = true;
  s.vehicle_running = false;
  assert(!control.update(s, false, false, at(1000)));
  assert(!control.update(s, false, false, at(30999))); // 29999ms elapsed.
  assert(control.update(s, false, false, at(31000))); // Exactly 30000ms.
  assert(control.update(s, false, false, at(100000)));
  s.obs_detect_st = false;
  assert(!control.update(s, false, false, at(100001)));
  s.obs_detect_st = true;
  assert(!control.update(s, false, false, at(100002)));
  s.vehicle_running = true;
  assert(!control.update(s, false, false, at(120000)));
  s.vehicle_running = false;
  assert(!control.update(s, false, false, at(121000)));
  assert(!control.update(s, false, false, at(150999)));
  assert(control.update(s, false, false, at(151000)));

  HazardLightControl late;
  assert(!late.update(s, false, false, at(0)));
  assert(late.update(s, false, false, at(90000))); // Late timer check.

  // Another cause must not mask obstacle timer reset or restart.
  s.vehicle_running = true;
  assert(late.update(s, true, false, at(90001)));
  s.vehicle_running = false;
  assert(late.update(s, true, false, at(90002)));
  assert(!late.update(s, false, false, at(120001)));
  assert(late.update(s, false, false, at(120002)));
}

void test_modes_and_unknown_state()
{
  for (int mode : {0, 1, 2, -1, 99}) {
    for (bool authorized : {false, true}) {
      HazardLightControl control;
      VoiceSceneState s;
      s.veh_mode_st = mode;
      s.auto_drive_enabled = authorized;
      s.alarm_diag_st = 1;
      s.battery_low_power_st = true;
      s.obs_detect_st = true;
      s.vehicle_running = false;
      assert(control.update(s, false, false, at(0)) == (mode == 2));
      assert(control.update(s, false, false, at(60000)) == (mode == 2));
      assert(control.update(s, true, false, at(60001)));
      assert(control.update(s, false, true, at(60002)));
      s.running_state_received = true;
      assert(control.update(s, false, false, at(60003)));
      s.vehicle_running = true;
      assert(control.update(s, false, false, at(60004)) == (mode == 2));
      s.battery_low_power_st = false;
      s.obs_detect_st = false;
      s.vehicle_running = false;
      for (int fault : {0, -1, -99}) {
        s.alarm_diag_st = fault;
        assert(control.update(s, false, false, at(60005)) == (mode == 2));
      }
    }
  }
}

int main()
{
  test_trigger_combinations();
  test_obstacle_timing();
  test_modes_and_unknown_state();
  std::cout << "Hazard light control checks passed\n";
}
