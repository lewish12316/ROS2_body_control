#include <cassert>
#include <iostream>
#include <limits>

#include "../src/aux_light.hpp"

auto at(int ms)
{
  return AuxLightControl::TimePoint{} + std::chrono::milliseconds(ms);
}

void test_turn_timing()
{
  for (int side : {1, 2}) {
    AuxLightControl control;
    VoiceSceneState s;
    assert(control.update(s, false, at(0)) == 0);
    s.turn_state = side;
    assert(control.update(s, false, at(1)) == side);
    assert(control.update(s, false, at(1000)) == side);
    s.turn_state = 0;
    assert(control.update(s, false, at(2000)) == side);
    assert(control.update(s, false, at(2999)) == side);
    assert(control.update(s, false, at(3000)) == 0);
    assert(control.update(s, false, at(6000)) == 0);
    s.turn_state = side;
    assert(control.update(s, false, at(6001)) == side);
    s.turn_state = 0;
    assert(control.update(s, false, at(6002)) == side);
    assert(control.update(s, false, at(10000)) == 0); // Late check.
    s.turn_state = side;
    assert(control.update(s, false, at(11000)) == side);
    s.turn_state = 3 - side;
    assert(control.update(s, false, at(11001)) == 3 - side); // No old-side hold.
    s.turn_state = 0;
    assert(control.update(s, false, at(11002)) == 3 - side);
    s.turn_state = side;
    assert(control.update(s, false, at(11500)) == side); // Replace pending side.
    s.turn_state = -1;
    assert(control.update(s, false, at(11501)) == side); // Invalid treated as straight.
    s.turn_state = side;
    assert(control.update(s, false, at(12000)) == side); // Same-side return cancels expiry.
    assert(control.update(s, false, at(14000)) == side);
    s.turn_state = 0;
    assert(control.update(s, false, at(14001)) == side);
    assert(control.update(s, false, at(15000)) == side);
    assert(control.update(s, false, at(15001)) == 0);
  }
}

void test_ambient_validation()
{
  AuxLightControl control;
  VoiceSceneState s;
  s.running_state_received = true;
  s.vehicle_running = true;
  assert(control.update(s, false, at(0)) == 0); // No light feedback.
  s.ambient_light_received = true;
  const double invalid[] = {-1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (double light : invalid) {
    s.ambient_light_level_st = light;
    assert(control.update(s, false, at(1)) == 0);
  }
  for (double light : {0.0, 4.999, 5.0}) {
    s.ambient_light_level_st = light;
    assert(control.update(s, false, at(2)) == 3);
  }
  for (double light : invalid) {
    s.ambient_light_level_st = light;
    assert(control.update(s, false, at(3)) == 3); // Preserve dark.
  }
  s.running_state_received = false;
  assert(control.update(s, false, at(4)) == 0);
  s.running_state_received = true;
  s.vehicle_running = false;
  assert(control.update(s, false, at(5)) == 0);
  s.vehicle_running = true;
  assert(control.update(s, false, at(6)) == 3);
  s.ambient_light_level_st = 5.001;
  assert(control.update(s, false, at(7)) == 0);
  for (double light : invalid) {
    s.ambient_light_level_st = light;
    assert(control.update(s, false, at(8)) == 0); // Preserve bright.
  }
}

void test_source_union_and_independence()
{
  for (int mode : {0, 1, 2, -1, 99}) {
    for (int side : {0, 1, 2}) {
      for (bool parking : {false, true}) {
        for (bool dark : {false, true}) {
          for (bool running : {false, true}) {
            AuxLightControl control;
            VoiceSceneState s;
            s.veh_mode_st = mode;
            s.auto_drive_enabled = false;
            s.alarm_diag_st = 1;
            s.turn_state = side;
            s.running_state_received = true;
            s.vehicle_running = running;
            s.ambient_light_received = true;
            s.ambient_light_level_st = dark ? 5.0 : 6.0;
            const int expected = parking || (dark && running) ? 3 : side;
            assert(control.update(s, parking, at(0)) == expected);
            s.auto_drive_enabled = true;
            assert(control.update(s, parking, at(10)) == expected);
          }
        }
      }
    }
  }
  AuxLightControl control;
  VoiceSceneState s;
  s.turn_state = 1;
  assert(control.update(s, true, at(0)) == 3);
  s.turn_state = 0;
  assert(control.update(s, true, at(1)) == 3);
  assert(control.update(s, false, at(500)) == 1); // Parking clear preserves turn hold.
  s.ambient_light_received = true;
  s.ambient_light_level_st = 0.0;
  s.running_state_received = true;
  s.vehicle_running = true;
  assert(control.update(s, true, at(1001)) == 3); // Turn expiry hidden by other sources.
  assert(control.update(s, false, at(1002)) == 3); // Dark running remains.
  s.vehicle_running = false;
  assert(control.update(s, false, at(1003)) == 0); // Expired turn must not resume.
  s.turn_state = 2;
  assert(control.update(s, true, at(1004)) == 3);
  assert(control.update(s, false, at(1005)) == 2);
}

int main()
{
  test_turn_timing();
  test_ambient_validation();
  test_source_union_and_independence();
  std::cout << "Aux light control checks passed\n";
}
