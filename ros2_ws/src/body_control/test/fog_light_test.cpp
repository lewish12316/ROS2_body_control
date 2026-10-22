#include <cassert>
#include <iostream>
#include <limits>

#include "../src/fog_light.hpp"

void test_mode_matrix()
{
  for (int mode : {0, 1, 2, -1, 99}) {
    for (bool power : {false, true}) {
      for (bool low : {false, true}) {
        for (bool dark : {false, true}) {
          for (bool command : {false, true}) {
            FogLightControl control;
            VoiceSceneState s;
            s.veh_mode_st = mode;
            s.ambient_light_received = true;
            s.ambient_light_level_st = dark ? 5.0 : 6.0;
            const bool expected = power &&
              ((mode == 1 && (low || dark)) || (mode == 2 && command));
            assert(control.update(s, power, low, command) == expected);
            s.auto_drive_enabled = true;
            s.running_state_received = true;
            s.vehicle_running = true;
            assert(control.update(s, power, low) == expected);
          }
        }
      }
    }
  }
}

void test_ambient_and_sources()
{
  FogLightControl control;
  VoiceSceneState s;
  s.veh_mode_st = 1;
  assert(!control.update(s, true, false)); // Unknown light is not dark.
  assert(control.update(s, true, true)); // Visibility independent of unknown light.
  assert(!control.update(s, true, false));
  s.ambient_light_received = true;
  const double invalid[] = {-1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (double value : invalid) {
    s.ambient_light_level_st = value;
    assert(!control.update(s, true, false));
  }
  for (double value : {0.0, 4.999, 5.0}) {
    s.ambient_light_level_st = value;
    assert(control.update(s, true, false));
  }
  for (double value : invalid) {
    s.ambient_light_level_st = value;
    assert(control.update(s, true, false)); // Preserve dark.
  }
  assert(control.update(s, true, true));
  assert(control.update(s, true, false)); // Clearing visibility preserves dark.
  s.ambient_light_level_st = 5.001;
  assert(control.update(s, true, true)); // Clearing dark preserves visibility.
  assert(!control.update(s, true, false));
  for (double value : invalid) {
    s.ambient_light_level_st = value;
    assert(!control.update(s, true, false)); // Preserve bright.
  }
  // Update light cache even while down; retain last valid value across power/modes.
  s.ambient_light_level_st = 5.0;
  assert(!control.update(s, false, false));
  s.ambient_light_level_st = -1.0;
  s.veh_mode_st = 0;
  assert(!control.update(s, true, false));
  s.veh_mode_st = 1;
  assert(control.update(s, true, false));
}

void test_command_lifetime()
{
  FogLightControl control;
  VoiceSceneState s;
  assert(!control.update(s, false, false, true)); // Ignore before power/mode.
  s.veh_mode_st = 2;
  assert(!control.update(s, false, false, true));
  assert(!control.update(s, true, true)); // Environment cannot auto-enable remote.
  assert(control.update(s, true, false, true));
  assert(control.update(s, true, false)); // Repeated power/mode doesn't clear.
  assert(control.update(s, true, true));
  assert(control.update(s, true, false)); // Visibility recovery doesn't clear.
  assert(!control.update(s, true, false, false));
  assert(!control.update(s, true, false));
  assert(control.update(s, true, false, true));
  assert(!control.update(s, false, false));
  assert(!control.update(s, false, false, true)); // Commands while down are dropped.
  assert(!control.update(s, true, false));
  assert(control.update(s, true, false, true));
  s.veh_mode_st = 1;
  assert(!control.update(s, true, false, true)); // Ignore automatic-mode command.
  s.veh_mode_st = 2;
  assert(!control.update(s, true, false)); // Old remote request was cleared.
  for (int mode : {0, -1, 99}) {
    assert(control.update(s, true, false, true));
    s.veh_mode_st = mode;
    assert(!control.update(s, true, false, true));
    s.veh_mode_st = 2;
    assert(!control.update(s, true, false));
  }
  s.veh_mode_st = 1;
  assert(control.update(s, true, true, false)); // Manual close cannot suppress auto.
  assert(control.update(s, true, true, true));
}

int main()
{
  test_mode_matrix();
  test_ambient_and_sources();
  test_command_lifetime();
  std::cout << "Fog light control checks passed\n";
}
