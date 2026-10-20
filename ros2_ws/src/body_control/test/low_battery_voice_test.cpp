#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "../src/auto_drive.hpp"
#include "../src/voice_scheduler.hpp"

VoiceScheduler::TimePoint at(std::int64_t milliseconds)
{
  return VoiceScheduler::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

VoiceSceneState ready_state()
{
  VoiceSceneState state;
  state.veh_mode_st = 1;
  state.auto_drive_enabled = true;
  state.running_state_received = true;
  state.vehicle_running = false;
  state.battery_low_power_st = true;
  return state;
}

void test_two_requests_and_repeated_feedback()
{
  VoiceScheduler scheduler;
  auto state = ready_state();
  state.auto_drive_enabled = false;
  assert(!scheduler.update(state, at(0))); // 低电量和静止已先到，但未启动。
  state.auto_drive_enabled = true;
  assert(scheduler.update(state, at(1000)) == 17);
  for (const int t : {1000, 1050, 2000, 6000, 10999}) {
    assert(!scheduler.update(state, at(t))); // 重复消息不重置或额外发送。
  }
  assert(scheduler.update(state, at(11000)) == 17);
  for (const int t : {11000, 16000, 21000, 100000}) {
    assert(!scheduler.update(state, at(t))); // 两次后不使用 5 秒或 10 秒无限循环。
  }
  state.battery_low_power_st = false;
  assert(!scheduler.update(state, at(101000)));
  state.battery_low_power_st = true;
  assert(!scheduler.update(state, at(102000))); // 完成本轮后不再次触发。
}

void test_delayed_battery_and_stopped_feedback()
{
  {
    VoiceScheduler scheduler;
    auto state = ready_state();
    state.battery_low_power_st = false;
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(10000))); // 启动后无低电量，可继续等。
    state.battery_low_power_st = true;
    assert(scheduler.update(state, at(15000)) == 17);
    assert(!scheduler.update(state, at(24999)));
    assert(scheduler.update(state, at(25000)) == 17);
  }
  {
    VoiceScheduler scheduler;
    auto state = ready_state();
    state.running_state_received = false;
    state.vehicle_running = true; // 默认值不是已收到的行驶状态。
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(10000)));
    state.running_state_received = true;
    state.vehicle_running = false;
    assert(scheduler.update(state, at(11000)) == 17);
    assert(scheduler.update(state, at(21000)) == 17);
  }
  {
    VoiceScheduler scheduler;
    auto state = ready_state();
    state.running_state_received = false;
    assert(!scheduler.update(state, at(0)));
    state.battery_low_power_st = false;
    assert(!scheduler.update(state, at(1000))); // 任务未开始，低电量解除不消费窗口。
    state.running_state_received = true;
    assert(!scheduler.update(state, at(2000)));
    state.battery_low_power_st = true;
    assert(scheduler.update(state, at(3000)) == 17);
  }
}

void test_both_feedbacks_missing_at_start()
{
  for (const bool battery_first : {false, true}) {
    VoiceScheduler scheduler;
    auto state = ready_state();
    state.running_state_received = false;
    state.vehicle_running = true; // 未收到消息，不作为已开始行驶处理。
    state.battery_low_power_st = false;
    assert(!scheduler.update(state, at(0)));
    assert(!scheduler.update(state, at(10000))); // 触发窗口没有 10 秒等待超时。
    if (battery_first) {
      state.battery_low_power_st = true;
    } else {
      state.running_state_received = true;
      state.vehicle_running = false;
    }
    assert(!scheduler.update(state, at(15000))); // 仅补齐一种反馈，不播报。
    state.battery_low_power_st = true;
    state.running_state_received = true;
    state.vehicle_running = false;
    assert(scheduler.update(state, at(25000)) == 17);
    assert(!scheduler.update(state, at(34999)));
    assert(scheduler.update(state, at(35000)) == 17); // 从条件补齐后的实际请求起算。
    assert(!scheduler.update(state, at(45000)));
  }

  VoiceScheduler scheduler;
  auto state = ready_state();
  state.running_state_received = false;
  state.battery_low_power_st = false;
  assert(!scheduler.update(state, at(0)));
  state.running_state_received = true;
  state.vehicle_running = true;
  assert(!scheduler.update(state, at(1000))); // 先确认已行驶，窗口应关闭。
  state.battery_low_power_st = true;
  assert(!scheduler.update(state, at(2000)));
  state.vehicle_running = false;
  assert(!scheduler.update(state, at(3000))); // 后续停车也不补触发。
}

void test_mode_and_start_authorization()
{
  for (const int mode : {0, 2, -1, 99}) {
    VoiceScheduler scheduler;
    AutoDriveGate gate;
    auto state = ready_state();
    state.veh_mode_st = mode;
    gate.set_mode(mode);
    gate.set_start_command(true);
    state.auto_drive_enabled = gate.enabled();
    const auto result = scheduler.update(state, at(0));
    if (mode == 2) {assert(result == 14);} else {assert(!result);}
    assert(!scheduler.update(state, at(10000)));
  }
  VoiceScheduler scheduler;
  AutoDriveGate gate;
  auto state = ready_state();
  gate.set_mode(1);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(0)));
  gate.set_start_command(true);
  state.auto_drive_enabled = gate.enabled();
  assert(scheduler.update(state, at(1000)) == 17);
  gate.set_start_command(true);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(2000)));
  assert(scheduler.update(state, at(11000)) == 17);
  gate.set_start_command(false);
  state.auto_drive_enabled = gate.enabled();
  assert(!scheduler.update(state, at(12000)));
  gate.set_start_command(true);
  state.auto_drive_enabled = gate.enabled();
  assert(scheduler.update(state, at(13000)) == 17); // 新一次有效启动允许新任务。
}

void test_cancellation_consumes_current_departure()
{
  // 在第一次发送前被高优先 P0 阻塞，或已发送第一次，均需处理取消。
  for (const bool blocked : {false, true}) {
    for (int reason = 0; reason != 4; ++reason) {
      VoiceScheduler scheduler;
      auto state = ready_state();
      if (blocked) {state.turn_state = 1;}
      assert(scheduler.update(state, at(0)) == (blocked ? 10 : 17));
      if (reason == 0) {state.vehicle_running = true;}
      if (reason == 1) {state.auto_drive_enabled = false;}
      if (reason == 2) {state.veh_mode_st = 0; state.auto_drive_enabled = false;}
      if (reason == 3) {state.battery_low_power_st = false;}
      scheduler.update(state, at(1000));
      state.turn_state = 0;
      scheduler.update(state, at(2000));
      state.vehicle_running = false;
      state.battery_low_power_st = true;
      assert(!scheduler.update(state, at(10000)));
      assert(!scheduler.update(state, at(20000)));
      // 全部原因都需要先撤销授权，再重新有效启动。
      state.auto_drive_enabled = false;
      assert(!scheduler.update(state, at(21000)));
      state.veh_mode_st = 1;
      state.auto_drive_enabled = true;
      assert(scheduler.update(state, at(22000)) == 17);
    }
  }
}

void test_moving_at_start_or_before_battery_arrives()
{
  for (const bool low_at_start : {false, true}) {
    VoiceScheduler scheduler;
    auto state = ready_state();
    state.battery_low_power_st = low_at_start;
    state.vehicle_running = true;
    assert(!scheduler.update(state, at(0))); // 挡位默认 P，不会产生 ID 7。
    state.battery_low_power_st = true;
    state.vehicle_running = false;
    assert(!scheduler.update(state, at(1000)));
    assert(!scheduler.update(state, at(10000))); // 随后停车不重新开启。
  }
  VoiceScheduler scheduler;
  auto state = ready_state();
  state.battery_low_power_st = false;
  assert(!scheduler.update(state, at(0)));
  state.vehicle_running = true;
  assert(!scheduler.update(state, at(1000)));
  state.vehicle_running = false;
  state.battery_low_power_st = true;
  assert(!scheduler.update(state, at(2000)));
}

void test_existing_p0_priority_and_delayed_first_request()
{
  for (const int id : {16, 12, 8, 11, 10, 9}) {
    VoiceScheduler scheduler;
    auto state = ready_state();
    if (id == 16) {state.alarm_diag_st = 1;}
    if (id == 12) {state.veh_gear_st = 1;}
    if (id == 8) {state.obs_detect_st = true;}
    if (id == 11) {state.pedestrian_crossing = true;}
    if (id == 10) {state.turn_state = 1;}
    if (id == 9) {state.turn_state = 2;}
    assert(scheduler.update(state, at(0)) == id);
    assert(!scheduler.update(state, at(1000))); // 不能利用 P0 的请求间隙播放。
    assert(scheduler.update(state, at(5000)) == id);
    assert(scheduler.update(state, at(20000)) == id); // 没有 10 秒待播丢弃规则。
    state.alarm_diag_st = 0;
    state.veh_gear_st = 0;
    state.obs_detect_st = false;
    state.pedestrian_crossing = false;
    state.turn_state = 0;
    assert(scheduler.update(state, at(25000)) == 17);
    assert(!scheduler.update(state, at(25000))); // 不集中补发两次。
    assert(!scheduler.update(state, at(34999)));
    assert(scheduler.update(state, at(35000)) == 17); // 从实际第一次请求起算。
    assert(!scheduler.update(state, at(45000)));
  }
}

void test_second_request_blocked_and_late_checks()
{
  VoiceScheduler scheduler;
  auto state = ready_state();
  assert(scheduler.update(state, at(0)) == 17);
  state.alarm_diag_st = 1;
  assert(scheduler.update(state, at(8000)) == 16);
  assert(!scheduler.update(state, at(10000)));
  assert(scheduler.update(state, at(13000)) == 16);
  state.alarm_diag_st = 0;
  assert(scheduler.update(state, at(15000)) == 17); // 第二次可延迟，但间隔至少 10 秒。
  assert(!scheduler.update(state, at(15000)));
  assert(!scheduler.update(state, at(25000)));

  VoiceScheduler late;
  state = ready_state();
  assert(late.update(state, at(0)) == 17);
  assert(late.update(state, at(60000)) == 17); // 迟到只发第二次，不产生第三次。
  assert(!late.update(state, at(60000)));
  assert(!late.update(state, at(70000)));
}

void test_normal_voice_after_cancellation_and_mode_switch()
{
  VoiceScheduler scheduler;
  auto state = ready_state();
  state.veh_gear_st = 3;
  assert(scheduler.update(state, at(0)) == 17);
  state.vehicle_running = true;
  assert(scheduler.update(state, at(1000)) == 7); // 不禁止起步；取消第二次低电量提示。
  assert(!scheduler.update(state, at(10000)));
  assert(scheduler.update(state, at(121000)) == 7);
  state.veh_mode_st = 2;
  state.auto_drive_enabled = false;
  assert(scheduler.update(state, at(122000)) == 14);
  assert(scheduler.update(state, at(182000)) == 14);
}

int main()
{
  test_two_requests_and_repeated_feedback();
  test_delayed_battery_and_stopped_feedback();
  test_both_feedbacks_missing_at_start();
  test_mode_and_start_authorization();
  test_cancellation_consumes_current_departure();
  test_moving_at_start_or_before_battery_arrives();
  test_existing_p0_priority_and_delayed_first_request();
  test_second_request_blocked_and_late_checks();
  test_normal_voice_after_cancellation_and_mode_switch();
  std::cout << "Low battery voice checks passed\n";
}
