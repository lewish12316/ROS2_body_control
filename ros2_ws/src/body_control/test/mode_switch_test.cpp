#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include "auto_drive_test_support.hpp"

using Gate = AutoDriveGate;

void no_request(Gate & g) {assert(!g.take_request().has_value());}
void request(Gate & g, bool expected)
{
  const auto actual = g.take_request();
  assert(actual.has_value() && *actual == expected);
  no_request(g);
}

void test_unknown_and_arrival_orders()
{
  // 所有七项反馈的到达顺序均须最后一项确认后才申请。
  std::array<int, 7> order{0, 1, 2, 3, 4, 5, 6};
  do {
    Gate g;
    g.set_start_command(true);
    no_request(g);
    for (int i = 0; i < 7; ++i) {
      switch (order[i]) {
        case 0: g.set_mode(0); break;
        case 1: g.set_self_check(true); break;
        case 2: g.set_emergency_stop(false); break;
        case 3: g.set_alarm(0); break;
        case 4: g.set_ads_abnormal(false); break;
        case 5: g.set_running(false); break;
        case 6: g.set_obstacle(false); break;
      }
      if (i != 6) {no_request(g);} else {request(g, true);}
      assert(!g.enabled());
    }
  } while (std::next_permutation(order.begin(), order.end()));
}

void test_entry_conditions_and_pending_safety()
{
  for (int reason = 0; reason < 8; ++reason) {
    Gate g;
    confirm_safe_feedback(g);
    switch (reason) {
      case 0: g.set_self_check(false); break;
      case 1: g.set_emergency_stop(true); break;
      case 2: g.set_alarm(1); break;
      case 3: g.set_alarm(-1); break;
      case 4: g.set_ads_abnormal(true); break;
      case 5: g.set_running(true); break;
      case 6: g.set_obstacle(true); break;
      case 7: g.set_mode(99); request(g, false); break;
    }
    if (reason != 7) {g.set_mode(0);}
    g.set_start_command(true);
    no_request(g);
    assert(!g.enabled());
  }
  for (bool motion : {false, true}) {
    Gate g;
    confirm_safe_feedback(g); g.set_mode(0); request(g, true);
    if (motion) {g.set_running(true);} else {g.set_obstacle(true);}
    request(g, false);
    g.set_running(false); g.set_obstacle(false); no_request(g);
    g.set_start_command(true); request(g, true);
  }
}

void test_request_feedback_and_start()
{
  Gate stopping_external;
  stopping_external.set_mode(1);
  stopping_external.set_start_command(false);
  request(stopping_external, false); // 即使本地没申请过，也不能吞掉明确的停止命令。
  stopping_external.set_start_command(false);
  no_request(stopping_external);
  Gate g;
  confirm_safe_feedback(g); g.set_mode(0); request(g, true);
  for (int i = 0; i < 20; ++i) {
    g.set_mode(0); confirm_safe_feedback(g); g.set_start_command(true);
    no_request(g); assert(!g.enabled()); // 申请不等于成功，也不重发。
  }
  g.set_mode(1); no_request(g); assert(!g.enabled());
  g.set_start_command(true); no_request(g); assert(g.enabled());
  g.set_mode(1); g.set_start_command(true); no_request(g); assert(g.enabled());
  g.set_running(true); g.set_obstacle(true); no_request(g); assert(g.enabled());
  g.set_start_command(false); request(g, false); assert(!g.enabled());
  g.set_start_command(false); no_request(g);
  g.set_mode(0); g.set_running(false); g.set_obstacle(false);
  confirm_safe_feedback(g); no_request(g); // 停止后不因安全反馈重复而重新申请。
  g.set_start_command(true); request(g, true); assert(!g.enabled());
  g.set_start_command(false); request(g, false);
  g.set_self_check(false); g.set_self_check(true); request(g, true);
  g.set_mode(1); assert(!g.enabled());
  g.set_start_command(true); assert(g.enabled());
  g.set_mode(0); request(g, false); assert(!g.enabled());
  g.set_mode(0); confirm_safe_feedback(g); no_request(g);
}

void test_health_revocation()
{
  for (bool started : {false, true}) {
    for (int reason = 0; reason < 5; ++reason) {
      Gate g;
      confirm_safe_feedback(g); g.set_mode(0); request(g, true);
      if (started) {g.set_mode(1); g.set_start_command(true); assert(g.enabled());}
      switch (reason) {
        case 0: g.set_self_check(false); break;
        case 1: g.set_emergency_stop(true); break;
        case 2: g.set_alarm(1); break;
        case 3: g.set_alarm(-1); break;
        case 4: g.set_ads_abnormal(true); break;
      }
      request(g, false); assert(!g.enabled());
      g.set_start_command(true); no_request(g); assert(!g.enabled());
      // 在自动模式中安全恢复不恢复授权，新的启动指令才恢复。
      if (started) {
        confirm_safe_feedback(g); no_request(g); assert(!g.enabled());
        g.set_start_command(true); request(g, true); assert(g.enabled());
      }
    }
  }
}

void test_remote_and_invalid_feedback()
{
  Gate boot_remote;
  boot_remote.set_mode(2);
  boot_remote.set_mode(99);
  boot_remote.set_start_command(false);
  confirm_safe_feedback(boot_remote);
  no_request(boot_remote); // 启动即遥控、尚未发布过请求，也不能漏出false。
  Gate g;
  confirm_safe_feedback(g); g.set_mode(0); request(g, true);
  g.set_mode(1); g.set_start_command(true); assert(g.enabled());
  g.set_mode(2); no_request(g); assert(!g.enabled());
  for (int mode : {2, 1, 99, -1, 1}) {
    g.set_mode(mode); g.set_start_command(false); g.set_start_command(true);
    g.set_self_check(false); confirm_safe_feedback(g);
    no_request(g); assert(!g.enabled());
  }
  g.set_mode(0); request(g, true);
  g.set_mode(1); assert(!g.enabled()); g.set_start_command(true); assert(g.enabled());
  g.set_mode(99); request(g, false); assert(!g.enabled());
  g.set_start_command(true); no_request(g);
  // 未经本节点申请的有效自动模式反馈仍是实际状态，但必须补齐安全反馈并重新启动。
  Gate external;
  external.set_mode(1); external.set_start_command(true); assert(!external.enabled());
  confirm_safe_feedback(external); no_request(external); assert(!external.enabled());
  external.set_start_command(true); request(external, true); assert(external.enabled());
}

int main()
{
  test_unknown_and_arrival_orders();
  test_entry_conditions_and_pending_safety();
  test_request_feedback_and_start();
  test_health_revocation();
  test_remote_and_invalid_feedback();
  std::cout << "Mode switch checks passed\n";
}
