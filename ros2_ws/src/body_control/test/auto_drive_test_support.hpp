#pragma once
#include "../src/auto_drive.hpp"

// 语音测试显式提供已确认的安全反馈，不再依赖模式控制器的默认值。
inline void confirm_safe_feedback(AutoDriveGate & gate)
{
  gate.set_self_check(true);
  gate.set_emergency_stop(false);
  gate.set_alarm(0);
  gate.set_ads_abnormal(false);
  gate.set_running(false);
  gate.set_obstacle(false);
}
