#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "voice_scene.hpp"

inline bool forward_voice_enabled(const VoiceSceneState & state)
{
  return state.veh_mode_st != 2 && state.auto_drive_enabled && state.running_state_received &&
         state.vehicle_running && state.veh_gear_st == 3;
}

// 全部成员均为内部调度状态，不新增业务信号或 ROS 接口。
// 由调用方传入单调时钟时间；一次检查最多产生一条播报请求。
class VoiceScheduler
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  std::optional<std::int32_t> update(const VoiceSceneState & state, TimePoint now)
  {
    const auto p0_id = select_voice_id(state);
    if (p0_id != active_p0_id_) {
      active_p0_id_ = p0_id;
      next_p0_due_ = p0_id != 0 ? std::optional<TimePoint>(now) : std::nullopt;
    }
    //自动驾驶前进场景的超时处理
    const bool forward_enabled = forward_voice_enabled(state);
    if (!forward_enabled) {
      next_forward_due_.reset();
      pending_forward_due_.reset();
    } else if (!forward_active_) {
      next_forward_due_ = now;  // 新进入前进场景，立即开始第一轮。
    }
    forward_active_ = forward_enabled;

    if (next_forward_due_ && now >= *next_forward_due_) {
      // 跳过历史轮次，仅保留最近一轮；周期以原到期时间为基准。
      const auto skipped = (now - *next_forward_due_) / kForwardPeriod;
      pending_forward_due_ = *next_forward_due_ + kForwardPeriod * skipped;
      next_forward_due_ = *pending_forward_due_ + kForwardPeriod;
    }
    if (pending_forward_due_ && now - *pending_forward_due_ >= kForwardTimeout) {
      pending_forward_due_.reset();  // 达到 10 秒即丢弃，不能在边界补播。
    }

    //远程、遥控驾驶的超时处理
    const bool remote_mode = state.veh_mode_st == 2;
    const bool remote_moving = remote_mode && state.running_state_received && state.vehicle_running;
    if (!remote_mode) {
      pending_remote_entry_due_.reset();
      next_remote_due_.reset();
      pending_remote_cycle_due_.reset();
    } else {
      if (!remote_active_) {
        pending_remote_entry_due_ = now; // 首次收到或重新切入远程/遥控模式。
      }
      if (!remote_moving) {
        // 停车仅取消行驶循环；未超时的切入提示仍然有效。
        next_remote_due_.reset();
        pending_remote_cycle_due_.reset();
      } else if (!remote_moving_) {
        next_remote_due_ = now; // 每次开始行驶立即进入新一轮。
      }
      if (next_remote_due_ && now >= *next_remote_due_) {
        const auto skipped = (now - *next_remote_due_) / kRemotePeriod;
        pending_remote_cycle_due_ = *next_remote_due_ + kRemotePeriod * skipped;
        next_remote_due_ = *pending_remote_cycle_due_ + kRemotePeriod;
      }
    }
    remote_active_ = remote_mode;
    remote_moving_ = remote_moving;
    // 两种请求分别计算 10 秒期限，不因另一个请求产生而延长期限。
    if (pending_remote_entry_due_ && now - *pending_remote_entry_due_ >= kRemoteTimeout) {
      pending_remote_entry_due_.reset();
    }
    if (pending_remote_cycle_due_ && now - *pending_remote_cycle_due_ >= kRemoteTimeout) {
      pending_remote_cycle_due_.reset();
    }

    // 场景有效即视为 P0 占用，不在其 5 秒请求间隙穿插 P1。
    if (active_p0_id_ != 0) {
      if (now >= *next_p0_due_) {
        const auto skipped = (now - *next_p0_due_) / kP0Period;
        *next_p0_due_ += kP0Period * (skipped + 1);
        return active_p0_id_;
      }
      return std::nullopt;
    }
    if (pending_remote_entry_due_ || pending_remote_cycle_due_) {
      // 相同 ID 的切入和行驶首轮合并发送，不重复请求播放。
      pending_remote_entry_due_.reset();
      pending_remote_cycle_due_.reset();
      return 14;
    }
    if (pending_forward_due_) {
      pending_forward_due_.reset();
      return 7;
    }
    return std::nullopt;
  }

private:
  static constexpr std::chrono::seconds kP0Period{5};
  static constexpr std::chrono::seconds kForwardPeriod{120};
  static constexpr std::chrono::seconds kForwardTimeout{10};
  static constexpr std::chrono::seconds kRemotePeriod{60};
  static constexpr std::chrono::seconds kRemoteTimeout{10};

  std::int32_t active_p0_id_{0};
  bool forward_active_{false};
  std::optional<TimePoint> next_p0_due_;
  std::optional<TimePoint> next_forward_due_;
  std::optional<TimePoint> pending_forward_due_;
  bool remote_active_{false};
  bool remote_moving_{false};
  std::optional<TimePoint> pending_remote_entry_due_;
  std::optional<TimePoint> next_remote_due_;
  std::optional<TimePoint> pending_remote_cycle_due_;
};
