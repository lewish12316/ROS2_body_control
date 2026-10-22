#pragma once

#include <cstdint>
#include <optional>

// VCU决定实际模式；此类仅维护ADC请求与任务启动授权。
class AutoDriveGate
{
public:
  bool set_mode(std::int32_t mode)
  {
    if (mode_ == mode) {return false;}
    mode_ = mode;
    started_ = false;
    if (mode == 2) {
      remote_locked_ = true;
      requested_ = false;
      pending_request_.reset(); // 遥控反馈后不再向底盘发送任何自动请求。
      last_request_.reset(); // 后续回待机时建立新的请求会话。
    } else if (mode == 0) {
      if (remote_locked_) {auto_apply_ = true;}
      remote_locked_ = false;
      if (requested_) {
        auto_apply_ = false; // VCU退出自动模式后不自行反复申请。
        set_request(false);
      }
    } else if (mode != 1) {
      auto_apply_ = false;
      set_request(false);
    }
    evaluate();
    return true;
  }

  // 文档未定义，经用户同意新增：adc_self_check_st，Bool。
  // true=ADC自检、地图、定位/位姿均就绪；未收到不作为通过。
  void set_self_check(bool ready)
  {
    if (ready && self_check_ != true) {auto_apply_ = true;}
    self_check_ = ready;
    evaluate();
  }
  void set_running(bool running) {running_ = running; evaluate();}
  void set_obstacle(bool obstacle) {obstacle_ = obstacle; evaluate();}
  void set_alarm(std::int32_t alarm) {alarm_ = alarm; evaluate();}
  // 文档未定义，经用户同意复用：emergency_stop_st，Bool，true=急停。
  void set_emergency_stop(bool active) {emergency_stop_ = active; evaluate();}
  // 文档未定义，经用户同意复用：ads_abnormal_st，Bool，true=异常/降级失效。
  void set_ads_abnormal(bool abnormal) {ads_abnormal_ = abnormal; evaluate();}

  void set_start_command(bool start)
  {
    if (!start) {
      started_ = false;
      auto_apply_ = false; // 停止之后不立即再次自动申请。
      if (!remote_locked_ && mode_ != 2) {set_request(false);}
      return;
    }
    if (remote_locked_) {return;}
    if (mode_ == 0) {
      auto_apply_ = true;
      evaluate(); // 按键可重新申请，不能缓存为将来的任务启动授权。
    } else if (mode_ == 1 && safety_ready()) {
      set_request(true);
      started_ = true;
    }
  }

  bool enabled() const {return mode_ == 1 && !remote_locked_ && started_ && safety_ready();}
  bool requested() const {return requested_;}
  std::optional<bool> take_request()
  {
    const auto result = pending_request_;
    pending_request_.reset();
    if (result) {last_request_ = result;}
    return result;
  }

  const char * status() const
  {
    if (remote_locked_) {return "remote takeover; waiting for standby";}
    if (!mode_ || (*mode_ != 0 && *mode_ != 1)) {return "mode unknown or invalid";}
    if (!self_check_) {return "self-check feedback unknown";}
    if (!*self_check_) {return "self-check/map/localization not ready";}
    if (!emergency_stop_) {return "emergency-stop feedback unknown";}
    if (*emergency_stop_) {return "emergency stop active";}
    if (!alarm_) {return "fault feedback unknown";}
    if (*alarm_ != 0) {return "fault feedback nonzero";}
    if (!ads_abnormal_) {return "ADS health feedback unknown";}
    if (*ads_abnormal_) {return "ADS abnormal";}
    if (enabled()) {return "automatic tasks authorized";}
    if (mode_ == 1) {return "automatic feedback confirmed; waiting for new start command";}
    if (requested_) {return "entry requested; waiting for VCU feedback";}
    if (!auto_apply_) {return "stopped or revoked; waiting for new start/self-check cycle";}
    if (!running_) {return "motion feedback unknown";}
    if (*running_) {return "entry blocked: vehicle moving";}
    if (!obstacle_) {return "obstacle feedback unknown";}
    if (*obstacle_) {return "entry blocked: obstacle present";}
    return "ready";
  }

private:
  bool safety_ready() const
  {
    return self_check_ == true && emergency_stop_ == false &&
           alarm_ == 0 && ads_abnormal_ == false;
  }
  void set_request(bool request)
  {
    if (remote_locked_) {return;}
    // 初次显式停止也应发出false，不能因本地没有申请过而吞掉退出命令。
    if (requested_ != request || !last_request_) {pending_request_ = request;}
    requested_ = request;
  }
  void evaluate()
  {
    if (remote_locked_) {return;}
    if (!safety_ready()) {
      if (requested_ || started_) {
        auto_apply_ = false;
        started_ = false;
        set_request(false);
      }
      return;
    }
    if (mode_ == 0 && requested_ && (running_ != false || obstacle_ != false)) {
      auto_apply_ = false; // 待确认的进入请求仍须满足静止、无障碍条件。
      set_request(false);
    }
    // 静止和障碍物仅约束进入；已经确认自动模式后不据此撤销运行授权。
    if (mode_ == 0 && running_ == false && obstacle_ == false && auto_apply_ && !requested_) {
      set_request(true);
    }
  }

  // 以下均为内部状态，非文档业务信号，经用户同意新增；optional区分未知反馈。
  std::optional<std::int32_t> mode_;
  std::optional<bool> self_check_;
  std::optional<bool> running_;
  std::optional<bool> obstacle_;
  std::optional<std::int32_t> alarm_;
  std::optional<bool> emergency_stop_;
  std::optional<bool> ads_abnormal_;
  bool started_{false};
  bool requested_{false};
  bool auto_apply_{true};
  bool remote_locked_{false};
  std::optional<bool> pending_request_;
  std::optional<bool> last_request_;
};
