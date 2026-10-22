#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "auto_drive.hpp"
#include "voice_scene.hpp"
#include "voice_scheduler.hpp"

using namespace std::chrono_literals;

class BodyControlNode : public rclcpp::Node
{
public:
  BodyControlNode() : Node("body_control_node")
  {
    // 发布语音播报请求。
    voice_pub_ = create_publisher<std_msgs::msg::Int32>("voice_broadcast_id", 10);
    // 发布自动驾驶模式状态请求。
    ad_mode_pub_ = create_publisher<std_msgs::msg::Bool>("ad_mode_req", 10);
    // 文档已有倒车灯请求：true-开启，false-关闭；仅复用已有挡位信号。
    reverse_light_pub_ = create_publisher<std_msgs::msg::Bool>("reverse_light_req", rclcpp::QoS(1).reliable());
    publish_reverse_light_request(); // 未收到挡位时，默认关闭。


    // 50ms 检查独立调度：P0 5 秒、箱门 10 秒、自动前进 120 秒、遥控行驶 60 秒等。
    timer_ = create_wall_timer(50ms, [this]() {
      publish_reverse_light_request();
      update_voice_scene();
    });


    //整车模式状态（0-待机，1-自动驾驶，2-远程遥控）
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
      "veh_mode_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        state_.veh_mode_st = msg->data; // 保存已有模式信号，供 ID 14 调度使用。
        if (auto_drive_.set_mode(msg->data)) {
          publish_ad_mode_request();
        }
        update_voice_scene();
      });
    //启动按键指令（0-停止，1-启动）
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      "mode_sw_cmd", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        auto_drive_.set_start_command(msg->data);
        // 先发布自动驾驶请求，再执行语音判断。
        publish_ad_mode_request();
        update_voice_scene();
      });
    // 整车挡位信号（0-P，1-R，2-N，3-D）。
    gear_sub_ = create_subscription<std_msgs::msg::Int32>(
      "veh_gear_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        state_.veh_gear_st = msg->data;
        publish_reverse_light_request(); // 先更新灯光，不等待定时器或语音仲裁。
        update_voice_scene();
      });
    // 车辆到站停靠状态（false-未到站，true-停靠完成）。
    arrive_sub_ = create_subscription<std_msgs::msg::Bool>(
      "veh_arrive_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.veh_arrive_st = msg->data;
        update_voice_scene();
      });
    // 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。
    // 临时 EPB 接口：Bool，false-未拉起，true-已拉起；仅接收，不自动生成。
    epb_sub_ = create_subscription<std_msgs::msg::Bool>(
      "veh_epb_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.veh_epb_st = msg->data;
        update_voice_scene();
      });
    // 货箱门锁状态：false-未锁，true-上锁；不是开锁控制指令。
    trunk_lock_sub_ = create_subscription<std_msgs::msg::Bool>(
      "trunk_lock_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.trunk_lock_state_received = true;
        state_.trunk_lock_st = msg->data;
        update_voice_scene();
      });
    // 障碍物检测状态（false-无，true-有）。
    obs_sub_ = create_subscription<std_msgs::msg::Bool>(
      "obs_detect_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.obs_detect_st = msg->data;
        update_voice_scene();
      });
    // 电池电量预警，由上游按 SOC<=20% 置 true，不新增 SOC 信号。
    battery_sub_ = create_subscription<std_msgs::msg::Bool>(
      "battery_low_power_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.battery_low_power_st = msg->data;
        update_voice_scene();
      });
    // 故障警告状态：正数故障，0 正常。
    alarm_sub_ = create_subscription<std_msgs::msg::Int32>(
      "alarm_diag_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        state_.alarm_diag_st = msg->data;
        update_voice_scene();
      });
    // 接收转弯状态。（0-直行，1-左转，2-右转）
    turn_sub_ = create_subscription<std_msgs::msg::Int32>(
      "veh_turn_st",
      rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        state_.turn_state =
          (msg->data == 1 || msg->data == 2) ? msg->data : 0;
        update_voice_scene();
      });
    //接受车辆行驶状态。（0-静止，1-行驶中）
    running_sub_ = create_subscription<std_msgs::msg::Bool>(
      "veh_running_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.running_state_received = true;
        state_.vehicle_running = msg->data;
        update_voice_scene();
      });
    // 文档未定义，经用户授权暂定，名称、类型及单位待后续确认。
    // 临时车速接口：Float32，km/h；每条消息更新内部序号，供 ID 6 识别起步后新反馈。
    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      "veh_speed_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
        state_.speed_state_received = true;
        state_.veh_speed_st = msg->data;
        ++state_.speed_feedback_seq; // 数值相同或非法也递增；序号不作为业务接口发布。
        update_voice_scene();
      });
    // 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。
    // 临时等灯接口：Bool，true-正在等红绿灯，不以普通停车代替。
    traffic_wait_sub_ = create_subscription<std_msgs::msg::Bool>(
      "traffic_light_wait_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.traffic_light_wait_st = msg->data;
        update_voice_scene();
      });
    // 文档已有光照数值接口，单位未定义；非法数值由等待任务忽略。
    // 阈值由用户暂定为5，待后续文档同步；文档未定义光照单位。
    ambient_light_sub_ = create_subscription<std_msgs::msg::Float64>(
      "ambient_light_level_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        state_.ambient_light_received = true;
        state_.ambient_light_level_st = msg->data;
        update_voice_scene();
      });
    //行人横穿检测状态。（0-未横穿，1-有）
    pedestrian_sub_ = create_subscription<std_msgs::msg::Bool>(
      "pedestrian_cross_st", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        state_.pedestrian_crossing = msg->data;
        update_voice_scene();
      });
  }

private:
  void publish_reverse_light_request()
  {
    std_msgs::msg::Bool msg;
    // 挂 R 挡即持续点亮，包括静止；其他值（含非法挡位）均关闭。
    // 独立于模式、启动授权、行驶状态和故障语音，不新增业务状态变量。
    msg.data = state_.veh_gear_st == 1;
    reverse_light_pub_->publish(msg);
  }

  void publish_ad_mode_request()//输出自动驾驶模式状态
  {
    std_msgs::msg::Bool msg;
    msg.data = auto_drive_.enabled();
    ad_mode_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "ad_mode_req=%s", msg.data ? "true" : "false");
  }

  void update_voice_scene()//更新语音播报状态
  {
    state_.auto_drive_enabled = auto_drive_.enabled();
    const auto voice_id = scheduler_.update(state_, VoiceScheduler::Clock::now());
    if (voice_id) {
      publish_voice(*voice_id);
    }
  }

  void publish_voice(std::int32_t voice_id)//输出语音播报ID
  {
    // 仅执行调度器批准的请求；ID 2、3、4、5、14、15、16 允许在非自动模式下发送。
    // 其他语音的启用条件由场景选择和调度模块保证。
    std_msgs::msg::Int32 msg;
    msg.data = voice_id;
    voice_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "voice_broadcast_id=%d", msg.data);
  }

  VoiceSceneState state_;
  AutoDriveGate auto_drive_;
  VoiceScheduler scheduler_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr voice_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ad_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reverse_light_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr turn_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arrive_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr epb_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trunk_lock_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obs_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr alarm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr running_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr traffic_wait_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ambient_light_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pedestrian_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyControlNode>());
  rclcpp::shutdown();
  return 0;
}
