# 车身控制节点

`body_control_node` 实现自动驾驶启动请求，以及行驶故障、故障停车、自动驾驶前进、障碍物、左右转、倒车、行人礼让语音请求。节点不负责底盘模式仲裁、路径绕行、靠边停车、刹车或播放音频。

## 输入和输出

| 方向 | 话题 | ROS 2 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/veh_mode_st` | `std_msgs/msg/Int32` | 0 待机、1 自动、2 远程遥控 |
| 输入 | `/mode_sw_cmd` | `std_msgs/msg/Bool` | true 启动、false 停止 |
| 输入 | `/veh_turn_st` | `std_msgs/msg/Int32` | 0 直行、1 左转、2 右转 |
| 输入 | `/veh_gear_st` | `std_msgs/msg/Int32` | 文档已定义：0 P、1 R、2 N、3 D |
| 输入 | `/obs_detect_st` | `std_msgs/msg/Bool` | 文档已定义：false 无障碍物、true 有障碍物 |
| 输入 | `/alarm_diag_st` | `std_msgs/msg/Int32` | 文档已有故障告警状态；用户确认 0 正常、正数故障，负数不视为已确认故障 |
| 输入 | `/veh_running_st` | `std_msgs/msg/Bool` | false 静止、true 行驶 |
| 输入 | `/pedestrian_cross_st` | `std_msgs/msg/Bool` | true 检测到行人横穿 |
| 输出 | `/ad_mode_req` | `std_msgs/msg/Bool` | true 自动驾驶模式请求、false 退出请求 |
| 输出 | `/voice_broadcast_id` | `std_msgs/msg/Int32` | 7 自动驾驶前进、8 障碍物、9 右转、10 左转、11 行人礼让、12 倒车、15 行驶故障、16 故障停车 |

规范中 `ad_mode_req` 是 0/1 枚举，本示例按当前需求用 Bool 承载 false/true。接入实际车辆时需和接收端约定一致的消息类型。

## 自动驾驶启动顺序

1. 接收车辆模式反馈 `veh_mode_st=1`，此时等待启动，不播报。
2. 随后接收 `mode_sw_cmd=true`，先发布 `ad_mode_req=true`，再启用语音场景判断。
3. 相同模式反馈、重复启动命令不会重新触发当前语音或重置播报周期。
4. 收到 `mode_sw_cmd=false` 或模式切换，发布 `ad_mode_req=false`，停止依赖自动启动授权的提示并清除 ID 7 周期和待播请求。有效的故障提示 ID 15、16 不受此限制，保持原周期。
5. 重新进入自动模式后，必须再次收到启动命令。非自动模式收到的启动命令不会缓存为之后的启动授权。

模式变化时发布一次 false，启动按钮每次更新时发布当前请求值。该实现使用模式反馈和按钮授权作为语音开关；请求发布不等于底盘执行成功。不同输出话题间不保证接收顺序，但节点内先调用请求发布，再处理语音。

## 语音触发逻辑

除 ID 15、16 外，其他语音均要求“自动模式且已启动”。

- `alarm_diag_st>0` 且已收到 `veh_running_st=true`：ID 15，车辆故障，正在靠边停车，请注意避让。任何模式及未按启动按钮时均可触发；首次立即发送，此后每 5 秒一次。
- `alarm_diag_st>0` 且已收到 `veh_running_st=false`：ID 16，车辆故障，请勿靠近，并请绕行。任何模式及未按启动按钮时均可触发；首次立即发送，此后每 5 秒一次。
- 已收到 `veh_running_st=true` 且 `veh_gear_st=3`（D 挡）：ID 7，自动驾驶中，请注意车距和避让。首次立即请求，此后每 120 秒一轮，优先级 P1。
- `veh_gear_st=1`（R 挡）：ID 12，倒车中，请注意安全。只以挂 R 挡为场景条件，不额外要求车辆已运动。
- `obs_detect_st=true`：ID 8，紧急避让，请注意安全；false 结束该提示。
- 有行人横穿，且确实收到行驶状态 false：ID 11，车辆停车中，请行人通行。
- 左转状态 1：ID 10，左转弯，请注意安全。
- 右转状态 2：ID 9，右转弯，请注意安全。

不再新增斑马线专用信号。只使用文档中的行人、行驶状态，因此无法单独识别斑马线位置或证明停车原因。上游应提供符合业务语义的行人横穿状态。

P0 场景首次满足时立即发送，此后每 5 秒发送一次；相同场景重复输入不重置计时。礼让退出后，如转弯条件仍成立，会立即恢复转弯提示。停止或退出自动模式后，仅有效的故障提示 ID 15、16 可以继续发送。自动驾驶启动时使用当前缓存的场景状态。

ID 15、16 仅使用文档已有的 `alarm_diag_st` 和 `veh_running_st`，没有新增业务状态变量、话题、订阅或消息字段。规范未列举故障等级取值，0/正数/负数的解释由用户明确确认。未收到过行驶状态时，两者均不触发。故障为正数时，行驶中选择 ID 15，静止选择 ID 16；两者切换立即发送新 ID，并从切换时刻重新计算 5 秒周期。故障值变为非正数时结束故障提示，恢复其他符合启用条件的场景。重复行驶或停车状态、不同正数等级间切换、单独停止按钮或模式变化，均不重置当前故障提示的周期。

ID 16 按用户选择的方案 1 实现：以“故障且静止”近似判断故障靠边停车完成，无法确认实际停车位置或证明靠边停车任务已完成。节点启动时已经静止且有故障，也允许触发，不要求此前发送过 ID 15。没有增加靠边完成信号，也不将 `veh_arrive_st` 到站停靠信号用作故障靠边完成信号。

障碍物提示按用户选择的方案 1 实现：以文档已有的 `obs_detect_st` 判断开始与结束，不新增绕行状态信号。这与规范“实际绕行时触发、绕行完成后停止”的语义有差异；检测到障碍物不代表正在绕行，检测清除也不代表规划绕行任务已结束。当前不额外要求行驶状态或前进挡，自动模式已启动且检测到障碍物就进入该场景（仍受同级仲裁影响）。

故障提示 ID 15、16、倒车、障碍物、礼让和左右转在规范中均为 P0，规范没有规定它们之间的顺序；当前约定故障 > 倒车 > 障碍物 > 礼让 > 左右转。ID 15、16 根据行驶状态互斥选择。高优先场景结束后按当前状态恢复其他 P0 提示；如无 P0 则按 ID 7 的到期和超时规则调度。每条语音消息表示一次新的播放请求，下游不能只检测 ID 数值变化。

### ID 7 的周期与超时

- 前进条件首次成立立即进入第一轮，此后以这个时刻为基准，每 120 秒一轮；重复输入或 P0 出现、结束均不改变该时间轴。
- 每轮到期若无 P0 场景，发送 ID 7；若有 P0 场景则最多等待 10 秒。P0 在不足 10 秒时结束且前进条件仍成立，发送本轮请求；等待达到 10 秒即丢弃，等待下一轮。
- 没有有效待播请求时，P0 解除不会额外触发 ID 7；丢弃或延迟发送不改变下一轮到期时间。调度延迟时跳过历史轮次，只考虑最近一轮且不集中补发。
- 停车、退出 D 挡、停止或退出自动模式，立即清除 ID 7 的周期和待播请求；再次满足前进条件时重新开始第一轮。
- 因无播放完成反馈，整个 P0 场景有效期间均视为占用，不会在两次 P0 请求的 5 秒间隔内穿插 ID 7。实际音频抢占由下游实现。

节点用一个 50ms 定时器及状态回调共同检查单调时钟到期时间，P0 的 5 秒周期与 P1 的 120 秒周期独立。新增计时字段仅属于内部实现，未新增业务接口或消息字段。

倒车功能只增加文档已有的 `veh_gear_st` 输入，没有新增文档外的信号接口。后续如需新增文档未定义的业务信号，必须先获得用户同意，并在代码中注明。

目前未实现输入超时、自动驾驶请求的自检和故障准入校验、模式请求应答超时及重试、音量请求和音频中断指令。故障播报不会自动拒绝自动驾驶启动请求。输入保留最后状态，通信中断不会自动清除授权、故障或行驶状态。停止发送请求不等于立即中断下游正在播放的音频。

## 编译和运行

```bash
source /opt/ros/humble/setup.bash
cd /home/user/桌面/function/ros2_ws
colcon build --packages-select body_control --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
source install/setup.bash
ros2 run body_control body_control_node
```

## 手动验证

下面其他功能的示例假定没有故障。若曾注入故障，先发送 `/alarm_diag_st` 的 `data: 0` 清除测试状态，避免 ID 15 或 ID 16 持续优先。

分别在两个终端中观察请求和语音，每个终端先加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /ad_mode_req
```

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /voice_broadcast_id
```

在第三个终端先给定右转场景，再按顺序进入自动模式并启动：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

此时没有语音。发送启动按钮：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

应看到请求 true 和语音 ID 9，此后每约 5 秒再次输出 9。再模拟有行人且车辆已停：

```bash
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

应切换为 ID 11。停止命令应输出请求 false 并停止后续语音：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
```

再次启动后，再模拟退出自动模式：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

也应输出请求 false 并停止后续语音。再次进入自动模式但不发启动命令时应保持静默。

## 倒车手动验证

启动节点，并用另一个终端观察 `/voice_broadcast_id`。在第三个终端加载 ROS 2 环境后执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 1}"
```

应立即输出 ID 12，之后每约 5 秒发送一次。重复发送 R 挡不应额外立即播报或重置计时。挂入 D 挡后停止倒车提示；若同时已收到行驶中状态，则会开始 ID 7 的新一轮：

```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 3}"
```

再次挂 R 挡应立即恢复 ID 12。R 挡期间发送停止按钮 false 或退出自动模式，也应停止后续提示。

## 障碍物提示手动验证

启动节点后，在另一个终端观察 `/voice_broadcast_id`，第三个终端执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 3}"
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

全新启动的节点尚未收到启动按钮，不应发出语音。再发送：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

应立即输出 ID 8，之后每约 5 秒输出一次。重复发送 `obs_detect_st=true` 不应额外立即输出或重置计时。发送 false 后停止该提示：

```bash
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: false}"
```

再次发送 true 应立即恢复 ID 8。检测为 true 时，按停止按钮或退出自动模式也应停止后续提示。清除障碍物后只有存在未超时的待播 ID 7 时才会补发，否则等下一轮到期。

## 自动驾驶前进提示手动验证

重启节点，在另一个终端观察 `/voice_broadcast_id`，然后依次执行：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 3}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

此时未启动，不应有语音。再发送启动命令：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

应立即发送 ID 7，之后在没有 P0 的情况下，每约 120 秒发送一次。重复发送行驶中、D 挡、相同模式或启动命令，不应额外发送或重置计时。

发送行驶状态 false 或换入 N 挡，应取消常规提示；恢复行驶中且 D 挡后，立即开始新一轮。停止按钮 false 或退出自动模式也应取消请求。

验证 P0 等待规则，可在新一轮启动前置 `obs_detect_st=true`，此时应先输出 ID 8；从前进条件成立的时刻开始，不足 10 秒清除障碍物应补发 ID 7，超过 10 秒清除则不补发。无论哪种情况，下一轮仍从本轮开始时刻起计算 120 秒。命令行和 DDS 调度有延迟，精确 10 秒及 120 秒边界由可控时间单元测试覆盖。

## 行驶故障提示手动验证

重启节点，不发送自动模式或启动命令，在另一个终端观察 `/voice_broadcast_id`。第三个终端先注入故障：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 1}"
```

此时尚未收到行驶状态，不应播报。再确认车辆行驶：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

应立即输出 ID 15，此后每约 5 秒输出一次。重复故障或将故障改为其他正数，不应额外立即输出或重置周期。在持续行驶且故障为正数时执行：

```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

ID 15 应保持原周期继续发送。发送停车状态应立即切换到 ID 16，随后每 5 秒一次：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

再次发送行驶状态 true 应立即恢复 ID 15，并重新开始 5 秒周期；将故障值改为 0 或负数应结束故障提示。测试结束清除故障：

```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
```

还可先发送行驶状态 true、再发送故障值，确认另一种输入顺序也能触发。自动模式已启动时，可结合倒车、障碍物、转弯提示验证 ID 15 抢占；故障清除后恢复相应场景。P1 在故障期间仍遵循 10 秒等待及超时规则。

## 故障停车提示手动验证

重启节点，不发送自动模式或启动命令，在另一个终端观察 `/voice_broadcast_id`。第三个终端先确认车辆静止：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

此时无故障，不应输出语音。再发送故障：

```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 1}"
```

应立即发送 ID 16，此后每约 5 秒重复。也可重启节点后先发送故障，再发送静止状态；只有完整接收到两个条件后才输出 ID 16，不需要先经历行驶故障。

重复发送静止状态、将故障等级改为正数 2、按停止按钮或切换到遥控模式，都不应额外立即发送或改变周期：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

发送行驶状态 true 应立即切换到 ID 15；再发送 false 应立即回到 ID 16，两次切换均重新开始 5 秒周期：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

故障清零或变为负数后停止故障提示；自动驾驶已启动时可恢复其他满足条件的场景。测试结束清除故障：

```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
```

## 逻辑测试

```bash
cd /home/user/桌面/function/ros2_ws/build/body_control
ctest --output-on-failure
```

`voice_scene_test` 覆盖 P0 场景、自动模式启动逻辑及故障取值；`voice_scheduler_test` 使用传入的单调时钟时间点，覆盖首次立即发送、120 秒周期、重复状态、P0 的 5 秒周期和切换、10 秒等待边界、固定周期、延迟轮次跳过、停止及重新启用，并覆盖行驶及静止故障输入顺序、等级变化、模式和按钮独立性、故障抢占与恢复、ID 15/16 双向切换和重新计时、故障期间的 P1 超时以及停车时取消 P1 待播。测试无需真实等待两分钟，不验证 DDS 通信、实际定时器抖动或硬件执行。
