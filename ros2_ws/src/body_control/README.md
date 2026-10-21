# 车身控制节点

`body_control_node` 实现自动驾驶启动请求，以及装卸货完成确认、开锁成功、装卸货和关闭箱门提醒、低电量、远程/遥控驾驶、行驶故障、故障停车、自动驾驶前进、障碍物、左右转、倒车、行人礼让语音请求。节点不负责底盘模式仲裁、门锁控制、路径绕行、靠边停车、刹车或播放音频。

## 输入和输出

| 方向 | 话题 | ROS 2 类型 | 含义 |
| --- | --- | --- | --- |
| 输入 | `/veh_mode_st` | `std_msgs/msg/Int32` | 0 待机、1 自动、2 远程遥控 |
| 输入 | `/mode_sw_cmd` | `std_msgs/msg/Bool` | true 启动、false 停止 |
| 输入 | `/veh_turn_st` | `std_msgs/msg/Int32` | 0 直行、1 左转、2 右转 |
| 输入 | `/veh_gear_st` | `std_msgs/msg/Int32` | 文档已定义：0 P、1 R、2 N、3 D |
| 输入 | `/veh_arrive_st` | `std_msgs/msg/Bool` | 文档已有到站停靠状态：false 未到站、true 停靠完成 |
| 输入 | `/veh_epb_st` | `std_msgs/msg/Bool` | 临时授权接口：false 未拉起、true 已拉起，默认 false；名称、类型及取值待确认 |
| 输入 | `/trunk_lock_st` | `std_msgs/msg/Bool` | 文档已有货箱门锁状态：false 未锁、true 上锁；不是开锁指令 |
| 输入 | `/obs_detect_st` | `std_msgs/msg/Bool` | 文档已定义：false 无障碍物、true 有障碍物 |
| 输入 | `/battery_low_power_st` | `std_msgs/msg/Bool` | 文档已有电池电量预警；上游按 SOC≤20% 置 true |
| 输入 | `/alarm_diag_st` | `std_msgs/msg/Int32` | 文档已有故障告警状态；用户确认 0 正常、正数故障，负数不视为已确认故障 |
| 输入 | `/veh_running_st` | `std_msgs/msg/Bool` | false 静止、true 行驶 |
| 输入 | `/pedestrian_cross_st` | `std_msgs/msg/Bool` | true 检测到行人横穿 |
| 输出 | `/ad_mode_req` | `std_msgs/msg/Bool` | true 自动驾驶模式请求、false 退出请求 |
| 输出 | `/voice_broadcast_id` | `std_msgs/msg/Int32` | 2 装卸货完成确认、3 装卸货、4 提醒关闭箱门、5 开锁成功、7 自动驾驶前进、8 障碍物、9 右转、10 左转、11 行人礼让、12 倒车、14 远程/遥控驾驶、15 行驶故障、16 故障停车、17 低电量 |

规范中 `ad_mode_req` 是 0/1 枚举，本示例按当前需求用 Bool 承载 false/true。接入实际车辆时需和接收端约定一致的消息类型。

**临时 EPB 协议：`veh_epb_st` 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。** 字段声明和订阅处均有相同标识。节点仅接收外部 EPB 状态，不自动生成，不以普通制动 `veh_brake_st` 代替 EPB；实际对接前需确认该临时协议。

## 自动驾驶启动顺序

1. 接收车辆模式反馈 `veh_mode_st=1`，此时等待启动，不播报。
2. 随后接收 `mode_sw_cmd=true`，先发布 `ad_mode_req=true`，再启用语音场景判断。
3. 相同模式反馈、重复启动命令不会重新触发当前语音或重置播报周期。
4. 收到 `mode_sw_cmd=false` 或模式切换，发布 `ad_mode_req=false`，停止依赖自动启动授权的提示并清除 ID 7 周期和待播请求。有效的故障提示 ID 15、16 不受此限制，保持原周期。
5. 重新进入自动模式后，必须再次收到启动命令。非自动模式收到的启动命令不会缓存为之后的启动授权。

模式变化时发布一次 false，启动按钮每次更新时发布当前请求值。自动驾驶相关语音使用模式反馈和按钮授权作为开关，ID 2、3、4、5、14 和故障提示另有启用条件；请求发布不等于底盘执行成功。不同输出话题间不保证接收顺序，但节点内先调用请求发布，再处理语音。远程/遥控模式下按钮 true 不会产生 `ad_mode_req=true`。

## 语音触发逻辑

除 ID 2、3、4、5、14、15、16 外，其他语音均要求“自动模式且已启动”。

- `veh_arrive_st=true`、`veh_gear_st=2`（N 挡）且 `veh_epb_st=true`：ID 3，车辆已到达目的地请及时装卸货，P2，每次到站最多请求一次，不限制模式和启动按钮。
- 相同到站条件同时建立 ID 4：装、卸货完成后请关闭箱门，谢谢，P2，每次到站最多请求一次。先请求 ID 3，后续调度再请求 ID 4，不额外判断装卸完成、箱门开闭或门锁状态。
- 已收到的 `trunk_lock_st` 从 true 变为 false：从事件时刻起第 60、120、180 秒各请求一轮 ID 2，关闭箱门前，请确认货物是否已全部装、卸完成，P1；不限制模式及其他车辆状态，重新上锁取消剩余轮次。
- 已收到的 `trunk_lock_st` 从 true 变为 false：ID 5，开锁成功，P2，每次有效转换最多一次；首次状态消息只初始化，不播报。
- `alarm_diag_st>0` 且已收到 `veh_running_st=true`：ID 15，车辆故障，正在靠边停车，请注意避让。任何模式及未按启动按钮时均可触发；首次立即发送，此后每 5 秒一次。
- `alarm_diag_st>0` 且已收到 `veh_running_st=false`：ID 16，车辆故障，请勿靠近，并请绕行。任何模式及未按启动按钮时均可触发；首次立即发送，此后每 5 秒一次。
- 切入 `veh_mode_st=2`：一次 ID 14，远程/遥控驾驶中，请注意避让；该模式下确认开始行驶时立即请求 ID 14，此后每 60 秒一轮，优先级 P1。切入时已在行驶，两种请求合并为一次。
- 已收到 `veh_running_st=true` 且 `veh_gear_st=3`（D 挡）：ID 7，自动驾驶中，请注意车距和避让。首次立即请求，此后每 120 秒一轮，优先级 P1。
- `veh_gear_st=1`（R 挡）：ID 12，倒车中，请注意安全。只以挂 R 挡为场景条件，不额外要求车辆已运动。
- `obs_detect_st=true`：ID 8，紧急避让，请注意安全；false 结束该提示。
- 有行人横穿，且确实收到行驶状态 false：ID 11，车辆停车中，请行人通行。
- 左转状态 1：ID 10，左转弯，请注意安全。
- 右转状态 2：ID 9，右转弯，请注意安全。
- 自动模式有效启动后、开始行驶前，确认静止且 `battery_low_power_st=true`：ID 17，当前电量低，请及时充电，P0。每次有效启动最多请求两次，正常间隔 10 秒。

不再新增斑马线专用信号。只使用文档中的行人、行驶状态，因此无法单独识别斑马线位置或证明停车原因。上游应提供符合业务语义的行人横穿状态。

除低电量 ID 17 的有限次数任务外，P0 场景首次满足时立即发送，此后每 5 秒发送一次；相同场景重复输入不重置计时。礼让退出后，如转弯条件仍成立，会立即恢复转弯提示。停止或退出自动模式后，有效的故障提示 ID 15、16 仍可发送；处于远程/遥控模式时 ID 14 独立调度。自动驾驶启动时使用当前缓存的场景状态。

ID 15、16 仅使用文档已有的 `alarm_diag_st` 和 `veh_running_st`，没有新增业务状态变量、话题、订阅或消息字段。规范未列举故障等级取值，0/正数/负数的解释由用户明确确认。未收到过行驶状态时，两者均不触发。故障为正数时，行驶中选择 ID 15，静止选择 ID 16；两者切换立即发送新 ID，并从切换时刻重新计算 5 秒周期。故障值变为非正数时结束故障提示，恢复其他符合启用条件的场景。重复行驶或停车状态、不同正数等级间切换、单独停止按钮或模式变化，均不重置当前故障提示的周期。

ID 16 按用户选择的方案 1 实现：以“故障且静止”近似判断故障靠边停车完成，无法确认实际停车位置或证明靠边停车任务已完成。节点启动时已经静止且有故障，也允许触发，不要求此前发送过 ID 15。没有增加靠边完成信号，也不将 `veh_arrive_st` 到站停靠信号用作故障靠边完成信号。

障碍物提示按用户选择的方案 1 实现：以文档已有的 `obs_detect_st` 判断开始与结束，不新增绕行状态信号。这与规范“实际绕行时触发、绕行完成后停止”的语义有差异；检测到障碍物不代表正在绕行，检测清除也不代表规划绕行任务已结束。当前不额外要求行驶状态或前进挡，自动模式已启动且检测到障碍物就进入该场景（仍受同级仲裁影响）。

故障提示 ID 15、16、倒车、障碍物、礼让、左右转和低电量在规范中均为 P0，规范没有规定它们之间的顺序；按用户确认的策略采用故障 > 倒车 > 障碍物 > 礼让 > 左右转 > 低电量。ID 15、16 根据行驶状态互斥选择。高优先场景结束后按当前状态恢复其他 P0 提示；如无 P0 及有效低电量任务，则将已到期且未超时的 P1 请求按 ID 2→14→7 调度，最后按用户确认的 P2 顺序 5→3→4 逐次调度。每条语音消息表示一次新的播放请求，下游不能只检测 ID 数值变化。新增 ID 14 不扩大其他 P0 在遥控模式下的启用范围，倒车、障碍物、礼让和转弯仍要求自动模式已启动。

### ID 7 的周期与超时

- 前进条件首次成立立即进入第一轮，此后以这个时刻为基准，每 120 秒一轮；重复输入或 P0 出现、结束均不改变该时间轴。
- 每轮到期若无 P0 场景，发送 ID 7；若有 P0 场景则最多等待 10 秒。P0 在不足 10 秒时结束且前进条件仍成立，发送本轮请求；等待达到 10 秒即丢弃，等待下一轮。
- 没有有效待播请求时，P0 解除不会额外触发 ID 7；丢弃或延迟发送不改变下一轮到期时间。调度延迟时跳过历史轮次，只考虑最近一轮且不集中补发。
- 停车、退出 D 挡、停止或退出自动模式，立即清除 ID 7 的周期和待播请求；再次满足前进条件时重新开始第一轮。
- 因无播放完成反馈，整个 P0 场景有效期间均视为占用，不会在两次 P0 请求的 5 秒间隔内穿插 ID 7。实际音频抢占由下游实现。

### ID 14 的切入提示、行驶循环与超时

- 复用已有的 `/veh_mode_st` 和 `/veh_running_st` 订阅，在语音状态中保存文档已有的模式值。首次收到模式 2 或从其他模式切回 2 时，立即生成一次切入提示；重复模式反馈不重复触发。
- 切入时未收到行驶状态或已确认静止，仍允许一次切入提示，但不启动循环。模式 2 且已确认行驶中时，每次从非行驶条件进入行驶条件立即请求，此后每 60 秒一轮；不要求 D 挡，不依赖启动按钮或自动驾驶授权。
- 切入与循环请求分别记录到期时刻，遇到有效 P0 场景均最多等待 10 秒，达到各自期限即丢弃。发送时如两种请求均有效，只发一条 ID 14 并清除当前两种请求；一种请求的产生不会延长另一种请求的期限。
- 停车取消行驶循环及其待播请求，但保留仍未超时的切入请求；再行驶立即启动全新 60 秒周期。离开模式 2 则取消所有 ID 14 周期和待播请求，再切入重新触发。
- P0 场景有效期间不穿插 ID 14。P0 解除后没有有效待播请求则不补播；延迟发送或丢弃均不改变下一轮 60 秒到期时间，调度延迟也不集中补发历史轮次。
- 自动与远程模式切换会取消原模式的 ID 7/14 周期，不取消 ID 2 任务。切回自动模式后仍须重新按启动按钮。已发切入提示后，随后才开始行驶属于新的触发，允许立即再发一次；切入时已经在行驶则合并为一次。

### ID 17 的单次发车窗口及有限次数任务

- 使用文档已有的 `battery_low_power_st`，不新增 SOC 数值信号。低电量阈值由上游负责，需将 SOC≤20% 映射为 true，本节点不计算或验证实际 SOC。
- 经用户确认，以自动模式下自动驾驶授权从未启动变为已启动，近似表示一次“远程端点击发车”。这不是独立的远程发车反馈信号。代码中已标注该近似映射。
- 每次有效启动打开一个窗口，在开始行驶前等待已确认静止且低电量。未知行驶状态不按静止处理，较晚到达的低电量或静止反馈允许触发。任务未开始前电量正常，则继续等待，不提前消费本次机会。
- 条件完整时建立两次请求的任务；若无其他 P0 则立即发送第一次，第二次以第一次实际产生发送请求的时刻加 10 秒为到期时刻。重复启动、静止、低电量反馈不会额外触发或重置计时；两次发完即结束，持续低电量不继续循环。
- 现有 P0 场景有效时低电量请求等待，不在其 5 秒请求间隙插播。文档未规定 ID 17 的等待超时，窗口有效时保留任务；第二次受阻时延后发送，实际间隔至少 10 秒。调度延迟不集中补发，两次请求间也不穿插 P1。
- 开始行驶、取消启动或退出自动模式时关闭本次窗口并取消剩余请求；随后停车不重开，必须再次有效启动。节点在启动时已确认行驶，也会直接关闭本次窗口。
- 任务开始后，低电量标志变为 false 则取消剩余请求，本次发车即使重新变为低电量也不再触发；需要新的有效启动。此规则也适用于任务已经建立但第一次请求被其他 P0 阻塞的情况。
- 次数按节点产生的 ID 17 发送请求计数，不表示扬声器已播放完成。本功能不延迟 `ad_mode_req`、不执行制动或禁止起步；车辆先行驶时取消未发送的低电量提示，不等待两次提示结束。

### ID 3、4 的单次到站及超时

- 复用已有 N 挡判断，接收文档已有到站状态和已授权的临时 EPB 接口；到站、N 挡、EPB 拉起三个条件同时满足时同时建立 ID 3 和 ID 4 请求。到站后允许等待其他反馈，启动时第一次收到完整条件也可触发。没有新增订阅、业务信号或消息字段，临时 EPB 接口待确认标识保持不变。
- 两条各自记录待播状态，按用户确认的顺序先 3 后 4，每次调度最多输出一条，允许后续调度在相同时间点取出下一条，不设置人工播放间隔。
- 建立请求即使用本次到站的机会，不以最终能否发出为准。重复状态不重新触发，也不延长等待。每条最多一次；发送、超时或取消后，即使 EPB 或挡位条件重新成立也不重试；只有收到 `veh_arrive_st=false` 才重新允许下一次到站建立两条新请求。
- 两条使用相同触发时刻计算 10 秒期限，达到期限丢弃尚未发送的请求，边界不补播。发送 ID 3 不重置 ID 4 的期限，ID 3 可能成功发送而 ID 4 超时丢弃。待播时离站、退出 N 挡或 EPB 释放即取消尚未发送的请求，已发请求不能撤回；模式和启动按钮变化不取消任务。
- 按已确认的请求级仲裁，所有有效 P0 及低电量任务优先，已到期且未超时的 ID 2/14/7 请求先发送。没有更高优先请求时按 3、4 顺序调度，一次最多一条。
- 没有播放完成反馈，所以只能保证请求调度顺序，实际音频排队由下游负责。ID 14 或 ID 3 请求发出后，下一次调度可发送尚未超时的下一条 P2 请求，不把请求发出当作播放完成。其他请求被优先调度不会重置 ID 3、4 的 10 秒期限。
- 到站任务不触发 ID 1、2，不控制挡位、EPB、车门或车辆运动。ID 2 和 ID 5 由统一识别的锁状态事件触发，不要求到站。尽管 ID 4 文案提到装卸完成，其触发条件严格沿用文档中的到站、N 挡和 EPB，不引入装卸完成或箱门信号。

### ID 2 的三轮装卸货完成确认提醒

- 复用现有 `/trunk_lock_st` 订阅及接收标记，与 ID 5 统一识别已接收锁状态的 true→false 事件。首次 true/false 仅初始化，重复 false 不触发、不重置。用锁状态变化近似表示开锁，不能识别 APP 点击来源；不新增业务信号、话题或消息字段。
- 以开锁事件的接收处理时刻为起点，在第 60、120、180 秒各产生一轮 ID 2 请求，没有立即提醒或第四轮。计时不依赖 ID 5 是否成功发送；ID 5 被阻塞、取消或超时不改变此时间轴。
- 每轮以原定到期时刻计算 10 秒期限，达到期限即丢弃，超时也计入三轮。第一轮若在第 69 秒发送，第二轮仍在第 120 秒到期。调度迟到会跳过已经超时的历史轮次，只发送仍有效的请求，不补偿或集中补发。
- 重新上锁立即取消待播请求及全部后续轮次；再次真正开锁从新事件重新等待 60 秒。模式、启动按钮、到站、挡位、EPB 或行驶状态变化均不取消任务，不额外要求这些条件。
- 有效 P0 场景及低电量有限次数任务优先；P0 整段有效期间不穿插 ID 2。P1 按 ID 2→14→7 调度，P2 保持 ID 5→3→4，每次调度最多一条。其他请求的发送不重置各自期限，阻塞期间也处理到期、超时及重新上锁取消。
- 只请求“关闭箱门前，请确认货物是否已全部装、卸完成”，不判断实际装卸完成，不控制门锁，也不撤回已发请求。节点保证请求调度顺序，实际音频排队由下游负责。

### ID 5 的锁状态事件与超时

- 仅新增文档已有的 `/trunk_lock_st` 输入，不新增 APP 开锁指令或其他文档外业务信号。内部接收标记、前一次锁状态及待播时间不是新增业务接口。
- 经用户选择方案 1，以已接收状态从 true（上锁）变为 false（未锁）近似表示开锁成功，不能识别 APP/小程序点击来源。首次收到 true 或 false 均只初始化，不触发；未收到消息时也不使用默认值构造事件。
- 每次 true→false 建立一次 ID 5 请求，并用同一事件启动独立的 ID 2 三轮计时，重复 false 不重新触发或延长期限。持续未锁不会循环播报。不同于到站任务，锁状态事件不受模式、启动按钮、到站、挡位或 EPB 条件限制。
- 从转换的接收处理时刻计算等待时间，达到 3 秒即丢弃，边界不补播。等待期间重新上锁取消旧请求；再次真实发生 true→false 时才建立新请求并使用新事件的时刻。
- 有效 P0、低电量任务、已到期且未超时的 P1 请求优先；P2 顺序为 ID 5→3→4，每次调度最多一条。发送其他请求不会延长 ID 5 的 3 秒期限，发送 ID 5 也不延长到站两条的原 10 秒期限。
- 不控制门锁，不撤回已发请求。没有音频完成反馈，因此仍只保证请求调度顺序，实际音频排队由下游负责。

节点用一个 50ms 定时器及状态回调共同检查单调时钟到期时间，P0 的 5 秒周期、自动前进的 120 秒周期、远程/遥控行驶的 60 秒周期、低电量的有限次数任务、ID 2 的固定三轮任务和到站装卸货的一次性任务分别管理。窗口、计数和计时字段仅属于内部实现；业务接口中另有上述经用户授权、待确认的临时 EPB 信号。

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

下面其他功能的示例假定没有故障且低电量标志为 false。若曾注入故障，先发送 `/alarm_diag_st` 的 `data: 0` 清除测试状态，避免 ID 15 或 ID 16 持续优先；曾注入低电量则发送 `/battery_low_power_st` 的 `data: false` 取消剩余任务。

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

也应输出自动驾驶请求 false，并停止自动驾驶相关语音；切入远程/遥控模式同时会请求一次 ID 14。再次进入自动模式但不发启动命令且无故障时应保持静默。

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

## 远程/遥控驾驶提示手动验证

重启节点，在另一个终端观察 `/voice_broadcast_id`，第三个终端先发送静止状态并切入远程/遥控模式：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

应立即输出一次 ID 14，保持静止时不应每 60 秒重复。重复发送模式 2 不应重新播报。再确认开始行驶：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

应立即输出 ID 14，此后每约 60 秒一次。重复行驶状态、模式反馈或启动按钮不应额外播报或重置周期；无需发送启动按钮，也无需 D 挡。观察 `/ad_mode_req` 时，远程模式下发送 `mode_sw_cmd=true` 只能得到 false 请求。

发送行驶状态 false 应取消循环；再次发送 true 立即重新开始。离开模式 2 应取消全部 ID 14 请求，再切入时重新开始：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

如要验证切入时已在行驶，两次模式切换前先置 `veh_running_st=true`；切入只应发一条 ID 14。也可不发送任何行驶状态直接切入，此时只应发一次切入提示，不启动循环。

验证 P0 阻塞，可在切入前注入正数故障并确认行驶状态，让 ID 15 或 ID 16 生效。切入提示在不足 10 秒清除故障后应补发，达到 10 秒后清除则丢弃本轮；行驶循环下一轮仍基于原定 60 秒时刻。精确边界和请求合并由可控时间测试覆盖。测试结束发送 `alarm_diag_st=0` 清除故障。

## 低电量提示手动验证

重启节点，在另一个终端观察 `/voice_broadcast_id`。第三个终端清除其他 P0，提供静止和低电量反馈，再进入自动模式：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /battery_low_power_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

未启动时不应有 ID 17。发送启动按钮：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

应立即输出一次 ID 17，约 10 秒后再输出一次，此后不再重复。重复发送启动按钮 true、静止或低电量反馈不应增加次数或改变间隔。若启动时先提供低电量 false，再在行驶前改为 true，也应从反馈完整时刻开始两次提示。

验证启动时两种反馈都未收到：重启节点，仅发送 `veh_mode_st=1` 和 `mode_sw_cmd=true`。即使等待超过 10 秒，也应保留触发窗口，不输出 ID 17。之后分别发送 `battery_low_power_st=true` 和 `veh_running_st=false`，顺序任意；仅收到其中一项时不播报，补齐后立即发送第一次，再过至少 10 秒发送第二次。若先收到 `veh_running_st=true`，则本次窗口关闭，之后才收到低电量或停车反馈也不补触发。

要验证取消，先发停止按钮 false，再发 true 建立新任务，第一次 ID 17 后立即发送行驶状态 true：

```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

第二次 ID 17 应取消。随后发送静止状态不应重新触发；必须停止并重新启动。首次发送后将低电量改为 false，或退出自动模式，也应取消第二次请求：

```bash
ros2 topic pub --once /battery_low_power_st std_msgs/msg/Bool "{data: false}"
```

验证 P0 阻塞，可在静止、低电量且启动前置 `alarm_diag_st=1`。此时 ID 16 优先，低电量任务保留；故障清零后发送第一次 ID 17，再过至少 10 秒发送第二次。第二次到期前再注入故障，则在清除后延迟发送第二次，不集中补发。精确时间边界由可控时间测试覆盖。

## 装卸货提示手动验证

重启节点，在另一个终端观察 `/voice_broadcast_id`。无需发送自动模式或启动按钮。第三个终端在无故障、无其他提示的条件下依次注入到站、N 挡、EPB 状态：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
```

三项齐全后应先输出一次 ID 3，后续调度再输出一次 ID 4；到达顺序可以改变。每条消息是一项独立请求，节点不等待音频播放完成。最后一条命令仅用于测试已授权的临时 EPB 协议，不能作为真实 EPB 已拉起的证明。重复消息、释放再拉起 EPB、换挡后恢复 N 挡均不应重开本次到站机会。

离站后再到站，应重新允许先 ID 3、后 ID 4，各一次：

```bash
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: true}"
```

验证阻塞与超时，可在新到站前先发送 `veh_running_st=false` 和 `alarm_diag_st=1`，使 ID 16 有效，再提供完整停靠条件。条件完整后不足 10 秒清除故障，应依次发送仍未超时的 ID 3、4；达到 10 秒后清除不应补播任何一条，重复完整条件也不重试。若只剩不足一个调度周期，可能只发出 ID 3 而 ID 4 超时。必须先发未到站，再发新到站才重开机会。

验证取消，可在两条 P2 被故障阻塞期间释放 EPB 或退出 N 挡，再恢复条件并清除故障；不应补播本次 ID 3、4。ID 3 已发而 ID 4 尚未发送时失去停靠条件，仅取消 ID 4，不重复 ID 3。驾驶模式或停止按钮变化不取消有效待播请求。遥控模式切入请求 ID 14 与这两条 P2 同时待播时，应依次输出 ID 14、3、4，每次调度一条且两条 P2 均未超时。

## 开锁成功提示手动验证

重启节点，在另一个终端观察 `/voice_broadcast_id`。无需自动模式、启动按钮或停靠条件。第三个终端先发送未锁状态：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

这只是首次状态反馈，不应输出 ID 5。随后依次模拟上锁、开锁：

```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

应仅在 true→false 时请求一次 ID 5。重复发送 false 不重播；再次 true→false 才产生新的 ID 5。以上命令仅模拟锁状态，不执行实际门锁控制。

验证阻塞可先发送静止状态和正数故障，使 ID 16 有效，再模拟 true→false。转换后不足 3 秒清除故障应补发 ID 5，达到 3 秒后清除不应补发；重复 false 不重试。锁状态在等待期间恢复 true 应取消旧请求。命令行与 DDS 有调度延迟，精确边界以可控时间测试为准。

P2 同时待播时应按 5、3、4 顺序请求；需要在测试中让三项在同次调度成为待播，或先用 P0 阻塞并在各自期限内解除。三条不会相互延长期限：ID 5 可在 3 秒时丢弃而到站两条仍有效，也可能较晚的 ID 5 先发出而到站两条随后达到原 10 秒期限被丢弃。

## 装卸货完成确认提示 ID 2 手动验证

重启节点并观察 `/voice_broadcast_id`，无需进入自动模式或提供到站条件。先收到 false 仅初始化，不应产生 ID 5，也不应开始 ID 2 计时。随后模拟真实的上锁→开锁事件：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

最后一条反馈触发一次 ID 5；从该事件起第 60、120、180 秒应分别请求一次 ID 2，之后持续未锁也没有第四轮。中途重复 false 不改变到期时刻，模式、按钮、到站、挡位、EPB 和行驶状态也不取消任务。命令仅模拟状态，不执行门锁控制。

任意时刻重新上锁，应取消本组剩余提醒；再次开锁从新事件重新计时：

```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

验证阻塞时，在某轮到期前发送静止和故障状态，使 ID 16 持续占用：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 1}"
```

在本轮到期后不足 10 秒时清除故障，应发送尚有效的 ID 2；达到 10 秒后清除应丢弃该轮，下一轮仍按开锁后的原定时刻到期：

```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
```

若故障在开锁时已有效并持续超过 3 秒，ID 5 可超时，但 ID 2 仍按该开锁事件的第 60、120、180 秒执行。P1 同时有效时应按 2→14→7 逐次请求，其他请求原期限不变。DDS 与命令行存在调度延迟，精确秒数和超时边界以可控时间测试为准，无需实际等待三分钟。

## 逻辑测试

```bash
cd /home/user/桌面/function/ros2_ws/build/body_control
ctest --output-on-failure
```

`voice_scene_test` 覆盖 P0 场景、自动模式启动逻辑及故障取值；`voice_scheduler_test` 使用传入的单调时钟时间点，覆盖首次立即发送、120 秒周期、重复状态、P0 的 5 秒周期和切换、10 秒等待边界、固定周期、延迟轮次跳过、停止及重新启用，并覆盖行驶及静止故障输入顺序、等级变化、模式和按钮独立性、故障抢占与恢复、ID 15/16 双向切换和重新计时、故障期间的 P1 超时以及停车时取消 P1 待播。

`remote_voice_test` 覆盖静止/行驶/未知状态切入、60 秒边界、重复输入、停止和重新行驶、模式退出及重进、按钮独立性、现有 P0 启用范围、P0 阻塞和 10 秒期限、切入和循环请求的独立超时及合并、停车只取消循环、延迟轮次跳过，以及自动和远程模式切换。全部测试采用可控时间，无需真实等待一或两分钟，不验证 DDS 通信、实际定时器抖动或硬件执行。

`low_battery_voice_test` 覆盖启动前和启动后反馈、未知行驶状态、有效自动启动、最多两次及 10 秒边界、重复消息、开始行驶/停止/模式退出/低电量解除取消、取消后不重开及新发车重新触发、现有 P0 优先和无超时等待、首次实际发送后重新计算间隔、第二次延迟、调度迟到不集中补发，以及取消低电量任务后 ID 7/14 正常执行。

`cargo_voice_test` 覆盖三个条件的全部到达顺序、缺少任一条件、启动即完整停靠、所有模式及未启动触发、每次到站先 3 后 4 且各一次、重复状态及抖动、超时和取消后不重试、离站后重开、P0/P1/低电量优先、10 秒边界、模式和按钮不取消，以及旧 ID 7 待播取消后不阻塞 N 挡到站任务。额外覆盖 ID 3 已发后 ID 4 受 P0/P1 阻塞、ID 4 保持共同触发时刻且可单独超时或被取消、ID 3 不重发，以及新到站从 3 开始重建请求。精确时间用可控单调时钟测试，不需真实等待。

`unlock_voice_test` 覆盖首次 true/false 初始化、未接收状态不构造事件、有效 true→false 和重复 false、多次开锁事件、P0/P1/低电量优先、3 秒边界、超时后不重试、重新上锁取消及新事件独立期限、所有模式和按钮独立性、到站/挡位/EPB 变化不取消，以及 P2 顺序 5→3→4 和三条请求的独立期限。测试采用传入的单调时钟时间，不需实际等待。

`cargo_completion_voice_test` 使用可控时间覆盖首次状态不误触发、统一开锁事件、ID 5 超时独立性、60/120/180 秒边界及最多三轮、重复反馈不重置、所有 P0 和低电量任务阻塞、10 秒丢弃边界和固定时间轴、迟到检查跳过历史轮次、重新上锁取消及再开锁重建、其他车辆状态不取消、P1 顺序 2→14→7、各请求独立期限，以及 P1 优先于到站 P2。无需真实等待三分钟。
