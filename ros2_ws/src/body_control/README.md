# 车身控制节点

`body_control_node` 实现危险警报灯、制动灯/高位制动灯、转向灯和倒车灯控制请求、自动驾驶启动请求，以及等红绿灯、车辆起步、箱门未关闭、装卸货完成确认、开锁成功、装卸货和关闭箱门提醒、低电量、远程/遥控驾驶、行驶故障、故障停车、自动驾驶前进、障碍物、左右转、倒车、行人礼让语音请求。节点不负责底盘模式仲裁、门锁控制、路径绕行、靠边停车、刹车或播放音频。

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
| 输入 | `/traffic_light_wait_st` | `std_msgs/msg/Bool` | 临时授权等灯接口：false 未等灯、true 正在等红绿灯 |
| 输入 | `/ambient_light_level_st` | `std_msgs/msg/Float64` | 文档已有环境光照数值；单位未定义，阈值 5 为用户暂定 |
| 输入 | `/veh_speed_st` | `std_msgs/msg/Float32` | 临时授权车速接口，单位 km/h；名称、类型及单位待确认 |
| 输入 | `/veh_running_st` | `std_msgs/msg/Bool` | false 静止、true 行驶 |
| 输入 | `/veh_brake_st` | `std_msgs/msg/Bool` | 文档已有车辆制动状态：false 解除、true 生效 |
| 输入 | `/emergency_stop_st` | `std_msgs/msg/Bool` | 文档未定义，经用户授权暂定：false 解除、true 急停有效，默认 false |
| 输入 | `/mode_transition_abnormal_st` | `std_msgs/msg/Bool` | 文档未定义，经用户授权暂定：false 解除、true 模式异常切换有效，默认 false |
| 输入 | `/pedestrian_cross_st` | `std_msgs/msg/Bool` | true 检测到行人横穿 |
| 输出 | `/ad_mode_req` | `std_msgs/msg/Bool` | true 自动驾驶模式请求、false 退出请求 |
| 输出 | `/reverse_light_req` | `std_msgs/msg/Bool` | 文档已有倒车灯请求：false 关闭、true 开启 |
| 输出 | `/turn_light_req` | `std_msgs/msg/Int32` | 文档已有转向灯请求：0 关闭、1 左转向、2 右转向；下游执行闪烁 |
| 输出 | `/brake_light_req` | `std_msgs/msg/Bool` | 文档已有制动灯请求：false 关闭、true 开启；下游同步控制制动灯和高位制动灯 |
| 输出 | `/hazard_light_req` | `std_msgs/msg/Bool` | 文档已有危险警报灯请求：false 关闭、true 请求双侧同步高频闪烁 |
| 输出 | `/voice_broadcast_id` | `std_msgs/msg/Int32` | 1 箱门未关闭、2 装卸货完成确认、3 装卸货、4 提醒关闭箱门、5 开锁成功、6 车辆起步、7 自动驾驶前进、8 障碍物、9 右转、10 左转、11 行人礼让、12 倒车、13 等红绿灯、14 远程/遥控驾驶、15 行驶故障、16 故障停车、17 低电量 |

规范中 `ad_mode_req` 是 0/1 枚举，本示例按当前需求用 Bool 承载 false/true。接入实际车辆时需和接收端约定一致的消息类型。

**临时 EPB 协议：`veh_epb_st` 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。** 字段声明和订阅处均有相同标识。节点仅接收外部 EPB 状态，不自动生成，不以普通制动 `veh_brake_st` 代替 EPB；实际对接前需确认该临时协议。

**临时车速协议：`veh_speed_st` 文档未定义，经用户授权暂定，名称、类型及单位待后续确认。** 字段声明和订阅处有相同标识；节点接收外部车速，不从行驶状态计算车速。

**临时等灯协议：`traffic_light_wait_st` 文档未定义，经用户授权暂定，名称、类型及取值待后续确认。** 声明、订阅和说明保留授权标识，不以普通停车代替等灯状态。

**光照判定：阈值由用户暂定，待后续文档同步；文档未定义光照单位。** 使用单阈值 5，≤5 为夜间，>5 为白天，替代此前讨论的 3/7 回差方案。

## 自动驾驶启动顺序

1. 接收车辆模式反馈 `veh_mode_st=1`，此时等待启动，不播报。
2. 随后接收 `mode_sw_cmd=true`，先发布 `ad_mode_req=true`，再启用语音场景判断。
3. 相同模式反馈、重复启动命令不会重新触发当前语音或重置播报周期。
4. 收到 `mode_sw_cmd=false` 或模式切换，发布 `ad_mode_req=false`，停止依赖自动启动授权的提示并清除 ID 7 周期和待播请求。有效的故障提示 ID 15、16 不受此限制，保持原周期。
5. 重新进入自动模式后，必须再次收到启动命令。非自动模式收到的启动命令不会缓存为之后的启动授权。

模式变化时发布一次 false，启动按钮每次更新时发布当前请求值。自动驾驶相关语音使用模式反馈和按钮授权作为开关，ID 2、3、4、5、14 和故障提示另有启用条件；请求发布不等于底盘执行成功。不同输出话题间不保证接收顺序，但节点内先调用请求发布，再处理语音。远程/遥控模式下按钮 true 不会产生 `ad_mode_req=true`。

## 语音触发逻辑

ID 1、6、7、8、9、10、11、12、13、17 统一调用 `automatic_voice_enabled(state)`：只有 `veh_mode_st==1` 且 `auto_drive_enabled=true` 才能进入各自场景判断。`auto_drive_enabled` 是由节点从 `AutoDriveGate::enabled()` 更新的内部有效启动授权，不是底盘接管成功反馈。待机、遥控及未知模式即使与授权 true 矛盾，也不放行自动相关语音。

ID 2、3、4、5、14、15、16 保留独立启用条件；故障判断在公共开关之前。各任务仍先更新触发、取消和超时再仲裁，不在总调度器或发布层统一拦截非自动模式。

- 自动模式已有效启动，且已收到 `trunk_lock_st=false`：ID 1，箱门未关闭，请关闭箱门，P1；条件完整后等待 10 秒首轮，此后每 10 秒一轮。未锁仅近似箱门未关闭，不能确认实际开闭。
- `veh_arrive_st=true`、`veh_gear_st=2`（N 挡）且 `veh_epb_st=true`：ID 3，车辆已到达目的地请及时装卸货，P2，每次到站最多请求一次，不限制模式和启动按钮。
- 相同到站条件同时建立 ID 4：装、卸货完成后请关闭箱门，谢谢，P2，每次到站最多请求一次。先请求 ID 3，后续调度再请求 ID 4，不额外判断装卸完成、箱门开闭或门锁状态。
- 已收到的 `trunk_lock_st` 从 true 变为 false：从事件时刻起第 60、120、180 秒各请求一轮 ID 2，关闭箱门前，请确认货物是否已全部装、卸完成，P1；不限制模式及其他车辆状态，重新上锁取消剩余轮次。
- 已收到的 `trunk_lock_st` 从 true 变为 false：ID 5，开锁成功，P2，每次有效转换最多一次；首次状态消息只初始化，不播报。
- `alarm_diag_st>0` 且已收到 `veh_running_st=true`：ID 15，车辆故障，正在靠边停车，请注意避让。任何模式及未按启动按钮时均可触发；首次立即发送，此后每 5 秒一次。
- `alarm_diag_st>0` 且已收到 `veh_running_st=false`：ID 16，车辆故障，请勿靠近，并请绕行。任何模式及未按启动按钮时均可触发；首次立即发送，此后每 5 秒一次。
- 自动模式已启动、`traffic_light_wait_st=true`、已确认静止且光照已知：ID 13，车辆等待中，请勿靠近，P1；白天立即首轮并每 10 秒循环，夜间按单次规则。
- 切入 `veh_mode_st=2`：一次 ID 14，远程/遥控驾驶中，请注意避让；该模式下确认开始行驶时立即请求 ID 14，此后每 60 秒一轮，优先级 P1。切入时已在行驶，两种请求合并为一次。
- 自动模式已有效启动，已知行驶状态从 false→true，且起步后不足 3 秒内新收到的车速满足 `0≤veh_speed_st<5` km/h：ID 6，车辆起步，请注意安全，P1，每次起步最多一次。
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

故障提示 ID 15、16、倒车、障碍物、礼让、左右转和低电量在规范中均为 P0，规范没有规定它们之间的顺序；按用户确认的策略采用故障 > 倒车 > 障碍物 > 礼让 > 左右转 > 低电量。ID 15、16 根据行驶状态互斥选择。高优先场景结束后按当前状态恢复其他 P0 提示；如无 P0 及有效低电量任务，则将已到期且未超时的 P1 请求按 ID 1→6→13→2→14→7 调度，最后按用户确认的 P2 顺序 5→3→4 逐次调度。每条语音消息表示一次新的播放请求，下游不能只检测 ID 数值变化。新增 ID 14 不扩大其他 P0 在遥控模式下的启用范围，倒车、障碍物、礼让和转弯仍要求自动模式已启动。

### ID 6 的起步事件与 3 秒期限

- 仅自动模式且已有效启动时启用，沿用有效启动近似表示 APP／小程序发车，无法识别实际点击来源。复用已有 `veh_running_st`，并使用已授权的临时车速接口；不额外要求挡位、到站或 EPB。
- 以已确认静止→行驶的转换作为起步事件。首次收到 true 仅初始化；授权建立时已行驶不补触发，授权外起步不缓存。授权建立时已静止则允许随后起步。每次停车后再起步可重新触发，持续行驶及重复输入不重试。
- 从起步边沿的接收处理时刻起保留候选任务，只使用该边沿之后新收到的车速反馈，且仍行驶、车速有效并满足 `0≤veh_speed_st<5` km/h 时才可发送。旧高速不取消本次候选，旧低速不提前触发；每次起步都必须重新收到反馈。总期限仍为 3 秒，达到即取消，不因车速补齐或阻塞延长。
- 每条车速消息递增内部接收序号，包括数值相同或非法的反馈；起步时记录当前序号，只有之后序号变化才认为收到新反馈。未收到新反馈不使用缓存或默认零值。负数、NaN、正负无穷均视为未知，后续有效新反馈可在原期限内补齐；非法反馈不能继续沿用旧低速。该机制仅识别本节点接收顺序，不新增全局有效期，也无法识别上游真实采集时间或传输延迟。
- 起步后新反馈确认车速≥5 km/h、重新停车、停止或退出自动模式立即取消未发请求；同次行驶中减速不重新触发。已经发送的请求不撤回。
- 有效 P0 及低电量任务优先，P0 整段有效期间不穿插。P1 顺序 1→6→13→2→14→7，P2 保持 5→3→4。等待车速不阻塞其他已就绪请求，也不延迟 ID 7；同时就绪时先请求 ID 6。每次调度最多一条，各自期限不变。
- 内部接收标记、前值、候选时间仅用于调度。此功能不控制车辆或门锁；节点保证请求顺序，实际播放排队由下游负责。

### ID 13 的等灯任务、昼夜切换与超时

- 同时满足自动模式、有效启动、`traffic_light_wait_st=true`、已收到行驶状态且为静止时，进入一次有效等待任务。未知光照不播报，之后补齐再执行对应规则。不额外要求挡位、到站或 EPB，也不从停车状态推断等灯原因。
- 使用文档已有的光照数值：≤5 为夜间，>5 为白天。**阈值由用户暂定，待后续文档同步；文档未定义光照单位。** 负数、NaN、无穷值忽略，保留上次有效昼夜判断；若从未收到有效值则继续未知。不新增光照反馈过期判断。
- 白天条件完整后立即首轮，此后每 10 秒一轮。重复状态不重置；每轮按原定到期时刻计算 3 秒期限，达到即丢弃。延迟或丢弃不改变下轮时间轴，迟到检查跳过过期历史轮次，不集中补发。
- 夜间每次等待最多一个请求机会。若本次已实际发出过 ID 13，则入夜不再发；否则建立一次待播请求，超时不重试。建立请求即使用夜间机会，光照反复切换不重开。
- 白天转夜间停止循环；未发且已有有效待播请求时沿用其原到期时刻，不延长 3 秒期限。没有待播请求且夜间机会未使用时，从切入时刻建立一次请求；白天旧轮已超时也不借用其旧期限。
- 夜间恢复白天时，取消旧阶段未发请求，立即建立白天首轮并重新计算 10 秒循环。再次入夜仍受“本次已发过”和夜间机会限制。
- 停止、退出自动模式、等灯状态解除或开始行驶，立即取消周期及待播并结束本次任务；再次条件完整可建立新任务及新夜间机会。光照的最后有效判断跨任务保留。
- 有效 P0 和低电量任务优先；P0 整段有效期间不在请求间隙穿插。P1 顺序 1→6→13→2→14→7，P2 保持 5→3→4；每次调度最多一条，各自期限不延长。阻塞期间也处理昼夜变化、取消和超时。
- 等灯判断仅发布语音请求，不控制车辆或信号灯，不推断灯色。节点保证请求顺序，实际音频排队由下游负责；接收、周期、已发送及夜间机会标记均为内部状态。

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
- 按已确认的请求级仲裁，所有有效 P0 及低电量任务优先，已到期且未超时的 ID 1/6/13/2/14/7 请求先发送。没有更高优先请求时按 3、4 顺序调度，一次最多一条。
- 没有播放完成反馈，所以只能保证请求调度顺序，实际音频排队由下游负责。ID 14 或 ID 3 请求发出后，下一次调度可发送尚未超时的下一条 P2 请求，不把请求发出当作播放完成。其他请求被优先调度不会重置 ID 3、4 的 10 秒期限。
- 到站任务不触发 ID 1、2，不控制挡位、EPB、车门或车辆运动。ID 2 和 ID 5 由统一识别的锁状态事件触发，不要求到站。尽管 ID 4 文案提到装卸完成，其触发条件严格沿用文档中的到站、N 挡和 EPB，不引入装卸完成或箱门信号。

### ID 1 的箱门未关闭提醒

- 复用已有模式、有效启动授权、`trunk_lock_st` 和接收标记，不新增业务信号、ROS 话题或消息字段。经用户确认，将自动模式有效启动近似表示 APP／小程序发车，将已收到未锁近似表示箱门未关闭。**未锁不等于门未关闭，现有接口无法确认箱门实际开闭或 APP 点击来源。**
- 自动模式已有效启动且确认未锁时，从条件完整时刻等待 10 秒后首次请求，此后每 10 秒一轮。启动时未知锁状态只等待；之后收到未锁再开始计时。首次锁反馈 false 可建立 ID 1 周期，但不构造 ID 2/5 的开锁事件。
- 重复模式、启动及未锁反馈不重新计时。上锁、停止按钮或退出自动模式立即取消周期和待播请求；同次授权期间再次未锁时重新建立周期，仍等待 10 秒首轮。行驶、到站、挡位和 EPB 变化不取消或重置。
- 每轮以原定到期时刻计算 3 秒期限，达到期限即丢弃；延迟发送或丢弃不改变后续 10 秒时间轴。调度迟到时跳过超时历史轮次，不补偿、不集中补发；没有有效待播请求时，阻塞解除不额外补播。
- 现有有效 P0 和低电量任务优先，整个 P0 有效期间不穿插 ID 1。P1 按 1→6→13→2→14→7，P2 按 5→3→4，每次调度最多一条，各自期限独立。阻塞期间也处理上锁、停止、模式退出及超时。
- 仅发布语音请求，不控制门锁、制动或自动驾驶请求，不以箱门提醒延迟发车。没有播放反馈，实际音频排队仍由下游负责。

### ID 2 的三轮装卸货完成确认提醒

- 复用现有 `/trunk_lock_st` 订阅及接收标记，与 ID 5 统一识别已接收锁状态的 true→false 事件。首次 true/false 仅初始化，重复 false 不触发、不重置。用锁状态变化近似表示开锁，不能识别 APP 点击来源；不新增业务信号、话题或消息字段。
- 以开锁事件的接收处理时刻为起点，在第 60、120、180 秒各产生一轮 ID 2 请求，没有立即提醒或第四轮。计时不依赖 ID 5 是否成功发送；ID 5 被阻塞、取消或超时不改变此时间轴。
- 每轮以原定到期时刻计算 10 秒期限，达到期限即丢弃，超时也计入三轮。第一轮若在第 69 秒发送，第二轮仍在第 120 秒到期。调度迟到会跳过已经超时的历史轮次，只发送仍有效的请求，不补偿或集中补发。
- 重新上锁立即取消待播请求及全部后续轮次；再次真正开锁从新事件重新等待 60 秒。模式、启动按钮、到站、挡位、EPB 或行驶状态变化均不取消任务，不额外要求这些条件。
- 有效 P0 场景及低电量有限次数任务优先；P0 整段有效期间不穿插 ID 2。P1 按 ID 1→6→13→2→14→7 调度，P2 保持 ID 5→3→4，每次调度最多一条。其他请求的发送不重置各自期限，阻塞期间也处理到期、超时及重新上锁取消。
- 只请求“关闭箱门前，请确认货物是否已全部装、卸完成”，不判断实际装卸完成，不控制门锁，也不撤回已发请求。节点保证请求调度顺序，实际音频排队由下游负责。

### ID 5 的锁状态事件与超时

- 仅新增文档已有的 `/trunk_lock_st` 输入，不新增 APP 开锁指令或其他文档外业务信号。内部接收标记、前一次锁状态及待播时间不是新增业务接口。
- 经用户选择方案 1，以已接收状态从 true（上锁）变为 false（未锁）近似表示开锁成功，不能识别 APP/小程序点击来源。首次收到 true 或 false 均只初始化，不触发；未收到消息时也不使用默认值构造事件。
- 每次 true→false 建立一次 ID 5 请求，并用同一事件启动独立的 ID 2 三轮计时，重复 false 不重新触发或延长期限。持续未锁不会循环播报。不同于到站任务，锁状态事件不受模式、启动按钮、到站、挡位或 EPB 条件限制。
- 从转换的接收处理时刻计算等待时间，达到 3 秒即丢弃，边界不补播。等待期间重新上锁取消旧请求；再次真实发生 true→false 时才建立新请求并使用新事件的时刻。
- 有效 P0、低电量任务、已到期且未超时的 P1 请求优先；P2 顺序为 ID 5→3→4，每次调度最多一条。发送其他请求不会延长 ID 5 的 3 秒期限，发送 ID 5 也不延长到站两条的原 10 秒期限。
- 不控制门锁，不撤回已发请求。没有音频完成反馈，因此仍只保证请求调度顺序，实际音频排队由下游负责。

节点用一个 50ms 定时器及状态回调共同检查单调时钟到期时间，P0 的 5 秒周期、自动前进的 120 秒周期、远程/遥控行驶的 60 秒周期、低电量的有限次数任务、ID 1 的 10 秒周期、ID 2 的固定三轮任务、ID 6 的单次起步任务、ID 13 的昼夜等待任务和到站装卸货的一次性任务分别管理。窗口、计数和计时字段仅属于内部实现；业务接口中另有上述经用户授权、待确认的临时 EPB、车速及等灯信号。

倒车语音和倒车灯复用文档已有的 `veh_gear_st` 输入，倒车灯输出使用文档已有的 `reverse_light_req`，没有新增文档外的信号接口。后续如需新增文档未定义的业务信号，必须先获得用户同意，并在代码中注明。

## 倒车灯控制

- 按用户确认的条件，`veh_gear_st=1`（R 挡）即输出 `reverse_light_req=true`，挂 R 挡但静止时也持续点亮。P/N/D、负数及其他非法挡位均输出 false。
- 不依赖模式、启动按钮、行驶状态或故障语音，自动、遥控及待机均按挡位控制。倒车语音仍遵守原来的自动模式启动条件。
- 启动时未收到挡位先发布 false；挡位回调更新后先发布灯光请求，再处理语音，不等待周期调度。复用现有 50ms 定时器刷新当前请求，QoS 为 reliable、深度 1、volatile，晚连接的接收端通过周期刷新获得状态。
- 不新增业务状态变量或配置参数。通信中断时保留最后收到的挡位，不增加未约定的输入超时规则。这里只输出灯光请求，实灯执行、硬件故障诊断和亮度控制由对应硬件接口实现。

### 倒车灯验证

按下一节编译并运行节点，在另一个已加载 ROS 环境的终端观察：

```bash
ros2 topic echo /reverse_light_req
```

在第三个已加载 ROS 环境的终端依次输入，预期挂 R 挡为 true，退出到 N 挡为 false；无需按启动按钮或提供行驶状态：

```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 2}"
```

自动验证（编译时启用 `BUILD_TESTING=ON`）：

```bash
cd /home/user/桌面/function/ros2_ws
colcon test --packages-select body_control --event-handlers console_direct+
colcon test-result --verbose
```

`reverse_light_test` 在独立命名空间启动实际节点，检查默认关闭、挡位切换、非法值、周期刷新、输出 QoS 及模式/启动/静止/故障组合。测试测量输入发送到输出接收的耗时并要求不超过 100ms，该测量包含双向传输，是本机节点响应的保守上界；不代表实车硬件响应或最坏调度延迟的保证。台架验收还需测量输入到达节点至实际灯光请求的响应及实灯效果。

目前未实现输入超时、自动驾驶请求的自检和故障准入校验、模式请求应答超时及重试、音量请求和音频中断指令。故障播报不会自动拒绝自动驾驶启动请求。输入保留最后状态，通信中断不会自动清除授权、故障或行驶状态。停止发送请求不等于立即中断下游正在播放的音频。

## 转向灯控制

- 复用已有 `veh_mode_st` 和 `veh_turn_st`。仅模式 1（自动）、2（遥控）启用，转弯状态 1/2 请求左/右转向；直行 0、非法转弯值、待机及未知模式均请求关闭。初始化输出 0，模式恢复时使用当前缓存的转弯状态，左右方向直接切换，不插入关闭间隔。
- 不依赖启动按钮、行驶状态或语音调度。模式和转弯回调先更新灯光请求，再处理语音；现有 50ms 定时器持续刷新，QoS 为 reliable、深度 1、volatile。通信中断保留缓存，没有新增输入超时规则。
- **用户确认的近似实现：收到转弯状态立即请求，收到直行状态关闭。** `veh_turn_st` 是车辆转弯状态，不能独立确认规划意图、人工转向指令或车身回正，因此无法保证文档要求的提前 1 秒开启及独立回正判定。没有新增文档外业务变量、订阅或配置参数。
- **下游接口约定：持续的 1/2 表示对应方向闪烁请求，不表示单侧常亮。** BCM/灯控负责实际均匀闪烁和接收 0 后关闭；本节点不交替发送方向和 0，也不定义新的闪烁频率。硬件故障、闪烁频率及实灯响应需在台架验证。

### 转向灯验证

按下一节编译运行节点，在另一个已加载 ROS 环境的终端观察：

```bash
ros2 topic echo /turn_light_req
```

在第三个已加载 ROS 环境的终端逐条执行，预期依次为自动模式无方向时 0、左转 1、右转 2、直行 0、待机 0：

```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 0}"
```

上述测试假定初始没有转弯输入，也没有其他程序发布同名输入。改为模式 2 可验证遥控，无需启动按钮。保持左转时切入待机，预期关闭；再切回模式 1/2，预期恢复左转请求。持续的方向输出应由下游转换成实灯闪烁。

编译启用 `BUILD_TESTING=ON` 后，运行前述 `colcon test` 命令执行全部测试。新增 `turn_light_test` 在独立命名空间启动实际节点，覆盖初始关闭、左右直接切换、非法值、重复输入、50ms 刷新、模式退出/恢复、两种输入到达顺序以及启动/静止/故障组合，并检查可靠传输和 volatile QoS。只对输出实际改变的事件测量输入发送到请求接收耗时，要求本机测试不超过 100ms；不以重复状态的周期消息充当输入响应。此耗时包含双向传输，不代表最坏调度或实灯响应保证。

## 制动灯与高位制动灯控制

- 直接跟随文档已有的 `veh_brake_st`：true 持续请求点亮，false 请求关闭。**按用户确认，解除即熄灭，包括车辆仍在行驶时，不额外要求确认静止。** 未收到制动反馈时默认关闭。
- 上游须在减速、驻车、坡道驻车和紧急制动需要亮灯期间持续提供 true；坡道长时间驻车保持亮灯依赖这一约定。本节点不从车速推断减速、不合并临时 EPB，不受模式、启动按钮、挡位、行驶状态或语音优先级影响。
- 输入采用 best_effort、深度 1；制动回调更新缓存后立即发布请求，不等待语音或周期调度。初始化发布 false，复用 50ms 定时器持续刷新，输出采用 reliable、深度 1、volatile。通信中断保留最后制动状态，无新增超时策略或文档外业务变量。
- `brake_light_req` 是两组灯的统一请求，由下游同步控制制动灯和高位制动灯。本节点不直接制动车辆；硬件故障诊断、两组实灯同步和实灯响应由下游实现并在台架验收。

### 制动灯验证

按下一节编译运行节点，在另一个已加载 ROS 环境的终端观察：

```bash
ros2 topic echo /brake_light_req
```

在第三个已加载 ROS 环境的终端逐条执行，预期初始 false、制动 true、行驶状态变化后仍为 true、制动解除后即为 false：

```bash
ros2 topic pub --once /veh_brake_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_brake_st std_msgs/msg/Bool "{data: false}"
```

无需发送模式或启动按钮。测试时应没有其他程序发布同名输入。保持制动状态 true 可验证持续点亮请求；EPB 拉起本身不会点亮制动灯。

启用 `BUILD_TESTING=ON` 编译后，运行前述 `colcon test` 命令。新增 `brake_light_test` 在独立命名空间启动实际节点，覆盖默认关闭、制动开启/解除、重复输入、缓存保持、50ms 刷新及模式/启动/行驶/挡位/EPB/故障组合；检查输入 best_effort 和输出 reliable、volatile QoS。只对输出实际改变的事件测量输入发送到请求接收耗时，要求本机测试不超过 100ms；该耗时包含双向传输，不代表最坏调度或硬件实灯响应保证。原有灯光和语音测试一并回归。

## 危险警报灯控制

以下六类条件任一成立即持续请求双闪，全部解除才关闭，不锁存、不依赖启动按钮：

| 场景 | 判定条件 |
| --- | --- |
| 急停 | `emergency_stop_st=true` |
| 系统故障驻车 | `alarm_diag_st>0` 且已确认静止 |
| 长时间遇障等待 | `obs_detect_st=true` 且已确认静止，连续满 30 秒 |
| 遥控接管 | `veh_mode_st=2`，包括静止和未按启动按钮 |
| 模式异常切换 | `mode_transition_abnormal_st=true` |
| 低电量停靠 | `battery_low_power_st=true` 且已确认静止 |

“已确认静止”须收到过 `veh_running_st=false`；未知状态不按静止处理，故障值 0 或负数不触发故障驻车。其他场景不限制模式，遥控模式退出只解除遥控原因，不能覆盖仍有效的急停等原因。

**文档未定义，经用户授权暂定：`emergency_stop_st` 和 `mode_transition_abnormal_st` 均为 Bool 输入，默认 false。** true 分别表示急停有效、模式异常切换有效，false 表示解除，由上游负责置位和解除；声明、订阅处保留授权标识。普通制动不是急停，未知模式值和普通模式切换也不自动视为异常。

**用户确认的近似：故障静止、遇障静止和低电量静止不能证明停车原因、靠边位置或实际驻车动作。** 不从 EPB 或到站信号推断这些场景。

**文档未规定遇障时长，经用户确认暂定 30 秒。** 从障碍物与已确认静止条件同时成立的本节点处理时刻，以单调时钟计时；重复消息、模式切换或其他双闪原因不会重置计时。障碍物消失或开始行驶立即清零，重新满足后重新计时。内部计时起点及有效性标记不作为业务接口发布；通信中断保留缓存，不增加超时规则。

相关状态回调立即更新请求，先于语音调度；复用 50ms 定时器检查遇障期限并刷新输出，初始化为 false。两个新增输入采用 best_effort、深度 1，输出采用 reliable、深度 1、volatile。

**下游 BCM 负责双侧同步高频闪烁和双闪优先仲裁。** 节点持续输出 true，不交替输出明暗，不改变 `turn_light_req`；双闪有效时下游覆盖单侧转向，解除后按当前转向请求恢复。不同话题间不保证到达顺序。本次不新增扬声器故障、灯具诊断或车辆停车控制功能；频率、同步、仲裁及硬件响应需台架验收。

### 危险警报灯验证

编译运行节点后，在另一个已加载 ROS 环境的终端观察：

```bash
ros2 topic echo /hazard_light_req
```

在第三个已加载 ROS 环境的终端逐条执行，预期急停开启为 true，叠加异常后仍为 true，解除急停后仍为 true，最后解除异常才为 false：

```bash
ros2 topic pub --once /emergency_stop_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mode_transition_abnormal_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /emergency_stop_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /mode_transition_abnormal_st std_msgs/msg/Bool "{data: false}"
```

确认其他原因均已解除、没有其他程序发布同名输入，再发送以下消息：预期立即仍为 false，连续等待满 30 秒后由定时器自动变为 true；障碍物消失后立即 false。开始行驶也会清零计时。

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: true}"
# 等待30秒，再清除障碍物。
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: false}"
```

自动测试沿用前述 `colcon test` 命令。`hazard_light_test` 使用可控时间覆盖 29999/30000ms 边界、重复输入、中断重计时、迟到检查、全部 64 种原因组合及逐项解除。`hazard_light_ros_test` 启动实际节点验证新接口及 QoS、周期刷新、即时触发响应、真实 30 秒到期、未知行驶状态、遥控静止保持、启动按钮无关性和单侧转向请求保留。仅对即时输出变化测量收发耗时并要求本机不超过 100ms，不代表最坏调度或实灯响应保证；其他灯光和语音测试一并回归。

## 编译和运行

```bash
source /opt/ros/humble/setup.bash
cd /home/user/桌面/function/ros2_ws
colcon build --packages-select body_control --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
source install/setup.bash
ros2 run body_control body_control_node
```

## 手动验证

### 通用准备

#### 1. 确认环境和工作目录

示例使用已安装 ROS 2 Humble 的 Bash 终端，工作空间路径为 `/home/user/桌面/function/ros2_ws`。如果项目放在其他位置，请将下面所有工作空间路径替换为实际路径。

打开三个终端，整个验证过程固定按下表分工：

| 终端 | 用途 | 运行期间的状态 |
| --- | --- | --- |
| 终端 1 | 编译、启动节点；查看自动驾驶请求和语音日志 | 节点持续运行，不在这里输入状态消息 |
| 终端 2 | 观察 `/voice_broadcast_id` 的输出 | `ros2 topic echo` 持续运行 |
| 终端 3 | 按各 ID 步骤发送模拟输入 | 每条命令执行完成后，继续输入下一条 |

这些示例以只有当前测试节点接收、处理模拟状态为前提。若其他程序持续发布同名输入，会覆盖手动输入，结果可能不同。

#### 2. 终端 1：编译并启动节点

首次验证或代码改动后，在**终端 1**执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/桌面/function/ros2_ws
colcon build --packages-select body_control --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
source /home/user/桌面/function/ros2_ws/install/setup.bash
ros2 run body_control body_control_node
```

先确认编译结果没有 `Failed`。最后一条命令运行后，终端不会回到输入提示符，这是节点正在运行的正常表现；尚未输入状态时，没有语音日志也正常。

后面各节要求“重启节点”时，**只在终端 1**按 `Ctrl+C` 停止当前节点，再执行：

```bash
ros2 run body_control body_control_node
```

不改代码时无需重新编译。若关闭过终端 1，重新打开后须先执行本节的两条 `source` 命令，再运行节点。不要在另一个终端额外启动第二个同名节点。

#### 3. 终端 2：持续观察语音请求

在**终端 2**执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/桌面/function/ros2_ws/install/setup.bash
ros2 topic echo /voice_broadcast_id
```

#### 4. 终端 3：加载环境并确认节点在线

在**终端 3**执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/桌面/function/ros2_ws/install/setup.bash
```

#### 5. 每个 ID 从干净状态开始

**每次切换到新 ID，或者重新做一组独立测试，都要先在终端 1 重启节点。** 节点会保存已收到的状态和任务计时，单纯停止发送消息不会自动清空它们。终端 2 保持监听，终端 3 的环境无需重复加载。

各节只发送验证该 ID 所需的状态。不要把上一个 ID 的输入一起重发。例如故障提示会抢占其他语音，低电量任务也会让其他提示等待。重启后不要主动给所有话题发布“默认值”：有些测试专门验证“尚未收到反馈”的情况。

#### 6. 如何判断通过

- 按本节顺序核对：**触发前是否无输出、触发时是否出现目标 ID、周期/次数是否正确、取消后是否停止新增请求**。已经打印出来的历史行不会消失。
- “立即”指条件齐全后节点随回调或约 50ms 调度检查发出请求，不表示音频已经播放完成。
- 时间从节点收到最后一项必要条件开始计算；文中的等待时间均为近似值。CLI 和 DDS 有延迟，不宜靠手动操作判定恰好 3 秒或 10 秒的边界。
- `/veh_epb_st`、`/veh_speed_st`、`/traffic_light_wait_st` 是已授权的临时接口；光照阈值 5 也是用户暂定值，正式含义和限制见前文接口说明。
- 精确超时、抢占和取消边界请使用本文末尾的“逻辑测试”；本节先验证最容易观察的正常触发、重复和结束行为。

### ID 1：箱门未关闭

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**进入自动模式、启动，并发送首次未锁反馈。


```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**第 2 步｜观察：**终端 1 应出现 `ad_mode_req=true`；终端 2 不应立即出现 ID 1，约 10 秒后出现 `data: 1`，之后每约 10 秒一次。第一次锁反馈为 false 只初始化，因此不会产生 ID 5 或 ID 2。这里以“未锁”近似“箱门未关闭”，无法确认实际开闭。

**第 3 步｜终端 3：**在已经看到 ID 1 后，再发送一次相同未锁状态。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**观察：**不应额外立即出现 ID 1，下一轮仍沿用原来的时间，不从这条重复消息重新等待 10 秒。

**第 4 步｜终端 3：**发送上锁状态。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 在之后至少 12 秒内不再新增 ID 1。同次启动授权内再次发送 false，会立即产生一次 ID 5、启动 ID 2 的延时任务，同时 ID 1 重新等待 10 秒；这些关联 ID 不表示测试失败。停止按钮 false 或退出自动模式也会取消 ID 1。


### ID 2：装卸货完成确认

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**只发送首次未锁状态，不进入自动模式。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**观察：**终端 2 无输出。首次状态不是一次开锁事件，不触发 ID 5，也不启动 ID 2 计时。

**第 2 步｜终端 3：**依次模拟上锁、开锁。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**第 3 步｜观察：**终端 2 先立即出现一次 `data: 5`，这是正常的开锁提示。从最后一条开锁反馈起持续观察：

| 开锁后时间 | 终端 2 的预期新增输出 |
| --- | --- |
| 约 60 秒 | 第一次 `data: 2` |
| 约 120 秒 | 第二次 `data: 2` |
| 约 180 秒 | 第三次 `data: 2` |
| 到约 240 秒 | 不出现第四次 ID 2 |

三个时间点都相对于开锁事件，不是相对于上一次实际输出。重复发送 false 不重新计时。手动完整观察需要约 4 分钟；无需等待的精确验证见“逻辑测试”。

**第 4 步｜终端 3：**如需另外验证取消，先再次上锁、开锁，等待任意短于 60 秒的时间，再执行最后一条上锁命令。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

在尚未到 60 秒时执行：


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
```

**观察：**这次新开锁可能产生 ID 5，但后续不得出现这组任务的 ID 2。用锁状态转换近似开锁，不能识别 APP 点击，也不判断实际装卸是否完成。

### ID 3：装卸货

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**发送到站和 N 挡，不发送自动模式或启动按钮。


```bash
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**终端 2 暂无输出，因为还缺 EPB 拉起条件。

**第 2 步｜终端 3：**补齐 EPB 拉起状态。


```bash
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
```

**第 3 步｜观察：**终端 2 应依次出现 `data: 3`、`data: 4`，各一次。两条可能很接近，请查看相邻的输出块。ID 3 和 ID 4 共用触发条件，因此同时看到另一条是正常行为。本节重点确认 ID 3“车辆已到达目的地请及时装卸货”。

**第 4 步｜终端 3：**重复同一状态，再释放并拉起 EPB。


```bash
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
```

**观察：**不应新增 ID 3 或 ID 4；同一次到站的机会已经使用，条件反复满足不会重播。

**第 5 步｜终端 3：**模拟离站后重新到站。


```bash
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 应再次依次出现一次 ID 3、一次 ID 4。两条均从停靠条件齐全时起最多等待 10 秒；ID 3 发出不延长 ID 4 的期限。临时 EPB 输入仅模拟反馈，不代表节点执行了实际驻车。

### ID 4：提醒关闭箱门

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**发送到站和 N 挡，不发送自动模式或启动按钮。


```bash
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**终端 2 暂无输出，因为还缺 EPB 拉起条件。

**第 2 步｜终端 3：**补齐 EPB 拉起状态。


```bash
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
```

**第 3 步｜观察：**终端 2 应依次出现 `data: 3`、`data: 4`，各一次。两条可能很接近，请查看相邻的输出块。ID 3 和 ID 4 共用触发条件，因此同时看到另一条是正常行为。本节重点确认 ID 4“装、卸货完成后请关闭箱门，谢谢”，且它排在 ID 3 之后。

**第 4 步｜终端 3：**重复同一状态，再释放并拉起 EPB。


```bash
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_epb_st std_msgs/msg/Bool "{data: true}"
```

**观察：**不应新增 ID 3 或 ID 4；同一次到站的机会已经使用，条件反复满足不会重播。

**第 5 步｜终端 3：**模拟离站后重新到站。


```bash
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_arrive_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 应再次依次出现一次 ID 3、一次 ID 4。两条均从停靠条件齐全时起最多等待 10 秒；ID 3 发出不延长 ID 4 的期限。临时 EPB 输入仅模拟反馈，不代表节点执行了实际驻车。

### ID 5：开锁成功

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**先发送未锁状态，不发送模式或启动按钮。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**观察：**终端 2 无输出，首次 false 只是初始化。

**第 2 步｜终端 3：**发送上锁后再未锁。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**观察：**终端 2 应立即出现一次 `data: 5`。

**第 3 步｜终端 3：**重复未锁反馈。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: false}"
```

**观察：**不应再出现 ID 5。若再次发送 true 后 false，则允许再出现一次 ID 5。每次真实转换同时启动 ID 2 的 60/120/180 秒提醒，长时间观察时可能看到 ID 2。

**第 4 步｜终端 3：**完成测试后上锁，取消尚未到期的 ID 2。


```bash
ros2 topic pub --once /trunk_lock_st std_msgs/msg/Bool "{data: true}"
```

本功能只用已知锁状态 true→false 近似开锁成功，不控制门锁、不识别 APP 点击来源。ID 5 若被高优先任务阻塞，最多等待 3 秒。

### ID 6：车辆起步

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**进入自动模式并启动，提供静止及一条旧高速反馈。这里故意用 8.0 验证旧车速不会取消下一次起步。

```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 8.0}"
```

**观察：**终端 1 有 `ad_mode_req=true`，终端 2 暂无 ID 6；静止时的车速反馈不能触发起步提示。

**第 2 步｜终端 3：**将下面两条命令整块复制并连续执行：先发送行驶 true，再于起步后不足 3 秒发送新的低速值。不要在两条命令之间停下来观察。

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 1.0}"
```

**观察：**仅行驶反馈本身不使用缓存的 8.0；收到新的 1.0 后，终端 2 出现一次 `data: 6`，说明旧高速没有错误取消任务。本例不设 D 挡，不触发 ID 7，也不提供箱门反馈，不触发 ID 1。

**第 3 步｜终端 3：**重复相同行驶和车速反馈。

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 1.0}"
```

**观察：**不新增 ID 6，同次行驶不重播。然后重新停车、起步，这次先不补车速：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

**观察：**即使缓存里仍有低速 1.0，也不应出现 ID 6。等待至少 4 秒，让原 3 秒期限结束，再发送：

```bash
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 1.0}"
```

**观察：**过期后不补发；新消息不会重新开启已经结束的起步任务。

**第 4 步｜终端 3：**验证同一数值重新发布也算新反馈。整块复制并连续执行：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 1.0}"
```

**观察：**终端 2 再出现一次 ID 6。最后的 1.0 虽然与缓存相同，但它在本次起步之后重新收到，因此有效。不是靠数值是否变化来识别新反馈。

**第 5 步｜终端 3：**验证新高速会取消本次机会。整块连续执行，注意车速 5.0 必须位于起步反馈之后：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 5.0}"
ros2 topic pub --once /veh_speed_st std_msgs/msg/Float32 "{data: 1.0}"
```

**观察：**本组不新增 ID 6；新车速≥5 已取消本次任务，随后减速也不重试。首次行驶反馈直接为 true，同样不构造起步事件。

车速接口为已授权临时协议，单位 km/h；条件为新反馈有效且 0≤车速<5。负数及非有限值不能触发，可在原期限内由后续有效新反馈补齐。停车、停止或退出自动模式也会取消待播。CLI 存在延迟，如果连续命令耗时已经达到 3 秒，可能正常超时；精确边界请用可控时间测试。

### ID 7：自动驾驶前进

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**先提供 D 挡和行驶状态，再进入自动模式，暂不启动。


```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 3}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 无输出；未取得启动授权不能发送 ID 7。首次行驶反馈直接为 true，也不会触发 ID 6。

**第 2 步｜终端 3：**发送启动按钮。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 1 出现 `ad_mode_req=true`，终端 2 立即出现 `data: 7`，之后每约 120 秒一次。要验证第二次，需要保持状态并等待约 2 分钟。

**第 3 步｜终端 3：**在首轮后重复行驶、D 挡、启动反馈。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 3}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**不额外立即发送，也不重置下一轮时间。

**第 4 步｜终端 3：**切入 N 挡。


```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**之后不再新增 ID 7。重新挂 D 挡：


```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 3}"
```

**观察：**立即再次出现 ID 7，并从此刻开始新一轮 120 秒周期。发送停止按钮可结束本例：


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
```

终端 1 应显示 `ad_mode_req=false`；终端 2 不再新增 ID 7。P0 阻塞时 ID 7 每轮最多等待 10 秒，延迟或丢弃不改变原定周期。

### ID 8：障碍物提示

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**提供有障碍物状态并进入自动模式，暂不启动。


```bash
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 无输出。

**第 2 步｜终端 3：**发送启动按钮。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 立即出现 `data: 8`，之后每约 5 秒一次。观察至少 12 秒，应能看到首轮和后续周期。本例不设 D 挡及行驶条件，以免引入 ID 7。

**第 3 步｜终端 3：**重复障碍物状态。


```bash
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: true}"
```

**观察：**不额外立即发送，不重置周期。随后清除障碍物：


```bash
ros2 topic pub --once /obs_detect_st std_msgs/msg/Bool "{data: false}"
```

**观察：**之后至少 6 秒不再新增 ID 8。再发 true 可立即恢复；按停止按钮或退出自动模式也会停止。这里用“检测到障碍物”近似需要避让，无法确认实际绕行过程。

### ID 9：右转弯

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**先提供转弯状态并进入自动模式，暂不启动。


```bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 无输出。

**第 2 步｜终端 3：**发送启动按钮。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 1 有 `ad_mode_req=true`；终端 2 立即出现 `data: 9`，此后每约 5 秒一次。至少观察 12 秒，确认有周期输出。

**第 3 步｜终端 3：**重复同一转弯状态。


```bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**不额外立即发送，也不重置周期。随后恢复直行：


```bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
```

**观察：**之后至少 6 秒不再新增 ID 9。再次发送本节转弯状态应立即恢复；停止按钮 false 或退出自动模式也会结束该提示。

### ID 10：左转弯

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**先提供转弯状态并进入自动模式，暂不启动。


```bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 无输出。

**第 2 步｜终端 3：**发送启动按钮。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 1 有 `ad_mode_req=true`；终端 2 立即出现 `data: 10`，此后每约 5 秒一次。至少观察 12 秒，确认有周期输出。

**第 3 步｜终端 3：**重复同一转弯状态。


```bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**不额外立即发送，也不重置周期。随后恢复直行：


```bash
ros2 topic pub --once /veh_turn_st std_msgs/msg/Int32 "{data: 0}"
```

**观察：**之后至少 6 秒不再新增 ID 10。再次发送本节转弯状态应立即恢复；停止按钮 false 或退出自动模式也会结束该提示。

### ID 11：行人礼让

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**启用自动模式并提供行人横穿状态，暂不发送车辆静止反馈。


```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 无输出；没有收到行驶状态时，不能把未知状态当成静止。

**第 2 步｜终端 3：**补齐静止反馈。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**终端 2 立即出现 `data: 11`，之后每约 5 秒一次，至少观察 12 秒。重复行人或静止反馈不额外触发，也不重置周期。

**第 3 步｜终端 3：**清除行人状态。


```bash
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: false}"
```

**观察：**之后至少 6 秒不再新增 ID 11。再发 true 可恢复，车辆开始行驶也会取消礼让提示。现有信号只能确认有行人且静止，不能单独证明斑马线位置或停车原因。

**第 4 步｜终端 3：**如要验证启动授权，先重新提供行人，再按停止按钮。


```bash
ros2 topic pub --once /pedestrian_cross_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
```

终端 2 在行人 true 时可出现一条 ID 11，但停止后不再新增；终端 1 显示 `ad_mode_req=false`。

### ID 12：倒车

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**启用自动模式后挂 R 挡。


```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 立即出现 `data: 12`，之后每约 5 秒一次；不需要发送行驶状态，挂 R 挡本身即可建立场景。

**第 2 步｜终端 3：**重复 R 挡反馈。


```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**不额外立即发送，不重置周期。随后换 N 挡：


```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**之后至少 6 秒不再新增 ID 12。再次挂 R 挡：


```bash
ros2 topic pub --once /veh_gear_st std_msgs/msg/Int32 "{data: 1}"
```

应立即恢复 ID 12。若换 D 挡且已经确认行驶，会改为触发 ID 7；本例选择 N 挡退出，便于只观察倒车提示是否结束。

### ID 13：等红绿灯

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**提供自动启动、静止和等灯状态，暂不提供光照。


```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /traffic_light_wait_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 无输出，因为昼夜尚未确定。普通静止不等于正在等灯，必须明确提供等灯状态。

**第 2 步｜终端 3：**发送白天光照。


```bash
ros2 topic pub --once /ambient_light_level_st std_msgs/msg/Float64 "{data: 6.0}"
```

**观察：**终端 2 立即出现 `data: 13`，此后每约 10 秒一次。重复光照、等灯或静止反馈不重置周期。

**第 3 步｜终端 3：**改成恰好为 5 的光照，切入夜间。


```bash
ros2 topic pub --once /ambient_light_level_st std_msgs/msg/Float64 "{data: 5.0}"
```

**观察：**停止白天循环，之后至少 12 秒没有新增 ID 13；本次等待已经发过提示，因此切入夜间不额外提示。

**第 4 步｜终端 3：**在夜间退出等待，再重新等灯。


```bash
ros2 topic pub --once /traffic_light_wait_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /traffic_light_wait_st std_msgs/msg/Bool "{data: true}"
```

**观察：**立即出现一次 `data: 13`，继续观察至少 12 秒，不应出现第二次。要恢复白天循环，发送：


```bash
ros2 topic pub --once /ambient_light_level_st std_msgs/msg/Float64 "{data: 6.0}"
```

**观察：**立即出现 ID 13，并从这次恢复白天的时刻重新每 10 秒循环。结束等待：


```bash
ros2 topic pub --once /traffic_light_wait_st std_msgs/msg/Bool "{data: false}"
```

**观察：**之后至少 12 秒不再新增 ID 13。停止、退出自动模式或开始行驶也会取消等待任务。

光照 ≤5 为夜间、>5 为白天；阈值由用户暂定，待后续文档同步，文档未定义光照单位。负数及非有限值忽略，保留之前的有效判断。每轮被阻塞最多等待 3 秒；夜间超时不重试，白天仍按原定 10 秒时间轴继续。

### ID 14：远程/遥控驾驶

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**先确认静止，再切入遥控模式；不发启动按钮。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**终端 2 立即出现一次 `data: 14`。保持静止超过 60 秒，不应循环发送；这是切入遥控模式的一次提示。终端 1 的 `ad_mode_req=false` 是正常的，因为本例不请求自动驾驶。

**第 2 步｜终端 3：**模拟开始行驶。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 立即再出现 ID 14，此后每约 60 秒一次。无需 D 挡，也无需启动授权。重复 true 或模式 2 不额外提示，不重置周期。

**第 3 步｜终端 3：**发送停车反馈。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**之后超过 60 秒不再新增 ID 14。再次发 true 可立即重新开始行驶循环。

**第 4 步｜终端 3：**验证遥控模式下启动按钮不会产生自动驾驶请求。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 1 仍显示 `ad_mode_req=false`，终端 2 不因按钮额外出现 ID 14。退出再切入模式：


```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 2}"
```

**观察：**再次切入时出现一次 ID 14。切入和行驶首轮同时发生时合并为一条；P0 阻塞时每个请求最多等待 10 秒。

### ID 15：行驶故障

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**只提供故障，不进入自动模式。


```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 无输出，尚未收到车辆是否行驶的反馈。

**第 2 步｜终端 3：**提供行驶状态。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 立即出现 `data: 15`，之后每约 5 秒一次；该提示不要求模式或启动授权。

**第 3 步｜终端 3：**改为另一正数故障等级，并发送停止按钮。


```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
```

**观察：**ID 15 继续原来的周期，不因等级变化额外立即提示，也不因停止按钮结束。随后模拟停车：


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**立即切换为 `data: 16`，之后每约 5 秒一次；再次发 true 则立即切回 ID 15。

**第 4 步｜终端 3：**清除故障。


```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
```

**观察：**之后至少 6 秒不再新增 ID 15 或 16。负数同样不视为已确认故障。节点只发语音请求，不会因该测试自动执行靠边或制动。

### ID 16：故障停车

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**只提供静止反馈，不进入自动模式。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**终端 2 无输出，因为没有故障。

**第 2 步｜终端 3：**提供正数故障。


```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 立即出现 `data: 16`，之后每约 5 秒一次。不需要此前出现过 ID 15。此处以“故障且静止”近似故障停车完成，无法确认实际靠边位置。

**第 3 步｜终端 3：**重复静止、改故障等级并按停止按钮。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
```

**观察：**ID 16 继续原周期，不额外立即提示。然后模拟行驶：


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

**观察：**立即切为 ID 15；再发静止：


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**立即切回 ID 16，两次切换均从切换时刻重新计算 5 秒周期。

**第 4 步｜终端 3：**清除故障。


```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
```

**观察：**之后至少 6 秒不再新增故障提示。模式切换或停止按钮不能代替故障清除。

### ID 17：低电量

**终端 1**：按通用准备重启节点。**终端 2**：保持语音观察命令运行；以下只核对本节开始后的新输出。

**第 1 步｜终端 3：**先提供静止、低电量和自动模式，暂不启动。


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /battery_low_power_st std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
```

**观察：**终端 2 无输出。

**第 2 步｜终端 3：**发送启动按钮。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**终端 2 立即出现一次 `data: 17`，约 10 秒后再出现一次；继续观察至少 12 秒，不应有第三次。每次有效启动最多两次，重复 true 不增加次数。低电量标志应由上游按 SOC≤20% 提供，节点不计算 SOC。

**第 3 步｜终端 3：**停止后重新启动，建立新的一组提示。


```bash
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

**观察：**立即出现这组的第一次 ID 17。看到后，**在第二次到来前**到终端 3 发送行驶反馈：


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: true}"
```

**观察：**之后至少 12 秒不出现该组第二次 ID 17。因本例未提供起步后的新车速，不会出现 ID 6；也未挂 D 挡，不会出现 ID 7。随后停车：


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**不自动重开低电量任务；必须再次有效启动才有新机会。停止、退出模式或任务开始后低电量标志变为 false，也会取消剩余请求。

**补充｜验证反馈稍后到达：**在终端 1 重启节点，终端 2 继续观察。终端 3 先只执行：


```bash
ros2 topic pub --once /veh_mode_st std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub --once /mode_sw_cmd std_msgs/msg/Bool "{data: true}"
```

即使等待超过 10 秒，终端 2 也无 ID 17，但发车窗口仍在等待。然后执行：


```bash
ros2 topic pub --once /battery_low_power_st std_msgs/msg/Bool "{data: true}"
```

此时仍无输出，因为尚未确认静止。最后补齐：


```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
```

**观察：**立即请求第一次 ID 17，至少 10 秒后请求第二次。若补齐前已经收到行驶 true，本次窗口会关闭，之后停车不补触发。

### 补充：按步骤验证高优先提示阻塞

完成正常流程后，可单独选择一个 ID 验证阻塞。**每次独立实验仍从终端 1 重启节点开始，终端 2 保持监听，所有输入在终端 3 执行。**

1. 先在终端 3 提供静止故障，使 ID 16 成为持续有效的 P0：

```bash
ros2 topic pub --once /veh_running_st std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 1}"
```

2. 在终端 2 确认每约 5 秒出现 ID 16。接着按目标 ID 的步骤提供条件。即使 ID 16 两条消息之间有空隙，目标低优先提示也不应插入。测试 ID 6 时，静止→行驶后故障提示会变成 ID 15，但仍属于阻塞。
3. 从目标请求的到期时刻开始计时，按下表选择在期限内或期限后清除故障。在终端 3 执行：

```bash
ros2 topic pub --once /alarm_diag_st std_msgs/msg/Int32 "{data: 0}"
```

| 目标 | 何时产生待播请求 | 等待期限 | 清除故障后的观察重点 |
| --- | --- | --- | --- |
| ID 1 | 条件齐全后每 10 秒到期，首轮也等 10 秒 | 3 秒 | 期限内可发本轮，过期跳过；下轮仍按原时间轴 |
| ID 2 | 开锁后第 60、120、180 秒 | 10 秒 | 期限内可发，过期也计入三轮，不增加补偿轮 |
| ID 3、4 | 到站、N 挡、EPB 齐全时 | 各 10 秒 | 期限内依次 3、4；ID 3 发出不延长 ID 4 的期限 |
| ID 5 | 已知上锁→未锁时 | 3 秒 | 过期后重复未锁不重试，须重新上锁再开锁 |
| ID 6 | 已知静止→行驶时 | 总计 3 秒 | 含等待起步后新车速时间；仍行驶且低于 5 才可发，过期不重试 |
| ID 7 | 行驶、D 挡、自动已启动首轮及之后每 120 秒 | 10 秒 | 延迟/丢弃不改变下一轮周期 |
| ID 13 | 白天首轮及每 10 秒，或唯一夜间请求 | 3 秒 | 白天下轮时间不变；夜间过期后本次不重试 |
| ID 14 | 切入遥控模式或行驶循环到期 | 10 秒 | 切入和循环分别计时；循环维持原 60 秒时间轴 |
| ID 17 | 有效启动后静止且低电量 | 无规定超时 | 清除后先发第一次，再至少间隔 10 秒发第二次；窗口必须仍有效 |

4. 需要测试取消时，在清除故障前先发送该任务的取消条件。例如 ID 5 重新上锁、ID 3/4 释放 EPB、ID 13 等灯 false；随后再清除故障，观察不能补发已取消请求。

手动操作只适合观察明显的期限内/期限外结果。不要把 CLI 命令执行耗时当成节点精确接收时间；3 秒和 10 秒边界、同次调度顺序、夜间超时、调度迟到等情况，以后面的可控时间逻辑测试为准。

### 没有看到预期输出时

1. 看终端 1：节点是否仍在运行，有没有异常退出？看终端 3 的 `ros2 node list` 是否含 `/body_control_node`。
2. 看终端 2：是否一直运行 `ros2 topic echo /voice_broadcast_id`，且在发送触发消息前已经开始监听？观察命令不显示历史请求。
3. 看终端 3：每条 `--once` 命令是否成功发送？若持续提示等待匹配订阅者，检查节点、话题拼写和三个终端的 ROS 环境是否一致。
4. 看是否遗漏前置条件：自动模式不等于已启动；未知行驶状态不等于静止；首次未锁不等于开锁事件；ID 13 必须有有效光照和等灯状态。
5. 若终端 2 持续出现 ID 15/16，说明故障正在抢占。若出现 ID 17，说明低电量任务正在执行。最简单的排查方式是终端 1 重启节点，再只执行目标 ID 的步骤；同时确认没有其他程序持续发布输入。
6. 输入命令发送的是状态，不是车辆执行动作；只要终端收到编号而没有扬声器声音，并不表示本节点验证失败，音频播放需要下游模块。

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

`cargo_completion_voice_test` 使用可控时间覆盖首次状态不误触发、统一开锁事件、ID 5 超时独立性、60/120/180 秒边界及最多三轮、重复反馈不重置、所有 P0 和低电量任务阻塞、10 秒丢弃边界和固定时间轴、迟到检查跳过历史轮次、重新上锁取消及再开锁重建、其他车辆状态不取消、原有 P1 顺序 2→14→7、各请求独立期限（加入 ID 1 后由新测试验证完整顺序），以及 P1 优先于到站 P2。无需真实等待三分钟。

`door_open_voice_test` 使用可控时间验证条件到达顺序、未知锁状态、首次等待 10 秒及重复输入、连续周期、上锁/停止/退出模式取消和重建、全部 P0 阻塞及 3 秒边界、低电量优先、行驶及其他车辆状态不取消、迟到跳过历史轮次、ID 1/2/7 同时待播顺序、其他请求原期限不变及 P1 优先于 P2。ID 14 与 ID 1 的模式条件互斥，其既有优先顺序由遥控和 ID 2 测试回归。

`start_voice_test` 使用可控时间验证首次行驶不误报、每次起步重新等待新反馈、旧高速不取消/旧低速不触发、相同数值及序号回绕的新反馈识别、无新反馈超时、新车速 0/5 边界及非法值补齐、P0 阻塞下原 3 秒期限、停车/新高速/停止/模式退出取消后不重试、原优先级 1→6→2→7、等待新车速不阻塞 ID 7/P2，以及行驶使低电量任务按原规则结束。

`traffic_wait_voice_test` 使用可控时间覆盖五项启用条件的全部到达顺序、未知/非法光照、阈值 5 的边界及有效判断保留、白天立即首轮与固定 10 秒周期、重复状态与迟到轮次跳过、夜间单次、全部可共存 P0/低电量阻塞、3 秒边界、昼夜转换时期限保留或替换、夜间机会不重开、退出/行驶/停止/模式变化取消及重建，以及 1→13→2 和 P1/P2 优先及各自期限。全部测试不需真实等待。

`automatic_voice_test` 覆盖模式 0/1/2/未知值与授权 true/false 的完整矩阵，确认所有自动相关模块只在自动且已启动时放行；异常模式与授权矛盾时也拒绝。验证故障、遥控、到站和开锁相关语音保留独立条件，以及准入失效后待播取消、重新有效准入后按原周期重建。
