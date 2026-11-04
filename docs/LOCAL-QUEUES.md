# UE 本地发布队列

本页描述未发布的 uBotROS 0.2.0，依赖 uBotCore 和 uBotSensor `^0.2.0`。请从配套源码统一重新编译，不能混用 0.1.x 与 0.2.0 DLL。

本地排队发生在 CDR 编码之后、socket 发送之前，与 ROS 2 的 DDS QoS 是独立配置。线协议和消息字节布局不变。

原三参数 Advertise 保留 Legacy 行为：best-effort 保留该发布者最新一条待发消息，reliable 按原队列顺序发送。新增四参数形式接受 `FUBotROSLocalQueueOptions`：

| 模式 | 行为 |
| --- | --- |
| Legacy | 沿用旧的、由 reliability 选择的策略 |
| LatestOnly | 不论 DDS reliability，只保留最新待发消息 |
| BoundedFifo | 按顺序保留，受数量、单调墙钟寿命和全局字节上限约束 |

FIFO 满时丢弃最旧待发消息；过期数据不会从本地队列交给 socket。同类活动发布者轮转取消息。控制消息及旧 reliable 队列的现有优先级保留。

最长等待针对尚未交给发送缓冲的数据，不是端到端时延保证。已经交给 socket 的 TCP 帧不会强制截断；网络阻塞、桥和订阅者还会增加延迟。

## 高频 IMU 默认值

在 Play 前选择 IMU 的 PhysicsTimeline 模式后，ROS 内置 IMU 发布器默认采用 FIFO、最多 128 条、100 ms 本地等待寿命。默认 DDS 深度为 128；显式 Sensor Data QoS Override 优先。可用 Sensor Bridge 的 Local Queue Override 覆盖本地策略。

原生自定义发布器使用 FUBotROSPublisherContext 时，可以显式设置 bSensorQosOverridden 和 LocalQueueOverride。Sensor 不依赖 ROS，不负责选择这些策略。运行中更改采样模式后，应重新初始化 ROS Sensor Bridge，使发布者重新声明 QoS。

原生 ConfigurePublisherQueue 只更新本地策略并清理旧待发数据，不修改 DDS；ClearPendingMessages 可用于明确的时间线重置。

128 条容量覆盖默认 500 Hz 物理频率、最多 250 步补算时的 200 Hz 正常批次；更长停顿仍可能产生可观察的丢弃。降低帧率后，仅把 DDS depth 加大并不能阻止 UE 本地 LatestOnly 覆盖样本。

## 清理与诊断

FIFO 在会话结束、发布者注销或显式清理时丢弃待发数据，不重放到新连接。IMU 时间线重置会请求清理待发队列；停止 Play 后重新运行 SLAM 的既有流程仍适用。

GetStats 是蓝图只读快照，可由现有 Get(WorldContext) 获取所属游戏世界的桥，再读取统计。总丢弃计数继续保留；新增 FIFO 的过期、溢出，以及本地队列清理、未连接丢弃分类和交给发送缓冲时的累计最大排队年龄。分类统计在世界/连接客户端的寿命内累计，稳态检查应计算窗口前后的差值。接收端应另行测量消息间隔和缺口，不能把这些本地计数当作 DDS 接收保证。

回归位于 UBotROS.Subsystem.LocalQueue.*，使用真实 TCP 字节解析验证保留、顺序、轮转、寿命、全局容量和断线清理。
