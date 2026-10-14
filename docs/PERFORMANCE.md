# 性能与限制

以下内容原样取自此前的 README。面向用户的说明见文档站的[连接与状态](https://astreox.github.io/uBotDocs/ros/connection/)和[发布传感器数据](https://astreox.github.io/uBotDocs/ros/publish-sensors/)。

## 性能与限制

- **发送队列（UE → 桥）**：best_effort 发布者只保留最新一条待发消息，旧的被替换并计入丢弃数；没有会话（未连接或重连中）时 best_effort 数据直接丢弃；reliable 数据按顺序排队，重连期间也排，总量受 `MaxQueuedBytes` 限制，超出时先丢弃最旧的 best_effort 数据，再拒绝新的 reliable 数据并警告（`Publish` 返回 `false`）；桥处于 `Disabled` 时什么都不排队。C++ 的 `GetStats()` 给出已发 / 已收帧数、丢弃数、拒绝数和队列字节数。
- **接收（桥 → UE）**：`MESSAGE` 在后台线程接收，在游戏线程 Tick 里按到达顺序分发；积压受 `MaxQueuedBytes` 限制，超出的新消息被丢弃。桥那一侧每个客户端有 128 MiB（`max_queued_bytes`）的发送队列，队列满时丢弃 best_effort 消息，慢客户端不会拖慢 ROS 执行器或其他客户端。
- **按订阅者编码**：桥每个客户端在订阅者数量变化时汇报 `SUB_COUNT`（最快每 0.5 秒一次）。`bSkipWhenNoSubscribers`（默认开）时，某话题的订阅者数为 0 就不编码，相机也不做 CPU 读回；在第一次汇报之前数量未知，照常发布。best_effort 话题在桥未连接时也不编码。
- **负载上限**：单帧负载默认 64 MiB（`MaxPayloadBytes`，两端要一致）。超出时 UE 拒绝并警告，桥丢弃并回 `message_too_large`。作为量级参考：`bgra8` 图像每帧 `宽 × 高 × 4` 字节（640x480 约 1.2 MB），点云每个点 22 字节。
- **没有额外的限频**：话题频率就是传感器的 `UpdateRateHz`。编码（包括点云逐点换算和图像拷贝）在游戏线程完成，socket 读写在后台线程。能持续传多少数据取决于 UE 与桥之间的链路（本机、WSL 转发或网络）、游戏线程上的编码耗时和 ROS 侧订阅者，请用 `ros2 topic hz` / `ros2 topic bw` 和桥日志里的 dropped 提示实测，必要时降低分辨率和采样频率。
- **连接与重连**：UE 每 2 秒发一次 PING，10 秒没有收到桥的任何数据就断开重连；一次连接尝试最长 3 秒。重连后重新声明全部发布者和订阅者（id 不变），重新发布 `/tf_static`，排队的 reliable 数据随后发出；已经交给 socket 的帧会丢失。UE 断开期间桥会销毁它的话题，ROS 侧的订阅者需要能处理发布者重新出现。
- **多个世界共用一个桥**：每个 Game / PIE 世界是独立的 TCP 连接，桥默认最多同时接受 32 个。不同世界发布同一个话题时共享桥里的同一个 ROS 发布者，类型和 QoS 必须一致，否则桥回 `type_conflict` / `qos_conflict`。
- **其他**：协议没有认证和加密；一个 Sensor Bridge 只搜索所属 Actor；消息类型只覆盖 `UBotROSMessageTypes.h` 里的那些，其余用 `FUBotCdrWriter` 手写；只验证了 ROS 2 Humble。
