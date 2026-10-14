# 架构与内部设计

以下章节原样取自此前的 README，面向维护者和开发者；面向用户的手册在[文档站](https://astreox.github.io/uBotDocs/ros/)。“运行时状态与问题”一节里的“上文‘WSL 与网络’”现在见[准备 ROS 2 端](https://astreox.github.io/uBotDocs/ros/ros2-setup/)。

## 简介

UBotROS 是一个**纯翻译层**，不含任何控制逻辑，**UE 侧不依赖 TempoROS、rclcpp 或任何第三方库**：

- UE 自己把消息编码成标准 ROS 2 CDR 字节（`FUBotCdrWriter`，以及与 ROS 2 Humble 消息逐字段对应的 `FUBotRos*` 结构），通过一条 TCP 连接（默认端口 8268）发给随插件附带的 ROS 2 桥节点 `ubot_ros_bridge`；
- `ubot_ros_bridge` 是一个小型的 rclpy 节点：把收到的字节原样发布到 ROS 2 话题，并把 UE 订阅的话题（如 `cmd_vel`）原样转发回 UE。

| 内容 | 说明 |
| --- | --- |
| `UUBotROSBridgeSubsystem` | 每个 Game / PIE 世界一个 TCP 连接：声明发布者和订阅者、`/tf`、聚合的 `/tf_static`、`/clock`、断线重连并重新注册 |
| uBot ROS Sensor Bridge（`UUBotROSSensorBridgeComponent`） | 发现所属 Actor 上的 uBot 传感器并发布到 ROS 2，同时发布传感器 TF。旧版 `Agent ROS Sensor Suite Publisher` 的继任者 |
| uBot ROS Velocity Command（`UUBotROSVelocityCommandComponent`） | 订阅 `geometry_msgs/msg/Twist`，换算后交给 `IUBotVelocityCommandable`，带超时和限速 |
| `IUBotROSSensorPublisherFactory` | 扩展点 `UBotROS.SensorPublisher`，为自定义传感器添加发布器；内置 7 个工厂，对应 UBotSensor 的 7 种传感器 |
| `ros2/ubot_ros_bridge` | ROS 2 侧的桥节点（ament_python 包，也可以不经 colcon 直接运行） |

插件只有一个 Runtime 模块 `UBotROS` 和一个日志类别 `LogUBotROS`。桥子系统只存在于 Game / PIE 世界，编辑器视口世界里没有，所以组件只在运行时（Play）发布。

## 架构

```
UE (Windows/Linux)                         ROS 2 机器 / WSL
UBotROS 插件 ──TCP 8268──▶ ubot_ros_bridge (rclpy) ──▶ ROS 2 话题 (/tf, /clock, 传感器)
   ▲                                   │
   └──────── cmd_vel CDR ◀─────────────┘
```

为什么这样设计：

- **UE 零依赖**：不需要 TempoROS、rclcpp 或预编译的 ROS 库，插件只用引擎自带的 `Sockets`、`Networking`、`Json` 模块，随引擎一起编译。
- **UE 在 Windows、ROS 在 WSL / Linux**：两边只通过一条 TCP 连接相连，ROS 不必装在 UE 所在的系统上。
- **原始 CDR**：UE 序列化的就是 `rclpy.serialization.serialize_message` 产生的字节，桥用 `Publisher.publish(bytes)` 发布、用 `create_subscription(..., raw=True)` 订阅，**桥里不做反序列化**，图像和点云这类大数据不会在 Python 里被转换。

数据流：传感器采样事件触发发布器（游戏线程）→ 编码成 CDR → `UUBotROSBridgeSubsystem::Publish` 入队 → 后台 I/O 线程写 `PUBLISH` 帧 → 桥发布。反方向，桥把订阅到的消息作为 `MESSAGE` 帧发来，I/O 线程接收后在**游戏线程的子系统 Tick 里**分发给订阅回调。游戏线程永远不会阻塞在 socket 上。

## 在 uBot 家族中的位置

UBotROS 是 **Adapter**（适配器）层，只依赖 `UBotCore ^0.1.2` 和 `UBotSensor ^0.1.0`（`.uplugin` 的 `"Plugins"` 与 `"UBot"` 块都声明了这两个依赖）。

| Package | 层 | 作用 |
| --- | --- | --- |
| `UBotCore` | Foundation | 帧头、仿真时钟 `UBot::Clock`、坐标 / 单位 / 时间约定 `UBot::Ros` 与 `UBot::Units`、身份组件、扩展注册表 |
| `UBotSensor` | Capability | 位姿、里程计、IMU、2D / 3D 激光雷达、RGB / 深度相机 |
| `UBotROS` | Adapter | 本插件：把上面两个 Package 的数据翻译成 ROS 2 |

所有坐标、单位和时间换算都走 Core 的 `UBot::Ros` / `UBot::Units`，插件内没有自己的换算公式。`"UBot"` 块声明的能力键（`Provides`）：`ROS.SensorPublishing`、`ROS.TF`、`ROS.Clock`、`ROS.VelocityCommand`。引擎模块依赖：公有 `Core`、`CoreUObject`、`Engine`、`DeveloperSettings`，私有 `Sockets`、`Networking`、`Json`。

## 运行时状态与问题

桥的连接状态会发布到 UBotCore 的运行时状态（`FUBotRuntimeStatus`），uBot 面板和 uBot Manager 都从这里取得。每个 Game / PIE 世界各有一个桥子系统，但只有一个 `ros.bridge` 状态项和一个问题；同时有多个世界时，显示状态最好的那个（已连接，其次正在连接，其次失败，最后是已断开）。

状态项 `ros.bridge`（标签 `ROS bridge`，中文界面 `ROS 桥接`；`detail` 是 `host:port`）：

| 桥子系统状态 | `value` | 严重度 | 面板文字（英文 / 中文） | 何时 |
| --- | --- | --- | --- | --- |
| `Connecting` | `connecting` | Info | Connecting / 正在连接 | 开始连接 |
| `Connected` | `connected` | Ok | Connected / 已连接 | 与桥握手完成 |
| `Error` | `disconnected` | Warning | Disconnected / 已断开 | 一次连接尝试失败，或已建立的连接断开；之后每 `ReconnectIntervalSeconds` 重试 |
| `Disabled` | `disconnected` | Info | Disconnected / 已断开 | 调用了 `Disconnect()` |

一次连接尝试失败或连接断开时，还会报告问题 `rosBridgeUnreachable`（严重度 Info，参数 `address` 为 `host:port`，消息 `Could not connect to ubot_ros_bridge at <address>.`，中文界面 `无法连接 <address> 上的 ubot_ros_bridge。`）。连接成功或 `Disconnect()` 后，问题被清除。

它们出现在这些地方：

- **uBot 面板**（`Window > uBot`）：Runtime 分区显示状态项，Problems 分区显示问题。
- **`<Project>/Saved/uBot/status.json`**：一次 PIE 或游戏运行结束时写入，运行结束时仍存在的问题会留在文件里（格式见 UBotCore 的 README“运行时状态”）。
- **uBot Manager**：在 Problems 视图的“上次运行”范围里显示这个问题，快速修复是“打开 ROS 设置”，直接跳到 设置 > ROS 桥接 修改主机和端口，那里的“测试连接”会做一次 TCP 连接。uBot Manager 自己也会在刷新时检查（设置里的“刷新时检查桥接”，默认开）：UBotROS 已安装并启用时，对工程配置的地址做一次 2 秒的 TCP 连接，连不上就在 Package 范围里报告同一个问题。

看到这个问题，先确认桥已经启动、地址和端口一致，WSL 和防火墙的注意事项见上文“WSL 与网络”。

## 协议

UE 与桥之间的 TCP 协议（UBRB v1）完整规范见 [`docs/PROTOCOL.md`](PROTOCOL.md)，只读这份文档就能实现一个兼容的桥。要点：每帧 12 字节头（魔数 `UBRB`、版本 1、op、flags、小端负载长度），12 种 op（`HELLO`、`HELLO_ACK`、`ADVERTISE`、`UNADVERTISE`、`PUBLISH`、`SUBSCRIBE`、`UNSUBSCRIBE`、`MESSAGE`、`SUB_COUNT`、`PING`、`PONG`、`ERROR`），话题和类型作为 JSON 声明，消息体是带 4 字节封装头的完整 CDR。UE 是客户端，每次（重）连接先发 `HELLO`，再重新声明全部发布者和订阅者。修改协议行为时，该文档、UE 端（`UBotROSProtocol.*`、`UBotROSBridgeSubsystem.*`）和 Python 端（`protocol.py`、`bridge_node.py`）要一起改。
