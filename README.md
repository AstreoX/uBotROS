<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/brand/ubot-mark-light.svg">
  <source media="(prefers-color-scheme: light)" srcset="docs/brand/ubot-mark-dark.svg">
  <img alt="uBot" src="docs/brand/ubot-mark-dark.svg" height="96">
</picture>

# uBot ROS

uBot 家族的 ROS 2 适配器插件，面向 UE 5.5：把 uBot 传感器、TF 和仿真时钟发布到 ROS 2，并把 ROS 2 的 `cmd_vel` 速度指令交给执行器。

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

UBotROS 是 **Adapter**（适配器）层，只依赖 `UBotCore ^0.1.3` 和 `UBotSensor ^0.1.0`（`.uplugin` 的 `"Plugins"` 与 `"UBot"` 块都声明了这两个依赖）。

| Package | 层 | 作用 |
| --- | --- | --- |
| `UBotCore` | Foundation | 帧头、仿真时钟 `UBot::Clock`、UE 与右手系（FLU，SI 单位）之间的坐标换算 `UBot::RightHanded`、单位 `UBot::Units`、身份组件、扩展注册表 |
| `UBotSensor` | Capability | 位姿、里程计、IMU、2D / 3D 激光雷达、RGB / 深度相机 |
| `UBotROS` | Adapter | 本插件：把上面两个 Package 的数据翻译成 ROS 2 |

坐标轴和单位的换算用 Core 的 `UBot::RightHanded` / `UBot::Units`，不在本插件里重复实现；只有 ROS 才有的部分在本插件的 `UBot::ROSConventions`：`builtin_interfaces/Time` 的秒拆分、相机光学系、ROS 名字合法化。另有两条本插件自己的规则：6x6 协方差的换号（`ConvertCovariance6`）和 LaserScan 的样本倒序（`MakeLaserScan`）。详见“坐标与时间约定”一节。

`"UBot"` 块声明的能力键（`Provides`）共 6 个：`ROS.SensorPublishing`、`ROS.TF`、`ROS.Clock`、`ROS.VelocityCommand`、`ROS.AttitudeCommand`、`ROS.MotorCommand`。引擎模块依赖：公有 `Core`、`CoreUObject`、`Engine`、`DeveloperSettings`，私有 `Sockets`、`Networking`、`Json`。

## 安装

### UE 侧

要求 UE 5.5，插件以源码形式分发，工程需要能编译 C++。代码只用引擎的 Sockets 模块，没有平台相关部分；开发和测试在 Windows 上进行，Linux 上的 UE 没有单独验证。

1. 先安装 UBotCore 和 UBotSensor，再把仓库克隆到它们旁边，即 `<Project>/Plugins/uBot/uBotROS`：

   ```powershell
   git clone https://github.com/AstreoX/uBotROS.git <Project>/Plugins/uBot/uBotROS
   ```

   也可以用 [uBot Manager](https://github.com/AstreoX/uBotManager) 安装：在 uBotCore 的 **uBot** 面板（`Window > uBot`）里点 *Open uBot Manager* 打开它，安装 UBotROS，或应用 Recipe “ROS 2 传感器”（`ros-sensors`）；缺失的依赖会一起装好。命令行等价于 `UnrealEditor-Cmd.exe <Project>.uproject -run=UBotPackage -Install=UBotROS`。

2. 启用插件：在 `Edit > Plugins` 中搜索 uBot ROS（分类 uBot）并勾选，或在 `.uproject` 中加入

   ```json
   "Plugins": [
       { "Name": "UBotCore", "Enabled": true },
       { "Name": "UBotSensor", "Enabled": true },
       { "Name": "UBotROS", "Enabled": true }
   ]
   ```

3. 重启编辑器。设置位于 `Project Settings > Plugins > uBot ROS`，保存在工程的 `Config/DefaultUBotROS.ini` 的 `[/Script/UBotROS.UBotROSSettings]` 段：

   ```ini
   [/Script/UBotROS.UBotROSSettings]
   BridgeHost=127.0.0.1
   BridgePort=8268
   ```

   uBot Manager 的 设置 > ROS 桥接 读写同一个文件里的 `BridgeHost` 和 `BridgePort`（只改这两行，其余内容不变；没有安装 UBotROS 的工程使用 uBot Manager 自己保存的值）。插件目录里自带的 `Config/DefaultUBotROS.ini` 只放 CoreRedirects，是另一个文件。

   **从旧版本迁移**：早期版本把这些设置写在 `Config/DefaultEngine.ini`。现在设置类使用 `Config=UBotROS`，只读取 `DefaultUBotROS.ini`，`DefaultEngine.ini` 里的 `[/Script/UBotROS.UBotROSSettings]` 段不再生效，升级后设置会回到默认值。把整段从 `DefaultEngine.ini` 剪切到工程的 `Config/DefaultUBotROS.ini`，或在 Project Settings 里重新设置一遍（编辑器会写入新位置）。

C++ 工程使用时，在自己模块的 `Build.cs` 中加入 `"UBotROS"`（以及用到的 `"UBotCore"`、`"UBotSensor"`）。

### ROS 侧

需要 ROS 2 Humble（Ubuntu 22.04，测试环境）、`rclpy`、`rosidl_runtime_py`，以及插件用到的消息包：`builtin_interfaces`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_msgs`、`rosgraph_msgs`。桥本身只用这些和 Python 标准库，不需要 C++ 编译器。桥只会加载 ament 索引里已安装的消息接口，所以用到自定义消息时要先 `source` 对应的工作空间。

桥的源码在插件的 `ros2/ubot_ros_bridge`。两种用法：

```bash
# A. 不经 colcon，直接运行（已验证的方式）
source /opt/ros/humble/setup.bash
cd <Project>/Plugins/uBot/uBotROS/ros2/ubot_ros_bridge
python3 -m ubot_ros_bridge.bridge_node --port 8268

# B. 复制（或软链接）到 colcon 工作空间并构建
mkdir -p ~/ubot_ws/src && cp -r <Project>/Plugins/uBot/uBotROS/ros2/ubot_ros_bridge ~/ubot_ws/src/
cd ~/ubot_ws && colcon build --packages-select ubot_ros_bridge && source install/setup.bash
ros2 run ubot_ros_bridge bridge --ros-args -p port:=8268
ros2 launch ubot_ros_bridge ubot_ros_bridge.launch.py port:=8268
```

包描述（`package.xml`、`setup.py`、`setup.cfg`、`resource`、`launch`）是完整的，但测试环境里没有 colcon，B 方式没有在本仓库的验证中跑过。参数、限制和排查见 `ros2/ubot_ros_bridge/README.md`。

### WSL 与网络

- **Windows 上的 UE + WSL 里的桥**：保持 `BridgeHost` 为默认的 `127.0.0.1` 即可。WSL2 的 localhost 转发会把 Windows 的 `127.0.0.1:8268` 转到 WSL 里监听的桥（在 NAT 网络模式、`localhostForwarding=true` 下，桥监听 `0.0.0.0` 和 `127.0.0.1` 两种情况都已验证）。
- **ROS 在另一台机器上**：把 `BridgeHost` 设为那台机器的地址，桥保持默认的 `0.0.0.0` 监听，并在防火墙上放行 TCP 8268。
- **没有认证和加密**：协议是明文 TCP，任何能连上端口的人都能让桥发布 / 订阅话题。桥默认监听 `0.0.0.0`，在可能被不可信网络访问的机器上请用 `--bind-address 127.0.0.1`（WSL 本机转发仍然可用）或防火墙限制来源。
- 桥进程的 `ROS_DOMAIN_ID` 决定话题出现在哪个 ROS 域里。

## 快速开始

1. **启动桥**（WSL 或 ROS 机器）：见上一节 A 方式。桥启动日志形如 `ubot_ros_bridge 0.1.0 listening on 0.0.0.0:8268 (ROS humble, ...)`。
2. **在 UE 里搭机器人**：在同一个 Actor 上添加若干 uBot 传感器组件，再添加 **uBot ROS Sensor Bridge**。可选添加 **uBot Identity**（uBotCore）命名机器人，例如 `Namespace = robot1`；不加的话命名空间取净化后的 Actor 名。需要接收速度指令时再添加 **uBot ROS Velocity Command**。
3. **检查项目设置**：`Project Settings > Plugins > uBot ROS`，默认的 `127.0.0.1:8268` 通常不用改。
4. **Play（PIE 或 Game）**。`bAutoConnect` 开启时世界开始时自动连接，UE 日志（`LogUBotROS`）出现 `uBot ROS bridge: connected to 127.0.0.1:8268 (ubot_ros_bridge 0.1.0, ROS humble)` 和 `ROS sensor bridge '...' on '...' ready: namespace '/robot1', N sensor(s), M topic(s)`；桥日志里能看到 `advertised /robot1/imu/data [sensor_msgs/msg/Imu]` 之类的行。编辑器的 uBot 面板（`Window > uBot`）的 Runtime 分区里，ROS bridge 一项同时显示 `Connected · 127.0.0.1:8268`。
5. **在 ROS 里查看**（已用桥和一个模拟 UE 的客户端验证过这些命令）：

   ```bash
   ros2 topic list
   ros2 topic echo /robot1/imu/data --once
   ros2 topic hz /robot1/imu/data
   ros2 topic echo /clock --once
   ros2 topic echo /tf_static --once --qos-durability transient_local --qos-reliability reliable
   ros2 run tf2_ros tf2_echo base_link imu_sensor
   ros2 topic pub -r 10 /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
   ```

   传感器话题是 best_effort，`ros2 topic echo` 会自动适配；自己写订阅者或在 RViz 里显示时，把 QoS 设为 best_effort。没有订阅者的话题默认不编码、不发送（`bSkipWhenNoSubscribers`），所以 `ros2 topic list` 里有话题不代表 UE 一直在发数据。

### 项目设置

| 设置 | 默认 | 说明 |
| --- | --- | --- |
| `BridgeHost` | `127.0.0.1` | 桥的地址（IP 或主机名；主机名在线程池线程里解析，不会卡住 I/O 线程） |
| `BridgePort` | `8268` | 桥的 TCP 端口 |
| `bAutoConnect` | 开 | 世界开始时自动连接；关闭后从蓝图或 C++ 调用子系统的 `Connect()` / `ConnectTo(Host, Port)` |
| `ReconnectIntervalSeconds` | `2` | 连接失败或断开后的重试间隔 |
| `bPublishClock` / `ClockTopic` | 开 / `/clock` | 是否发布 `/clock` 及其话题 |
| `MaxPayloadBytes`（高级） | 64 MiB | 单帧负载上限，需与桥的 `max_payload_bytes` 一致 |
| `MaxQueuedBytes`（高级） | 256 MiB | 发送队列和接收分发积压的字节预算；小于 `MaxPayloadBytes + 12` 时连接时自动提高并警告 |

地址、端口、重连间隔和限制在连接启动时读取（对同一端点重复 `Connect()` 是空操作，要先 `Disconnect()`）；`bPublishClock` 和 `ClockTopic` 每个 Tick 都会重新读取。子系统的蓝图接口（类别 `uBot|ROS`，通过 Get World Subsystem 取得）：`IsBridgeConnected`、`GetBridgeStatus`（`Disabled` / `Connecting` / `Connected` / `Error`）、`Connect`、`ConnectTo`、`Disconnect`、`GetLastBridgeError`、`GetBridgeEndpoint`，以及事件 `OnBridgeConnected`、`OnBridgeDisconnected(Reason)`。

### uBot ROS Sensor Bridge 属性

| 属性 | 默认 | 说明 |
| --- | --- | --- |
| `RobotNamespaceOverride` | 空 | 空则用 `UBot::Identity::ResolveNamespace`（身份组件的 `Namespace`，否则净化后的 RobotId / Actor 名）；`/` 表示根命名空间；每一段都会被净化 |
| `BaseFrameOverride` | 空 | 空则用身份组件的基坐标系（默认 `base_link`） |
| `BaseLinkComponent` | 空 | 空或无法解析则用身份组件的 `BaseComponent`（默认根组件） |
| `bPrefixFramesWithNamespace` | 关 | 所有 frame id 变成 `<namespace>/<frame>`，见 TF 树一节 |
| `bAutoInitialize` | 开 | `BeginPlay` 时自动初始化 |
| `bPublishInvalidSamples` | 关 | 是否也发布无效样本（`WarmingUp` 等） |
| `bSkipWhenNoSubscribers` | 开 | 桥报告某话题没有订阅者时跳过编码（相机还跳过 GPU 读回） |
| `bPublishDynamicTF` / `bPublishStaticSensorTF` | 开 / 开 | 是否发布 `odom -> base` 和 `base -> 传感器` |
| `OdometryFrame` | `odom` | 里程计坐标系，动态 TF 的父坐标系 |
| `TopicOverrides` | 空 | 按逻辑键替换默认话题，见下一节 |
| `SensorDataQosOverride` / `StateQosOverride` | 不覆盖 | 覆盖传感器数据（默认 best_effort / volatile / 5）和状态（pose、odom，默认 reliable / volatile / 10）的 QoS |

蓝图接口与旧版一致：`IsROSBridgeReady`、`GetROSBridgeStatus`（`Uninitialized` / `Initializing` / `Ready` / `Error` / `Stopped`）、`GetROSBridgeError`、`GetDiscoveredSensorCount`、`GetActivePublisherTopics`、`RefreshSensorPublishers`、`RestartROSBridge`、`StopROSBridge`，事件 `OnROSBridgeReady`、`OnROSBridgeError`、`OnROSPublishersRefreshed`；新增 `GetResolvedNamespace`、`GetResolvedBaseFrameId`。组件在发布器建好后就进入 `Ready`，**不等**桥连接成功：断开期间 best_effort 数据直接丢弃，reliable 数据排队，组件只在断开持续约 2 秒后记一条 Warning。

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

## 话题与消息

默认话题是 `/<namespace>/<路径>`。同一个 Actor 上同类型传感器不止一个时，改用各自净化后的 `SensorId`。

| uBot 传感器 | 默认话题 | 同类型多个时 | ROS 类型 | frame_id | QoS |
| --- | --- | --- | --- | --- | --- |
| Pose | `pose` | `<SensorId>` | `geometry_msgs/msg/PoseStamped` | 传感器的 `ReferenceFrameId`（默认 `world`） | reliable / volatile / 10 |
| Odometry | `odom` | `<SensorId>` | `nav_msgs/msg/Odometry` | `header` 为传感器的 `OdometryFrameId`（`odom`），`child_frame_id` 为 `ChildFrameId`（`base_link`） | reliable / volatile / 10 |
| IMU | `imu/data` | `<SensorId>` | `sensor_msgs/msg/Imu` | 传感器 `FrameId`（默认 `imu_sensor`） | best_effort / volatile / 5 |
| 2D LiDAR | `scan` | `<SensorId>` | `sensor_msgs/msg/LaserScan` | 传感器 `FrameId`（默认 `lidar_2d`） | best_effort / volatile / 5 |
| 3D LiDAR | `lidar/points` | `<SensorId>` | `sensor_msgs/msg/PointCloud2` | 传感器 `FrameId`（默认 `lidar_3d`） | best_effort / volatile / 5 |
| RGB 相机 | `camera/color/image_raw`<br>`camera/color/camera_info` | `<SensorId>/image_raw`<br>`<SensorId>/camera_info` | `sensor_msgs/msg/Image`（`bgra8`）<br>`sensor_msgs/msg/CameraInfo` | `<FrameId>_optical`（默认 `rgb_camera_optical`） | Image best_effort / volatile / 5<br>CameraInfo reliable / volatile / 1 |
| 深度相机 | `camera/depth/image_raw`<br>`camera/depth/camera_info` | `<SensorId>/image_raw`<br>`<SensorId>/camera_info` | `sensor_msgs/msg/Image`（`32FC1`，米）<br>`sensor_msgs/msg/CameraInfo` | `<FrameId>_optical`（默认 `depth_camera_optical`） | 同上 |

其余话题：

| 话题 | 类型 | QoS | 来源 |
| --- | --- | --- | --- |
| `/tf` | `tf2_msgs/msg/TFMessage` | reliable / volatile / 100 | 动态 `odom -> base` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | reliable / transient_local / 1 | 所有 Sensor Bridge 的静态 TF 合并成一条锁存消息 |
| `/clock`（`ClockTopic`） | `rosgraph_msgs/msg/Clock` | reliable / volatile / 10 | 子系统 |
| `/<namespace>/cmd_vel` | `geometry_msgs/msg/Twist` | best_effort / volatile / 10 | ROS 发给 UE（订阅） |
| `/<namespace>/cmd_attitude` | `mavros_msgs/msg/AttitudeTarget` | best_effort / volatile / 10 | ROS 发给 UE（订阅），见“飞行指令” |
| `/<namespace>/cmd_motors` | `actuator_msgs/msg/Actuators` | best_effort / volatile / 10 | ROS 发给 UE（订阅），见“飞行指令” |

消息要点：

- 时间戳都是传感器采样时的仿真时间（见“时间”一节），经 `UBot::ROSConventions::SecondsToRosTime` 转成 `builtin_interfaces/Time`。
- **PointCloud2**：无组织点云（`height = 1`），字段 `x`、`y`、`z`（float32，偏移 0 / 4 / 8）、`intensity`（float32，12）、`ring`（uint16，16）、`time`（float32，18），`point_step = 22`，小端；点经 `VectorFromUE`（y 取反）；帧不含强度时 `intensity` 为 0；所有坐标有限时 `is_dense = true`。
- **LaserScan**：按 ROS 约定镜像（角度向左为正），未命中的样本为 `+inf`，帧里有强度时一并发布。
- **Image / CameraInfo**：RGB 为 `bgra8`（`step = width * 4`），深度为 `32FC1`（米，无效像素为 0）；CameraInfo 是理想针孔相机：`plumb_bob`、`d` 为 5 个 0、`K` / `P` 由 `Fx`、`Fy`、`Cx`、`Cy` 得到、`R` 为单位阵。CPU 像素读回（会同步等待渲染）在 `bSkipWhenNoSubscribers` 开启时只在图像话题有订阅者时才做，读回失败时只记一次 Warning。
- **Odometry**：位姿和线速度经 `VectorFromUE` / `RotationFromUE`，角速度经 `AngularVelocityFromUE`，两个 6x6 协方差按镜像 `diag(+,-,+,-,+,-)` 换号。
- **IMU**：朝向、角速度、线加速度分别经 `RotationFromUE`、`AngularVelocityFromUE`、`VectorFromUE`，三个方差填入协方差对角线。

### 话题覆盖与校验

`TopicOverrides` 的键是逻辑名：`pose`、`odom`、`imu`、`scan`、`lidar`、`rgb_camera`、`rgb_camera.camera_info`、`depth_camera`、`depth_camera.camera_info`；同类型多个时用净化后的 `SensorId` 和 `<SensorId>.camera_info`。覆盖值按绝对话题处理（缺少前导 `/` 会补上，不再加命名空间）。

配置有问题时组件进入 `Error`，记录日志并触发 `OnROSBridgeError`，不发布任何话题：话题不是合法的绝对 ROS 名称、同一话题被分配两次、传感器的 `SensorId` 或 `FrameId` 为空、同类型内 `SensorId` 重复、传感器 `FrameId` 重复。组件只搜索所属 Actor 本身和它的组件，不搜索子 Actor；没有对应工厂的传感器会被跳过。

## TF 树

```
map                              由 SLAM / AMCL 发布，UBotROS 不发布
└── odom
    └── base_link                /tf（动态，来自里程计传感器）
        ├── imu_sensor           /tf_static
        ├── lidar_2d             /tf_static
        └── rgb_camera           /tf_static
            └── rgb_camera_optical   /tf_static（相机的光学系）
```

- **静态 TF** `base -> 传感器`：用 `IUBotSensor::GetSensorFrameComponent` 相对基组件的变换，在组件初始化（以及 `RefreshSensorPublishers`）时计算一次，去掉缩放；之后传感器相对机体移动不会更新。相机另外发布 `<frame> -> <frame>_optical`：零平移，旋转 `UBot::ROSConventions::CameraLinkToOpticalRotation()`，即 `(x=-0.5, y=0.5, z=-0.5, w=0.5)`。
- **里程计传感器没有静态 TF**：它的 `FrameId`（默认 `odom`）是里程计坐标系，也就是 base 的**父**坐标系，再发布 `base -> odom` 会和动态的 `odom -> base` 成环，并且与 SLAM / AMCL 的 `map -> odom` 打架。其他传感器的 frame 如果恰好等于里程计坐标系，会被跳过并记 Warning；frame 等于基坐标系的传感器不需要静态 TF。
- **动态 TF** `odom -> base`：取第一个里程计传感器，每个有效样本发布一条到 `/tf`。前提是该传感器跟踪的组件就是基组件，且它的 `OdometryFrameId` / `ChildFrameId` 与组件的 `OdometryFrame` / 基坐标系一致，否则记 Error 并关闭动态 TF（组件仍然 `Ready`）。
- **`/tf_static` 聚合**：每个 Sensor Bridge 通过 `SetStaticTransforms(Owner, ...)` 登记自己的静态变换，子系统把所有登记者合并成**一条**锁存消息，集合变化时和每次重连后重新发布。多个登记者设置同一个 child frame 时，后注册者的变换生效（内容不同会警告）。
- **`map` 坐标系不由 UBotROS 发布**。
- **`bPrefixFramesWithNamespace`**（默认关）：开启后所有 frame id 变成 `<namespace>/<frame>`（如 `robot1/base_link`、`robot1/odom`、`robot1/rgb_camera_optical`），已经以命名空间开头的不会重复加；世界参考的位姿所用的 `world` 不加前缀。同一个世界里有多个机器人时，`/tf`、`/tf_static`、`/clock` 是共享的，frame id 重名会冲突（静态 TF 里后注册者覆盖前者），请开启它或给每个机器人设置不同的 `BaseFrameOverride` 和传感器 `FrameId`。
- `bPublishDynamicTF` / `bPublishStaticSensorTF` 可以分别关闭两类 TF；基组件无效时静态 TF 被跳过并记 Error。

## 时间

- 所有时间戳都是 uBot 仿真时间 `UBot::Clock::GetSimTimeSeconds`，基准是 `UWorld::GetTimeSeconds()`（游戏时间，遵循暂停和时间膨胀），不是墙上时间；每个新的 PIE / Game 会话从 0 开始。
- **`/clock` 由 UBotROS 发布**：子系统在 `bPublishClock` 开启（默认）、桥已连接时，每个世界 Tick 发布一条 `rosgraph_msgs/msg/Clock`（reliable / volatile / 10）。子系统在暂停时也 Tick，所以暂停期间 `/clock` 继续以冻结的值发布；暂停时传感器不采样。运行中改 `ClockTopic` 会重建发布者。
- ROS 侧的节点要使用仿真时间：

  ```bash
  ros2 run <package> <executable> --ros-args -p use_sim_time:=true
  ros2 launch <package> <launch file> use_sim_time:=true   # 取决于 launch 文件是否暴露该参数
  ```

  新的 PIE 会话把时间重新从 0 开始，对 ROS 来说是时间回跳，TF 缓冲等通常会被清空。桥节点本身不使用时间戳。

## 坐标与时间约定

UE 是左手系（X 前、Y 右、Z 上，厘米），ROS（REP-103）是右手系（X 前、Y 左、Z 上，米、弧度），两者只差一个 Y 轴镜像加单位换算。这些换算分在两个地方：**轴和单位的换算在 uBotCore**（中立，不带 ROS 的名字，别的右手 Z-up 接入点也能用），**只有 ROS 才有的部分在本插件**。

| 内容 | 位置 | 规则 |
| --- | --- | --- |
| 位置、线速度、加速度 | uBotCore `UBot::RightHanded::PositionFromUE` / `VectorFromUE`（反向 `PositionToUE` / `VectorToUE`） | `(x, -y, z)`；位置另外厘米换成米 |
| 角速度、力矩 | uBotCore `AngularVelocityFromUE` / `AngularVelocityToUE` | `(-x, y, -z)` |
| 旋转 | uBotCore `RotationFromUE` / `RotationToUE` | 归一化的 `(-x, y, -z, w)`，退化输入得到单位四元数 |
| 变换（TF） | uBotCore `TransformFromUE` / `TransformToUE` | 旋转镜像，厘米换成米，去掉缩放 |
| 偏航角、扫描角 | uBotCore `YawFromUE` / `YawToUE` | 取反 |
| `builtin_interfaces/Time` | 本插件 `UBot::ROSConventions::SecondsToRosTime` / `RosTimeToSeconds` | 负数或非有限值得到 `(0, 0)`；纳秒四舍五入并进位；超出 int32 时饱和为 `(2147483647, 999999999)` |
| 相机光学系 | 本插件 `UBot::ROSConventions::CameraLinkToOpticalRotation` / `OpticalFrameSuffix` | 旋转 `(x=-0.5, y=0.5, z=-0.5, w=0.5)`，frame id 后缀 `_optical` |
| ROS 名字 | 本插件 `UBot::ROSConventions::SanitizeRosName` | 只保留 `[A-Za-z0-9_]`，其余换成 `_`，合并连续的下划线，首字符是数字时前面加 `_`，空名字得到 `ubot` |
| 6x6 协方差 | 本插件 `UBot::ROSConvert::ConvertCovariance6`（内部函数） | `C' = S C S`，`S = diag(+,-,+,-,+,-)` |
| LaserScan 的样本顺序 | 本插件 `UBot::ROSConvert::MakeLaserScan`（内部函数） | 倒序，见“话题与消息” |

C++ 里分别包含 Core 的 `UBotRightHandedConventions.h` 和本插件的 `UBotROSConventionHelpers.h`。本插件 `UBotROSMessageTypes.h` 里的 `UBot::Ros::Encode` / `UBot::Ros::Decode`（CDR 编解码）与这些换算无关，没有变化。发布和接收的数值与 0.1.2 相同，变的只是换算的来源。

### 蓝图节点

本插件的节点在类别 `uBot|ROS|Conventions`（`UUBotROSConventionsLibrary`），都是纯函数：

| 节点 | 作用 |
| --- | --- |
| Camera Link To Optical Rotation | 相机光学系（z 前、x 右、y 下）相对相机连杆系的旋转，ROS 约定 |
| Get Optical Frame Suffix | 光学系 frame id 的后缀 `_optical` |
| Seconds To ROS Time | 秒换成 `builtin_interfaces/Time` 的 `Sec` 和 `Nanosec` |
| ROS Time To Seconds | `Sec`、`Nanosec` 换成秒；负的 `Nanosec` 按 0 |
| Sanitize ROS Name | 把任意文本变成合法的 ROS 名字 |

UE 与右手系之间的轴换算节点在 uBotCore（类别 `uBot|Conventions`，`UE To Right-Handed Position` 等，搜索 `ROS` 也能找到）。uBotCore 0.1.3 起，那里旧的 `UE To ROS …` / `ROS To UE …` 节点已弃用，结果与新节点相同；Core 里的 5 个 ROS 专属节点（相机光学系、时间、名字）也已弃用，它们移到了本插件，名字不变。用着旧节点的蓝图照常运行，编译时会提示弃用，换成新节点即可。

## 速度指令

**uBot ROS Velocity Command** 订阅 `/<namespace>/cmd_vel`（`RobotNamespaceOverride` 与 Sensor Bridge 规则相同；`CommandTopicOverride` 非空时替代整个话题），QoS 为 best_effort / volatile / 10。它不含控制逻辑，只做解码、换算、限速、分发和超时。

| 属性 | 默认 | 说明 |
| --- | --- | --- |
| `bAutoStartReceiver` | 开 | `BeginPlay` 时订阅；关闭后调用 `RestartCommandReceiver` |
| `TargetComponent` | 空 | 接收指令的组件，必须实现 `IUBotVelocityCommandable`；为空则用 `UBot::Actuation::FindVelocityCommandable(Owner)`（Actor 本身，否则第一个实现它的组件） |
| `CommandTimeoutSeconds` | `0.5` | 超过这么多仿真秒没有新指令就停车；`0` 关闭超时 |
| `MaxLinearSpeedMetersPerSecond` | `0` | 线速度的向量模长上限，保持方向；`0` 不限制 |
| `MaxAngularSpeedRadiansPerSecond` | `0` | 角速度的向量模长上限，保持方向；`0` 不限制 |

**换算规则**（`UBot::ROSConvert::MakeVelocityCommand`，机体系，ROS → UE）：

| 量 | 映射 | 说明 |
| --- | --- | --- |
| `linear` | `(x, y, z)` → `(x, -y, z)` m/s | `VectorToUE` |
| `angular` | `(x, y, z)` → `(-x, y, -z)` rad/s | `AngularVelocityToUE`；ROS `angular.z = +0.25`（向左转）对应 UE `Z = -0.25`（UE 里 +Z 是向右偏航） |

得到的 `FUBotVelocityCommand` 带 `Source = "ROS"` 和接收时的仿真时间。流程：解码（畸形消息或含非有限值的消息被丢弃并触发 `OnCommandError`）→ 换算 → 限速 → 分发给目标的 `ApplyVelocityCommand` → 触发 `OnVelocityCommand`。目标拒绝指令（返回 `false`）只报告一次。没有目标时组件照常解码、触发事件并运行超时，方便用蓝图自己处理。

**超时**：组件的 Tick 只在有活动指令时开启（没有循环定时器）。超过 `CommandTimeoutSeconds` 的仿真时间没有新指令，目标收到**一次** `StopMotion`，随后触发 `OnCommandTimeout`；组件 `EndPlay` 时若指令仍然有效，也会让目标 `StopMotion`。暂停时仿真时间不前进，超时也不会触发。

蓝图接口：`IsCommandReceiverReady`、`IsCommandActive`、`GetLastCommand`、`GetResolvedCommandTopic`、`GetCommandError`、`GetCommandTarget`、`SetTargetComponent`、`RestartCommandReceiver`、`StopCommandReceiver`，事件 `OnVelocityCommand`、`OnCommandTimeout`、`OnCommandError`。

车辆或无人机要响应 `cmd_vel`，需要它自己（或其中的某个组件）实现 `IUBotVelocityCommandable`，控制器、油门 / 转向映射等都在实现里，不在 UBotROS 里：

```cpp
UCLASS()
class UMyDriveComponent : public UActorComponent, public IUBotVelocityCommandable
{
    GENERATED_BODY()

public:
    // 机体系 UE 轴，线速度 m/s，角速度 rad/s。返回 false 表示拒绝。
    virtual bool ApplyVelocityCommand_Implementation(const FUBotVelocityCommand& Command) override;
    virtual void StopMotion_Implementation() override;
};
```

## 飞行指令

研究飞控时，可以从 ROS 2 直接发姿态、角速度或单个电机的指令。两个组件的话题规则、`TargetComponent`、`CommandTimeoutSeconds`（默认 `0.5`）、错误事件和蓝图接口都与 uBot ROS Velocity Command 相同，同样不含控制逻辑。桥需要额外的消息包：`sudo apt install ros-humble-mavros-msgs ros-humble-actuator-msgs`。

| 组件 | 默认话题 | 消息 | 目标接口（uBotCore） |
| --- | --- | --- | --- |
| **uBot ROS Attitude Command** | `/<namespace>/cmd_attitude` | `mavros_msgs/msg/AttitudeTarget` | `IUBotAttitudeCommandable` |
| **uBot ROS Motor Command** | `/<namespace>/cmd_motors` | `actuator_msgs/msg/Actuators` | `IUBotMotorCommandable` |

**AttitudeTarget 的换算**（`UBot::ROSConvert::MakeAttitudeTargetCommand`）：

| `type_mask` | 结果 |
| --- | --- |
| 不含 `IGNORE_ATTITUDE`（128） | 姿态指令：`orientation`（ROS 世界系）经 `RotationToUE`；`body_rate` 不使用 |
| 含 `IGNORE_ATTITUDE` | 角速度指令：`body_rate`（FLU）经 `AngularVelocityToUE`；带 `IGNORE_ROLL_RATE`（1）、`IGNORE_PITCH_RATE`（2）、`IGNORE_YAW_RATE`（4）的轴为 0 |
| 不含 `IGNORE_THRUST`（64） | `thrust`（飞行器最大总推力的比例）截到 0..1 后使用 |
| 含 `IGNORE_THRUST` | 不给推力，飞行器自己保持高度 |

**Actuators 的换算**（`UBot::ROSConvert::MakeMotorCommand`）：`velocity[]` 有值时按电机转速（rad/s，负值按 0），否则用 `normalized[]`（截到 0..1）；`position[]` 不使用。值的个数与目标的 `GetMotorCount()` 不同时不下发，触发 `OnCommandError`。

**超时**：超过 `CommandTimeoutSeconds` 没有新指令时，目标收到一次 `ReleaseAttitudeCommand` / `ReleaseMotorCommand`，由飞行器自己兜底（uBotAerial 的物理无人机默认刹车悬停）。`EndPlay` 时指令仍有效也会释放。

```bash
# 姿态：机头转向 +90°（向左），推力 60%
ros2 topic pub -r 50 /drone/cmd_attitude mavros_msgs/msg/AttitudeTarget "{orientation: {z: 0.7071068, w: 0.7071068}, thrust: 0.6}"
# 角速度：绕 z 轴 1 rad/s（向左偏航），不给推力（保持高度）
ros2 topic pub -r 50 /drone/cmd_attitude mavros_msgs/msg/AttitudeTarget "{type_mask: 192, body_rate: {z: 1.0}}"
# 四个电机的转速（rad/s）
ros2 topic pub -r 100 /drone/cmd_motors actuator_msgs/msg/Actuators "{velocity: [800.0, 800.0, 800.0, 800.0]}"
```

## 扩展：为自定义传感器添加发布器

Sensor Bridge 为每个传感器选择扩展点 `UBotROS.SensorPublisher` 上支持它的、优先级最高的工厂（`GetPriority()`；相同优先级保持注册顺序，所以优先级 0 的内置工厂赢得平局，要替换内置传感器的发布器就返回大于 0 的优先级）。

下面为一个假想的距离传感器 `UMyRangeSensorComponent`（实现 `IUBotSensor`，每次采样触发 `OnRangeUpdated(const FMyRangeData&)`）发布 `sensor_msgs/msg/Range`。`UBotROSMessageTypes.h` 里没有这个消息的结构体，所以用 `FUBotCdrWriter` 手写；有结构体的消息（`FUBotRos…`，字段名与 ROS 一致）直接 `PublishMessage(TopicId, Message)`。

```cpp
// MyRangePublisher.h
UCLASS(Transient)
class UMyRangePublisher : public UUBotROSSensorPublisher
{
    GENERATED_BODY()

public:
    virtual bool Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError) override;
    virtual void Shutdown() override;

private:
    UFUNCTION()
    void HandleRange(const FMyRangeData& Data);

    int32 TopicId = 0;
};

// MyRangePublisher.cpp
bool UMyRangePublisher::Initialize(UObject* Sensor, const FUBotROSPublisherContext& Context, FString& OutError)
{
    UMyRangeSensorComponent* RangeSensor = Cast<UMyRangeSensorComponent>(Sensor);
    if (!RangeSensor || !Super::Initialize(Sensor, Context, OutError))   // Super 缓存传感器、桥和 frame id
    {
        return false;
    }

    // 逻辑键（TopicOverrides 的键）和默认路径 /<namespace>/range；同类型多个时改用 SensorId。
    const FString Key = IsSoleSensorOfType() ? FString(TEXT("range")) : GetSensorToken();
    TopicId = AdvertiseTopic(Key, Key, TEXT("sensor_msgs/msg/Range"), GetContext().SensorQos, OutError);
    if (TopicId <= 0)
    {
        Shutdown();
        return false;
    }
    RangeSensor->OnRangeUpdated.AddUniqueDynamic(this, &UMyRangePublisher::HandleRange);
    return true;
}

void UMyRangePublisher::Shutdown()
{
    if (UMyRangeSensorComponent* RangeSensor = Cast<UMyRangeSensorComponent>(GetSensor()))
    {
        RangeSensor->OnRangeUpdated.RemoveDynamic(this, &UMyRangePublisher::HandleRange);
    }
    TopicId = 0;
    Super::Shutdown();   // 取消声明所有话题
}

void UMyRangePublisher::HandleRange(const FMyRangeData& Data)
{
    if (!ShouldPublishSample(Data.Header) || !WantsMessage(TopicId))   // 样本无效，或没有订阅者时不编码
    {
        return;
    }

    FUBotRosHeader Header;
    Header.stamp = FUBotRosTime::FromSeconds(Data.Header.TimestampSeconds);
    Header.frame_id = GetSensorFrameId();   // 已净化，必要时带命名空间前缀

    FUBotCdrWriter Writer;
    Header.Serialize(Writer);
    Writer.WriteU8(1);   // radiation_type = INFRARED
    Writer.WriteF32(Data.FieldOfViewRadians);
    Writer.WriteF32(Data.MinRangeMeters);
    Writer.WriteF32(Data.MaxRangeMeters);
    Writer.WriteF32(Data.RangeMeters);
    PublishCdr(TopicId, Writer.Finish());
}

// 工厂和注册（模块 MyRangeROS，Build.cs 依赖 "UBotROS" 和 "UBotCore"）
class FMyRangePublisherFactory final : public IUBotROSSensorPublisherFactory
{
public:
    virtual FName GetExtensionName() const override { return FName(TEXT("MyRangeROS.Range")); }
    virtual bool SupportsSensor(const UObject* Sensor) const override { return Sensor && Sensor->IsA<UMyRangeSensorComponent>(); }
    virtual UUBotROSSensorPublisher* CreatePublisher(UObject* Outer, UObject* Sensor) const override
    {
        return NewObject<UMyRangePublisher>(Outer, NAME_None, RF_Transient);
    }
};

void FMyRangeROSModule::StartupModule()
{
    FUBotExtensionRegistry::Get().RegisterExtension(UBot::ROSPublishing::GetExtensionPointName(),
        FName(TEXT("MyRangeROS")), MakeShared<FMyRangePublisherFactory>());
}

void FMyRangeROSModule::ShutdownModule()
{
    FUBotExtensionRegistry::Get().UnregisterAllFromPackage(FName(TEXT("MyRangeROS")));
}
```

要点：

- 发布器由 Sensor Bridge 拥有，传感器的静态 TF 也由它按 `IUBotSensor::GetSensorFrameComponent` 自动处理。传感器 frame 不是机器人上的物理连杆时重写 `PublishesPhysicalFrame()` 返回 `false`；发布光学坐标系（相机）时重写 `UsesOpticalFrame()` 返回 `true`，并使用 `GetOpticalFrameId()`。
- `AdvertiseTopic` 经上下文里的 `ResolveTopic` 处理 `TopicOverrides`、话题校验和冲突检测；`GetContext()` 提供命名空间、QoS 等。消息类型必须是 ROS 侧已安装的接口，否则桥回 `unknown_type`。
- 不经过传感器组件，也可以直接用子系统发布和订阅：

  ```cpp
  if (UUBotROSBridgeSubsystem* Bridge = UUBotROSBridgeSubsystem::Get(this))
  {
      const int32 Id = Bridge->Advertise(TEXT("/robot1/battery"), TEXT("std_msgs/msg/Float32"), FUBotROSQos::State());
      FUBotCdrWriter Writer;
      Writer.WriteF32(0.87f);
      Bridge->Publish(Id, Writer.Finish());

      Bridge->Subscribe(TEXT("/robot1/cmd_vel"), FUBotRosTwist::TypeName(),
          FUBotROSQos(EUBotROSReliability::BestEffort, EUBotROSDurability::Volatile, 10),
          [](TConstArrayView<uint8> Cdr)   // 游戏线程；视图只在回调期间有效
          {
              FUBotRosTwist Twist;
              if (UBot::Ros::Decode(Cdr, Twist)) { /* ... */ }
          });
  }
  ```

## 性能与限制

- **发送队列（UE → 桥）**：best_effort 发布者只保留最新一条待发消息，旧的被替换并计入丢弃数；没有会话（未连接或重连中）时 best_effort 数据直接丢弃；reliable 数据按顺序排队，重连期间也排，总量受 `MaxQueuedBytes` 限制，超出时先丢弃最旧的 best_effort 数据，再拒绝新的 reliable 数据并警告（`Publish` 返回 `false`）；桥处于 `Disabled` 时什么都不排队。C++ 的 `GetStats()` 给出已发 / 已收帧数、丢弃数、拒绝数和队列字节数。
- **接收（桥 → UE）**：`MESSAGE` 在后台线程接收，在游戏线程 Tick 里按到达顺序分发；积压受 `MaxQueuedBytes` 限制，超出的新消息被丢弃。桥那一侧每个客户端有 128 MiB（`max_queued_bytes`）的发送队列，队列满时丢弃 best_effort 消息，慢客户端不会拖慢 ROS 执行器或其他客户端。
- **按订阅者编码**：桥每个客户端在订阅者数量变化时汇报 `SUB_COUNT`（最快每 0.5 秒一次）。`bSkipWhenNoSubscribers`（默认开）时，某话题的订阅者数为 0 就不编码，相机也不做 CPU 读回；在第一次汇报之前数量未知，照常发布。best_effort 话题在桥未连接时也不编码。
- **负载上限**：单帧负载默认 64 MiB（`MaxPayloadBytes`，两端要一致）。超出时 UE 拒绝并警告，桥丢弃并回 `message_too_large`。作为量级参考：`bgra8` 图像每帧 `宽 × 高 × 4` 字节（640x480 约 1.2 MB），点云每个点 22 字节。
- **没有额外的限频**：话题频率就是传感器的 `UpdateRateHz`。编码（包括点云逐点换算和图像拷贝）在游戏线程完成，socket 读写在后台线程。能持续传多少数据取决于 UE 与桥之间的链路（本机、WSL 转发或网络）、游戏线程上的编码耗时和 ROS 侧订阅者，请用 `ros2 topic hz` / `ros2 topic bw` 和桥日志里的 dropped 提示实测，必要时降低分辨率和采样频率。
- **连接与重连**：UE 每 2 秒发一次 PING，10 秒没有收到桥的任何数据就断开重连；一次连接尝试最长 3 秒。重连后重新声明全部发布者和订阅者（id 不变），重新发布 `/tf_static`，排队的 reliable 数据随后发出；已经交给 socket 的帧会丢失。UE 断开期间桥会销毁它的话题，ROS 侧的订阅者需要能处理发布者重新出现。
- **多个世界共用一个桥**：每个 Game / PIE 世界是独立的 TCP 连接，桥默认最多同时接受 32 个。不同世界发布同一个话题时共享桥里的同一个 ROS 发布者，类型和 QoS 必须一致，否则桥回 `type_conflict` / `qos_conflict`。
- **其他**：协议没有认证和加密；一个 Sensor Bridge 只搜索所属 Actor；消息类型只覆盖 `UBotROSMessageTypes.h` 里的那些，其余用 `FUBotCdrWriter` 手写；只验证了 ROS 2 Humble。

## 相对旧版桥接的修正

与旧版 `AgentSensorROSBridge` 相比，输出数据有这些**有意的修正**：

| 项目 | 旧版 | 现在 |
| --- | --- | --- |
| 角速度符号 | Odometry 的 `twist.angular` 和 IMU 的 `angular_velocity` 当作普通向量，用 `(x, -y, z)` | 角速度是轴向量，用 `AngularVelocityFromUE` = `(-x, y, -z)`：UE 里向右偏航现在得到负的 ROS `z`。Odometry 的 6x6 协方差按镜像 `diag(+,-,+,-,+,-)` 换号 |
| LaserScan | 角度和距离原样发布，而 UE 的扫描角向右为正、ROS 向左为正，所以扫描是镜像的；强度被清空 | `angle_min = YawFromUE(AngleMax)`、`angle_max = YawFromUE(AngleMin)`，`ranges` 和 `intensities` 倒序，未命中为 `+inf`，帧里有强度时发布 |
| PointCloud2 | 字段 `x`、`y`、`z`、`ring`、`time`，`point_step = 18`，没有强度 | 增加 `intensity`（float32，偏移 12），`ring` 在 16、`time` 在 18，`point_step = 22`；没有强度数据时为 0。按 `fields` 里的偏移读取的消费者不受影响，写死偏移的代码需要更新 |
| 相机坐标系 | Image / CameraInfo 用相机自己的 frame，没有光学系 TF | Image / CameraInfo 使用 `<frame>_optical`，并发布静态 TF `<frame> -> <frame>_optical`；REP-103 约定相机图像使用光学坐标系（z 前、x 右、y 下） |
| 里程计的静态 TF | 每个传感器（包括里程计传感器）都发布静态 `base_link -> frame`，里程计的 `odom` 与动态 `odom -> base_link` 成环 | 里程计传感器没有静态 TF |
| `/tf_static` | 每个组件通过自己的 ROS 节点各自发布 | 所有组件合并成一条锁存消息，变化时和重连后重新发布 |
| 换算 | 旧版转换器里自带 `ConvertVector`、`ConvertQuaternion`、`ConvertTimestamp` | 位置、向量、旋转、角速度、偏航角、变换的轴和单位换算走 Core 的 `UBot::RightHanded` / `UBot::Units`，时间、光学系、名字走本插件的 `UBot::ROSConventions`，协方差换号在 `ConvertCovariance6`；每种换算只有一份实现 |
| 默认命名空间 | 属性 `RobotNamespace`，默认 `robot` | 来自 uBot 身份（`UBot::Identity`），可由 `RobotNamespaceOverride` 覆盖 |
| 依赖 | TempoROS（每个组件一个 ROS 节点） | 没有 TempoROS、没有 rclcpp，每个世界一条 TCP 连接，ROS 节点只有桥 |

位置和四元数的映射（`(x, -y, z)`、`(-x, y, -z, w)`）与旧版相同；默认话题路径、逻辑键、校验规则和蓝图 API 保持不变。

## 从旧插件迁移

| 旧插件 | 现在用什么 |
| --- | --- |
| `AgentSensorROSBridge`：`Agent ROS Sensor Suite Publisher`（`UAgentROSSensorSuitePublisherComponent`） | **uBot ROS Sensor Bridge**（`UUBotROSSensorBridgeComponent`）。资产里的旧类、属性、结构、枚举和委托签名由 `Config/DefaultUBotROS.ini` 自动重定向，见下 |
| `AgentVehicleROSBridge`：`Agent ROS Vehicle Command` | **uBot ROS Velocity Command** + 车辆自己实现 `IUBotVelocityCommandable`。旧组件里的比例速度控制器、油门 / 刹车 / 转向映射和失效制动不在 UBotROS 里，要在目标里实现。**没有重定向**，手工替换组件 |
| `AgentDroneROSBridge`：`Agent ROS Drone Command` | 同上：uBot ROS Velocity Command + 无人机实现 `IUBotVelocityCommandable`，**没有重定向** |
| TempoROS | 不再需要：启动 `ubot_ros_bridge` |
| `AgentSensorCore` 的传感器 | 由 UBotSensor 替代（它有自己的重定向），Sensor Bridge 发现的是 UBotSensor 的组件 |

### CoreRedirects

重定向列表**只有一份**：插件目录里 `Config/DefaultUBotROS.ini` 的 `[CoreRedirects]` 段，共 11 条（工程的 `Config/DefaultUBotROS.ini` 存放项目设置，是另一个文件）。

| 类型 | 条数 | 内容 |
| --- | --- | --- |
| `PackageRedirects` | 1 | `/Script/AgentSensorROSBridge` 到 `/Script/UBotROS`，兜住没有单独重定向的内容 |
| `ClassRedirects` | 1 | `AgentROSSensorSuitePublisherComponent` 到 `UBotROSSensorBridgeComponent` |
| `PropertyRedirects` | 2 | `RobotNamespace` 到 `RobotNamespaceOverride`，`BaseLinkFrame` 到 `BaseFrameOverride` |
| `StructRedirects` | 2 | `AgentROSTopicOverride` 到 `UBotROSTopicOverride`，`AgentROSQOSOverride` 到 `UBotROSQosOverride` |
| `EnumRedirects` | 2 | `EAgentROSBridgeStatus` 到 `EUBotROSSensorBridgeStatus`，`EAgentROSReliability` 到 `EUBotROSReliability` |
| `ObjectRedirects` | 3 | 三个事件的动态委托签名 |

注意：

- 引擎在配置已启用的插件时加载这份 ini，所以重定向**只在编辑器启动时生效**；运行中新启用的插件需要重启编辑器。
- 重定向只处理序列化数据。引用 `UAgent*` 等旧名称的 C++ 代码要手工改名。旧资源第一次打开后请重新保存，迁移完成后可以从工程中移除旧插件。
- **`FAgentROSQOSOverride` {`bOverride`, `Reliability`, `QueueSize`}** 变成 `FUBotROSQosOverride` {`bOverride`, `Qos`}：旧的 `Reliability` 和 `QueueSize` 加载进只读不存的成员，再由 `PostSerialize` 移入 `Qos.Reliability` / `Qos.Depth`；覆盖开启时记一条 Warning（`Migrated a legacy QoS override`），重新保存后用新格式存储。
- **`RobotNamespace` 的默认值不同**：旧默认是 `robot`，新属性 `RobotNamespaceOverride` 默认为空，意思是改用 uBot 身份的命名空间。保持旧默认值的组件迁移后话题命名空间会改变，需要时显式填写 `RobotNamespaceOverride`。`BaseLinkFrame`（旧默认 `base_link`）同理，空值回落到身份的基坐标系，默认仍是 `base_link`。
- `NodeNameOverride` 没有后继（不再有每组件的 ROS 节点），旧值被忽略。
- 显示名与类别变了：`Agent ROS Sensor Suite Publisher` 变为 `uBot ROS Sensor Bridge`，类别 `Agent ROS Bridge|…` 变为 `uBot ROS|…`；Blueprint 函数名和事件名不变，引脚继续有效。
- 改动重定向时只编辑这个 ini；测试 `UBotROS.Redirects.Resolve` 和 `UBotROS.Redirects.QosOverrideValues` 会逐条检查。

## 协议

UE 与桥之间的 TCP 协议（UBRB v1）完整规范见 [`docs/PROTOCOL.md`](docs/PROTOCOL.md)，只读这份文档就能实现一个兼容的桥。要点：每帧 12 字节头（魔数 `UBRB`、版本 1、op、flags、小端负载长度），12 种 op（`HELLO`、`HELLO_ACK`、`ADVERTISE`、`UNADVERTISE`、`PUBLISH`、`SUBSCRIBE`、`UNSUBSCRIBE`、`MESSAGE`、`SUB_COUNT`、`PING`、`PONG`、`ERROR`），话题和类型作为 JSON 声明，消息体是带 4 字节封装头的完整 CDR。UE 是客户端，每次（重）连接先发 `HELLO`，再重新声明全部发布者和订阅者。修改协议行为时，该文档、UE 端（`UBotROSProtocol.*`、`UBotROSBridgeSubsystem.*`）和 Python 端（`protocol.py`、`bridge_node.py`）要一起改。

## 测试

### UE 自动化测试

测试名都以 `UBotROS.` 开头，共 87 项，分类为 `Cdr`（26，其中 `Cdr.Golden.*` 18 项）、`Convert`（17）、`Conventions`（4）、`Protocol`（4）、`Subsystem`（14）、`Factories`（3）、`SensorBridge`（3）、`VelocityCommand`（9）、`AttitudeCommand` 与 `MotorCommand`（共 3）、`Redirects`（2）和 `EndToEnd`（2）：

- `Cdr.*`：基本类型、对齐、字符串、序列、数组、读取越界保护；`Cdr.Golden.*` 把 `FUBotRos*` 结构编码后与 `rclpy.serialization.serialize_message` 生成的字节逐字节比较（`Time`、`Header`、`Imu`、`Odometry`、`LaserScan`、`PointCloud2`、`Image`、`CameraInfo`、`TfMessage`、`Clock`、`Twist`、`AttitudeTarget`、`Actuators`），`Twist` 还做解码往返。参考数据由 `ros2/tools/gen_golden.py` 在 ROS 2 环境里生成到 `Private/Tests/UBotROSGoldenData.h`（rmw_fastrtps 不初始化对齐填充，生成器会标出哪些字节是填充）。
- `Convert.*`：角速度符号、LaserScan 镜像（UE +30° 对应 ROS −30°）、PointCloud2 布局、协方差符号、Image / CameraInfo、光学系 TF、速度 / 姿态 / 电机指令换算。
- `Conventions.*`：相机光学系旋转（含随机 UE 相机朝向到光学系各轴的检查）和 `_optical` 后缀；`builtin_interfaces/Time` 的秒拆分（进位、饱和、非法输入、往返误差）；ROS 名字合法化（对照表加随机输入，输出必为合法名字）；`uBot|ROS|Conventions` 的 5 个蓝图节点（与 C++ 函数结果一致，类别、显示名和关键字正确）。
- `Protocol.*`：帧构造、增量解析（拆包、粘包）、非法头部拒绝、名称校验。
- `Subsystem.*`：对一个进程内的回环 TCP 测试服务器验证握手、声明与发布的帧格式、`SUB_COUNT`、`MESSAGE` 在游戏线程分发、队列策略、重连后重新声明、静态 TF 聚合、`/clock`、主机名解析，以及连接状态发布成运行时状态项和 `rosBridgeUnreachable` 问题（单个世界和多个世界）。
- `Factories.*`、`SensorBridge.*`、`VelocityCommand.*`、`AttitudeCommand.*`、`MotorCommand.*`、`Redirects.*`：工厂选择与优先级、话题与 TF、配置校验、限速 / 超时 / 目标选择、姿态与电机指令的分发和超时、CoreRedirects 与 QoS 迁移。
- `EndToEnd.Bridge`、`EndToEnd.FlightCommands`：需要真实的桥，未设置环境变量（分别是 `UBOT_ROS_E2E=1` 和 `UBOT_ROS_E2E_FLIGHT=1`）时直接通过并输出一条提示。

在启用了 UBotCore、UBotSensor、UBotROS 的工程里，用带编辑器上下文的 `UnrealEditor` 运行。编辑器内：`Tools` 菜单打开 Session Frontend 的 Automation 标签页，过滤 `UBotROS`。命令行（开发宿主工程是 `DevHost/UBotDev.uproject`）：

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotROS; Quit' '-TestExit=Automation Test Queue Empty'
```

把 `UBotROS` 换成更具体的前缀（如 `UBotROS.Subsystem`）可以只跑一类。

### Python 测试（桥）

在 WSL（Ubuntu-22.04，ROS 2 Humble）里运行，共 77 项：`test_protocol.py`（45，纯协议，不需要 ROS 图）、`test_bridge_loopback.py`（27，启动真实的桥进程，用 Python 客户端说协议并用 rclpy 验证发布 / 订阅）、`test_bridge_inprocess.py`（5，桥在测试进程内运行，用于执行器卡死、线程创建失败等故障注入）。

```bash
wsl.exe -d Ubuntu-22.04 -e bash -lc 'source /opt/ros/humble/setup.bash; cd <plugin>/ros2/ubot_ros_bridge && python3 -m pytest -q test'
```

### 端到端

UE 测试 `UBotROS.EndToEnd.Bridge` 与 `ros2/ubot_ros_bridge/test/e2e_check.py` 配合，夹具值定义在 `/ubot_e2e` 命名空间下（IMU、点云、`/tf_static`、`/clock`、`cmd_vel`），细节见桥的 README。步骤：

```bash
# WSL 终端 1：桥
source /opt/ros/humble/setup.bash
cd <plugin>/ros2/ubot_ros_bridge && python3 -m ubot_ros_bridge.bridge_node --port 8268
# WSL 终端 2：检查脚本（全部匹配时退出码 0，否则 1 并打印缺失项）
python3 test/e2e_check.py --timeout 30
```

```powershell
# Windows：设置环境变量后运行 UE 测试
$env:UBOT_ROS_E2E = '1'
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Project>\<Project>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotROS.EndToEnd.Bridge; Quit' '-TestExit=Automation Test Queue Empty'
```

UE 测试发布 IMU、点云、静态 TF 和 `/clock`，并订阅 `/ubot_e2e/cmd_vel`。检查脚本退出码 0 表示它在 `--timeout` 内收到并核对了 IMU、点云、`/tf_static` 和至少一条 `/clock`；UE 测试在 20 秒内收到脚本发布的 Twist（linear `(1.5, 0, 0)`，angular `(0, 0, 0.25)`）即通过。

`UBotROS.EndToEnd.FlightCommands` 不用检查脚本：桥需要 `mavros_msgs` 和 `actuator_msgs`，在 WSL 里用 `ros2 topic pub` 向 `/ubot_e2e/cmd_attitude` 和 `/ubot_e2e/cmd_motors` 发消息（命令写在该测试的源码注释里），设置 `UBOT_ROS_E2E_FLIGHT=1` 后运行，20 秒内两个组件都收到并换算出正确的指令即通过。

## 版本与许可

当前版本 **0.1.3**（Beta，`IsBetaVersion`），仓库 <https://github.com/AstreoX/uBotROS>。

Copyright 2026 AstreoX。本项目以 [Apache License 2.0](LICENSE) 发布，可以自由使用、修改和再分发（包括商业用途），再分发时需保留 `LICENSE` 与 `NOTICE` 文件，并注明你修改过的文件。
