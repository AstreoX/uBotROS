
# 从旧插件迁移与输出数据的修正

以下章节原样取自此前的 README。文档站的[从旧插件迁移](https://astreox.github.io/uBotDocs/develop/migration/)是面向用户的版本，这里保留了完整的修正表、重定向清单和迁移注意事项。

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
