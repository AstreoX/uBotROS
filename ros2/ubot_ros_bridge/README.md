# ubot_ros_bridge

UBotROS 插件的 ROS 2 端：一个很小的 rclpy 节点，监听 TCP（默认 `0.0.0.0:8268`），把 Unreal Engine
发来的、已经编码好的 ROS 2 CDR 消息**原样**发布到 ROS 2 话题上，并把 UE 订阅的话题（例如 `cmd_vel`）
转发回 UE。

```
UE (Windows/Linux)                         ROS 2 机器 / WSL
UBotROS 插件 ──TCP 8268──▶ ubot_ros_bridge (rclpy) ──▶ ROS 2 话题 (/tf, /clock, 传感器)
   ▲                                   │
   └──────── cmd_vel CDR ◀─────────────┘
```

UE 端不依赖 TempoROS、rclcpp 或任何第三方库；本包只依赖 `rclpy`、`rosidl_runtime_py` 和 Python 标准库。
消息不在桥里反序列化：发布用 `Publisher.publish(bytes)`（即 `publish_raw`），订阅用
`create_subscription(..., raw=True)`，所以任何已安装的消息类型都能透传。

## 环境要求

- ROS 2 Humble（其他使用 rclpy 3.x 的发行版理论上也可以），Python ≥ 3.8。
- 需要能被 `rosidl_runtime_py.utilities.get_message` 加载的消息包。UBotROS 用到
  `std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`tf2_msgs`、`rosgraph_msgs`；
  自定义消息需要先 `source` 对应工作空间。
- 不需要 colcon 和 C++ 编译器即可运行。

## 运行

### 直接运行（不需要 colcon）

```bash
source /opt/ros/humble/setup.bash
cd <UBotROS 插件>/ros2/ubot_ros_bridge
python3 -m ubot_ros_bridge.bridge_node --port 8268
```

### 作为 ament_python 包

```bash
mkdir -p ~/ubot_ws/src && ln -s <UBotROS 插件>/ros2/ubot_ros_bridge ~/ubot_ws/src/
cd ~/ubot_ws && colcon build --packages-select ubot_ros_bridge
source install/setup.bash
ros2 run ubot_ros_bridge bridge                       # 或带参数：--ros-args -p port:=8268
ros2 launch ubot_ros_bridge ubot_ros_bridge.launch.py port:=8268 bind_address:=0.0.0.0
```

包描述（`package.xml`、`setup.py`、`setup.cfg`、`resource`、`launch`）是完整的，但测试环境里没有 colcon，
`colcon build` / `ros2 run` / `ros2 launch` 这条路径没有实际跑过；上面“直接运行”的方式是测试覆盖的方式。

`Ctrl+C`（SIGINT）或 SIGTERM 会关闭监听、断开所有客户端、销毁全部发布者/订阅者后以退出码 0 结束。

### 与 Windows 上的 UE 配合（WSL）

桥在 WSL 中监听时，Windows 上的 UE 直接连接 `127.0.0.1:8268` 即可（WSL2 的 localhost 转发；
在 NAT 网络模式、`localhostForwarding=true` 下，桥监听默认的 `0.0.0.0` 和 `--bind-address 127.0.0.1`
两种情况都已验证）。UE 侧地址和端口在 `Project Settings > Plugins > uBot ROS`
（`BridgeHost` / `BridgePort`）中设置。跨机器部署时桥要监听对方可达的地址（默认 `0.0.0.0`），
并在防火墙上放行 TCP 8268。

## 参数

命令行参数提供默认值，ROS 参数（`--ros-args -p 名称:=值` 或 launch 文件）优先级更高。

| ROS 参数 | 命令行 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `bind_address` | `--bind-address` | `0.0.0.0` | 监听地址；含 `:` 时按 IPv6 处理 |
| `port` | `--port` | `8268` | 监听端口（`0` 表示随机端口，实际端口见启动日志） |
| `max_payload_bytes` | `--max-payload-bytes` | `67108864`（64 MiB） | 单帧负载上限，收发双向生效；需与 UE 的 `MaxPayloadBytes` 一致 |
| `max_queued_bytes` | `--max-queued-bytes` | `134217728`（128 MiB） | 每个客户端发送队列上限，见“发送队列”；小于 `max_payload_bytes + 12`（一个最大帧）时自动提高到该值并打印警告 |
| `idle_timeout_seconds` | `--idle-timeout-seconds` | `10.0` | 客户端连续这么久没有发来任何数据就断开并释放其全部实体；`0` 表示关闭。UE 每 2 s 发一次 PING，正常连接不受影响 |
| `hello_timeout_seconds` | `--hello-timeout-seconds` | `5.0` | 连接建立后这么久还没完成 HELLO 就断开（即使一直在发别的数据）；`0` 表示关闭 |
| `max_clients` | `--max-clients` | `32` | 同时接受的客户端数量，超出的连接收到 `too_many_clients` 错误后被关闭；`0` 表示不限制。一个 UE 世界（PIE/游戏）占用一个客户端 |

`idle_timeout_seconds` / `hello_timeout_seconds` 可以写成整数或小数（`-p idle_timeout_seconds:=5`）。`port` 必须在 0 到 65535 之间，`idle_timeout_seconds` / `hello_timeout_seconds` / `max_clients` 不能为负，`max_payload_bytes` 必须在 1024 到 4294967295 之间，否则启动失败并以退出码 1 结束。`--version` 打印 `ubot_ros_bridge <版本>` 后退出。

节点名为 `ubot_ros_bridge`；`ROS_DOMAIN_ID` 等环境变量照常生效。启动日志（info 级别）会打印实际监听地址、端口和生效的限制，形如 `ubot_ros_bridge 0.1.0 listening on 0.0.0.0:8268 (ROS humble, max payload 64 MiB, ...)`。

## 协议要点

完整规范见插件仓库的 `docs/PROTOCOL.md`。每帧格式：

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 4 | 魔数 `55 42 52 42`（"UBRB"） |
| 4 | 1 | 版本 = 1 |
| 5 | 1 | op |
| 6 | 2 | flags（u16 LE，保留，发送 0，接收时忽略） |
| 8 | 4 | 负载长度 N（u32 LE，≤ `max_payload_bytes`） |
| 12 | N | 负载 |

| op | 名称 | 方向 | 负载 |
| --- | --- | --- | --- |
| 1 | HELLO | C→S | `{"protocol":1,"client":"uBot UE <版本>"}` |
| 2 | HELLO_ACK | S→C | `{"protocol":1,"server":"ubot_ros_bridge <版本>","ros_distro":"humble"}` |
| 3 | ADVERTISE | C→S | `{"id":u32,"topic":"/abs/topic","type":"sensor_msgs/msg/Imu","qos":Q}` |
| 4 | UNADVERTISE | C→S | `{"id":u32}` |
| 5 | PUBLISH | C→S | u32 LE 发布者 id + 完整 CDR（含 4 字节封装头） |
| 6 | SUBSCRIBE | C→S | `{"id":u32,"topic":...,"type":...,"qos":Q}` |
| 7 | UNSUBSCRIBE | C→S | `{"id":u32}` |
| 8 | MESSAGE | S→C | u32 LE 订阅 id + 完整 CDR |
| 9 | SUB_COUNT | S→C | `{"counts":{"<id>":n,...}}` |
| 10 / 11 | PING / PONG | 双向 | 空 |
| 12 | ERROR | S→C | `{"id":u32 或 null,"code":"...","message":"..."}` |

`Q` = `{"reliability":"reliable"|"best_effort","durability":"volatile"|"transient_local","depth":int}`，
映射为 `KEEP_LAST(depth)` 加对应的 reliability / durability；缺省字段按 reliable / volatile / 10 处理，
`depth` 必须 ≥ 1。

桥的行为：

- **握手**：客户端必须先发 HELLO。之前收到的其他请求回 `hello_required` 错误并忽略（PING/PONG 除外；PUBLISH 只报一次）。
  HELLO 中 `protocol` 不是 1 时回 `unsupported_protocol` 并关闭连接。
- **话题与类型**：话题必须是绝对名称（以 `/` 开头）；类型写作 `pkg/msg/Name`（也接受 `pkg/Name`）。
- **共享发布者**：同一话题在桥内只有一个 rclpy 发布者，被所有客户端的 ADVERTISE 共享并引用计数，
  最后一个 UNADVERTISE（或断开）时销毁。同话题不同类型 → `type_conflict`；同类型不同 QoS →
  `qos_conflict`。订阅也参与类型冲突检查（同一话题在桥内只能有一种类型）。
- **SUB_COUNT**：每个客户端已 ADVERTISE 的 id 对应的订阅者数量（`get_subscription_count()`，
  包括桥自身为其他客户端创建的订阅）；内容变化时发送，最快每 0.5 s 一次，总是包含该客户端当前全部 id。
- **PING/PONG 与超时**：收到 PING 立即回 PONG；桥本身不主动 PING。桥在读线程里执行 PROTOCOL.md 5.6 的空闲规则：
  客户端连续 `idle_timeout_seconds`（默认 10 s）没有发来任何字节就断开，并释放它注册的全部实体
  （只在“没有任何可读数据”时计时，所以内核缓冲里已有的 PING 不会造成误判；套接字本身保持阻塞，
  因此不会影响写线程的 `sendall`）。没有在 `hello_timeout_seconds`（默认 5 s）内完成 HELLO 的连接也会被断开。
  另外对接受的套接字启用 TCP keepalive（空闲 10 s 开始探测，间隔 5 s，3 次失败），所以对端不发 FIN/RST 就消失
  （断网、崩溃）时不会留下僵尸连接。
- **帧大小限制**：HELLO 之前每帧最多 4 KiB；之后 JSON 类操作（HELLO、ADVERTISE、UNADVERTISE、SUBSCRIBE、
  UNSUBSCRIBE）最多 64 KiB，PING/PONG 和其他操作最多 4 KiB，只有 PUBLISH 可以到 `max_payload_bytes`。
  超出视为帧头错误并断开连接。负载缓冲随实际收到的字节增长，所以一个只发了 12 字节帧头、声称 64 MiB 的连接
  不会让桥预先分配 64 MiB。
- **错误回复限流**：每个连接最多一次性回复 30 个 ERROR，之后每秒 10 个；超出部分只在日志里汇总
  （最多每 5 s 一条），10 s 内被抑制的错误超过 1000 个会让该连接被断开（`too many protocol errors`）。
  回复和日志中引用的客户端文本（话题、类型、客户端名称）会被截断并转义控制字符。
- **类型检查**：先在 ament 索引（`rosidl_interfaces` 资源）里确认 `pkg/msg/Name` 是已安装的接口，再用
  `get_message` 加载；因此对端无法借类型名让桥 `import` 任意 Python 模块（如 `this/msg/Foo`）。
- **安全**：协议没有认证。桥默认监听 `0.0.0.0`（跨机器连接需要），UE 与桥在同一台机器上（包括 Windows 到 WSL2 的
  localhost 转发）时，请用 `--bind-address 127.0.0.1`；在不可信网络中也可以用防火墙限制访问，并保持 `max_clients` 为有限值。
- **未知 op** → `unknown_op`；客户端发送仅由桥发送的 op → `unexpected_op`；连接保持。
- **帧头错误**（魔数、版本、超长）→ 尽力回一个 `protocol_error` 后关闭连接。
- **断开**：释放该客户端的全部发布者引用和订阅。客户端重连后需重新 ADVERTISE / SUBSCRIBE。

### 发送队列（桥 → 客户端）

每个客户端一个按字节计的有界队列，由独立的写线程发送，所有写操作经过每客户端的锁串行化；
一个慢客户端不会阻塞 ROS 执行器或其他客户端。队列满时：

1. best_effort 订阅的 MESSAGE 帧直接丢弃；
2. reliable 订阅的 MESSAGE 帧先挤掉队列中最旧的 best_effort 帧，仍放不下则丢弃该帧；
3. 控制帧（HELLO_ACK、PONG、ERROR、SUB_COUNT）总是入队，超过上限 1 MiB 余量时认为客户端已停止读取并断开。

丢帧以 warn 日志汇报（每客户端最多每 5 s 一条）。超过 `max_payload_bytes` 的 ROS 消息不会转发，
并对该订阅 id 回一次 `message_too_large`。

### ERROR 代码

| code | 含义 |
| --- | --- |
| `bad_request` | 负载不是 JSON 对象（包括嵌套过深）或 `id` 缺失/不是 u32 |
| `unsupported_protocol` | HELLO 的 `protocol` ≠ 1（随后断开） |
| `hello_required` | 在 HELLO 之前发送了请求 |
| `unknown_op` / `unexpected_op` | 未定义的 op / 只应由桥发送的 op |
| `invalid_topic` | 话题不是合法的绝对 ROS 2 名称 |
| `unknown_type` | 类型名格式错误、不是已安装的接口或无法加载 |
| `invalid_qos` | QoS 字段取值非法 |
| `duplicate_id` | 本连接上该 id 已存在 |
| `unknown_id` | UNADVERTISE/UNSUBSCRIBE/PUBLISH 引用了不存在的 id（PUBLISH 每个 id 只报一次） |
| `type_conflict` / `qos_conflict` | 与桥内同话题已有实体冲突 |
| `bad_payload` | PUBLISH 负载短于 4 字节 id 或 CDR 短于 4 字节封装头 |
| `publish_failed` | rclpy 发布失败（每个 id 只报一次） |
| `message_too_large` | 订阅到的消息超过负载上限（每个订阅只报一次） |
| `ros_error` | 创建发布者/订阅者失败 |
| `protocol_error` | 帧头错误（随后断开） |
| `internal_error` | 桥正在关闭或 ROS 执行器无响应 |
| `too_many_clients` | 已达到 `max_clients`（发送后立即关闭连接） |

## 线程模型

- rclpy 执行器（`SingleThreadedExecutor`）运行在独立线程；所有创建/销毁发布者和订阅者的操作都通过
  guard condition 投递到该线程执行，读线程等待结果，从而保证同一连接内请求的顺序（ADVERTISE 返回后
  紧跟的 PUBLISH 一定能找到发布者）。
- 一个接受连接的线程；每个客户端一个读线程（解析帧、直接调用 `publish`、检查空闲/HELLO 期限）和一个写线程。
  接受线程对单个连接的任何初始化失败（例如无法创建线程）都只丢弃该连接，不会退出；主线程也会监视它，
  万一退出则以退出码 1 结束，交给 launch/systemd 重启。
- 客户端断开时释放其实体的操作不会因 10 s 的执行器超时而取消：执行器被卡住时它留在队列里，恢复后按顺序执行
  （只有 ADVERTISE/SUBSCRIBE/UNADVERTISE/UNSUBSCRIBE 这类有应答的请求在超时时才会取消并回 `internal_error`）。
- 连接、断开、advertise/subscribe 以 info 级别记录，错误以 warn 级别记录。

## 测试

在包目录下、已 `source /opt/ros/humble/setup.bash` 的环境中：

```bash
python3 -m pytest test/test_protocol.py          # 纯协议单元测试，不需要 ROS 图
python3 -m pytest test/test_bridge_loopback.py   # 启动真实的桥进程做回环测试
python3 -m pytest test/test_bridge_inprocess.py  # 桥在测试进程内运行：执行器卡死、线程创建失败、套接字选项
python3 -m pytest                                # 全部（也可用 python3 -m unittest discover -s test）
```

共 77 项：`test_protocol.py` 45、`test_bridge_loopback.py` 27、`test_bridge_inprocess.py` 5。回环和进程内测试需要 ROS 2 环境；没有 ROS 时它们会被跳过，只有协议单元测试会运行。

回环测试以 `python3 -m ubot_ros_bridge.bridge_node` 启动桥（每个测试类使用独立的随机
`ROS_DOMAIN_ID`），用 Python 客户端说协议，覆盖：HELLO/HELLO_ACK、PING/PONG、未知 op；
ADVERTISE `sensor_msgs/msg/Imu` 后用 `serialize_message` 生成的 CDR 发布并验证 rclpy 订阅者收到完全相同的消息；
SUBSCRIBE → MESSAGE（字节级一致）、transient_local 锁存、SUB_COUNT 随订阅者变化；
共享发布者引用计数、各类冲突与错误码；断开后实体释放；帧头错误断开；慢客户端丢帧且不影响其他客户端；
空闲/未 HELLO 连接被断开、被阻塞的写线程不受空闲逻辑影响、客户端数量上限；嵌套过深的 JSON 回 `bad_request`；
类型名不会导入任意模块；错误回复限流与客户端文本截断；SIGINT/SIGTERM 干净退出；以及下面的端到端检查脚本。
进程内测试（`test_bridge_inprocess.py`）把 `OP_TIMEOUT_SECONDS` 调小并卡住执行器，验证断开时的释放不会丢失，
并验证接受线程在单个连接初始化失败后仍然存活。

### 端到端检查（契约第 12 节）

`test/e2e_check.py` 配合 UE 自动化测试 `UBotROS.EndToEnd.Bridge`（需设置环境变量 `UBOT_ROS_E2E=1`）：

```bash
# 终端 1（WSL）
python3 -m ubot_ros_bridge.bridge_node --port 8268
# 终端 2（WSL）
python3 test/e2e_check.py --timeout 30
# 然后在 Windows 上运行 UE 测试 UBotROS.EndToEnd.Bridge
```

脚本在 `/ubot_e2e` 命名空间下检查：

- `/ubot_e2e/imu`（`sensor_msgs/msg/Imu`）：frame_id `e2e_imu`，时间戳 12.5 s，orientation (0,0,0,1)，
  angular_velocity (0.1,0.2,0.3)，linear_acceleration (0,0,9.81)；
- `/ubot_e2e/points`（`sensor_msgs/msg/PointCloud2`）：frame_id `e2e_lidar`，uBot 布局
  （x/y/z/intensity float32 @0/4/8/12，ring uint16 @16，time float32 @18，point_step 22，小端），
  点 (1,2,3) (4,5,6) (7,8,9)，intensity 10/20/30，ring 0/1/2，time 0；
- `/tf_static`：`base_link` → `e2e_imu`，平移 (1,2,3)，单位旋转（以 transient_local 订阅）；
- `/clock`：至少一条。

同时以 10 Hz 在 `/ubot_e2e/cmd_vel` 发布 Twist：linear (1.5,0,0)，angular (0,0,0.25)。全部检查通过后，
脚本继续发布直到有订阅者匹配满 `--cmd-vel-linger` 秒（默认 3 s，受超时限制），保证 UE 测试能收到指令。

退出码：`0` 表示超时内全部收到且数值一致（浮点比较容差 `--tolerance`，默认 1e-6）；`1` 表示缺项、
数值不符或被中断——每个缺项都会打印原因（未收到时附带图上该话题的发布者及其 QoS，收到但不符时给出
最后一次的差异），例如：

```
[e2e] MISSING  imu       /ubot_e2e/imu [sensor_msgs/msg/Imu]: 40 message(s) received, none matched; last problem: angular_velocity = (0.1, -0.2, 0.3) expected (0.1, 0.2, 0.3)
[e2e] MISSING  tf_static /tf_static [tf2_msgs/msg/TFMessage]: no message received (publishers: ubot_ros_bridge [tf2_msgs/msg/TFMessage] reliability=reliable durability=volatile)
[e2e] RESULT: FAIL after 30.0 s, missing: imu, tf_static
```

## 故障排查

- **UE 一直连接不上**：确认桥的启动日志 `listening on <地址>:8268` 里的地址和端口与 UE 的 `BridgeHost` /
  `BridgePort` 对得上；跨机器时桥要监听 `0.0.0.0` 或对方可达的地址；检查防火墙与端口占用（端口被占用时桥会打印错误并以退出码 1 结束）。
- **`unknown_type`**：对应消息包未安装或没有 `source` 自定义消息所在的工作空间。
- **ROS 侧收不到数据**：检查 QoS——best_effort 发布者只能被 best_effort 订阅者接收；`/tf_static` 是
  transient_local，订阅端也要用 transient_local 才能拿到锁存消息。也要确认双方 `ROS_DOMAIN_ID` 一致。
- **`qos_conflict` / `type_conflict`**：同一话题被不同客户端以不同 QoS/类型声明，统一 UE 侧配置即可。
- **日志中出现 dropped**：客户端读取太慢或网络带宽不足（例如大图像）；降低发布频率或分辨率，
  或增大 `max_queued_bytes`。
- **日志中出现 `idle timeout` / `no HELLO`**：客户端在 `idle_timeout_seconds` / `hello_timeout_seconds` 内没有发数据
  （UE 被调试器暂停、网络中断）；UE 会自动重连。需要长时间断点调试时可用 `--idle-timeout-seconds 0`。
- **`too_many_clients`**：已有 `max_clients` 个连接（常见原因是关闭过的 PIE 会话尚未释放，或多个 UE 进程共用一个桥）。

## 文件结构

```
package.xml / setup.py / setup.cfg / resource/ubot_ros_bridge   ament_python 包描述
ubot_ros_bridge/protocol.py      协议编解码、增量帧解析、请求校验、发送队列与 SUB_COUNT 节流（纯 Python）
ubot_ros_bridge/bridge_node.py   rclpy 节点、TCP 服务器、客户端连接（入口 main，控制台脚本 bridge）
launch/ubot_ros_bridge.launch.py launch 文件（参数 bind_address、port、max_payload_bytes、max_queued_bytes、
                                 idle_timeout_seconds、hello_timeout_seconds、max_clients）
test/test_protocol.py            协议单元测试（pytest / unittest，无需 ROS 图）
test/test_bridge_loopback.py     真实桥进程的回环测试（需要 ROS 2）
test/test_bridge_inprocess.py    进程内测试（私有 ROS context，可改常量、注入故障）
test/e2e_check.py                端到端检查脚本（契约第 12 节）
```
