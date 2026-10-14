# 测试

以下章节原样取自此前的 README。

## 测试

### UE 自动化测试

测试名都以 `UBotROS.` 开头，共 75 项，分类为 `Cdr`（24，其中 `Cdr.Golden.*` 16 项）、`Convert`（15）、`Protocol`（4）、`Subsystem`（14）、`Factories`（3）、`SensorBridge`（3）、`VelocityCommand`（9）、`Redirects`（2）和 `EndToEnd`（1）：

- `Cdr.*`：基本类型、对齐、字符串、序列、数组、读取越界保护；`Cdr.Golden.*` 把 `FUBotRos*` 结构编码后与 `rclpy.serialization.serialize_message` 生成的字节逐字节比较（`Time`、`Header`、`Imu`、`Odometry`、`LaserScan`、`PointCloud2`、`Image`、`CameraInfo`、`TfMessage`、`Clock`、`Twist` 等），`Twist` 还做解码往返。参考数据由 `ros2/tools/gen_golden.py` 在 ROS 2 环境里生成到 `Private/Tests/UBotROSGoldenData.h`（rmw_fastrtps 不初始化对齐填充，生成器会标出哪些字节是填充）。
- `Convert.*`：角速度符号、LaserScan 镜像（UE +30° 对应 ROS −30°）、PointCloud2 布局、协方差符号、Image / CameraInfo、光学系 TF、速度指令换算。
- `Protocol.*`：帧构造、增量解析（拆包、粘包）、非法头部拒绝、名称校验。
- `Subsystem.*`：对一个进程内的回环 TCP 测试服务器验证握手、声明与发布的帧格式、`SUB_COUNT`、`MESSAGE` 在游戏线程分发、队列策略、重连后重新声明、静态 TF 聚合、`/clock`、主机名解析，以及连接状态发布成运行时状态项和 `rosBridgeUnreachable` 问题（单个世界和多个世界）。
- `Factories.*`、`SensorBridge.*`、`VelocityCommand.*`、`Redirects.*`：工厂选择与优先级、话题与 TF、配置校验、限速 / 超时 / 目标选择、CoreRedirects 与 QoS 迁移。
- `EndToEnd.Bridge`：需要真实的桥，未设置环境变量 `UBOT_ROS_E2E=1` 时直接通过并输出一条提示。

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
