# CLAUDE.md

Instructions for coding agents and contributors working on UBotROS (AstreoX/uBotROS). `README.md` and `ros2/ubot_ros_bridge/README.md` are the user manuals (written in Chinese); `docs/PROTOCOL.md` is the wire spec (English).

## 1. What this is

UBotROS is the ROS 2 adapter plugin of the uBot family for Unreal Engine 5.5. It publishes uBot sensors, `/tf`, `/tf_static` and `/clock` to ROS 2 and feeds `cmd_vel` to velocity-commandable actors, through a TCP bridge (`ros2/ubot_ros_bridge`, an rclpy node). It is a translation layer with no control logic.

- Depends on `UBotCore ^0.1.0` and `UBotSensor ^0.1.0` (`UBotROS.uplugin`: `Plugins` and `UBot.Requires`). `UBot.Provides`: `ROS.SensorPublishing`, `ROS.TF`, `ROS.Clock`, `ROS.VelocityCommand`.
- No third-party code in UE: `UBotROS.Build.cs` lists engine modules only (`Core`, `CoreUObject`, `Engine`, `DeveloperSettings`, `Sockets`, `Networking`, `Json`) plus the two uBot modules. Nothing links rclcpp. UE encodes ROS 2 CDR itself (`FUBotCdrWriter`, `FUBotRos*` structs); the bridge forwards the bytes without decoding them.

## 2. Layout

```
UBotROS.uplugin            VersionName, dependencies, "UBot" block
Config/DefaultUBotROS.ini  plugin CoreRedirects (project settings go to the host project's file of the same name)
Config/FilterPlugin.ini    extra files staged with the plugin
Source/UBotROS/            single Runtime module UBotROS (no editor module), log category LogUBotROS
  Public/                  exported API: UBotROSBridgeSubsystem.h, UBotROSSensorBridgeComponent.h,
                           UBotROSVelocityCommandComponent.h, UBotROSCommandReceiverComponent.h,
                           UBotROSAttitudeCommandComponent.h, UBotROSMotorCommandComponent.h,
                           UBotROSSensorPublisher.h, UBotROSSettings.h, UBotROSTypes.h,
                           UBotROSMessageTypes.h, UBotCdr.h
  Private/                 UBotROSProtocol.*, UBotROSBridgeSubsystem.cpp, UBotROSConverters.*,
                           UBotROSBuiltinPublishers.*, UBotROSFlightCommandComponents.cpp, UBotROSRuntimeStatus.*
  Private/Tests/           *Tests.cpp automation tests, UBotROSGoldenData.h (generated)
ros2/ubot_ros_bridge/      ament_python package: ubot_ros_bridge/{protocol,bridge_node}.py, launch/, test/
ros2/tools/gen_golden.py   generates Private/Tests/UBotROSGoldenData.h
docs/                      PROTOCOL.md, api.json (generated), brand/
```

## 3. Wire protocol (v1)

- Spec: `docs/PROTOCOL.md`. Protocol changes touch the spec, the UE side (`Source/UBotROS/Private/UBotROSProtocol.{h,cpp}`, `UBotROSBridgeSubsystem.cpp`) and the Python side (`ros2/ubot_ros_bridge/ubot_ros_bridge/protocol.py`, `bridge_node.py`) in one change.
- Frame: 12-byte header (magic `UBRB`, version, op, u16 flags, u32 payload length; little endian) plus payload. Ops 1-12: `UBot::ROSProtocol::EOp` / `protocol.Op` (HELLO, HELLO_ACK, ADVERTISE, UNADVERTISE, PUBLISH, SUBSCRIBE, UNSUBSCRIBE, MESSAGE, SUB_COUNT, PING, PONG, ERROR).
- Control payloads are compact JSON. PUBLISH/MESSAGE carry a u32 id followed by the complete CDR bytes (with the 4-byte encapsulation header), identical to `rclpy.serialization.serialize_message`. The bridge publishes with `publish(bytes)` and subscribes with `raw=True`.
- UE is the TCP client: `UUBotROSBridgeSubsystem`, one connection per Game/PIE world, socket I/O on a background thread (`FUBotROSBridgeClient`). Defaults: UE connects to `127.0.0.1:8268` (`UUBotROSSettings`), the bridge listens on `0.0.0.0:8268`.
- Connection: HELLO first; the session counts as connected only after HELLO_ACK with protocol 1. PING every 2 s; a side that receives nothing for 10 s drops the connection. One attempt times out after 3 s; retry every `ReconnectIntervalSeconds` (default 2 s) after a failed attempt or lost connection.
- Reconnect order (`BeginSession`): HELLO, every ADVERTISE (ascending id), every SUBSCRIBE (ascending id), reliable PUBLISHes queued while disconnected, then live traffic. Ids are never reused. The merged `/tf_static` is re-queued after HELLO_ACK arrives, so it follows the queued PUBLISHes.
- Queues: a best_effort publisher keeps only its newest pending message and drops data while no session is active; reliable messages queue in order up to `MaxQueuedBytes`. `MaxPayloadBytes` (UE) must equal `max_payload_bytes` (bridge).
- Framing and name validation are pure code (no sockets, threads or UObjects) so they can be unit tested alone.

## 4. Messages, topics and mappings

Topics without a leading slash are relative to `/<namespace>/`; with several sensors of one type the sanitized `SensorId` replaces the fixed path. QoS presets: `FUBotROSQos::SensorData/State/Latched/Tf` (`Public/UBotROSTypes.h`).

| Topic | ROS type | Dir | Converter (`UBot::ROSConvert`, `Private/UBotROSConverters.cpp`) | Wiring |
|---|---|---|---|---|
| `imu/data` | `sensor_msgs/msg/Imu` | out | `MakeImu` | `UUBotROSImuPublisher` |
| `odom` | `nav_msgs/msg/Odometry` | out | `MakeOdometry` | `UUBotROSOdometryPublisher` |
| `pose` | `geometry_msgs/msg/PoseStamped` | out | `MakePose` | `UUBotROSPosePublisher` |
| `scan` | `sensor_msgs/msg/LaserScan` | out | `MakeLaserScan` | `UUBotROSLidar2DPublisher` |
| `lidar/points` | `sensor_msgs/msg/PointCloud2` | out | `MakePointCloud2` | `UUBotROSLidar3DPublisher` |
| `camera/{color,depth}/{image_raw,camera_info}` | `sensor_msgs/msg/Image`, `CameraInfo` | out | `MakeImage`, `MakeCameraInfo` | `UUBotROSRgbCameraPublisher`, `UUBotROSDepthCameraPublisher` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | out | `MakeTransform`, `MakeOpticalTransform` | `UBotROSSensorBridgeComponent.cpp`; static set merged in `UUBotROSBridgeSubsystem::SetStaticTransforms` |
| `/clock` (`ClockTopic`) | `rosgraph_msgs/msg/Clock` | out | none | `UpdateClock` in `UBotROSBridgeSubsystem.cpp` |
| `cmd_vel` | `geometry_msgs/msg/Twist` | in | `MakeVelocityCommand` | `UUBotROSVelocityCommandComponent` |
| `cmd_attitude` | `mavros_msgs/msg/AttitudeTarget` | in | `MakeAttitudeTargetCommand` | `UUBotROSAttitudeCommandComponent` |
| `cmd_motors` | `actuator_msgs/msg/Actuators` | in | `MakeMotorCommand` | `UUBotROSMotorCommandComponent` |

Publishers live in `Private/UBotROSBuiltinPublishers.{h,cpp}`; message structs in `Public/UBotROSMessageTypes.h`.

Add a sensor mapping (steps follow the existing ones):
1. Message struct missing? Add `FUBotRos<Name>` to `UBotROSMessageTypes.h` (ROS field names and order, `TypeName()`, `Serialize`, `Deserialize`) and implement it in `UBotROSMessageTypes.cpp`.
2. Pin the bytes: add a case to `ros2/tools/gen_golden.py`, regenerate `UBotROSGoldenData.h` in a ROS 2 shell, add a `UBOTROS_GOLDEN_TEST` line with identical values in `UBotROSCdrTests.cpp`.
3. Add `Make<Name>` to `UBotROSConverters.{h,cpp}` (pure function, frame ids passed in) and a `UBotROS.Convert.<Name>` test.
4. Add `UUBotROS<Name>Publisher : UUBotROSSensorPublisher`: `Initialize` calls `AdvertiseTopic(...)` and binds the sensor delegate; the handler checks `ShouldPublishSample` and `WantsMessage`, then `PublishMessage`. Register a `TUBotROSBuiltinPublisherFactory<Sensor, Publisher>` in `RegisterBuiltinFactories` and add its name to `GetBuiltinFactoryNames`; update `UBotROS.Factories.BuiltinCoverage` (it asserts the factory count).
5. Update the topic tables in `README.md`. A new message package also needs an `<exec_depend>` in `ros2/ubot_ros_bridge/package.xml`; the bridge itself needs no per-type code.

For ROS to UE, derive from `UUBotROSCommandReceiverComponent` (subscription, topic and target resolution, timeout and error reporting; the attitude and motor command components in `Private/UBotROSFlightCommandComponents.cpp` are the examples): implement the topic leaf, message type, target interface, `HandlePayload` (`UBot::Ros::Decode`, a converter) and `ReleaseTarget`. `UUBotROSVelocityCommandComponent` predates the base class and keeps its own copy of that logic. Third-party sensors register an `IUBotROSSensorPublisherFactory` at extension point `UBotROS.SensorPublisher` instead.

## 5. Coordinates, units, time

- UE: left-handed, X forward, Y right, Z up, centimeters. ROS: right-handed, REP-103, meters, radians.
- Do not reimplement the conversions in this plugin. Use `UBot::Ros` (`UBotRosConventions.h`) and `UBot::Units` (`UBotUnits.h`) from UBotCore. Positions and vectors map to `(x, -y, z)`, quaternions to `(-x, y, -z, w)`, angular velocity to `(-x, y, -z)`, yaw and scan angles are negated. Plugin-local rules: `ConvertCovariance6` (`S C S`, `S = diag(+,-,+,-,+,-)`) and the reversed sample order in `MakeLaserScan`.
- Converters receive uBot data already in SI units on UE axes; only `FTransform` inputs (`MakeTransform`) are centimeters. `FUBotRos*::FromVector/FromQuat` copy components without converting.
- Timestamps are uBot sim time (`UBot::Clock::GetSimTimeSeconds`) via `UBot::Ros::SecondsToRosTime`, never wall time. Camera messages use `<frame>_optical`.
- Tests: `UBotROS.Convert.*`, `UBotROS.Cdr.Golden.*`.

## 6. Build and test

The plugin only builds inside a host UE 5.5 project that enables UBotCore, UBotSensor and UBotROS (clone them under `<Host>/Plugins/uBot/`, or list their folder in `AdditionalPluginDirectories` of the `.uproject`). Close the editor before building.

```powershell
& '<UE_5.5>\Engine\Build\BatchFiles\Build.bat' <Host>Editor Win64 Development '-Project=<Host>.uproject' -WaitMutex -NoHotReloadFromIDE
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -unattended -nop4 -nosplash -NullRHI '-ExecCmds=Automation RunTests UBotROS; Quit' '-TestExit=Automation Test Queue Empty'
```

- All automation tests start with `UBotROS.`; narrow with `UBotROS.Subsystem`, `UBotROS.Convert`, `UBotROS.Cdr`, and so on. `Subsystem` tests run against an in-process loopback TCP server in `UBotROSSubsystemTests.cpp`.
- `UBotROS.EndToEnd.Bridge` passes without doing anything unless `UBOT_ROS_E2E=1` is set. To run it: start the bridge, run `python3 test/e2e_check.py --timeout 30` from `ros2/ubot_ros_bridge`, then run the UE test with `$env:UBOT_ROS_E2E = '1'`.
- Bridge: targets ROS 2 Humble (the only distro verified). Run the node directly, or build with colcon from a workspace outside this repo; the direct path is the one the tests cover, the colcon, `ros2 run` and launch path is documented in the bridge README but not exercised by them.

```bash
# run the bridge: from ros2/ubot_ros_bridge
source /opt/ros/humble/setup.bash
python3 -m ubot_ros_bridge.bridge_node --port 8268
# or with colcon: link or copy ros2/ubot_ros_bridge into <ws>/src, then from <ws>
colcon build --packages-select ubot_ros_bridge && source install/setup.bash
ros2 run ubot_ros_bridge bridge --ros-args -p port:=8268
ros2 launch ubot_ros_bridge ubot_ros_bridge.launch.py port:=8268
# bridge tests: from ros2/ubot_ros_bridge, ROS 2 sourced (loopback and in-process tests skip without it)
python3 -m pytest -q test
# regenerate golden CDR data: from the repo root, ROS 2 sourced
# (needs ros-humble-mavros-msgs and ros-humble-actuator-msgs for the flight command cases)
python3 ros2/tools/gen_golden.py --output Source/UBotROS/Private/Tests/UBotROSGoldenData.h
```

Cloud sessions have no Unreal Engine and usually no ROS 2. They can edit code and run the pure protocol tests (`cd ros2/ubot_ros_bridge && python3 -m unittest discover -s test -p 'test_protocol.py'`), but cannot compile UE C++, run automation tests, run the bridge against rclpy or run the end-to-end check. Say which checks were not run in the PR; never claim tests passed that did not run.

## 7. Conventions

- C++: Unreal style, four spaces, Allman braces, PascalCase, `A/U/F/E/I` prefixes, `b`-prefixed booleans, `UBOTROS_API` on exported types. Public headers in `Public/`, implementations in `Private/`, tests in `Private/Tests/`. Doc comments on public API (they become tooltips in `docs/api.json`). Subsystem methods are game-thread only; never block the game thread on the socket.
- Tests: `IMPLEMENT_SIMPLE_AUTOMATION_TEST` named `UBotROS.<Group>.<Case>`, flags `EditorContext | EngineFilter`, wrapped in `#if WITH_DEV_AUTOMATION_TESTS`. Reflected test-only types go in headers compiled in every configuration (`UBotROSLegacyTestTypes.h`, `UBotROSVelocityCommandTestTypes.h`). Fix bugs with a focused regression test. `UBotROSGoldenData.h` is generated; do not edit it.
- Blueprint API is declared in `Public/` with `UFUNCTION(BlueprintCallable|BlueprintPure, Category=...)`, `UPROPERTY(EditAnywhere|BlueprintAssignable, ...)`, dynamic multicast delegates for events, `UENUM/USTRUCT(BlueprintType)`, and components as `UCLASS(ClassGroup=(uBot), meta=(BlueprintSpawnableComponent, DisplayName="uBot ROS ..."))`. The protocol, CDR, converter and message-struct code is plain C++ and not reflected.
- Blueprint-visible names (classes, functions, properties, enum values, struct members, delegate signatures, component display names) are what user Blueprints bind to. Renaming or removing one breaks user assets. Add new members instead. Existing CoreRedirects live only in `Config/DefaultUBotROS.ini` and are checked by `UBotROS.Redirects.*`; `FUBotROSQosOverride` keeps load-only `_DEPRECATED` members for the same reason.
- Python (`ros2/`): four spaces, type hints, `from __future__ import annotations`, double quotes, lines up to 100 columns, docstrings that state behavior (`gen_golden.py` uses single quotes; match the file you edit). Keep `protocol.py` free of ROS imports. Tests are `unittest` classes in `test/test_*.py`, runnable with pytest.
- Docs: update `README.md` (settings, topics) and `docs/PROTOCOL.md` section 11 (UE defaults) when behavior changes. `Config/FilterPlugin.ini` lists the extra files staged with a packaged plugin (README, LICENSE, NOTICE, `docs/brand/`).

## 8. docs/api.json

`docs/api.json` is the Blueprint API description exported by UBotCore's `UBotDocExport` commandlet. It is generated: never hand-edit it. Regenerate it after changing reflected API (anything in section 7's Blueprint list, including tooltips and categories) and before tagging a release. This needs a local UE install with the host project (flags are documented in the UBotCore README); the file contains Chinese fields, so export the zh-Hans pass first and merge it:

```powershell
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -run=UBotDocExport -Plugin=UBotROS -Out=Saved/Docs/UBotROS.zh.json -culture=zh-Hans -NullRHI
& '<UE_5.5>\Engine\Binaries\Win64\UnrealEditor-Cmd.exe' '<Host>.uproject' -run=UBotDocExport -Plugin=UBotROS -Out=<repo>/docs/api.json -MergeZh=Saved/Docs/UBotROS.zh.json -culture=en -NullRHI
```

Cloud sessions cannot regenerate it: note in the PR that it is stale.

## 9. Versioning and hygiene

- `VersionName` in `UBotROS.uplugin` is the plugin version; release tags are `vX.Y.Z` and equal it. Keep `ClientVersion` in `Private/UBotROSProtocol.h` equal to it (`UBotROS.Protocol.FrameBuild` checks) and regenerate `docs/api.json` (it records the version).
- The bridge package is versioned separately (`package.xml`, `setup.py`, `ubot_ros_bridge/__init__.py`; keep the three equal).
- The package index lives in the uBotCore repo (`Index/index.json`): a release needs a version entry there whose `ref` is the tag. Update `UBot.Requires` and `Provides` in the `.uplugin` and in the index together.
- Never commit `Binaries/`, `Intermediate/`, `Saved/`, `DerivedDataCache/`, or colcon `build/`, `install/`, `log/` (run colcon outside the repo).
- Conventional Commits, imperative and scoped: `fix(bridge): ...`, `feat(ros): ...`, `docs: ...`.
- Keep local paths, credentials and server addresses (beyond the localhost defaults) out of code, docs and commits.
