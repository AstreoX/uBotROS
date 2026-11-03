#!/usr/bin/env python3
"""Instrument the real bridge and probe TF for UBotROS.EndToEnd.SensorTiming.

Run the bridge and probe subcommands in a dedicated ROS_DOMAIN_ID, then run UE with
UBOT_ROS_TIMING=1, UBOT_ROS_TIMING_PORT and UBOT_ROS_TIMING_OUT configured. Stop
the probes with SIGTERM after UE finishes and use analyze to join all three traces.
The bridge observer delegates each unmodified payload to the production handler.
"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from pathlib import Path


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def save(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
    temporary.replace(path)


def bridge(args):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from ubot_ros_bridge import bridge_node as impl

    events = []
    message_types = {}
    original = impl.ClientConnection._on_publish

    def observe(connection, payload):
        pub_id, cdr = impl.proto.split_id_payload(payload)
        shared = connection.publications.get(pub_id)
        if shared is not None:
            if shared.type_name not in message_types:
                message_types[shared.type_name] = get_message(shared.type_name)
            message = deserialize_message(bytes(cdr), message_types[shared.type_name])
            if hasattr(message, "transforms"):
                for transform in message.transforms:
                    events.append(dict(kind="tf_static" if shared.topic == "/tf_static" else "tf",
                                       frame=transform.child_frame_id,
                                       stamp_ns=stamp_ns(transform.header.stamp), wall=time.monotonic()))
            elif hasattr(message, "header"):
                kind = {"sensor_msgs/msg/PointCloud2": "cloud", "nav_msgs/msg/Odometry": "odom",
                        "sensor_msgs/msg/Imu": "imu"}.get(shared.type_name, "other")
                events.append(dict(kind=kind, topic=shared.topic, frame=message.header.frame_id,
                                   stamp_ns=stamp_ns(message.header.stamp), wall=time.monotonic()))
        return original(connection, payload)

    impl.ClientConnection._on_publish = observe
    try:
        return impl.main(["sensor_timing_bridge", "--port", str(args.port)])
    finally:
        impl.ClientConnection._on_publish = original
        save(args.out, dict(events=events))


def probe(args):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    stopped = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stopped.set())
    events = []
    buffer = Buffer(cache_time=Duration(seconds=30.0))
    tf_node = rclpy.create_node("ubot_timing_tf")

    class Listener(TransformListener):
        def __del__(self):
            # Shutdown below is explicit; do not run the base destructor again after destroying nodes.
            pass

        def callback(self, message):
            super().callback(message)
            now = time.monotonic()
            for transform in message.transforms:
                events.append(dict(kind="tf", frame=transform.child_frame_id,
                                   stamp_ns=stamp_ns(transform.header.stamp), wall=now))

    listener = Listener(buffer, tf_node, spin_thread=True)
    sensor_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    nodes = []
    executors = []
    threads = []
    pending = []

    def lookup(message, timeout=0.0):
        namespace = message.header.frame_id.rsplit("/", 1)[0]
        transform = buffer.lookup_transform(namespace + "/odom", message.header.frame_id,
                                            Time.from_msg(message.header.stamp),
                                            timeout=Duration(seconds=timeout))
        return transform.transform.translation.x

    def cloud_callback(mode):
        def callback(message):
            now = time.monotonic()
            system_start_ns = time.time_ns()
            entry = dict(kind="cloud", mode=mode, frame=message.header.frame_id,
                         stamp_ns=stamp_ns(message.header.stamp), wall=now,
                         points=message.width * message.height)
            try:
                entry["x"] = lookup(message, 0.2 if mode == "wait_200ms" else 0.0)
                entry["ok"] = True
            except TransformException as error:
                entry["ok"] = False
                entry["error"] = str(error)
            entry["callback_ms"] = (time.monotonic() - now) * 1000.0
            entry["system_elapsed_ms"] = (time.time_ns() - system_start_ns) / 1.e6
            events.append(entry)
            if mode == "zero" and not entry["ok"]:
                pending.append((message, entry, now + 1.0))
        return callback

    def retry():
        for message, entry, deadline in pending[:]:
            try:
                x = lookup(message)
                entry["eventual_ok"] = True
                entry["eventual_x"] = x
                entry["recovery_ms"] = (time.monotonic() - entry["wall"]) * 1000.0
            except TransformException:
                if time.monotonic() < deadline:
                    continue
                entry["eventual_ok"] = False
            pending.remove((message, entry, deadline))

    for mode in ("zero", "wait_200ms"):
        node = rclpy.create_node("ubot_timing_" + mode)
        for index in range(30):
            namespace = "/ubot_timing/c{:02d}".format(index)
            node.create_subscription(PointCloud2, namespace + "/lidar/points", cloud_callback(mode), sensor_qos)
            if mode == "zero":
                for kind, msgtype, topic in (("odom", Odometry, "/odom"), ("imu", Imu, "/imu/data")):
                    def record(message, kind=kind, namespace=namespace):
                        events.append(dict(kind=kind, namespace=namespace.lstrip("/"),
                                           stamp_ns=stamp_ns(message.header.stamp), wall=time.monotonic()))
                    node.create_subscription(msgtype, namespace + topic, record, sensor_qos)
        if mode == "zero":
            node.create_timer(0.005, retry)
            if args.rtabmap_cases:
                from rtabmap_msgs.msg import Info
                for index in map(int, args.rtabmap_cases.split(",")):
                    for strategy in ("zero", "wait"):
                        topic = "/ubot_timing/rt_c{:02d}_{}/info".format(index, strategy)
                        def record_info(message, index=index, strategy=strategy):
                            events.append(dict(kind="rtabmap", case=index, strategy=strategy,
                                               stamp_ns=stamp_ns(message.header.stamp), wall=time.monotonic()))
                        node.create_subscription(Info, topic, record_info, sensor_qos)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        nodes.append(node)
        executors.append(executor)
        threads.append(thread)
    print("TIMING_PROBE_READY", flush=True)
    try:
        stopped.wait()
    finally:
        for executor in executors:
            executor.shutdown()
        for thread in threads:
            thread.join()
        listener.executor.shutdown()
        listener.dedicated_listener_thread.join()
        listener.unregister()
        for node in nodes + [tf_node]:
            node.destroy_node()
        rclpy.shutdown()
        save(args.out, dict(events=events))
    return 0


def analyze(args):
    ue = json.loads(Path(args.ue).read_text(encoding="utf-8-sig"))
    wire = json.loads(Path(args.wire).read_text())["events"]
    ros = json.loads(Path(args.ros).read_text())["events"]
    rows = []
    errors = []
    for case in ue["cases"]:
        namespace = case["namespace"]
        events = case["events"]
        clouds = [e for e in events if e["kind"] == "cloud" and e["measured"]]
        stamps = {e["stamp_ns"] for e in clouds}
        odom_stamps = {e["stamp_ns"] for e in events if e["kind"] == "odom" and e["valid"]}
        zero = {e["stamp_ns"]: e for e in ros if e["kind"] == "cloud" and e["mode"] == "zero"
                and e["frame"] == namespace + "/lidar" and e["stamp_ns"] in stamps}
        waiting = {e["stamp_ns"]: e for e in ros if e["kind"] == "cloud" and e["mode"] == "wait_200ms"
                   and e["frame"] == namespace + "/lidar" and e["stamp_ns"] in stamps}
        wire_tf = {e["stamp_ns"]: index for index, e in enumerate(wire)
                   if e["kind"] == "tf" and e["frame"] == namespace + "/base_link"}
        wire_clouds = {e["stamp_ns"]: index for index, e in enumerate(wire)
                       if e["kind"] == "cloud" and e["frame"] == namespace + "/lidar"}
        native_odom = {e["stamp_ns"]: i for i, e in enumerate(events) if e["kind"] == "odom" and e["valid"]}
        native_cloud = {e["stamp_ns"]: i for i, e in enumerate(events) if e["kind"] == "cloud"}
        eventual = sum(e["ok"] or e.get("eventual_ok", False) for e in zero.values())
        error_x = []
        for e in zero.values():
            x = e.get("x", e.get("eventual_x"))
            if x is not None:
                expected = 0.2 * (e["stamp_ns"] * 1.e-9 - case["motion_start"])
                error_x.append(abs(x - expected))
        common = stamps & odom_stamps
        row = {k: v for k, v in case.items() if k != "events"}
        row.update(clouds=len(clouds), exact_matches=len(common),
                   native_tf_sample_first=sum(native_odom[s] < native_cloud[s] for s in common),
                   wire_received=sum(s in wire_clouds for s in stamps),
                   wire_tf_first=sum(wire_tf[s] < wire_clouds[s] for s in common if s in wire_tf and s in wire_clouds),
                   ros_received=len(zero), zero_ok=sum(e["ok"] for e in zero.values()), eventual_ok=eventual,
                   wait_received=len(waiting), wait_ok=sum(e["ok"] for e in waiting.values()),
                   max_recovery_ms=max((e.get("recovery_ms", 0) for e in zero.values()), default=0),
                   max_wait_ms=max((e["callback_ms"] for e in waiting.values()), default=0),
                   max_pose_error_m=max(error_x, default=0))
        row["steady_200ms_ok"] = sum(e["ok"] or (e.get("eventual_ok", False) and e["recovery_ms"] <= 200.0)
                                      for e in zero.values())
        row["native_wait_clock_jumps"] = [
            {key: e.get(key) for key in ("stamp_ns", "callback_ms", "system_elapsed_ms", "error")}
            for e in waiting.values() if not e["ok"] and
            e.get("system_elapsed_ms", 0) - e["callback_ms"] > 10.0]
        row["rtabmap"] = {strategy: sum(e["kind"] == "rtabmap" and e["case"] == case["case"]
                                       and e["strategy"] == strategy for e in ros)
                          for strategy in ("zero", "wait")}
        rows.append(row)
        if len(zero) != len(clouds) or eventual != len(clouds):
            errors.append("c{:02d}: missing cloud or TF still unavailable after 1 second".format(case["case"]))
        if len(waiting) != len(clouds) or row["wait_ok"] != len(clouds):
            errors.append("c{:02d}: independent TF listener + 200 ms wait failed".format(case["case"]))
        if row["max_pose_error_m"] > 1.e-5:
            errors.append("c{:02d}: incorrect interpolated TF pose".format(case["case"]))
        if case["transport_rejected"]:
            errors.append("c{:02d}: reliable transport messages rejected".format(case["case"]))
    save(args.out, dict(cases=rows, errors=errors))
    print("case fps odom/cloud order exact native_first wire_first clouds zero eventual wait max_wait_ms")
    for row in rows:
        print("c{case:02d} {fps:3d} {odom_hz:2d}/{cloud_hz:<2d} {lidar_first!s:5s} "
              "{exact_matches:3d} {native_tf_sample_first:3d} {wire_tf_first:3d} {clouds:3d} "
              "{zero_ok:3d} {eventual_ok:3d} {wait_ok:3d} {max_wait_ms:.1f}".format(**row))
    print("ASSERTIONS:", "PASS" if not errors else errors)
    return int(bool(errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    p = commands.add_parser("bridge")
    p.add_argument("--port", type=int, default=8278)
    p.add_argument("--out", required=True)
    p = commands.add_parser("probe")
    p.add_argument("--out", required=True)
    p.add_argument("--rtabmap-cases", default="")
    p = commands.add_parser("analyze")
    for field in ("ue", "wire", "ros", "out"):
        p.add_argument("--" + field, required=True)
    args = parser.parse_args()
    return {"bridge": bridge, "probe": probe, "analyze": analyze}[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
