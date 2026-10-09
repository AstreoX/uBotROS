#!/usr/bin/env python3
"""ROS-side checker for the UE automation test ``UBotROS.EndToEnd.Bridge``.

Start the bridge, run this script, then run the UE test with ``UBOT_ROS_E2E=1``
(the order of the last two does not matter)::

    source /opt/ros/humble/setup.bash
    python3 test/e2e_check.py [--timeout 30]

It subscribes to the fixture topics under ``/ubot_e2e`` (contract section 12)
and publishes ``geometry_msgs/msg/Twist`` on ``/ubot_e2e/cmd_vel`` at 10 Hz
(linear 1.5, 0, 0; angular 0, 0, 0.25) for the UE side to receive.

Exit code 0: the IMU (values exact), the point cloud (fields and values exact),
the static TF ``base_link -> e2e_imu`` and at least one ``/clock`` message were
received within the timeout. Exit code 1: something is missing or wrong; every
missing item is printed with the reason. After all checks pass, the script keeps
publishing cmd_vel until a subscriber has been matched for ``--cmd-vel-linger``
seconds (bounded by the timeout) so the UE test can receive it.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import time
from typing import Any, Dict, List, Optional, Sequence

NAMESPACE = "/ubot_e2e"
IMU_TOPIC = NAMESPACE + "/imu"
POINTS_TOPIC = NAMESPACE + "/points"
CMD_VEL_TOPIC = NAMESPACE + "/cmd_vel"
TF_STATIC_TOPIC = "/tf_static"
CLOCK_TOPIC = "/clock"

IMU_FRAME_ID = "e2e_imu"
IMU_STAMP_SECONDS = 12.5
IMU_ORIENTATION = (0.0, 0.0, 0.0, 1.0)
IMU_ANGULAR_VELOCITY = (0.1, 0.2, 0.3)
IMU_LINEAR_ACCELERATION = (0.0, 0.0, 9.81)

POINTS_FRAME_ID = "e2e_lidar"
POINTFIELD_UINT16 = 4
POINTFIELD_FLOAT32 = 7
POINT_FIELDS = [
    ("x", 0, POINTFIELD_FLOAT32, 1),
    ("y", 4, POINTFIELD_FLOAT32, 1),
    ("z", 8, POINTFIELD_FLOAT32, 1),
    ("intensity", 12, POINTFIELD_FLOAT32, 1),
    ("ring", 16, POINTFIELD_UINT16, 1),
    ("time", 18, POINTFIELD_FLOAT32, 1),
]
POINT_STEP = 22
POINT_STRUCT = struct.Struct("<ffffHf")
POINTS = [
    # x, y, z, intensity, ring, time
    (1.0, 2.0, 3.0, 10.0, 0, 0.0),
    (4.0, 5.0, 6.0, 20.0, 1, 0.0),
    (7.0, 8.0, 9.0, 30.0, 2, 0.0),
]

TF_PARENT = "base_link"
TF_CHILD = "e2e_imu"
TF_TRANSLATION = (1.0, 2.0, 3.0)
TF_ROTATION = (0.0, 0.0, 0.0, 1.0)

CMD_VEL_LINEAR = (1.5, 0.0, 0.0)
CMD_VEL_ANGULAR = (0.0, 0.0, 0.25)
CMD_VEL_PERIOD_SECONDS = 0.1

DEFAULT_TIMEOUT_SECONDS = 30.0
DEFAULT_TOLERANCE = 1e-6
DEFAULT_CMD_VEL_LINGER_SECONDS = 3.0


# ---------------------------------------------------------------------------
# Pure checks (take message objects, return None when OK or a problem string)
# ---------------------------------------------------------------------------


def _differs(actual: float, expected: float, tolerance: float) -> bool:
    return not math.isfinite(actual) or abs(actual - expected) > tolerance


def _compare_vector(
    name: str,
    actual: Sequence[float],
    expected: Sequence[float],
    tolerance: float,
    problems: List[str],
) -> None:
    if any(_differs(a, e, tolerance) for a, e in zip(actual, expected)):
        problems.append(
            "{} = ({}) expected ({})".format(
                name, ", ".join(repr(v) for v in actual), ", ".join(repr(v) for v in expected)
            )
        )


def _stamp_seconds(stamp: Any) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def check_imu(msg: Any, tolerance: float) -> Optional[str]:
    problems: List[str] = []
    if msg.header.frame_id != IMU_FRAME_ID:
        problems.append("frame_id {!r} expected {!r}".format(msg.header.frame_id, IMU_FRAME_ID))
    stamp = _stamp_seconds(msg.header.stamp)
    if _differs(stamp, IMU_STAMP_SECONDS, tolerance):
        problems.append(
            "stamp {}.{:09d} expected {}".format(
                msg.header.stamp.sec, msg.header.stamp.nanosec, IMU_STAMP_SECONDS
            )
        )
    o = msg.orientation
    _compare_vector("orientation", (o.x, o.y, o.z, o.w), IMU_ORIENTATION, tolerance, problems)
    w = msg.angular_velocity
    _compare_vector("angular_velocity", (w.x, w.y, w.z), IMU_ANGULAR_VELOCITY, tolerance, problems)
    a = msg.linear_acceleration
    _compare_vector(
        "linear_acceleration", (a.x, a.y, a.z), IMU_LINEAR_ACCELERATION, tolerance, problems
    )
    return "; ".join(problems) or None


def check_point_cloud(msg: Any, tolerance: float) -> Optional[str]:
    problems: List[str] = []
    if msg.header.frame_id != POINTS_FRAME_ID:
        problems.append("frame_id {!r} expected {!r}".format(msg.header.frame_id, POINTS_FRAME_ID))
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    if fields != POINT_FIELDS:
        problems.append("fields {} expected {}".format(fields, POINT_FIELDS))
    if msg.is_bigendian:
        problems.append("is_bigendian is true")
    if msg.point_step != POINT_STEP:
        problems.append("point_step {} expected {}".format(msg.point_step, POINT_STEP))
    if msg.height * msg.width != len(POINTS):
        problems.append(
            "height*width = {}*{} expected {} points".format(msg.height, msg.width, len(POINTS))
        )
    if msg.row_step != msg.point_step * msg.width:
        problems.append("row_step {} != point_step*width".format(msg.row_step))
    data = bytes(msg.data)
    if len(data) != msg.row_step * msg.height:
        problems.append(
            "data has {} bytes, row_step*height = {}".format(len(data), msg.row_step * msg.height)
        )
    if problems:
        return "; ".join(problems)
    for index, expected in enumerate(POINTS):
        actual = POINT_STRUCT.unpack_from(data, index * POINT_STEP)
        floats_ok = all(not _differs(actual[i], expected[i], tolerance) for i in (0, 1, 2, 3, 5))
        if not floats_ok or actual[4] != expected[4]:
            problems.append(
                "point {} (x, y, z, intensity, ring, time) = {} expected {}".format(
                    index, actual, expected
                )
            )
    return "; ".join(problems) or None


def check_tf_static(msg: Any, tolerance: float) -> Optional[str]:
    seen = []
    for transform in msg.transforms:
        parent = transform.header.frame_id
        child = transform.child_frame_id
        seen.append("{}->{}".format(parent, child))
        if parent != TF_PARENT or child != TF_CHILD:
            continue
        problems: List[str] = []
        t = transform.transform.translation
        _compare_vector("translation", (t.x, t.y, t.z), TF_TRANSLATION, tolerance, problems)
        r = transform.transform.rotation
        _compare_vector("rotation", (r.x, r.y, r.z, r.w), TF_ROTATION, tolerance, problems)
        if problems:
            return "{}->{}: {}".format(TF_PARENT, TF_CHILD, "; ".join(problems))
        return None
    return "no {}->{} transform in message (contains: {})".format(
        TF_PARENT, TF_CHILD, ", ".join(seen) or "nothing"
    )


def check_clock(msg: Any, tolerance: float) -> Optional[str]:
    return None


class Expectation:
    def __init__(self, key: str, topic: str, type_name: str, requirement: str) -> None:
        self.key = key
        self.topic = topic
        self.type_name = type_name
        self.requirement = requirement
        self.received = 0
        self.passed = False
        self.passed_after: Optional[float] = None
        self.last_problem: Optional[str] = None

    def record(self, problem: Optional[str], elapsed: float) -> bool:
        """Record one message; return True when this message made the check pass first."""
        self.received += 1
        if problem is not None:
            self.last_problem = problem
            return False
        if self.passed:
            return False
        self.passed = True
        self.passed_after = elapsed
        return True


def _log(message: str) -> None:
    print("[e2e] " + message, flush=True)


def parse_arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help="seconds to wait for all fixture messages (default: %(default)s)",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="absolute tolerance for floating-point comparisons (default: %(default)s)",
    )
    parser.add_argument(
        "--cmd-vel-linger",
        type=float,
        default=DEFAULT_CMD_VEL_LINGER_SECONDS,
        help="after all checks pass, keep publishing cmd_vel until a subscriber has been matched "
        "for this many seconds (0 = exit immediately; default: %(default)s)",
    )
    return parser.parse_args(list(argv))


def _describe_publishers(node: Any, topic: str) -> str:
    try:
        infos = node.get_publishers_info_by_topic(topic)
    except Exception as exc:  # pragma: no cover - diagnostics only
        return "graph query failed: {!r}".format(exc)
    if not infos:
        return "no publisher on the ROS graph"
    parts = []
    for info in infos:
        qos = info.qos_profile
        parts.append(
            "{} [{}] reliability={} durability={}".format(
                info.node_name or "?",
                info.topic_type,
                getattr(qos.reliability, "name", qos.reliability).lower(),
                getattr(qos.durability, "name", qos.durability).lower(),
            )
        )
    return "publishers: " + "; ".join(parts)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, PointCloud2
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        _log(
            "FAIL: cannot import ROS 2 Python packages ({}); "
            "source /opt/ros/humble/setup.bash".format(exc)
        )
        return 1

    volatile_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    latched_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    command_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    expectations = [
        (
            Expectation(
                "imu",
                IMU_TOPIC,
                "sensor_msgs/msg/Imu",
                "frame e2e_imu, stamp 12.5 s, orientation (0,0,0,1), "
                "angular_velocity (0.1,0.2,0.3), linear_acceleration (0,0,9.81)",
            ),
            Imu,
            volatile_qos,
            check_imu,
        ),
        (
            Expectation(
                "points",
                POINTS_TOPIC,
                "sensor_msgs/msg/PointCloud2",
                "frame e2e_lidar, uBot layout (point_step 22), points (1,2,3) (4,5,6) (7,8,9), "
                "intensity 10/20/30, ring 0/1/2, time 0",
            ),
            PointCloud2,
            volatile_qos,
            check_point_cloud,
        ),
        (
            Expectation(
                "tf_static",
                TF_STATIC_TOPIC,
                "tf2_msgs/msg/TFMessage",
                "base_link -> e2e_imu, translation (1,2,3), identity rotation (transient_local)",
            ),
            TFMessage,
            latched_qos,
            check_tf_static,
        ),
        (
            Expectation("clock", CLOCK_TOPIC, "rosgraph_msgs/msg/Clock", "at least one message"),
            Clock,
            volatile_qos,
            check_clock,
        ),
    ]

    rclpy.init()
    node = rclpy.create_node("ubot_e2e_check")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    start = time.monotonic()

    def make_callback(expectation: Expectation, check: Any) -> Any:
        def callback(msg: Any) -> None:
            elapsed = time.monotonic() - start
            if expectation.record(check(msg, args.tolerance), elapsed):
                _log(
                    "+{:5.2f} s  OK       {:<9} {}".format(
                        elapsed, expectation.key, expectation.topic
                    )
                )

        return callback

    for expectation, message_type, qos, check in expectations:
        node.create_subscription(
            message_type, expectation.topic, make_callback(expectation, check), qos
        )

    twist = Twist()
    twist.linear.x, twist.linear.y, twist.linear.z = CMD_VEL_LINEAR
    twist.angular.x, twist.angular.y, twist.angular.z = CMD_VEL_ANGULAR
    command_publisher = node.create_publisher(Twist, CMD_VEL_TOPIC, command_qos)
    command_stats: Dict[str, int] = {"published": 0, "matched": 0}

    def publish_command() -> None:
        command_publisher.publish(twist)
        command_stats["published"] += 1
        if command_publisher.get_subscription_count() > 0:
            command_stats["matched"] += 1

    node.create_timer(CMD_VEL_PERIOD_SECONDS, publish_command)

    _log(
        "waiting up to {:.0f} s for {} (ROS_DOMAIN_ID={}); publishing {} at 10 Hz".format(
            args.timeout,
            ", ".join(e.topic for e, _, _, _ in expectations),
            _domain_id(),
            CMD_VEL_TOPIC,
        )
    )

    interrupted = False
    linger_announced = False
    deadline = start + args.timeout
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if all(e.passed for e, _, _, _ in expectations):
                matched_seconds = command_stats["matched"] * CMD_VEL_PERIOD_SECONDS
                if matched_seconds >= args.cmd_vel_linger:
                    break
                if not linger_announced:
                    linger_announced = True
                    _log(
                        "all checks passed; publishing cmd_vel until a subscriber has "
                        "been matched for {:.1f} s".format(args.cmd_vel_linger)
                    )
    except (KeyboardInterrupt, ExternalShutdownException):
        interrupted = True

    elapsed = time.monotonic() - start
    missing = [e for e, _, _, _ in expectations if not e.passed]
    for expectation, _, _, _ in expectations:
        if expectation.passed:
            _log(
                "OK       {:<9} {} [{}]: {} message(s), matched after {:.2f} s".format(
                    expectation.key,
                    expectation.topic,
                    expectation.type_name,
                    expectation.received,
                    expectation.passed_after or 0.0,
                )
            )
            continue
        if expectation.received == 0:
            reason = "no message received ({})".format(
                _describe_publishers(node, expectation.topic)
            )
        else:
            reason = "{} message(s) received, none matched; last problem: {}".format(
                expectation.received, expectation.last_problem
            )
        _log(
            "MISSING  {:<9} {} [{}]: {}".format(
                expectation.key, expectation.topic, expectation.type_name, reason
            )
        )
        _log("         expected: {}".format(expectation.requirement))
    _log(
        "cmd_vel: published {} message(s), {} while a subscriber was matched "
        "(subscribers now: {})".format(
            command_stats["published"],
            command_stats["matched"],
            command_publisher.get_subscription_count(),
        )
    )

    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()

    if interrupted:
        _log("RESULT: FAIL (interrupted after {:.1f} s)".format(elapsed))
        return 1
    if missing:
        _log(
            "RESULT: FAIL after {:.1f} s, missing: {}".format(
                elapsed, ", ".join(e.key for e in missing)
            )
        )
        return 1
    _log("RESULT: PASS ({} checks in {:.1f} s)".format(len(expectations), elapsed))
    return 0


def _domain_id() -> str:
    return os.environ.get("ROS_DOMAIN_ID", "0")


if __name__ == "__main__":
    sys.exit(main())
