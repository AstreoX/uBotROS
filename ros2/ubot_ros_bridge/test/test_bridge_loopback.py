"""Loopback tests: start the real bridge process and speak the wire protocol to it.

Needs a sourced ROS 2 environment (rclpy, sensor_msgs, geometry_msgs, std_msgs,
tf2_msgs, rosgraph_msgs); skipped otherwise. Each test class uses its own
ROS_DOMAIN_ID so it does not see or disturb other ROS traffic.

Run from the package directory::

    python3 -m pytest test/test_bridge_loopback.py
"""

import json
import os
import random
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from collections import deque

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
E2E_CHECK = os.path.join(PACKAGE_DIR, "test", "e2e_check.py")
sys.path.insert(0, PACKAGE_DIR)

from ubot_ros_bridge import protocol as proto  # noqa: E402
from ubot_ros_bridge.protocol import ErrorCode, Op  # noqa: E402

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message, serialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    from ubot_ros_bridge import bridge_node

    ROS_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - depends on the environment
    ROS_IMPORT_ERROR = exc

_DOMAIN_BASE = random.Random().randint(20, 80)
LOOPBACK_DOMAIN = _DOMAIN_BASE
SLOW_CLIENT_DOMAIN = _DOMAIN_BASE + 1
SHUTDOWN_DOMAIN = _DOMAIN_BASE + 2
E2E_DOMAIN = _DOMAIN_BASE + 3
E2E_EMPTY_DOMAIN = _DOMAIN_BASE + 4
MAX_CLIENTS_DOMAIN = _DOMAIN_BASE + 5

_rclpy_initialized = False


def setUpModule():
    global _rclpy_initialized
    if ROS_IMPORT_ERROR is not None:
        raise unittest.SkipTest("ROS 2 Python packages not available: {}".format(ROS_IMPORT_ERROR))
    os.environ["ROS_DOMAIN_ID"] = str(LOOPBACK_DOMAIN)
    rclpy.init()
    _rclpy_initialized = True


def tearDownModule():
    if _rclpy_initialized:
        rclpy.try_shutdown()


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def qos(reliability="reliable", durability="volatile", depth=10):
    return {"reliability": reliability, "durability": durability, "depth": depth}


def ros_qos(reliability="reliable", durability="volatile", depth=10):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=(
            ReliabilityPolicy.BEST_EFFORT
            if reliability == "best_effort"
            else ReliabilityPolicy.RELIABLE
        ),
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if durability == "transient_local"
            else DurabilityPolicy.VOLATILE
        ),
    )


def child_env(domain_id):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain_id)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    return env


class BridgeProcess:
    """The bridge started exactly like a user would: python3 -m ubot_ros_bridge.bridge_node."""

    def __init__(self, domain_id, extra_args=()):
        self.port = free_port()
        self._log = tempfile.TemporaryFile()
        command = [
            sys.executable,
            "-m",
            "ubot_ros_bridge.bridge_node",
            "--port",
            str(self.port),
            "--bind-address",
            "127.0.0.1",
        ] + list(extra_args)
        self.process = subprocess.Popen(
            command,
            cwd=PACKAGE_DIR,
            env=child_env(domain_id),
            stdout=self._log,
            stderr=subprocess.STDOUT,
        )
        self._wait_listening()

    def _wait_listening(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError("bridge exited early:\n" + self.output())
            if "listening on" in self.output():
                return
            time.sleep(0.1)
        raise RuntimeError("bridge did not start listening:\n" + self.output())

    def output(self):
        self._log.flush()
        self._log.seek(0)
        return self._log.read().decode("utf-8", "replace")

    def stop(self, signum=signal.SIGINT, timeout=10.0):
        if self.process.poll() is None:
            self.process.send_signal(signum)
            try:
                self.process.wait(timeout)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(5)
                raise AssertionError(
                    "bridge did not exit within {} s:\n{}".format(timeout, self.output())
                )
        return self.process.returncode

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(5)
        finally:
            self._log.close()


class ConnectionClosed(Exception):
    pass


class ProtocolClient:
    """Minimal protocol client, the way UE talks to the bridge."""

    def __init__(self, port, hello=True, name="uBot UE loopback-test"):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.parser = proto.FrameParser()
        self.backlog = deque()
        self.ack = None
        if hello:
            self.send_json(Op.HELLO, proto.make_hello(name))
            self.ack = json.loads(bytes(self.wait_for(Op.HELLO_ACK).payload))

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def send(self, op, payload=b""):
        self.sock.sendall(proto.encode_frame(op, payload))

    def send_raw(self, data):
        self.sock.sendall(data)

    def send_json(self, op, obj):
        self.send(op, proto.encode_json(obj))

    def advertise(self, pub_id, topic, type_name, qos_obj):
        self.send_json(
            Op.ADVERTISE, {"id": pub_id, "topic": topic, "type": type_name, "qos": qos_obj}
        )

    def unadvertise(self, pub_id):
        self.send_json(Op.UNADVERTISE, {"id": pub_id})

    def subscribe(self, sub_id, topic, type_name, qos_obj):
        self.send_json(
            Op.SUBSCRIBE, {"id": sub_id, "topic": topic, "type": type_name, "qos": qos_obj}
        )

    def unsubscribe(self, sub_id):
        self.send_json(Op.UNSUBSCRIBE, {"id": sub_id})

    def publish(self, pub_id, cdr):
        self.sock.sendall(proto.encode_id_frame(Op.PUBLISH, pub_id, cdr))

    def _read(self, timeout):
        self.sock.settimeout(max(timeout, 0.001))
        try:
            data = self.sock.recv(1 << 20)
        except socket.timeout:
            return
        if not data:
            raise ConnectionClosed()
        self.backlog.extend(self.parser.feed(data))

    def poll(self, op, predicate=None, timeout=0.0):
        """Return the first matching frame, reading at least once even when ``timeout`` is 0."""
        deadline = time.monotonic() + timeout
        read_once = False
        while True:
            for index, frame in enumerate(self.backlog):
                if frame.op == op and (predicate is None or predicate(frame)):
                    del self.backlog[index]
                    return frame
            remaining = deadline - time.monotonic()
            if remaining <= 0 and read_once:
                return None
            self._read(remaining)
            read_once = True

    def wait_for(self, op, predicate=None, timeout=5.0):
        frame = self.poll(op, predicate, timeout)
        if frame is None:
            raise AssertionError(
                "no {} frame within {} s (backlog: {})".format(
                    Op(op).name,
                    timeout,
                    [Op(f.op).name if f.op in proto.KNOWN_OPS else f.op for f in self.backlog],
                )
            )
        return frame

    def wait_error(self, code, request_id="any", timeout=5.0):
        def matches(frame):
            payload = json.loads(bytes(frame.payload))
            return payload["code"] == code and (request_id == "any" or payload["id"] == request_id)

        return json.loads(bytes(self.wait_for(Op.ERROR, matches, timeout).payload))

    def wait_sub_count(self, pub_id, predicate, timeout=10.0):
        def matches(frame):
            counts = json.loads(bytes(frame.payload))["counts"]
            return str(pub_id) in counts and predicate(counts[str(pub_id)])

        return json.loads(bytes(self.wait_for(Op.SUB_COUNT, matches, timeout).payload))["counts"]

    def wait_closed(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                self._read(deadline - time.monotonic())
        except (ConnectionClosed, ConnectionResetError, BrokenPipeError):
            return True
        except proto.ProtocolError:
            return False
        return False

    def ping(self, timeout=5.0):
        self.send(Op.PING)
        self.wait_for(Op.PONG, timeout=timeout)


class RosSide:
    """Test-side rclpy node, spun from the test thread only (no background executor)."""

    def __init__(self, name):
        self.node = rclpy.create_node(name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def destroy(self):
        self.executor.shutdown()
        self.node.destroy_node()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self.executor.spin_once(timeout_sec=min(remaining, 0.02))

    def collect(self, message_type, topic, qos_profile, raw=False):
        received = []
        subscription = self.node.create_subscription(
            message_type, topic, received.append, qos_profile, raw=raw
        )
        return received, subscription

    def wait_until(self, condition, timeout=10.0, step=None, step_period=0.1):
        deadline = time.monotonic() + timeout
        next_step = 0.0
        while time.monotonic() < deadline:
            if step is not None and time.monotonic() >= next_step:
                step()
                next_step = time.monotonic() + step_period
            self.spin(0.02)
            if condition():
                return True
        return condition()


def make_imu():
    msg = Imu()
    msg.header.stamp.sec = 1234
    msg.header.stamp.nanosec = 567890123
    msg.header.frame_id = "imu_link_odd"
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = (
        0.1,
        -0.2,
        0.3,
        0.927,
    )
    msg.orientation_covariance = [0.01 * i for i in range(9)]
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = 0.5, -1.25, 2.0
    msg.angular_velocity_covariance = [0.02 * i for i in range(9)]
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
        0.25,
        0.5,
        9.81,
    )
    msg.linear_acceleration_covariance = [0.03 * i for i in range(9)]
    return msg


def make_twist(linear_x=1.5, angular_z=0.25):
    msg = Twist()
    msg.linear.x = linear_x
    msg.linear.y = -0.5
    msg.angular.z = angular_z
    return msg


class BridgeLoopbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = BridgeProcess(LOOPBACK_DOMAIN)
        cls.ros = RosSide("ubot_bridge_loopback_test")

    @classmethod
    def tearDownClass(cls):
        cls.ros.destroy()
        try:
            code = cls.bridge.stop()
            if code != 0:
                print(cls.bridge.output(), file=sys.stderr)
        finally:
            cls.bridge.close()

    def setUp(self):
        self.clients = []

    def tearDown(self):
        for client in self.clients:
            client.close()

    def client(self, **kwargs):
        client = ProtocolClient(self.bridge.port, **kwargs)
        self.clients.append(client)
        return client

    def test_hello_ack(self):
        client = self.client()
        self.assertEqual(client.ack["protocol"], 1)
        self.assertTrue(client.ack["server"].startswith("ubot_ros_bridge "), client.ack)
        self.assertEqual(client.ack["ros_distro"], os.environ.get("ROS_DISTRO", "unknown"))

    def test_ping_pong_unknown_and_unexpected_ops(self):
        client = self.client()
        client.ping()
        client.send(99, b"whatever")
        client.wait_error(ErrorCode.UNKNOWN_OP, None)
        client.send(int(Op.MESSAGE), b"\x00" * 8)
        client.wait_error(ErrorCode.UNEXPECTED_OP, None)
        client.send(int(Op.PONG))
        client.ping()

    def test_requests_before_hello_are_rejected(self):
        client = self.client(hello=False)
        client.ping()
        client.advertise(1, "/ubot_test/early", "std_msgs/msg/String", qos())
        client.wait_error(ErrorCode.HELLO_REQUIRED, None)
        client.send_json(Op.HELLO, {"protocol": 1, "client": "late"})
        client.wait_for(Op.HELLO_ACK)
        client.advertise(1, "/ubot_test/early", "std_msgs/msg/String", qos())
        client.ping()
        self.assertIsNone(client.poll(Op.ERROR, timeout=0.2))

    def test_unsupported_protocol_closes_connection(self):
        client = self.client(hello=False)
        client.send_json(Op.HELLO, {"protocol": 2, "client": "future"})
        client.wait_error(ErrorCode.UNSUPPORTED_PROTOCOL, None)
        self.assertTrue(client.wait_closed())

    def test_publish_imu_reaches_ros_subscriber(self):
        topic = "/ubot_test/imu"
        received, subscription = self.ros.collect(Imu, topic, ros_qos("best_effort", depth=5))
        client = self.client()
        client.advertise(1, topic, "sensor_msgs/msg/Imu", qos("best_effort", "volatile", 5))
        counts = client.wait_sub_count(1, lambda n: n >= 1)
        self.assertGreaterEqual(counts["1"], 1)

        original = make_imu()
        cdr = serialize_message(original)
        self.assertEqual(cdr[:4], b"\x00\x01\x00\x00")
        self.assertTrue(
            self.ros.wait_until(lambda: received, timeout=10, step=lambda: client.publish(1, cdr)),
            "the rclpy subscriber never received the IMU published through the bridge",
        )
        self.assertEqual(received[0], original)
        self.assertEqual(serialize_message(received[0]), cdr)
        client.ping()
        self.assertIsNone(client.poll(Op.ERROR, timeout=0.1))

        # SUB_COUNT follows the subscriber going away.
        self.ros.node.destroy_subscription(subscription)
        self.assertEqual(client.wait_sub_count(1, lambda n: n == 0)["1"], 0)

    def test_subscribe_forwards_ros_messages(self):
        topic = "/ubot_test/cmd_vel"
        publisher = self.ros.node.create_publisher(Twist, topic, ros_qos("reliable", depth=10))
        try:
            client = self.client()
            client.subscribe(
                7, topic, "geometry_msgs/msg/Twist", qos("best_effort", "volatile", 10)
            )
            client.ping()
            twist = make_twist()
            expected = serialize_message(twist)
            frames = []

            def step():
                publisher.publish(twist)
                frame = client.poll(Op.MESSAGE, timeout=0.05)
                if frame is not None:
                    frames.append(frame)

            self.assertTrue(
                self.ros.wait_until(lambda: frames, timeout=10, step=step, step_period=0.05)
            )
            sub_id, cdr = proto.split_id_payload(frames[0].payload)
            self.assertEqual(sub_id, 7)
            self.assertEqual(bytes(cdr), expected)
            self.assertEqual(deserialize_message(bytes(cdr), Twist), twist)

            # After UNSUBSCRIBE no more MESSAGE frames arrive.
            client.unsubscribe(7)
            client.ping()
            client.backlog.clear()
            for _ in range(5):
                publisher.publish(twist)
                self.ros.spin(0.05)
            self.assertIsNone(client.poll(Op.MESSAGE, timeout=0.3))
        finally:
            self.ros.node.destroy_publisher(publisher)

    def test_transient_local_subscription_gets_latched_message(self):
        topic = "/ubot_test/latched"
        latched = ros_qos("reliable", "transient_local", 1)
        publisher = self.ros.node.create_publisher(String, topic, latched)
        try:
            publisher.publish(String(data="latched-hello"))
            client = self.client()
            client.subscribe(
                3, topic, "std_msgs/msg/String", qos("reliable", "transient_local", 1)
            )
            frames = []

            def step():
                frame = client.poll(Op.MESSAGE, timeout=0.05)
                if frame is not None:
                    frames.append(frame)

            self.assertTrue(
                self.ros.wait_until(lambda: frames, timeout=10, step=step, step_period=0.05)
            )
            _, cdr = proto.split_id_payload(frames[0].payload)
            self.assertEqual(deserialize_message(bytes(cdr), String).data, "latched-hello")
        finally:
            self.ros.node.destroy_publisher(publisher)

    def test_shared_publisher_is_reference_counted(self):
        topic = "/ubot_test/shared"
        received, subscription = self.ros.collect(String, topic, ros_qos("reliable", depth=50))
        try:
            first = self.client()
            second = self.client()
            first.advertise(1, topic, "std_msgs/msg/String", qos("reliable", "volatile", 10))
            second.advertise(9, topic, "std_msgs/msg/String", qos("reliable", "volatile", 10))
            first.wait_sub_count(1, lambda n: n >= 1)
            second.wait_sub_count(9, lambda n: n >= 1)
            self.assertEqual(
                self.ros.node.count_publishers(topic), 1, "the bridge must share one publisher"
            )

            first.publish(1, serialize_message(String(data="from-first")))
            second.publish(9, serialize_message(String(data="from-second")))
            self.assertTrue(self.ros.wait_until(lambda: len(received) >= 2, timeout=10))
            self.assertEqual(sorted(m.data for m in received), ["from-first", "from-second"])

            first.unadvertise(1)
            first.ping()
            first.publish(1, serialize_message(String(data="stale")))
            first.wait_error(ErrorCode.UNKNOWN_ID, 1)
            second.publish(9, serialize_message(String(data="still-alive")))
            self.assertTrue(
                self.ros.wait_until(lambda: any(m.data == "still-alive" for m in received))
            )
            self.assertEqual(self.ros.node.count_publishers(topic), 1)

            second.unadvertise(9)
            second.ping()
            self.assertTrue(
                self.ros.wait_until(lambda: self.ros.node.count_publishers(topic) == 0)
            )
        finally:
            self.ros.node.destroy_subscription(subscription)

    def test_conflicts_and_request_errors(self):
        first = self.client()
        second = self.client()
        topic = "/ubot_test/conflict"
        first.advertise(1, topic, "sensor_msgs/msg/Imu", qos("reliable", "volatile", 10))
        first.ping()
        self.assertIsNone(first.poll(Op.ERROR, timeout=0.1))

        second.advertise(2, topic, "std_msgs/msg/String", qos("reliable", "volatile", 10))
        self.assertIn(
            "sensor_msgs/msg/Imu", second.wait_error(ErrorCode.TYPE_CONFLICT, 2)["message"]
        )
        second.advertise(3, topic, "sensor_msgs/msg/Imu", qos("best_effort", "volatile", 10))
        second.wait_error(ErrorCode.QOS_CONFLICT, 3)
        second.subscribe(4, topic, "std_msgs/msg/String", qos())
        second.wait_error(ErrorCode.TYPE_CONFLICT, 4)

        first.advertise(1, "/ubot_test/other", "std_msgs/msg/String", qos())
        first.wait_error(ErrorCode.DUPLICATE_ID, 1)
        first.subscribe(5, "/ubot_test/sub_dup", "std_msgs/msg/String", qos())
        first.subscribe(5, "/ubot_test/sub_dup", "std_msgs/msg/String", qos())
        first.wait_error(ErrorCode.DUPLICATE_ID, 5)

        first.advertise(6, "/ubot_test/nope", "nonexistent_msgs/msg/Nope", qos())
        first.wait_error(ErrorCode.UNKNOWN_TYPE, 6)
        first.advertise(7, "/ubot_test/nope", "std_msgs/msg/NoSuchType", qos())
        first.wait_error(ErrorCode.UNKNOWN_TYPE, 7)
        first.advertise(8, "relative/topic", "std_msgs/msg/String", qos())
        first.wait_error(ErrorCode.INVALID_TOPIC, 8)
        first.advertise(9, "/ubot_test/bad_qos", "std_msgs/msg/String", qos(depth=0))
        first.wait_error(ErrorCode.INVALID_QOS, 9)
        first.send(Op.ADVERTISE, b"{not json")
        first.wait_error(ErrorCode.BAD_REQUEST, None)

        first.unadvertise(4242)
        first.wait_error(ErrorCode.UNKNOWN_ID, 4242)
        first.unsubscribe(4243)
        first.wait_error(ErrorCode.UNKNOWN_ID, 4243)
        first.publish(77, b"\x00\x01\x00\x00")
        first.wait_error(ErrorCode.UNKNOWN_ID, 77)
        first.publish(77, b"\x00\x01\x00\x00")  # reported once per id only
        first.send(Op.PUBLISH, b"\x01\x00")
        first.wait_error(ErrorCode.BAD_PAYLOAD, None)
        first.publish(1, b"\x00\x01")
        first.wait_error(ErrorCode.BAD_PAYLOAD, 1)
        first.ping()
        remaining = [json.loads(bytes(f.payload)) for f in first.backlog if f.op == Op.ERROR]
        self.assertFalse([e for e in remaining if e["id"] == 77], remaining)

        # The topic becomes free for other types once its only user is gone.
        first.unadvertise(1)
        first.ping()
        second.advertise(10, topic, "std_msgs/msg/String", qos())
        second.ping()
        self.assertIsNone(
            second.poll(Op.ERROR, lambda f: json.loads(bytes(f.payload))["id"] == 10, 0.2)
        )

    def test_disconnect_releases_ros_entities(self):
        pub_topic = "/ubot_test/release_pub"
        sub_topic = "/ubot_test/release_sub"
        client = self.client()
        client.advertise(1, pub_topic, "std_msgs/msg/String", qos())
        client.subscribe(2, sub_topic, "std_msgs/msg/String", qos())
        client.ping()
        self.assertTrue(
            self.ros.wait_until(
                lambda: self.ros.node.count_publishers(pub_topic) == 1
                and self.ros.node.count_subscribers(sub_topic) == 1
            )
        )
        client.close()
        self.assertTrue(
            self.ros.wait_until(
                lambda: self.ros.node.count_publishers(pub_topic) == 0
                and self.ros.node.count_subscribers(sub_topic) == 0
            )
        )
        # The topic can be advertised again with another type by a new client.
        other = self.client()
        other.advertise(1, pub_topic, "sensor_msgs/msg/Imu", qos())
        other.ping()
        self.assertIsNone(other.poll(Op.ERROR, timeout=0.2))

    def test_deeply_nested_json_is_a_bad_request(self):
        """json.loads raises RecursionError for these; that must not kill the session."""
        client = self.client()
        for payload in (b"[" * 5000, b'{"id":' + b"[" * 5000, b"{" * 5000):
            for op in (Op.ADVERTISE, Op.SUBSCRIBE, Op.UNADVERTISE, Op.UNSUBSCRIBE):
                with self.subTest(op=Op(op).name, size=len(payload)):
                    client.send(op, payload)
                    client.wait_error(ErrorCode.BAD_REQUEST, None)
        client.send(Op.HELLO, b"[" * 5000)
        client.wait_error(ErrorCode.BAD_REQUEST, None)
        client.ping()  # still connected
        self.assertNotIn("RecursionError", self.bridge.output())
        self.assertNotIn("reader failed", self.bridge.output())

    def test_type_names_never_import_arbitrary_modules(self):
        """A remote peer must not be able to run top-level code of any module on sys.path."""
        client = self.client()
        client.advertise(1, "/ubot_test/zen", "this/msg/Foo", qos())
        client.wait_error(ErrorCode.UNKNOWN_TYPE, 1)
        client.subscribe(2, "/ubot_test/zen", "this/msg/Foo", qos())
        client.wait_error(ErrorCode.UNKNOWN_TYPE, 2)
        # A real package with a missing message is reported too.
        client.advertise(3, "/ubot_test/zen", "std_msgs/msg/NoSuchMessage", qos())
        self.assertIn("NoSuchMessage", client.wait_error(ErrorCode.UNKNOWN_TYPE, 3)["message"])
        client.ping()
        self.assertNotIn("Beautiful is better than ugly", self.bridge.output())

    def test_oversized_frames_are_protocol_errors(self):
        """Only PUBLISH may be large and nothing is large before HELLO."""
        before_hello = self.client(hello=False)
        before_hello.send_raw(proto.encode_header(Op.PUBLISH, 1024 * 1024))
        before_hello.wait_error(ErrorCode.PROTOCOL_ERROR, None)
        self.assertTrue(before_hello.wait_closed(), "an oversized frame before HELLO must close")

        for op, size in (
            (Op.ADVERTISE, proto.CONTROL_PAYLOAD_LIMIT_BYTES + 1),
            (Op.HELLO, 10 * 1024 * 1024),
            (Op.PING, proto.SMALL_PAYLOAD_LIMIT_BYTES + 1),
            (99, proto.SMALL_PAYLOAD_LIMIT_BYTES + 1),
        ):
            with self.subTest(op=op, size=size):
                client = self.client()
                client.send_raw(proto.encode_header(op, size))
                client.wait_error(ErrorCode.PROTOCOL_ERROR, None)
                self.assertTrue(client.wait_closed())
        # The bridge keeps serving other clients.
        self.client().ping()

    def test_large_publish_right_behind_hello(self):
        """HELLO, ADVERTISE and a 1 MiB PUBLISH in one write: the limit rises while parsing."""
        topic = "/ubot_test/big_after_hello"
        latched = ros_qos("reliable", "transient_local", 5)
        received, subscription = self.ros.collect(String, topic, latched)
        try:
            client = self.client(hello=False)
            payload = serialize_message(String(data="y" * (1024 * 1024)))
            client.send_raw(
                b"".join(
                    (
                        proto.encode_frame(Op.HELLO, proto.encode_json(proto.make_hello("burst"))),
                        proto.encode_frame(
                            Op.ADVERTISE,
                            proto.encode_json(
                                {
                                    "id": 1,
                                    "topic": topic,
                                    "type": "std_msgs/msg/String",
                                    "qos": qos("reliable", "transient_local", 5),
                                }
                            ),
                        ),
                        proto.encode_id_frame(Op.PUBLISH, 1, payload),
                    )
                )
            )
            self.assertTrue(
                self.ros.wait_until(lambda: received, timeout=15),
                "the large PUBLISH behind the HELLO never arrived\n" + self.bridge.output(),
            )
            self.assertEqual(len(received[0].data), 1024 * 1024)
            client.wait_for(Op.HELLO_ACK)
            self.assertIsNone(client.poll(Op.ERROR, timeout=0.2))
        finally:
            self.ros.node.destroy_subscription(subscription)

    def test_error_replies_are_rate_limited(self):
        """Thousands of bad frames get a bounded number of replies, then the connection drops."""
        client = self.client()
        client.send_raw(
            b"".join(
                proto.encode_id_frame(Op.PUBLISH, 100000 + index, b"\x00\x01\x00\x00")
                for index in range(5000)
            )
        )
        started = time.monotonic()
        closed = False
        try:
            while time.monotonic() - started < 10:
                client._read(0.2)
        except (ConnectionClosed, ConnectionResetError, BrokenPipeError):
            closed = True
        elapsed = time.monotonic() - started
        errors = [f for f in client.backlog if f.op == Op.ERROR]
        budget = bridge_node.ERROR_REPLY_BURST + bridge_node.ERROR_REPLY_REFILL_PER_SECOND * elapsed
        self.assertGreaterEqual(len(errors), 1)
        self.assertLessEqual(
            len(errors), budget + 2, "5000 bad frames produced {} replies".format(len(errors))
        )
        self.assertTrue(closed, "a connection that sends thousands of bad frames must be dropped")
        self.assertIn("too many protocol errors", self.bridge.output())
        self.client().ping()  # other clients are unaffected

    def test_client_text_is_bounded_and_cannot_forge_log_lines(self):
        client = self.client(name="evil\nFORGED LOG LINE\x1b[31mred")
        client.advertise(1, "/" + "a" * 100 + "-bad", "std_msgs/msg/String", qos())
        message = client.wait_error(ErrorCode.INVALID_TOPIC, 1)["message"]
        self.assertLess(len(message), 400, message)
        client.advertise(2, "/" + "b" * 60000, "std_msgs/msg/String", qos())
        message = client.wait_error(ErrorCode.INVALID_TOPIC, 2)["message"]
        self.assertLess(len(message), 400, message)
        client.advertise(3, "/ok_topic", "x" * 50000, qos())
        message = client.wait_error(ErrorCode.UNKNOWN_TYPE, 3)["message"]
        self.assertLess(len(message), 400, message)
        client.ping()
        deadline = time.monotonic() + 3
        while "HELLO from" not in self.bridge.output() and time.monotonic() < deadline:
            time.sleep(0.05)
        output = self.bridge.output()
        self.assertIn("evil\\nFORGED LOG LINE\\x1b[31mred", output)
        self.assertNotIn("\x1b", output)
        self.assertNotIn("\nFORGED LOG LINE", output)
        self.assertLess(len(output), 20000)

    def test_malformed_headers_close_connection(self):
        bad_magic = b"XBRB" + proto.encode_frame(Op.PING)[4:]
        bad_version = bytearray(proto.encode_frame(Op.PING))
        bad_version[4] = 7
        oversize = proto.encode_header(Op.PUBLISH, proto.DEFAULT_MAX_PAYLOAD_BYTES + 1)
        for name, data in (
            ("bad magic", bad_magic),
            ("bad version", bytes(bad_version)),
            ("oversize", oversize),
        ):
            with self.subTest(name):
                client = self.client()
                client.send_raw(data)
                self.assertTrue(client.wait_closed(), "connection not closed after " + name)
        # The bridge keeps serving other clients.
        self.client().ping()


class SlowClientTest(unittest.TestCase):
    """A client that never reads must not stall the bridge or other clients."""

    @classmethod
    def setUpClass(cls):
        cls.bridge = BridgeProcess(
            SLOW_CLIENT_DOMAIN,
            [
                "--max-payload-bytes",
                str(2 * 1024 * 1024),
                "--max-queued-bytes",
                str(4 * 1024 * 1024),
            ],
        )

    @classmethod
    def tearDownClass(cls):
        try:
            cls.bridge.stop()
        finally:
            cls.bridge.close()

    def test_best_effort_frames_are_dropped_for_a_stalled_client(self):
        topic = "/ubot_test/flood"
        stalled = ProtocolClient(self.bridge.port)
        stalled.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024)
        healthy = ProtocolClient(self.bridge.port)
        producer = ProtocolClient(self.bridge.port)
        try:
            stalled.subscribe(1, topic, "std_msgs/msg/String", qos("best_effort", "volatile", 100))
            healthy.subscribe(1, topic, "std_msgs/msg/String", qos("best_effort", "volatile", 100))
            producer.advertise(1, topic, "std_msgs/msg/String", qos("reliable", "volatile", 100))
            producer.wait_sub_count(1, lambda n: n >= 2, timeout=10)
            payload = serialize_message(String(data="x" * (1024 * 1024)))
            healthy_messages = 0
            start = time.monotonic()
            for _ in range(60):
                producer.publish(1, payload)
                # The healthy client keeps reading; the stalled one never does.
                window_end = time.monotonic() + 0.05
                while True:
                    frame = healthy.poll(
                        Op.MESSAGE, timeout=max(0.0, window_end - time.monotonic())
                    )
                    if frame is None:
                        break
                    healthy_messages += 1
            deadline = time.monotonic() + 5
            while (
                time.monotonic() < deadline and healthy.poll(Op.MESSAGE, timeout=0.2) is not None
            ):
                healthy_messages += 1
            healthy.ping(timeout=5)
            producer.ping(timeout=5)
            self.assertGreaterEqual(healthy_messages, 50, "healthy client starved")
            self.assertLess(time.monotonic() - start, 30)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and "dropped" not in self.bridge.output():
                time.sleep(0.2)
            self.assertIn("dropped", self.bridge.output())
        finally:
            for client in (stalled, healthy, producer):
                client.close()


class IdleTimeoutTest(unittest.TestCase):
    """Silent, half-open and handshake-less clients are dropped; busy and pinging ones are not."""

    IDLE_SECONDS = 2.0
    HELLO_SECONDS = 1.5

    @classmethod
    def setUpClass(cls):
        # Same domain as the test process: RosSide watches the graph from here.
        cls.bridge = BridgeProcess(
            LOOPBACK_DOMAIN,
            [
                "--idle-timeout-seconds",
                str(cls.IDLE_SECONDS),
                "--hello-timeout-seconds",
                str(cls.HELLO_SECONDS),
            ],
        )
        cls.ros = RosSide("ubot_bridge_idle_test")

    @classmethod
    def tearDownClass(cls):
        cls.ros.destroy()
        try:
            cls.bridge.stop()
        finally:
            cls.bridge.close()

    def test_silent_client_is_dropped_and_its_entities_released(self):
        pub_topic = "/ubot_test/idle_pub"
        sub_topic = "/ubot_test/idle_sub"
        client = ProtocolClient(self.bridge.port)
        try:
            client.advertise(1, pub_topic, "std_msgs/msg/String", qos())
            client.subscribe(2, sub_topic, "std_msgs/msg/String", qos())
            client.ping()
            # Discovery can take longer than the idle timeout: keep pinging until the graph shows
            # the entities, then go silent.
            self.assertTrue(
                self.ros.wait_until(
                    lambda: self.ros.node.count_publishers(pub_topic) == 1
                    and self.ros.node.count_subscribers(sub_topic) == 1,
                    step=client.ping,
                    step_period=0.5,
                )
            )
            started = time.monotonic()
            self.assertTrue(
                client.wait_closed(self.IDLE_SECONDS + 4.0),
                "a client that went silent was never dropped\n" + self.bridge.output(),
            )
            elapsed = time.monotonic() - started
            self.assertGreaterEqual(elapsed, self.IDLE_SECONDS - 0.5)
            self.assertLessEqual(elapsed, self.IDLE_SECONDS + 2.5)
            self.assertTrue(
                self.ros.wait_until(
                    lambda: self.ros.node.count_publishers(pub_topic) == 0
                    and self.ros.node.count_subscribers(sub_topic) == 0
                ),
                "the idle client's ROS entities were not released",
            )
            self.assertIn("idle timeout", self.bridge.output())
        finally:
            client.close()

    def test_pinging_client_stays_connected(self):
        client = ProtocolClient(self.bridge.port)
        try:
            end = time.monotonic() + self.IDLE_SECONDS + 2.0
            while time.monotonic() < end:
                client.ping()
                time.sleep(0.5)
            client.ping()
            self.assertIsNone(client.poll(Op.ERROR, timeout=0.1))
        finally:
            client.close()

    def test_connection_without_hello_is_dropped_even_while_it_sends(self):
        client = ProtocolClient(self.bridge.port, hello=False)
        try:
            started = time.monotonic()
            closed = False
            try:
                while time.monotonic() - started < self.HELLO_SECONDS + 4.0:
                    client.send(Op.PING)  # data keeps flowing, but the handshake never happens
                    client._read(0.2)
            except (ConnectionClosed, ConnectionResetError, BrokenPipeError):
                closed = True
            self.assertTrue(closed, "a connection that never says HELLO must be dropped")
            elapsed = time.monotonic() - started
            self.assertGreaterEqual(elapsed, self.HELLO_SECONDS - 0.5)
            self.assertIn("no HELLO", self.bridge.output())
        finally:
            client.close()

    def test_blocked_writer_is_not_timed_out(self):
        """The idle logic must not touch the socket's blocking mode: a long sendall is fine."""
        topic = "/ubot_test/idle_flood"
        stalled = ProtocolClient(self.bridge.port)
        stalled.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024)
        producer = ProtocolClient(self.bridge.port)
        try:
            stalled.subscribe(1, topic, "std_msgs/msg/String", qos("reliable", "volatile", 50))
            producer.advertise(1, topic, "std_msgs/msg/String", qos("reliable", "volatile", 50))
            producer.wait_sub_count(1, lambda n: n >= 1, timeout=10)
            payload = serialize_message(String(data="z" * (1024 * 1024)))
            for _ in range(20):
                producer.publish(1, payload)
                time.sleep(0.02)
            # The writer of the stalled client is now blocked in sendall (it never reads).
            # Keep the connection alive with PINGs for longer than the idle timeout.
            end = time.monotonic() + self.IDLE_SECONDS + 1.5
            while time.monotonic() < end:
                stalled.send(Op.PING)
                producer.ping()
                time.sleep(0.3)
            # Now read: the connection must still be open and deliver the queued data and the PONGs.
            frame = stalled.wait_for(Op.MESSAGE, timeout=10)
            self.assertEqual(proto.split_id_payload(frame.payload)[0], 1)
            stalled.send(Op.PING)
            stalled.wait_for(Op.PONG, timeout=20)
            self.assertNotIn("send failed", self.bridge.output())
        finally:
            stalled.close()
            producer.close()


class MaxClientsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = BridgeProcess(MAX_CLIENTS_DOMAIN, ["--max-clients", "2"])

    @classmethod
    def tearDownClass(cls):
        try:
            cls.bridge.stop()
        finally:
            cls.bridge.close()

    def test_surplus_connections_are_refused_and_slots_are_reusable(self):
        first = ProtocolClient(self.bridge.port)
        second = ProtocolClient(self.bridge.port)
        third = ProtocolClient(self.bridge.port, hello=False)
        try:
            third.wait_error(ErrorCode.TOO_MANY_CLIENTS, None)
            self.assertTrue(third.wait_closed())
            first.ping()
            second.ping()
            first.close()
            deadline = time.monotonic() + 10
            replacement = None
            while replacement is None and time.monotonic() < deadline:
                try:
                    replacement = ProtocolClient(self.bridge.port)
                except (AssertionError, ConnectionClosed, OSError):
                    time.sleep(0.2)
            self.assertIsNotNone(replacement, "the slot of the closed client was never freed")
            replacement.ping()
            replacement.close()
            self.assertIn("max_clients=2", self.bridge.output())
        finally:
            for client in (first, second, third):
                client.close()


class ShutdownTest(unittest.TestCase):
    def _run(self, signum):
        bridge = BridgeProcess(SHUTDOWN_DOMAIN)
        try:
            client = ProtocolClient(bridge.port)
            client.advertise(1, "/ubot_test/shutdown", "std_msgs/msg/String", qos())
            client.subscribe(2, "/ubot_test/shutdown_sub", "std_msgs/msg/String", qos())
            client.ping()
            started = time.monotonic()
            self.assertEqual(bridge.stop(signum), 0, bridge.output())
            self.assertLess(time.monotonic() - started, 10)
            self.assertTrue(client.wait_closed(2.0))
            self.assertIn("shutdown requested", bridge.output())
            self.assertNotIn("Traceback", bridge.output())
            client.close()
        finally:
            bridge.close()

    def test_sigint_shuts_down_cleanly(self):
        self._run(signal.SIGINT)

    def test_sigterm_shuts_down_cleanly(self):
        self._run(signal.SIGTERM)


def make_e2e_fixtures(clock_seconds):
    imu = Imu()
    imu.header.stamp.sec = 12
    imu.header.stamp.nanosec = 500000000
    imu.header.frame_id = "e2e_imu"
    imu.orientation.w = 1.0
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = 0.1, 0.2, 0.3
    imu.linear_acceleration.z = 9.81

    cloud = PointCloud2()
    cloud.header.stamp.sec = 12
    cloud.header.frame_id = "e2e_lidar"
    cloud.height = 1
    cloud.width = 3
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 22
    cloud.row_step = 66
    points = [(1, 2, 3, 10, 0, 0), (4, 5, 6, 20, 1, 0), (7, 8, 9, 30, 2, 0)]
    cloud.data = b"".join(struct.pack("<ffffHf", *p) for p in points)
    cloud.is_dense = True

    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "e2e_imu"
    transform.transform.translation.x, transform.transform.translation.y = 1.0, 2.0
    transform.transform.translation.z = 3.0
    transform.transform.rotation.w = 1.0
    tf_static = TFMessage(transforms=[transform])

    clock = Clock(clock=Time(sec=int(clock_seconds), nanosec=int((clock_seconds % 1) * 1e9)))
    return imu, cloud, tf_static, clock


class E2eCheckerTest(unittest.TestCase):
    """Runs test/e2e_check.py against a fake UE client that publishes the section 12 fixtures."""

    def test_checker_passes_and_cmd_vel_reaches_client(self):
        bridge = BridgeProcess(E2E_DOMAIN)
        client = None
        checker = None
        checker_log = tempfile.TemporaryFile()
        try:
            client = ProtocolClient(bridge.port, name="uBot UE fake-e2e")
            client.advertise(
                1, "/ubot_e2e/imu", "sensor_msgs/msg/Imu", qos("best_effort", "volatile", 5)
            )
            client.advertise(
                2,
                "/ubot_e2e/points",
                "sensor_msgs/msg/PointCloud2",
                qos("best_effort", "volatile", 5),
            )
            client.advertise(
                3, "/tf_static", "tf2_msgs/msg/TFMessage", qos("reliable", "transient_local", 1)
            )
            client.advertise(
                4, "/clock", "rosgraph_msgs/msg/Clock", qos("reliable", "volatile", 10)
            )
            client.subscribe(
                1,
                "/ubot_e2e/cmd_vel",
                "geometry_msgs/msg/Twist",
                qos("best_effort", "volatile", 10),
            )
            client.ping()
            _, _, tf_static, _ = make_e2e_fixtures(0.0)
            client.publish(3, serialize_message(tf_static))

            checker = subprocess.Popen(
                [sys.executable, E2E_CHECK, "--timeout", "25"],
                cwd=PACKAGE_DIR,
                env=child_env(E2E_DOMAIN),
                stdout=checker_log,
                stderr=subprocess.STDOUT,
            )
            twists = []
            clock_seconds = 100.0
            deadline = time.monotonic() + 40
            while checker.poll() is None and time.monotonic() < deadline:
                imu, cloud, _, clock = make_e2e_fixtures(clock_seconds)
                client.publish(1, serialize_message(imu))
                client.publish(2, serialize_message(cloud))
                client.publish(4, serialize_message(clock))
                clock_seconds += 0.1
                frame = client.poll(Op.MESSAGE, timeout=0.1)
                while frame is not None:
                    twists.append(frame)
                    frame = client.poll(Op.MESSAGE, timeout=0.0)
            if checker.poll() is None:
                checker.kill()
            checker.wait(5)
            checker_log.seek(0)
            output = checker_log.read().decode("utf-8", "replace")
            self.assertEqual(checker.returncode, 0, output)
            self.assertIn("RESULT: PASS", output)
            self.assertTrue(twists, "the fake UE client never received cmd_vel\n" + output)
            sub_id, cdr = proto.split_id_payload(twists[0].payload)
            self.assertEqual(sub_id, 1)
            twist = deserialize_message(bytes(cdr), Twist)
            self.assertEqual((twist.linear.x, twist.linear.y, twist.linear.z), (1.5, 0.0, 0.0))
            self.assertEqual((twist.angular.x, twist.angular.y, twist.angular.z), (0.0, 0.0, 0.25))
        finally:
            if checker is not None and checker.poll() is None:
                checker.kill()
            checker_log.close()
            if client is not None:
                client.close()
            bridge.stop()
            bridge.close()

    def test_checker_reports_missing_items(self):
        result = subprocess.run(
            [sys.executable, E2E_CHECK, "--timeout", "3"],
            cwd=PACKAGE_DIR,
            env=child_env(E2E_EMPTY_DOMAIN),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=60,
        )
        output = result.stdout.decode("utf-8", "replace")
        self.assertEqual(result.returncode, 1, output)
        for key in ("imu", "points", "tf_static", "clock"):
            self.assertRegex(output, r"MISSING\s+{}\s".format(key))
        self.assertIn("RESULT: FAIL", output)
        self.assertIn("no message received", output)


if __name__ == "__main__":
    unittest.main()
