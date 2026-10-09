"""In-process tests: the bridge core and server run inside the test process.

The loopback tests start the bridge as a child process and therefore cannot change its
module constants or inject failures. These tests do, in a private ROS context (and domain),
for the fault handling that is otherwise hard to provoke: a stalled ROS executor, a thread
that cannot be started, socket options.

Run from the package directory::

    python3 -m pytest test/test_bridge_inprocess.py
"""

import json
import os
import random
import socket
import sys
import threading
import time
import unittest
from unittest import mock

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE_DIR)

from ubot_ros_bridge import protocol as proto  # noqa: E402
from ubot_ros_bridge.protocol import Op  # noqa: E402

try:
    import rclpy
    from rclpy.context import Context

    from ubot_ros_bridge import bridge_node

    ROS_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - depends on the environment
    ROS_IMPORT_ERROR = exc

DOMAIN_ID = random.Random().randint(20, 80)


def setUpModule():
    if ROS_IMPORT_ERROR is not None:
        raise unittest.SkipTest("ROS 2 Python packages not available: {}".format(ROS_IMPORT_ERROR))


def qos_json():
    return {"reliability": "reliable", "durability": "volatile", "depth": 10}


class RawClient:
    """Just enough protocol client to drive a BridgeServer."""

    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.parser = proto.FrameParser()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    def send(self, op, obj=None):
        payload = b"" if obj is None else proto.encode_json(obj)
        self.sock.sendall(proto.encode_frame(op, payload))

    def read_frames(self, timeout):
        """Frames received within ``timeout`` seconds; None when the peer closed the connection."""
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(1 << 16)
        except socket.timeout:
            return []
        except ConnectionError:
            return None
        if not data:
            return None
        return self.parser.feed(data)

    def wait_for(self, op, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frames = self.read_frames(max(0.01, deadline - time.monotonic()))
            if frames is None:
                raise AssertionError("connection closed while waiting for {}".format(Op(op).name))
            for frame in frames:
                if frame.op == op:
                    return frame
        raise AssertionError("no {} frame within {} s".format(Op(op).name, timeout))

    def hello(self):
        self.send(Op.HELLO, proto.make_hello("in-process test"))
        return json.loads(bytes(self.wait_for(Op.HELLO_ACK).payload))

    def ping(self):
        self.send(Op.PING)
        self.wait_for(Op.PONG)

    def wait_closed(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.read_frames(max(0.01, deadline - time.monotonic())) is None:
                return True
        return False


class InProcessBridgeCase(unittest.TestCase):
    """A BridgeCore + BridgeServer on a private context, plus an observer node."""

    config_overrides = {}

    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context, domain_id=DOMAIN_ID)
        self.addCleanup(self.context.try_shutdown)
        self.observer = rclpy.create_node("ubot_inprocess_observer", context=self.context)
        self.addCleanup(self.observer.destroy_node)
        config = bridge_node.BridgeConfig(
            bind_address="127.0.0.1", port=0, **self.config_overrides
        )
        self.core = bridge_node.BridgeCore(config, context=self.context)
        self.core.start()
        self.server = bridge_node.BridgeServer(self.core)
        self.server.start()
        self.clients = []

    def tearDown(self):
        for client in self.clients:
            client.close()
        self.server.stop()
        self.core.stop()
        self.core.destroy()

    def connect(self):
        client = RawClient(self.server.port)
        self.clients.append(client)
        return client

    def wait_until(self, condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.05)
        return condition()


class ExecutorStallTest(InProcessBridgeCase):
    def test_disconnect_during_a_stall_still_releases_every_entity(self):
        """The release of a dead client must survive the 10 s executor timeout.

        Before the fix the queued release was cancelled when the executor did not answer in time,
        leaking the publisher, its topic claim and the raw subscription until the bridge restarted.
        """
        pub_topic = "/inproc/leak_pub"
        sub_topic = "/inproc/leak_sub"
        with mock.patch.object(bridge_node, "OP_TIMEOUT_SECONDS", 1.0):
            client = self.connect()
            client.hello()
            client.send(
                Op.ADVERTISE,
                {"id": 1, "topic": pub_topic, "type": "std_msgs/msg/String", "qos": qos_json()},
            )
            client.send(
                Op.SUBSCRIBE,
                {"id": 2, "topic": sub_topic, "type": "std_msgs/msg/String", "qos": qos_json()},
            )
            client.ping()  # both requests are done once the PING behind them is answered
            self.assertEqual(self.observer.count_publishers(pub_topic), 1)
            self.assertEqual(self.observer.count_subscribers(sub_topic), 1)
            self.assertIn(pub_topic, self.core._publishers_by_topic)
            self.assertIn(sub_topic, self.core._topic_types)

            # Stall the executor for 3 s (3 x the shortened operation timeout).
            stall_started = threading.Event()

            def stall():
                stall_started.set()
                time.sleep(3.0)

            def post_stall():
                try:
                    self.core.run_in_ros_thread(stall)
                except bridge_node.RequestError:
                    pass  # this caller times out after 1 s, the stall itself keeps running

            threading.Thread(target=post_stall, daemon=True).start()
            self.assertTrue(stall_started.wait(5.0))
            client.close()  # the peer goes away while the executor is stalled

            self.assertTrue(
                self.wait_until(lambda: not self.core.client_snapshot(), timeout=8.0),
                "the client was not discarded",
            )
            self.assertTrue(
                self.wait_until(
                    lambda: self.observer.count_publishers(pub_topic) == 0
                    and self.observer.count_subscribers(sub_topic) == 0,
                    timeout=10.0,
                ),
                "the dead client's rclpy publisher/subscription were never released",
            )
            self.assertEqual(self.core._topic_types, {})
            self.assertEqual(self.core._publishers_by_topic, {})

            # The topic is free for another type afterwards.
            other = self.connect()
            other.hello()
            other.send(
                Op.ADVERTISE,
                {"id": 1, "topic": pub_topic, "type": "sensor_msgs/msg/Imu", "qos": qos_json()},
            )
            other.ping()
            self.assertEqual(self.observer.count_publishers(pub_topic), 1)

    def test_cancelled_requests_stay_cancelled(self):
        """Request operations (not the release) are still cancelled when the executor is late."""
        with mock.patch.object(bridge_node, "OP_TIMEOUT_SECONDS", 0.5):
            stall_started = threading.Event()

            def stall():
                stall_started.set()
                time.sleep(1.5)

            def post_stall():
                try:
                    self.core.run_in_ros_thread(stall, cancel_on_timeout=False)
                except bridge_node.RequestError:
                    pass  # the caller gives up after 0.5 s, the stall itself keeps running

            threading.Thread(target=post_stall, daemon=True).start()
            self.assertTrue(stall_started.wait(5.0))
            ran = []
            with self.assertRaises(bridge_node.RequestError):
                self.core.run_in_ros_thread(lambda: ran.append("cancelled"))
            time.sleep(2.0)
            self.assertEqual(ran, [], "a timed-out request must not run after the caller gave up")


class AcceptLoopTest(InProcessBridgeCase):
    def test_setup_failure_does_not_kill_the_accept_thread(self):
        original_start = bridge_node.ClientConnection.start
        failures = []

        def flaky_start(connection):
            if not failures:
                failures.append(connection)
                connection._writer.start()  # the writer is up when the reader "cannot start"
                raise RuntimeError("can't start new thread")
            return original_start(connection)

        with mock.patch.object(bridge_node.ClientConnection, "start", flaky_start):
            first = self.connect()
            self.assertTrue(first.wait_closed(), "the failed connection must be closed")
            self.assertTrue(self.server.is_alive(), "the accept thread died")
            self.assertTrue(
                self.wait_until(lambda: not self.core.client_snapshot(), timeout=2.0),
                "the failed client must be detached",
            )

            second = self.connect()
            self.assertEqual(second.hello()["protocol"], 1)
            second.ping()

        started = time.monotonic()
        self.server.stop()
        self.assertLess(
            time.monotonic() - started, 3.0, "stop() waited for a client that never started"
        )
        self.assertFalse(
            failures[0]._writer.is_alive(), "the writer thread of the failed client leaked"
        )

    def test_accepted_sockets_get_keepalive_probes(self):
        listener = socket.create_server(("127.0.0.1", 0))
        self.addCleanup(listener.close)
        peer = socket.create_connection(listener.getsockname())
        self.addCleanup(peer.close)
        conn, _ = listener.accept()
        self.addCleanup(conn.close)
        bridge_node.BridgeServer._configure_socket(conn)
        self.assertIsNone(conn.gettimeout(), "the socket must stay blocking for the writer")
        self.assertTrue(conn.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE))
        self.assertTrue(conn.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY))
        if hasattr(socket, "TCP_KEEPIDLE"):
            self.assertEqual(
                conn.getsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE),
                bridge_node.KEEPALIVE_IDLE_SECONDS,
            )
            self.assertEqual(
                conn.getsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL),
                bridge_node.KEEPALIVE_INTERVAL_SECONDS,
            )
            self.assertEqual(
                conn.getsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT),
                bridge_node.KEEPALIVE_PROBES,
            )


class ConfigValidationTest(unittest.TestCase):
    def test_invalid_deadline_parameters_are_rejected(self):
        context = Context()
        rclpy.init(context=context, domain_id=DOMAIN_ID)
        try:
            invalid = (
                {"idle_timeout_seconds": -1.0},
                {"hello_timeout_seconds": -0.5},
                {"max_clients": -1},
            )
            for overrides in invalid:
                with self.subTest(overrides=overrides):
                    config = bridge_node.BridgeConfig(bind_address="127.0.0.1", port=0, **overrides)
                    with self.assertRaises(ValueError):
                        bridge_node.BridgeCore(config, context=context)
        finally:
            context.try_shutdown()


if __name__ == "__main__":
    unittest.main()
