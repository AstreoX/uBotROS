"""ubot_ros_bridge: TCP <-> ROS 2 bridge for the uBot UE plugin (wire protocol v1).

UE connects over TCP, advertises/subscribes topics and streams ready-made CDR
bytes. This node publishes them unchanged with ``Publisher.publish(bytes)`` and
forwards raw subscription data back as MESSAGE frames.

Threading model:

* one rclpy executor thread; every change to the ROS graph (create/destroy
  publishers and subscriptions) runs there, queued through a guard condition,
* one accept thread,
* per client a reader thread (parses frames, publishes, issues graph changes and
  waits for their result so requests stay ordered; it also enforces the HELLO
  deadline and the idle timeout) and a writer thread that drains a bounded send
  queue; all socket writes go through a per-client lock.

Run it as ``ros2 run ubot_ros_bridge bridge`` or, without colcon, from the package
directory with ``python3 -m ubot_ros_bridge.bridge_node --port 8268``.
"""

from __future__ import annotations

import argparse
import functools
import os
import selectors
import signal
import socket
import sys
import threading
import time
from collections import deque
from concurrent.futures import CancelledError, Future
from concurrent.futures import TimeoutError as FutureTimeoutError
from dataclasses import dataclass
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Set, Tuple

import rclpy
from ament_index_python.resources import get_resource
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import InvalidTopicNameException
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rclpy.validate_full_topic_name import validate_full_topic_name
from rosidl_runtime_py.utilities import get_message

from ubot_ros_bridge import __version__
from ubot_ros_bridge import protocol as proto
from ubot_ros_bridge.protocol import ErrorCode, FrameClass, Op, PutResult, RequestError

SERVER_NAME = "ubot_ros_bridge {}".format(__version__)
NODE_NAME = "ubot_ros_bridge"

RECV_BUFFER_BYTES = 256 * 1024
OP_TIMEOUT_SECONDS = 10.0
SUB_COUNT_POLL_SECONDS = 0.1
DROP_LOG_INTERVAL_SECONDS = 5.0
WRITER_DRAIN_TIMEOUT_SECONDS = 2.0
CLIENT_JOIN_TIMEOUT_SECONDS = 5.0
MIN_PAYLOAD_LIMIT_BYTES = 1024
CDR_ENCAPSULATION_BYTES = 4
#: Longest sleep of the reader between two checks of its deadlines.
MAX_DEADLINE_POLL_SECONDS = 1.0
MIN_DEADLINE_POLL_SECONDS = 0.05
#: Per-connection budget of ERROR replies: a burst, then a steady rate (a client gets a final
#: PROTOCOL_ERROR / UNSUPPORTED_PROTOCOL reply regardless).
ERROR_REPLY_BURST = 30
ERROR_REPLY_REFILL_PER_SECOND = 10.0
#: More suppressed replies than this within the window and the connection is dropped.
ERROR_SUPPRESSED_ABORT_COUNT = 1000
ERROR_SUPPRESSED_WINDOW_SECONDS = 10.0
#: Publisher ids remembered for the "reported once per id" rule.
MAX_REPORTED_PUBLISH_IDS = 1024
SURPLUS_LOG_INTERVAL_SECONDS = 5.0
ACCEPT_RETRY_SECONDS = 0.2
ACCEPT_RETRY_MAX_SECONDS = 2.0
#: TCP keepalive for accepted sockets: first probe after 10 s of silence, then every 5 s, 3 misses.
KEEPALIVE_IDLE_SECONDS = 10
KEEPALIVE_INTERVAL_SECONDS = 5
KEEPALIVE_PROBES = 3


@dataclass
class BridgeConfig:
    bind_address: str = proto.DEFAULT_BIND_ADDRESS
    port: int = proto.DEFAULT_PORT
    max_payload_bytes: int = proto.DEFAULT_MAX_PAYLOAD_BYTES
    max_queued_bytes: int = proto.DEFAULT_MAX_QUEUED_BYTES
    idle_timeout_seconds: float = proto.DEFAULT_IDLE_TIMEOUT_SECONDS
    hello_timeout_seconds: float = proto.DEFAULT_HELLO_TIMEOUT_SECONDS
    max_clients: int = proto.DEFAULT_MAX_CLIENTS


class BridgeStopped(Exception):
    """Raised when a ROS graph change is requested while the bridge shuts down."""


class SharedPublisher:
    """One rclpy publisher per topic, shared by every client that advertises it."""

    __slots__ = ("topic", "type_name", "qos", "publisher", "refs")

    def __init__(self, topic: str, type_name: str, qos: proto.QosSpec, publisher: Any) -> None:
        self.topic = topic
        self.type_name = type_name
        self.qos = qos
        self.publisher = publisher
        self.refs = 1


class ClientSubscription:
    """A raw rclpy subscription owned by one client subscription id."""

    __slots__ = ("id", "topic", "type_name", "qos", "subscription", "oversize_reported")

    def __init__(self, sub_id: int, topic: str, type_name: str, qos: proto.QosSpec) -> None:
        self.id = sub_id
        self.topic = topic
        self.type_name = type_name
        self.qos = qos
        self.subscription: Any = None
        self.oversize_reported = False


def to_qos_profile(qos: proto.QosSpec) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=qos.depth,
        reliability=(
            ReliabilityPolicy.BEST_EFFORT if qos.is_best_effort else ReliabilityPolicy.RELIABLE
        ),
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if qos.durability is proto.Durability.TRANSIENT_LOCAL
            else DurabilityPolicy.VOLATILE
        ),
    )


def _format_bytes(count: int) -> str:
    if count >= 1024 * 1024 and count % (1024 * 1024) == 0:
        return "{} MiB".format(count // (1024 * 1024))
    return "{} bytes".format(count)


class ClientConnection:
    """One TCP client (normally one UE world)."""

    def __init__(
        self, core: "BridgeCore", sock: socket.socket, address: Tuple[Any, ...], number: int
    ) -> None:
        self._core = core
        self._sock = sock
        self._logger = core.logger
        self._max_payload_bytes = core.config.max_payload_bytes
        self.label = "client #{} ({}:{})".format(number, address[0], address[1])
        self.client_name = ""
        self.hello_done = False
        self._connected_at = time.monotonic()
        # Nothing may be large before the HELLO; _on_hello raises the limit to max_payload_bytes.
        self._parser = proto.FrameParser(
            min(self._max_payload_bytes, proto.PRE_HELLO_PAYLOAD_LIMIT_BYTES), per_op_limits=True
        )
        # Mutated only on the ROS executor thread; read by the reader thread.
        self.publications: Dict[int, SharedPublisher] = {}
        self.subscriptions: Dict[int, ClientSubscription] = {}
        self.sub_count_throttle = proto.SubCountThrottle()
        self.finished = threading.Event()
        self._queue = proto.BoundedFrameQueue(core.config.max_queued_bytes)
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._abort_reason: Optional[str] = None
        self._close_reason: Optional[str] = None
        self._reported_publish_ids: Set[int] = set()
        self._hello_required_reported = False
        self._drops_since_log = 0
        self._last_drop_log = float("-inf")
        # ERROR reply budget; guarded by _state_lock because replies come from the reader and the
        # executor thread.
        self._error_bucket = proto.TokenBucket(ERROR_REPLY_BURST, ERROR_REPLY_REFILL_PER_SECOND)
        self._suppressed_errors = 0
        self._suppressed_unreported = 0
        self._suppressed_window_start = 0.0
        self._suppressed_in_window = 0
        self._last_suppressed_log = float("-inf")
        self._reader = threading.Thread(
            target=self._read_loop, name="ubot-client-{}-rx".format(number), daemon=True
        )
        self._writer = threading.Thread(
            target=self._write_loop, name="ubot-client-{}-tx".format(number), daemon=True
        )

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        self._writer.start()
        self._reader.start()

    @property
    def is_closing(self) -> bool:
        return self._queue.closed or self._close_reason is not None

    def abort(self, reason: str) -> None:
        """Drop the connection now (any thread)."""
        with self._state_lock:
            if self._abort_reason is None:
                self._abort_reason = reason
        self._queue.close(discard=True)
        self._shutdown_socket()

    def _request_close(self, reason: str) -> None:
        """Close once queued frames (e.g. a final ERROR) are sent; reader thread only."""
        if self._close_reason is None:
            self._close_reason = reason

    def _shutdown_socket(self) -> None:
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    # -- sending -----------------------------------------------------------

    def send_frame(self, frame: bytes, frame_class: FrameClass = FrameClass.CONTROL) -> PutResult:
        result = self._queue.put(frame, frame_class)
        if result is PutResult.OVERFLOW:
            self._logger.warn(
                "{}: send queue overflow ({} queued), the client is not reading; "
                "dropping the connection".format(
                    self.label, _format_bytes(self._queue.queued_bytes)
                )
            )
            self.abort("send queue overflow")
        return result

    def send_json(self, op: Op, obj: Dict[str, Any]) -> None:
        self.send_frame(proto.encode_json_frame(op, obj))

    def send_error(
        self, request_id: Optional[int], code: str, message: str, force: bool = False
    ) -> None:
        """Log and queue an ERROR reply.

        Replies are rate limited per connection (``force`` bypasses the limit for the final
        reply before a close), and the text is bounded and free of control characters, so a
        peer cannot flood the log, the send queue or the disk with them.
        """
        message = proto.clip_text(message, proto.MAX_ERROR_MESSAGE_CHARS)
        if not force and not self._allow_error_reply():
            return
        self._logger.warn(
            "{}: ERROR {} (id={}): {}".format(
                self.label, code, "null" if request_id is None else request_id, message
            )
        )
        self.send_json(Op.ERROR, proto.make_error(request_id, code, message))

    def _allow_error_reply(self) -> bool:
        with self._state_lock:
            if self._error_bucket.try_take(time.monotonic()):
                return True
        self._note_suppressed_error()
        return False

    def _note_suppressed_error(self) -> None:
        """Count an error reply that was not sent; drop a connection that keeps provoking them."""
        now = time.monotonic()
        summary = 0
        with self._state_lock:
            self._suppressed_errors += 1
            self._suppressed_unreported += 1
            if now - self._suppressed_window_start > ERROR_SUPPRESSED_WINDOW_SECONDS:
                self._suppressed_window_start = now
                self._suppressed_in_window = 0
            self._suppressed_in_window += 1
            too_many = self._suppressed_in_window > ERROR_SUPPRESSED_ABORT_COUNT
            if now - self._last_suppressed_log >= DROP_LOG_INTERVAL_SECONDS:
                self._last_suppressed_log = now
                summary = self._suppressed_unreported
                self._suppressed_unreported = 0
        if summary:
            self._logger.warn(
                "{}: {} error repl{} suppressed (over the per-connection error budget)".format(
                    self.label, summary, "y" if summary == 1 else "ies"
                )
            )
        if too_many:
            self._logger.warn(
                "{}: too many protocol errors, dropping the connection".format(self.label)
            )
            self.abort("too many protocol errors")

    def deliver_ros_message(self, entry: ClientSubscription, data: bytes) -> None:
        """Queue a MESSAGE frame (called on the ROS executor thread)."""
        if self.is_closing:
            return
        if len(data) + 4 > self._max_payload_bytes:
            if not entry.oversize_reported:
                entry.oversize_reported = True
                self.send_error(
                    entry.id,
                    ErrorCode.MESSAGE_TOO_LARGE,
                    "message of {} bytes on {} exceeds the payload limit of {}; such messages "
                    "are dropped".format(
                        len(data), entry.topic, _format_bytes(self._max_payload_bytes)
                    ),
                )
            return
        frame_class = FrameClass.BEST_EFFORT if entry.qos.is_best_effort else FrameClass.RELIABLE
        result = self.send_frame(proto.encode_id_frame(Op.MESSAGE, entry.id, data), frame_class)
        if result in (PutResult.DROPPED, PutResult.REJECTED):
            self._note_drop()

    def _note_drop(self) -> None:
        self._drops_since_log += 1
        now = time.monotonic()
        if now - self._last_drop_log < DROP_LOG_INTERVAL_SECONDS:
            return
        self._logger.warn(
            "{}: send queue full ({} of {}), dropped {} MESSAGE frame(s) since the last report "
            "(total: {} best-effort dropped, {} reliable rejected)".format(
                self.label,
                _format_bytes(self._queue.queued_bytes),
                _format_bytes(self._queue.max_bytes),
                self._drops_since_log,
                self._queue.dropped_best_effort,
                self._queue.rejected_reliable,
            )
        )
        self._drops_since_log = 0
        self._last_drop_log = now

    def _write_loop(self) -> None:
        try:
            while True:
                frame = self._queue.get()
                if frame is None:
                    return
                with self._send_lock:
                    self._sock.sendall(frame)
        except OSError as exc:
            self.abort("send failed: {}".format(exc))
        except Exception as exc:  # pragma: no cover - defensive
            self._logger.error("{}: writer failed: {!r}".format(self.label, exc))
            self.abort("writer failed: {!r}".format(exc))

    # -- receiving ---------------------------------------------------------

    def _read_loop(self) -> None:
        reason = "connection closed by peer"
        graceful = False
        parser = self._parser
        buffer = bytearray(RECV_BUFFER_BYTES)
        view = memoryview(buffer)
        config = self._core.config
        selector: Optional[selectors.BaseSelector] = None
        try:
            if config.idle_timeout_seconds > 0 or config.hello_timeout_seconds > 0:
                # Deadlines are enforced here, on the reader side only: the socket itself stays
                # blocking, because a socket timeout would also time out the writer's sendall.
                selector = selectors.DefaultSelector()
                selector.register(self._sock, selectors.EVENT_READ)
            last_rx = time.monotonic()
            while self._close_reason is None:
                if selector is not None:
                    expired = self._wait_readable(selector, last_rx)
                    if expired is not None:
                        reason = expired
                        break
                count = self._sock.recv_into(buffer)
                if count == 0:
                    break
                last_rx = time.monotonic()
                for frame in parser.iter_frames(view[:count]):
                    self._handle_frame(frame)
                    if self._close_reason is not None:
                        break
                # Handling a frame can block for a while (a graph change waits for the executor).
                last_rx = time.monotonic()
        except proto.ProtocolError as exc:
            reason = "protocol error: {}".format(exc)
            graceful = True
            self.send_error(
                None,
                ErrorCode.PROTOCOL_ERROR,
                "{}; closing the connection".format(exc),
                force=True,
            )
        except OSError as exc:
            reason = "connection error: {}".format(exc)
        except Exception as exc:  # pragma: no cover - defensive
            reason = "internal error: {!r}".format(exc)
            self._logger.error("{}: reader failed: {!r}".format(self.label, exc))
        finally:
            if selector is not None:
                selector.close()
            if self._close_reason is not None:
                reason = self._close_reason
                graceful = True
            self._finish(reason, graceful)

    def _wait_readable(self, selector: selectors.BaseSelector, last_rx: float) -> Optional[str]:
        """Block until the socket is readable. Returns a drop reason once a deadline has expired.

        Idle time only counts while nothing at all is readable, so data that is already
        buffered by the kernel (a PING that arrived while the reader was busy) never causes
        a false timeout.
        """
        config = self._core.config
        deadlines = [
            t for t in (config.idle_timeout_seconds, config.hello_timeout_seconds) if t > 0
        ]
        poll = max(
            MIN_DEADLINE_POLL_SECONDS, min(MAX_DEADLINE_POLL_SECONDS, min(deadlines) / 4.0)
        )
        while True:
            now = time.monotonic()
            if (
                not self.hello_done
                and config.hello_timeout_seconds > 0
                and now - self._connected_at > config.hello_timeout_seconds
            ):
                self._logger.warn(
                    "{}: no HELLO within {:g} s, dropping the connection".format(
                        self.label, config.hello_timeout_seconds
                    )
                )
                return "no HELLO within {:g} s".format(config.hello_timeout_seconds)
            if selector.select(poll):
                return None
            now = time.monotonic()
            if config.idle_timeout_seconds > 0 and now - last_rx > config.idle_timeout_seconds:
                self._logger.warn(
                    "{}: no data for {:g} s, dropping the connection".format(
                        self.label, config.idle_timeout_seconds
                    )
                )
                return "idle timeout (no data for {:g} s)".format(config.idle_timeout_seconds)

    def _finish(self, reason: str, graceful: bool) -> None:
        with self._state_lock:
            if self._abort_reason is not None:
                reason = self._abort_reason
                graceful = False
        self._queue.close(discard=not graceful)
        if graceful:
            self._writer.join(WRITER_DRAIN_TIMEOUT_SECONDS)
        self._shutdown_socket()
        self._writer.join(WRITER_DRAIN_TIMEOUT_SECONDS)
        try:
            self._sock.close()
        except OSError:
            pass
        try:
            self._core.release_client(self)
        except Exception as exc:  # pragma: no cover - defensive
            self._logger.warn("{}: cleanup failed: {!r}".format(self.label, exc))
        if self._suppressed_errors:
            self._logger.info(
                "{}: {} error repl{} suppressed in total".format(
                    self.label,
                    self._suppressed_errors,
                    "y was" if self._suppressed_errors == 1 else "ies were",
                )
            )
        self._logger.info("{} disconnected: {}".format(self.label, reason))
        self.finished.set()

    def _handle_frame(self, frame: proto.Frame) -> None:
        op = frame.op
        if op == Op.PING:
            self.send_frame(proto.encode_frame(Op.PONG))
            return
        if op == Op.PONG:
            return
        if op == Op.HELLO:
            self._on_hello(frame.payload)
            return
        if op not in proto.KNOWN_OPS:
            self.send_error(None, ErrorCode.UNKNOWN_OP, "unknown op {}".format(op))
            return
        if op not in proto.CLIENT_OPS:
            self.send_error(
                None,
                ErrorCode.UNEXPECTED_OP,
                "op {} is only sent by the bridge".format(Op(op).name),
            )
            return
        if not self.hello_done:
            if op != Op.PUBLISH or not self._hello_required_reported:
                if op == Op.PUBLISH:
                    self._hello_required_reported = True
                self.send_error(
                    None, ErrorCode.HELLO_REQUIRED, "send HELLO before {}".format(Op(op).name)
                )
            return
        try:
            if op == Op.PUBLISH:
                self._on_publish(frame.payload)
            elif op == Op.ADVERTISE:
                request = proto.parse_endpoint_request(proto.decode_json_object(frame.payload))
                self._call_ros(self._core.advertise, self, request)
                self._reported_publish_ids.discard(request.id)
            elif op == Op.UNADVERTISE:
                pub_id = proto.parse_id(proto.decode_json_object(frame.payload))
                self._call_ros(self._core.unadvertise, self, pub_id)
            elif op == Op.SUBSCRIBE:
                request = proto.parse_endpoint_request(proto.decode_json_object(frame.payload))
                self._call_ros(self._core.subscribe, self, request)
            elif op == Op.UNSUBSCRIBE:
                sub_id = proto.parse_id(proto.decode_json_object(frame.payload))
                self._call_ros(self._core.unsubscribe, self, sub_id)
        except RequestError as exc:
            self.send_error(exc.request_id, exc.code, exc.message)

    def _call_ros(self, fn: Callable[..., Any], *args: Any) -> Any:
        try:
            return self._core.run_in_ros_thread(fn, *args)
        except BridgeStopped:
            raise RequestError(ErrorCode.INTERNAL_ERROR, "the bridge is shutting down")

    def _on_hello(self, payload: proto.BytesLike) -> None:
        try:
            client = proto.parse_hello(proto.decode_json_object(payload))
        except RequestError as exc:
            self.send_error(
                None, exc.code, exc.message, force=exc.code == ErrorCode.UNSUPPORTED_PROTOCOL
            )
            if exc.code == ErrorCode.UNSUPPORTED_PROTOCOL:
                self._request_close("unsupported protocol")
            return
        self.client_name = client
        self.hello_done = True
        self._parser.max_payload_bytes = self._max_payload_bytes
        self.send_json(Op.HELLO_ACK, proto.make_hello_ack(SERVER_NAME, self._core.ros_distro))
        self._logger.info("{}: HELLO from '{}'".format(self.label, client))

    def _on_publish(self, payload: proto.BytesLike) -> None:
        pub_id, cdr = proto.split_id_payload(payload)
        shared = self.publications.get(pub_id)
        if shared is None:
            self._report_publish_problem(
                pub_id,
                ErrorCode.UNKNOWN_ID,
                "PUBLISH for id {} which is not advertised".format(pub_id),
            )
            return
        if len(cdr) < CDR_ENCAPSULATION_BYTES:
            self._report_publish_problem(
                pub_id,
                ErrorCode.BAD_PAYLOAD,
                "CDR data shorter than the 4-byte encapsulation header",
            )
            return
        try:
            shared.publisher.publish(bytes(cdr))
        except Exception as exc:
            self._report_publish_problem(
                pub_id,
                ErrorCode.PUBLISH_FAILED,
                "publish on {} failed: {}".format(shared.topic, exc),
            )

    def _report_publish_problem(self, pub_id: int, code: str, message: str) -> None:
        """Report a PUBLISH problem once per id (until the id is advertised again)."""
        if pub_id in self._reported_publish_ids:
            return
        if len(self._reported_publish_ids) >= MAX_REPORTED_PUBLISH_IDS:
            # Remembering more ids would let a peer grow this set without bound.
            self._note_suppressed_error()
            return
        self._reported_publish_ids.add(pub_id)
        self.send_error(pub_id, code, message + " (reported once per id)")


class BridgeCore:
    """Owns the rclpy node, the executor thread and the shared ROS entities."""

    def __init__(self, defaults: BridgeConfig, context: Optional[Any] = None) -> None:
        # ``context`` lets tests run the core in a private ROS context (and domain).
        self.node = rclpy.create_node(NODE_NAME, context=context)
        self.logger = self.node.get_logger()
        self.config = self._read_parameters(defaults)
        self.ros_distro = os.environ.get("ROS_DISTRO", "unknown")
        self._publishers_by_topic: Dict[str, SharedPublisher] = {}
        self._topic_types: Dict[str, List[Any]] = {}  # topic -> [type_name, users]
        self._message_classes: Dict[str, Any] = {}
        self._clients: Set[ClientConnection] = set()
        self._clients_lock = threading.Lock()
        self._ops: Deque[Tuple[Future, Callable[..., Any], Tuple[Any, ...]]] = deque()
        self._ops_lock = threading.Lock()
        self._accepting_ops = True
        self._ops_guard = self.node.create_guard_condition(self._run_pending_ops)
        self._sub_count_timer = self.node.create_timer(
            SUB_COUNT_POLL_SECONDS, self._update_sub_counts
        )
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_thread_ident: Optional[int] = None

    def _read_parameters(self, defaults: BridgeConfig) -> BridgeConfig:
        node = self.node
        config = BridgeConfig(
            bind_address=node.declare_parameter("bind_address", defaults.bind_address).value,
            port=node.declare_parameter("port", defaults.port).value,
            max_payload_bytes=node.declare_parameter(
                "max_payload_bytes", defaults.max_payload_bytes
            ).value,
            max_queued_bytes=node.declare_parameter(
                "max_queued_bytes", defaults.max_queued_bytes
            ).value,
            # Dynamic typing: "-p idle_timeout_seconds:=5" (an integer) is accepted as well.
            idle_timeout_seconds=float(
                node.declare_parameter(
                    "idle_timeout_seconds",
                    float(defaults.idle_timeout_seconds),
                    ParameterDescriptor(dynamic_typing=True),
                ).value
            ),
            hello_timeout_seconds=float(
                node.declare_parameter(
                    "hello_timeout_seconds",
                    float(defaults.hello_timeout_seconds),
                    ParameterDescriptor(dynamic_typing=True),
                ).value
            ),
            max_clients=node.declare_parameter("max_clients", defaults.max_clients).value,
        )
        if not 0 <= config.port <= 65535:
            raise ValueError("port must be in [0, 65535], got {}".format(config.port))
        if config.idle_timeout_seconds < 0 or config.hello_timeout_seconds < 0:
            raise ValueError(
                "idle_timeout_seconds and hello_timeout_seconds must be >= 0 (0 disables), "
                "got {} and {}".format(config.idle_timeout_seconds, config.hello_timeout_seconds)
            )
        if config.max_clients < 0:
            raise ValueError(
                "max_clients must be >= 0 (0 = unlimited), got {}".format(config.max_clients)
            )
        if not MIN_PAYLOAD_LIMIT_BYTES <= config.max_payload_bytes <= 0xFFFFFFFF:
            raise ValueError(
                "max_payload_bytes must be in [{}, {}], got {}".format(
                    MIN_PAYLOAD_LIMIT_BYTES, 0xFFFFFFFF, config.max_payload_bytes
                )
            )
        minimum_queue = config.max_payload_bytes + proto.HEADER_SIZE
        if config.max_queued_bytes < minimum_queue:
            self.logger.warn(
                "max_queued_bytes ({}) is smaller than one maximum frame; using {}".format(
                    config.max_queued_bytes, minimum_queue
                )
            )
            config.max_queued_bytes = minimum_queue
        return config

    # -- executor ----------------------------------------------------------

    def start(self) -> None:
        self._executor = SingleThreadedExecutor(context=self.node.context)
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._spin, name="ubot-ros-executor", daemon=True
        )
        self._spin_thread.start()

    def _spin(self) -> None:
        self._spin_thread_ident = threading.get_ident()
        assert self._executor is not None
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:  # pragma: no cover - defensive
            self.logger.error("ROS executor stopped unexpectedly: {!r}".format(exc))

    def is_spinning(self) -> bool:
        return self._spin_thread is not None and self._spin_thread.is_alive()

    def stop(self) -> None:
        with self._ops_lock:
            self._accepting_ops = False
            pending = list(self._ops)
            self._ops.clear()
        for future, _, _ in pending:
            future.cancel()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        if self._spin_thread is not None:
            self._spin_thread.join(3.0)

    def destroy(self) -> None:
        self.node.destroy_node()

    def run_in_ros_thread(
        self, fn: Callable[..., Any], *args: Any, cancel_on_timeout: bool = True
    ) -> Any:
        """Run ``fn(*args)`` on the executor thread and return its result (or raise its error).

        When the executor does not get to it within :data:`OP_TIMEOUT_SECONDS` a
        :class:`RequestError` is raised. The queued operation is then cancelled, unless
        ``cancel_on_timeout`` is false: such an operation (releasing a disconnected client's
        entities) must still run once the executor recovers.
        """
        if threading.get_ident() == self._spin_thread_ident:
            return fn(*args)
        future: Future = Future()
        with self._ops_lock:
            if not self._accepting_ops:
                raise BridgeStopped()
            self._ops.append((future, fn, args))
        self._ops_guard.trigger()
        try:
            return future.result(timeout=OP_TIMEOUT_SECONDS)
        except CancelledError:
            raise BridgeStopped()
        except FutureTimeoutError:
            if cancel_on_timeout:
                future.cancel()
            raise RequestError(
                ErrorCode.INTERNAL_ERROR,
                "the ROS executor did not respond within {} s".format(OP_TIMEOUT_SECONDS),
            )

    def _run_pending_ops(self) -> None:
        while True:
            with self._ops_lock:
                if not self._ops:
                    return
                future, fn, args = self._ops.popleft()
            if not future.set_running_or_notify_cancel():
                continue
            try:
                future.set_result(fn(*args))
            except Exception as exc:
                future.set_exception(exc)

    # -- clients -----------------------------------------------------------

    def attach_client(self, client: ClientConnection) -> None:
        with self._clients_lock:
            self._clients.add(client)

    def client_snapshot(self) -> List[ClientConnection]:
        with self._clients_lock:
            return list(self._clients)

    def detach_client(self, client: ClientConnection) -> None:
        with self._clients_lock:
            self._clients.discard(client)

    def release_client(self, client: ClientConnection) -> None:
        """Release every ROS entity of a disconnected client.

        The release is never cancelled: if the executor is stalled for longer than
        :data:`OP_TIMEOUT_SECONDS` the operation stays queued and runs (in order) when it
        recovers, so a stall cannot leak publishers, subscriptions or topic claims.
        """
        try:
            self.run_in_ros_thread(self._release_client_entities, client, cancel_on_timeout=False)
        except BridgeStopped:
            pass  # destroy_node() releases everything
        except RequestError:
            self.logger.warn(
                "{}: ROS executor busy; entity release deferred until it recovers".format(
                    client.label
                )
            )
        finally:
            self.detach_client(client)

    def _release_client_entities(self, client: ClientConnection) -> None:
        for shared in list(client.publications.values()):
            self._release_publisher(shared)
        client.publications.clear()
        for entry in list(client.subscriptions.values()):
            self._destroy_subscription(entry)
        client.subscriptions.clear()

    # -- ROS graph changes (executor thread only) --------------------------

    @staticmethod
    def _require_installed_message(request: proto.EndpointRequest) -> None:
        """Reject types that are not an installed ROS interface, without importing anything.

        ``get_message`` imports ``<package>.msg``; for a remote peer that could be any module on
        ``sys.path`` (``this/msg/Foo`` imports the stdlib ``this``). The ament index lists the
        interfaces of every installed interface package, so it is checked first.
        """
        package, _, name = request.type_name.split("/")
        try:
            content, _ = get_resource("rosidl_interfaces", package)
        except (LookupError, OSError):
            raise RequestError(
                ErrorCode.UNKNOWN_TYPE,
                "unknown message package '{}'".format(package),
                request.id,
            )
        entries = set(content.splitlines())
        if "msg/{}.msg".format(name) not in entries and "msg/{}.idl".format(name) not in entries:
            raise RequestError(
                ErrorCode.UNKNOWN_TYPE,
                "package '{}' has no message '{}'".format(package, name),
                request.id,
            )

    def _message_class(self, request: proto.EndpointRequest) -> Any:
        message_class = self._message_classes.get(request.type_name)
        if message_class is None:
            self._require_installed_message(request)
            try:
                message_class = get_message(request.type_name)
            except Exception as exc:
                raise RequestError(
                    ErrorCode.UNKNOWN_TYPE,
                    "cannot load message type '{}': {}".format(
                        request.type_name, proto.clip_text(exc, 200)
                    ),
                    request.id,
                )
            self._message_classes[request.type_name] = message_class
        return message_class

    @staticmethod
    def _validate_ros_topic(request: proto.EndpointRequest) -> None:
        try:
            validate_full_topic_name(request.topic)
        except InvalidTopicNameException as exc:
            raise RequestError(ErrorCode.INVALID_TOPIC, str(exc), request.id)

    def _claim_topic(self, topic: str, type_name: str, request_id: int) -> None:
        entry = self._topic_types.get(topic)
        if entry is None:
            self._topic_types[topic] = [type_name, 1]
            return
        if entry[0] != type_name:
            raise RequestError(
                ErrorCode.TYPE_CONFLICT,
                "topic {} is already used by the bridge with type {}, requested {}".format(
                    topic, entry[0], type_name
                ),
                request_id,
            )
        entry[1] += 1

    def _release_topic(self, topic: str) -> None:
        entry = self._topic_types.get(topic)
        if entry is None:
            return
        entry[1] -= 1
        if entry[1] <= 0:
            del self._topic_types[topic]

    def advertise(self, client: ClientConnection, request: proto.EndpointRequest) -> None:
        if request.id in client.publications:
            raise RequestError(
                ErrorCode.DUPLICATE_ID,
                "publisher id {} is already advertised on this connection".format(request.id),
                request.id,
            )
        message_class = self._message_class(request)
        self._validate_ros_topic(request)
        shared = self._publishers_by_topic.get(request.topic)
        if shared is not None:
            if shared.type_name != request.type_name:
                raise RequestError(
                    ErrorCode.TYPE_CONFLICT,
                    "topic {} is already advertised with type {}, requested {}".format(
                        request.topic, shared.type_name, request.type_name
                    ),
                    request.id,
                )
            if shared.qos != request.qos:
                raise RequestError(
                    ErrorCode.QOS_CONFLICT,
                    "topic {} is already advertised with QoS {}, requested {}".format(
                        request.topic, shared.qos, request.qos
                    ),
                    request.id,
                )
            shared.refs += 1
        else:
            self._claim_topic(request.topic, request.type_name, request.id)
            try:
                publisher = self.node.create_publisher(
                    message_class, request.topic, to_qos_profile(request.qos)
                )
            except Exception as exc:
                self._release_topic(request.topic)
                raise RequestError(
                    ErrorCode.ROS_ERROR, "create_publisher failed: {}".format(exc), request.id
                )
            shared = SharedPublisher(request.topic, request.type_name, request.qos, publisher)
            self._publishers_by_topic[request.topic] = shared
        client.publications[request.id] = shared
        self.logger.info(
            "{}: advertised {} [{}] id={} qos={} (clients sharing it: {})".format(
                client.label,
                request.topic,
                request.type_name,
                request.id,
                request.qos,
                shared.refs,
            )
        )

    def unadvertise(self, client: ClientConnection, pub_id: int) -> None:
        shared = client.publications.pop(pub_id, None)
        if shared is None:
            raise RequestError(
                ErrorCode.UNKNOWN_ID, "publisher id {} is not advertised".format(pub_id), pub_id
            )
        self._release_publisher(shared)
        self.logger.info("{}: unadvertised {} id={}".format(client.label, shared.topic, pub_id))

    def _release_publisher(self, shared: SharedPublisher) -> None:
        shared.refs -= 1
        if shared.refs > 0:
            return
        if self._publishers_by_topic.get(shared.topic) is shared:
            del self._publishers_by_topic[shared.topic]
        self._release_topic(shared.topic)
        try:
            self.node.destroy_publisher(shared.publisher)
        except Exception as exc:  # pragma: no cover - defensive
            self.logger.warn("destroy_publisher({}) failed: {!r}".format(shared.topic, exc))

    def subscribe(self, client: ClientConnection, request: proto.EndpointRequest) -> None:
        if request.id in client.subscriptions:
            raise RequestError(
                ErrorCode.DUPLICATE_ID,
                "subscription id {} already exists on this connection".format(request.id),
                request.id,
            )
        message_class = self._message_class(request)
        self._validate_ros_topic(request)
        self._claim_topic(request.topic, request.type_name, request.id)
        entry = ClientSubscription(request.id, request.topic, request.type_name, request.qos)
        try:
            entry.subscription = self.node.create_subscription(
                message_class,
                request.topic,
                functools.partial(self._on_ros_message, client, entry),
                to_qos_profile(request.qos),
                raw=True,
            )
        except Exception as exc:
            self._release_topic(request.topic)
            raise RequestError(
                ErrorCode.ROS_ERROR, "create_subscription failed: {}".format(exc), request.id
            )
        client.subscriptions[request.id] = entry
        self.logger.info(
            "{}: subscribed {} [{}] id={} qos={}".format(
                client.label, request.topic, request.type_name, request.id, request.qos
            )
        )

    def unsubscribe(self, client: ClientConnection, sub_id: int) -> None:
        entry = client.subscriptions.pop(sub_id, None)
        if entry is None:
            raise RequestError(
                ErrorCode.UNKNOWN_ID, "subscription id {} does not exist".format(sub_id), sub_id
            )
        self._destroy_subscription(entry)
        self.logger.info("{}: unsubscribed {} id={}".format(client.label, entry.topic, sub_id))

    def _destroy_subscription(self, entry: ClientSubscription) -> None:
        self._release_topic(entry.topic)
        if entry.subscription is None:
            return
        try:
            self.node.destroy_subscription(entry.subscription)
        except Exception as exc:  # pragma: no cover - defensive
            self.logger.warn("destroy_subscription({}) failed: {!r}".format(entry.topic, exc))
        entry.subscription = None

    # -- executor callbacks ------------------------------------------------

    def _on_ros_message(
        self, client: ClientConnection, entry: ClientSubscription, data: bytes
    ) -> None:
        try:
            client.deliver_ros_message(entry, data)
        except Exception as exc:  # pragma: no cover - defensive
            self.logger.warn(
                "{}: forwarding {} failed: {!r}".format(client.label, entry.topic, exc)
            )

    def _update_sub_counts(self) -> None:
        try:
            now = time.monotonic()
            cache: Dict[int, int] = {}
            for client in self.client_snapshot():
                if not client.hello_done or client.is_closing:
                    continue
                counts: Dict[str, int] = {}
                for pub_id, shared in client.publications.items():
                    key = id(shared)
                    if key not in cache:
                        cache[key] = shared.publisher.get_subscription_count()
                    counts[str(pub_id)] = cache[key]
                to_send = client.sub_count_throttle.update(counts, now)
                if to_send is not None:
                    client.send_json(Op.SUB_COUNT, proto.make_sub_count(to_send))
        except Exception as exc:  # pragma: no cover - defensive
            self.logger.warn("SUB_COUNT update failed: {!r}".format(exc))


class BridgeServer:
    """TCP listener that creates one :class:`ClientConnection` per accepted socket."""

    def __init__(self, core: BridgeCore) -> None:
        self._core = core
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stopping = threading.Event()
        self._surplus_rejected = 0
        self._last_surplus_log = float("-inf")
        self.port = core.config.port

    def start(self) -> None:
        address = self._core.config.bind_address
        family = socket.AF_INET6 if ":" in address else socket.AF_INET
        self._sock = socket.create_server(
            (address, self._core.config.port), family=family, backlog=16
        )
        self._sock.settimeout(0.5)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._accept_loop, name="ubot-accept", daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        """False once the accept thread has died (the process would otherwise listen in vain)."""
        return self._thread is not None and self._thread.is_alive()

    @staticmethod
    def _configure_socket(conn: socket.socket) -> None:
        """Blocking mode (the reader enforces the deadlines), no Nagle delay, keepalive probes."""
        conn.settimeout(None)
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            # The kernel default would take about two hours to notice a vanished peer.
            for name, value in (
                ("TCP_KEEPIDLE", KEEPALIVE_IDLE_SECONDS),
                ("TCP_KEEPINTVL", KEEPALIVE_INTERVAL_SECONDS),
                ("TCP_KEEPCNT", KEEPALIVE_PROBES),
            ):
                option = getattr(socket, name, None)
                if option is not None:
                    conn.setsockopt(socket.IPPROTO_TCP, option, value)
        except OSError:
            pass

    def _reject_surplus(self, conn: socket.socket, address: Tuple[Any, ...]) -> None:
        """Close a connection beyond ``max_clients`` after a best-effort ERROR (never blocks)."""
        limit = self._core.config.max_clients
        now = time.monotonic()
        self._surplus_rejected += 1
        if now - self._last_surplus_log >= SURPLUS_LOG_INTERVAL_SECONDS:
            self._last_surplus_log = now
            self._core.logger.warn(
                "{} client(s) rejected so far: {} clients are connected (max_clients={})".format(
                    self._surplus_rejected, limit, limit
                )
            )
        try:
            conn.setblocking(False)
            error = proto.make_error(
                None,
                ErrorCode.TOO_MANY_CLIENTS,
                "the bridge already serves {} clients (max_clients)".format(limit),
            )
            conn.send(proto.encode_json_frame(Op.ERROR, error))
        except OSError:
            pass
        try:
            conn.close()
        except OSError:
            pass

    def _accept_loop(self) -> None:
        assert self._sock is not None
        number = 0
        accept_failures = 0
        while not self._stopping.is_set():
            try:
                conn, address = self._sock.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                if self._stopping.is_set():
                    return
                # Typically out of file descriptors: back off instead of spinning, log rarely.
                accept_failures += 1
                if accept_failures == 1 or accept_failures % 25 == 0:
                    self._core.logger.warn(
                        "accept failed ({} time(s) in a row): {}".format(accept_failures, exc)
                    )
                time.sleep(min(ACCEPT_RETRY_MAX_SECONDS, ACCEPT_RETRY_SECONDS * accept_failures))
                continue
            accept_failures = 0

            max_clients = self._core.config.max_clients
            if max_clients > 0 and len(self._core.client_snapshot()) >= max_clients:
                self._reject_surplus(conn, address)
                continue

            number += 1
            client: Optional[ClientConnection] = None
            try:
                self._configure_socket(conn)
                client = ClientConnection(self._core, conn, address, number)
                self._core.attach_client(client)
                self._core.logger.info("{} connected".format(client.label))
                client.start()
            except Exception as exc:
                # Typically "can't start new thread". A per-connection failure must never end
                # this loop: the bridge would keep running but never accept again.
                self._core.logger.error(
                    "connection #{} from {}:{} dropped during setup: {!r}".format(
                        number, address[0], address[1], exc
                    )
                )
                if client is not None:
                    self._core.detach_client(client)
                    client.abort("setup failed")  # wakes a writer that was already started
                try:
                    conn.close()
                except OSError:
                    pass
                time.sleep(ACCEPT_RETRY_SECONDS)

    def stop(self) -> None:
        self._stopping.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(2.0)
        clients = self._core.client_snapshot()
        for client in clients:
            client.abort("bridge shutting down")
        deadline = time.monotonic() + CLIENT_JOIN_TIMEOUT_SECONDS
        for client in clients:
            client.finished.wait(max(0.0, deadline - time.monotonic()))


def parse_arguments(argv: Sequence[str]) -> BridgeConfig:
    parser = argparse.ArgumentParser(
        prog="ubot_ros_bridge",
        description="TCP bridge between the uBot UE plugin (UBotROS) and ROS 2. "
        "ROS parameters (--ros-args -p name:=value) override these options.",
    )
    parser.add_argument(
        "--bind-address",
        "--bind_address",
        dest="bind_address",
        default=proto.DEFAULT_BIND_ADDRESS,
        help="listen address (default: %(default)s)",
    )
    parser.add_argument(
        "--port", type=int, default=proto.DEFAULT_PORT, help="listen port (default: %(default)s)"
    )
    parser.add_argument(
        "--max-payload-bytes",
        "--max_payload_bytes",
        dest="max_payload_bytes",
        type=int,
        default=proto.DEFAULT_MAX_PAYLOAD_BYTES,
        help="largest accepted/sent frame payload (default: %(default)s)",
    )
    parser.add_argument(
        "--max-queued-bytes",
        "--max_queued_bytes",
        dest="max_queued_bytes",
        type=int,
        default=proto.DEFAULT_MAX_QUEUED_BYTES,
        help="per-client send queue limit before MESSAGE frames are dropped "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--idle-timeout-seconds",
        "--idle_timeout_seconds",
        dest="idle_timeout_seconds",
        type=float,
        default=proto.DEFAULT_IDLE_TIMEOUT_SECONDS,
        help="drop a client that sends nothing for this long; 0 disables (default: %(default)s)",
    )
    parser.add_argument(
        "--hello-timeout-seconds",
        "--hello_timeout_seconds",
        dest="hello_timeout_seconds",
        type=float,
        default=proto.DEFAULT_HELLO_TIMEOUT_SECONDS,
        help="drop a connection that has not sent HELLO after this long; 0 disables "
        "(default: %(default)s)",
    )
    parser.add_argument(
        "--max-clients",
        "--max_clients",
        dest="max_clients",
        type=int,
        default=proto.DEFAULT_MAX_CLIENTS,
        help="concurrent clients accepted, more are refused; 0 = unlimited (default: %(default)s)",
    )
    parser.add_argument("--version", action="version", version=SERVER_NAME)
    namespace = parser.parse_args(list(argv))
    return BridgeConfig(
        bind_address=namespace.bind_address,
        port=namespace.port,
        max_payload_bytes=namespace.max_payload_bytes,
        max_queued_bytes=namespace.max_queued_bytes,
        idle_timeout_seconds=namespace.idle_timeout_seconds,
        hello_timeout_seconds=namespace.hello_timeout_seconds,
        max_clients=namespace.max_clients,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv if argv is None else argv)
    defaults = parse_arguments(remove_ros_args(args=args)[1:])

    stop_event = threading.Event()

    def _request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, _request_stop)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    core: Optional[BridgeCore] = None
    server: Optional[BridgeServer] = None
    exit_code = 0
    try:
        core = BridgeCore(defaults)
        core.start()
        server = BridgeServer(core)
        server.start()
        core.logger.info(
            "{} listening on {}:{} (ROS {}, max payload {}, max queued {} per client, "
            "idle timeout {:g} s, HELLO timeout {:g} s, max clients {})".format(
                SERVER_NAME,
                core.config.bind_address,
                server.port,
                core.ros_distro,
                _format_bytes(core.config.max_payload_bytes),
                _format_bytes(core.config.max_queued_bytes),
                core.config.idle_timeout_seconds,
                core.config.hello_timeout_seconds,
                core.config.max_clients,
            )
        )
        while not stop_event.wait(0.5):
            if not core.is_spinning():
                core.logger.error("the ROS executor stopped; shutting down")
                exit_code = 1
                break
            if not server.is_alive():
                core.logger.error("the accept thread stopped; shutting down")
                exit_code = 1
                break
        if stop_event.is_set():
            core.logger.info("shutdown requested")
    except Exception as exc:
        message = "ubot_ros_bridge failed: {}".format(exc)
        if core is not None:
            core.logger.error(message)
        else:
            print(message, file=sys.stderr)
        exit_code = 1
    finally:
        if server is not None:
            server.stop()
        if core is not None:
            core.stop()
            core.destroy()
        rclpy.try_shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
