"""Wire protocol v1 of the uBot ROS bridge.

This module is pure Python (standard library only, no ROS imports) so it can be
unit tested without a ROS graph. It implements:

* frame encoding and an incremental frame parser that copes with partial reads
  and several frames per read,
* JSON payload helpers and validation of ADVERTISE / SUBSCRIBE requests,
* the bounded per-client send queue and the SUB_COUNT throttle used by the
  bridge node.

Frame layout (all integers little-endian)::

    offset  size  field
    0       4     magic b"UBRB"
    4       1     version (1)
    5       1     op
    6       2     flags (reserved, 0)
    8       4     payload length N
    12      N     payload
"""

from __future__ import annotations

import json
import re
import struct
import threading
from collections import deque
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Any, Deque, Dict, Iterator, List, Mapping, Optional, Tuple, Union

MAGIC = b"UBRB"
VERSION = 1
PROTOCOL_VERSION = 1
HEADER_SIZE = 12
DEFAULT_PORT = 8268
DEFAULT_BIND_ADDRESS = "0.0.0.0"
DEFAULT_MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
DEFAULT_MAX_QUEUED_BYTES = 128 * 1024 * 1024
#: A client that sends nothing for this long is dropped (0 disables); UE pings every 2 s.
DEFAULT_IDLE_TIMEOUT_SECONDS = 10.0
#: A connection must complete the HELLO within this many seconds of being accepted (0 disables).
DEFAULT_HELLO_TIMEOUT_SECONDS = 5.0
#: Concurrent clients accepted (0 = unlimited). One UE world is one client.
DEFAULT_MAX_CLIENTS = 32
MAX_ID = 0xFFFFFFFF
MAX_QOS_DEPTH = 0x7FFFFFFF
SUB_COUNT_MIN_INTERVAL_SECONDS = 0.5

#: Longest topic / type name accepted (also checked before any pattern matching).
MAX_TOPIC_LENGTH = 255
MAX_TYPE_NAME_LENGTH = 255
#: Client text (HELLO name, topic, type, ...) is clipped to this many characters when it is logged
#: or echoed.
MAX_CLIENT_NAME_CHARS = 128
MAX_ECHOED_CHARS = 96
MAX_ERROR_MESSAGE_CHARS = 512

#: Payload limits the bridge applies on top of ``max_payload_bytes``. A 12-byte header must not be
#: able to make the bridge reserve megabytes, so only PUBLISH may be large and nothing is large
#: before HELLO.
PRE_HELLO_PAYLOAD_LIMIT_BYTES = 4 * 1024
CONTROL_PAYLOAD_LIMIT_BYTES = 64 * 1024
SMALL_PAYLOAD_LIMIT_BYTES = 4 * 1024

_HEADER_STRUCT = struct.Struct("<4sBBHI")
_ID_STRUCT = struct.Struct("<I")

BytesLike = Union[bytes, bytearray, memoryview]


class Op(IntEnum):
    """Frame operation codes."""

    HELLO = 1
    HELLO_ACK = 2
    ADVERTISE = 3
    UNADVERTISE = 4
    PUBLISH = 5
    SUBSCRIBE = 6
    UNSUBSCRIBE = 7
    MESSAGE = 8
    SUB_COUNT = 9
    PING = 10
    PONG = 11
    ERROR = 12


#: Ops a client (UE) may send to the bridge.
CLIENT_OPS = frozenset(
    {
        Op.HELLO,
        Op.ADVERTISE,
        Op.UNADVERTISE,
        Op.PUBLISH,
        Op.SUBSCRIBE,
        Op.UNSUBSCRIBE,
        Op.PING,
        Op.PONG,
    }
)

#: Ops only the bridge sends.
SERVER_OPS = frozenset({Op.HELLO_ACK, Op.MESSAGE, Op.SUB_COUNT, Op.PING, Op.PONG, Op.ERROR})

#: Client ops whose payload is a (small) JSON object.
_JSON_OPS = frozenset({Op.HELLO, Op.ADVERTISE, Op.UNADVERTISE, Op.SUBSCRIBE, Op.UNSUBSCRIBE})

KNOWN_OPS = frozenset(int(op) for op in Op)


class ErrorCode:
    """Values of the ``code`` field of ERROR frames."""

    BAD_REQUEST = "bad_request"
    UNSUPPORTED_PROTOCOL = "unsupported_protocol"
    HELLO_REQUIRED = "hello_required"
    UNKNOWN_OP = "unknown_op"
    UNEXPECTED_OP = "unexpected_op"
    INVALID_TOPIC = "invalid_topic"
    UNKNOWN_TYPE = "unknown_type"
    INVALID_QOS = "invalid_qos"
    DUPLICATE_ID = "duplicate_id"
    UNKNOWN_ID = "unknown_id"
    TYPE_CONFLICT = "type_conflict"
    QOS_CONFLICT = "qos_conflict"
    BAD_PAYLOAD = "bad_payload"
    PUBLISH_FAILED = "publish_failed"
    MESSAGE_TOO_LARGE = "message_too_large"
    ROS_ERROR = "ros_error"
    PROTOCOL_ERROR = "protocol_error"
    INTERNAL_ERROR = "internal_error"
    TOO_MANY_CLIENTS = "too_many_clients"


class ProtocolError(Exception):
    """Fatal framing error (bad magic, version or size). The connection must be closed."""


class RequestError(Exception):
    """Non-fatal request error, reported to the client as an ERROR frame."""

    def __init__(self, code: str, message: str, request_id: Optional[int] = None) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.request_id = request_id

    def to_payload(self) -> Dict[str, Any]:
        return {"id": self.request_id, "code": self.code, "message": self.message}


@dataclass(frozen=True)
class FrameHeader:
    op: int
    flags: int
    length: int


@dataclass(frozen=True)
class Frame:
    op: int
    payload: BytesLike = b""
    flags: int = 0


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------


def encode_header(op: int, length: int, flags: int = 0) -> bytes:
    """Return the 12-byte header of a frame."""
    if not 0 <= op <= 0xFF:
        raise ValueError("op out of range: {}".format(op))
    if not 0 <= length <= 0xFFFFFFFF:
        raise ValueError("payload too large for the length field: {}".format(length))
    return _HEADER_STRUCT.pack(MAGIC, VERSION, op, flags, length)


def encode_frame(op: int, payload: BytesLike = b"", flags: int = 0) -> bytes:
    """Return a complete frame (header followed by the payload)."""
    return encode_header(op, len(payload), flags) + bytes(payload)


def payload_limit_for_op(op: int, max_payload_bytes: int) -> int:
    """Largest payload the bridge accepts for ``op`` from a client.

    Only PUBLISH may approach ``max_payload_bytes``; the JSON control ops are capped at
    :data:`CONTROL_PAYLOAD_LIMIT_BYTES` and everything else (PING, PONG, ops the client must not
    send, unknown ops) at :data:`SMALL_PAYLOAD_LIMIT_BYTES`.
    """
    if op == Op.PUBLISH:
        return max_payload_bytes
    if op in _JSON_OPS:
        return min(max_payload_bytes, CONTROL_PAYLOAD_LIMIT_BYTES)
    return min(max_payload_bytes, SMALL_PAYLOAD_LIMIT_BYTES)


def decode_header(
    data: BytesLike,
    max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES,
    per_op_limits: bool = False,
) -> FrameHeader:
    """Decode and validate a 12-byte header.

    Raises :class:`ProtocolError` on bad magic, unsupported version or a
    payload larger than ``max_payload_bytes`` (or, with ``per_op_limits``, larger
    than :func:`payload_limit_for_op` for its op). Flags are reserved and ignored.
    """
    if len(data) != HEADER_SIZE:
        raise ProtocolError("header must be {} bytes, got {}".format(HEADER_SIZE, len(data)))
    magic, version, op, flags, length = _HEADER_STRUCT.unpack(bytes(data))
    if magic != MAGIC:
        raise ProtocolError("bad magic {!r}".format(magic))
    if version != VERSION:
        raise ProtocolError("unsupported protocol version {}".format(version))
    limit = payload_limit_for_op(op, max_payload_bytes) if per_op_limits else max_payload_bytes
    if length > limit:
        raise ProtocolError(
            "payload of {} bytes exceeds the limit of {} bytes{}".format(
                length, limit, " for op {}".format(op) if per_op_limits else ""
            )
        )
    return FrameHeader(op=op, flags=flags, length=length)


class FrameParser:
    """Incremental frame parser.

    Feed it whatever ``recv`` returned; it yields every completed frame and keeps
    partial data for the next call. After a :class:`ProtocolError` the stream is
    unusable and the connection must be closed. The parser copies the input, so
    the caller may reuse its receive buffer after each call.

    The payload buffer grows with the bytes that really arrive: the length field
    of a header is untrusted, so a peer announcing 64 MiB reserves nothing until
    it sends data. ``max_payload_bytes`` may be changed between frames (the bridge
    keeps it small until the HELLO has arrived); with ``per_op_limits`` the limit
    also depends on the op (see :func:`payload_limit_for_op`).
    """

    def __init__(
        self, max_payload_bytes: int = DEFAULT_MAX_PAYLOAD_BYTES, per_op_limits: bool = False
    ) -> None:
        self._max_payload_bytes = max_payload_bytes
        self._per_op_limits = per_op_limits
        self._header = bytearray()
        self._current: Optional[FrameHeader] = None
        self._payload: Optional[bytearray] = None
        self._filled = 0

    @property
    def max_payload_bytes(self) -> int:
        return self._max_payload_bytes

    @max_payload_bytes.setter
    def max_payload_bytes(self, value: int) -> None:
        """Applies to every header parsed from now on (a frame in progress keeps its limit)."""
        self._max_payload_bytes = value

    @property
    def buffered_bytes(self) -> int:
        """Bytes of the current, not yet complete frame."""
        if self._current is None:
            return len(self._header)
        return HEADER_SIZE + self._filled

    def feed(self, data: BytesLike) -> List[Frame]:
        """Consume ``data`` and return the frames it completed."""
        return list(self.iter_frames(data))

    def iter_frames(self, data: BytesLike) -> Iterator[Frame]:
        """Consume ``data`` lazily, yielding frames as soon as they are complete.

        The generator must be exhausted before the next call.
        """
        view = memoryview(data).cast("B")
        size = len(view)
        pos = 0
        while pos < size:
            if self._current is None:
                end = min(pos + HEADER_SIZE - len(self._header), size)
                self._header += view[pos:end]
                pos = end
                if len(self._header) < HEADER_SIZE:
                    break
                header = decode_header(self._header, self._max_payload_bytes, self._per_op_limits)
                self._header.clear()
                if header.length == 0:
                    yield Frame(op=header.op, payload=b"", flags=header.flags)
                    continue
                self._current = header
                self._payload = bytearray()
                self._filled = 0
            else:
                assert self._payload is not None
                end = min(pos + self._current.length - self._filled, size)
                self._payload += view[pos:end]
                self._filled += end - pos
                pos = end
                if self._filled == self._current.length:
                    frame = Frame(
                        op=self._current.op, payload=self._payload, flags=self._current.flags
                    )
                    self._current = None
                    self._payload = None
                    self._filled = 0
                    yield frame


# ---------------------------------------------------------------------------
# Payload helpers
# ---------------------------------------------------------------------------


def encode_json(obj: Mapping[str, Any]) -> bytes:
    return json.dumps(obj, separators=(",", ":")).encode("utf-8")


def encode_json_frame(op: int, obj: Mapping[str, Any]) -> bytes:
    return encode_frame(op, encode_json(obj))


def clip_text(value: Any, limit: int = MAX_ECHOED_CHARS) -> str:
    """Make client-supplied text safe to log or echo: bounded and without control characters.

    Non-printable characters (newlines, ESC, bidi/format characters, lone surrogates, ...) are
    replaced by their escape sequence, and anything beyond ``limit`` characters is cut off and
    summarised as ``...(+N chars)``.
    """
    text = value if isinstance(value, str) else str(value)
    shown = text[:limit]
    clean = "".join(
        ch if ch.isprintable() else ch.encode("unicode_escape").decode("ascii") for ch in shown
    )
    if len(text) > limit:
        clean += "...(+{} chars)".format(len(text) - limit)
    return clean


def decode_json_object(payload: BytesLike) -> Dict[str, Any]:
    """Decode a JSON object payload; raises :class:`RequestError` (bad_request)."""
    try:
        obj = json.loads(bytes(payload).decode("utf-8"))
    except RecursionError:
        # Deeply nested arrays/objects: still just a bad request, not a reader failure.
        raise RequestError(ErrorCode.BAD_REQUEST, "payload JSON is nested too deeply")
    except (UnicodeDecodeError, ValueError) as exc:
        raise RequestError(
            ErrorCode.BAD_REQUEST, "payload is not valid UTF-8 JSON: {}".format(clip_text(exc, 200))
        )
    if not isinstance(obj, dict):
        raise RequestError(ErrorCode.BAD_REQUEST, "payload must be a JSON object")
    return obj


def encode_id_frame(op: int, entity_id: int, data: BytesLike) -> bytes:
    """Build a PUBLISH or MESSAGE frame: u32 LE id followed by CDR bytes."""
    if not 0 <= entity_id <= MAX_ID:
        raise ValueError("id out of range: {}".format(entity_id))
    length = _ID_STRUCT.size + len(data)
    return b"".join((encode_header(op, length), _ID_STRUCT.pack(entity_id), bytes(data)))


def split_id_payload(payload: BytesLike) -> Tuple[int, memoryview]:
    """Split a PUBLISH / MESSAGE payload into (id, CDR bytes view)."""
    view = memoryview(payload).cast("B")
    if len(view) < _ID_STRUCT.size:
        raise RequestError(ErrorCode.BAD_PAYLOAD, "payload shorter than the 4-byte id")
    (entity_id,) = _ID_STRUCT.unpack_from(view, 0)
    return entity_id, view[_ID_STRUCT.size:]


def make_hello(client: str) -> Dict[str, Any]:
    return {"protocol": PROTOCOL_VERSION, "client": client}


def make_hello_ack(server: str, ros_distro: str) -> Dict[str, Any]:
    return {"protocol": PROTOCOL_VERSION, "server": server, "ros_distro": ros_distro}


def make_error(request_id: Optional[int], code: str, message: str) -> Dict[str, Any]:
    return {"id": request_id, "code": code, "message": message}


def make_sub_count(counts: Mapping[str, int]) -> Dict[str, Any]:
    return {"counts": dict(counts)}


def is_valid_id(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= MAX_ID


def parse_id(obj: Mapping[str, Any], key: str = "id") -> int:
    value = obj.get(key)
    if not is_valid_id(value):
        raise RequestError(
            ErrorCode.BAD_REQUEST, "'{}' must be an unsigned 32-bit integer".format(key)
        )
    return int(value)


def parse_hello(obj: Mapping[str, Any]) -> str:
    """Validate a HELLO payload and return the client description."""
    protocol = obj.get("protocol")
    if isinstance(protocol, bool) or protocol != PROTOCOL_VERSION:
        raise RequestError(
            ErrorCode.UNSUPPORTED_PROTOCOL,
            "bridge speaks protocol {}, client requested {}".format(
                PROTOCOL_VERSION, clip_text(repr(protocol), MAX_ECHOED_CHARS)
            ),
        )
    # The name only goes into logs: bounded and without control characters.
    return clip_text(obj.get("client", ""), MAX_CLIENT_NAME_CHARS)


# ---------------------------------------------------------------------------
# Names and QoS
# ---------------------------------------------------------------------------

_TOPIC_RE = re.compile(r"^(?:/[A-Za-z_][A-Za-z0-9_]*)+$")
_TYPE_RE = re.compile(r"^([A-Za-z][A-Za-z0-9_]*)/(?:msg/)?([A-Za-z][A-Za-z0-9_]*)$")


def validate_topic(topic: Any) -> str:
    """Return ``topic`` if it is an absolute ROS 2 topic name, else raise ValueError."""
    if not isinstance(topic, str) or not topic:
        raise ValueError("topic must be a non-empty string")
    if len(topic) > MAX_TOPIC_LENGTH:
        raise ValueError(
            "topic of {} characters is longer than the limit of {}".format(
                len(topic), MAX_TOPIC_LENGTH
            )
        )
    if not topic.startswith("/"):
        raise ValueError("topic '{}' must be absolute (start with '/')".format(clip_text(topic)))
    if not _TOPIC_RE.fullmatch(topic):
        raise ValueError(
            "topic '{}' is not a valid ROS 2 name (tokens of [A-Za-z0-9_] not starting "
            "with a digit, separated by single '/')".format(clip_text(topic))
        )
    return topic


def normalize_type_name(type_name: Any) -> str:
    """Normalize ``pkg/Name`` or ``pkg/msg/Name`` to ``pkg/msg/Name``.

    Raises ValueError for anything else.
    """
    if not isinstance(type_name, str):
        raise ValueError("type must be a string like 'sensor_msgs/msg/Imu'")
    if len(type_name) > MAX_TYPE_NAME_LENGTH:
        raise ValueError(
            "type of {} characters is longer than the limit of {}".format(
                len(type_name), MAX_TYPE_NAME_LENGTH
            )
        )
    match = _TYPE_RE.fullmatch(type_name.strip())
    if not match:
        raise ValueError(
            "type '{}' is not of the form 'pkg/msg/Name'".format(clip_text(type_name))
        )
    return "{}/msg/{}".format(match.group(1), match.group(2))


class Reliability(str, Enum):
    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"


class Durability(str, Enum):
    VOLATILE = "volatile"
    TRANSIENT_LOCAL = "transient_local"


@dataclass(frozen=True)
class QosSpec:
    reliability: Reliability = Reliability.RELIABLE
    durability: Durability = Durability.VOLATILE
    depth: int = 10

    @property
    def is_best_effort(self) -> bool:
        return self.reliability is Reliability.BEST_EFFORT

    def to_json(self) -> Dict[str, Any]:
        return {
            "reliability": self.reliability.value,
            "durability": self.durability.value,
            "depth": self.depth,
        }

    def __str__(self) -> str:
        return "{}/{}/{}".format(self.reliability.value, self.durability.value, self.depth)


def parse_qos(obj: Any) -> QosSpec:
    """Parse the ``qos`` object; missing keys default to reliable / volatile / 10."""
    if obj is None:
        return QosSpec()
    if not isinstance(obj, dict):
        raise ValueError("qos must be a JSON object")
    try:
        reliability = Reliability(obj.get("reliability", Reliability.RELIABLE.value))
    except ValueError:
        raise ValueError("qos.reliability must be 'reliable' or 'best_effort'")
    try:
        durability = Durability(obj.get("durability", Durability.VOLATILE.value))
    except ValueError:
        raise ValueError("qos.durability must be 'volatile' or 'transient_local'")
    depth = obj.get("depth", 10)
    if isinstance(depth, bool) or not isinstance(depth, int) or not 1 <= depth <= MAX_QOS_DEPTH:
        raise ValueError("qos.depth must be an integer >= 1")
    return QosSpec(reliability=reliability, durability=durability, depth=depth)


@dataclass(frozen=True)
class EndpointRequest:
    """A validated ADVERTISE or SUBSCRIBE request."""

    id: int
    topic: str
    type_name: str
    qos: QosSpec


def parse_endpoint_request(obj: Mapping[str, Any]) -> EndpointRequest:
    request_id = parse_id(obj)
    try:
        topic = validate_topic(obj.get("topic"))
    except ValueError as exc:
        raise RequestError(ErrorCode.INVALID_TOPIC, str(exc), request_id)
    try:
        type_name = normalize_type_name(obj.get("type"))
    except ValueError as exc:
        raise RequestError(ErrorCode.UNKNOWN_TYPE, str(exc), request_id)
    try:
        qos = parse_qos(obj.get("qos"))
    except ValueError as exc:
        raise RequestError(ErrorCode.INVALID_QOS, str(exc), request_id)
    return EndpointRequest(id=request_id, topic=topic, type_name=type_name, qos=qos)


# ---------------------------------------------------------------------------
# Send queue and SUB_COUNT throttle
# ---------------------------------------------------------------------------


class FrameClass(IntEnum):
    """How a queued frame may be treated when the queue is full."""

    CONTROL = 0  # HELLO_ACK, PONG, ERROR, SUB_COUNT: small, never dropped
    RELIABLE = 1  # MESSAGE of a reliable subscription
    BEST_EFFORT = 2  # MESSAGE of a best-effort subscription: dropped first


class PutResult(Enum):
    ACCEPTED = "accepted"
    DROPPED = "dropped"  # best-effort frame dropped because the queue is full
    REJECTED = "rejected"  # reliable frame rejected (no best-effort data left to evict)
    OVERFLOW = "overflow"  # control frames exceed the hard limit: the client is not reading
    CLOSED = "closed"


class BoundedFrameQueue:
    """Thread-safe FIFO of encoded frames bounded by bytes.

    Policy when ``max_bytes`` would be exceeded:

    * best-effort MESSAGE frames are dropped,
    * a reliable MESSAGE frame first evicts queued best-effort frames (oldest
      first); if that cannot free enough room it is rejected,
    * control frames are always accepted up to ``max_bytes + control_slack_bytes``;
      beyond that :attr:`PutResult.OVERFLOW` tells the owner to drop the client.
    """

    def __init__(self, max_bytes: int, control_slack_bytes: int = 1024 * 1024) -> None:
        if max_bytes <= 0:
            raise ValueError("max_bytes must be positive")
        self._max_bytes = max_bytes
        self._hard_limit = max_bytes + control_slack_bytes
        self._items: Deque[Tuple[bytes, FrameClass]] = deque()
        self._bytes = 0
        self._best_effort_bytes = 0
        self._closed = False
        self._cond = threading.Condition()
        self.dropped_best_effort = 0
        self.rejected_reliable = 0

    @property
    def max_bytes(self) -> int:
        return self._max_bytes

    @property
    def queued_bytes(self) -> int:
        with self._cond:
            return self._bytes

    @property
    def closed(self) -> bool:
        with self._cond:
            return self._closed

    def __len__(self) -> int:
        with self._cond:
            return len(self._items)

    def put(self, frame: bytes, frame_class: FrameClass) -> PutResult:
        size = len(frame)
        with self._cond:
            if self._closed:
                return PutResult.CLOSED
            if frame_class is FrameClass.CONTROL:
                if self._bytes + size > self._hard_limit:
                    return PutResult.OVERFLOW
            elif self._bytes + size > self._max_bytes:
                if frame_class is FrameClass.BEST_EFFORT:
                    self.dropped_best_effort += 1
                    return PutResult.DROPPED
                if self._bytes - self._best_effort_bytes + size > self._max_bytes:
                    self.rejected_reliable += 1
                    return PutResult.REJECTED
                self._evict_best_effort(self._bytes + size - self._max_bytes)
            self._items.append((frame, frame_class))
            self._bytes += size
            if frame_class is FrameClass.BEST_EFFORT:
                self._best_effort_bytes += size
            self._cond.notify()
            return PutResult.ACCEPTED

    def _evict_best_effort(self, needed: int) -> None:
        freed = 0
        kept: Deque[Tuple[bytes, FrameClass]] = deque()
        for frame, frame_class in self._items:
            if freed < needed and frame_class is FrameClass.BEST_EFFORT:
                freed += len(frame)
                self.dropped_best_effort += 1
                continue
            kept.append((frame, frame_class))
        self._items = kept
        self._bytes -= freed
        self._best_effort_bytes -= freed

    def get(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Pop the oldest frame; ``None`` on timeout or when closed and drained."""
        with self._cond:
            if not self._items and not self._closed:
                self._cond.wait_for(lambda: self._items or self._closed, timeout)
            if not self._items:
                return None
            frame, frame_class = self._items.popleft()
            self._bytes -= len(frame)
            if frame_class is FrameClass.BEST_EFFORT:
                self._best_effort_bytes -= len(frame)
            return frame

    def close(self, discard: bool = False) -> None:
        """Refuse new frames; queued frames are still returned unless ``discard``."""
        with self._cond:
            self._closed = True
            if discard:
                self._items.clear()
                self._bytes = 0
                self._best_effort_bytes = 0
            self._cond.notify_all()


class SubCountThrottle:
    """Decides when to send SUB_COUNT: on change, at most every ``min_interval`` seconds."""

    def __init__(self, min_interval: float = SUB_COUNT_MIN_INTERVAL_SECONDS) -> None:
        self._min_interval = min_interval
        self._last_sent: Dict[str, int] = {}
        self._last_time: Optional[float] = None

    def update(self, counts: Mapping[str, int], now: float) -> Optional[Dict[str, int]]:
        """Return the counts to send now, or ``None`` if nothing should be sent."""
        if dict(counts) == self._last_sent:
            return None
        if self._last_time is not None and now - self._last_time < self._min_interval:
            return None
        self._last_sent = dict(counts)
        self._last_time = now
        return dict(self._last_sent)


class TokenBucket:
    """Token bucket rate limiter driven by an explicit clock (``now`` in seconds)."""

    def __init__(self, capacity: float, refill_per_second: float) -> None:
        if capacity < 1 or refill_per_second < 0:
            raise ValueError("capacity must be >= 1 and refill_per_second >= 0")
        self._capacity = float(capacity)
        self._refill = float(refill_per_second)
        self._tokens = float(capacity)
        self._last: Optional[float] = None

    def try_take(self, now: float) -> bool:
        """Take one token if available; ``False`` when the caller is over its budget."""
        if self._last is not None and now > self._last:
            self._tokens = min(self._capacity, self._tokens + (now - self._last) * self._refill)
        self._last = now if self._last is None else max(self._last, now)
        if self._tokens >= 1.0:
            self._tokens -= 1.0
            return True
        return False
