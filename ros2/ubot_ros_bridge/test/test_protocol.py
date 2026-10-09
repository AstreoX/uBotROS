"""Unit tests for ubot_ros_bridge.protocol (no ROS graph needed).

Run with ``python3 -m pytest test/test_protocol.py`` or
``python3 -m unittest discover -s test -p 'test_protocol.py'`` from the package directory.
"""

import json
import os
import struct
import sys
import threading
import time
import tracemalloc
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ubot_ros_bridge import protocol as proto  # noqa: E402
from ubot_ros_bridge.protocol import (  # noqa: E402
    BoundedFrameQueue,
    ErrorCode,
    FrameClass,
    FrameParser,
    Op,
    ProtocolError,
    PutResult,
    QosSpec,
    RequestError,
    SubCountThrottle,
    TokenBucket,
)


def _frame(op, payload=b""):
    return proto.encode_frame(op, payload)


class FramingTest(unittest.TestCase):
    def test_header_layout_matches_contract(self):
        frame = proto.encode_frame(Op.PING)
        self.assertEqual(frame, bytes([0x55, 0x42, 0x52, 0x42, 1, 10, 0, 0, 0, 0, 0, 0]))
        frame = proto.encode_frame(Op.HELLO, b"abc")
        self.assertEqual(frame[:4], b"UBRB")
        self.assertEqual(frame[4], 1)
        self.assertEqual(frame[5], int(Op.HELLO))
        self.assertEqual(frame[6:8], b"\x00\x00")
        self.assertEqual(frame[8:12], struct.pack("<I", 3))
        self.assertEqual(frame[12:], b"abc")

    def test_op_values(self):
        expected = {
            "HELLO": 1,
            "HELLO_ACK": 2,
            "ADVERTISE": 3,
            "UNADVERTISE": 4,
            "PUBLISH": 5,
            "SUBSCRIBE": 6,
            "UNSUBSCRIBE": 7,
            "MESSAGE": 8,
            "SUB_COUNT": 9,
            "PING": 10,
            "PONG": 11,
            "ERROR": 12,
        }
        self.assertEqual({op.name: int(op) for op in Op}, expected)

    def test_decode_header_round_trip(self):
        header = proto.decode_header(proto.encode_header(Op.PUBLISH, 1234, flags=7))
        self.assertEqual((header.op, header.flags, header.length), (int(Op.PUBLISH), 7, 1234))

    def test_decode_header_rejects_bad_magic(self):
        data = bytearray(proto.encode_header(Op.PING, 0))
        data[0] = 0x00
        with self.assertRaises(ProtocolError):
            proto.decode_header(bytes(data))

    def test_decode_header_rejects_bad_version(self):
        data = bytearray(proto.encode_header(Op.PING, 0))
        data[4] = 2
        with self.assertRaises(ProtocolError):
            proto.decode_header(bytes(data))

    def test_decode_header_rejects_oversize(self):
        with self.assertRaises(ProtocolError):
            proto.decode_header(proto.encode_header(Op.PUBLISH, 101), max_payload_bytes=100)
        proto.decode_header(proto.encode_header(Op.PUBLISH, 100), max_payload_bytes=100)
        with self.assertRaises(ProtocolError):
            proto.decode_header(
                proto.encode_header(Op.PUBLISH, proto.DEFAULT_MAX_PAYLOAD_BYTES + 1)
            )

    def test_id_frames(self):
        frame = proto.encode_id_frame(Op.MESSAGE, 0xDEADBEEF, b"\x00\x01\x00\x00xyz")
        parsed = FrameParser().feed(frame)
        self.assertEqual(len(parsed), 1)
        self.assertEqual(parsed[0].op, int(Op.MESSAGE))
        entity_id, cdr = proto.split_id_payload(parsed[0].payload)
        self.assertEqual(entity_id, 0xDEADBEEF)
        self.assertEqual(bytes(cdr), b"\x00\x01\x00\x00xyz")
        self.assertEqual(frame[12:16], b"\xef\xbe\xad\xde")

    def test_split_id_payload_rejects_short_payload(self):
        with self.assertRaises(RequestError) as ctx:
            proto.split_id_payload(b"\x01\x02\x03")
        self.assertEqual(ctx.exception.code, ErrorCode.BAD_PAYLOAD)


class FrameParserTest(unittest.TestCase):
    def setUp(self):
        self.frames = [
            _frame(Op.HELLO, b'{"protocol":1}'),
            _frame(Op.PING),
            proto.encode_id_frame(Op.PUBLISH, 3, bytes(range(256)) * 5),
            _frame(Op.PONG),
        ]
        self.stream = b"".join(self.frames)

    def _check(self, frames):
        self.assertEqual([f.op for f in frames], [Op.HELLO, Op.PING, Op.PUBLISH, Op.PONG])
        self.assertEqual(bytes(frames[0].payload), b'{"protocol":1}')
        self.assertEqual(bytes(frames[1].payload), b"")
        self.assertEqual(bytes(frames[2].payload), self.frames[2][12:])

    def test_all_frames_in_one_read(self):
        self._check(FrameParser().feed(self.stream))

    def test_byte_by_byte(self):
        parser = FrameParser()
        frames = []
        for index in range(len(self.stream)):
            frames.extend(parser.feed(self.stream[index:index + 1]))
        self._check(frames)
        self.assertEqual(parser.buffered_bytes, 0)

    def test_arbitrary_split_points(self):
        for chunk in (2, 5, 11, 12, 13, 64, 700):
            parser = FrameParser()
            frames = []
            for start in range(0, len(self.stream), chunk):
                frames.extend(parser.feed(memoryview(self.stream)[start:start + chunk]))
            self._check(frames)

    def test_partial_frame_waits_for_more_data(self):
        parser = FrameParser()
        self.assertEqual(parser.feed(self.frames[2][:20]), [])
        self.assertEqual(parser.buffered_bytes, 20)
        frames = parser.feed(self.frames[2][20:])
        self.assertEqual(len(frames), 1)

    def test_parser_copies_input(self):
        parser = FrameParser()
        buffer = bytearray(self.frames[0])
        frames = parser.feed(buffer)
        buffer[:] = b"\x00" * len(buffer)
        self.assertEqual(bytes(frames[0].payload), b'{"protocol":1}')

    def test_bad_magic_raises(self):
        parser = FrameParser()
        with self.assertRaises(ProtocolError):
            parser.feed(b"XXXX" + self.frames[1][4:])

    def test_bad_version_raises(self):
        bad = bytearray(self.frames[1])
        bad[4] = 9
        with self.assertRaises(ProtocolError):
            FrameParser().feed(bytes(bad))

    def test_oversize_raises_before_payload_arrives(self):
        parser = FrameParser(max_payload_bytes=16)
        with self.assertRaises(ProtocolError):
            parser.feed(proto.encode_header(Op.PUBLISH, 17))

    def test_frames_before_error_are_yielded(self):
        parser = FrameParser()
        produced = []
        with self.assertRaises(ProtocolError):
            for frame in parser.iter_frames(self.frames[1] + b"BAD!" + b"\x00" * 8):
                produced.append(frame)
        self.assertEqual([f.op for f in produced], [Op.PING])

    def test_nonzero_flags_are_accepted(self):
        frames = FrameParser().feed(proto.encode_frame(Op.PING, b"", flags=0x8001))
        self.assertEqual(frames[0].flags, 0x8001)

    def test_announced_length_reserves_no_memory(self):
        """A 12-byte header announcing 64 MiB must not make the parser allocate 64 MiB."""
        header = proto.encode_header(Op.PUBLISH, proto.DEFAULT_MAX_PAYLOAD_BYTES)
        tracemalloc.start()
        try:
            parser = FrameParser()
            self.assertEqual(parser.feed(header), [])
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        self.assertLess(peak, 1024 * 1024, "peak allocation after the header: {}".format(peak))
        self.assertEqual(parser.buffered_bytes, proto.HEADER_SIZE)

    def test_large_frame_still_assembles_from_small_reads(self):
        payload = bytes(range(256)) * 4096  # 1 MiB
        frame = proto.encode_id_frame(Op.PUBLISH, 7, payload)
        parser = FrameParser()
        frames = []
        for start in range(0, len(frame), 65536):
            frames.extend(parser.feed(frame[start:start + 65536]))
        self.assertEqual(len(frames), 1)
        entity_id, cdr = proto.split_id_payload(frames[0].payload)
        self.assertEqual((entity_id, bytes(cdr)), (7, payload))

    def test_per_op_limits(self):
        limits = {
            Op.PUBLISH: proto.DEFAULT_MAX_PAYLOAD_BYTES,
            Op.HELLO: proto.CONTROL_PAYLOAD_LIMIT_BYTES,
            Op.ADVERTISE: proto.CONTROL_PAYLOAD_LIMIT_BYTES,
            Op.SUBSCRIBE: proto.CONTROL_PAYLOAD_LIMIT_BYTES,
            Op.UNADVERTISE: proto.CONTROL_PAYLOAD_LIMIT_BYTES,
            Op.UNSUBSCRIBE: proto.CONTROL_PAYLOAD_LIMIT_BYTES,
            Op.PING: proto.SMALL_PAYLOAD_LIMIT_BYTES,
            Op.PONG: proto.SMALL_PAYLOAD_LIMIT_BYTES,
            Op.MESSAGE: proto.SMALL_PAYLOAD_LIMIT_BYTES,
            99: proto.SMALL_PAYLOAD_LIMIT_BYTES,
        }
        for op, limit in limits.items():
            with self.subTest(op=op):
                # Exactly at the limit the header is accepted, one byte more is a protocol error.
                parser = FrameParser(per_op_limits=True)
                self.assertEqual(parser.feed(proto.encode_header(op, limit)), [])
                with self.assertRaises(ProtocolError):
                    FrameParser(per_op_limits=True).feed(proto.encode_header(op, limit + 1))
        # Without per_op_limits only max_payload_bytes applies (the client side of the tests).
        FrameParser().feed(proto.encode_header(Op.PING, proto.SMALL_PAYLOAD_LIMIT_BYTES + 1))

    def test_limit_can_be_raised_between_frames(self):
        """The bridge keeps the limit small until HELLO and raises it while parsing one read."""
        parser = FrameParser(proto.PRE_HELLO_PAYLOAD_LIMIT_BYTES, per_op_limits=True)
        big = proto.encode_id_frame(Op.PUBLISH, 1, b"x" * 100_000)
        stream = _frame(Op.HELLO, b'{"protocol":1}') + big
        seen = []
        for frame in parser.iter_frames(stream):
            seen.append(frame.op)
            if frame.op == Op.HELLO:
                parser.max_payload_bytes = proto.DEFAULT_MAX_PAYLOAD_BYTES
        self.assertEqual(seen, [Op.HELLO, Op.PUBLISH])
        # Before the HELLO the same frame is a protocol error.
        with self.assertRaises(ProtocolError):
            FrameParser(proto.PRE_HELLO_PAYLOAD_LIMIT_BYTES, per_op_limits=True).feed(big)


class JsonPayloadTest(unittest.TestCase):
    def test_hello_and_ack(self):
        self.assertEqual(
            proto.parse_hello({"protocol": 1, "client": "uBot UE 0.1.0"}), "uBot UE 0.1.0"
        )
        ack = proto.make_hello_ack("ubot_ros_bridge 0.1.0", "humble")
        self.assertEqual(
            ack, {"protocol": 1, "server": "ubot_ros_bridge 0.1.0", "ros_distro": "humble"}
        )
        for bad in ({"protocol": 2}, {}, {"protocol": True}, {"protocol": "1"}):
            with self.assertRaises(RequestError) as ctx:
                proto.parse_hello(bad)
            self.assertEqual(ctx.exception.code, ErrorCode.UNSUPPORTED_PROTOCOL)

    def test_error_and_sub_count_payloads(self):
        frame = proto.encode_json_frame(Op.ERROR, proto.make_error(None, "unknown_op", "x"))
        payload = json.loads(frame[12:].decode("utf-8"))
        self.assertEqual(payload, {"id": None, "code": "unknown_op", "message": "x"})
        frame = proto.encode_json_frame(Op.SUB_COUNT, proto.make_sub_count({"1": 2, "7": 0}))
        self.assertEqual(json.loads(frame[12:]), {"counts": {"1": 2, "7": 0}})

    def test_decode_json_object_errors(self):
        bad = (
            b"not json",
            b"[1,2]",
            b"\xff\xfe",
            b"",
            # Deeply nested payloads raise RecursionError in json.loads: still a bad request.
            b"[" * 5000,
            b"[" * 100000,
            b'{"a":' * 5000,
            b'{"id":' + b"[" * 5000,
        )
        for payload in bad:
            with self.subTest(payload=payload[:12], size=len(payload)):
                with self.assertRaises(RequestError) as ctx:
                    proto.decode_json_object(payload)
                self.assertEqual(ctx.exception.code, ErrorCode.BAD_REQUEST)
        self.assertEqual(proto.decode_json_object(bytearray(b'{"id":1}')), {"id": 1})

    def test_clip_text_bounds_and_escapes(self):
        self.assertEqual(proto.clip_text("plain text"), "plain text")
        self.assertEqual(proto.clip_text("a\nb\x1b[31mc"), "a\\nb\\x1b[31mc")
        self.assertEqual(proto.clip_text("x\u2028y\u202ez"), "x\\u2028y\\u202ez")
        self.assertEqual(proto.clip_text("\ud800"), "\\ud800")
        clipped = proto.clip_text("a" * 1000, 10)
        self.assertEqual(clipped, "a" * 10 + "...(+990 chars)")
        self.assertEqual(proto.clip_text(ValueError("boom")), "boom")
        self.assertEqual(proto.clip_text("x" * 5, 5), "xxxxx")

    def test_echoed_client_text_is_bounded(self):
        with self.assertRaises(ValueError) as ctx:
            proto.validate_topic("/" + "a" * 100000 + "!")
        self.assertLess(len(str(ctx.exception)), 200)
        with self.assertRaises(ValueError) as ctx:
            proto.validate_topic("relative\nline\x1b")
        self.assertNotIn("\n", str(ctx.exception))
        self.assertNotIn("\x1b", str(ctx.exception))
        with self.assertRaises(ValueError) as ctx:
            proto.normalize_type_name("a/" + "b" * 100000)
        self.assertLess(len(str(ctx.exception)), 200)
        with self.assertRaises(ValueError) as ctx:
            proto.normalize_type_name("not a type\r\nFAKE")
        self.assertNotIn("\r", str(ctx.exception))
        # Exactly at the limit is judged by the pattern, not the length.
        longest = "/" + "a" * (proto.MAX_TOPIC_LENGTH - 1)
        self.assertEqual(proto.validate_topic(longest), longest)
        with self.assertRaises(ValueError):
            proto.validate_topic("/" + "a" * proto.MAX_TOPIC_LENGTH)
        # A trailing newline is not a valid topic (the regex "$" alone would accept it).
        with self.assertRaises(ValueError):
            proto.validate_topic("/ok\n")

    def test_hello_client_name_is_sanitised(self):
        name = proto.parse_hello({"protocol": 1, "client": "evil\nFAKE LINE\x1b[31m" + "z" * 1000})
        self.assertNotIn("\n", name)
        self.assertNotIn("\x1b", name)
        self.assertLess(len(name), proto.MAX_CLIENT_NAME_CHARS + 40)
        self.assertEqual(proto.parse_hello({"protocol": 1, "client": 42}), "42")
        self.assertEqual(proto.parse_hello({"protocol": 1}), "")
        with self.assertRaises(RequestError) as ctx:
            proto.parse_hello({"protocol": "x" * 100000})
        self.assertLess(len(ctx.exception.message), 300)

    def test_parse_endpoint_request(self):
        request = proto.parse_endpoint_request(
            {
                "id": 5,
                "topic": "/robot_1/imu/data",
                "type": "sensor_msgs/msg/Imu",
                "qos": {"reliability": "best_effort", "durability": "volatile", "depth": 5},
            }
        )
        self.assertEqual(request.id, 5)
        self.assertEqual(request.topic, "/robot_1/imu/data")
        self.assertEqual(request.type_name, "sensor_msgs/msg/Imu")
        self.assertEqual(
            request.qos, QosSpec(proto.Reliability.BEST_EFFORT, proto.Durability.VOLATILE, 5)
        )
        self.assertTrue(request.qos.is_best_effort)
        self.assertEqual(str(request.qos), "best_effort/volatile/5")

    def test_type_name_normalization(self):
        self.assertEqual(proto.normalize_type_name("sensor_msgs/Imu"), "sensor_msgs/msg/Imu")
        self.assertEqual(
            proto.normalize_type_name("tf2_msgs/msg/TFMessage"), "tf2_msgs/msg/TFMessage"
        )
        for bad in ("Imu", "sensor_msgs/srv/Imu", "a/b/c/d", "", None, 3):
            with self.assertRaises(ValueError):
                proto.normalize_type_name(bad)

    def test_topic_validation(self):
        for good in ("/tf", "/tf_static", "/ubot_e2e/imu", "/a/_b/c1"):
            self.assertEqual(proto.validate_topic(good), good)
        for bad in ("tf", "/", "/a/", "//a", "/a//b", "/1a", "/a b", "/a-b", "", None, "~/x"):
            with self.assertRaises(ValueError):
                proto.validate_topic(bad)

    def test_request_errors_carry_codes_and_ids(self):
        base = {"id": 9, "topic": "/x", "type": "std_msgs/msg/String", "qos": {}}
        cases = [
            (dict(base, id=-1), ErrorCode.BAD_REQUEST, None),
            (dict(base, id=2**32), ErrorCode.BAD_REQUEST, None),
            (dict(base, id=True), ErrorCode.BAD_REQUEST, None),
            (dict(base, id=1.0), ErrorCode.BAD_REQUEST, None),
            ({k: v for k, v in base.items() if k != "id"}, ErrorCode.BAD_REQUEST, None),
            (dict(base, topic="relative"), ErrorCode.INVALID_TOPIC, 9),
            (dict(base, type="nope"), ErrorCode.UNKNOWN_TYPE, 9),
            (dict(base, qos={"reliability": "sometimes"}), ErrorCode.INVALID_QOS, 9),
            (dict(base, qos={"durability": "forever"}), ErrorCode.INVALID_QOS, 9),
            (dict(base, qos={"depth": 0}), ErrorCode.INVALID_QOS, 9),
            (dict(base, qos={"depth": "5"}), ErrorCode.INVALID_QOS, 9),
            (dict(base, qos=[1]), ErrorCode.INVALID_QOS, 9),
        ]
        for obj, code, request_id in cases:
            with self.subTest(obj=obj):
                with self.assertRaises(RequestError) as ctx:
                    proto.parse_endpoint_request(obj)
                self.assertEqual(ctx.exception.code, code)
                self.assertEqual(ctx.exception.request_id, request_id)
                self.assertEqual(ctx.exception.to_payload()["code"], code)

    def test_qos_defaults(self):
        self.assertEqual(proto.parse_qos(None), QosSpec())
        self.assertEqual(
            proto.parse_qos({}), QosSpec(proto.Reliability.RELIABLE, proto.Durability.VOLATILE, 10)
        )
        latched = proto.parse_qos(
            {"reliability": "reliable", "durability": "transient_local", "depth": 1}
        )
        self.assertEqual(
            latched.to_json(),
            {"reliability": "reliable", "durability": "transient_local", "depth": 1},
        )
        self.assertNotEqual(latched, QosSpec())

    def test_ids(self):
        self.assertTrue(proto.is_valid_id(0))
        self.assertTrue(proto.is_valid_id(proto.MAX_ID))
        self.assertFalse(proto.is_valid_id(proto.MAX_ID + 1))
        self.assertFalse(proto.is_valid_id(False))
        self.assertEqual(proto.parse_id({"id": 42}), 42)


class BoundedFrameQueueTest(unittest.TestCase):
    def test_fifo(self):
        queue = BoundedFrameQueue(100)
        self.assertIs(queue.put(b"a" * 10, FrameClass.CONTROL), PutResult.ACCEPTED)
        self.assertIs(queue.put(b"b" * 10, FrameClass.RELIABLE), PutResult.ACCEPTED)
        self.assertIs(queue.put(b"c" * 10, FrameClass.BEST_EFFORT), PutResult.ACCEPTED)
        self.assertEqual(queue.queued_bytes, 30)
        self.assertEqual(
            [queue.get(0), queue.get(0), queue.get(0)], [b"a" * 10, b"b" * 10, b"c" * 10]
        )
        self.assertIsNone(queue.get(0.01))
        self.assertEqual(queue.queued_bytes, 0)

    def test_best_effort_dropped_when_full(self):
        queue = BoundedFrameQueue(100)
        self.assertIs(queue.put(b"x" * 60, FrameClass.BEST_EFFORT), PutResult.ACCEPTED)
        self.assertIs(queue.put(b"y" * 50, FrameClass.BEST_EFFORT), PutResult.DROPPED)
        self.assertEqual(queue.dropped_best_effort, 1)
        self.assertEqual(len(queue), 1)

    def test_reliable_evicts_oldest_best_effort(self):
        queue = BoundedFrameQueue(100)
        queue.put(b"1" * 30, FrameClass.BEST_EFFORT)
        queue.put(b"r" * 20, FrameClass.RELIABLE)
        queue.put(b"2" * 30, FrameClass.BEST_EFFORT)
        self.assertIs(queue.put(b"R" * 40, FrameClass.RELIABLE), PutResult.ACCEPTED)
        self.assertEqual(queue.dropped_best_effort, 1)
        self.assertEqual(queue.queued_bytes, 90)
        self.assertEqual([queue.get(0) for _ in range(3)], [b"r" * 20, b"2" * 30, b"R" * 40])

    def test_reliable_rejected_without_evicting_when_it_cannot_fit(self):
        queue = BoundedFrameQueue(100)
        queue.put(b"r" * 70, FrameClass.RELIABLE)
        queue.put(b"b" * 20, FrameClass.BEST_EFFORT)
        self.assertIs(queue.put(b"R" * 40, FrameClass.RELIABLE), PutResult.REJECTED)
        self.assertEqual(queue.rejected_reliable, 1)
        self.assertEqual(queue.dropped_best_effort, 0)
        self.assertEqual(len(queue), 2)

    def test_control_frames_use_slack_then_overflow(self):
        queue = BoundedFrameQueue(100, control_slack_bytes=50)
        queue.put(b"r" * 100, FrameClass.RELIABLE)
        self.assertIs(queue.put(b"c" * 40, FrameClass.CONTROL), PutResult.ACCEPTED)
        self.assertIs(queue.put(b"c" * 20, FrameClass.CONTROL), PutResult.OVERFLOW)

    def test_close_drains_or_discards(self):
        queue = BoundedFrameQueue(100)
        queue.put(b"a", FrameClass.CONTROL)
        queue.close()
        self.assertIs(queue.put(b"b", FrameClass.CONTROL), PutResult.CLOSED)
        self.assertEqual(queue.get(), b"a")
        self.assertIsNone(queue.get())
        queue = BoundedFrameQueue(100)
        queue.put(b"a", FrameClass.CONTROL)
        queue.close(discard=True)
        self.assertIsNone(queue.get())
        self.assertTrue(queue.closed)

    def test_blocking_get_wakes_on_put_and_close(self):
        queue = BoundedFrameQueue(100)
        results = []
        thread = threading.Thread(target=lambda: results.extend([queue.get(), queue.get()]))
        thread.start()
        time.sleep(0.05)
        queue.put(b"x", FrameClass.CONTROL)
        time.sleep(0.05)
        queue.close()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results, [b"x", None])


class TokenBucketTest(unittest.TestCase):
    def test_burst_then_refill(self):
        bucket = TokenBucket(3, 2.0)
        self.assertEqual([bucket.try_take(0.0) for _ in range(5)], [True, True, True, False, False])
        self.assertFalse(bucket.try_take(0.4))  # 0.8 token
        self.assertTrue(bucket.try_take(0.5))  # 1 token
        self.assertFalse(bucket.try_take(0.5))
        self.assertEqual([bucket.try_take(100.0) for _ in range(4)], [True, True, True, False])

    def test_clock_going_backwards_is_harmless(self):
        bucket = TokenBucket(1, 1.0)
        self.assertTrue(bucket.try_take(10.0))
        self.assertFalse(bucket.try_take(5.0))
        self.assertTrue(bucket.try_take(11.0))

    def test_invalid_arguments(self):
        with self.assertRaises(ValueError):
            TokenBucket(0, 1.0)
        with self.assertRaises(ValueError):
            TokenBucket(1, -1.0)


class SubCountThrottleTest(unittest.TestCase):
    def test_on_change_and_rate_limited(self):
        throttle = SubCountThrottle(0.5)
        self.assertIsNone(throttle.update({}, 0.0))
        self.assertEqual(throttle.update({"1": 0}, 0.0), {"1": 0})
        self.assertIsNone(throttle.update({"1": 0}, 0.1))
        self.assertIsNone(throttle.update({"1": 1}, 0.3))
        self.assertEqual(throttle.update({"1": 1}, 0.5), {"1": 1})
        self.assertIsNone(throttle.update({"1": 1}, 5.0))
        self.assertEqual(throttle.update({}, 5.0), {})
        self.assertIsNone(throttle.update({}, 6.0))


if __name__ == "__main__":
    unittest.main()
