#!/usr/bin/env python3
"""Generate golden ROS 2 CDR bytes for the UBotROS UE automation tests (UBotROS.Cdr.Golden.*).

Every case below is serialized with rclpy.serialization.serialize_message, i.e. by the ROS 2
middleware itself, and written as uint8 arrays into a C++ header. The UE tests build the very same
messages with the FUBotRos* structs and require byte-for-byte equality.

Run it in a ROS 2 Humble environment (no colcon build needed):

    source /opt/ros/humble/setup.bash
    python3 gen_golden.py --output ../../Source/UBotROS/Private/Tests/UBotROSGoldenData.h

Without --output the header is printed to stdout.

Keep the values here identical to the ones in Source/UBotROS/Private/Tests/UBotROSCdrTests.cpp.
Floating point values are chosen to be exactly representable (or are plain decimal literals for
float64 fields) so that Python and C++ produce identical bit patterns.

rmw_fastrtps does not initialize CDR alignment padding, so the reference bytes contain arbitrary
values there. For every case the generator therefore also lays the message out itself (a small
reference XCDR1 encoder driven by the rosidl field types), checks that every non-padding byte and
the length agree with rclpy, checks that rclpy deserializes the zero-padded layout back to the same
message, and emits a layout string that tells the UE test which bytes are padding.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
from typing import Any, Callable, Dict, List, Sequence, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

GOLDEN_NAMESPACE = 'UBotROSGoldenData'
BYTES_PER_LINE = 16
ENCAPSULATION_HEADER = b'\x00\x01\x00\x00'

# Layout characters, one per byte of a reference message:
# 'H' encapsulation header; '.' significant byte, must match exactly; 'p' CDR alignment padding,
# arbitrary in the reference and zero from FUBotCdrWriter; 't' rmw_fastrtps size padding of
# fixed-size types (not part of the CDR stream).
LAYOUT_HEADER = 'H'
LAYOUT_DATA = '.'
LAYOUT_PADDING = 'p'
LAYOUT_TRAILING = 't'

# rosidl primitive type name -> (struct format, size). Alignment equals size (XCDR1).
PRIMITIVES: Dict[str, Tuple[str, int]] = {
    'boolean': ('<B', 1),
    'octet': ('<B', 1),
    'byte': ('<B', 1),
    'char': ('<B', 1),
    'uint8': ('<B', 1),
    'int8': ('<b', 1),
    'uint16': ('<H', 2),
    'int16': ('<h', 2),
    'uint32': ('<I', 4),
    'int32': ('<i', 4),
    'uint64': ('<Q', 8),
    'int64': ('<q', 8),
    'float': ('<f', 4),
    'double': ('<d', 8),
}


class CdrLayout:
    """Minimal XCDR1 little-endian encoder that records which bytes are alignment padding."""

    def __init__(self) -> None:
        self.data = bytearray(ENCAPSULATION_HEADER)
        self.layout: List[str] = [LAYOUT_HEADER] * len(ENCAPSULATION_HEADER)

    def _append(self, payload: bytes, kind: str) -> None:
        self.data += payload
        self.layout.extend(kind * len(payload))

    def align(self, alignment: int) -> None:
        body_offset = len(self.data) - len(ENCAPSULATION_HEADER)
        padding = (-body_offset) % alignment
        self._append(b'\x00' * padding, LAYOUT_PADDING)

    def primitive(self, type_name: str, value: Any) -> None:
        fmt, size = PRIMITIVES[type_name]
        self.align(size)
        if type_name == 'boolean':
            value = 1 if value else 0
        elif type_name in ('byte', 'char') and isinstance(value, (bytes, str)):
            value = ord(value)
        self._append(struct.pack(fmt, value), LAYOUT_DATA)

    def string(self, value: str) -> None:
        encoded = value.encode('utf-8') + b'\x00'
        self.primitive('uint32', len(encoded))
        self._append(encoded, LAYOUT_DATA)

    def single(self, type_name: str, value: Any) -> None:
        if type_name in PRIMITIVES:
            self.primitive(type_name, value)
        elif type_name == 'string':
            self.string(value)
        else:
            self.message(value)

    def field(self, type_name: str, value: Any) -> None:
        if type_name.startswith('sequence<'):
            element_type = type_name[len('sequence<'):-1]
            if ',' in element_type:
                raise ValueError(f'bounded sequences are not supported: {type_name}')
            self.primitive('uint32', len(value))
            for element in value:
                self.single(element_type, element)
        elif type_name.endswith(']'):
            element_type, count = type_name[:-1].split('[')
            if len(value) != int(count):
                raise ValueError(f'{type_name} holds {len(value)} elements')
            for element in value:
                self.single(element_type, element)
        else:
            self.single(type_name, value)

    def message(self, message: Any) -> None:
        for name, type_name in message.get_fields_and_field_types().items():
            self.field(type_name, getattr(message, name))


def build_reference(name: str, message: Any) -> Tuple[bytes, str]:
    """Serialize with rclpy and return (reference bytes, layout string) after cross-checking."""
    reference = bytes(serialize_message(message))

    expected = CdrLayout()
    expected.message(message)
    cdr_size = len(expected.data)

    # rmw_fastrtps reports the size of fixed-size types rounded up to a multiple of 4. (It also
    # over-estimates the size of messages with empty float64 sequences; avoid those here.)
    trailing = len(reference) - cdr_size
    if trailing < 0 or trailing > 3 or (trailing > 0 and len(reference) % 4 != 0):
        raise RuntimeError(f'{name}: rclpy produced {len(reference)} bytes, layout {cdr_size}')
    for index in range(cdr_size):
        if expected.layout[index] != LAYOUT_PADDING and reference[index] != expected.data[index]:
            raise RuntimeError(f'{name}: byte {index} differs (rclpy 0x{reference[index]:02X}, '
                               f'layout 0x{expected.data[index]:02X})')

    if deserialize_message(bytes(expected.data), type(message)) != message:
        raise RuntimeError(f'{name}: rclpy does not read the zero-padded layout back')

    layout = ''.join(expected.layout) + LAYOUT_TRAILING * trailing
    return reference, layout


def make_time(sec: int, nanosec: int) -> Time:
    stamp = Time()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def make_header(sec: int, nanosec: int, frame_id: str) -> Header:
    header = Header()
    header.stamp = make_time(sec, nanosec)
    header.frame_id = frame_id
    return header


def make_vector3(x: float, y: float, z: float) -> Vector3:
    vector = Vector3()
    vector.x = x
    vector.y = y
    vector.z = z
    return vector


def make_quaternion(x: float, y: float, z: float, w: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.x = x
    quaternion.y = y
    quaternion.z = z
    quaternion.w = w
    return quaternion


def make_transform_stamped(
    header: Header,
    child_frame_id: str,
    translation: Tuple[float, float, float],
    rotation: Tuple[float, float, float, float],
) -> TransformStamped:
    transform = TransformStamped()
    transform.header = header
    transform.child_frame_id = child_frame_id
    transform.transform.translation = make_vector3(*translation)
    transform.transform.rotation = make_quaternion(*rotation)
    return transform


def case_time() -> Any:
    return make_time(1700000123, 456789012)


def case_header() -> Any:
    return make_header(12, 500000000, 'map')


def case_header_empty_frame() -> Any:
    return make_header(0, 0, '')


def case_pose_stamped() -> Any:
    message = PoseStamped()
    message.header = make_header(2, 3, 'ab')
    message.pose = Pose()
    message.pose.position.x = -1.0
    message.pose.position.y = 2.0
    message.pose.position.z = -3.0
    message.pose.orientation = make_quaternion(0.5, 0.5, -0.5, 0.5)
    return message


def case_transform_stamped() -> Any:
    return make_transform_stamped(
        make_header(4, 5, 'odom'),
        'base_footprint',
        (3.5, -4.25, 0.0),
        (0.0, 0.0, -0.5, 0.8660254037844386),
    )


def case_imu() -> Any:
    message = Imu()
    message.header = make_header(100, 250000000, 'imu_frame')
    message.orientation = make_quaternion(0.125, -0.25, 0.375, 0.875)
    message.orientation_covariance = [1.0 + index * 0.125 for index in range(9)]
    message.angular_velocity = make_vector3(0.1, -0.2, 0.3)
    message.angular_velocity_covariance = [2.0 + index * 0.125 for index in range(9)]
    message.linear_acceleration = make_vector3(-1.5, 2.25, 9.81)
    message.linear_acceleration_covariance = [3.0 + index * 0.125 for index in range(9)]
    return message


def case_odometry() -> Any:
    message = Odometry()
    message.header = make_header(7, 123456789, 'world')
    message.child_frame_id = 'base_link'
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -2.5
    message.pose.pose.position.z = 3.75
    message.pose.pose.orientation = make_quaternion(
        0.0, 0.0, 0.38268343236508984, 0.9238795325112867)
    message.pose.covariance = [index * 0.5 for index in range(36)]
    message.twist.twist.linear = make_vector3(0.5, -0.125, 0.0625)
    message.twist.twist.angular = make_vector3(-0.03125, 0.015625, 0.75)
    message.twist.covariance = [-(index * 0.25) for index in range(36)]
    return message


def case_laser_scan() -> Any:
    message = LaserScan()
    message.header = make_header(3, 999999999, 'laser')
    message.angle_min = -1.5
    message.angle_max = 1.5
    message.angle_increment = 0.75
    message.time_increment = 0.0009765625
    message.scan_time = 0.0625
    message.range_min = 0.125
    message.range_max = 32.0
    message.ranges = [1.0, 2.5, float('inf'), 0.5, 12.25]
    message.intensities = [100.0, 0.0, 50.5, 7.75, 1.0]
    return message


def make_point_field(name: str, offset: int, datatype: int, count: int) -> PointField:
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = datatype
    field.count = count
    return field


def case_point_field() -> Any:
    return make_point_field('intensity', 12, PointField.FLOAT32, 1)


def case_point_cloud2() -> Any:
    message = PointCloud2()
    message.header = make_header(5, 5, 'lidar_top')
    message.height = 1
    message.width = 3
    message.fields = [
        make_point_field('x', 0, PointField.FLOAT32, 1),
        make_point_field('ring', 4, PointField.UINT16, 1),
    ]
    message.is_bigendian = False
    message.point_step = 6
    message.row_step = 18
    points = [(1.5, 0), (-2.25, 1), (3.0, 2)]
    message.data = b''.join(struct.pack('<fH', x, ring) for x, ring in points)
    message.is_dense = True
    return message


def case_image() -> Any:
    message = Image()
    message.header = make_header(9, 87654321, 'cam_optical')
    message.height = 2
    message.width = 3
    message.encoding = 'rgb8'
    message.is_bigendian = 0
    message.step = 9
    message.data = bytes((index * 37 + 11) & 0xFF for index in range(18))
    return message


def case_region_of_interest() -> Any:
    message = RegionOfInterest()
    message.x_offset = 1
    message.y_offset = 2
    message.height = 3
    message.width = 4
    message.do_rectify = True
    return message


def case_camera_info() -> Any:
    message = CameraInfo()
    message.header = make_header(9, 87654321, 'cam_optical')
    message.height = 480
    message.width = 640
    message.distortion_model = 'plumb_bob'
    message.d = [0.125, -0.25, 0.0009765625, -0.001953125, 0.5]
    message.k = [500.0, 0.0, 320.5, 0.0, 501.0, 240.5, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [500.0, 0.0, 320.5, -35.0, 0.0, 501.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    message.binning_x = 1
    message.binning_y = 2
    message.roi.x_offset = 10
    message.roi.y_offset = 20
    message.roi.height = 30
    message.roi.width = 40
    message.roi.do_rectify = True
    return message


def case_tf_message() -> Any:
    message = TFMessage()
    message.transforms = [
        make_transform_stamped(
            make_header(11, 1, 'world'),
            'base_link',
            (1.0, -2.0, 0.5),
            (0.0, 0.0, 0.0, 1.0),
        ),
        make_transform_stamped(
            make_header(11, 1, 'base_link'),
            'laser',
            (0.25, 0.0, 0.125),
            (0.0, 0.0, 0.7071067811865476, 0.7071067811865476),
        ),
    ]
    return message


def case_clock() -> Any:
    message = Clock()
    message.clock = make_time(98765, 4321)
    return message


def case_twist() -> Any:
    message = Twist()
    message.linear = make_vector3(1.5, -0.25, 0.125)
    message.angular = make_vector3(-0.5, 0.75, 0.3)
    return message


# (C++ array name, description, factory). The order is the order in the generated header.
CASES: Sequence[Tuple[str, str, Callable[[], Any]]] = (
    ('Time',
     'builtin_interfaces/msg/Time (1700000123 s, 456789012 ns)',
     case_time),
    ('Header',
     "std_msgs/msg/Header (12.5 s, frame 'map')",
     case_header),
    ('HeaderEmptyFrame',
     "std_msgs/msg/Header (0 s, frame '')",
     case_header_empty_frame),
    ('PoseStamped',
     "geometry_msgs/msg/PoseStamped (frame 'ab')",
     case_pose_stamped),
    ('TransformStamped',
     "geometry_msgs/msg/TransformStamped ('odom' -> 'base_footprint')",
     case_transform_stamped),
    ('Imu',
     "sensor_msgs/msg/Imu (frame 'imu_frame')",
     case_imu),
    ('Odometry',
     "nav_msgs/msg/Odometry ('world' -> 'base_link')",
     case_odometry),
    ('LaserScan',
     "sensor_msgs/msg/LaserScan (5 ranges incl. +inf, frame 'laser')",
     case_laser_scan),
    ('PointField',
     "sensor_msgs/msg/PointField ('intensity', FLOAT32 @12)",
     case_point_field),
    ('PointCloud2',
     'sensor_msgs/msg/PointCloud2 (fields x:FLOAT32@0, ring:UINT16@4; 3 points)',
     case_point_cloud2),
    ('Image',
     "sensor_msgs/msg/Image (3x2 rgb8, frame 'cam_optical')",
     case_image),
    ('RegionOfInterest',
     'sensor_msgs/msg/RegionOfInterest (1, 2, 3, 4, true)',
     case_region_of_interest),
    ('CameraInfo',
     "sensor_msgs/msg/CameraInfo (640x480 plumb_bob, frame 'cam_optical')",
     case_camera_info),
    ('TfMessage',
     'tf2_msgs/msg/TFMessage (2 transforms)',
     case_tf_message),
    ('Clock',
     'rosgraph_msgs/msg/Clock (98765 s, 4321 ns)',
     case_clock),
    ('Twist',
     'geometry_msgs/msg/Twist',
     case_twist),
)


def split_chunks(values: Sequence[Any], size: int) -> List[Sequence[Any]]:
    return [values[start:start + size] for start in range(0, len(values), size)]


def format_case(name: str, description: str, data: bytes, layout: str) -> List[str]:
    padding = layout.count(LAYOUT_PADDING)
    trailing = layout.count(LAYOUT_TRAILING)
    lines = [
        f'    // {description}: {len(data)} bytes '
        f'({padding} alignment padding, {trailing} trailing)',
        f'    inline constexpr uint8 {name}[] =',
        '    {',
    ]
    for chunk in split_chunks(data, BYTES_PER_LINE):
        lines.append('        ' + ', '.join(f'0x{value:02X}' for value in chunk) + ',')
    lines.append('    };')
    lines.append(f'    inline constexpr ANSICHAR {name}Layout[] =')
    chunks = split_chunks(layout, BYTES_PER_LINE)
    for index, chunk in enumerate(chunks):
        terminator = ';' if index == len(chunks) - 1 else ''
        lines.append(f'        "{chunk}"{terminator}')
    return lines


def generate_header() -> str:
    rmw = os.environ.get('RMW_IMPLEMENTATION') or 'rmw_fastrtps_cpp (default)'
    distro = os.environ.get('ROS_DISTRO', 'unknown')
    lines = [
        '#pragma once',
        '',
        '#include "CoreMinimal.h"',
        '',
        '// GENERATED FILE - do not edit by hand.',
        '// Generator: ros2/tools/gen_golden.py (rclpy.serialization.serialize_message).',
        f'// ROS distro: {distro}; RMW: {rmw}.',
        '// The messages are built from the same values in UBotROSCdrTests.cpp.',
        '//',
        '// <Name>[] holds the bytes produced by rclpy. <Name>Layout has one character per byte:',
        f"//   '{LAYOUT_HEADER}' encapsulation header, '{LAYOUT_DATA}' significant byte,",
        f"//   '{LAYOUT_PADDING}' CDR alignment padding (rmw_fastrtps leaves it uninitialized, so",
        '//       these bytes change between generator runs; FUBotCdrWriter writes 0),',
        f"//   '{LAYOUT_TRAILING}' rmw_fastrtps size padding of fixed-size types to a multiple of",
        '//       4 bytes (not part of the CDR stream).',
        '',
        f'namespace {GOLDEN_NAMESPACE}',
        '{',
    ]
    for index, (name, description, factory) in enumerate(CASES):
        if index > 0:
            lines.append('')
        data, layout = build_reference(name, factory())
        lines.extend(format_case(name, description, data, layout))
    lines.append('}')
    lines.append('')
    return '\n'.join(lines)


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description='Generate UBotROS golden CDR bytes.')
    parser.add_argument('--output', help='Header file to write (default: stdout).')
    arguments = parser.parse_args(argv)

    content = generate_header()
    if arguments.output:
        with open(arguments.output, 'w', encoding='utf-8', newline='\n') as output_file:
            output_file.write(content)
    else:
        sys.stdout.write(content)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
