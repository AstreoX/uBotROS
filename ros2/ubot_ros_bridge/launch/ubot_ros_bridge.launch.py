"""Launch the uBot ROS bridge.

ros2 launch ubot_ros_bridge ubot_ros_bridge.launch.py port:=8268 bind_address:=0.0.0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "bind_address", default_value="0.0.0.0", description="TCP listen address"
        ),
        DeclareLaunchArgument("port", default_value="8268", description="TCP listen port"),
        DeclareLaunchArgument(
            "max_payload_bytes",
            default_value=str(64 * 1024 * 1024),
            description="largest accepted/sent frame payload in bytes",
        ),
        DeclareLaunchArgument(
            "max_queued_bytes",
            default_value=str(128 * 1024 * 1024),
            description="per-client send queue limit in bytes",
        ),
        DeclareLaunchArgument(
            "idle_timeout_seconds",
            default_value="10.0",
            description="drop a client that sends nothing for this long (0 disables)",
        ),
        DeclareLaunchArgument(
            "hello_timeout_seconds",
            default_value="5.0",
            description="drop a connection that has not sent HELLO after this long (0 disables)",
        ),
        DeclareLaunchArgument(
            "max_clients",
            default_value="32",
            description="concurrent clients accepted (0 = unlimited)",
        ),
    ]
    bridge = Node(
        package="ubot_ros_bridge",
        executable="bridge",
        name="ubot_ros_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "bind_address": ParameterValue(
                    LaunchConfiguration("bind_address"), value_type=str
                ),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "max_payload_bytes": ParameterValue(
                    LaunchConfiguration("max_payload_bytes"), value_type=int
                ),
                "max_queued_bytes": ParameterValue(
                    LaunchConfiguration("max_queued_bytes"), value_type=int
                ),
                "idle_timeout_seconds": ParameterValue(
                    LaunchConfiguration("idle_timeout_seconds"), value_type=float
                ),
                "hello_timeout_seconds": ParameterValue(
                    LaunchConfiguration("hello_timeout_seconds"), value_type=float
                ),
                "max_clients": ParameterValue(LaunchConfiguration("max_clients"), value_type=int),
            }
        ],
    )
    return LaunchDescription(arguments + [bridge])
