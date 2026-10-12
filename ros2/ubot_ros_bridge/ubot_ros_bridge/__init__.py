"""ROS 2 side of the uBot UE bridge.

UE (the UBotROS plugin) encodes ROS 2 messages as CDR itself and streams them over
TCP to this package, which publishes and subscribes them unchanged with rclpy.
See ``docs/PROTOCOL.md`` in the UBotROS plugin for the wire protocol.
"""

__version__ = "0.1.1"

__all__ = ["__version__"]
