import os
from glob import glob

from setuptools import setup

package_name = "ubot_ros_bridge"

setup(
    name=package_name,
    version="0.1.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AstreoX",
    maintainer_email="astreox@users.noreply.github.com",
    description="TCP bridge between the uBot UE plugin (UBotROS) and ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = ubot_ros_bridge.bridge_node:main",
        ],
    },
)
