"""serial_only.launch.py — Launch only the serial and control nodes for hardware testing."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("rover_bringup")
    params = os.path.join(pkg, "config", "rover_params.yaml")

    return LaunchDescription([
        Node(package="rover_serial",  executable="serial_node",  name="rover_serial",  output="screen", parameters=[params]),
        Node(package="rover_control", executable="control_node", name="rover_control", output="screen", parameters=[params]),
    ])
