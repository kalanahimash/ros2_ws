"""
rover.launch.py — Main bringup launch file.

Starts all rover nodes with shared parameter configuration.

Usage:
    ros2 launch rover_bringup rover.launch.py

Optional overrides:
    ros2 launch rover_bringup rover.launch.py serial_port:=/dev/ttyACM0
    ros2 launch rover_bringup rover.launch.py dashboard_port:=8080
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory("rover_bringup")
    params_file = os.path.join(pkg_bringup, "config", "rover_params.yaml")

    # ── Launch arguments ────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Arduino USB serial port",
    )
    dashboard_port_arg = DeclareLaunchArgument(
        "dashboard_port",
        default_value="5000",
        description="Flask dashboard port",
    )

    serial_port    = LaunchConfiguration("serial_port")
    dashboard_port = LaunchConfiguration("dashboard_port")

    # ── Nodes ───────────────────────────────────────────────────────────────
    serial_node = Node(
        package="rover_serial",
        executable="serial_node",
        name="rover_serial",
        output="screen",
        parameters=[
            params_file,
            {"serial_port": serial_port},
        ],
    )

    camera_node = Node(
        package="rover_camera",
        executable="camera_node",
        name="rover_camera",
        output="screen",
        parameters=[params_file],
    )

    tracking_node = Node(
        package="rover_tracking",
        executable="tracking_node",
        name="rover_tracking",
        output="screen",
        parameters=[params_file],
    )

    navigation_node = Node(
        package="rover_navigation",
        executable="navigation_node",
        name="rover_navigation",
        output="screen",
        parameters=[params_file],
    )

    control_node = Node(
        package="rover_control",
        executable="control_node",
        name="rover_control",
        output="screen",
        parameters=[params_file],
    )

    dashboard_node = Node(
        package="rover_dashboard",
        executable="dashboard_node",
        name="rover_dashboard",
        output="screen",
        parameters=[
            params_file,
            {"port": dashboard_port},
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        dashboard_port_arg,
        LogInfo(msg="═══ Starting Rover System ═══"),
        serial_node,
        camera_node,
        tracking_node,
        navigation_node,
        control_node,
        dashboard_node,
        LogInfo(msg="═══ All nodes launched ═══"),
    ])
