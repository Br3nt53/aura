#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dev", default_value="/dev/aura_thermal0"),
            # Add nodes/actions that use LaunchConfiguration("dev") here, e.g.:
            # Node(package="...", executable="...", parameters=[{"device": LaunchConfiguration("dev")}]),
        ]
    )
