import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the share directory of the aura_examples package
    pkg_share_dir = get_package_share_directory("aura_examples")
    repo_root = os.path.abspath(os.path.join(pkg_share_dir, "../../../.."))

    # Declare the scenario argument with a default value relative to the package
    default_scenario_path = os.path.join(
        repo_root, "scenarios", "crossing_targets.yaml"
    )
    keystore_path = os.path.join(repo_root, "security")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "secure",
                default_value="false",
                description="Enable ROS 2 security (SROS).",
            ),
            # Set environment variables for SROS if secure:=true
            SetEnvironmentVariable(
                name="ROS_SECURITY_ENABLE",
                value="true",
                condition=IfCondition(LaunchConfiguration("secure")),
            ),
            SetEnvironmentVariable(
                name="ROS_SECURITY_STRATEGY",
                value="Enforce",
                condition=IfCondition(LaunchConfiguration("secure")),
            ),
            SetEnvironmentVariable(
                name="ROS_SECURITY_ROOT_KEYSTORE",
                value=keystore_path,
                condition=IfCondition(LaunchConfiguration("secure")),
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value=default_scenario_path,
                description="Full path to the scenario YAML file",
            ),
            Node(
                package="aura_examples",
                executable="scenario_player_node",
                name="scenario_player",
                output="screen",
                parameters=[{"scenario": LaunchConfiguration("scenario")}],
            ),
            Node(
                package="aura_examples",
                executable="fusion_tracker_node",
                name="fusion_tracker",
                output="screen",
            ),
        ]
    )
