import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the share directory of the aura_examples package
    aura_examples_share_dir = get_package_share_directory("aura_examples")

    # Declare the scenario argument with a default value relative to the package
    default_scenario_path = os.path.join(
        aura_examples_share_dir, "..", "..", "scenarios", "crossing_targets.yaml"
    )

    return LaunchDescription(
        [
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
            # ... add other nodes from your pipeline here
        ]
    )
