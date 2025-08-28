from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    workdir = LaunchConfiguration('workdir')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='scenarios/crossing_targets.yaml'),
        DeclareLaunchArgument('workdir', default_value='out/tmp'),
        Node(package='aura_examples', executable='scenario_player', name='scenario_player', output='log',
             parameters=[{'scenario': scenario}]),
        Node(package='aura_examples', executable='mock_rf_node', name='mock_rf_node', output='log'),
        Node(package='aura_examples', executable='fusion_tracker_node', name='fusion_sim_node', output='log'),
        Node(package='aura_examples', executable='recorder_node', name='recorder_node', output='log',
             parameters=[{'workdir': workdir}]),
    ])
