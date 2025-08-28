from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dev = LaunchConfiguration('dev')
    return LaunchDescription([
        DeclareLaunchArgument('dev', default_value='/dev/aura_thermal0'),
        # Replace with your real thermal bridge node
        Node(package='aura_examples', executable='mock_rf_node', name='thermal_bridge', output='log')
    ])
