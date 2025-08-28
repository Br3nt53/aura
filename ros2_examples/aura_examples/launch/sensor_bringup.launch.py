from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='hardware/config/sensors.yaml'),
        Node(package='aura_examples', executable='sensor_manager_node', name='sensor_manager', output='log',
             parameters=[{'config': config}]),
    ])
