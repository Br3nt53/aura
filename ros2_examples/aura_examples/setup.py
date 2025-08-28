from setuptools import setup

package_name = 'aura_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f"{package_name}.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AURA Dev',
    maintainer_email='devnull@example.com',
    description='ROS 2 example nodes and launch for AURA pipeline.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scenario_player = aura_examples.nodes.scenario_player_node:main',
            'mock_rf_node = aura_examples.nodes.mock_rf_node:main',
            'fusion_sim_node = aura_examples.nodes.fusion_sim_node:main',
            'recorder_node = aura_examples.nodes.recorder_node:main',
            'teleop_target_node = aura_examples.nodes.teleop_target_node:main',
            'fusion_dynamic_node = aura_examples.nodes.fusion_dynamic_node:main',
            'sensor_manager_node = aura_examples.nodes.sensor_manager_node:main',
        ],
    },
)
