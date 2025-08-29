from setuptools import find_packages, setup
import os
from glob import glob

package_name = "aura_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sharkbait",
    maintainer_email="sharkbait@todo.todo",
    description="AURA examples and integration code",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fusion_tracker_node = aura_examples.nodes.fusion_tracker_node:main",
            "fusion_dynamic_node = aura_examples.nodes.fusion_dynamic_node:main",
            "sensor_manager_node = aura_examples.nodes.sensor_manager_node:main",
            "recorder_node = aura_examples.nodes.recorder_node:main",
            "scenario_player_node = aura_examples.nodes.scenario_player_node:main",
            "teleop_target_node = aura_examples.nodes.teleop_target_node:main",
        ],
    },
)
