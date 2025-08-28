from setuptools import setup

package_name = 'aura_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AURA Dev',
    maintainer_email='devnull@example.com',
    description='Utility nodes for AURA experiments (param bridge for live tuning).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'param_bridge = aura_tools.param_bridge:main',
        ],
    },
)
