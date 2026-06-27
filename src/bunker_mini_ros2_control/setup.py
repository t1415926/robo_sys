from setuptools import find_packages, setup

package_name = 'bunker_mini_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bunker_mini_cmd.launch.py',
            'launch/can_driver.launch.py',
            'launch/cmd_vel_bridge.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Safe ROS2 Twist command publisher for BUNKER MINI chassis control.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bunker_mini_cmd = bunker_mini_ros2_control.bunker_mini_cmd:main',
            'can_driver = bunker_mini_ros2_control.can_driver:main',
            'cmd_vel_bridge = bunker_mini_ros2_control.cmd_vel_bridge:main',
        ],
    },
)
