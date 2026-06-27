from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface = LaunchConfiguration('can_interface')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')
    cmd_timeout = LaunchConfiguration('cmd_timeout')

    return LaunchDescription([
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('max_linear_x', default_value='0.10'),
        DeclareLaunchArgument('max_angular_z', default_value='0.20'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        Node(
            package='bunker_mini_ros2_control',
            executable='can_driver',
            name='bunker_mini_can_driver',
            output='screen',
            parameters=[{
                'can_interface': can_interface,
                'cmd_vel_topic': cmd_vel_topic,
                'max_linear_x': max_linear_x,
                'max_angular_z': max_angular_z,
                'cmd_timeout': cmd_timeout,
            }],
        ),
    ])
