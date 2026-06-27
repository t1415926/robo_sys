from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_angular_z = LaunchConfiguration('max_angular_z')
    cmd_timeout = LaunchConfiguration('cmd_timeout')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/cmd_vel',
            description='Twist commands from Nav2 or another planner.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/smoother_cmd_vel',
            description='Twist topic consumed by the BUNKER MINI base driver.',
        ),
        DeclareLaunchArgument('max_linear_x', default_value='0.30'),
        DeclareLaunchArgument('max_linear_y', default_value='0.0'),
        DeclareLaunchArgument('max_angular_z', default_value='0.60'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        Node(
            package='bunker_mini_ros2_control',
            executable='cmd_vel_bridge',
            name='bunker_mini_cmd_vel_bridge',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'max_linear_x': max_linear_x,
                'max_linear_y': max_linear_y,
                'max_angular_z': max_angular_z,
                'cmd_timeout': cmd_timeout,
            }],
        ),
    ])
