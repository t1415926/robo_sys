from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    repeat_demo = LaunchConfiguration('repeat_demo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/smoother_cmd_vel',
            description='Twist topic subscribed by the BUNKER MINI base driver.',
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.05',
            description='Low forward speed in m/s for first real-vehicle test.',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.20',
            description='Low yaw rate in rad/s for first real-vehicle test.',
        ),
        DeclareLaunchArgument(
            'repeat_demo',
            default_value='false',
            description='Repeat forward-stop-turn-stop demo until interrupted.',
        ),
        Node(
            package='bunker_mini_ros2_control',
            executable='bunker_mini_cmd',
            name='bunker_mini_cmd',
            output='screen',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'linear_speed': linear_speed,
                'angular_speed': angular_speed,
                'repeat_demo': repeat_demo,
            }],
        ),
    ])
