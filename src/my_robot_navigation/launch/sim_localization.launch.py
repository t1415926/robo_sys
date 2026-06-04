from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ground_truth_topic = LaunchConfiguration('ground_truth_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ground_truth_topic', default_value='/ground_truth/odom'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        Node(
            package='my_robot_navigation',
            executable='diff_drive_interface_bridge.py',
            name='diff_drive_interface_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='my_robot_navigation',
            executable='sim_localization_node.py',
            name='sim_localization_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ground_truth_topic': ground_truth_topic,
                'odom_topic': odom_topic,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'publish_rate': 30.0,
            }],
            output='screen',
        ),
    ])
