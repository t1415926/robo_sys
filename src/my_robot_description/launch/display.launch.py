from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    robot_description = {
        'robot_description': Command(['xacro ', model, ' use_ros2_control:=false'])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'my_robot.urdf.xacro',
            ]),
            description='Absolute path to robot xacro file.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'rviz',
                    'display.rviz',
                ]),
            ],
            output='screen',
        ),
    ])
