from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'launch',
                'navigation.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        gazebo,
        TimerAction(period=8.0, actions=[navigation]),
    ])
