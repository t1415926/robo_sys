from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    gazebo_omni = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'launch',
                'gazebo_omni.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'initial_x': '0.0',
            'initial_y': '0.0',
            'initial_z': '0.08',
            'initial_yaw': '0.0',
        }.items(),
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
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
            'auto_goal': auto_goal,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
            'map': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'maps',
                'simple_map.yaml',
            ]),
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.2'),
        DeclareLaunchArgument('goal_y', default_value='1.8'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        gazebo_omni,
        map_to_odom,
        TimerAction(period=5.0, actions=[navigation]),
    ])
