from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'simple_omni_robot.urdf.xacro',
            ]),
        ]),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    omni_base = Node(
        package='my_robot_navigation',
        executable='simple_omni_base_node.py',
        name='simple_omni_base_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'publish_rate': 50.0,
            'cmd_timeout': 0.5,
        }],
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
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.2'),
        DeclareLaunchArgument('goal_y', default_value='1.8'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        robot_state_publisher,
        map_to_odom,
        omni_base,
        TimerAction(period=0.5, actions=[navigation]),
    ])
