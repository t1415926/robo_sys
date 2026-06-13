from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration('bag_path')
    play_bag = LaunchConfiguration('play_bag')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    pointlio_params = [
        PathJoinSubstitution([
            FindPackageShare('my_robot_bringup'),
            'config',
            'pointlio_mid360_nav.yaml',
        ]),
        {
            'use_sim_time': use_sim_time,
        },
    ]

    pointlio_mapping = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=pointlio_params,
        remappings=[
            ('/livox/lidar', '/livox/mid360/lidar'),
            ('/livox/imu', '/livox/mid360/imu'),
            ('/aft_mapped_to_init', '/odom'),
        ],
    )

    pointcloud_grid = Node(
        package='my_robot_navigation',
        executable='pointcloud_to_occupancy_grid.py',
        name='pointcloud_to_occupancy_grid',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cloud_topic': '/Laser_map',
            'map_topic': '/map',
            'frame_id': 'map',
            'resolution': 0.10,
            'width_m': 40.0,
            'height_m': 40.0,
            'origin_x': -20.0,
            'origin_y': -20.0,
            'min_z': 0.05,
            'max_z': 1.50,
            'inflate_radius_m': 0.25,
        }],
    )

    base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_static_tf',
        output='screen',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
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
            'use_map_server': 'false',
            'use_rviz': use_rviz,
            'auto_goal': auto_goal,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
            'params_file': PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'config',
                'nav2_params.yaml',
            ]),
        }.items(),
    )

    bag_play = ExecuteProcess(
        condition=IfCondition(play_bag),
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='/home/dtc/point_lio_ws/test_bag'),
        DeclareLaunchArgument('play_bag', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        base_to_link,
        pointlio_mapping,
        pointcloud_grid,
        TimerAction(period=4.0, actions=[navigation]),
        TimerAction(period=6.0, actions=[bag_play]),
    ])
