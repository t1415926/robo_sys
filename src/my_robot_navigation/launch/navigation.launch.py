import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    navigation_dir = get_package_share_directory('my_robot_navigation')
    default_map_file = os.path.join(navigation_dir, 'maps', 'simple_map.yaml')
    default_params_file = os.path.join(navigation_dir, 'config', 'nav2_params.yaml')

    use_sim_time = as_bool(LaunchConfiguration('use_sim_time').perform(context))
    map_file = LaunchConfiguration('map').perform(context) or default_map_file
    params_file = LaunchConfiguration('params_file').perform(context) or default_params_file
    use_rviz = LaunchConfiguration('use_rviz')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_x = LaunchConfiguration('goal_x').perform(context)
    goal_y = LaunchConfiguration('goal_y').perform(context)
    goal_yaw = LaunchConfiguration('goal_yaw').perform(context)

    if not os.path.isfile(map_file):
        raise RuntimeError(f'Map file does not exist: {map_file}')
    if not os.path.isfile(params_file):
        raise RuntimeError(f'Nav2 params file does not exist: {params_file}')

    sim_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_navigation'),
                'launch',
                'sim_localization.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim_time': str(use_sim_time).lower(),
            'ground_truth_topic': '/ground_truth/odom',
            'odom_topic': '/odom',
        }.items(),
    )

    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
    ]

    nav2_params = [params_file, {'use_sim_time': use_sim_time}]

    return [
        sim_localization,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=nav2_params,
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=nav2_params,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=nav2_params,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=nav2_params,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('my_robot_navigation'),
                    'rviz',
                    'nav2_sim.rviz',
                ]),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(auto_goal),
            package='my_robot_navigation',
            executable='send_goal_node.py',
            name='send_goal_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_x': float(goal_x),
                'goal_y': float(goal_y),
                'goal_yaw': float(goal_yaw),
                'delay_sec': 8.0,
            }],
        ),
    ]


def generate_launch_description():
    navigation_dir = get_package_share_directory('my_robot_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_dir, 'maps', 'simple_map.yaml'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_dir, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.2'),
        DeclareLaunchArgument('goal_y', default_value='1.8'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
