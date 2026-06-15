from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_goal = LaunchConfiguration('auto_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    map_file = PathJoinSubstitution([
        FindPackageShare('my_robot_navigation'),
        'maps',
        'simple_map.yaml',
    ])

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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.2'),
        DeclareLaunchArgument('goal_y', default_value='1.8'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            output='screen',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
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
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
        Node(
            package='my_robot_navigation',
            executable='astar_planner_node.py',
            name='astar_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_topic': '/map',
                'goal_topic': '/goal_pose',
                'plan_topic': '/plan',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'robot_radius': 0.22,
                'extra_inflation_radius': 0.05,
                'occupied_threshold': 65,
                'unknown_is_obstacle': True,
                'allow_diagonal': True,
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
                    'astar_planning.rviz',
                ]),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    condition=IfCondition(auto_goal),
                    package='my_robot_navigation',
                    executable='send_pose_goal_node.py',
                    name='send_pose_goal_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'goal_x': ParameterValue(goal_x, value_type=float),
                        'goal_y': ParameterValue(goal_y, value_type=float),
                        'goal_yaw': ParameterValue(goal_yaw, value_type=float),
                        'delay_sec': 1.0,
                    }],
                ),
            ],
        ),
    ])
