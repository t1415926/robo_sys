from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    entity_name = LaunchConfiguration('entity_name')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_yaw = LaunchConfiguration('initial_yaw')

    robot_description = {
        'robot_description': Command(['xacro ', model])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ]),
        launch_arguments={'world': world}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic',
            'robot_description',
            '-entity',
            entity_name,
            '-x',
            initial_x,
            '-y',
            initial_y,
            '-z',
            initial_z,
            '-Y',
            initial_yaw,
        ],
        output='screen',
    )

    gazebo_omni_base = Node(
        package='my_robot_gazebo',
        executable='gazebo_omni_base_node.py',
        name='gazebo_omni_base_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': entity_name,
            'reference_frame': 'world',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'publish_rate': 50.0,
            'cmd_timeout': 0.5,
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_z': initial_z,
            'initial_yaw': initial_yaw,
        }],
    )

    delayed_omni_base = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[gazebo_omni_base],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'worlds',
                'simple_obstacles.world',
            ]),
            description='Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'urdf',
                'simple_omni_robot.urdf.xacro',
            ]),
            description='Simple omni robot xacro file.',
        ),
        DeclareLaunchArgument('entity_name', default_value='simple_omni_robot'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.08'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        delayed_omni_base,
    ])
