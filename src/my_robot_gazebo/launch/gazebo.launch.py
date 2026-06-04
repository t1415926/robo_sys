from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')

    robot_description = {
        'robot_description': Command(['xacro ', model, ' use_ros2_control:=true'])
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
            'my_robot',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.08',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true.',
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
                'my_robot.urdf.xacro',
            ]),
            description='Robot xacro file.',
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster,
        delayed_diff_drive,
    ])
