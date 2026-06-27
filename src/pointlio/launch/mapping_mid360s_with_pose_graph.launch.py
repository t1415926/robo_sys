from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz.')
    startup_config_arg = DeclareLaunchArgument(
        'startup_config', default_value='mid360s_live_frontend_no_prior.yaml',
        description='Point-LIO frontend parameter file under share/point_lio/config.')
    graph_config_arg = DeclareLaunchArgument(
        'graph_config', default_value='pose_graph_mid360s.yaml',
        description='Pose graph backend parameter file under share/point_lio/config.')

    point_lio_params = [
        {
            'use_imu_as_input': True,
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 60,
            'point_filter_num': 1,
            'space_down_sample': True,
            'filter_size_surf': 0.2,
            'filter_size_map': 0.2,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
        },
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config',
            LaunchConfiguration('startup_config')
        ]),
    ]

    graph_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'config',
            LaunchConfiguration('graph_config')
        ]),
    ]

    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=point_lio_params,
    )

    pose_graph_node = Node(
        package='point_lio',
        executable='pose_graph_backend',
        name='pose_graph_backend',
        output='screen',
        parameters=graph_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg',
            'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice',
    )

    return LaunchDescription([
        rviz_arg,
        startup_config_arg,
        graph_config_arg,
        laser_mapping_node,
        pose_graph_node,
        GroupAction(actions=[rviz_node], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
