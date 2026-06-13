from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz')
    
    mapping_arg = DeclareLaunchArgument(
        'mapping', default_value='false',
        description='Enable mapping mode (false for localization)')
    
    # 配置文件路径
    config_path = PathJoinSubstitution([
        FindPackageShare('prior_map_localization_ros2'),
        'config', 'with_map.yaml'
    ])
    
    # 地图文件默认路径
    default_map_path = PathJoinSubstitution([
        FindPackageShare('prior_map_localization_ros2'),
        'maps', 'prior_map.pcd'
    ])
    
    # 点云地图节点 (SLAM建图)
    point_lio_mapping = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_path]
    )
    
    # 先验地图定位节点
    prior_localizer = Node(
        package='prior_map_localization_ros2',
        executable='prior_map_localizer_node',
        name='prior_map_localization',
        output='screen',
        parameters=[
            config_path,
            {'mapping_en': LaunchConfiguration('mapping')},
            {'map_path': default_map_path}
        ],
        remappings=[
            ('points_in', '/livox/lidar'),
            ('imu_in', '/livox/imu')
        ]
    )
    
    # RViz可视化
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('prior_map_localization_ros2'),
        'rviz', 'prior_map_localization.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )
    
    # TF静态广播器 (可选的IMU-LiDAR外参)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar_imu',
        arguments=[
            '--x', '0.04412', '--y', '-0.02329', '--z', '-0.011',
            '--qx', '0.5', '--qy', '0.5', '--qz', '0.5', '--qw', '0.5',
            '--frame-id', 'livox_frame', '--child-frame-id', 'imu'
        ]
    )
    
    # 构建启动描述
    return LaunchDescription([
        rviz_arg,
        mapping_arg,
        prior_localizer,
        GroupAction(
            actions=[point_lio_mapping],
            condition=IfCondition(LaunchConfiguration('mapping'))
        ),
        rviz_node,
        static_tf
    ])