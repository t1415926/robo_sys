from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('marker_topic', default_value='/camera_projection_marker'),
        DeclareLaunchArgument('plane_frame', default_value='map'),
        DeclareLaunchArgument('plane_width', default_value='4.0'),
        DeclareLaunchArgument('plane_height', default_value='3.0'),
        DeclareLaunchArgument('position_x', default_value='0.0'),
        DeclareLaunchArgument('position_y', default_value='0.0'),
        DeclareLaunchArgument('position_z', default_value='0.02'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('alpha', default_value='1.0'),
        Node(
            package='my_robot_navigation',
            executable='image_to_plane_marker.py',
            name='image_to_plane_marker',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'marker_topic': LaunchConfiguration('marker_topic'),
                'plane_frame': LaunchConfiguration('plane_frame'),
                'plane_width': ParameterValue(LaunchConfiguration('plane_width'), value_type=float),
                'plane_height': ParameterValue(LaunchConfiguration('plane_height'), value_type=float),
                'position_x': ParameterValue(LaunchConfiguration('position_x'), value_type=float),
                'position_y': ParameterValue(LaunchConfiguration('position_y'), value_type=float),
                'position_z': ParameterValue(LaunchConfiguration('position_z'), value_type=float),
                'roll': ParameterValue(LaunchConfiguration('roll'), value_type=float),
                'pitch': ParameterValue(LaunchConfiguration('pitch'), value_type=float),
                'yaw': ParameterValue(LaunchConfiguration('yaw'), value_type=float),
                'alpha': ParameterValue(LaunchConfiguration('alpha'), value_type=float),
            }],
        ),
    ])
