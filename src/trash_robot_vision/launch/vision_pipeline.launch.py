from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vlm_config_file', default_value='/home/sunrise/trash_robot_v3/config/perception/vlm_trash_classifier.yaml'),
        DeclareLaunchArgument('vlm_provider', default_value=''),
        DeclareLaunchArgument('vlm_enabled', default_value='true'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('pixel_image_width', default_value='640'),
        DeclareLaunchArgument('pixel_image_height', default_value='480'),
        DeclareLaunchArgument('depth_window', default_value='61'),
        DeclareLaunchArgument('enable_legacy_depth_locator', default_value='false'),
        Node(
            package='trash_robot_vision',
            executable='vlm_trash_classifier',
            name='vlm_trash_classifier',
            output='screen',
            parameters=[{
                'enabled': LaunchConfiguration('vlm_enabled'),
                'config_file': LaunchConfiguration('vlm_config_file'),
                'provider': LaunchConfiguration('vlm_provider'),
                'image_topic': '/camera/camera/color/image_raw',
            }],
        ),
        Node(
            package='trash_robot_vision',
            executable='pixel_depth_locator',
            name='pixel_depth_locator',
            output='screen',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('enable_legacy_depth_locator'), 'true')),
            parameters=[{
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'pixel_image_width': LaunchConfiguration('pixel_image_width'),
                'pixel_image_height': LaunchConfiguration('pixel_image_height'),
                'depth_window': LaunchConfiguration('depth_window'),
            }],
        ),
    ])
