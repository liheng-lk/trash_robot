from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('auto_grasp', default_value='false'),
        DeclareLaunchArgument('auto_execute', default_value='false'),
        DeclareLaunchArgument('camera_point_source', default_value='vlm'),
        DeclareLaunchArgument('use_legacy_camera_point', default_value='false'),
        DeclareLaunchArgument('handeye_file', default_value='/home/sunrise/trash_robot_v3/config/grasp/handeye_point.yaml'),
        DeclareLaunchArgument('sort_config_file', default_value='/home/sunrise/trash_robot_v3/config/grasp/trash_sort_params.yaml'),

        Node(
            package='trash_robot_grasp',
            executable='handeye_target_transformer',
            name='handeye_target_transformer',
            output='screen',
            parameters=[{
                'handeye_file': LaunchConfiguration('handeye_file'),
                'camera_point_source': LaunchConfiguration('camera_point_source'),
                'use_legacy_camera_point': LaunchConfiguration('use_legacy_camera_point'),
                'output_topic': '/trash_target_point_arm',
            }],
        ),
        Node(
            package='trash_robot_grasp',
            executable='roarm_sort_grasper',
            name='roarm_sort_grasper',
            output='screen',
            parameters=[{
                'dry_run': LaunchConfiguration('dry_run'),
                'auto_grasp': LaunchConfiguration('auto_grasp'),
                'auto_execute': LaunchConfiguration('auto_execute'),
                'sort_config_file': LaunchConfiguration('sort_config_file'),
                'clear_vlm_cache_before_manual_grasp': False,
            }],
        ),
    ])
