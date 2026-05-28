from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_launch = PathJoinSubstitution([
        FindPackageShare('trash_robot_vision'),
        'launch',
        'vision_pipeline.launch.py',
    ])
    grasp_launch = PathJoinSubstitution([
        FindPackageShare('trash_robot_grasp'),
        'launch',
        'grasp_pipeline.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('auto_grasp', default_value='false'),
        DeclareLaunchArgument('auto_execute', default_value='false'),
        DeclareLaunchArgument('camera_point_source', default_value='vlm'),
        DeclareLaunchArgument('use_legacy_camera_point', default_value='false'),
        DeclareLaunchArgument('handeye_file', default_value='/home/sunrise/trash_robot_v3/config/grasp/handeye_point.yaml'),
        DeclareLaunchArgument('sort_config_file', default_value='/home/sunrise/trash_robot_v3/config/grasp/trash_sort_params.yaml'),
        DeclareLaunchArgument('vlm_config_file', default_value='/home/sunrise/trash_robot_v3/config/perception/vlm_trash_classifier.yaml'),
        DeclareLaunchArgument('vlm_provider', default_value=''),
        DeclareLaunchArgument('pixel_image_width', default_value='640'),
        DeclareLaunchArgument('pixel_image_height', default_value='480'),
        DeclareLaunchArgument('min_score', default_value='0.12'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch),
            launch_arguments={
                'vlm_config_file': LaunchConfiguration('vlm_config_file'),
                'vlm_provider': LaunchConfiguration('vlm_provider'),
                'pixel_image_width': LaunchConfiguration('pixel_image_width'),
                'pixel_image_height': LaunchConfiguration('pixel_image_height'),
                'min_score': LaunchConfiguration('min_score'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(grasp_launch),
            launch_arguments={
                'dry_run': LaunchConfiguration('dry_run'),
                'auto_grasp': LaunchConfiguration('auto_grasp'),
                'auto_execute': LaunchConfiguration('auto_execute'),
                'camera_point_source': LaunchConfiguration('camera_point_source'),
                'use_legacy_camera_point': LaunchConfiguration('use_legacy_camera_point'),
                'handeye_file': LaunchConfiguration('handeye_file'),
                'sort_config_file': LaunchConfiguration('sort_config_file'),
            }.items(),
        ),
    ])
