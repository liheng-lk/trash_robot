from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():
    project_root = Path(os.environ.get('TRASH_ROBOT_ROOT', '/home/sunrise/trash_robot_v3'))
    params_file = project_root / 'config' / 'navigation' / 'depth_to_scan.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/depth/camera_info'),
        DeclareLaunchArgument('scan_topic', default_value='/scan_depth'),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[str(params_file)],
            remappings=[
                ('depth', LaunchConfiguration('depth_topic')),
                ('depth_camera_info', LaunchConfiguration('camera_info_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
            ],
        )
    ])
