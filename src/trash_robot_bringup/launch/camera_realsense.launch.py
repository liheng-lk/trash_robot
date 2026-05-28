import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _bool_text(value, default=False):
    return 'true' if bool(value if value is not None else default) else 'false'


def generate_launch_description():
    project_root = Path(os.environ.get('TRASH_ROBOT_ROOT', '/home/sunrise/trash_robot_v3'))
    config_file = project_root / 'config' / 'hardware' / 'camera_realsense.yaml'
    config = yaml.safe_load(config_file.read_text(encoding='utf-8')) if config_file.exists() else {}
    params = ((config or {}).get('camera') or {}).get('ros__parameters') or {}
    mode = os.environ.get('TRASH_CAMERA_MODE', 'full').strip().lower()

    rs_launch = Path(get_package_share_directory('realsense2_camera')) / 'launch' / 'rs_launch.py'
    enable_depth = params.get('enable_depth', True)
    enable_sync = params.get('enable_sync', True)
    align_depth = params.get('align_depth_enable', True)
    pointcloud = params.get('pointcloud_enable', True)
    if mode in ('handeye', 'color', 'color_only'):
        enable_depth = False
        enable_sync = False
        align_depth = False
        pointcloud = False

    arguments = {
        'camera_name': str(params.get('camera_name', 'camera')),
        'enable_color': _bool_text(params.get('enable_color'), True),
        'enable_depth': _bool_text(enable_depth, True),
        'enable_sync': _bool_text(enable_sync, True),
        'align_depth.enable': _bool_text(align_depth, True),
        'pointcloud.enable': _bool_text(pointcloud, True),
        'rgb_camera.color_profile': str(params.get('color_profile', '640,480,15')),
        'depth_module.depth_profile': str(params.get('depth_profile', '640,480,15')),
        'initial_reset': _bool_text(params.get('initial_reset'), False),
        'reconnect_timeout': str(params.get('reconnect_timeout', 6.0)),
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rs_launch)),
            launch_arguments=arguments.items(),
        ),
    ])
