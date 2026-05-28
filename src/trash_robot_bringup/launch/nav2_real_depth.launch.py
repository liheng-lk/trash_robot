from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():
    project_root = Path(os.environ.get('TRASH_ROBOT_ROOT', '/home/sunrise/trash_robot_v3'))
    base_params = project_root / 'config' / 'navigation' / 'nav2_params.yaml'
    overlay_params = project_root / 'config' / 'navigation' / 'nav2_depth_overlay.yaml'
    default_map = project_root / 'maps' / '344.yaml'
    if not default_map.exists():
        maps = sorted(project_root.glob('maps/*.yaml'))
        default_map = maps[0] if maps else default_map
    map_file = LaunchConfiguration('map')
    nav_params = [str(base_params)]
    if overlay_params.exists():
        nav_params.append(str(overlay_params))

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(default_map)),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[*nav_params, {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=nav_params
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=nav_params
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=nav_params
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=nav_params
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=nav_params
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
