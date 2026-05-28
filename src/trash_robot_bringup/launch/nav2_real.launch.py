from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

def generate_launch_description():
    project_root = Path(os.environ.get('TRASH_ROBOT_ROOT', '/home/sunrise/trash_robot_v3'))
    params_file = project_root / 'config' / 'navigation' / 'nav2_params.yaml'
    default_map = project_root / 'maps' / '344.yaml'
    if not default_map.exists():
        maps = sorted(project_root.glob('maps/*.yaml'))
        default_map = maps[0] if maps else default_map
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(default_map)),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[str(params_file), {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[str(params_file)]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[str(params_file)]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[str(params_file)]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[str(params_file)]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[str(params_file)]
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
