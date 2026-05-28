from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value='/home/sunrise/trash_robot_v3/config/mission/patrol_routes.yaml'),
        DeclareLaunchArgument('sort_config_file', default_value='/home/sunrise/trash_robot_v3/config/grasp/trash_sort_params.yaml'),
        DeclareLaunchArgument('auto_start', default_value='false'),
        Node(
            package='trash_robot_mission',
            executable='mission_supervisor',
            name='trash_mission_supervisor',
            output='screen',
            parameters=[{
                'route_file': LaunchConfiguration('route_file'),
                'sort_config_file': LaunchConfiguration('sort_config_file'),
                'auto_start': LaunchConfiguration('auto_start'),
            }],
        ),
    ])
