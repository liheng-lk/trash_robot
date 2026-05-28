from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value='/home/sunrise/trash_robot_v3'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8095'),
        DeclareLaunchArgument('video_url', default_value='http://192.168.1.121:8092/stream.mjpg'),
        Node(
            package='trash_robot_web',
            executable='web_console',
            name='trash_robot_web_console',
            output='screen',
            parameters=[{
                'project_root': LaunchConfiguration('project_root'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'video_url': LaunchConfiguration('video_url'),
            }],
        ),
    ])
