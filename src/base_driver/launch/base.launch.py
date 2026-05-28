from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_driver',
            executable='serial_base_node',
            name='base_driver',
            output='screen',
            parameters=[{
                'port': '/dev/base',
                'baud': 115200,
                'cmd_rate': 20.0,
                'cmd_timeout': 0.3,

                'x_cmd_sign': 1.0,
                'z_cmd_sign': 1.0,

                'x_feedback_sign': 1.0,
                'z_feedback_sign': 1.0,

                'z_odom_scale': 1.0,

                'acc_lsb_per_g': 16384.0,
                'gyro_lsb_per_dps': 65.5,
            }]
        )
    ])