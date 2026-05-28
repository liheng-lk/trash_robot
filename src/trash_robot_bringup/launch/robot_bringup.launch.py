from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

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
                'x_feedback_sign': 1.0,
                'z_cmd_sign': 1.0,
                'z_feedback_sign': 1.0,
                'acc_lsb_per_g': 16384.0,
                'gyro_lsb_per_dps': 65.5,
                'yaw_source': 'encoder',
                'yaw_blend_alpha': 0.25,
                'vz_enc_scale': 1.03,
                'imu_yaw_sign': -1.0,
                'imu_lpf_alpha': 0.2,
                'yaw_deadband': 0.01,
                'min_effective_z_cmd': 0.35,
                'linear_deadband': 0.005,
                'freeze_yaw_when_stationary': True,
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                'serial_port': '/dev/sllidar',
                'serial_baudrate': '115200',
                'frame_id': 'laser_frame'
            }.items()
        ),

        # New body with rear trash-bin extension:
        # total length 350 mm, base_link at vehicle center, base_link height 53 mm.
        # LiDAR is 260 mm forward from rear edge: -175 + 260 = +85 mm.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                "--x", "0.085", "--y", "0.0", "--z", "0.177",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "laser_frame"
            ]
        ),

        # Camera lens is 368 mm forward from rear edge: -175 + 368 = +193 mm.
        # Lens height is 80 mm from ground: 80 - 53 = 27 mm above base_link.
        # The camera looks downward by 20 deg, so this is pitch, not yaw.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                "--x", "0.193", "--y", "0.0", "--z", "0.027",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "camera_link"
            ]
        ),
    ])
