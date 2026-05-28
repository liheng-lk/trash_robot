from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    path = Path(get_package_share_directory(package_name)) / relative_path
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("roarm_description", package_name="roarm_moveit")
        .to_moveit_configs()
    )
    servo_params = {
        "moveit_servo": load_yaml(
            "trash_robot_bringup",
            Path("config") / "roarm_servo.yaml",
        )
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([servo_node])
