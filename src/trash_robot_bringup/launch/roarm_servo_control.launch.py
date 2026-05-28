from pathlib import Path

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    project_root = Path("/home/sunrise/trash_robot_v3")
    servo_yaml = project_root / "src" / "moveit_servo" / "config" / "roarm_simulated_config.yaml"
    moveit_config = (
        MoveItConfigsBuilder("roarm_description", package_name="roarm_moveit")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            output="screen",
            parameters=[
                {"moveit_servo": load_yaml(servo_yaml)},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
    ])
