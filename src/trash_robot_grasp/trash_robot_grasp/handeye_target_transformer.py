from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String


class HandeyeTargetTransformer(Node):
    def __init__(self) -> None:
        super().__init__('handeye_target_transformer')

        self.declare_parameter('handeye_file', '/home/sunrise/trash_robot_v3/config/grasp/handeye_point.yaml')
        self.declare_parameter('fallback_handeye_file', '')
        self.declare_parameter('camera_point_source', 'vlm')
        self.declare_parameter('use_legacy_camera_point', False)
        self.declare_parameter('input_topic', '/trash_target_camera_point')
        self.declare_parameter('output_topic', '/trash_target_point_arm')
        self.declare_parameter('arm_frame', 'roarm_sdk_base')

        self.active_camera_source, self.input_topic = self.resolve_input_source()
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.legacy_enabled = self.active_camera_source == 'legacy'
        self.handeye_path: Optional[Path] = None
        self.camera_frame_from_yaml = ''
        self.arm_frame_from_yaml = ''
        self.rot, self.trans = self.load_handeye()
        self.pub = self.create_publisher(PointStamped, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, '/trash_handeye_status', 10)
        self.camera_to_arm_status_pub = self.create_publisher(String, '/trash_camera_to_arm_status', 10)
        self.create_subscription(PointStamped, self.input_topic, self.callback, 10)

    def resolve_input_source(self) -> tuple[str, str]:
        source = str(self.get_parameter('camera_point_source').value or 'vlm').strip().lower()
        legacy_enabled = self.parameter_bool(self.get_parameter('use_legacy_camera_point').value)
        configured_input = str(self.get_parameter('input_topic').value or '').strip()
        if legacy_enabled or source == 'legacy':
            return 'legacy', '/trash_target_point_camera'
        if configured_input and configured_input != '/trash_target_camera_point':
            return 'custom', configured_input
        return 'vlm', '/trash_target_camera_point'

    @staticmethod
    def parameter_bool(value: object) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def load_handeye(self) -> tuple[np.ndarray, np.ndarray]:
        candidates = [Path(str(self.get_parameter('handeye_file').value))]
        fallback = str(self.get_parameter('fallback_handeye_file').value).strip()
        if fallback:
            candidates.append(Path(fallback))
        for path in candidates:
            if not path.exists():
                continue
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            frames = data.get('frames') if isinstance(data.get('frames'), dict) else {}
            self.handeye_path = path
            self.camera_frame_from_yaml = str(frames.get('camera_frame') or '')
            self.arm_frame_from_yaml = str(frames.get('arm_frame') or '')
            if 'matrix_row_major' in data:
                mat = np.array(data['matrix_row_major'], dtype=np.float64).reshape(4, 4)
                self.get_logger().info(
                    f'loaded handeye matrix: {path}; transform_direction=camera_to_arm; formula=arm = R @ camera + t'
                )
                return mat[:3, :3], mat[:3, 3]
            if 'rotation_matrix' in data and 'translation_m' in data:
                rot = np.array(data['rotation_matrix'], dtype=np.float64).reshape(3, 3)
                trans = np.array(data['translation_m'], dtype=np.float64)
                self.get_logger().info(
                    f'loaded handeye rotation/translation: {path}; transform_direction=camera_to_arm; formula=arm = R @ camera + t'
                )
                return rot, trans
        raise FileNotFoundError('No valid handeye calibration file found.')

    def callback(self, msg: PointStamped) -> None:
        camera = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        arm = self.rot @ camera + self.trans

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(self.get_parameter('arm_frame').value)
        out.point.x = float(arm[0])
        out.point.y = float(arm[1])
        out.point.z = float(arm[2])
        self.pub.publish(out)

        status = {
            'input_topic': self.input_topic,
            'output_topic': self.output_topic,
            'active_camera_source': self.active_camera_source,
            'legacy_enabled': self.legacy_enabled,
            'camera_point_m': [float(camera[0]), float(camera[1]), float(camera[2])],
            'arm_point_m': [float(arm[0]), float(arm[1]), float(arm[2])],
            'transform_direction': 'camera_to_arm',
            'unit': 'm',
            'formula': 'arm_point = R @ camera_point + t',
            'rotation_matrix': self.rot.tolist(),
            'translation_m': self.trans.tolist(),
            'handeye_file': str(self.handeye_path or ''),
            'camera_frame_from_msg': str(msg.header.frame_id or ''),
            'camera_frame_from_yaml': self.camera_frame_from_yaml,
            'arm_frame_from_yaml': self.arm_frame_from_yaml,
            'output_frame': out.header.frame_id,
        }
        status_msg = String(data=json.dumps(status, ensure_ascii=False))
        self.status_pub.publish(status_msg)
        self.camera_to_arm_status_pub.publish(status_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HandeyeTargetTransformer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
