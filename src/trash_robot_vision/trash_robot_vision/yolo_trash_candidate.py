from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

try:
    from ai_msgs.msg import PerceptionTargets
except ImportError:
    PerceptionTargets = None  # type: ignore[assignment]


DEFAULT_ALLOW_CLASSES = (
    'bottle,cup,bowl,book,banana,apple,orange,cell phone,remote,scissors,'
    'vase,trash,garbage,waste,debris,'
    'paper,crumpled paper,plastic bottle,can,carton,box'
)

DEFAULT_IGNORE_CLASSES = (
    'person,chair,couch,bed,dining table,toilet,tv,laptop,keyboard,mouse,'
    'potted plant,door,wall,floor'
)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def parse_csv_set(text: str) -> set[str]:
    return {part.strip().lower() for part in str(text or '').split(',') if part.strip()}


def rough_trash_label(class_name: str) -> str:
    name = class_name.lower()
    if any(key in name for key in ('battery', 'cell', 'remote', 'phone')):
        return 'GARBAGE_HAZARD'
    if any(key in name for key in ('banana', 'apple', 'orange', 'food', 'peel')):
        return 'GARBAGE_KITCHEN'
    if any(key in name for key in ('bottle', 'cup', 'book', 'paper', 'can', 'carton', 'box')):
        return 'GARBAGE_RECYCLE'
    return 'GARBAGE_OTHER'


class YoloTrashCandidate(Node):
    """Convert RDK YOLO ai_msgs detections into trash patrol candidates.

    This node is intentionally only for patrol-time discovery. It does not
    command the arm and does not replace the VLM grasp planner.
    """

    def __init__(self) -> None:
        super().__init__('yolo_trash_candidate')
        if PerceptionTargets is None:
            raise RuntimeError('ai_msgs is not available; source /opt/tros/humble/setup.bash first')

        self.declare_parameter('detection_topic', '/hobot_dnn_detection')
        self.declare_parameter('candidate_topic', '/trash_yolo_candidate')
        self.declare_parameter('debug_camera_point_topic', '/trash_yolo_target_camera_point')
        self.declare_parameter('status_topic', '/trash_yolo_candidate_status')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('model_input_width', 640)
        self.declare_parameter('model_input_height', 640)
        self.declare_parameter('roi_coordinate_mode', 'letterbox')
        self.declare_parameter('min_confidence', 0.30)
        self.declare_parameter('min_depth_m', 0.12)
        self.declare_parameter('max_depth_m', 2.80)
        self.declare_parameter('min_valid_depth_ratio', 0.025)
        self.declare_parameter('reject_edge_margin_norm', 0.04)
        self.declare_parameter('min_bbox_area_norm', 0.002)
        self.declare_parameter('roi_shrink', 0.55)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('max_process_rate_hz', 0.0)
        self.declare_parameter('candidate_ttl_sec', 0.8)
        self.declare_parameter('require_depth', True)
        self.declare_parameter('allow_classes', DEFAULT_ALLOW_CLASSES)
        self.declare_parameter('ignore_classes', DEFAULT_IGNORE_CLASSES)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth_image: Optional[np.ndarray] = None
        self.depth_stamp = None
        self.depth_frame = ''
        self.camera_info: Optional[CameraInfo] = None
        self.latest_candidate: dict[str, Any] = {}
        self.latest_candidate_stamp = 0.0
        self.last_process_stamp = 0.0
        self.last_no_candidate_pub = 0.0

        self.candidate_pub = self.create_publisher(String, self.param_str('candidate_topic'), 10)
        self.status_pub = self.create_publisher(String, self.param_str('status_topic'), 10)
        self.camera_point_pub = self.create_publisher(
            PointStamped,
            self.param_str('debug_camera_point_topic'),
            10,
        )
        self.create_subscription(
            Image,
            self.param_str('depth_topic'),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.param_str('camera_info_topic'),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PerceptionTargets,
            self.param_str('detection_topic'),
            self.detection_callback,
            det_qos,
        )
        period = 1.0 / max(1.0, self.param_float('publish_rate_hz'))
        self.timer = self.create_timer(period, self.publish_latest)
        self.get_logger().info(
            f'yolo trash candidate ready detection={self.param_str("detection_topic")} '
            f'candidate={self.param_str("candidate_topic")}'
        )

    def param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.status_pub.publish(String(data=f'DEPTH_CONVERT_FAILED {exc}'))
            return
        depth_np = np.asarray(depth)
        if depth_np.ndim > 2:
            depth_np = depth_np[:, :, 0]
        with self.lock:
            self.depth_image = depth_np.copy()
            self.depth_stamp = msg.header.stamp
            self.depth_frame = str(msg.header.frame_id or '')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        with self.lock:
            self.camera_info = msg

    def detection_callback(self, msg: Any) -> None:
        now = time.time()
        max_process_rate = self.param_float('max_process_rate_hz')
        if max_process_rate > 0.0:
            min_interval = 1.0 / max(0.1, max_process_rate)
            if now - self.last_process_stamp < min_interval:
                return
            self.last_process_stamp = now
        with self.lock:
            depth = None if self.depth_image is None else self.depth_image.copy()
            camera_info = self.camera_info
            depth_frame = self.depth_frame
        allow = parse_csv_set(self.param_str('allow_classes'))
        ignore = parse_csv_set(self.param_str('ignore_classes'))
        best: Optional[dict[str, Any]] = None
        best_score = -1.0
        for target in getattr(msg, 'targets', []):
            candidate = self.target_to_candidate(target, depth, camera_info, depth_frame)
            if candidate is None:
                continue
            class_name = str(candidate.get('class_name', '')).lower()
            if class_name in ignore:
                continue
            if allow and class_name not in allow and not any(key in class_name for key in allow):
                continue
            score = float(candidate.get('confidence', 0.0))
            center = candidate.get('center_norm', [0.5, 0.5])
            try:
                score += 0.10 * clamp(float(center[1]), 0.0, 1.0)
            except (TypeError, ValueError, IndexError):
                pass
            if score > best_score:
                best = candidate
                best_score = score
        with self.lock:
            if best is not None:
                self.latest_candidate = best
                self.latest_candidate_stamp = now
            elif now - self.latest_candidate_stamp > self.param_float('candidate_ttl_sec'):
                self.latest_candidate = {}

    def target_to_candidate(
        self,
        target: Any,
        depth: Optional[np.ndarray],
        camera_info: Optional[CameraInfo],
        depth_frame: str,
    ) -> Optional[dict[str, Any]]:
        roi_msg = self.best_roi(target)
        if roi_msg is None:
            return None
        class_name = str(getattr(target, 'type', '') or getattr(roi_msg, 'type', '') or 'object').strip()
        confidence = float(getattr(roi_msg, 'confidence', 0.0) or 0.0)
        if confidence < self.param_float('min_confidence'):
            return None

        rect = roi_msg.rect
        image_w = int(self.get_parameter('image_width').value)
        image_h = int(self.get_parameter('image_height').value)
        if camera_info is not None and camera_info.width > 0 and camera_info.height > 0:
            image_w = int(camera_info.width)
            image_h = int(camera_info.height)
        raw_x0 = float(getattr(rect, 'x_offset', 0))
        raw_y0 = float(getattr(rect, 'y_offset', 0))
        raw_width = float(getattr(rect, 'width', 0))
        raw_height = float(getattr(rect, 'height', 0))
        if raw_width <= 1 or raw_height <= 1:
            return None
        raw_x1 = raw_x0 + raw_width
        raw_y1 = raw_y0 + raw_height

        x0, y0, x1, y1 = self.roi_to_image_rect(raw_x0, raw_y0, raw_x1, raw_y1, image_w, image_h)
        if x1 <= x0 + 1 or y1 <= y0 + 1:
            return None
        width = x1 - x0
        height = y1 - y0
        cx = x0 + width * 0.5
        cy = y0 + height * 0.5
        camera_point = None
        depth_m = None
        valid_ratio = 0.0
        camera_frame = depth_frame or self.param_str('camera_frame_id')
        if depth is not None and camera_info is not None:
            estimate = self.estimate_depth_point(depth, camera_info, x0, y0, x1, y1, cx, cy)
            if estimate is not None:
                camera_point, depth_m, valid_ratio = estimate
                camera_frame = depth_frame or str(camera_info.header.frame_id or camera_frame)
        if self.param_bool('require_depth') and camera_point is None:
            return None

        bbox_norm = [
            clamp(x0 / max(1.0, float(image_w)), 0.0, 1.0),
            clamp(y0 / max(1.0, float(image_h)), 0.0, 1.0),
            clamp(x1 / max(1.0, float(image_w)), 0.0, 1.0),
            clamp(y1 / max(1.0, float(image_h)), 0.0, 1.0),
        ]
        bbox_w = max(0.0, bbox_norm[2] - bbox_norm[0])
        bbox_h = max(0.0, bbox_norm[3] - bbox_norm[1])
        bbox_area = bbox_w * bbox_h
        if bbox_area < max(0.0, self.param_float('min_bbox_area_norm')):
            return None
        edge_margin = clamp(self.param_float('reject_edge_margin_norm'), 0.0, 0.20)
        if edge_margin > 0.0 and (
            bbox_norm[0] <= edge_margin
            or bbox_norm[1] <= edge_margin
            or bbox_norm[2] >= 1.0 - edge_margin
            or bbox_norm[3] >= 1.0 - edge_margin
        ):
            return None
        center_norm = [
            clamp(cx / max(1.0, float(image_w)), 0.0, 1.0),
            clamp(cy / max(1.0, float(image_h)), 0.0, 1.0),
        ]
        out = {
            'has_candidate': True,
            'provider': 'rdk_yolov8n',
            'class_name': class_name,
            'object_name': class_name,
            'trash_label': rough_trash_label(class_name),
            'confidence': round(confidence, 4),
            'bbox_norm': [round(v, 5) for v in bbox_norm],
            'center_norm': [round(v, 5) for v in center_norm],
            'raw_bbox': [round(raw_x0, 2), round(raw_y0, 2), round(raw_x1, 2), round(raw_y1, 2)],
            'track_id': int(getattr(target, 'track_id', 0) or 0),
            'source_topic': self.param_str('detection_topic'),
            'depth_m': None if depth_m is None else round(float(depth_m), 4),
            'depth_valid_ratio': round(float(valid_ratio), 4),
            'camera_frame': camera_frame,
            'stamp': time.time(),
        }
        if camera_point is not None:
            out['camera_point_m'] = [round(float(v), 5) for v in camera_point]
        return out

    def roi_to_image_rect(
        self,
        raw_x0: float,
        raw_y0: float,
        raw_x1: float,
        raw_y1: float,
        image_w: int,
        image_h: int,
    ) -> tuple[int, int, int, int]:
        mode = self.param_str('roi_coordinate_mode').strip().lower()
        if mode == 'auto':
            model_w = int(self.get_parameter('model_input_width').value)
            model_h = int(self.get_parameter('model_input_height').value)
            mode = 'letterbox' if model_w > 0 and model_h > 0 and (model_w != image_w or model_h != image_h) else 'image'

        if mode == 'letterbox':
            model_w = max(1.0, float(self.get_parameter('model_input_width').value))
            model_h = max(1.0, float(self.get_parameter('model_input_height').value))
            scale = min(model_w / max(1.0, float(image_w)), model_h / max(1.0, float(image_h)))
            pad_x = (model_w - float(image_w) * scale) * 0.5
            pad_y = (model_h - float(image_h) * scale) * 0.5
            x0 = (raw_x0 - pad_x) / scale
            y0 = (raw_y0 - pad_y) / scale
            x1 = (raw_x1 - pad_x) / scale
            y1 = (raw_y1 - pad_y) / scale
        else:
            x0, y0, x1, y1 = raw_x0, raw_y0, raw_x1, raw_y1

        return (
            int(round(clamp(x0, 0.0, float(image_w - 1)))),
            int(round(clamp(y0, 0.0, float(image_h - 1)))),
            int(round(clamp(x1, 0.0, float(image_w)))),
            int(round(clamp(y1, 0.0, float(image_h)))),
        )

    def best_roi(self, target: Any) -> Any:
        rois = list(getattr(target, 'rois', []) or [])
        if not rois:
            return None
        return max(rois, key=lambda item: float(getattr(item, 'confidence', 0.0) or 0.0))

    def estimate_depth_point(
        self,
        depth: np.ndarray,
        camera_info: CameraInfo,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        cx: float,
        cy: float,
    ) -> Optional[tuple[list[float], float, float]]:
        h, w = depth.shape[:2]
        x0 = max(0, min(w - 1, x0))
        x1 = max(0, min(w, x1))
        y0 = max(0, min(h - 1, y0))
        y1 = max(0, min(h, y1))
        if x1 <= x0 + 1 or y1 <= y0 + 1:
            return None

        shrink = clamp(self.param_float('roi_shrink'), 0.1, 1.0)
        roi_w = x1 - x0
        roi_h = y1 - y0
        pad_x = int(roi_w * (1.0 - shrink) * 0.5)
        pad_y = int(roi_h * (1.0 - shrink) * 0.5)
        sx0 = max(x0, x0 + pad_x)
        sx1 = min(x1, x1 - pad_x)
        sy0 = max(y0, y0 + pad_y)
        sy1 = min(y1, y1 - pad_y)
        roi = depth[sy0:sy1, sx0:sx1]
        if roi.size == 0:
            return None

        roi_float = roi.astype(np.float32)
        if np.issubdtype(depth.dtype, np.integer):
            roi_float *= 0.001
        valid = np.isfinite(roi_float)
        min_depth = self.param_float('min_depth_m')
        max_depth = self.param_float('max_depth_m')
        valid &= roi_float >= min_depth
        valid &= roi_float <= max_depth
        valid_ratio = float(np.count_nonzero(valid)) / max(1, int(roi_float.size))
        if valid_ratio < self.param_float('min_valid_depth_ratio'):
            return None
        depth_m = float(np.median(roi_float[valid]))
        if not math.isfinite(depth_m):
            return None

        k = camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        ppx = float(k[2])
        ppy = float(k[5])
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return None
        x = (float(cx) - ppx) * depth_m / fx
        y = (float(cy) - ppy) * depth_m / fy
        return [x, y, depth_m], depth_m, valid_ratio

    def publish_latest(self) -> None:
        now = time.time()
        ttl = self.param_float('candidate_ttl_sec')
        with self.lock:
            candidate = dict(self.latest_candidate)
            age = now - self.latest_candidate_stamp
        if candidate and age <= ttl:
            candidate['age_sec'] = round(age, 3)
            text = json.dumps(candidate, ensure_ascii=False)
            self.candidate_pub.publish(String(data=text))
            point = candidate.get('camera_point_m')
            if isinstance(point, list) and len(point) == 3:
                msg = PointStamped()
                msg.header.frame_id = str(candidate.get('camera_frame') or self.param_str('camera_frame_id'))
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = float(point[0])
                msg.point.y = float(point[1])
                msg.point.z = float(point[2])
                self.camera_point_pub.publish(msg)
            return

        if now - self.last_no_candidate_pub >= 0.5:
            self.last_no_candidate_pub = now
            self.candidate_pub.publish(
                String(data=json.dumps({'has_candidate': False, 'provider': 'rdk_yolov8n'}, ensure_ascii=False))
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloTrashCandidate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
