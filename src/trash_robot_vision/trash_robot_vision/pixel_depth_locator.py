from __future__ import annotations

import json
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class PixelDepthLocator(Node):
    def __init__(self) -> None:
        super().__init__('pixel_depth_locator')

        self.declare_parameter('pixel_topic', '/trash_target_pixel')
        self.declare_parameter('bbox_topic', '/trash_target_bbox')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('fallback_depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('depth_max_age_sec', 0.75)
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_topic', '/trash_target_point_camera')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('pixel_image_width', 1920)
        self.declare_parameter('pixel_image_height', 1080)
        self.declare_parameter('depth_window', 61)
        self.declare_parameter('bbox_max_age_sec', 1.0)
        self.declare_parameter('bbox_inner_margin_ratio', 0.12)
        self.declare_parameter('bbox_depth_percentile', 20.0)
        self.declare_parameter('bbox_near_band_m', 0.025)
        self.declare_parameter('bbox_near_band_ratio', 0.10)
        self.declare_parameter('bbox_min_near_pixels', 8)
        self.declare_parameter('local_depth_window', 21)
        self.declare_parameter('local_min_valid_pixels', 12)
        self.declare_parameter('stable_history_size', 8)
        self.declare_parameter('stable_min_samples', 3)
        self.declare_parameter('stable_cluster_radius_m', 0.045)
        self.declare_parameter('max_target_age_sec', 1.0)
        self.declare_parameter('front_z_min_m', 0.05)
        self.declare_parameter('front_z_max_m', 0.60)
        self.declare_parameter('smoothing_enabled', True)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('smoothing_reset_jump_m', 0.08)

        self.bridge = CvBridge()
        self.last_pixel: Optional[PointStamped] = None
        self.last_bbox: Optional[dict] = None
        self.primary_depth: Optional[Image] = None
        self.primary_depth_time = 0.0
        self.primary_depth_topic = str(self.get_parameter('depth_topic').value).strip()
        self.fallback_depth: Optional[Image] = None
        self.fallback_depth_time = 0.0
        self.fallback_depth_topic = str(self.get_parameter('fallback_depth_topic').value).strip()
        self.camera_info: Optional[CameraInfo] = None
        self.last_wait_status_time = 0.0
        self.smoothed_xyz: Optional[tuple[float, float, float]] = None
        self.depth_history: list[tuple[float, float, float]] = []
        realsense_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            PointStamped,
            str(self.get_parameter('pixel_topic').value),
            self.pixel_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('bbox_topic').value),
            self.bbox_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.primary_depth_topic,
            lambda msg, topic=self.primary_depth_topic: self.depth_callback(msg, topic),
            realsense_qos,
        )
        if self.fallback_depth_topic and self.fallback_depth_topic != self.primary_depth_topic:
            self.create_subscription(
                Image,
                self.fallback_depth_topic,
                lambda msg, topic=self.fallback_depth_topic: self.depth_callback(msg, topic),
                realsense_qos,
            )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback,
            realsense_qos,
        )

        self.point_pub = self.create_publisher(PointStamped, str(self.get_parameter('output_topic').value), 10)
        self.status_pub = self.create_publisher(String, '/trash_depth_status', 10)
        self.timer = self.create_timer(0.1, self.process)
        self.get_logger().info(
            'pixel_depth_locator subscribed: '
            f'pixel={self.get_parameter("pixel_topic").value}, '
            f'depth={self.primary_depth_topic}, fallback_depth={self.fallback_depth_topic}, '
            f'camera_info={self.get_parameter("camera_info_topic").value}'
        )

    def pixel_callback(self, msg: PointStamped) -> None:
        self.last_pixel = msg

    def bbox_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            self.last_bbox = data

    def depth_callback(self, msg: Image, topic: str) -> None:
        now = time.time()
        if topic == self.primary_depth_topic:
            self.primary_depth = msg
            self.primary_depth_time = now
            return
        if topic == self.fallback_depth_topic:
            self.fallback_depth = msg
            self.fallback_depth_time = now
            return
        self.fallback_depth = msg
        self.fallback_depth_time = now

    def _select_depth(self) -> tuple[Optional[Image], str, float]:
        max_age = max(0.1, float(self.get_parameter('depth_max_age_sec').value))
        now = time.time()
        if self.primary_depth is not None:
            age = now - self.primary_depth_time
            if age <= max_age:
                return self.primary_depth, self.primary_depth_topic, age
        if self.fallback_depth is not None:
            age = now - self.fallback_depth_time
            if age <= max_age:
                return self.fallback_depth, self.fallback_depth_topic, age
        return None, '', 0.0

    def _depth_wait_detail(self) -> str:
        now = time.time()
        parts = []
        if self.primary_depth is None:
            parts.append('primary=none')
        else:
            parts.append(f'primary_age={now - self.primary_depth_time:.2f}s')
        if self.fallback_depth_topic:
            if self.fallback_depth is None:
                parts.append('fallback=none')
            else:
                parts.append(f'fallback_age={now - self.fallback_depth_time:.2f}s')
        return '/'.join(parts)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _depth_meters(self, image: np.ndarray, u: int, v: int) -> Optional[float]:
        half = max(0, int(self.get_parameter('depth_window').value) // 2)
        y0 = max(0, v - half)
        y1 = min(image.shape[0], v + half + 1)
        x0 = max(0, u - half)
        x1 = min(image.shape[1], u + half + 1)
        roi = image[y0:y1, x0:x1]
        if roi.size == 0:
            return None

        if roi.dtype == np.uint16:
            values = roi.astype(np.float32) / 1000.0
        else:
            values = roi.astype(np.float32)

        values = values[np.isfinite(values)]
        values = values[(values > 0.03) & (values < 3.0)]
        if values.size == 0:
            return None
        return float(np.median(values))

    @staticmethod
    def _valid_depth_values(roi: np.ndarray) -> np.ndarray:
        if roi.dtype == np.uint16:
            values = roi.astype(np.float32) / 1000.0
        else:
            values = roi.astype(np.float32)
        values = values[np.isfinite(values)]
        return values[(values > 0.03) & (values < 3.0)]

    def _local_median_depth_at_grasp(
        self,
        image: np.ndarray,
        grasp_u: int,
        grasp_v: int,
        inner_x0: int,
        inner_y0: int,
        inner_x1: int,
        inner_y1: int,
    ) -> Optional[tuple[float, int, int]]:
        window = max(5, int(self.get_parameter('local_depth_window').value))
        half = max(2, window // 2)
        x0 = max(inner_x0, grasp_u - half)
        x1 = min(inner_x1, grasp_u + half)
        y0 = max(inner_y0, grasp_v - half)
        y1 = min(inner_y1, grasp_v + half)
        if x1 <= x0 or y1 <= y0:
            return None

        values = self._valid_depth_values(image[y0 : y1 + 1, x0 : x1 + 1])
        min_valid = max(3, int(self.get_parameter('local_min_valid_pixels').value))
        if values.size < min_valid:
            return None

        median = float(np.median(values))
        # Reject speckle/edge reflections inside the local patch. A low
        # percentile is tempting for ground objects, but it jumps to gripper,
        # glossy floor, or bbox edges in dim light. The VLM already selected a
        # semantic grasp point, so keep the pixel fixed and estimate depth from
        # the local dominant cluster around that exact point.
        band = max(0.018, median * 0.06)
        inliers = values[np.abs(values - median) <= band]
        if inliers.size >= min_valid:
            median = float(np.median(inliers))
        return median, grasp_u, grasp_v

    def _depth_from_bbox(
        self,
        image: np.ndarray,
        bbox: dict,
        u_source: float,
        v_source: float,
    ) -> Optional[tuple[float, int, int]]:
        src_w = float(bbox.get('image_width', 0.0) or 0.0)
        src_h = float(bbox.get('image_height', 0.0) or 0.0)
        if src_w <= 0.0 or src_h <= 0.0:
            return None

        x0 = int(round(float(bbox.get('u0', 0.0)) * image.shape[1] / src_w))
        x1 = int(round(float(bbox.get('u1', 0.0)) * image.shape[1] / src_w))
        y0 = int(round(float(bbox.get('v0', 0.0)) * image.shape[0] / src_h))
        y1 = int(round(float(bbox.get('v1', 0.0)) * image.shape[0] / src_h))
        x0, x1 = sorted((max(0, x0), min(image.shape[1] - 1, x1)))
        y0, y1 = sorted((max(0, y0), min(image.shape[0] - 1, y1)))
        if x1 <= x0 or y1 <= y0:
            return None

        grasp_u = int(round(float(bbox.get('grasp_u', u_source)) * image.shape[1] / src_w))
        grasp_v = int(round(float(bbox.get('grasp_v', v_source)) * image.shape[0] / src_h))
        grasp_u = max(x0, min(x1, grasp_u))
        grasp_v = max(y0, min(y1, grasp_v))

        margin_ratio = max(0.0, min(0.35, float(self.get_parameter('bbox_inner_margin_ratio').value)))
        margin_x = int(round((x1 - x0 + 1) * margin_ratio))
        margin_y = int(round((y1 - y0 + 1) * margin_ratio))
        inner_x0 = min(x1, x0 + margin_x)
        inner_x1 = max(inner_x0, x1 - margin_x)
        inner_y0 = min(y1, y0 + margin_y)
        inner_y1 = max(inner_y0, y1 - margin_y)
        grasp_u = max(inner_x0, min(inner_x1, grasp_u))
        grasp_v = max(inner_y0, min(inner_y1, grasp_v))

        local_depth = self._local_median_depth_at_grasp(
            image,
            grasp_u,
            grasp_v,
            inner_x0,
            inner_y0,
            inner_x1,
            inner_y1,
        )
        if local_depth is not None:
            return local_depth

        local_depth = self._depth_from_local_patch(image, grasp_u, grasp_v, inner_x0, inner_y0, inner_x1, inner_y1)
        if local_depth is not None:
            return local_depth

        roi = image[inner_y0 : inner_y1 + 1, inner_x0 : inner_x1 + 1]
        if roi.size == 0:
            return None
        depth_result = self._foreground_depth_from_roi(roi, inner_x0, inner_y0)
        if depth_result is None:
            return None
        z, u, v = depth_result
        return z, u, v

    def _foreground_depth_from_roi(
        self,
        roi: np.ndarray,
        offset_x: int,
        offset_y: int,
    ) -> Optional[tuple[float, int, int]]:
        if roi.dtype == np.uint16:
            values = roi.astype(np.float32) / 1000.0
        else:
            values = roi.astype(np.float32)
        valid = np.isfinite(values) & (values > 0.03) & (values < 3.0)
        if not np.any(valid):
            return None

        valid_values = values[valid]
        percentile = max(1.0, min(50.0, float(self.get_parameter('bbox_depth_percentile').value)))
        # RealSense depth is distance from camera. The object surface is usually
        # the foreground part of the bbox, while the median can fall on the floor
        # or background for flat/irregular trash. Use a low percentile plus a
        # small band to estimate the physical grasp surface.
        z = float(np.percentile(valid_values, percentile))
        band = max(
            float(self.get_parameter('bbox_near_band_m').value),
            z * float(self.get_parameter('bbox_near_band_ratio').value),
        )
        near = valid & (values <= z + band)
        min_near = max(1, int(self.get_parameter('bbox_min_near_pixels').value))
        if int(np.count_nonzero(near)) < min_near:
            z = float(np.median(valid_values))
            near = valid & (np.abs(values - z) <= band)
        if int(np.count_nonzero(near)) < min_near:
            near = valid
        ys, xs = np.where(near)
        if xs.size == 0 or ys.size == 0:
            return None
        u = int(round(float(np.median(xs + offset_x))))
        v = int(round(float(np.median(ys + offset_y))))
        return z, u, v

    def _depth_from_local_patch(
        self,
        image: np.ndarray,
        grasp_u: int,
        grasp_v: int,
        inner_x0: int,
        inner_y0: int,
        inner_x1: int,
        inner_y1: int,
    ) -> Optional[tuple[float, int, int]]:
        depth_window = max(5, int(self.get_parameter('depth_window').value))
        half = max(4, min(depth_window // 2, (inner_x1 - inner_x0 + 1) // 3, (inner_y1 - inner_y0 + 1) // 3))
        x0 = max(inner_x0, grasp_u - half)
        x1 = min(inner_x1, grasp_u + half)
        y0 = max(inner_y0, grasp_v - half)
        y1 = min(inner_y1, grasp_v + half)
        if x1 <= x0 or y1 <= y0:
            return None
        roi = image[y0 : y1 + 1, x0 : x1 + 1]
        if roi.size == 0:
            return None
        result = self._foreground_depth_from_roi(roi, x0, y0)
        if result is None:
            return None
        z, u, v = result
        # The VLM selected a semantic grasp point. Keep the final pixel close to
        # that point so bbox foreground depth cannot drift to another object edge.
        max_shift = max(3.0, float(half) * 0.75)
        if ((float(u - grasp_u) ** 2 + float(v - grasp_v) ** 2) ** 0.5) > max_shift:
            u, v = grasp_u, grasp_v
        return z, u, v

    def process(self) -> None:
        depth_msg, active_depth_topic, depth_age = self._select_depth()
        if self.last_pixel is None or depth_msg is None or self.camera_info is None:
            now = time.time()
            if now - self.last_wait_status_time > 1.0:
                missing = []
                if self.last_pixel is None:
                    missing.append('pixel')
                if depth_msg is None:
                    missing.append(f'depth({self._depth_wait_detail()})')
                if self.camera_info is None:
                    missing.append('camera_info')
                self.status_pub.publish(String(data=f'WAIT_INPUT missing={",".join(missing)}'))
                self.last_wait_status_time = now
            return

        age = (self.get_clock().now().nanoseconds * 1e-9) - (
            self.last_pixel.header.stamp.sec + self.last_pixel.header.stamp.nanosec * 1e-9
        )
        if age > float(self.get_parameter('max_target_age_sec').value):
            self.status_pub.publish(String(data=f'TARGET_PIXEL_TOO_OLD age={age:.2f}s'))
            return

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        u_source = float(self.last_pixel.point.x)
        v_source = float(self.last_pixel.point.y)
        u = u_source
        v = v_source
        pixel_w = float(self.get_parameter('pixel_image_width').value)
        pixel_h = float(self.get_parameter('pixel_image_height').value)
        if pixel_w > 0.0 and pixel_h > 0.0:
            u = u * float(depth.shape[1]) / pixel_w
            v = v * float(depth.shape[0]) / pixel_h
        u = int(round(u))
        v = int(round(v))
        if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
            self.status_pub.publish(String(data=f'PIXEL_OUT_OF_IMAGE u={u} v={v}'))
            return

        method = 'window'
        z = None
        bbox = self.last_bbox
        if bbox is not None:
            bbox_age = time.time() - float(bbox.get('stamp', 0.0) or 0.0)
            src_w = float(bbox.get('image_width', pixel_w) or pixel_w)
            src_h = float(bbox.get('image_height', pixel_h) or pixel_h)
            in_bbox = (
                bbox_age <= float(self.get_parameter('bbox_max_age_sec').value)
                and src_w > 0.0
                and src_h > 0.0
                and float(bbox.get('u0', -1.0)) <= u_source <= float(bbox.get('u1', -1.0))
                and float(bbox.get('v0', -1.0)) <= v_source <= float(bbox.get('v1', -1.0))
            )
            if in_bbox:
                bbox_depth = self._depth_from_bbox(depth, bbox, u_source, v_source)
                if bbox_depth is not None:
                    z, u, v = bbox_depth
                    method = 'bbox_grasp_local'

        if z is None:
            z = self._depth_meters(depth, u, v)
        if z is None:
            self.status_pub.publish(String(data=f'NO_VALID_DEPTH u={u} v={v}'))
            return

        k = self.camera_info.k
        fx, fy = float(k[0]), float(k[4])
        cx, cy = float(k[2]), float(k[5])
        if fx == 0.0 or fy == 0.0:
            self.status_pub.publish(String(data='INVALID_CAMERA_INFO'))
            return

        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = str(self.get_parameter('camera_frame').value)
        point.point.x = (float(u) - cx) * z / fx
        point.point.y = (float(v) - cy) * z / fy
        point.point.z = z
        smoothed = False
        stable_xyz = self._stable_cluster_xyz((float(point.point.x), float(point.point.y), float(point.point.z)))
        if stable_xyz is None:
            return
        point.point.x, point.point.y, point.point.z = stable_xyz
        if bool(self.get_parameter('smoothing_enabled').value):
            raw_xyz = (float(point.point.x), float(point.point.y), float(point.point.z))
            point.point.x, point.point.y, point.point.z, smoothed = self._smooth_xyz(raw_xyz)

        front_z_min = float(self.get_parameter('front_z_min_m').value)
        front_z_max = float(self.get_parameter('front_z_max_m').value)
        if not (front_z_min <= point.point.z <= front_z_max):
            self.status_pub.publish(
                String(
                    data=(
                        f'CAMERA_FRONT_REJECT z={point.point.z:.3f} '
                        f'range=({front_z_min:.3f},{front_z_max:.3f}) '
                        f'camera=({point.point.x:.3f},{point.point.y:.3f},{point.point.z:.3f})'
                    )
                )
            )
            return

        self.point_pub.publish(point)
        self.status_pub.publish(
            String(
                data=(
                    f'camera=({point.point.x:.3f},{point.point.y:.3f},{point.point.z:.3f}) '
                    f'raw_z={z:.3f} depth_topic={active_depth_topic} depth_age={depth_age:.2f}s '
                    f'method={method} smoothed={smoothed}'
                )
            )
        )

    def _smooth_xyz(self, raw_xyz: tuple[float, float, float]) -> tuple[float, float, float, bool]:
        if self.smoothed_xyz is None:
            self.smoothed_xyz = raw_xyz
            return raw_xyz[0], raw_xyz[1], raw_xyz[2], False
        dx = raw_xyz[0] - self.smoothed_xyz[0]
        dy = raw_xyz[1] - self.smoothed_xyz[1]
        dz = raw_xyz[2] - self.smoothed_xyz[2]
        jump = (dx * dx + dy * dy + dz * dz) ** 0.5
        reset_jump = max(0.01, float(self.get_parameter('smoothing_reset_jump_m').value))
        if jump > reset_jump:
            self.smoothed_xyz = raw_xyz
            return raw_xyz[0], raw_xyz[1], raw_xyz[2], False
        alpha = max(0.05, min(1.0, float(self.get_parameter('smoothing_alpha').value)))
        smoothed = (
            self.smoothed_xyz[0] * (1.0 - alpha) + raw_xyz[0] * alpha,
            self.smoothed_xyz[1] * (1.0 - alpha) + raw_xyz[1] * alpha,
            self.smoothed_xyz[2] * (1.0 - alpha) + raw_xyz[2] * alpha,
        )
        self.smoothed_xyz = smoothed
        return smoothed[0], smoothed[1], smoothed[2], True

    def _stable_cluster_xyz(self, raw_xyz: tuple[float, float, float]) -> Optional[tuple[float, float, float]]:
        history_size = max(1, int(self.get_parameter('stable_history_size').value))
        self.depth_history.append(raw_xyz)
        if len(self.depth_history) > history_size:
            self.depth_history = self.depth_history[-history_size:]

        min_samples = max(1, int(self.get_parameter('stable_min_samples').value))
        if len(self.depth_history) < min_samples:
            self.status_pub.publish(
                String(data=f'TARGET_DEPTH_WAIT_STABLE samples={len(self.depth_history)}/{min_samples}')
            )
            return None

        points = np.array(self.depth_history, dtype=np.float32)
        best_indices: Optional[np.ndarray] = None
        radius = max(0.01, float(self.get_parameter('stable_cluster_radius_m').value))
        for point in points:
            distances = np.linalg.norm(points - point, axis=1)
            indices = np.where(distances <= radius)[0]
            if best_indices is None or indices.size > best_indices.size:
                best_indices = indices

        if best_indices is None or int(best_indices.size) < min_samples:
            latest_mm = ','.join(f'{value * 1000.0:.1f}' for value in raw_xyz)
            self.status_pub.publish(
                String(
                    data=(
                        f'TARGET_DEPTH_UNSTABLE samples={0 if best_indices is None else int(best_indices.size)}/'
                        f'{min_samples} latest_mm={latest_mm}'
                    )
                )
            )
            return None

        cluster = points[best_indices]
        median = np.median(cluster, axis=0)
        return float(median[0]), float(median[1]), float(median[2])


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PixelDepthLocator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
