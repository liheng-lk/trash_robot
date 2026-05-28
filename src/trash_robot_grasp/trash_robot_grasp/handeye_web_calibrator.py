import json
import math
import os
import re
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    from roarm_moveit.srv import GetPoseCmd
except Exception:  # pragma: no cover - allows camera-only UI without RoArm package.
    GetPoseCmd = None


DEFAULT_OUTPUT = "/home/sunrise/trash_robot_v3/config/grasp/handeye_point.yaml"
DEFAULT_IMAGE_DIR = "/home/sunrise/trash_robot_v3/handeye_point_images"
DEFAULT_SAMPLE_FILE = "/home/sunrise/trash_robot_v3/runtime/handeye_samples/current_samples.yaml"


def fit_camera_to_arm(camera_pts: np.ndarray, arm_pts: np.ndarray):
    camera_center = camera_pts.mean(axis=0)
    arm_center = arm_pts.mean(axis=0)
    h = (camera_pts - camera_center).T @ (arm_pts - arm_center)
    u, _, vt = np.linalg.svd(h)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = arm_center - rot @ camera_center
    pred = (rot @ camera_pts.T).T + trans
    err = np.linalg.norm(pred - arm_pts, axis=1)
    return rot, trans, pred, err


def rot_to_quat(rot: np.ndarray):
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = [
            (rot[2, 1] - rot[1, 2]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
            (rot[1, 0] - rot[0, 1]) / s,
            0.25 * s,
        ]
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        quat = [
            0.25 * s,
            (rot[0, 1] + rot[1, 0]) / s,
            (rot[0, 2] + rot[2, 0]) / s,
            (rot[2, 1] - rot[1, 2]) / s,
        ]
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        quat = [
            (rot[0, 1] + rot[1, 0]) / s,
            0.25 * s,
            (rot[1, 2] + rot[2, 1]) / s,
            (rot[0, 2] - rot[2, 0]) / s,
        ]
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        quat = [
            (rot[0, 2] + rot[2, 0]) / s,
            (rot[1, 2] + rot[2, 1]) / s,
            0.25 * s,
            (rot[1, 0] - rot[0, 1]) / s,
        ]
    quat = np.array(quat, dtype=np.float64)
    quat /= max(np.linalg.norm(quat), 1e-12)
    return quat


def sample_geometry_quality(points: np.ndarray) -> Dict:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
        return {
            "rank": 0,
            "singular_values_m": [],
            "span_m": [0.0, 0.0, 0.0],
        }

    centered = points - points.mean(axis=0)
    singular_values = np.linalg.svd(centered, compute_uv=False)
    max_singular = float(singular_values[0]) if len(singular_values) else 0.0
    tolerance = max(1e-6, max_singular * 1e-3)
    rank = int(np.sum(singular_values > tolerance))
    span = np.ptp(points, axis=0)
    return {
        "rank": rank,
        "singular_values_m": [float(v) for v in singular_values],
        "span_m": [float(v) for v in span],
    }


class HandeyeWebCalibrator(Node):
    def __init__(self):
        super().__init__("handeye_web_calibrator")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("target_type", "chessboard")
        self.declare_parameter("pattern_cols", 4)
        self.declare_parameter("pattern_rows", 4)
        self.declare_parameter("square_size", 0.005)
        self.declare_parameter("aruco_dict", "DICT_6X6_1000")
        self.declare_parameter("aruco_id", 0)
        self.declare_parameter("marker_size", 0.100)
        self.declare_parameter("output_file", DEFAULT_OUTPUT)
        self.declare_parameter("image_dir", DEFAULT_IMAGE_DIR)
        self.declare_parameter("sample_file", DEFAULT_SAMPLE_FILE)
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8093)
        self.declare_parameter("detect_period_s", 1.0)
        self.declare_parameter("stream_period_s", 0.5)
        self.declare_parameter("jpeg_quality", 45)
        self.declare_parameter("stream_max_width", 480)
        self.declare_parameter("use_chessboard_sb", False)
        self.declare_parameter("detect_scale", 0.25)
        self.declare_parameter("image_accept_period_s", 0.16)
        self.declare_parameter("get_pose_service", "/get_pose_cmd")
        self.declare_parameter("get_pose_timeout_s", 2.0)
        self.declare_parameter("max_detection_age_s", 1.2)
        self.declare_parameter("image_qos_reliability", "reliable")
        self.declare_parameter("video_mode", "webrtc")
        self.declare_parameter("webrtc_url", "http://127.0.0.1:8889/handeye/")
        self.declare_parameter("rtsp_url", "rtsp://127.0.0.1:8554/handeye")
        self.declare_parameter("webrtc_stream_width", 640)
        self.declare_parameter("webrtc_stream_height", 360)
        self.declare_parameter("show_detection_overlay", False)
        self.declare_parameter("enable_internal_stream", False)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.sample_write_lock = threading.Lock()
        self.image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.last_detection: Optional[Dict] = None
        self.cached_jpg: Optional[bytes] = None
        self.cached_jpg_time = 0.0
        self.stream_clients = 0
        self.last_image_accept_time = 0.0
        self.samples = []
        self.last_solve = None
        self.httpd = None
        self.shutdown_event = threading.Event()
        self.detect_thread = None
        self.render_thread = None
        self.last_detect_error = ""
        self.last_detect_method = ""
        self.last_render_error = ""
        self.detect_count = 0
        self.render_count = 0
        self.load_samples()
        self.get_pose_client = None
        if GetPoseCmd is not None:
            self.get_pose_client = self.create_client(
                GetPoseCmd,
                str(self.get_parameter("get_pose_service").value),
            )

        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.image_cb,
            self.image_qos_profile(),
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )
        self.start_workers()
        self.start_web_server()

    def image_qos_profile(self) -> QoSProfile:
        reliability_text = str(self.get_parameter("image_qos_reliability").value).strip().lower()
        reliability = ReliabilityPolicy.RELIABLE
        if reliability_text in ("best_effort", "besteffort", "best-effort"):
            reliability = ReliabilityPolicy.BEST_EFFORT
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=reliability,
        )

    def video_mode(self) -> str:
        mode = str(self.get_parameter("video_mode").value).strip().lower()
        if mode not in ("webrtc", "mjpeg", "snapshot", "none"):
            return "webrtc"
        return mode

    def internal_stream_enabled(self) -> bool:
        mode = self.video_mode()
        return bool(self.get_parameter("enable_internal_stream").value) or mode in ("mjpeg", "snapshot")

    def start_workers(self):
        self.detect_thread = threading.Thread(target=self.detect_loop, name="handeye_detect", daemon=True)
        self.render_thread = threading.Thread(target=self.render_loop, name="handeye_render", daemon=True)
        self.detect_thread.start()
        self.render_thread.start()

    def detect_loop(self):
        while not self.shutdown_event.is_set() and rclpy.ok():
            start = time.time()
            try:
                self.detect_timer()
                with self.lock:
                    self.detect_count += 1
                    self.last_detect_error = ""
            except Exception as exc:
                with self.lock:
                    self.last_detect_error = str(exc)
                self.get_logger().warning(f"handeye target detection failed: {exc}")
            period = max(0.5, float(self.get_parameter("detect_period_s").value))
            elapsed = time.time() - start
            self.shutdown_event.wait(max(0.05, period - elapsed))

    def render_loop(self):
        while not self.shutdown_event.is_set() and rclpy.ok():
            start = time.time()
            try:
                self.render_timer()
                with self.lock:
                    self.render_count += 1
                    self.last_render_error = ""
            except Exception as exc:
                with self.lock:
                    self.last_render_error = str(exc)
                self.get_logger().warning(f"handeye stream render failed: {exc}")
            period = max(0.2, float(self.get_parameter("stream_period_s").value))
            elapsed = time.time() - start
            self.shutdown_event.wait(max(0.03, period - elapsed))

    def sample_file(self) -> Path:
        return Path(str(self.get_parameter("sample_file").value))

    @staticmethod
    def sample_to_yaml(sample: Dict) -> Dict:
        return {
            "camera_m": [float(v) for v in sample["camera_m"]],
            "arm_m": [float(v) for v in sample["arm_m"]],
            "reprojection_error_px": float(sample.get("reprojection_error_px", 0.0)),
            "debug_image": str(sample.get("debug_image", "")),
        }

    def persist_samples(self):
        path = self.sample_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.lock:
            samples = [self.sample_to_yaml(sample) for sample in self.samples]
        payload = {
            "target_type": str(self.get_parameter("target_type").value),
            "output_file": str(self.get_parameter("output_file").value),
            "image_dir": str(self.get_parameter("image_dir").value),
            "samples": samples,
            "updated_at": time.time(),
        }
        tmp = path.with_suffix(path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
        tmp.replace(path)

    def load_samples(self):
        path = self.sample_file()
        if not path.exists():
            return
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            loaded = []
            for item in data.get("samples", []):
                loaded.append(
                    {
                        "camera_m": np.array(item["camera_m"], dtype=np.float64),
                        "arm_m": np.array(item["arm_m"], dtype=np.float64),
                        "reprojection_error_px": float(item.get("reprojection_error_px", 0.0)),
                        "debug_image": str(item.get("debug_image", "")),
                    }
                )
            self.samples = loaded
            if loaded:
                self.get_logger().info(f"loaded {len(loaded)} persisted handeye samples: {path}")
        except Exception as exc:
            self.get_logger().warning(f"failed to load persisted handeye samples {path}: {exc}")

    def image_cb(self, msg: Image):
        now = time.time()
        if now - self.last_image_accept_time < float(self.get_parameter("image_accept_period_s").value):
            return
        self.last_image_accept_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.image = image
                self.last_render_error = ""
        except Exception as exc:
            with self.lock:
                self.last_render_error = f"image_cb failed: {exc}"
            self.get_logger().warning(f"handeye image callback failed: {exc}")

    def camera_info_cb(self, msg: CameraInfo):
        with self.lock:
            self.camera_info = msg

    @staticmethod
    def copy_detection(detection: Optional[Dict]) -> Optional[Dict]:
        if detection is None:
            return None
        copied = dict(detection)
        for key, value in copied.items():
            if isinstance(value, np.ndarray):
                copied[key] = value.copy()
        return copied

    def get_frame(self):
        with self.lock:
            image = None if self.image is None else self.image.copy()
            camera_info = self.camera_info
        return image, camera_info

    def object_points(self):
        cols = int(self.get_parameter("pattern_cols").value)
        rows = int(self.get_parameter("pattern_rows").value)
        square = float(self.get_parameter("square_size").value)
        obj = np.zeros((cols * rows, 3), np.float32)
        obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        obj *= square
        return obj

    def aruco_object_points(self):
        size = float(self.get_parameter("marker_size").value)
        half = size / 2.0
        return np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

    def aruco_dictionary(self):
        dict_name = str(self.get_parameter("aruco_dict").value)
        if not hasattr(cv2, "aruco"):
            return None
        dict_id = getattr(cv2.aruco, dict_name, None)
        if dict_id is None:
            self.get_logger().warn(f"unknown aruco_dict={dict_name}, fallback DICT_6X6_1000")
            dict_id = cv2.aruco.DICT_6X6_1000
        return cv2.aruco.getPredefinedDictionary(dict_id)

    def detect_aruco(self) -> Optional[Dict]:
        image, camera_info = self.get_frame()
        if image is None or camera_info is None:
            return None
        dictionary = self.aruco_dictionary()
        if dictionary is None:
            return None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detect_scale = float(self.get_parameter("detect_scale").value)
        detect_scale = max(0.25, min(1.0, detect_scale))
        detect_gray = gray
        if detect_scale < 0.999:
            detect_gray = cv2.resize(
                gray,
                (
                    max(1, int(gray.shape[1] * detect_scale)),
                    max(1, int(gray.shape[0] * detect_scale)),
                ),
                interpolation=cv2.INTER_AREA,
            )

        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            corners, ids, _ = detector.detectMarkers(detect_gray)
        else:
            params = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(detect_gray, dictionary, parameters=params)

        if ids is None or len(corners) == 0:
            return None

        target_id = int(self.get_parameter("aruco_id").value)
        ids_flat = ids.reshape(-1).astype(int)
        matches = np.where(ids_flat == target_id)[0]
        if len(matches) == 0:
            return None
        idx = int(matches[0])
        marker_corners = corners[idx].astype(np.float32)
        if detect_scale < 0.999:
            marker_corners = marker_corners / detect_scale

        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(camera_info.d, dtype=np.float64).reshape(-1, 1)
        obj = self.aruco_object_points()
        img_pts = marker_corners.reshape(4, 2)
        flags = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE
        ok, rvec, tvec = cv2.solvePnP(obj, img_pts, k, d, flags=flags)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(obj, img_pts, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None

        projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
        reproj = np.linalg.norm(img_pts - projected.reshape(-1, 2), axis=1)
        return {
            "target_type": "aruco",
            "corners": marker_corners,
            "ids": ids,
            "marker_id": target_id,
            "center_camera_m": tvec.reshape(3),
            "reprojection_error_px": float(np.mean(reproj)),
            "time": time.time(),
        }

    def find_chessboard_corners(self, gray: np.ndarray, pattern) -> Tuple[bool, Optional[np.ndarray], str]:
        """Find checkerboard corners with a cheap path and robust fallback.

        The hand-eye board is small in the 640x480 calibration view. A 0.25
        scale image is cheap, but it can erase 4x4 inner corners. Keep that
        fast path, then fall back to full resolution before declaring failure.
        """
        detect_scale = float(self.get_parameter("detect_scale").value)
        detect_scale = max(0.25, min(1.0, detect_scale))

        images = []
        if detect_scale < 0.999:
            scaled = cv2.resize(
                gray,
                (
                    max(1, int(gray.shape[1] * detect_scale)),
                    max(1, int(gray.shape[0] * detect_scale)),
                ),
                interpolation=cv2.INTER_AREA,
            )
            images.append((f"scaled_{detect_scale:.2f}", scaled, detect_scale))
            images.append(("full", gray, 1.0))
        else:
            images.append(("full", gray, 1.0))

        classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        methods = [
            ("classic_fast", classic_flags | cv2.CALIB_CB_FAST_CHECK),
            ("classic", classic_flags),
        ]

        for image_name, detect_gray, scale in images:
            for method_name, flags in methods:
                found, corners = cv2.findChessboardCorners(detect_gray, pattern, flags)
                if found and corners is not None:
                    if scale < 0.999:
                        corners = corners / scale
                    return True, corners, f"{method_name}:{image_name}"

            if hasattr(cv2, "findChessboardCornersSB"):
                try:
                    found, corners = cv2.findChessboardCornersSB(
                        detect_gray,
                        pattern,
                        flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
                    )
                    if found and corners is not None:
                        if scale < 0.999:
                            corners = corners / scale
                        return True, corners, f"sb:{image_name}"
                except cv2.error as exc:
                    self.last_detect_error = f"findChessboardCornersSB failed on {image_name}: {exc}"

        return False, None, ""

    def detect_chessboard(self) -> Optional[Dict]:
        image, camera_info = self.get_frame()
        if image is None or camera_info is None:
            return None

        cols = int(self.get_parameter("pattern_cols").value)
        rows = int(self.get_parameter("pattern_rows").value)
        pattern = (cols, rows)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners, method = self.find_chessboard_corners(gray, pattern)
        if found:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
            self.last_detect_method = method

        if not found or corners is None:
            self.last_detect_method = ""
            return None

        k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(camera_info.d, dtype=np.float64).reshape(-1, 1)
        obj = self.object_points()
        ok, rvec, tvec = cv2.solvePnP(obj, corners, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None

        rot, _ = cv2.Rodrigues(rvec)
        center_camera = (rot @ obj.mean(axis=0).reshape(3, 1) + tvec).reshape(3)
        projected, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
        reproj = np.linalg.norm(corners.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)
        return {
            "target_type": "chessboard",
            "corners": corners,
            "detect_method": method,
            "center_camera_m": center_camera,
            "reprojection_error_px": float(np.mean(reproj)),
            "time": time.time(),
        }

    def detect_target(self) -> Optional[Dict]:
        target_type = str(self.get_parameter("target_type").value).strip().lower()
        if target_type == "aruco":
            return self.detect_aruco()
        if target_type == "chessboard":
            return self.detect_chessboard()
        detection = self.detect_aruco()
        if detection is not None:
            return detection
        return self.detect_chessboard()

    def detect_timer(self):
        detection = self.detect_target()
        with self.lock:
            self.last_detection = detection

    def render_timer(self):
        if not self.internal_stream_enabled():
            return

        with self.lock:
            stream_clients = self.stream_clients
            cache_age = time.time() - self.cached_jpg_time if self.cached_jpg is not None else 999.0
        if stream_clients <= 0 and cache_age < 5.0:
            return

        image = self.annotated_image()
        max_width = int(self.get_parameter("stream_max_width").value)
        if max_width > 0 and image.shape[1] > max_width:
            scale = max_width / float(image.shape[1])
            image = cv2.resize(
                image,
                (max_width, max(1, int(image.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )

        quality = int(self.get_parameter("jpeg_quality").value)
        quality = max(25, min(85, quality))
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return
        with self.lock:
            self.cached_jpg = encoded.tobytes()
            self.cached_jpg_time = time.time()

    def get_cached_jpeg(self) -> Optional[bytes]:
        with self.lock:
            return self.cached_jpg

    def annotated_image(self):
        image, _ = self.get_frame()
        if image is None:
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(image, "NO IMAGE", (230, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return image

        with self.lock:
            detection = self.last_detection
            samples = len(self.samples)

        if detection is not None:
            corners = detection["corners"].reshape(-1, 2)
            if detection.get("target_type") == "aruco" and hasattr(cv2, "aruco"):
                cv2.aruco.drawDetectedMarkers(image, [detection["corners"]], np.array([[detection["marker_id"]]], dtype=np.int32))
            else:
                cols = int(self.get_parameter("pattern_cols").value)
                rows = int(self.get_parameter("pattern_rows").value)
                cv2.drawChessboardCorners(image, (cols, rows), detection["corners"], True)
            center_px = tuple(np.mean(corners, axis=0).astype(int))
            cv2.circle(image, center_px, 6, (0, 255, 255), -1)
            c = detection["center_camera_m"]
            text = f"OK cam=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) err={detection['reprojection_error_px']:.2f}px"
            color = (0, 220, 0)
        else:
            text = "NO TARGET"
            color = (0, 0, 255)
        cv2.rectangle(image, (0, 0), (image.shape[1], 64), (20, 20, 20), -1)
        cv2.putText(image, text, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2)
        cv2.putText(image, f"samples={samples}", (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        return image

    def status(self):
        image, camera_info = self.get_frame()
        with self.lock:
            detection = self.last_detection
            samples = list(self.samples)
            last_solve = self.last_solve
        payload = {
            "image": image is not None,
            "camera_info": camera_info is not None,
            "target": detection is not None,
            "chessboard": detection is not None,
            "target_type": str(self.get_parameter("target_type").value),
            "samples": len(samples),
            "last_solve": last_solve,
            "video_mode": self.video_mode(),
            "webrtc_url": str(self.get_parameter("webrtc_url").value),
            "rtsp_url": str(self.get_parameter("rtsp_url").value),
            "internal_stream_enabled": self.internal_stream_enabled(),
            "webrtc_stream_width": int(self.get_parameter("webrtc_stream_width").value),
            "webrtc_stream_height": int(self.get_parameter("webrtc_stream_height").value),
            "show_detection_overlay": bool(self.get_parameter("show_detection_overlay").value),
            "stream_cached": self.cached_jpg is not None,
            "stream_age_s": time.time() - self.cached_jpg_time if self.cached_jpg is not None else None,
            "stream_clients": self.stream_clients,
            "detect_count": self.detect_count,
            "render_count": self.render_count,
            "last_detect_error": self.last_detect_error,
            "last_detect_method": self.last_detect_method,
            "last_render_error": self.last_render_error,
            "max_detection_age_s": float(self.get_parameter("max_detection_age_s").value),
        }
        if detection is not None:
            c = detection["center_camera_m"]
            corners = detection.get("corners")
            corners_pixel = []
            if corners is not None:
                corners_pixel = [
                    [float(p[0]), float(p[1])]
                    for p in corners.reshape(-1, 2)
                ]
            payload.update(
                {
                    "camera_m": [float(c[0]), float(c[1]), float(c[2])],
                    "reprojection_error_px": detection["reprojection_error_px"],
                    "detection_age_s": time.time() - detection["time"],
                    "detect_method": detection.get("detect_method", ""),
                    "corners_pixel": corners_pixel,
                    "image_width": int(image.shape[1]) if image is not None else None,
                    "image_height": int(image.shape[0]) if image is not None else None,
                }
            )
        return payload

    def add_sample_mm(self, x_mm: float, y_mm: float, z_mm: float):
        with self.sample_write_lock:
            with self.lock:
                detection = self.copy_detection(self.last_detection)
            if detection is None:
                return {"ok": False, "message": "no calibration target"}
            detection_age = time.time() - float(detection.get("time", 0.0))
            max_age = float(self.get_parameter("max_detection_age_s").value)
            if max_age > 0.0 and detection_age > max_age:
                return {
                    "ok": False,
                    "message": f"calibration target is stale: age={detection_age:.2f}s > {max_age:.2f}s",
                }

            arm_m = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0
            camera_m = detection["center_camera_m"].astype(np.float64, copy=True)

            with self.lock:
                sid = len(self.samples) + 1
            image_dir = Path(str(self.get_parameter("image_dir").value)).expanduser()
            image_dir.mkdir(parents=True, exist_ok=True)
            debug = self.annotated_image()
            debug_path = str(image_dir / f"sample_{sid:03d}.png")
            if not cv2.imwrite(debug_path, debug):
                return {"ok": False, "message": f"failed to write debug image: {debug_path}"}

            sample = {
                "camera_m": camera_m,
                "arm_m": arm_m,
                "reprojection_error_px": detection["reprojection_error_px"],
                "debug_image": debug_path,
            }
            with self.lock:
                self.samples.append(sample)
                count = len(self.samples)
            self.persist_samples()

            return {
                "ok": True,
                "samples": count,
                "camera_m": [float(v) for v in camera_m],
                "arm_m": [float(v) for v in arm_m],
                "debug_image": debug_path,
            }

    def get_arm_pose_mm(self):
        timeout_s = float(self.get_parameter("get_pose_timeout_s").value)
        service_name = str(self.get_parameter("get_pose_service").value)
        # The ROS2 CLI has proven more reliable than an in-process rclpy client
        # here because this node also owns an embedded HTTP server and image
        # processing timers. Sampling is low frequency, so the CLI overhead is
        # acceptable and avoids user-visible button hangs.
        return self.get_arm_pose_mm_by_cli(service_name, timeout_s)

    def get_arm_pose_mm_by_cli(self, service_name: str, timeout_s: float):
        actual_timeout_s = max(timeout_s + 4.0, 8.0)
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    service_name,
                    "roarm_moveit/srv/GetPoseCmd",
                    "{}",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=actual_timeout_s,
            )
        except subprocess.TimeoutExpired:
            return {
                "ok": False,
                "message": (
                    f"{service_name} timeout after {actual_timeout_s:.1f}s "
                    f"(configured get_pose_timeout_s={timeout_s:.1f}s)"
                ),
            }
        except Exception as exc:
            return {"ok": False, "message": f"{service_name} call failed: {exc}"}

        text = result.stdout or ""
        if result.returncode != 0:
            return {"ok": False, "message": text[-400:] or f"{service_name} failed"}

        values = {}
        for key in ("x", "y", "z"):
            match = re.search(rf"\b{key}\s*[:=]\s*([-+0-9.eE]+)", text)
            if match:
                values[key] = float(match.group(1))
        if len(values) != 3:
            return {"ok": False, "message": "failed to parse /get_pose_cmd response", "raw": text[-400:]}
        return {
            "ok": True,
            "x_mm": values["x"] * 1000.0,
            "y_mm": values["y"] * 1000.0,
            "z_mm": values["z"] * 1000.0,
            "service": service_name,
            "method": "ros2cli",
        }

    def add_sample_from_current_arm_pose(self):
        pose = self.get_arm_pose_mm()
        if not pose.get("ok"):
            return pose
        result = self.add_sample_mm(pose["x_mm"], pose["y_mm"], pose["z_mm"])
        result["arm_pose_source"] = pose["service"]
        return result

    def pop_sample(self):
        with self.sample_write_lock:
            with self.lock:
                if self.samples:
                    self.samples.pop()
                count = len(self.samples)
            self.persist_samples()
            return {"ok": True, "samples": count}

    def solve(self):
        with self.sample_write_lock:
            with self.lock:
                samples = list(self.samples)
            if len(samples) < 4:
                return {"ok": False, "message": "need at least 4 samples"}

            camera_pts = np.array([s["camera_m"] for s in samples], dtype=np.float64)
            arm_pts = np.array([s["arm_m"] for s in samples], dtype=np.float64)
            camera_quality = sample_geometry_quality(camera_pts)
            arm_quality = sample_geometry_quality(arm_pts)
            if camera_quality["rank"] < 2 or arm_quality["rank"] < 2:
                return {
                    "ok": False,
                    "message": "sample points are nearly collinear; move calibration board through left/right and near/far positions",
                    "camera_sample_quality": camera_quality,
                    "arm_sample_quality": arm_quality,
                }

            rot, trans, pred, err = fit_camera_to_arm(camera_pts, arm_pts)
            quat = rot_to_quat(rot)
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = rot
            matrix[:3, 3] = trans

            output_path = Path(str(self.get_parameter("output_file").value)).expanduser()
            if not output_path.is_absolute():
                output_path = Path.cwd() / output_path
            output_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "description": "arm_point = R * camera_point + t",
                "frames": {
                    "camera_frame": "camera_color_optical_frame",
                    "arm_frame": "roarm_sdk_base",
                },
                "pattern": {
                    "target_type": str(self.get_parameter("target_type").value),
                    "cols": int(self.get_parameter("pattern_cols").value),
                    "rows": int(self.get_parameter("pattern_rows").value),
                    "square_size_m": float(self.get_parameter("square_size").value),
                    "aruco_dict": str(self.get_parameter("aruco_dict").value),
                    "aruco_id": int(self.get_parameter("aruco_id").value),
                    "marker_size_m": float(self.get_parameter("marker_size").value),
                },
                "samples": len(samples),
                "camera_sample_quality": camera_quality,
                "arm_sample_quality": arm_quality,
                "mean_error_m": float(np.mean(err)),
                "max_error_m": float(np.max(err)),
                "translation_m": [float(v) for v in trans],
                "translation_mm": [float(v * 1000.0) for v in trans],
                "quaternion_xyzw": [float(v) for v in quat],
                "rotation_matrix": [[float(v) for v in row] for row in rot],
                "matrix_row_major": [float(v) for v in matrix.reshape(-1)],
                "sample_points": [
                    {
                        "camera_m": [float(v) for v in samples[i]["camera_m"]],
                        "arm_m": [float(v) for v in samples[i]["arm_m"]],
                        "pred_arm_m": [float(v) for v in pred[i]],
                        "error_m": float(err[i]),
                        "debug_image": samples[i]["debug_image"],
                    }
                    for i in range(len(samples))
                ],
            }
            with output_path.open("w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

            result = {
                "ok": True,
                "output_file": str(output_path),
                "samples": len(samples),
                "translation_mm": data["translation_mm"],
                "mean_error_mm": float(np.mean(err) * 1000.0),
                "max_error_mm": float(np.max(err) * 1000.0),
            }
            with self.lock:
                self.last_solve = result
            self.persist_samples()
            return result

    def start_web_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def send_json(self, payload, code=HTTPStatus.OK):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def send_jpeg(self, jpg: bytes):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)

            def read_body(self):
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                if length > 4096:
                    raise ValueError(f"request body too large: {length} bytes > 4096 bytes")
                body = self.rfile.read(length).decode("utf-8")
                ctype = self.headers.get("Content-Type", "")
                if "application/json" in ctype:
                    return json.loads(body)
                parsed = parse_qs(body)
                return {key: value[0] for key, value in parsed.items()}

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    body = node.html_page().encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif parsed.path == "/status":
                    self.send_json(node.status())
                elif parsed.path == "/arm_pose":
                    self.send_json(node.get_arm_pose_mm())
                elif parsed.path == "/snapshot.jpg":
                    if not node.internal_stream_enabled():
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "internal stream disabled")
                        return
                    with node.lock:
                        cache_age = time.time() - node.cached_jpg_time if node.cached_jpg is not None else 999.0
                    if node.get_cached_jpeg() is None or cache_age > 1.5:
                        node.render_timer()
                    jpg = node.get_cached_jpeg()
                    if jpg is None:
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera frame not ready")
                        return
                    self.send_jpeg(jpg)
                elif parsed.path == "/stream.mjpg":
                    if not node.internal_stream_enabled():
                        self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "internal stream disabled")
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    period = float(node.get_parameter("stream_period_s").value)
                    with node.lock:
                        node.stream_clients += 1
                    try:
                        node.render_timer()
                        while rclpy.ok():
                            jpg = node.get_cached_jpeg()
                            if jpg is None:
                                time.sleep(period)
                                continue
                            try:
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                                self.wfile.write(jpg)
                                self.wfile.write(b"\r\n")
                            except (BrokenPipeError, ConnectionResetError):
                                break
                            time.sleep(period)
                    finally:
                        with node.lock:
                            node.stream_clients = max(0, node.stream_clients - 1)
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self):
                parsed = urlparse(self.path)
                try:
                    data = self.read_body()
                    if parsed.path == "/sample":
                        result = node.add_sample_mm(
                            float(data["x_mm"]),
                            float(data["y_mm"]),
                            float(data["z_mm"]),
                        )
                    elif parsed.path == "/sample_auto":
                        result = node.add_sample_from_current_arm_pose()
                    elif parsed.path == "/pop":
                        result = node.pop_sample()
                    elif parsed.path == "/solve":
                        result = node.solve()
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                        return
                    self.send_json(result)
                except Exception as exc:
                    self.send_json({"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST)

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.httpd = ThreadingHTTPServer((host, port), Handler)
        thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(f"handeye web ui: http://{host}:{port}")

    def html_page(self) -> str:
        config = {
            "video_mode": self.video_mode(),
            "webrtc_url": str(self.get_parameter("webrtc_url").value),
            "rtsp_url": str(self.get_parameter("rtsp_url").value),
            "internal_stream_enabled": self.internal_stream_enabled(),
            "webrtc_stream_width": int(self.get_parameter("webrtc_stream_width").value),
            "webrtc_stream_height": int(self.get_parameter("webrtc_stream_height").value),
            "show_detection_overlay": bool(self.get_parameter("show_detection_overlay").value),
        }
        config_json = json.dumps(config, ensure_ascii=False)
        return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Handeye Calibration</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #101418; color: #e8edf2; }}
    header {{ height: 54px; display: flex; align-items: center; padding: 0 18px; background: #161c22; border-bottom: 1px solid #2b343d; }}
    h1 {{ font-size: 18px; margin: 0; font-weight: 650; }}
    main {{ display: grid; grid-template-columns: minmax(0, 1fr) 340px; gap: 16px; padding: 16px; }}
    .video-wrap {{ background: #050607; border: 1px solid #2b343d; min-height: calc(100vh - 86px); display: flex; align-items: center; justify-content: center; }}
    .video-wrap {{ position: relative; }}
    .video-wrap iframe, .video-wrap img {{ width: 100%; height: 100%; border: 0; object-fit: contain; max-height: calc(100vh - 96px); }}
    #overlayCanvas {{ position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; z-index: 3; display: none; }}
    .video-caption {{ position: absolute; left: 12px; top: 10px; right: 12px; color: #aab6c2; font-size: 12px; pointer-events: none; background: rgba(5, 6, 7, 0.65); padding: 6px 8px; border-radius: 4px; }}
    .video-placeholder {{ color: #aab6c2; text-align: center; line-height: 1.8; padding: 20px; }}
    aside {{ display: flex; flex-direction: column; gap: 12px; }}
    section {{ background: #161c22; border: 1px solid #2b343d; padding: 14px; }}
    h2 {{ font-size: 14px; margin: 0 0 10px; color: #aab6c2; font-weight: 650; }}
    .status {{ display: grid; grid-template-columns: 120px 1fr; gap: 7px 10px; font-size: 14px; }}
    .ok {{ color: #66d985; font-weight: 700; }}
    .bad {{ color: #ff6b6b; font-weight: 700; }}
    label {{ display: block; font-size: 12px; color: #aab6c2; margin: 8px 0 4px; }}
    input {{ width: 100%; padding: 9px 10px; border: 1px solid #33414e; background: #0f1419; color: #e8edf2; font-size: 14px; outline: none; }}
    button {{ width: 100%; margin-top: 10px; padding: 10px; border: 1px solid #40515f; background: #24313b; color: #eef4f8; font-size: 14px; cursor: pointer; }}
    button:hover {{ background: #2b3c48; }}
    .row {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }}
    pre {{ margin: 0; white-space: pre-wrap; word-break: break-word; font-size: 12px; color: #d7e1e9; max-height: 220px; overflow: auto; }}
    @media (max-width: 900px) {{ main {{ grid-template-columns: 1fr; }} .video-wrap {{ min-height: 45vh; }} }}
  </style>
</head>
<body>
  <header><h1>Handeye Calibration WebUI</h1></header>
  <main>
    <div class="video-wrap" id="videoWrap"></div>
    <aside>
      <section>
        <h2>状态</h2>
        <div class="status">
          <div>图像</div><div id="image">-</div>
          <div>相机参数</div><div id="cameraInfo">-</div>
          <div>标定板</div><div id="chessboard">-</div>
          <div>视频模式</div><div id="videoMode">-</div>
          <div>相机坐标(m)</div><div id="cameraM">-</div>
          <div>重投影误差</div><div id="err">-</div>
          <div>检测年龄</div><div id="detectAge">-</div>
          <div>样本数</div><div id="samples">0</div>
        </div>
      </section>
      <section>
        <h2>保存样本(mm)</h2>
        <label>X</label><input id="x" inputmode="decimal" placeholder="例如 377.893">
        <label>Y</label><input id="y" inputmode="decimal" placeholder="例如 -1.739">
        <label>Z</label><input id="z" inputmode="decimal" placeholder="例如 -246.470">
        <button onclick="saveSample()">保存当前样本</button>
        <button onclick="saveSampleAuto()">读取机械臂坐标并保存</button>
        <button onclick="readArmPose()">可选：读取当前机械臂坐标</button>
        <div class="row">
          <button onclick="popSample()">删除最后一组</button>
          <button onclick="solve()">求解矩阵</button>
        </div>
      </section>
      <section>
        <h2>输出</h2>
        <pre id="log">等待操作...</pre>
      </section>
    </aside>
  </main>
  <script>
    const videoConfig = {config_json};
    const logEl = document.getElementById('log');
    function renderVideo() {{
      const wrap = document.getElementById('videoWrap');
      const mode = videoConfig.video_mode || 'none';
      const overlay = videoConfig.show_detection_overlay ? '<canvas id="overlayCanvas"></canvas>' : '';
      if (mode === 'webrtc') {{
        wrap.innerHTML = `<iframe title="handeye-webrtc" src="${{videoConfig.webrtc_url}}"></iframe>${{overlay}}<div class="video-caption">WebRTC: ${{videoConfig.webrtc_url}}</div>`;
      }} else if (mode === 'mjpeg') {{
        wrap.innerHTML = `<img id="cameraImage" src="/stream.mjpg" alt="camera stream">${{overlay}}`;
      }} else if (mode === 'snapshot') {{
        wrap.innerHTML = `<img id="cameraImage" alt="camera snapshot">${{overlay}}`;
        const img = document.getElementById('cameraImage');
        img.onerror = () => {{ wrap.innerHTML = '<div class="video-placeholder">snapshot 暂无图像，请检查 /snapshot.jpg 与相机 topic。</div>'; }};
        const update = () => {{ img.src = '/snapshot.jpg?t=' + Date.now(); }};
        update();
        setInterval(update, 500);
      }} else {{
        wrap.innerHTML = '<div class="video-placeholder">视频流未启用<br>当前标定入口为 scripts/start_handeye.sh；如需外部视频流请在 WebUI/RViz Web 中查看相机话题</div>';
      }}
    }}
    function drawOverlay(s) {{
      if (!videoConfig.show_detection_overlay) return;
      const canvas = document.getElementById('overlayCanvas');
      const wrap = document.getElementById('videoWrap');
      if (!canvas || !wrap) return;
      const rect = wrap.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(rect.width * dpr));
      canvas.height = Math.max(1, Math.round(rect.height * dpr));
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, rect.width, rect.height);
      if (!s.target || !s.corners_pixel || !s.corners_pixel.length || !s.image_width || !s.image_height) return;

      const mode = videoConfig.video_mode || 'none';
      let sourceW = Number(s.image_width);
      let sourceH = Number(s.image_height);
      if (mode === 'webrtc') {{
        sourceW = Number(s.webrtc_stream_width || videoConfig.webrtc_stream_width || sourceW);
        sourceH = Number(s.webrtc_stream_height || videoConfig.webrtc_stream_height || sourceH);
      }}
      const scale = Math.min(rect.width / sourceW, rect.height / sourceH);
      const drawW = sourceW * scale;
      const drawH = sourceH * scale;
      const offX = (rect.width - drawW) * 0.5;
      const offY = (rect.height - drawH) * 0.5;
      const imageW = Number(s.image_width);
      const imageH = Number(s.image_height);
      const streamW = sourceW;
      const streamH = sourceH;
      function mapPoint(p) {{
        const sx = (Number(p[0]) / imageW) * streamW;
        const sy = (Number(p[1]) / imageH) * streamH;
        return [offX + sx * scale, offY + sy * scale];
      }}
      const pts = s.corners_pixel.map(mapPoint);
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#00ff66';
      ctx.fillStyle = '#ffdd00';
      ctx.beginPath();
      pts.forEach((p, i) => {{
        if (i === 0) ctx.moveTo(p[0], p[1]);
        else ctx.lineTo(p[0], p[1]);
      }});
      ctx.stroke();
      pts.forEach((p, i) => {{
        ctx.beginPath();
        ctx.fillStyle = i === 0 ? '#ff3b30' : '#ffdd00';
        ctx.arc(p[0], p[1], i === 0 ? 5 : 4, 0, Math.PI * 2);
        ctx.fill();
      }});
      ctx.fillStyle = 'rgba(0,0,0,0.65)';
      ctx.fillRect(offX + 8, offY + 8, 230, 28);
      ctx.fillStyle = '#66ff99';
      ctx.font = '14px monospace';
      ctx.fillText(`target OK  err=${{Number(s.reprojection_error_px || 0).toFixed(2)}}px`, offX + 16, offY + 27);
    }}
    function cls(id, ok, text) {{
      const el = document.getElementById(id);
      el.className = ok ? 'ok' : 'bad';
      el.textContent = text;
    }}
    async function refreshStatus() {{
      const s = await fetch('/status').then(r => r.json());
      cls('image', s.image, s.image ? 'OK' : 'NO');
      cls('cameraInfo', s.camera_info, s.camera_info ? 'OK' : 'NO');
      cls('chessboard', s.target, s.target ? (s.target_type || 'OK') : 'NO');
      document.getElementById('videoMode').textContent = `${{s.video_mode || '-'}} / ${{s.rtsp_url || '-'}}`;
      document.getElementById('samples').textContent = s.samples;
      document.getElementById('cameraM').textContent = s.camera_m ? s.camera_m.map(v => v.toFixed(4)).join(' ') : '-';
      document.getElementById('err').textContent = s.reprojection_error_px !== undefined ? s.reprojection_error_px.toFixed(3) + ' px' : '-';
      const ageEl = document.getElementById('detectAge');
      if (s.detection_age_s !== undefined) {{
        const maxAge = Number(s.max_detection_age_s || 0);
        const stale = maxAge > 0 && s.detection_age_s > maxAge;
        ageEl.className = stale ? 'bad' : 'ok';
        ageEl.textContent = s.detection_age_s.toFixed(2) + ' s' + (maxAge > 0 ? '' : ' / 不限制');
      }} else {{
        ageEl.className = 'bad';
        ageEl.textContent = '-';
      }}
      if (s.last_detect_error || s.last_render_error) {{
        logEl.textContent = JSON.stringify({{detect_error: s.last_detect_error, render_error: s.last_render_error}}, null, 2);
      }}
      if (videoConfig.show_detection_overlay) drawOverlay(s);
    }}
    async function post(path, data = {{}}) {{
      const r = await fetch(path, {{
        method: 'POST',
        headers: {{'Content-Type': 'application/json'}},
        body: JSON.stringify(data)
      }});
      const j = await r.json();
      logEl.textContent = JSON.stringify(j, null, 2);
      await refreshStatus();
    }}
    function saveSample() {{
      post('/sample', {{
        x_mm: document.getElementById('x').value,
        y_mm: document.getElementById('y').value,
        z_mm: document.getElementById('z').value
      }});
    }}
    function saveSampleAuto() {{ post('/sample_auto'); }}
    async function readArmPose() {{
      const j = await fetch('/arm_pose').then(r => r.json());
      logEl.textContent = JSON.stringify(j, null, 2);
      if (j.ok) {{
        document.getElementById('x').value = j.x_mm.toFixed(3);
        document.getElementById('y').value = j.y_mm.toFixed(3);
        document.getElementById('z').value = j.z_mm.toFixed(3);
      }}
    }}
    function popSample() {{ post('/pop'); }}
    function solve() {{ post('/solve'); }}
    renderVideo();
    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>
"""

    def destroy_node(self):
        self.shutdown_event.set()
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
        for thread in (self.detect_thread, self.render_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = HandeyeWebCalibrator()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
