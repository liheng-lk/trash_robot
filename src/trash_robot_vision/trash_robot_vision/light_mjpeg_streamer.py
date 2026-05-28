from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlparse

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Trash Robot Video</title>
  <style>
    html,body{margin:0;background:#071016;color:#dce8f2;font-family:Arial,sans-serif}
    header{padding:10px 14px;background:#111c25;border-bottom:1px solid #263847;font-weight:700}
    main{padding:12px}
    img{display:block;max-width:100%;height:auto;background:#000;border:1px solid #2c4152}
    .hint{margin-top:8px;color:#8fa6b7;font-size:13px}
  </style>
</head>
<body>
  <header>Trash Robot Realtime Video + VLM/YOLO Overlay</header>
  <main>
    <img src="/stream.mjpg">
    <div class="hint">source: ROS image topic + /trash_vlm_result green overlay + /trash_yolo_candidate blue overlay</div>
  </main>
</body>
</html>
"""


class ReuseThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class LightMjpegStreamer(Node):
    def __init__(self) -> None:
        super().__init__('light_mjpeg_streamer')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('vlm_result_topic', '/trash_vlm_result')
        self.declare_parameter('yolo_candidate_topic', '/trash_yolo_candidate')
        self.declare_parameter('camera_point_topic', '/trash_target_point_camera')
        self.declare_parameter('arm_point_topic', '/trash_target_point_arm')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8092)
        self.declare_parameter('jpeg_quality', 55)
        self.declare_parameter('max_width', 640)
        self.declare_parameter('max_fps', 8.0)
        self.declare_parameter('idle_fps', 0.5)
        self.declare_parameter('show_detections', True)
        self.declare_parameter('show_yolo_candidate', True)
        self.declare_parameter('overlay_max_age_sec', 0.8)
        self.declare_parameter('yolo_overlay_max_age_sec', 0.6)
        self.declare_parameter('coord_max_age_sec', 1.0)
        self.declare_parameter('max_clients', 8)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_jpeg: Optional[bytes] = None
        self.last_stamp = 0.0
        self.last_encode_time = 0.0
        self.last_vlm_result: dict = {}
        self.last_vlm_stamp = 0.0
        self.last_yolo_candidate: dict = {}
        self.last_yolo_stamp = 0.0
        self.last_yolo_rx_stamp = 0.0
        self.last_camera_point: Optional[tuple[float, float, float]] = None
        self.last_camera_point_stamp = 0.0
        self.last_arm_point: Optional[tuple[float, float, float]] = None
        self.last_arm_point_stamp = 0.0
        self.active_clients = 0
        self.image_frames = 0
        self.encoded_frames = 0
        self.callback_errors = 0
        self.last_error = ''
        self.last_error_stamp = 0.0
        self.last_error_log_stamp = 0.0

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback,
            image_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('vlm_result_topic').value),
            self.vlm_result_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('yolo_candidate_topic').value),
            self.yolo_candidate_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('camera_point_topic').value),
            self.camera_point_callback,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('arm_point_topic').value),
            self.arm_point_callback,
            10,
        )

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self.httpd = self._make_server(host, port)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'video stream: http://{host}:{port} image={self.get_parameter("image_topic").value} '
            f'vlm={self.get_parameter("vlm_result_topic").value} '
            f'yolo={self.get_parameter("yolo_candidate_topic").value}'
        )

    def _make_server(self, host: str, port: int) -> ThreadingHTTPServer:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args) -> None:
                return

            def do_GET(self) -> None:
                path = urlparse(self.path).path
                if path in ('/', '/index.html'):
                    body = HTML.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path == '/health':
                    body = json.dumps(node.health(), ensure_ascii=False).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if path == '/snapshot.jpg':
                    jpeg = node.get_jpeg()
                    if jpeg is None:
                        self.send_error(503, 'No image yet')
                        return
                    self.send_response(200)
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                    return

                if path == '/stream.mjpg':
                    self.connection.settimeout(2.0)
                    with node.lock:
                        max_clients = max(1, int(node.get_parameter('max_clients').value))
                        if node.active_clients >= max_clients:
                            self.send_error(503, 'Too many video clients')
                            return
                        node.active_clients += 1
                    self.send_response(200)
                    self.send_header('Age', '0')
                    self.send_header('Cache-Control', 'no-cache, private')
                    self.send_header('Pragma', 'no-cache')
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()

                    delay = 1.0 / max(1.0, float(node.get_parameter('max_fps').value))
                    try:
                        while rclpy.ok():
                            jpeg = node.get_jpeg()
                            if jpeg is None:
                                time.sleep(0.1)
                                continue
                            try:
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode('ascii'))
                                self.wfile.write(jpeg)
                                self.wfile.write(b'\r\n')
                                self.wfile.flush()
                                time.sleep(delay)
                            except (BrokenPipeError, ConnectionResetError, OSError):
                                break
                    finally:
                        with node.lock:
                            node.active_clients = max(0, node.active_clients - 1)
                    return

                self.send_error(404)

        return ReuseThreadingHTTPServer((host, port), Handler)

    def get_jpeg(self) -> Optional[bytes]:
        with self.lock:
            return self.last_jpeg

    def health(self) -> dict:
        now = time.time()
        with self.lock:
            last_stamp = float(self.last_stamp or 0.0)
            yolo_stamp = float(self.last_yolo_stamp or 0.0)
            yolo_rx_stamp = float(self.last_yolo_rx_stamp or 0.0)
            return {
                'state': 'has_frame' if self.last_jpeg is not None and now - last_stamp < 3.0 else 'no_frame',
                'image_topic': str(self.get_parameter('image_topic').value),
                'yolo_candidate_topic': str(self.get_parameter('yolo_candidate_topic').value),
                'image_frames': int(self.image_frames),
                'encoded_frames': int(self.encoded_frames),
                'active_clients': int(self.active_clients),
                'max_clients': int(self.get_parameter('max_clients').value),
                'last_frame_age_sec': None if not last_stamp else round(now - last_stamp, 3),
                'last_yolo_age_sec': None if not yolo_rx_stamp else round(now - yolo_rx_stamp, 3),
                'last_yolo_positive_age_sec': None if not yolo_stamp else round(now - yolo_stamp, 3),
                'callback_errors': int(self.callback_errors),
                'last_error': self.last_error,
                'last_error_age_sec': None if not self.last_error_stamp else round(now - self.last_error_stamp, 3),
            }

    def vlm_result_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            with self.lock:
                self.last_vlm_result = data
                self.last_vlm_stamp = time.time()

    def yolo_candidate_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            with self.lock:
                self.last_yolo_rx_stamp = time.time()
                if bool(data.get('has_candidate', False)):
                    self.last_yolo_candidate = data
                    self.last_yolo_stamp = self.last_yolo_rx_stamp

    def camera_point_callback(self, msg: PointStamped) -> None:
        with self.lock:
            self.last_camera_point = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
            self.last_camera_point_stamp = time.time()

    def arm_point_callback(self, msg: PointStamped) -> None:
        with self.lock:
            self.last_arm_point = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
            self.last_arm_point_stamp = time.time()

    def _draw_detections(self, image) -> None:
        if not bool(self.get_parameter('show_detections').value):
            return
        with self.lock:
            vlm_result = dict(self.last_vlm_result)
            vlm_age = time.time() - self.last_vlm_stamp
        max_age = float(self.get_parameter('overlay_max_age_sec').value)
        if vlm_age > max_age or not bool(vlm_result.get('has_target', False)):
            return

        bbox = vlm_result.get('bbox_norm', [])
        if not isinstance(bbox, list) or len(bbox) != 4:
            return
        x1 = int(round(float(bbox[0]) * image.shape[1]))
        y1 = int(round(float(bbox[1]) * image.shape[0]))
        x2 = int(round(float(bbox[2]) * image.shape[1]))
        y2 = int(round(float(bbox[3]) * image.shape[0]))
        x1 = max(0, min(image.shape[1] - 1, x1))
        x2 = max(0, min(image.shape[1] - 1, x2))
        y1 = max(0, min(image.shape[0] - 1, y1))
        y2 = max(0, min(image.shape[0] - 1, y2))
        if x2 <= x1 or y2 <= y1:
            return

        strategy = str(vlm_result.get('grasp_strategy') or '').strip()
        label = (
            f'VLM {vlm_result.get("object_name") or vlm_result.get("trash_label")} '
            f'{float(vlm_result.get("confidence", 0.0)):.2f}'
        )
        if strategy:
            label = f'{label} {strategy}'
        color = (94, 255, 144)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        tw, th = text_size
        label_y1 = max(0, y1 - th - 7)
        cv2.rectangle(image, (x1, label_y1), (min(image.shape[1] - 1, x1 + tw + 8), y1), color, -1)
        cv2.putText(image, label, (x1 + 4, max(th + 1, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (12, 28, 34), 1, cv2.LINE_AA)

        center = vlm_result.get('grasp_point_norm') or vlm_result.get('center_norm') or []
        if isinstance(center, list) and len(center) == 2:
            gu = int(round(float(center[0]) * image.shape[1]))
            gv = int(round(float(center[1]) * image.shape[0]))
            gu = max(0, min(image.shape[1] - 1, gu))
            gv = max(0, min(image.shape[0] - 1, gv))
            cv2.drawMarker(image, (gu, gv), (0, 80, 255), markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2, line_type=cv2.LINE_AA)
            cv2.circle(image, (gu, gv), 6, (0, 80, 255), 2, lineType=cv2.LINE_AA)

    def _draw_yolo_candidate(self, image) -> None:
        if not bool(self.get_parameter('show_yolo_candidate').value):
            return
        with self.lock:
            candidate = dict(self.last_yolo_candidate)
            age = time.time() - self.last_yolo_stamp
        max_age = float(self.get_parameter('yolo_overlay_max_age_sec').value)
        if age > max_age or not bool(candidate.get('has_candidate', False)):
            return

        bbox = candidate.get('bbox_norm', [])
        if not isinstance(bbox, list) or len(bbox) != 4:
            return
        x1 = int(round(float(bbox[0]) * image.shape[1]))
        y1 = int(round(float(bbox[1]) * image.shape[0]))
        x2 = int(round(float(bbox[2]) * image.shape[1]))
        y2 = int(round(float(bbox[3]) * image.shape[0]))
        x1 = max(0, min(image.shape[1] - 1, x1))
        x2 = max(0, min(image.shape[1] - 1, x2))
        y1 = max(0, min(image.shape[0] - 1, y1))
        y2 = max(0, min(image.shape[0] - 1, y2))
        if x2 <= x1 or y2 <= y1:
            return

        label = (
            f'YOLO {candidate.get("class_name") or candidate.get("object_name") or candidate.get("trash_label")} '
            f'{float(candidate.get("confidence", 0.0)):.2f}'
        )
        color = (255, 0, 0)
        text_color = (245, 250, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        tw, th = text_size
        label_y0 = y2 + 4
        label_y1 = label_y0 + th + 8
        if label_y1 >= image.shape[0]:
            label_y1 = max(0, y1 - 2)
            label_y0 = max(0, label_y1 - th - 8)
        cv2.rectangle(image, (x1, label_y0), (min(image.shape[1] - 1, x1 + tw + 8), label_y1), color, -1)
        cv2.putText(image, label, (x1 + 4, label_y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_color, 1, cv2.LINE_AA)

        center = candidate.get('center_norm') or []
        if isinstance(center, list) and len(center) == 2:
            gu = int(round(float(center[0]) * image.shape[1]))
            gv = int(round(float(center[1]) * image.shape[0]))
            gu = max(0, min(image.shape[1] - 1, gu))
            gv = max(0, min(image.shape[0] - 1, gv))
            cv2.drawMarker(image, (gu, gv), color, markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2, line_type=cv2.LINE_AA)
            cv2.circle(image, (gu, gv), 5, color, 2, lineType=cv2.LINE_AA)

    def _draw_coord_panel(self, image) -> None:
        now = time.time()
        max_age = float(self.get_parameter('coord_max_age_sec').value)
        with self.lock:
            camera_point = self.last_camera_point
            camera_age = now - self.last_camera_point_stamp
            arm_point = self.last_arm_point
            arm_age = now - self.last_arm_point_stamp

        lines = []
        if camera_point is not None and camera_age <= max_age:
            x, y, z = camera_point
            lines.append(f'camera mm: {x * 1000:+.0f} {y * 1000:+.0f} {z * 1000:+.0f}')
        else:
            lines.append('camera mm: --')

        if arm_point is not None and arm_age <= max_age:
            x, y, z = arm_point
            lines.append(f'arm mm: {x * 1000:+.0f} {y * 1000:+.0f} {z * 1000:+.0f}')
        else:
            lines.append('arm mm: --')

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.46
        thickness = 1
        pad = 8
        line_h = 19
        widths = [cv2.getTextSize(line, font, scale, thickness)[0][0] for line in lines]
        panel_w = max(widths) + pad * 2
        panel_h = line_h * len(lines) + pad
        x0, y0 = 8, 8
        overlay = image.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (9, 19, 25), -1)
        cv2.addWeighted(overlay, 0.72, image, 0.28, 0, image)
        cv2.rectangle(image, (x0, y0), (x0 + panel_w, y0 + panel_h), (255, 220, 0), 1)

        for idx, line in enumerate(lines):
            cv2.putText(image, line, (x0 + pad, y0 + pad + 13 + idx * line_h), font, scale, (230, 246, 255), thickness, cv2.LINE_AA)

    def image_callback(self, msg: Image) -> None:
        now = time.time()
        with self.lock:
            self.image_frames += 1
            has_clients = self.active_clients > 0
        try:
            active_fps = max(1.0, float(self.get_parameter('max_fps').value))
            idle_fps = max(0.0, float(self.get_parameter('idle_fps').value))
            target_fps = active_fps if has_clients else idle_fps
            if target_fps <= 0.0:
                return
            if now - self.last_encode_time < (1.0 / target_fps):
                return
            self.last_encode_time = now

            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            max_width = int(self.get_parameter('max_width').value)
            if max_width > 0 and image.shape[1] > max_width:
                scale = max_width / float(image.shape[1])
                image = cv2.resize(image, (max_width, int(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)

            self._draw_detections(image)
            self._draw_yolo_candidate(image)
            self._draw_coord_panel(image)

            quality = int(self.get_parameter('jpeg_quality').value)
            ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if not ok:
                raise RuntimeError('cv2.imencode returned false')

            with self.lock:
                self.last_jpeg = encoded.tobytes()
                self.last_stamp = now
                self.encoded_frames += 1
        except Exception as exc:  # noqa: BLE001
            should_log = now - self.last_error_log_stamp > 2.0
            with self.lock:
                self.callback_errors += 1
                self.last_error = f'{type(exc).__name__}: {exc}'
                self.last_error_stamp = time.time()
                if should_log:
                    self.last_error_log_stamp = now
            if should_log:
                self.get_logger().warning(f'image encode failed: {exc}')

    def destroy_node(self) -> bool:
        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LightMjpegStreamer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
