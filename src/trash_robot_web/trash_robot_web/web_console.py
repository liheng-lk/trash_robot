from __future__ import annotations

import json
import math
import os
import re
import shutil
import socket
import subprocess
import threading
import time
import asyncio
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

def ensure_robot_dds_environment() -> None:
    root = os.environ.get('TRASH_ROBOT_ROOT') or '/home/sunrise/trash_robot_v3'
    os.environ['TRASH_ROBOT_ROOT'] = root
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('TRASH_ROS_DOMAIN_ID') or '1'
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = f'file://{root}/config/dds/cyclonedds_unicast.xml'
    for key in (
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'RMW_FASTRTPS_USE_QOS_FROM_XML',
        'ROS_DISABLE_LOANED_MESSAGES',
    ):
        os.environ.pop(key, None)


ensure_robot_dds_environment()

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
import yaml

try:
    from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse
    from fastapi.staticfiles import StaticFiles
    import uvicorn
except Exception as exc:  # pragma: no cover - reported at runtime on the robot
    FastAPI = None  # type: ignore[assignment]
    _FASTAPI_IMPORT_ERROR = exc


def yaw_to_quaternion(yaw: float) -> dict[str, float]:
    half = yaw * 0.5
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(half), 'w': math.cos(half)}


def quaternion_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= math.tau
    while yaw <= -math.pi:
        yaw += math.tau
    return yaw


def duration_to_sec(msg: Any) -> float:
    return float(getattr(msg, 'sec', 0)) + float(getattr(msg, 'nanosec', 0)) / 1e9


def goal_status_text(status: int | None) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return mapping.get(int(status), str(status)) if status is not None else 'UNKNOWN'


class WebConsoleNode(Node):
    def __init__(self) -> None:
        super().__init__('trash_robot_web_bridge')
        self.declare_parameter('project_root', '/home/sunrise/trash_robot_v3')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8095)
        self.declare_parameter('video_url', 'http://192.168.1.121:8092/stream.mjpg')
        self.declare_parameter('manual_cmd_topic', '/cmd_vel')
        self.declare_parameter('manual_max_linear', 0.10)
        self.declare_parameter('manual_max_angular', 0.35)

        self.root = Path(str(self.get_parameter('project_root').value))
        self.route_file = self.root / 'config' / 'mission' / 'patrol_routes.yaml'
        self.vlm_config_file = self.root / 'config' / 'perception' / 'vlm_trash_classifier.yaml'
        self.vlm_registry_file = self.root / 'config' / 'perception' / 'vlm_provider_registry.yaml'
        self.maps_dir = self.root / 'maps'
        self.video_url = str(self.get_parameter('video_url').value)
        self.manual_cmd_topic = str(self.get_parameter('manual_cmd_topic').value)
        self.manual_max_linear = float(self.get_parameter('manual_max_linear').value)
        self.manual_max_angular = float(self.get_parameter('manual_max_angular').value)

        self.state_lock = threading.Lock()
        self.latest_manager: dict[str, Any] = {}
        self.latest_mission: dict[str, Any] = {}
        self.latest_target: dict[str, Any] = {}
        self.latest_grasp_runtime: dict[str, Any] = {}
        self.latest_amcl: PoseWithCovarianceStamped | None = None
        self.latest_map: OccupancyGrid | None = None
        self.nav2d_lock = threading.Lock()
        self.nav2d_seq = {
            'map': 0,
            'map_marker': 0,
            'global_costmap': 0,
            'local_costmap': 0,
            'global_path': 0,
            'local_path': 0,
            'scan': 0,
            'scan_depth': 0,
            'footprint': 0,
            'nav_status': 0,
            'goal': 0,
        }
        self.latest_map_marker: dict[str, Any] | None = None
        self.latest_global_costmap: OccupancyGrid | None = None
        self.latest_local_costmap: OccupancyGrid | None = None
        self.latest_global_path: NavPath | None = None
        self.latest_local_path: NavPath | None = None
        self.latest_scan_points: dict[str, Any] = {}
        self.latest_scan_depth_points: dict[str, Any] = {}
        self.latest_footprint: dict[str, Any] | None = None
        self.latest_nav_status: dict[str, Any] = {}
        self.latest_goal: dict[str, Any] | None = None
        self.nav2d_last_error = ''
        self.last_scan_process: dict[str, float] = {}
        self.manual_enabled_until = 0.0
        self.status_cache_lock = threading.Lock()
        self.status_cache: dict[str, Any] | None = None
        self.status_cache_stamp = 0.0
        self.status_cache_ttl_sec = 1.2

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(String, '/trash_system_status', self.manager_status_cb, 10)
        self.create_subscription(String, '/trash_robot_v3/manager/system_state', self.manager_status_cb, 10)
        self.create_subscription(String, '/trash_mission_status', self.mission_status_cb, 10)
        self.create_subscription(String, '/trash_target_label', self.target_label_cb, 10)
        self.create_subscription(PointStamped, '/trash_target_point_arm', self.arm_target_cb, 10)
        self.create_subscription(PointStamped, '/trash_target_camera_point', self.camera_target_cb, 10)
        self.create_subscription(String, '/trash_grasp_plan', self.grasp_plan_cb, 10)
        self.create_subscription(String, '/trash_vlm_result', self.vlm_result_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, latched_qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched_qos)
        self.create_subscription(Marker, '/map_occupied_marker', self.map_marker_cb, 10)
        self.create_subscription(MarkerArray, '/map_occupied_marker_array', self.map_marker_array_cb, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_cb, latched_qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_cb, latched_qos)
        self.create_subscription(NavPath, '/plan', self.global_path_cb, 10)
        self.create_subscription(NavPath, '/local_plan', self.local_path_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.create_subscription(LaserScan, '/scan_depth', self.scan_depth_cb, sensor_qos)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self.footprint_cb, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.manual_cmd_topic, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cancel_nav_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.trigger_clients: dict[str, Any] = {}
        for key, service in {
            'start_base': '/trash_manager/start_base',
            'stop_base': '/trash_manager/stop_base',
            'start_camera': '/trash_manager/start_camera',
            'stop_camera': '/trash_manager/stop_camera',
            'start_navigation': '/trash_manager/start_navigation',
            'stop_navigation': '/trash_manager/stop_navigation',
            'start_video': '/trash_manager/start_video',
            'stop_video': '/trash_manager/stop_video',
            'start_arm': '/trash_manager/start_arm',
            'stop_arm': '/trash_manager/stop_arm',
            'start_grasp_dry': '/trash_manager/start_grasp_vlm_dry',
            'start_grasp_live': '/trash_manager/start_grasp_vlm_live',
            'stop_grasp': '/trash_manager/stop_grasp',
            'stop_all': '/trash_manager/stop_all',
            'estop': '/trash_manager/estop_trigger',
            'estop_reset': '/trash_manager/estop_reset',
            'safety_stop': '/trash_safety/stop',
            'patrol_start': '/trash_mission/start_patrol',
            'patrol_pause': '/trash_mission/pause_patrol',
            'patrol_resume': '/trash_mission/resume_patrol',
            'patrol_stop': '/trash_mission/stop_patrol',
            'patrol_reload': '/trash_mission/reload_route',
            'grasp_once': '/trash_mission/grasp_once',
            'direct_grasp_once': '/trash_grasp_once',
            'vlm_refresh': '/trash_vlm/refresh',
        }.items():
            self.trigger_clients[key] = self.create_client(Trigger, service)
        self.set_grasp_client = self.create_client(SetBool, '/trash_mission/set_grasp_enabled')
        self.grasper_param_client = self.create_client(GetParameters, '/roarm_sort_grasper/get_parameters')
        self.timer = self.create_timer(0.2, self.manual_deadman_tick)
        self.get_logger().info(f'trash_robot_web_bridge ready root={self.root}')

    def manager_status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {'raw': msg.data}
        with self.state_lock:
            self.latest_manager = data

    def mission_status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {'raw': msg.data}
        with self.state_lock:
            self.latest_mission = data

    def target_label_cb(self, msg: String) -> None:
        with self.state_lock:
            self.latest_target = {'label': msg.data, 'stamp': time.time()}

    @staticmethod
    def point_msg_snapshot(msg: PointStamped) -> dict[str, Any]:
        return {
            'stamp': time.time(),
            'frame_id': msg.header.frame_id,
            'point': [
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
            ],
        }

    def arm_target_cb(self, msg: PointStamped) -> None:
        with self.state_lock:
            self.latest_grasp_runtime['arm_target'] = self.point_msg_snapshot(msg)

    def camera_target_cb(self, msg: PointStamped) -> None:
        with self.state_lock:
            self.latest_grasp_runtime['camera_target'] = self.point_msg_snapshot(msg)

    def grasp_plan_cb(self, msg: String) -> None:
        with self.state_lock:
            self.latest_grasp_runtime['grasp_plan'] = {'stamp': time.time(), 'data': msg.data}

    def vlm_result_cb(self, msg: String) -> None:
        with self.state_lock:
            self.latest_grasp_runtime['vlm_result'] = {'stamp': time.time(), 'data': msg.data}

    def amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        with self.state_lock:
            self.latest_amcl = msg

    def map_cb(self, msg: OccupancyGrid) -> None:
        with self.state_lock:
            self.latest_map = msg
        self.bump_nav2d_seq('map')

    def map_marker_cb(self, msg: Marker) -> None:
        payload = self.marker_payload([msg])
        with self.nav2d_lock:
            self.latest_map_marker = payload
        self.bump_nav2d_seq('map_marker')

    def map_marker_array_cb(self, msg: MarkerArray) -> None:
        if not msg.markers:
            return
        payload = self.marker_payload(list(msg.markers))
        with self.nav2d_lock:
            self.latest_map_marker = payload
        self.bump_nav2d_seq('map_marker')

    def bump_nav2d_seq(self, key: str) -> None:
        with self.nav2d_lock:
            self.nav2d_seq[key] = int(self.nav2d_seq.get(key, 0)) + 1

    def global_costmap_cb(self, msg: OccupancyGrid) -> None:
        with self.nav2d_lock:
            self.latest_global_costmap = msg
        self.bump_nav2d_seq('global_costmap')

    def local_costmap_cb(self, msg: OccupancyGrid) -> None:
        with self.nav2d_lock:
            self.latest_local_costmap = msg
        self.bump_nav2d_seq('local_costmap')

    def global_path_cb(self, msg: NavPath) -> None:
        with self.nav2d_lock:
            self.latest_global_path = msg
        self.bump_nav2d_seq('global_path')

    def local_path_cb(self, msg: NavPath) -> None:
        with self.nav2d_lock:
            self.latest_local_path = msg
        self.bump_nav2d_seq('local_path')

    def scan_cb(self, msg: LaserScan) -> None:
        self.scan_points_cb('scan', msg)

    def scan_depth_cb(self, msg: LaserScan) -> None:
        self.scan_points_cb('scan_depth', msg)

    def scan_points_cb(self, key: str, msg: LaserScan) -> None:
        now = time.monotonic()
        if now - self.last_scan_process.get(key, 0.0) < 0.18:
            return
        self.last_scan_process[key] = now
        payload = self.scan_payload(msg)
        with self.nav2d_lock:
            if key == 'scan_depth':
                self.latest_scan_depth_points = payload
            else:
                self.latest_scan_points = payload
        self.bump_nav2d_seq(key)

    def footprint_cb(self, msg: PolygonStamped) -> None:
        source_frame = msg.header.frame_id or 'map'
        points, tf_ok, detail = self.points_to_map(
            source_frame,
            [[float(p.x), float(p.y)] for p in msg.polygon.points],
        )
        with self.nav2d_lock:
            self.latest_footprint = {
                'frame_id': 'map' if tf_ok else source_frame,
                'source_frame_id': source_frame,
                'stamp': self.stamp_to_float(msg.header.stamp),
                'points': points,
                'tf_ok': tf_ok,
                'tf_detail': detail,
            }
        self.bump_nav2d_seq('footprint')

    def nav_status_cb(self, msg: GoalStatusArray) -> None:
        active_codes = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        latest_status = None
        if msg.status_list:
            latest_status = int(msg.status_list[-1].status)
        with self.nav2d_lock:
            self.latest_nav_status = {
                'stamp': time.time(),
                'active': any(int(item.status) in active_codes for item in msg.status_list),
                'latest_status': latest_status,
                'latest_status_text': goal_status_text(latest_status),
                'count': len(msg.status_list),
            }
        self.bump_nav2d_seq('nav_status')

    def manual_deadman_tick(self) -> None:
        if self.manual_enabled_until and time.monotonic() > self.manual_enabled_until:
            self.manual_enabled_until = 0.0
            self.publish_zero('manual deadman timeout')

    def service_ready(self, key: str) -> bool:
        client = self.trigger_clients.get(key)
        return bool(client and client.service_is_ready())

    def manager_ready_detail(self) -> tuple[bool, str]:
        nodes = self.node_names_snapshot()
        manager_node = self.node_exists(nodes, '/trash_robot_manager')
        required = {
            'start_camera': self.service_ready('start_camera'),
            'start_video': self.service_ready('start_video'),
            'start_grasp_live': self.service_ready('start_grasp_live'),
        }
        return manager_node and all(required.values()), f'manager_node={manager_node} services={required}'

    def start_manager_backend(self) -> str:
        script = self.root / 'scripts' / 'start_web_console.sh'
        if not script.exists():
            return f'manager auto-start script missing: {script}'
        env = os.environ.copy()
        env['TRASH_ROBOT_ROOT'] = str(self.root)
        try:
            result = subprocess.run(
                [str(script), 'start'],
                cwd=str(self.root),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=35.0,
                check=False,
            )
            detail = '; '.join((result.stdout or '').strip().splitlines())
            return f'manager auto-start rc={result.returncode}: {detail[-260:]}'
        except Exception as exc:  # noqa: BLE001 - report through WebUI API
            return f'manager auto-start failed: {exc}'

    def ensure_manager_ready(self, timeout_sec: float = 10.0) -> tuple[bool, str]:
        ready, detail = self.manager_ready_detail()
        if ready:
            return True, 'manager ready'
        start_detail = self.start_manager_backend()
        deadline = time.monotonic() + timeout_sec
        last_detail = detail
        while time.monotonic() < deadline:
            ready, last_detail = self.manager_ready_detail()
            if ready:
                return True, f'manager ready; {start_detail}'
            time.sleep(0.3)
        return False, f'manager not ready after auto-start: {last_detail}; {start_detail}'

    def video_stream_ready(self, timeout_sec: float = 0.6) -> bool:
        parsed = urlparse(self.video_url)
        host = parsed.hostname or '127.0.0.1'
        if host in {'0.0.0.0', '192.168.1.121'}:
            host = '127.0.0.1'
        port = parsed.port or (443 if parsed.scheme == 'https' else 80)
        try:
            with socket.create_connection((host, port), timeout=timeout_sec):
                return True
        except OSError:
            return False

    def ensure_video_ready(self) -> tuple[bool, str]:
        if self.video_stream_ready():
            return True, 'video stream already ready'
        if not self.service_ready('start_video'):
            return False, 'video service is not ready; manager is offline or not in the same DDS environment'
        start_ok, start_msg = self.call_trigger('start_video', timeout_sec=90.0)
        if not start_ok:
            return False, f'start_video failed: {start_msg}'
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if self.video_stream_ready():
                return True, f'video stream ready; {start_msg}'
            time.sleep(0.5)
        return False, f'video stream not reachable after start: {self.video_url}; {start_msg}'

    @staticmethod
    def normalize_node_name(name: str, namespace: str = '') -> str:
        node = str(name or '').strip()
        ns = str(namespace or '').strip()
        if not node:
            return ''
        if node.startswith('/'):
            full = node
        elif ns and ns != '/':
            full = f'/{ns.strip("/")}/{node}'
        else:
            full = f'/{node}'
        return re.sub(r'/+', '/', full)

    def node_names_snapshot(self) -> list[str]:
        names: set[str] = set()
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                full = self.normalize_node_name(name, namespace)
                if full:
                    names.add(full)
        except Exception as exc:  # noqa: BLE001 - status must stay best-effort
            self.get_logger().debug(f'node graph snapshot failed: {exc}')
        return sorted(names)

    @staticmethod
    def node_exists(nodes: list[str] | set[str], *candidates: str) -> bool:
        node_set = set(nodes)
        for candidate in candidates:
            normalized = WebConsoleNode.normalize_node_name(candidate)
            if normalized in node_set:
                return True
        return False

    def topic_publishers(self, topic: str) -> int:
        return len(self.get_publishers_info_by_topic(topic))

    def topic_subscribers(self, topic: str) -> int:
        return len(self.get_subscriptions_info_by_topic(topic))

    def build_readiness(
        self,
        topics: dict[str, int],
        services: dict[str, bool],
        manager: dict[str, Any],
        nodes: list[str],
    ) -> dict[str, Any]:
        components = manager.get('components') if isinstance(manager.get('components'), dict) else {}
        has_camera_color = topics.get('/camera/color', 0) > 0
        has_depth = topics.get('/camera/aligned_depth', 0) > 0 or topics.get('/camera/depth', 0) > 0
        has_nav_goal = self.topic_subscribers('/goal_pose') > 0
        has_initial_pose = self.topic_subscribers('/initialpose') > 0
        cmd_vel_pubs = self.topic_publishers('/cmd_vel')
        cmd_vel_subs = self.topic_subscribers('/cmd_vel')
        manual_pubs = self.topic_publishers(self.manual_cmd_topic)
        manual_subs = self.topic_subscribers(self.manual_cmd_topic)
        return {
            'base_ready': bool(
                topics.get('/odom', 0) > 0
                and (components.get('base') or self.node_exists(nodes, '/serial_base_node', '/base_driver'))
            ),
            'lidar_ready': topics.get('/scan', 0) > 0,
            'camera_ready': bool(has_camera_color and has_depth),
            'depth_ready': bool(has_depth),
            'depth_avoid_ready': topics.get('/scan_depth', 0) > 0,
            'navigation_ready': bool(topics.get('/map', 0) > 0 and has_nav_goal),
            'initial_pose_ready': bool(topics.get('/map', 0) > 0 and has_initial_pose),
            'mission_ready': bool(
                services.get('patrol_start')
                and self.node_exists(nodes, '/trash_mission_supervisor')
            ),
            'vlm_ready': bool(
                services.get('vlm_refresh')
                and self.node_exists(nodes, '/vlm_trash_classifier')
            ),
            'direct_grasp_ready': bool(
                services.get('direct_grasp_once')
                and self.node_exists(nodes, '/roarm_sort_grasper')
            ),
            'mission_grasp_ready': bool(
                services.get('grasp_once')
                and self.node_exists(nodes, '/trash_mission_supervisor')
            ),
            'video_ready': bool(components.get('video')),
            'arm_ready': bool(components.get('arm')),
            'manager_ready': self.node_exists(nodes, '/trash_robot_manager'),
            'web_ready': self.node_exists(nodes, '/trash_robot_web_bridge'),
            'cmd_vel': {
                'topic': '/cmd_vel',
                'publishers': cmd_vel_pubs,
                'subscribers': cmd_vel_subs,
            },
            'manual_cmd': {
                'topic': self.manual_cmd_topic,
                'publishers': manual_pubs,
                'subscribers': manual_subs,
            },
            'notes': [
                'readiness uses publisher counts and expected node names; topic/service names alone are not enough',
            ],
        }

    def set_nav2d_error(self, message: str) -> None:
        self.nav2d_last_error = str(message)[-220:]

    def transform_xy_yaw_to_map(
        self,
        frame_id: str,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> tuple[float, float, float, bool, str]:
        source = (frame_id or 'map').lstrip('/')
        if source == 'map':
            return float(x), float(y), normalize_yaw(float(yaw)), True, 'map'
        try:
            tf = self.tf_buffer.lookup_transform('map', source, Time())
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            tf_yaw = quaternion_to_yaw(tf.transform.rotation)
            cos_yaw = math.cos(tf_yaw)
            sin_yaw = math.sin(tf_yaw)
            mx = tx + float(x) * cos_yaw - float(y) * sin_yaw
            my = ty + float(x) * sin_yaw + float(y) * cos_yaw
            self.nav2d_last_error = ''
            return mx, my, normalize_yaw(tf_yaw + float(yaw)), True, f'tf map->{source}'
        except Exception as exc:  # noqa: BLE001 - UI health should carry the transform error
            detail = f'tf map->{source} failed: {str(exc)[-160:]}'
            self.set_nav2d_error(detail)
            return float(x), float(y), normalize_yaw(float(yaw)), False, detail

    def points_to_map(self, frame_id: str, points: list[list[float]]) -> tuple[list[list[float]], bool, str]:
        source = (frame_id or 'map').lstrip('/')
        if source == 'map':
            return [[float(x), float(y)] for x, y in points], True, 'map'
        try:
            tf = self.tf_buffer.lookup_transform('map', source, Time())
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            yaw = quaternion_to_yaw(tf.transform.rotation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            transformed = [
                [tx + float(x) * cos_yaw - float(y) * sin_yaw, ty + float(x) * sin_yaw + float(y) * cos_yaw]
                for x, y in points
            ]
            self.nav2d_last_error = ''
            return transformed, True, f'tf map->{source}'
        except Exception as exc:  # noqa: BLE001
            detail = f'tf map->{source} failed: {str(exc)[-160:]}'
            self.set_nav2d_error(detail)
            return [[float(x), float(y)] for x, y in points], False, detail

    def robot_tf_pose(self) -> dict[str, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return {
                'x': float(tf.transform.translation.x),
                'y': float(tf.transform.translation.y),
                'yaw_deg': math.degrees(quaternion_to_yaw(tf.transform.rotation)),
                'source': 'tf:map->base_link',
            }
        except Exception:
            return None

    def amcl_pose_snapshot(self) -> dict[str, float] | None:
        with self.state_lock:
            msg = self.latest_amcl
        if msg is None:
            return None
        pose = msg.pose.pose
        return {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw_deg': math.degrees(quaternion_to_yaw(pose.orientation)),
            'source': '/amcl_pose',
        }

    def current_pose(self) -> dict[str, float] | None:
        return self.robot_tf_pose() or self.amcl_pose_snapshot()

    def status_snapshot(self) -> dict[str, Any]:
        mono_now = time.monotonic()
        with self.status_cache_lock:
            if self.status_cache is not None and mono_now - self.status_cache_stamp < self.status_cache_ttl_sec:
                return self.status_cache
        with self.state_lock:
            manager = dict(self.latest_manager)
            mission = dict(self.latest_mission)
            target = dict(self.latest_target)
            grasp_runtime = json.loads(json.dumps(self.latest_grasp_runtime))
            map_msg = self.latest_map
        now = time.time()
        for item in grasp_runtime.values():
            if isinstance(item, dict) and isinstance(item.get('stamp'), (int, float)):
                item['age_sec'] = round(now - float(item['stamp']), 3)
        topics = {
            '/scan': self.topic_publishers('/scan'),
            '/odom': self.topic_publishers('/odom'),
            '/map': self.topic_publishers('/map'),
            '/amcl_pose': self.topic_publishers('/amcl_pose'),
            '/camera/color': self.topic_publishers('/camera/camera/color/image_raw'),
            '/camera/depth': self.topic_publishers('/camera/camera/depth/image_rect_raw'),
            '/camera/aligned_depth': self.topic_publishers('/camera/camera/aligned_depth_to_color/image_raw'),
            '/scan_depth': self.topic_publishers('/scan_depth'),
            '/trash_mission_status': self.topic_publishers('/trash_mission_status'),
            '/trash_vlm_result': self.topic_publishers('/trash_vlm_result'),
            '/trash_grasp_plan': self.topic_publishers('/trash_grasp_plan'),
        }
        services = {key: self.service_ready(key) for key in self.trigger_clients}
        services['set_grasp_enabled'] = self.set_grasp_client.service_is_ready()
        nodes = self.node_names_snapshot()
        readiness = self.build_readiness(topics, services, manager, nodes)
        snapshot = {
            'ok': True,
            'stamp': time.time(),
            'root': str(self.root),
            'video_url': self.video_url,
            'pose': self.current_pose(),
            'map': None if map_msg is None else {
                'width': map_msg.info.width,
                'height': map_msg.info.height,
                'resolution': map_msg.info.resolution,
                'frame_id': map_msg.header.frame_id,
            },
            'topics': topics,
            'services': services,
            'nodes': nodes,
            'readiness': readiness,
            'manager': manager,
            'mission': mission,
            'target': target,
            'grasp_runtime': grasp_runtime,
            'manual': {
                'cmd_topic': self.manual_cmd_topic,
                'max_linear': self.manual_max_linear,
                'max_angular': self.manual_max_angular,
                'enabled': bool(self.manual_enabled_until and time.monotonic() < self.manual_enabled_until),
            },
        }
        with self.status_cache_lock:
            self.status_cache = snapshot
            self.status_cache_stamp = mono_now
        return snapshot

    def call_trigger(self, key: str, timeout_sec: float = 4.0) -> tuple[bool, str]:
        client = self.trigger_clients.get(key)
        if client is None:
            return False, f'unknown service key: {key}'
        if str(client.srv_name).startswith('/trash_manager/'):
            manager_ok, manager_msg = self.ensure_manager_ready(timeout_sec=min(max(timeout_sec, 8.0), 20.0))
            if not manager_ok:
                return False, manager_msg
        if not client.wait_for_service(timeout_sec=0.2):
            return False, f'service not ready: {client.srv_name}'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                return bool(result.success), str(result.message)
            time.sleep(0.02)
        return False, f'service timeout: {client.srv_name}'

    def get_grasper_bool_param(self, name: str, timeout_sec: float = 0.8) -> bool | None:
        if not self.grasper_param_client.wait_for_service(timeout_sec=0.1):
            return None
        req = GetParameters.Request()
        req.names = [name]
        future = self.grasper_param_client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                if not result.values:
                    return None
                value = result.values[0]
                if value.type == ParameterType.PARAMETER_BOOL:
                    return bool(value.bool_value)
                return None
            time.sleep(0.02)
        return None

    def wait_for_fresh_grasp_target(self, since: float, timeout_sec: float = 24.0) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last_detail = 'no target yet'
        while time.monotonic() < deadline:
            with self.state_lock:
                label = dict(self.latest_target)
                runtime = json.loads(json.dumps(self.latest_grasp_runtime))
            arm = runtime.get('arm_target') if isinstance(runtime.get('arm_target'), dict) else {}
            camera = runtime.get('camera_target') if isinstance(runtime.get('camera_target'), dict) else {}
            plan = runtime.get('grasp_plan') if isinstance(runtime.get('grasp_plan'), dict) else {}
            label_stamp = float(label.get('stamp') or 0.0)
            arm_stamp = float(arm.get('stamp') or 0.0)
            camera_stamp = float(camera.get('stamp') or 0.0)
            plan_stamp = float(plan.get('stamp') or 0.0)
            if label_stamp >= since and arm_stamp >= since and camera_stamp >= since and plan_stamp >= since:
                arm_point = arm.get('point') or [0.0, 0.0, 0.0]
                arm_mm = [float(v) * 1000.0 for v in arm_point]
                return True, (
                    f"fresh target label={label.get('label', '')} "
                    f"arm_mm={arm_mm[0]:.1f},{arm_mm[1]:.1f},{arm_mm[2]:.1f}"
                )
            last_detail = (
                f"waiting fresh target label_age={time.time() - label_stamp:.1f}s "
                f"arm_age={time.time() - arm_stamp:.1f}s "
                f"camera_age={time.time() - camera_stamp:.1f}s "
                f"plan_age={time.time() - plan_stamp:.1f}s"
            )
            time.sleep(0.15)
        return False, f'no fresh VLM/grasp target after refresh: {last_detail}'

    def ensure_live_grasp_ready(self) -> tuple[bool, str]:
        messages: list[str] = []
        dry_run = self.get_grasper_bool_param('dry_run')
        needs_start = (
            dry_run is not False
            or not self.service_ready('vlm_refresh')
            or not self.service_ready('direct_grasp_once')
        )
        if not needs_start:
            return True, 'live grasp services already ready'

        if not self.service_ready('start_grasp_live'):
            return False, 'manager grasp-live service is not ready; check WebUI/manager DDS environment'

        start_ok, start_msg = self.call_trigger('start_grasp_live', timeout_sec=120.0)
        messages.append(f'start_grasp_live={start_ok}:{start_msg}')
        if not start_ok:
            return False, '; '.join(messages)

        deadline = time.monotonic() + 45.0
        last_detail = 'waiting live grasp stack'
        while time.monotonic() < deadline:
            dry_run = self.get_grasper_bool_param('dry_run')
            vlm_ready = self.service_ready('vlm_refresh')
            grasp_ready = self.service_ready('direct_grasp_once')
            if dry_run is False and vlm_ready and grasp_ready:
                messages.append('live grasp services ready')
                return True, '; '.join(messages)
            last_detail = f'dry_run={dry_run} vlm_refresh={vlm_ready} direct_grasp_once={grasp_ready}'
            time.sleep(0.5)
        messages.append(f'timeout waiting live grasp services: {last_detail}')
        return False, '; '.join(messages)

    def prepare_live_grasp_once(self) -> tuple[bool, str]:
        messages: list[str] = []
        manager_ok, manager_msg = self.ensure_manager_ready()
        messages.append(manager_msg)
        if not manager_ok:
            return False, '; '.join(messages)
        video_ok, video_msg = self.ensure_video_ready()
        messages.append(video_msg)
        if not video_ok:
            return False, '; '.join(messages)
        ready_ok, ready_msg = self.ensure_live_grasp_ready()
        messages.append(ready_msg)
        if not ready_ok:
            return False, '; '.join(messages)

        since = time.time()
        refresh_ok, refresh_msg = self.call_trigger('vlm_refresh', timeout_sec=8.0)
        messages.append(f'vlm_refresh={refresh_ok}:{refresh_msg}')
        if not refresh_ok:
            return False, '; '.join(messages)
        fresh_ok, fresh_msg = self.wait_for_fresh_grasp_target(since, timeout_sec=30.0)
        messages.append(fresh_msg)
        return fresh_ok, '; '.join(messages)

    def grasp_once_from_web(self) -> tuple[bool, str]:
        ready_ok, ready_msg = self.prepare_live_grasp_once()
        if not ready_ok:
            return False, ready_msg
        ok, msg = self.call_trigger('direct_grasp_once', timeout_sec=90.0)
        return ok, f'{ready_msg}; grasp_once={ok}:{msg}'

    def call_set_grasp(self, enabled: bool, timeout_sec: float = 4.0) -> tuple[bool, str]:
        if not self.set_grasp_client.wait_for_service(timeout_sec=0.2):
            return False, 'service not ready: /trash_mission/set_grasp_enabled'
        req = SetBool.Request()
        req.data = bool(enabled)
        future = self.set_grasp_client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                return bool(result.success), str(result.message)
            time.sleep(0.02)
        return False, 'service timeout: /trash_mission/set_grasp_enabled'

    def read_yaml_dict(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        return data if isinstance(data, dict) else {}

    def write_yaml_dict(self, path: Path, data: dict[str, Any], tag: str) -> Path | None:
        backup_path = None
        if path.exists():
            backup_dir = path.parent / 'backups'
            backup_dir.mkdir(parents=True, exist_ok=True)
            backup_path = backup_dir / f'{path.name}.before_{tag}_{time.strftime("%Y%m%d_%H%M%S")}'
            shutil.copy2(path, backup_path)
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
        return backup_path

    def vlm_model_snapshot(self) -> dict[str, Any]:
        cfg = self.read_yaml_dict(self.vlm_config_file)
        reg = self.read_yaml_dict(self.vlm_registry_file)
        cfg_providers = cfg.get('providers', {}) if isinstance(cfg.get('providers'), dict) else {}
        reg_providers = reg.get('providers', {}) if isinstance(reg.get('providers'), dict) else {}
        names = sorted(set(cfg_providers.keys()) | set(reg_providers.keys()))
        active = str(cfg.get('active_provider') or reg.get('active_provider') or '').strip()
        providers = []
        for name in names:
            cfg_item = cfg_providers.get(name, {}) if isinstance(cfg_providers.get(name, {}), dict) else {}
            reg_item = reg_providers.get(name, {}) if isinstance(reg_providers.get(name, {}), dict) else {}
            merged = {**reg_item, **cfg_item}
            model = str(merged.get('primary_model') or merged.get('model') or '').strip()
            raw_candidates = merged.get('model_candidates', [])
            candidates = [model] if model else []
            if isinstance(raw_candidates, list):
                candidates.extend(str(item).strip() for item in raw_candidates if str(item).strip())
            candidates = list(dict.fromkeys(candidates))
            api_key_env = str(merged.get('api_key_env') or '').strip()
            providers.append({
                'name': name,
                'enabled': bool(merged.get('enabled', False)),
                'model': model,
                'model_candidates': candidates,
                'base_url': str(merged.get('base_url') or ''),
                'api_key_env': api_key_env,
                'api_key_present': bool(api_key_env and os.environ.get(api_key_env, '').strip()),
                'local': name == 'local_hobot',
            })
        return {
            'ok': True,
            'active_provider': active,
            'config_file': str(self.vlm_config_file),
            'registry_file': str(self.vlm_registry_file),
            'providers': providers,
        }

    def set_vlm_model(self, provider: str, model: str) -> tuple[bool, str, dict[str, Any]]:
        provider = str(provider or '').strip()
        model = str(model or '').strip()
        if not provider:
            return False, 'provider required', {}
        cfg = self.read_yaml_dict(self.vlm_config_file)
        reg = self.read_yaml_dict(self.vlm_registry_file)
        cfg.setdefault('providers', {})
        reg.setdefault('providers', {})
        if not isinstance(cfg['providers'], dict) or not isinstance(reg['providers'], dict):
            return False, 'invalid VLM provider config structure', {}
        known = set(cfg['providers'].keys()) | set(reg['providers'].keys())
        if provider not in known:
            return False, f'unknown VLM provider: {provider}', {}

        snapshot = self.vlm_model_snapshot()
        selected = next((item for item in snapshot['providers'] if item['name'] == provider), None)
        if selected is None:
            return False, f'provider not configured: {provider}', {}
        candidates = list(selected.get('model_candidates') or [])
        if not model:
            model = str(selected.get('model') or (candidates[0] if candidates else '')).strip()
        if provider == 'local_hobot' and not model:
            model = 'hobot_dosod'
        if not model:
            return False, f'model required for provider: {provider}', {}

        def update_doc(data: dict[str, Any], primary_key: str) -> None:
            data['active_provider'] = provider
            data['fallback_order'] = [provider]
            providers = data.setdefault('providers', {})
            if not isinstance(providers, dict):
                data['providers'] = providers = {}
            for name, item in providers.items():
                if isinstance(item, dict):
                    item['enabled'] = name == provider
            item = providers.setdefault(provider, {})
            if isinstance(item, dict):
                item['enabled'] = True
                item[primary_key] = model
                current_candidates = item.get('model_candidates', [])
                merged_candidates = [model]
                if isinstance(current_candidates, list):
                    merged_candidates.extend(str(v).strip() for v in current_candidates if str(v).strip())
                item['model_candidates'] = list(dict.fromkeys(merged_candidates))

        update_doc(cfg, 'model')
        update_doc(reg, 'primary_model')
        cfg_backup = self.write_yaml_dict(self.vlm_config_file, cfg, 'vlm_select')
        reg_backup = self.write_yaml_dict(self.vlm_registry_file, reg, 'vlm_select')
        return True, f'VLM model selected provider={provider} model={model}', {
            'active_provider': provider,
            'model': model,
            'config_backup': str(cfg_backup) if cfg_backup else '',
            'registry_backup': str(reg_backup) if reg_backup else '',
        }

    def upsert_vlm_provider(
        self,
        provider: str,
        model: str,
        base_url: str,
        api_key_env: str,
        api_key: str = '',
        activate: bool = True,
    ) -> tuple[bool, str, dict[str, Any]]:
        provider = str(provider or '').strip()
        model = str(model or '').strip()
        base_url = str(base_url or '').strip().rstrip('/')
        api_key_env = str(api_key_env or '').strip()
        api_key = str(api_key or '').strip()
        if not provider:
            return False, 'provider required', {}
        if not re.match(r'^[A-Za-z0-9_-]+$', provider):
            return False, 'provider must use letters, numbers, _ or -', {}
        if provider != 'local_hobot' and not base_url:
            return False, 'base_url required for API provider', {}
        if base_url.endswith('/chat/completions'):
            base_url = base_url[: -len('/chat/completions')].rstrip('/')
        if not model:
            return False, 'model required', {}
        if provider != 'local_hobot' and not api_key_env:
            api_key_env = f'{provider.upper().replace("-", "_")}_API_KEY'

        cfg = self.read_yaml_dict(self.vlm_config_file)
        reg = self.read_yaml_dict(self.vlm_registry_file)
        cfg.setdefault('providers', {})
        reg.setdefault('providers', {})
        if not isinstance(cfg['providers'], dict) or not isinstance(reg['providers'], dict):
            return False, 'invalid VLM provider config structure', {}

        def merge_candidates(item: dict[str, Any]) -> list[str]:
            current = item.get('model_candidates', [])
            values = [model]
            if isinstance(current, list):
                values.extend(str(v).strip() for v in current if str(v).strip())
            return list(dict.fromkeys(values))

        cfg_item = cfg['providers'].setdefault(provider, {})
        if not isinstance(cfg_item, dict):
            cfg['providers'][provider] = cfg_item = {}
        cfg_item.update({
            'enabled': True,
            'model': model,
            'model_candidates': merge_candidates(cfg_item),
            'base_url': base_url,
            'api_key_env': api_key_env,
        })

        reg_item = reg['providers'].setdefault(provider, {})
        if not isinstance(reg_item, dict):
            reg['providers'][provider] = reg_item = {}
        reg_item.update({
            'enabled': True,
            'primary_model': model,
            'model_candidates': merge_candidates(reg_item),
            'base_url': base_url,
            'api_key_env': api_key_env,
        })

        if activate:
            cfg['active_provider'] = provider
            cfg['fallback_order'] = [provider]
            reg['active_provider'] = provider
            reg['fallback_order'] = [provider]
            for items in (cfg['providers'], reg['providers']):
                for name, item in items.items():
                    if isinstance(item, dict):
                        item['enabled'] = name == provider

        key_msg = ''
        if api_key and api_key_env:
            secrets_dir = self.root / 'runtime' / 'secrets'
            secrets_dir.mkdir(parents=True, exist_ok=True)
            secrets_file = secrets_dir / 'vlm.env'
            existing: list[str] = []
            if secrets_file.exists():
                existing = secrets_file.read_text(encoding='utf-8').splitlines()
            prefix = f'export {api_key_env}='
            kept = [line for line in existing if not line.startswith(prefix) and not line.startswith(f'{api_key_env}=')]
            shell_key = "'" + api_key.replace("'", "'\"'\"'") + "'"
            kept.append(f'export {api_key_env}={shell_key}')
            secrets_file.write_text('\n'.join(kept) + '\n', encoding='utf-8')
            os.environ[api_key_env] = api_key
            key_msg = f'; key saved to {secrets_file}'

        cfg_backup = self.write_yaml_dict(self.vlm_config_file, cfg, 'vlm_provider')
        reg_backup = self.write_yaml_dict(self.vlm_registry_file, reg, 'vlm_provider')
        return True, f'VLM provider saved provider={provider} model={model}{key_msg}', {
            'active_provider': provider if activate else str(cfg.get('active_provider') or ''),
            'model': model,
            'config_backup': str(cfg_backup) if cfg_backup else '',
            'registry_backup': str(reg_backup) if reg_backup else '',
        }

    def publish_zero(self, reason: str = 'zero') -> None:
        msg = Twist()
        for _ in range(4):
            self.cmd_pub.publish(msg)
            time.sleep(0.02)
        self.get_logger().warning(f'zero velocity published: {reason}')

    def software_estop(self) -> tuple[bool, str]:
        self.manual_enabled_until = 0.0
        self.publish_zero('web estop')
        safety_ok, safety_msg = self.call_trigger('safety_stop', timeout_sec=2.0)
        manager_ok, manager_msg = self.call_trigger('estop', timeout_sec=3.0)
        return True, f'estop sent; safety={safety_ok}:{safety_msg}; manager={manager_ok}:{manager_msg}'

    def manual_cmd(self, linear: float, angular: float, hold_sec: float, safety_confirm: bool) -> tuple[bool, str]:
        del safety_confirm
        status = self.status_snapshot()
        if bool(status.get('manager', {}).get('motion_lock', {}).get('estop_active')):
            self.publish_zero('manual command blocked by estop')
            return False, 'estop is active'
        msg = Twist()
        msg.linear.x = clamp(float(linear), -self.manual_max_linear, self.manual_max_linear)
        msg.angular.z = clamp(float(angular), -self.manual_max_angular, self.manual_max_angular)
        self.manual_enabled_until = time.monotonic() + clamp(float(hold_sec), 0.1, 0.8)
        self.cmd_pub.publish(msg)
        return True, f'manual cmd linear={msg.linear.x:.2f} angular={msg.angular.z:.2f}'

    def publish_initial_pose(self, x: float, y: float, yaw_deg: float) -> tuple[bool, str]:
        nav_ok, nav_msg = self.wait_for_map_ready(timeout_sec=0.5)
        if not nav_ok:
            return False, f'navigation/map not ready: {nav_msg}; start navigation from Service Control first'
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = yaw_to_quaternion(math.radians(float(yaw_deg)))
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pose_pub.publish(msg)
        return True, f'{nav_msg}; initial pose published x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f}'

    def send_nav_goal(self, x: float, y: float, yaw_deg: float, safety_confirm: bool) -> tuple[bool, str]:
        del safety_confirm
        map_ok, map_msg = self.wait_for_map_ready(timeout_sec=0.5)
        if not map_ok:
            return False, f'navigation/map not ready: {map_msg}; start navigation from Service Control first'
        nav_ok, nav_msg = self.wait_for_goal_pose_ready(timeout_sec=0.5)
        if not nav_ok:
            return False, f'navigation goal interface not ready: {nav_msg}; start navigation from Service Control first'
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        q = yaw_to_quaternion(math.radians(float(yaw_deg)))
        pose.pose.orientation.x = q['x']
        pose.pose.orientation.y = q['y']
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']
        self.goal_pose_pub.publish(pose)
        with self.nav2d_lock:
            self.latest_goal = {
                'x': float(x),
                'y': float(y),
                'yaw_deg': float(yaw_deg),
                'stamp': time.time(),
                'active': True,
                'state': 'PUBLISHED',
                'status_text': 'PUBLISHED',
                'sent_via': '/goal_pose',
            }
        self.bump_nav2d_seq('goal')
        return True, f'{map_msg}; {nav_msg}; /goal_pose published x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f}'

    def cancel_navigation(self, timeout_sec: float = 3.0) -> tuple[bool, str]:
        if not self.cancel_nav_client.wait_for_service(timeout_sec=0.2):
            return False, 'service not ready: /navigate_to_pose/_action/cancel_goal'
        req = CancelGoal.Request()
        future = self.cancel_nav_client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                result = future.result()
                with self.nav2d_lock:
                    if self.latest_goal is not None:
                        self.latest_goal['cancel_requested'] = True
                        self.latest_goal['state'] = 'CANCELING'
                        self.latest_goal['status_text'] = 'CANCELING'
                self.bump_nav2d_seq('goal')
                return True, f'cancel navigation requested via /navigate_to_pose/_action/cancel_goal return_code={getattr(result, "return_code", "")}'
            time.sleep(0.02)
        return False, 'cancel navigation timeout'

    def wait_for_map_ready(self, timeout_sec: float = 25.0) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last = 'waiting for /map and /initialpose subscriber'
        while time.monotonic() < deadline:
            map_pubs = self.topic_publishers('/map')
            initial_subs = self.topic_subscribers('/initialpose')
            if map_pubs > 0 and initial_subs > 0:
                return True, 'navigation map ready: /map online and AMCL accepts initial pose'
            last = f'/map publishers={map_pubs}, /initialpose subscribers={initial_subs}'
            time.sleep(0.2)
        return False, last

    def wait_for_goal_pose_ready(self, timeout_sec: float = 25.0) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last = 'waiting for /goal_pose subscriber'
        while time.monotonic() < deadline:
            goal_subs = self.topic_subscribers('/goal_pose')
            if goal_subs > 0:
                return True, '/goal_pose subscriber online'
            last = f'/goal_pose subscribers={goal_subs}, manager_service={self.service_ready("start_navigation")}'
            time.sleep(0.2)
        return False, last

    def ensure_navigation_map_ready(self, timeout_sec: float = 45.0) -> tuple[bool, str]:
        ready_ok, ready_msg = self.wait_for_map_ready(timeout_sec=0.5)
        if ready_ok:
            return True, ready_msg
        start_ok, start_msg = self.call_trigger('start_navigation', timeout_sec=8.0)
        if not start_ok:
            return False, start_msg
        ready_ok, ready_msg = self.wait_for_map_ready(timeout_sec=max(1.0, timeout_sec - 8.0))
        if not ready_ok:
            return False, f'{start_msg}; {ready_msg}'
        return True, f'{start_msg}; {ready_msg}'

    def ensure_navigation_ready(self, timeout_sec: float = 50.0) -> tuple[bool, str]:
        map_ok, map_msg = self.ensure_navigation_map_ready(timeout_sec=min(45.0, timeout_sec))
        if not map_ok:
            return False, map_msg
        goal_ok, goal_msg = self.wait_for_goal_pose_ready(timeout_sec=max(1.0, timeout_sec - 45.0))
        if not goal_ok:
            return False, f'{map_msg}; {goal_msg}'
        return True, f'{map_msg}; {goal_msg}'

    def read_routes(self) -> dict[str, Any]:
        if not self.route_file.exists():
            return {'active_route': 'office_loop', 'routes': {}, 'mission': {}}
        data = yaml.safe_load(self.route_file.read_text(encoding='utf-8')) or {}
        return data if isinstance(data, dict) else {'active_route': 'office_loop', 'routes': {}, 'mission': {}}

    def write_routes(self, data: dict[str, Any]) -> Path | None:
        backup_path = None
        if self.route_file.exists():
            backup_dir = self.route_file.parent / 'backups'
            backup_dir.mkdir(parents=True, exist_ok=True)
            backup_path = backup_dir / f'{self.route_file.name}.before_web_{time.strftime("%Y%m%d_%H%M%S")}'
            shutil.copy2(self.route_file, backup_path)
        self.route_file.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding='utf-8',
        )
        return backup_path

    def record_current_waypoint(self, route: str, name: str, reset: bool = False) -> tuple[bool, str, dict[str, Any]]:
        pose = self.current_pose()
        if pose is None:
            return False, 'no map pose available; set initial pose first', {}
        return self.add_waypoint(route, name, float(pose['x']), float(pose['y']), float(pose['yaw_deg']), reset)

    def add_waypoint(
        self,
        route: str,
        name: str,
        x: float,
        y: float,
        yaw_deg: float,
        reset: bool = False,
        activate: bool = True,
    ) -> tuple[bool, str, dict[str, Any]]:
        data = self.read_routes()
        routes = data.setdefault('routes', {})
        if reset or route not in routes or not isinstance(routes.get(route), dict):
            routes[route] = {'loop': True, 'waypoints': []}
        selected = routes[route]
        waypoints = selected.setdefault('waypoints', [])
        wp = {
            'name': name,
            'x': round(float(x), 3),
            'y': round(float(y), 3),
            'yaw_deg': round(float(yaw_deg), 1),
        }
        replaced = False
        for idx, item in enumerate(waypoints):
            if isinstance(item, dict) and item.get('name') == name:
                waypoints[idx] = wp
                replaced = True
                break
        if not replaced:
            waypoints.append(wp)
        if activate:
            data['active_route'] = route
        backup = self.write_routes(data)
        return True, f'waypoint {"updated" if replaced else "added"}: {route}/{name}', {
            'waypoint': wp,
            'backup': str(backup) if backup else '',
        }

    def delete_waypoint(self, route: str, index: int) -> tuple[bool, str]:
        data = self.read_routes()
        routes = data.setdefault('routes', {})
        selected = routes.get(route)
        if not isinstance(selected, dict):
            return False, f'route not found: {route}'
        waypoints = selected.get('waypoints', [])
        if not isinstance(waypoints, list) or index < 0 or index >= len(waypoints):
            return False, f'waypoint index out of range: {index}'
        removed = waypoints.pop(index)
        self.write_routes(data)
        return True, f'waypoint removed: {route}/{removed.get("name", index)}'

    def set_active_route(self, route: str) -> tuple[bool, str]:
        data = self.read_routes()
        routes = data.setdefault('routes', {})
        if route not in routes:
            routes[route] = {'loop': True, 'waypoints': []}
        data['active_route'] = route
        self.write_routes(data)
        return True, f'active route set: {route}'

    def save_route(self, route: str, waypoints: list[Any], loop: bool = True, activate: bool = True) -> tuple[bool, str]:
        clean: list[dict[str, Any]] = []
        for idx, item in enumerate(waypoints):
            if not isinstance(item, dict):
                continue
            clean.append({
                'name': str(item.get('name') or f'p{idx + 1}'),
                'x': round(float(item.get('x', 0.0)), 3),
                'y': round(float(item.get('y', 0.0)), 3),
                'yaw_deg': round(float(item.get('yaw_deg', 0.0)), 1),
            })
        data = self.read_routes()
        routes = data.setdefault('routes', {})
        routes[route] = {'loop': bool(loop), 'waypoints': clean}
        if activate:
            data['active_route'] = route
        self.write_routes(data)
        return True, f'route saved: {route} points={len(clean)}'

    @staticmethod
    def stamp_to_float(stamp: Any) -> float:
        return float(getattr(stamp, 'sec', 0)) + float(getattr(stamp, 'nanosec', 0)) / 1e9

    def marker_payload(self, markers: list[Marker]) -> dict[str, Any]:
        payload_markers: list[dict[str, Any]] = []
        latest_stamp = 0.0
        for marker in markers:
            if int(marker.action) in (Marker.DELETE, Marker.DELETEALL):
                continue
            source_frame = marker.header.frame_id or 'map'
            latest_stamp = max(latest_stamp, self.stamp_to_float(marker.header.stamp))
            pose = marker.pose
            yaw = quaternion_to_yaw(pose.orientation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            raw_points = []
            source_points = list(marker.points)
            if not source_points:
                raw_points.append([float(pose.position.x), float(pose.position.y)])
            for point in source_points:
                px = float(pose.position.x) + float(point.x) * cos_yaw - float(point.y) * sin_yaw
                py = float(pose.position.y) + float(point.x) * sin_yaw + float(point.y) * cos_yaw
                raw_points.append([px, py])
            points, tf_ok, detail = self.points_to_map(source_frame, raw_points)
            payload_markers.append({
                'ns': marker.ns,
                'id': int(marker.id),
                'type': int(marker.type),
                'action': int(marker.action),
                'frame_id': 'map' if tf_ok else source_frame,
                'source_frame_id': source_frame,
                'stamp': self.stamp_to_float(marker.header.stamp),
                'tf_ok': tf_ok,
                'tf_detail': detail,
                'scale': {
                    'x': float(marker.scale.x),
                    'y': float(marker.scale.y),
                    'z': float(marker.scale.z),
                },
                'color': {
                    'r': float(marker.color.r),
                    'g': float(marker.color.g),
                    'b': float(marker.color.b),
                    'a': float(marker.color.a),
                },
                'points': points,
            })
        return {
            'ok': bool(payload_markers),
            'frame_id': 'map',
            'stamp': latest_stamp,
            'markers': payload_markers,
        }

    def grid_payload(self, msg: OccupancyGrid, max_cells: int = 220000) -> dict[str, Any]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        factor = max(1, math.ceil(math.sqrt((width * height) / max_cells)))
        data = list(msg.data)
        if factor > 1:
            sampled = []
            for y in range(0, height, factor):
                row = y * width
                for x in range(0, width, factor):
                    sampled.append(int(data[row + x]))
            out_width = math.ceil(width / factor)
            out_height = math.ceil(height / factor)
        else:
            sampled = [int(v) for v in data]
            out_width = width
            out_height = height
        source_frame = msg.header.frame_id or 'map'
        origin_yaw = quaternion_to_yaw(msg.info.origin.orientation)
        origin_x, origin_y, origin_yaw, tf_ok, tf_detail = self.transform_xy_yaw_to_map(
            source_frame,
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            origin_yaw,
        )
        return {
            'ok': True,
            'width': out_width,
            'height': out_height,
            'source_width': width,
            'source_height': height,
            'downsample': factor,
            'resolution': float(msg.info.resolution) * factor,
            'origin': {
                'x': origin_x,
                'y': origin_y,
                'yaw_deg': math.degrees(origin_yaw),
            },
            'frame_id': 'map' if tf_ok else source_frame,
            'source_frame_id': source_frame,
            'tf_ok': tf_ok,
            'tf_detail': tf_detail,
            'stamp': self.stamp_to_float(msg.header.stamp),
            'data': sampled,
        }

    def path_payload(self, msg: NavPath | None) -> dict[str, Any]:
        if msg is None:
            return {'points': [], 'frame_id': 'map', 'stamp': 0.0}
        source_frame = msg.header.frame_id or 'map'
        raw_points = [[float(pose.pose.position.x), float(pose.pose.position.y)] for pose in msg.poses]
        points, tf_ok, detail = self.points_to_map(source_frame, raw_points)
        return {
            'frame_id': 'map' if tf_ok else source_frame,
            'source_frame_id': source_frame,
            'stamp': self.stamp_to_float(msg.header.stamp),
            'points': points,
            'tf_ok': tf_ok,
            'tf_detail': detail,
        }

    def scan_payload(self, msg: LaserScan, max_points: int = 900) -> dict[str, Any]:
        frame_id = msg.header.frame_id
        points: list[list[float]] = []
        try:
            tf = self.tf_buffer.lookup_transform('map', frame_id, Time())
            tx = float(tf.transform.translation.x)
            ty = float(tf.transform.translation.y)
            yaw = quaternion_to_yaw(tf.transform.rotation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            tf_ok = True
            detail = 'tf map->' + frame_id
        except Exception as exc:  # noqa: BLE001 - UI should show scan health, not crash callbacks
            tx = ty = yaw = 0.0
            cos_yaw = 1.0
            sin_yaw = 0.0
            tf_ok = False
            detail = str(exc)[-160:]

        ranges = list(msg.ranges)
        step = max(1, math.ceil(len(ranges) / max_points))
        for idx in range(0, len(ranges), step):
            r = float(ranges[idx])
            if not math.isfinite(r) or r < float(msg.range_min) or r > float(msg.range_max):
                continue
            angle = float(msg.angle_min) + float(idx) * float(msg.angle_increment)
            sx = r * math.cos(angle)
            sy = r * math.sin(angle)
            if tf_ok:
                points.append([tx + sx * cos_yaw - sy * sin_yaw, ty + sx * sin_yaw + sy * cos_yaw])
        return {
            'frame_id': 'map' if tf_ok else frame_id,
            'source_frame_id': frame_id,
            'stamp': self.stamp_to_float(msg.header.stamp),
            'tf_ok': tf_ok,
            'tf_detail': detail,
            'points': points,
        }

    def nav2d_payload(self, last_versions: dict[str, int] | None = None) -> dict[str, Any]:
        last_versions = last_versions or {}
        with self.state_lock:
            map_msg = self.latest_map
        tf_pose = self.robot_tf_pose()
        pose = tf_pose or self.amcl_pose_snapshot()
        with self.nav2d_lock:
            seq = dict(self.nav2d_seq)
            map_marker = json.loads(json.dumps(self.latest_map_marker)) if self.latest_map_marker else None
            global_costmap = self.latest_global_costmap
            local_costmap = self.latest_local_costmap
            global_path = self.latest_global_path
            local_path = self.latest_local_path
            scan = json.loads(json.dumps(self.latest_scan_points or {}))
            scan_depth = json.loads(json.dumps(self.latest_scan_depth_points or {}))
            footprint = json.loads(json.dumps(self.latest_footprint)) if self.latest_footprint else None
            nav_status = json.loads(json.dumps(self.latest_nav_status or {}))
            goal = json.loads(json.dumps(self.latest_goal)) if self.latest_goal else None

        payload: dict[str, Any] = {
            'op': 'nav2d',
            'type': 'nav2d_snapshot',
            'seq': seq,
            'stamp': time.time(),
            'pose': pose,
            'status': {
                'nav_active': bool(nav_status.get('active')),
                'nav_status': nav_status,
                'has_map': map_msg is not None,
                'has_map_marker': bool(map_marker and map_marker.get('markers')),
                'has_tf': tf_pose is not None,
                'has_global_path': bool(global_path and global_path.poses),
                'has_local_path': bool(local_path and local_path.poses),
                'has_scan': bool(scan.get('points')),
                'has_scan_depth': bool(scan_depth.get('points')),
                'has_global_costmap': global_costmap is not None,
                'has_local_costmap': local_costmap is not None,
                'last_error': self.nav2d_last_error,
            },
        }
        if goal is not None:
            payload['goal'] = goal
        if map_marker is not None and seq.get('map_marker', 0) != last_versions.get('map_marker', -1):
            payload['map_marker'] = map_marker
        if footprint is not None and seq.get('footprint', 0) != last_versions.get('footprint', -1):
            payload['footprint'] = footprint
        if map_msg is not None and seq.get('map', 0) != last_versions.get('map', -1):
            payload['map'] = self.grid_payload(map_msg)
        if global_costmap is not None and seq.get('global_costmap', 0) != last_versions.get('global_costmap', -1):
            payload['global_costmap'] = self.grid_payload(global_costmap)
        if local_costmap is not None and seq.get('local_costmap', 0) != last_versions.get('local_costmap', -1):
            payload['local_costmap'] = self.grid_payload(local_costmap)
        if seq.get('global_path', 0) != last_versions.get('global_path', -1):
            payload['global_path'] = self.path_payload(global_path)
        if seq.get('local_path', 0) != last_versions.get('local_path', -1):
            payload['local_path'] = self.path_payload(local_path)
        if seq.get('scan', 0) != last_versions.get('scan', -1):
            payload['scan'] = scan
        if seq.get('scan_depth', 0) != last_versions.get('scan_depth', -1):
            payload['scan_depth'] = scan_depth
        return payload

    def map_payload(self, max_cells: int = 160000) -> dict[str, Any]:
        with self.state_lock:
            msg = self.latest_map
        if msg is None:
            fallback = self.map_file_payload(max_cells=max_cells)
            if fallback.get('ok'):
                return fallback
            return {'ok': False, 'message': 'map not received'}
        width = int(msg.info.width)
        height = int(msg.info.height)
        factor = max(1, math.ceil(math.sqrt((width * height) / max_cells)))
        data = list(msg.data)
        if factor > 1:
            sampled = []
            for y in range(0, height, factor):
                row = y * width
                for x in range(0, width, factor):
                    sampled.append(int(data[row + x]))
            out_width = math.ceil(width / factor)
            out_height = math.ceil(height / factor)
        else:
            sampled = [int(v) for v in data]
            out_width = width
            out_height = height
        return {
            'ok': True,
            'width': out_width,
            'height': out_height,
            'source_width': width,
            'source_height': height,
            'downsample': factor,
            'resolution': float(msg.info.resolution) * factor,
            'origin': {
                'x': float(msg.info.origin.position.x),
                'y': float(msg.info.origin.position.y),
                'yaw_deg': math.degrees(quaternion_to_yaw(msg.info.origin.orientation)),
            },
            'frame_id': msg.header.frame_id,
            'data': sampled,
        }

    def current_map_yaml(self) -> Path | None:
        current = self.root / 'runtime' / 'current_map.txt'
        candidates = []
        try:
            text = current.read_text(encoding='utf-8').strip()
            if text:
                candidates.append(Path(text))
        except OSError:
            pass
        candidates.append(self.root / 'maps' / '344.yaml')
        candidates.extend(sorted(self.maps_dir.glob('*.yaml')))
        for path in candidates:
            if not path.is_absolute():
                path = self.root / path
            if path.exists():
                return path
        return None

    @staticmethod
    def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
        raw = path.read_bytes()
        tokens: list[bytes] = []
        i = 0
        while len(tokens) < 4 and i < len(raw):
            while i < len(raw) and raw[i] in b' \t\r\n':
                i += 1
            if i < len(raw) and raw[i] == ord('#'):
                while i < len(raw) and raw[i] not in b'\r\n':
                    i += 1
                continue
            start = i
            while i < len(raw) and raw[i] not in b' \t\r\n':
                i += 1
            if start < i:
                tokens.append(raw[start:i])
        if len(tokens) < 4 or tokens[0] != b'P5':
            raise ValueError(f'unsupported PGM file: {path}')
        while i < len(raw) and raw[i] in b' \t\r\n':
            i += 1
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
        pixels = raw[i:i + width * height]
        if len(pixels) < width * height:
            raise ValueError(f'truncated PGM file: {path}')
        return width, height, max_value, pixels

    def map_file_payload(self, max_cells: int = 160000) -> dict[str, Any]:
        map_yaml = self.current_map_yaml()
        if map_yaml is None:
            return {'ok': False, 'message': 'map file not found'}
        try:
            meta = yaml.safe_load(map_yaml.read_text(encoding='utf-8')) or {}
            image_path = Path(str(meta.get('image') or ''))
            if not image_path.is_absolute():
                image_path = map_yaml.parent / image_path
            width, height, max_value, pixels = self.read_pgm(image_path)
            resolution = float(meta.get('resolution') or 0.05)
            origin = meta.get('origin') if isinstance(meta.get('origin'), list) else [0.0, 0.0, 0.0]
            negate = int(meta.get('negate') or 0)
            occupied_thresh = float(meta.get('occupied_thresh') or 0.65)
            free_thresh = float(meta.get('free_thresh') or 0.25)
        except Exception as exc:  # noqa: BLE001 - fallback should report clear UI error
            return {'ok': False, 'message': f'map file load failed: {exc}'}

        factor = max(1, math.ceil(math.sqrt((width * height) / max_cells)))

        def pixel_to_occ(pixel: int) -> int:
            value = (pixel / max(1, max_value))
            if negate:
                value = 1.0 - value
            occ = 1.0 - value
            if occ > occupied_thresh:
                return 100
            if occ < free_thresh:
                return 0
            return -1

        sampled = []
        for y in range(0, height, factor):
            row = y * width
            for x in range(0, width, factor):
                sampled.append(pixel_to_occ(pixels[row + x]))
        return {
            'ok': True,
            'source': 'map_file',
            'map_file': str(map_yaml),
            'width': math.ceil(width / factor),
            'height': math.ceil(height / factor),
            'source_width': width,
            'source_height': height,
            'downsample': factor,
            'resolution': resolution * factor,
            'origin': {
                'x': float(origin[0] if len(origin) > 0 else 0.0),
                'y': float(origin[1] if len(origin) > 1 else 0.0),
                'yaw_deg': math.degrees(float(origin[2] if len(origin) > 2 else 0.0)),
            },
            'frame_id': 'map',
            'data': sampled,
        }

    def list_maps(self) -> list[dict[str, str]]:
        out = []
        for path in sorted(self.maps_dir.glob('*.yaml')):
            out.append({'name': path.stem, 'path': str(path)})
        return out

    def tail_log(self, name: str, lines: int = 120) -> dict[str, Any]:
        allowed = {
            'navigation': self.root / 'runtime/logs/navigation/nav2.log',
            'mission': self.root / 'runtime/logs/navigation/mission_supervisor.log',
            'web': self.root / 'runtime/logs/web/web_console.log',
            'manager': self.root / 'runtime/logs/web/robot_manager.log',
            'vlmapi': self.root / 'runtime/logs/manager/grasp_vlm_dry.log',
            'vlm_pipeline': self.root / 'runtime/logs/grasp_vlm/dry/grasp_pipeline.log',
            'vlm_dosod': self.root / 'runtime/logs/grasp_vlm/dry/local_hobot_dosod.log',
            'grasp': self.root / 'runtime/logs/manager/grasp_vlm_live.log',
            'camera': self.root / 'runtime/logs/camera/realsense.log',
        }
        path = allowed.get(name, allowed['web'])
        if not path.exists():
            return {'ok': False, 'path': str(path), 'text': ''}
        try:
            text = '\n'.join(path.read_text(encoding='utf-8', errors='replace').splitlines()[-lines:])
        except OSError as exc:
            return {'ok': False, 'path': str(path), 'text': str(exc)}
        return {'ok': True, 'path': str(path), 'text': text}


def api_response(ok: bool, message: str, **extra: Any) -> dict[str, Any]:
    return {'ok': ok, 'message': message, **extra}


def create_app(node: WebConsoleNode) -> FastAPI:
    if FastAPI is None:
        raise RuntimeError(f'FastAPI dependencies are missing: {_FASTAPI_IMPORT_ERROR}')
    app = FastAPI(title='Trash Robot V3 WebUI Demo', version='0.1.0')
    static_dir = Path(__file__).parent / 'static'
    app.mount('/static', StaticFiles(directory=str(static_dir)), name='static')

    @app.get('/')
    def index() -> FileResponse:
        return FileResponse(str(static_dir / 'index.html'))

    @app.get('/api/status')
    def status() -> dict[str, Any]:
        return node.status_snapshot()

    @app.get('/api/map')
    def map_data() -> dict[str, Any]:
        return node.map_payload()

    @app.get('/api/maps')
    def maps() -> dict[str, Any]:
        return {'ok': True, 'maps': node.list_maps()}

    @app.get('/api/routes')
    def routes() -> dict[str, Any]:
        return {'ok': True, 'route_file': str(node.route_file), 'data': node.read_routes()}

    @app.post('/api/routes/record_current')
    def record_current(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        route = str(payload.get('route') or 'office_loop')
        name = str(payload.get('name') or f'p{int(time.time())}')
        ok, msg, data = node.record_current_waypoint(route, name, bool(payload.get('reset')))
        return api_response(ok, msg, **data)

    @app.post('/api/routes/add_waypoint')
    def add_waypoint(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        route = str(payload.get('route') or 'office_loop')
        name = str(payload.get('name') or f'p{int(time.time())}')
        ok, msg, data = node.add_waypoint(
            route,
            name,
            float(payload.get('x', 0.0)),
            float(payload.get('y', 0.0)),
            float(payload.get('yaw_deg', 0.0)),
            bool(payload.get('reset')),
            bool(payload.get('activate', True)),
        )
        return api_response(ok, msg, **data)

    @app.post('/api/routes/delete_waypoint')
    def delete_waypoint(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg = node.delete_waypoint(str(payload.get('route') or 'office_loop'), int(payload.get('index', -1)))
        return api_response(ok, msg)

    @app.post('/api/routes/set_active')
    def set_active_route(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg = node.set_active_route(str(payload.get('route') or 'office_loop'))
        return api_response(ok, msg)

    @app.post('/api/routes/save')
    def save_route(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg = node.save_route(
            str(payload.get('route') or 'office_loop'),
            list(payload.get('waypoints') or []),
            bool(payload.get('loop', True)),
            bool(payload.get('activate', True)),
        )
        return api_response(ok, msg)

    @app.post('/api/routes/reload')
    def reload_route() -> dict[str, Any]:
        ok, msg = node.call_trigger('patrol_reload')
        return api_response(ok, msg)

    @app.post('/api/navigation/initial_pose')
    def initial_pose(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg = node.publish_initial_pose(
            float(payload.get('x', 0.0)),
            float(payload.get('y', 0.0)),
            float(payload.get('yaw_deg', 0.0)),
        )
        return api_response(ok, msg)

    @app.post('/api/navigation/goal')
    def navigation_goal(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg = node.send_nav_goal(
            float(payload.get('x', 0.0)),
            float(payload.get('y', 0.0)),
            float(payload.get('yaw_deg', 0.0)),
            True,
        )
        return api_response(ok, msg)

    @app.post('/api/navigation/cancel')
    def navigation_cancel(payload: dict[str, Any] = Body(default={})) -> dict[str, Any]:
        del payload
        ok, msg = node.cancel_navigation()
        return api_response(ok, msg)

    @app.post('/api/mission/{action}')
    def mission(action: str, payload: dict[str, Any] = Body(default={})) -> dict[str, Any]:
        mapping = {
            'start': 'patrol_start',
            'pause': 'patrol_pause',
            'resume': 'patrol_resume',
            'stop': 'patrol_stop',
            'grasp_once': 'grasp_once',
        }
        if action not in mapping:
            raise HTTPException(status_code=404, detail='unknown mission action')
        del payload
        ok, msg = node.call_trigger(mapping[action], timeout_sec=6.0)
        return api_response(ok, msg)

    @app.post('/api/grasp/enable')
    def grasp_enable(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        enabled = bool(payload.get('enabled'))
        ok, msg = node.call_set_grasp(enabled)
        return api_response(ok, msg)

    @app.post('/api/grasp/once')
    def grasp_once_direct(payload: dict[str, Any] = Body(default={})) -> dict[str, Any]:
        del payload
        ok, msg = node.grasp_once_from_web()
        return api_response(ok, msg)

    @app.get('/api/vlm/models')
    def vlm_models() -> dict[str, Any]:
        return node.vlm_model_snapshot()

    @app.post('/api/vlm/select')
    def vlm_select(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg, data = node.set_vlm_model(str(payload.get('provider') or ''), str(payload.get('model') or ''))
        if ok and bool(payload.get('restart')):
            stop_ok, stop_msg = node.call_trigger('stop_grasp', timeout_sec=20.0)
            start_ok, start_msg = node.call_trigger('start_grasp_dry', timeout_sec=120.0)
            ok = stop_ok and start_ok
            msg = f'{msg}; restart dry VLM stop={stop_ok}:{stop_msg}; start={start_ok}:{start_msg}'
        elif ok:
            refresh_ok, refresh_msg = node.call_trigger('vlm_refresh', timeout_sec=4.0)
            msg = f'{msg}; refresh={refresh_ok}:{refresh_msg}'
        return api_response(ok, msg, **data)

    @app.post('/api/vlm/provider')
    def vlm_provider(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg, data = node.upsert_vlm_provider(
            str(payload.get('provider') or ''),
            str(payload.get('model') or ''),
            str(payload.get('base_url') or ''),
            str(payload.get('api_key_env') or ''),
            str(payload.get('api_key') or ''),
            bool(payload.get('activate', True)),
        )
        if ok and bool(payload.get('restart')):
            stop_ok, stop_msg = node.call_trigger('stop_grasp', timeout_sec=20.0)
            start_ok, start_msg = node.call_trigger('start_grasp_dry', timeout_sec=120.0)
            ok = stop_ok and start_ok
            msg = f'{msg}; restart dry VLM stop={stop_ok}:{stop_msg}; start={start_ok}:{start_msg}'
        elif ok:
            refresh_ok, refresh_msg = node.call_trigger('vlm_refresh', timeout_sec=4.0)
            msg = f'{msg}; refresh={refresh_ok}:{refresh_msg}'
            if str(payload.get('api_key') or '').strip():
                msg = f'{msg}; new API key requires VLM restart to enter the running process environment'
        return api_response(ok, msg, **data)

    @app.post('/api/manager/{component}/{action}')
    def manager(component: str, action: str, payload: dict[str, Any] = Body(default={})) -> dict[str, Any]:
        if component == 'system' and action == 'stop_all':
            ok, msg = node.call_trigger('stop_all', timeout_sec=12.0)
            return api_response(ok, msg)
        key = f'{action}_{component}'
        if component == 'grasp' and action == 'start':
            key = 'start_grasp_live' if payload.get('live') else 'start_grasp_dry'
        timeout_by_key = {
            'start_navigation': 90.0,
            'start_video': 90.0,
            'start_camera': 60.0,
            'start_arm': 120.0,
            'start_grasp_dry': 120.0,
            'start_grasp_live': 120.0,
        }
        timeout_sec = timeout_by_key.get(key, 10.0)
        ok, msg = node.call_trigger(key, timeout_sec=timeout_sec)
        if ok and key == 'start_navigation':
            map_ok, map_msg = node.wait_for_map_ready(timeout_sec=45.0)
            goal_ok, goal_msg = node.wait_for_goal_pose_ready(timeout_sec=15.0)
            ok = map_ok
            msg = f'{msg}; {map_msg}; {goal_msg}'
        return api_response(ok, msg)

    @app.post('/api/rviz_web/{action}')
    def rviz_web(action: str) -> dict[str, Any]:
        if action not in {'start', 'stop', 'restart', 'status'}:
            return api_response(False, 'invalid rviz_web action')
        script = node.root / 'scripts' / 'start_rviz_web.sh'
        if not script.exists():
            return api_response(False, f'missing script: {script}')
        timeout_sec = 180 if action in {'start', 'restart'} else 30
        env = dict(os.environ)
        env.setdefault('TRASH_ROBOT_HOST', '192.168.1.121')
        try:
            result = subprocess.run(
                [str(script), action],
                cwd=str(node.root),
                env=env,
                text=True,
                capture_output=True,
                timeout=timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return api_response(False, f'rviz_web {action} timeout after {timeout_sec}s')
        output = '\n'.join(part for part in (result.stdout.strip(), result.stderr.strip()) if part)
        return api_response(result.returncode == 0, output or f'rviz_web {action} done')

    @app.post('/api/safety/estop')
    def estop() -> dict[str, Any]:
        ok, msg = node.software_estop()
        return api_response(ok, msg)

    @app.post('/api/safety/reset')
    def estop_reset(payload: dict[str, Any] = Body(default={})) -> dict[str, Any]:
        del payload
        ok, msg = node.call_trigger('estop_reset')
        return api_response(ok, msg)

    @app.post('/api/manual/cmd')
    def manual_cmd(payload: dict[str, Any] = Body(...)) -> dict[str, Any]:
        ok, msg = node.manual_cmd(
            float(payload.get('linear', 0.0)),
            float(payload.get('angular', 0.0)),
            float(payload.get('hold_sec', 0.35)),
            True,
        )
        return api_response(ok, msg)

    @app.post('/api/manual/stop')
    def manual_stop() -> dict[str, Any]:
        node.manual_enabled_until = 0.0
        node.publish_zero('web manual stop')
        return api_response(True, 'zero velocity sent')

    @app.get('/api/logs')
    def logs(name: str = 'web', lines: int = 120) -> dict[str, Any]:
        return node.tail_log(name, clamp(int(lines), 20, 300))

    @app.websocket('/ws/telemetry')
    async def telemetry(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                await ws.send_text(json.dumps(node.status_snapshot(), ensure_ascii=False))
                await asyncio.sleep(1.0)
        except WebSocketDisconnect:
            return

    @app.websocket('/ws/nav2d')
    async def nav2d(ws: WebSocket) -> None:
        await ws.accept()
        last_versions: dict[str, int] = {}
        alive = True
        send_lock = asyncio.Lock()

        async def send_json(data: dict[str, Any]) -> None:
            async with send_lock:
                await ws.send_text(json.dumps(data, ensure_ascii=False))

        async def receiver() -> None:
            nonlocal alive
            while alive:
                try:
                    text = await ws.receive_text()
                except WebSocketDisconnect:
                    alive = False
                    return
                try:
                    data = json.loads(text)
                except json.JSONDecodeError:
                    await send_json({'op': 'error', 'message': 'invalid json'})
                    continue
                op = str(data.get('op') or '')
                payload = data.get('payload') if isinstance(data.get('payload'), dict) else data
                if op == 'publish_goal':
                    ok, msg = node.send_nav_goal(
                        float(payload.get('x', 0.0)),
                        float(payload.get('y', 0.0)),
                        float(payload.get('yaw_deg', 0.0)),
                        True,
                    )
                    await send_json({'op': 'command_result', 'command': op, 'ok': ok, 'message': msg})
                elif op == 'publish_initial_pose':
                    ok, msg = node.publish_initial_pose(
                        float(payload.get('x', 0.0)),
                        float(payload.get('y', 0.0)),
                        float(payload.get('yaw_deg', 0.0)),
                    )
                    await send_json({'op': 'command_result', 'command': op, 'ok': ok, 'message': msg})
                elif op == 'cancel_navigation':
                    ok, msg = node.cancel_navigation()
                    await send_json({'op': 'command_result', 'command': op, 'ok': ok, 'message': msg})
                elif op == 'reset_versions':
                    last_versions.clear()
                    await send_json({'op': 'command_result', 'command': op, 'ok': True, 'message': 'versions reset'})

        recv_task = asyncio.create_task(receiver())
        try:
            while alive:
                payload = node.nav2d_payload(last_versions)
                last_versions.update({key: int(value) for key, value in payload.get('seq', {}).items()})
                await send_json(payload)
                await asyncio.sleep(0.2)
        except WebSocketDisconnect:
            alive = False
        finally:
            recv_task.cancel()

    @app.exception_handler(Exception)
    def handle_error(_, exc: Exception) -> JSONResponse:
        return JSONResponse(status_code=500, content={'ok': False, 'message': str(exc)})

    return app


def main(args: list[str] | None = None) -> None:
    if FastAPI is None:
        raise RuntimeError(
            'FastAPI console dependencies are missing. Install with: '
            'python3 -m pip install --user fastapi uvicorn pydantic'
        )
    ensure_robot_dds_environment()
    rclpy.init(args=args)
    node = WebConsoleNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_loop() -> None:
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.05)
            except ExternalShutdownException:
                break
            except Exception as exc:  # noqa: BLE001 - keep WebUI alive and surface the error
                node.get_logger().warning(f'web bridge spin_once failed: {exc}')
                time.sleep(0.05)
            time.sleep(0.2)

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()
    host = str(os.environ.get('TRASH_WEB_HOST') or node.get_parameter('host').value)
    port = int(os.environ.get('TRASH_WEB_PORT') or node.get_parameter('port').value)
    try:
        uvicorn.run(create_app(node), host=host, port=port, log_level='warning', access_log=False)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_zero('web console shutdown')
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
