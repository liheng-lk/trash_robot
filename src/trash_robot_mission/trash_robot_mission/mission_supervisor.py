from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from trash_robot_interfaces.action import SortGrasp
except Exception:  # pragma: no cover - generated action exists after colcon build on RDK.
    SortGrasp = None

from trash_robot_mission.mission_behaviors import create_default_scheduler


def yaw_to_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quat_rotate_vector(qx: float, qy: float, qz: float, qw: float, vec: np.ndarray) -> np.ndarray:
    q_vec = np.array([qx, qy, qz], dtype=np.float64)
    uv = np.cross(q_vec, vec)
    uuv = np.cross(q_vec, uv)
    return vec + 2.0 * (qw * uv + uuv)


class MissionSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('trash_mission_supervisor')

        self.declare_parameter('route_file', '/home/sunrise/trash_robot_v3/config/mission/patrol_routes.yaml')
        self.declare_parameter('sort_config_file', '/home/sunrise/trash_robot_v3/config/grasp/trash_sort_params.yaml')
        self.declare_parameter('auto_start', False)

        self.lock = threading.Lock()
        self.state = 'IDLE'
        self.last_event = 'READY'
        self.active_route = ''
        self.waypoints: list[dict[str, float | str]] = []
        self.waypoint_index = 0
        self.loop_route = True
        self.route_mode = 'closed_loop'
        self.patrol_direction = 1
        self.grasp_enabled = False

        self.arm_target: Optional[np.ndarray] = None
        self.arm_target_stamp = 0.0
        self.camera_target: Optional[np.ndarray] = None
        self.camera_target_msg: Optional[PointStamped] = None
        self.camera_target_frame = ''
        self.camera_target_stamp = 0.0
        self.label = ''
        self.label_stamp = 0.0
        self.local_candidate: dict[str, Any] = {}
        self.local_candidate_stamp = 0.0
        self.vlm_visual_candidate: dict[str, Any] = {}
        self.vlm_visual_candidate_stamp = 0.0
        self.vlm_result: dict[str, Any] = {}
        self.vlm_result_stamp = 0.0
        self.visual_camera_target_msg: Optional[PointStamped] = None
        self.visual_camera_target_stamp = 0.0
        self.grasp_plan: dict[str, Any] = {}
        self.grasp_plan_stamp = 0.0

        self.nav_goal_handle = None
        self.nav_goal_active = False
        self.nav_goal_kind = ''
        self.nav_goal_sequence = 0
        self.grasp_future = None
        self.grasp_result_future = None
        self.grasp_goal_handle = None
        self.grasp_uses_action = False
        self.grasp_start_time = 0.0
        self.local_approach_start_time = 0.0
        self.target_nav_start_time = 0.0
        self.visual_align_start_time = 0.0
        self.visual_align_stable_start_time = 0.0
        self.target_confirm_start_time = 0.0
        self.target_refresh_start_time = 0.0
        self.recovery_start_time = 0.0
        self.patrol_dwell_start_time = 0.0
        self.patrol_dwell_sec = 0.0
        self.patrol_dwell_waypoint = ''
        self.patrol_dwell_route_done = False
        self.stop_until_time = 0.0
        self.resume_waypoint_index = 0
        self.resume_patrol_direction = 1
        self.resume_route_done = False
        self.target_map: Optional[np.ndarray] = None
        self.target_approach_goal: Optional[tuple[float, float, float]] = None
        self.strategy_scheduler = create_default_scheduler()

        self.load_configs()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, '/trash_mission_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(PointStamped, '/trash_target_point_arm', self.arm_target_callback, 10)
        self.create_subscription(PointStamped, '/trash_target_camera_point', self.camera_target_callback, 10)
        self.create_subscription(PointStamped, '/trash_target_point_camera', self.camera_target_callback, 10)
        self.create_subscription(String, '/trash_target_label', self.label_callback, 10)
        self.create_subscription(String, '/trash_grasp_status', self.grasp_status_callback, 10)
        self.create_subscription(String, '/trash_local_candidate', self.local_candidate_callback, 10)
        self.create_subscription(String, '/trash_yolo_candidate', self.local_candidate_callback, 10)
        self.create_subscription(String, '/trash_vlm_result', self.vlm_result_callback, 10)
        self.create_subscription(String, '/trash_grasp_plan', self.grasp_plan_callback, 10)

        self.create_service(Trigger, '/trash_mission/start_patrol', self.start_patrol_callback)
        self.create_service(Trigger, '/trash_mission/stop_patrol', self.stop_patrol_callback)
        self.create_service(Trigger, '/trash_mission/pause_patrol', self.pause_patrol_callback)
        self.create_service(Trigger, '/trash_mission/resume_patrol', self.resume_patrol_callback)
        self.create_service(Trigger, '/trash_mission/reload_route', self.reload_route_callback)
        self.create_service(SetBool, '/trash_mission/set_grasp_enabled', self.set_grasp_enabled_callback)
        self.create_service(Trigger, '/trash_mission/grasp_once', self.grasp_once_callback)
        self.create_service(Trigger, '/trash_safety/stop', self.safety_stop_callback)

        self.grasp_client = self.create_client(Trigger, '/trash_grasp_once')
        self.vlm_clear_client = self.create_client(Trigger, '/trash_vlm/clear_cache')
        self.vlm_refresh_client = self.create_client(Trigger, '/trash_vlm/refresh')
        self.grasp_action_client = ActionClient(self, SortGrasp, '/trash_grasp_sort') if SortGrasp is not None else None
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info(
            f'mission supervisor ready route={self.active_route} waypoints={len(self.waypoints)} '
            f'grasp_enabled={self.grasp_enabled}'
        )

        if bool(self.get_parameter('auto_start').value):
            self.start_patrol('AUTO_START')

    def load_configs(self) -> None:
        route_file = Path(str(self.get_parameter('route_file').value))
        route_data = yaml.safe_load(route_file.read_text(encoding='utf-8')) if route_file.exists() else {}
        if not isinstance(route_data, dict):
            route_data = {}

        mission = route_data.get('mission', {})
        if not isinstance(mission, dict):
            mission = {}
        self.grasp_enabled = bool(mission.get('grasp_enabled_default', False))
        self.target_max_age_sec = float(mission.get('target_max_age_sec', 1.2))
        self.label_max_age_sec = float(mission.get('label_max_age_sec', 1.5))
        self.grasp_timeout_sec = float(mission.get('grasp_timeout_sec', 120.0))
        self.detection_confirm_sec = float(mission.get('detection_confirm_sec', 0.4))
        self.recovery_hold_sec = float(mission.get('recovery_hold_sec', 1.0))
        self.local_approach_timeout_sec = float(mission.get('local_approach_timeout_sec', 8.0))
        self.local_linear_speed = float(mission.get('local_approach_linear_speed', 0.035))
        self.local_angular_gain = float(mission.get('local_approach_angular_gain', 1.2))
        self.local_max_angular = float(mission.get('local_approach_max_angular', 0.18))
        self.camera_center_deadband_m = float(mission.get('camera_center_deadband_m', 0.025))
        self.camera_desired_z_m = float(mission.get('camera_desired_z_m', 0.145))
        self.camera_z_tolerance_m = float(mission.get('camera_z_tolerance_m', 0.018))
        self.camera_grasp_min_z_m = float(
            mission.get('camera_grasp_min_z_m', self.camera_desired_z_m - self.camera_z_tolerance_m)
        )
        self.camera_grasp_max_z_m = float(
            mission.get('camera_grasp_max_z_m', self.camera_desired_z_m + self.camera_z_tolerance_m)
        )
        self.target_nav_enabled = bool(mission.get('target_nav_enabled', True))
        self.target_nav_standoff_m = float(mission.get('target_nav_standoff_m', 0.45))
        self.target_nav_min_travel_m = float(mission.get('target_nav_min_travel_m', 0.18))
        self.target_nav_timeout_sec = float(mission.get('target_nav_timeout_sec', 35.0))
        self.target_nav_tf_timeout_sec = float(mission.get('target_nav_tf_timeout_sec', 0.2))
        self.target_nav_use_local_fallback = bool(mission.get('target_nav_use_local_fallback', True))
        self.target_refresh_timeout_sec = float(mission.get('target_refresh_timeout_sec', 12.0))
        self.target_refresh_min_wait_sec = float(mission.get('target_refresh_min_wait_sec', 0.5))
        self.local_candidate_enabled = bool(mission.get('local_candidate_enabled', False))
        self.local_candidate_max_age_sec = float(mission.get('local_candidate_max_age_sec', 0.8))
        self.local_candidate_min_confidence = float(mission.get('local_candidate_min_confidence', 0.12))
        self.vlm_visual_candidate_enabled = bool(mission.get('vlm_visual_candidate_enabled', True))
        self.vlm_visual_candidate_max_age_sec = float(mission.get('vlm_visual_candidate_max_age_sec', 1.0))
        self.vlm_visual_candidate_max_latency_ms = float(mission.get('vlm_visual_candidate_max_latency_ms', 15000.0))
        self.visual_approach_enabled = bool(mission.get('visual_approach_enabled', True))
        self.visual_approach_linear_speed = float(mission.get('visual_approach_linear_speed', 0.018))
        self.visual_approach_angular_gain = float(mission.get('visual_approach_angular_gain', 0.45))
        self.visual_approach_max_angular = float(mission.get('visual_approach_max_angular', 0.14))
        self.visual_approach_center_deadband_norm = float(mission.get('visual_approach_center_deadband_norm', 0.08))
        self.visual_approach_stop_center_y_norm = float(mission.get('visual_approach_stop_center_y_norm', 0.82))
        self.visual_approach_stop_area_norm = float(mission.get('visual_approach_stop_area_norm', 0.045))
        self.visual_align_timeout_sec = float(mission.get('visual_align_timeout_sec', 5.0))
        self.visual_align_stable_sec = float(mission.get('visual_align_stable_sec', 0.8))
        self.visual_align_no_candidate_wait_sec = float(mission.get('visual_align_no_candidate_wait_sec', 1.2))
        self.advance_patrol_after_grasp = bool(mission.get('advance_patrol_after_grasp', True))
        self.patrol_goal_yaw_mode = str(mission.get('patrol_goal_yaw_mode', 'path')).strip().lower()
        self.patrol_start_skip_radius_m = max(0.0, float(mission.get('patrol_start_skip_radius_m', 0.45)))
        self.default_waypoint_dwell_sec = max(
            0.0,
            float(mission.get('waypoint_dwell_sec', mission.get('patrol_dwell_sec', 0.0))),
        )

        routes = route_data.get('routes', {})
        if not isinstance(routes, dict) or not routes:
            routes = {'origin_hold': {'loop': True, 'waypoints': [{'name': 'origin', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0}]}}
        self.active_route = str(route_data.get('active_route') or next(iter(routes)))
        selected_route = routes.get(self.active_route, {})
        if isinstance(selected_route, dict):
            legacy_loop = bool(selected_route.get('loop', True))
            mode = str(selected_route.get('mode') or '').strip().lower()
            if mode in ('closed', 'loop', 'closed_loop'):
                self.route_mode = 'closed_loop'
            elif mode in ('open', 'open_loop'):
                self.route_mode = 'open_loop'
            elif mode in ('once', 'single'):
                self.route_mode = 'once'
            else:
                self.route_mode = 'closed_loop' if legacy_loop else 'open_loop'
            self.loop_route = self.route_mode == 'closed_loop'
            selected = selected_route.get('waypoints', [])
        else:
            self.loop_route = True
            self.route_mode = 'closed_loop'
            selected = selected_route
        self.waypoints = []
        for item in selected:
            if not isinstance(item, dict):
                continue
            if 'yaw_deg' in item:
                yaw = math.radians(float(item.get('yaw_deg', 0.0)))
            else:
                yaw = float(item.get('yaw', 0.0))
            self.waypoints.append({
                'name': str(item.get('name', f'wp_{len(self.waypoints)}')),
                'x': float(item.get('x', 0.0)),
                'y': float(item.get('y', 0.0)),
                'yaw': yaw,
                'dwell_sec': max(0.0, float(item.get('dwell_sec', self.default_waypoint_dwell_sec))),
            })
        if not self.waypoints:
            self.waypoints = [
                {
                    'name': 'origin',
                    'x': 0.0,
                    'y': 0.0,
                    'yaw': 0.0,
                    'dwell_sec': self.default_waypoint_dwell_sec,
                }
            ]

        sort_file = Path(str(self.get_parameter('sort_config_file').value))
        sort_data = yaml.safe_load(sort_file.read_text(encoding='utf-8')) if sort_file.exists() else {}
        if not isinstance(sort_data, dict):
            sort_data = {}
        window = sort_data.get('grasp_window_mm', {})
        if not isinstance(window, dict):
            window = {}
        self.grasp_min = np.array([
            float(window.get('x', [240.0, 390.0])[0]),
            float(window.get('y', [-180.0, 160.0])[0]),
            float(window.get('z', [-340.0, -110.0])[0]),
        ], dtype=np.float64) / 1000.0
        self.grasp_max = np.array([
            float(window.get('x', [240.0, 390.0])[1]),
            float(window.get('y', [-180.0, 160.0])[1]),
            float(window.get('z', [-340.0, -110.0])[1]),
        ], dtype=np.float64) / 1000.0

    def arm_target_callback(self, msg: PointStamped) -> None:
        with self.lock:
            self.arm_target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
            self.arm_target_stamp = time.time()

    def camera_target_callback(self, msg: PointStamped) -> None:
        with self.lock:
            self.camera_target = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
            self.camera_target_msg = msg
            self.camera_target_frame = str(msg.header.frame_id or '')
            self.camera_target_stamp = time.time()

    def label_callback(self, msg: String) -> None:
        with self.lock:
            self.label = msg.data.strip()
            self.label_stamp = time.time()

    def local_candidate_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        if not bool(data.get('has_candidate', False)):
            with self.lock:
                if time.time() - self.local_candidate_stamp > self.local_candidate_max_age_sec:
                    self.local_candidate = {}
                    self.local_candidate_stamp = 0.0
            return
        camera_msg: Optional[PointStamped] = None
        camera_point = data.get('camera_point_m')
        if isinstance(camera_point, list) and len(camera_point) == 3:
            try:
                camera_msg = PointStamped()
                camera_msg.header.frame_id = str(data.get('camera_frame') or 'camera_color_optical_frame')
                camera_msg.header.stamp = self.get_clock().now().to_msg()
                camera_msg.point.x = float(camera_point[0])
                camera_msg.point.y = float(camera_point[1])
                camera_msg.point.z = float(camera_point[2])
            except (TypeError, ValueError):
                camera_msg = None
        provider = str(data.get('provider') or '').strip().lower()
        if camera_msg is None and provider in ('local_image_blob', 'vlm_visual_blob'):
            return
        label = str(data.get('trash_label') or '').strip()
        if label not in ('GARBAGE_RECYCLE', 'GARBAGE_OTHER', 'GARBAGE_HAZARD', 'GARBAGE_KITCHEN'):
            label = 'GARBAGE_OTHER'
        with self.lock:
            self.local_candidate = data
            self.local_candidate_stamp = time.time()
            if camera_msg is not None:
                self.visual_camera_target_msg = camera_msg
                self.visual_camera_target_stamp = self.local_candidate_stamp
                self.label = label
                self.label_stamp = self.local_candidate_stamp

    def vlm_result_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        now = time.time()
        with self.lock:
            self.vlm_result = data
            self.vlm_result_stamp = now
        if not bool(data.get('has_target', False)):
            return
        label = str(data.get('trash_label') or '').strip()
        if label not in ('GARBAGE_RECYCLE', 'GARBAGE_OTHER', 'GARBAGE_HAZARD', 'GARBAGE_KITCHEN'):
            return
        raw_camera = data.get('camera_point_raw_m')
        camera_msg: Optional[PointStamped] = None
        if isinstance(raw_camera, list) and len(raw_camera) == 3:
            try:
                camera_msg = PointStamped()
                camera_msg.header.frame_id = str(
                    data.get('camera_frame')
                    or data.get('depth_frame_id')
                    or data.get('configured_depth_frame_id')
                    or 'camera_color_optical_frame'
                )
                camera_msg.header.stamp = self.get_clock().now().to_msg()
                camera_msg.point.x = float(raw_camera[0])
                camera_msg.point.y = float(raw_camera[1])
                camera_msg.point.z = float(raw_camera[2])
            except (TypeError, ValueError):
                camera_msg = None
        with self.lock:
            self.vlm_visual_candidate = data
            self.vlm_visual_candidate_stamp = now
            self.label = label
            self.label_stamp = now
            if camera_msg is not None:
                self.visual_camera_target_msg = camera_msg
                self.visual_camera_target_stamp = now

    def grasp_plan_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        now = time.time()
        with self.lock:
            if bool(data.get('has_target', False)):
                self.grasp_plan = data
                self.grasp_plan_stamp = now
            elif now - self.grasp_plan_stamp > self.target_max_age_sec:
                self.grasp_plan = {}
                self.grasp_plan_stamp = 0.0

    def grasp_status_callback(self, msg: String) -> None:
        text = msg.data
        with self.lock:
            if self.state == 'GRASP_SORT' and ('SORT_DONE' in text or 'MANUAL_TRIGGER_DONE ok=True' in text):
                self.apply_resume_waypoint_locked()
                self.last_event = text
                self.state = 'RESUME_PATROL'
            elif self.state == 'GRASP_SORT' and ('SORT_FAILED' in text or 'MANUAL_TRIGGER_DONE ok=False' in text):
                self.last_event = text
                self.state = 'RECOVERY_HOME'
                self.recovery_start_time = time.time()

    def start_patrol_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, msg = self.start_patrol('SERVICE_START')
        response.success = ok
        response.message = msg
        return response

    def stop_patrol_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_patrol('SERVICE_STOP')
        response.success = True
        response.message = 'STOPPED'
        return response

    def pause_patrol_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self.lock:
            self.state = 'IDLE'
            self.last_event = 'PAUSED'
        self.cancel_nav()
        self.publish_stop()
        response.success = True
        response.message = 'PAUSED'
        return response

    def resume_patrol_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, msg = self.start_patrol('SERVICE_RESUME')
        response.success = ok
        response.message = msg
        return response

    def reload_route_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        was_running = False
        with self.lock:
            was_running = self.state not in ('IDLE',)
        self.cancel_nav()
        self.publish_stop()
        self.load_configs()
        with self.lock:
            self.waypoint_index = 0
            self.target_confirm_start_time = 0.0
            self.local_approach_start_time = 0.0
            self.visual_align_start_time = 0.0
            self.visual_align_stable_start_time = 0.0
            self.target_refresh_start_time = 0.0
            self.recovery_start_time = 0.0
            self.target_nav_start_time = 0.0
            self.patrol_dwell_start_time = 0.0
            self.patrol_dwell_sec = 0.0
            self.patrol_dwell_waypoint = ''
            self.patrol_dwell_route_done = False
            self.resume_waypoint_index = 0
            self.resume_patrol_direction = 1
            self.resume_route_done = False
            self.target_map = None
            self.target_approach_goal = None
            self.local_candidate = {}
            self.local_candidate_stamp = 0.0
            self.vlm_visual_candidate = {}
            self.vlm_visual_candidate_stamp = 0.0
            self.vlm_result = {}
            self.vlm_result_stamp = 0.0
            self.visual_camera_target_msg = None
            self.visual_camera_target_stamp = 0.0
            self.grasp_plan = {}
            self.grasp_plan_stamp = 0.0
            self.last_event = f'ROUTE_RELOADED {self.active_route} points={len(self.waypoints)}'
            self.state = 'PATROL_NAVIGATING' if was_running else 'IDLE'
        if was_running:
            self.send_current_nav_goal()
        response.success = True
        response.message = self.last_event
        return response

    def set_grasp_enabled_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        with self.lock:
            self.grasp_enabled = bool(request.data)
            self.last_event = f'GRASP_ENABLED={self.grasp_enabled}'
        response.success = True
        response.message = self.last_event
        return response

    def grasp_once_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, msg = self.trigger_grasp('MANUAL_MISSION_GRASP')
        response.success = ok
        response.message = msg
        return response

    def safety_stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self.lock:
            self.state = 'IDLE'
            self.last_event = 'SAFETY_STOP'
            self.target_confirm_start_time = 0.0
            self.local_approach_start_time = 0.0
            self.visual_align_start_time = 0.0
            self.visual_align_stable_start_time = 0.0
            self.target_refresh_start_time = 0.0
            self.vlm_result = {}
            self.vlm_result_stamp = 0.0
            self.stop_until_time = time.time() + 1.5
        self.cancel_nav()
        self.publish_stop(repeat=10)
        response.success = True
        response.message = 'SAFETY_STOPPED'
        return response

    def start_patrol(self, event: str) -> tuple[bool, str]:
        if self.vlm_clear_client.service_is_ready():
            try:
                self.vlm_clear_client.call_async(Trigger.Request())
            except Exception:
                pass
        with self.lock:
            self.state = 'PATROL_NAVIGATING'
            self.last_event = event
            self.waypoint_index = 0
            self.patrol_direction = 1
            self.target_confirm_start_time = 0.0
            self.local_approach_start_time = 0.0
            self.visual_align_start_time = 0.0
            self.visual_align_stable_start_time = 0.0
            self.target_refresh_start_time = 0.0
            self.recovery_start_time = 0.0
            self.target_nav_start_time = 0.0
            self.patrol_dwell_start_time = 0.0
            self.patrol_dwell_sec = 0.0
            self.patrol_dwell_waypoint = ''
            self.patrol_dwell_route_done = False
            self.resume_waypoint_index = self.waypoint_index
            self.resume_patrol_direction = self.patrol_direction
            self.resume_route_done = False
            self.target_map = None
            self.target_approach_goal = None
            self.local_candidate = {}
            self.local_candidate_stamp = 0.0
            self.vlm_visual_candidate = {}
            self.vlm_visual_candidate_stamp = 0.0
            self.vlm_result = {}
            self.vlm_result_stamp = 0.0
            self.visual_camera_target_msg = None
            self.visual_camera_target_stamp = 0.0
            self.grasp_plan = {}
            self.grasp_plan_stamp = 0.0
        start_index, start_direction, start_reason = self.choose_patrol_start()
        with self.lock:
            if self.waypoints:
                self.waypoint_index = start_index % len(self.waypoints)
            self.patrol_direction = start_direction
            self.resume_waypoint_index = self.waypoint_index
            self.resume_patrol_direction = self.patrol_direction
            self.last_event = f'{event} {start_reason}'
        self.send_current_nav_goal()
        start_name = 'none'
        if self.waypoints:
            start_name = str(self.waypoints[self.waypoint_index % len(self.waypoints)].get('name', self.waypoint_index))
        return True, f'PATROL_STARTED route={self.active_route} mode={self.route_mode} start={start_name} {start_reason}'

    def stop_patrol(self, event: str) -> None:
        with self.lock:
            self.state = 'IDLE'
            self.last_event = event
            self.target_confirm_start_time = 0.0
            self.target_refresh_start_time = 0.0
            self.visual_align_start_time = 0.0
            self.visual_align_stable_start_time = 0.0
            self.patrol_dwell_start_time = 0.0
            self.patrol_dwell_sec = 0.0
            self.patrol_dwell_waypoint = ''
            self.patrol_dwell_route_done = False
            self.patrol_direction = 1
            self.resume_patrol_direction = 1
            self.resume_route_done = False
            self.target_map = None
            self.target_approach_goal = None
            self.local_candidate = {}
            self.local_candidate_stamp = 0.0
            self.vlm_visual_candidate = {}
            self.vlm_visual_candidate_stamp = 0.0
            self.vlm_result = {}
            self.vlm_result_stamp = 0.0
            self.visual_camera_target_msg = None
            self.visual_camera_target_stamp = 0.0
            self.grasp_plan = {}
            self.grasp_plan_stamp = 0.0
        self.cancel_nav()
        self.publish_stop()

    def is_target_fresh(self) -> bool:
        now = time.time()
        with self.lock:
            has_arm = self.arm_target is not None and now - self.arm_target_stamp <= self.target_max_age_sec
            has_label = bool(self.label) and now - self.label_stamp <= self.label_max_age_sec
        return bool(has_arm and has_label)

    def is_local_candidate_fresh(self) -> bool:
        if not self.local_candidate_enabled:
            return False
        now = time.time()
        with self.lock:
            candidate = dict(self.local_candidate)
            age = now - self.local_candidate_stamp
        if not bool(candidate.get('has_candidate', False)) or age > self.local_candidate_max_age_sec:
            return False
        raw_conf = candidate.get('confidence')
        if raw_conf is not None:
            try:
                if float(raw_conf) < self.local_candidate_min_confidence:
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def is_vlm_visual_candidate_fresh(self) -> bool:
        if not self.vlm_visual_candidate_enabled:
            return False
        now = time.time()
        with self.lock:
            candidate = dict(self.vlm_visual_candidate)
            age = now - self.vlm_visual_candidate_stamp
        if not bool(candidate.get('has_target', False)) or age > self.vlm_visual_candidate_max_age_sec:
            return False
        try:
            latency_ms = float(candidate.get('latency_ms') or 0.0)
        except (TypeError, ValueError):
            latency_ms = 0.0
        return bool(latency_ms <= self.vlm_visual_candidate_max_latency_ms)

    def has_target_after_refresh_start(self) -> bool:
        now = time.time()
        with self.lock:
            start = self.target_refresh_start_time
            min_wait = self.target_refresh_min_wait_sec
            has_arm = (
                self.arm_target is not None
                and self.arm_target_stamp > start
                and now - self.arm_target_stamp <= self.target_max_age_sec
            )
            has_camera = (
                self.camera_target is not None
                and self.camera_target_stamp > start
                and now - self.camera_target_stamp <= self.target_max_age_sec
            )
            has_label = bool(self.label) and self.label_stamp > start and now - self.label_stamp <= self.label_max_age_sec
            has_visual_camera = (
                self.visual_camera_target_msg is not None
                and self.visual_camera_target_stamp > start
                and now - self.visual_camera_target_stamp <= self.target_max_age_sec
            )
        return bool(
            start > 0.0
            and now - start >= min_wait
            and has_label
            and ((has_arm and has_camera) or (self.target_nav_enabled and has_visual_camera))
        )

    def clear_vlm_cache_async(self) -> None:
        if not self.vlm_clear_client.service_is_ready():
            return
        try:
            self.vlm_clear_client.call_async(Trigger.Request())
        except Exception:
            pass

    def request_vlm_refresh_async(self) -> None:
        if self.vlm_refresh_client.service_is_ready():
            try:
                self.vlm_refresh_client.call_async(Trigger.Request())
                return
            except Exception as exc:
                with self.lock:
                    self.last_event = f'VLM_REFRESH_CALL_FAILED {exc}'
        self.clear_vlm_cache_async()

    def begin_target_refresh(self, event: str) -> None:
        self.cancel_nav()
        self.publish_stop(repeat=10)
        with self.lock:
            self.state = 'TARGET_REFRESH'
            self.target_refresh_start_time = time.time()
            self.target_map = None
            self.target_approach_goal = None
            self.vlm_result = {}
            self.vlm_result_stamp = 0.0
            self.last_event = event
        self.request_vlm_refresh_async()

    def begin_final_vlm_refresh(self, event: str) -> None:
        self.publish_stop(repeat=10)
        with self.lock:
            self.state = 'FINAL_VLM_REFRESH'
            self.target_refresh_start_time = time.time()
            self.camera_target = None
            self.camera_target_msg = None
            self.camera_target_frame = ''
            self.camera_target_stamp = 0.0
            self.arm_target = None
            self.arm_target_stamp = 0.0
            self.grasp_plan = {}
            self.grasp_plan_stamp = 0.0
            self.vlm_result = {}
            self.vlm_result_stamp = 0.0
            self.last_event = event
        self.request_vlm_refresh_async()

    def begin_visual_align(self, event: str) -> None:
        self.publish_stop(repeat=6)
        with self.lock:
            self.state = 'VISUAL_ALIGN'
            self.visual_align_start_time = time.time()
            self.visual_align_stable_start_time = 0.0
            self.last_event = event

    def begin_recovery(self, event: str) -> None:
        self.publish_stop()
        if self.grasp_goal_handle is not None:
            try:
                self.grasp_goal_handle.cancel_goal_async()
            except Exception:
                pass
        with self.lock:
            self.state = 'RECOVERY_HOME'
            self.last_event = event
            self.recovery_start_time = time.time()

    def is_arm_safe(self, point: Optional[np.ndarray] = None) -> bool:
        with self.lock:
            target = self.arm_target.copy() if point is None and self.arm_target is not None else point
        if target is None:
            return False
        return bool(np.all(target >= self.grasp_min) and np.all(target <= self.grasp_max))

    def is_camera_grasp_distance_ok(self) -> bool:
        now = time.time()
        with self.lock:
            cam = None if self.camera_target is None else self.camera_target.copy()
            cam_age = now - self.camera_target_stamp
        if cam is None or cam_age > self.target_max_age_sec:
            return False
        depth = float(cam[2])
        return bool(self.camera_grasp_min_z_m <= depth <= self.camera_grasp_max_z_m)

    def is_grasp_ready(self) -> bool:
        return bool(self.is_arm_safe() and self.is_camera_grasp_distance_ok())

    def has_vlm_camera_after_refresh_start(self) -> bool:
        now = time.time()
        with self.lock:
            start = self.target_refresh_start_time
            has_camera = (
                self.camera_target is not None
                and self.camera_target_stamp > start
                and now - self.camera_target_stamp <= self.target_max_age_sec
            )
            has_label = bool(self.label) and self.label_stamp > start and now - self.label_stamp <= self.label_max_age_sec
        return bool(start > 0.0 and has_camera and has_label)

    def vlm_no_target_after_refresh_start(self) -> tuple[bool, str]:
        now = time.time()
        with self.lock:
            start = self.target_refresh_start_time
            stamp = self.vlm_result_stamp
            result = dict(self.vlm_result)
        if start <= 0.0 or stamp <= start or now - stamp > max(1.0, self.target_refresh_timeout_sec):
            return False, ''
        if bool(result.get('has_target', False)):
            return False, ''
        reason = str(
            result.get('reason')
            or result.get('reject_reason')
            or result.get('message')
            or result.get('status')
            or 'NO_TARGET'
        )
        return True, reason

    def is_grasp_plan_ready_after_refresh_start(self) -> bool:
        now = time.time()
        with self.lock:
            start = self.target_refresh_start_time
            plan = dict(self.grasp_plan)
            stamp = self.grasp_plan_stamp
        if start <= 0.0 or stamp <= start or now - stamp > self.target_max_age_sec:
            return False
        if not bool(plan.get('has_target', False)):
            return False
        if plan.get('depth_ok') is not True:
            return False
        if plan.get('camera_point_average_ready') is False:
            return False
        return True

    def is_final_vlm_grasp_ready(self) -> bool:
        now = time.time()
        with self.lock:
            start = self.target_refresh_start_time
            arm = None if self.arm_target is None else self.arm_target.copy()
            arm_stamp = self.arm_target_stamp
            cam = None if self.camera_target is None else self.camera_target.copy()
            cam_stamp = self.camera_target_stamp
            label_ok = bool(self.label) and self.label_stamp > start and now - self.label_stamp <= self.label_max_age_sec
        if start <= 0.0 or arm is None or cam is None:
            return False
        if arm_stamp <= start or cam_stamp <= start:
            return False
        if now - arm_stamp > self.target_max_age_sec or now - cam_stamp > self.target_max_age_sec:
            return False
        if not label_ok:
            return False
        depth = float(cam[2])
        return bool(
            self.is_arm_safe(arm)
            and self.camera_grasp_min_z_m <= depth <= self.camera_grasp_max_z_m
            and self.is_grasp_plan_ready_after_refresh_start()
        )

    def next_patrol_index_locked(self) -> tuple[Optional[int], bool]:
        count = len(self.waypoints)
        if count <= 0:
            return None, True
        if count == 1:
            return None, True

        if self.route_mode == 'open_loop':
            next_index = self.waypoint_index + self.patrol_direction
            if next_index >= count:
                self.patrol_direction = -1
                next_index = count - 2
            elif next_index < 0:
                self.patrol_direction = 1
                next_index = 1
            return next_index, False

        next_index = self.waypoint_index + 1
        if self.route_mode == 'once':
            if next_index >= count:
                return None, True
            return next_index, False
        return next_index % count, False

    def set_resume_after_target_locked(self) -> None:
        self.resume_route_done = False
        self.resume_waypoint_index = self.waypoint_index
        self.resume_patrol_direction = self.patrol_direction
        if not self.advance_patrol_after_grasp or not self.waypoints:
            return
        next_index, route_done = self.next_patrol_index_locked()
        if route_done or next_index is None:
            self.resume_route_done = True
            return
        self.resume_waypoint_index = next_index % len(self.waypoints)
        self.resume_patrol_direction = self.patrol_direction

    def apply_resume_waypoint_locked(self) -> None:
        if self.waypoints and not self.resume_route_done:
            self.waypoint_index = self.resume_waypoint_index % len(self.waypoints)
            self.patrol_direction = self.resume_patrol_direction
        self.target_map = None
        self.target_approach_goal = None

    def lookup_transform(self, target_frame: str, source_frame: str, stamp) -> Any:
        timeout = Duration(seconds=max(0.0, self.target_nav_tf_timeout_sec))
        try:
            if stamp.sec or stamp.nanosec:
                return self.tf_buffer.lookup_transform(target_frame, source_frame, Time.from_msg(stamp), timeout=timeout)
        except TransformException:
            pass
        return self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=timeout)

    def transform_point_to_map(self, msg: PointStamped) -> Optional[np.ndarray]:
        source_frame = str(msg.header.frame_id or '').strip()
        if not source_frame:
            with self.lock:
                self.last_event = 'TARGET_NAV_NO_SOURCE_FRAME'
            return None
        try:
            tf = self.lookup_transform('map', source_frame, msg.header.stamp)
        except TransformException as exc:
            with self.lock:
                self.last_event = f'TARGET_NAV_TF_FAILED {source_frame}->map {exc}'
            return None

        trans = tf.transform.translation
        rot = tf.transform.rotation
        point = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        rotated = quat_rotate_vector(rot.x, rot.y, rot.z, rot.w, point)
        return rotated + np.array([trans.x, trans.y, trans.z], dtype=np.float64)

    def robot_xy_in_map(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=max(0.0, self.target_nav_tf_timeout_sec)),
            )
        except TransformException as exc:
            with self.lock:
                self.last_event = f'TARGET_NAV_ROBOT_TF_FAILED {exc}'
            return None
        return np.array([tf.transform.translation.x, tf.transform.translation.y], dtype=np.float64)

    def is_robot_near_waypoint(self, index: int, radius: Optional[float] = None) -> tuple[bool, float]:
        if not self.waypoints:
            return False, float('inf')
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            return False, float('inf')
        wp = self.waypoints[index % len(self.waypoints)]
        distance = math.hypot(float(wp['x']) - float(robot_xy[0]), float(wp['y']) - float(robot_xy[1]))
        threshold = float(self.patrol_start_skip_radius_m if radius is None else radius)
        return distance <= threshold, distance

    def choose_patrol_start(self) -> tuple[int, int, str]:
        count = len(self.waypoints)
        if count <= 1:
            return 0, 1, 'START_SINGLE_WAYPOINT'
        radius = float(getattr(self, 'patrol_start_skip_radius_m', 0.0))
        if radius <= 0.0:
            return 0, 1, 'START_SKIP_DISABLED'

        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            return 0, 1, 'START_NO_ROBOT_POSE'

        nearest_index = 0
        nearest_distance = float('inf')
        for idx, wp in enumerate(self.waypoints):
            dist = math.hypot(float(wp['x']) - float(robot_xy[0]), float(wp['y']) - float(robot_xy[1]))
            if dist < nearest_distance:
                nearest_index = idx
                nearest_distance = dist

        nearest = self.waypoints[nearest_index]
        nearest_name = str(nearest.get('name', nearest_index))
        if nearest_distance > radius:
            return 0, 1, f'START_FROM_FIRST nearest={nearest_name} dist={nearest_distance:.2f}m'

        if self.route_mode == 'open_loop':
            if nearest_index >= count - 1:
                start_index = count - 2
                start_direction = -1
            else:
                start_index = nearest_index + 1
                start_direction = 1
        elif self.route_mode == 'once':
            start_index = min(nearest_index + 1, count - 1)
            start_direction = 1
        else:
            start_index = (nearest_index + 1) % count
            start_direction = 1

        start_name = str(self.waypoints[start_index].get('name', start_index))
        return (
            start_index,
            start_direction,
            f'START_SKIP_NEAR_WAYPOINT nearest={nearest_name} dist={nearest_distance:.2f}m next={start_name}',
        )

    def make_target_approach_pose(self) -> Optional[PoseStamped]:
        now = time.time()
        with self.lock:
            msg = self.camera_target_msg
            camera_age = now - self.camera_target_stamp
            visual_msg = self.visual_camera_target_msg
            visual_age = now - self.visual_camera_target_stamp
        if (msg is None or camera_age > self.target_max_age_sec) and (
            visual_msg is not None and visual_age <= self.target_max_age_sec
        ):
            msg = visual_msg
            camera_age = visual_age
        if msg is None or camera_age > self.target_max_age_sec:
            with self.lock:
                self.last_event = 'TARGET_NAV_NO_FRESH_CAMERA_POINT'
            return None

        target_map = self.transform_point_to_map(msg)
        robot_xy = self.robot_xy_in_map()
        if target_map is None or robot_xy is None:
            return None

        delta = target_map[:2] - robot_xy
        distance = float(np.linalg.norm(delta))
        if distance < 1e-3:
            with self.lock:
                self.last_event = 'TARGET_NAV_ZERO_TARGET_VECTOR'
            return None

        unit = delta / distance
        standoff = clamp(self.target_nav_standoff_m, 0.25, 0.80)
        goal_xy = target_map[:2] - unit * standoff
        travel = float(np.linalg.norm(goal_xy - robot_xy))
        if travel < self.target_nav_min_travel_m:
            with self.lock:
                self.target_map = target_map
                self.target_approach_goal = (float(goal_xy[0]), float(goal_xy[1]), math.atan2(unit[1], unit[0]))
                self.last_event = f'TARGET_NAV_SKIP_CLOSE travel={travel:.2f}'
            return None

        yaw = math.atan2(float(unit[1]), float(unit[0]))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal_xy[0])
        pose.pose.position.y = float(goal_xy[1])
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        with self.lock:
            self.target_map = target_map
            self.target_approach_goal = (float(goal_xy[0]), float(goal_xy[1]), yaw)
        return pose

    def send_current_nav_goal(self) -> None:
        if not self.waypoints:
            with self.lock:
                self.state = 'IDLE'
                self.last_event = 'NO_WAYPOINTS'
            return
        wp_index = self.waypoint_index % len(self.waypoints)
        wp = self.waypoints[wp_index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['x'])
        pose.pose.position.y = float(wp['y'])
        yaw = self.patrol_goal_yaw(wp_index)
        qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.send_nav_goal(pose, 'patrol', f'NAV_GOAL_SENT {wp["name"]} yaw={math.degrees(yaw):.1f}')

    def patrol_goal_yaw(self, target_index: int) -> float:
        count = len(self.waypoints)
        target = self.waypoints[target_index % count]
        recorded_yaw = float(target['yaw'])
        if self.patrol_goal_yaw_mode not in ('path', 'segment', 'auto') or count < 2:
            return recorded_yaw

        direction = self.patrol_direction if self.route_mode == 'open_loop' else 1
        source_index = (target_index - direction) % count
        if self.route_mode == 'once' and target_index <= 0:
            return recorded_yaw
        if self.route_mode == 'open_loop' and target_index == 0 and direction > 0:
            return recorded_yaw
        source = self.waypoints[source_index]
        dx = float(target['x']) - float(source['x'])
        dy = float(target['y']) - float(source['y'])
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return recorded_yaw
        return math.atan2(dy, dx)

    def send_nav_goal(self, pose: PoseStamped, kind: str, event: str) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            with self.lock:
                self.last_event = 'NAV_ACTION_NOT_READY'
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        with self.lock:
            self.nav_goal_sequence += 1
            seq = self.nav_goal_sequence
            self.nav_goal_active = True
            self.nav_goal_kind = kind
            self.last_event = event
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda done, goal_kind=kind, goal_seq=seq: self.nav_goal_response_callback(done, goal_kind, goal_seq))
        return True

    def nav_goal_response_callback(self, future, kind: str, seq: int) -> None:
        with self.lock:
            if seq != self.nav_goal_sequence:
                return
        try:
            goal_handle = future.result()
        except Exception as exc:
            with self.lock:
                if seq != self.nav_goal_sequence:
                    return
                self.nav_goal_active = False
                self.nav_goal_kind = ''
                self.last_event = f'NAV_GOAL_RESPONSE_ERROR {exc}'
            return
        if not goal_handle or not goal_handle.accepted:
            with self.lock:
                if seq != self.nav_goal_sequence:
                    return
                self.nav_goal_active = False
                self.nav_goal_kind = ''
                self.last_event = 'NAV_GOAL_REJECTED'
            return
        with self.lock:
            if seq != self.nav_goal_sequence:
                return
            self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, goal_kind=kind, goal_seq=seq: self.nav_result_callback(done, goal_kind, goal_seq)
        )

    def nav_result_callback(self, future, kind: str, seq: int) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            with self.lock:
                if seq != self.nav_goal_sequence:
                    return
                self.nav_goal_active = False
                self.nav_goal_kind = ''
                self.last_event = f'NAV_RESULT_ERROR {exc}'
            return

        should_send_next_patrol = False
        should_stop_for_dwell = False
        target_succeeded = False
        target_failed_event = ''
        patrol_failure_near_goal = False
        patrol_failure_distance = float('inf')
        if kind == 'patrol' and status != GoalStatus.STATUS_SUCCEEDED:
            with self.lock:
                patrol_index = self.waypoint_index
            patrol_failure_near_goal, patrol_failure_distance = self.is_robot_near_waypoint(patrol_index)
        with self.lock:
            if seq != self.nav_goal_sequence:
                return
            self.nav_goal_active = False
            self.nav_goal_kind = ''
            if kind == 'patrol':
                if self.state != 'PATROL_NAVIGATING':
                    return
                if status == GoalStatus.STATUS_SUCCEEDED or patrol_failure_near_goal:
                    wp = self.waypoints[self.waypoint_index % len(self.waypoints)]
                    wp_name = str(wp.get('name', f'wp_{self.waypoint_index}'))
                    dwell_sec = max(0.0, float(wp.get('dwell_sec', self.default_waypoint_dwell_sec)))
                    next_index, route_done = self.next_patrol_index_locked()
                    reached_event = f'NAV_REACHED_WAYPOINT {wp_name}'
                    if patrol_failure_near_goal and status != GoalStatus.STATUS_SUCCEEDED:
                        reached_event = f'NAV_NEAR_WAYPOINT_AFTER_FAILURE {wp_name} dist={patrol_failure_distance:.2f}m status={status}'
                    if dwell_sec > 0.0:
                        self.state = 'PATROL_DWELL'
                        self.patrol_dwell_start_time = time.time()
                        self.patrol_dwell_sec = dwell_sec
                        self.patrol_dwell_waypoint = wp_name
                        self.patrol_dwell_route_done = route_done
                        if not route_done and next_index is not None:
                            self.waypoint_index = next_index % len(self.waypoints)
                        self.last_event = f'{reached_event} DWELL {dwell_sec:.1f}s'
                        should_stop_for_dwell = True
                    elif route_done or next_index is None:
                        self.state = 'IDLE'
                        self.last_event = 'NAV_ROUTE_DONE'
                        return
                    else:
                        self.waypoint_index = next_index % len(self.waypoints)
                        self.last_event = reached_event
                        should_send_next_patrol = True
                else:
                    self.last_event = f'NAV_DONE status={status}'
                    should_send_next_patrol = True
            elif kind == 'target':
                if self.state != 'TARGET_NAV_APPROACH':
                    return
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.last_event = 'TARGET_NAV_REACHED'
                    target_succeeded = True
                else:
                    target_failed_event = f'TARGET_NAV_DONE status={status}'
            else:
                self.last_event = f'NAV_DONE kind={kind} status={status}'

        if should_stop_for_dwell:
            self.publish_stop(repeat=8)
        elif should_send_next_patrol:
            self.send_current_nav_goal()
        elif target_succeeded:
            self.begin_visual_align('TARGET_NAV_REACHED_ALIGN')
        elif target_failed_event:
            self.begin_visual_align(f'{target_failed_event}_ALIGN')

    def cancel_nav(self) -> None:
        handle = self.nav_goal_handle
        self.nav_goal_handle = None
        self.nav_goal_active = False
        self.nav_goal_kind = ''
        self.nav_goal_sequence += 1
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def publish_stop(self, repeat: int = 3) -> None:
        msg = Twist()
        for _ in range(repeat):
            self.cmd_vel_pub.publish(msg)
        with self.lock:
            self.stop_until_time = max(self.stop_until_time, time.time() + 0.3)

    def trigger_grasp(self, event: str) -> tuple[bool, str]:
        if self.grasp_result_future is not None and not self.grasp_result_future.done():
            return False, 'GRASP_ALREADY_RUNNING'
        if self.grasp_future is not None and not self.grasp_future.done():
            return False, 'GRASP_ALREADY_RUNNING'

        if self.grasp_action_client is not None and self.grasp_action_client.wait_for_server(timeout_sec=0.1):
            goal = SortGrasp.Goal()
            goal.command = event
            self.grasp_uses_action = True
            self.grasp_goal_handle = None
            self.grasp_result_future = None
            self.grasp_future = self.grasp_action_client.send_goal_async(goal, feedback_callback=self.grasp_feedback_callback)
            self.grasp_future.add_done_callback(self.grasp_action_goal_response_callback)
            self.grasp_start_time = time.time()
            with self.lock:
                self.state = 'GRASP_SORT'
                self.last_event = f'{event}_ACTION_SENT'
            return True, event

        self.grasp_uses_action = False
        if not self.grasp_client.wait_for_service(timeout_sec=1.0):
            with self.lock:
                self.last_event = 'GRASP_SERVICE_NOT_READY'
            return False, 'GRASP_SERVICE_NOT_READY'
        self.grasp_future = self.grasp_client.call_async(Trigger.Request())
        self.grasp_start_time = time.time()
        with self.lock:
            self.state = 'GRASP_SORT'
            self.last_event = event
        return True, event

    def grasp_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        with self.lock:
            self.last_event = f'GRASP_FEEDBACK {feedback.stage} {feedback.message}'.strip()

    def grasp_action_goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.begin_recovery(f'GRASP_ACTION_GOAL_ERROR {exc}')
            return
        if not goal_handle or not goal_handle.accepted:
            self.begin_recovery('GRASP_ACTION_REJECTED')
            return
        self.grasp_goal_handle = goal_handle
        self.grasp_result_future = goal_handle.get_result_async()
        self.grasp_result_future.add_done_callback(self.grasp_action_result_callback)

    def grasp_action_result_callback(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.begin_recovery(f'GRASP_ACTION_RESULT_ERROR {exc}')
            return
        with self.lock:
            if self.state != 'GRASP_SORT':
                return
            if bool(result.success):
                self.apply_resume_waypoint_locked()
                self.last_event = result.message or 'GRASP_ACTION_DONE'
                self.state = 'RESUME_PATROL'
            else:
                self.state = 'RECOVERY_HOME'
                self.recovery_start_time = time.time()
                self.last_event = result.message or 'GRASP_ACTION_FAILED'

    def begin_local_candidate_target_nav(self, event: str) -> bool:
        self.cancel_nav()
        self.publish_stop(repeat=8)
        with self.lock:
            self.set_resume_after_target_locked()
            self.target_refresh_start_time = 0.0
        if self.target_nav_enabled and self.start_target_nav_approach(event=f'{event}_NAV_GOAL_SENT'):
            return True

        with self.lock:
            last_event = self.last_event
        if (not self.target_nav_enabled) or last_event.startswith('TARGET_NAV_SKIP_CLOSE'):
            self.begin_visual_align(f'{event}_CLOSE_ALIGN')
            return True

        with self.lock:
            self.state = 'RESUME_PATROL'
            self.last_event = f'{event}_NAV_LOCK_FAILED {last_event}'
        return False

    def start_target_nav_approach(self, event: str = 'TARGET_NAV_GOAL_SENT') -> bool:
        pose = self.make_target_approach_pose()
        if pose is None:
            return False
        with self.lock:
            self.state = 'TARGET_NAV_APPROACH'
            self.target_nav_start_time = time.time()
        ok = self.send_nav_goal(pose, 'target', event)
        if not ok:
            with self.lock:
                self.state = 'STOP_NAV'
        return ok

    def visual_align_tick(self) -> None:
        now = time.time()
        with self.lock:
            start = self.visual_align_start_time
            stable_start = self.visual_align_stable_start_time
            candidate = dict(self.local_candidate)
            age = now - self.local_candidate_stamp
        if start <= 0.0:
            self.begin_visual_align('VISUAL_ALIGN_START')
            return
        if now - start > self.visual_align_timeout_sec:
            self.begin_final_vlm_refresh('VISUAL_ALIGN_TIMEOUT_REFRESH_VLM')
            return

        if not self.visual_approach_enabled:
            self.begin_final_vlm_refresh('VISUAL_ALIGN_DISABLED_REFRESH_VLM')
            return

        if not bool(candidate.get('has_candidate', False)) or age > self.local_candidate_max_age_sec:
            self.publish_stop()
            if now - start >= self.visual_align_no_candidate_wait_sec:
                self.begin_final_vlm_refresh('VISUAL_ALIGN_NO_CANDIDATE_REFRESH_VLM')
            else:
                with self.lock:
                    self.last_event = 'VISUAL_ALIGN_WAIT_CANDIDATE'
            return

        center = candidate.get('center_norm') or [0.5, 0.5]
        try:
            cx = float(center[0])
        except (TypeError, ValueError, IndexError):
            self.publish_stop()
            self.begin_final_vlm_refresh('VISUAL_ALIGN_BAD_CANDIDATE_REFRESH_VLM')
            return

        x_error = cx - 0.5
        if abs(x_error) <= self.visual_approach_center_deadband_norm:
            self.publish_stop()
            if stable_start <= 0.0:
                with self.lock:
                    self.visual_align_stable_start_time = now
                    self.last_event = f'VISUAL_ALIGN_STABLE cx={cx:.2f}'
                return
            if now - stable_start >= self.visual_align_stable_sec:
                self.begin_final_vlm_refresh(f'VISUAL_ALIGN_DONE cx={cx:.2f}_REFRESH_VLM')
                return
            with self.lock:
                self.last_event = f'VISUAL_ALIGN_HOLD cx={cx:.2f}'
            return

        twist = Twist()
        twist.angular.z = clamp(
            -self.visual_approach_angular_gain * x_error,
            -self.visual_approach_max_angular,
            self.visual_approach_max_angular,
        )
        self.cmd_vel_pub.publish(twist)
        with self.lock:
            self.visual_align_stable_start_time = 0.0
            self.last_event = f'VISUAL_ALIGN_ROTATE cx={cx:.2f} err={x_error:.2f}'

    def local_approach_tick(self) -> None:
        now = time.time()
        if now - self.local_approach_start_time > self.local_approach_timeout_sec:
            self.begin_recovery('LOCAL_APPROACH_TIMEOUT')
            return

        with self.lock:
            cam = None if self.camera_target is None else self.camera_target.copy()
            cam_age = now - self.camera_target_stamp
        if cam is None or cam_age > self.target_max_age_sec:
            self.publish_stop()
            with self.lock:
                self.last_event = 'LOCAL_APPROACH_WAIT_VLM_TARGET'
            return

        if self.is_final_vlm_grasp_ready():
            self.publish_stop()
            self.trigger_grasp('LOCAL_APPROACH_GRASP_DISTANCE_OK')
            return

        twist = Twist()
        lateral = float(cam[0])
        depth = float(cam[2])
        if abs(lateral) > self.camera_center_deadband_m:
            twist.angular.z = clamp(-self.local_angular_gain * lateral, -self.local_max_angular, self.local_max_angular)
        if depth > self.camera_desired_z_m + self.camera_z_tolerance_m:
            twist.linear.x = self.local_linear_speed
        elif depth < self.camera_desired_z_m - self.camera_z_tolerance_m:
            twist.linear.x = -self.local_linear_speed * 0.5
        self.cmd_vel_pub.publish(twist)

    def final_vlm_refresh_tick(self) -> None:
        now = time.time()
        with self.lock:
            start = self.target_refresh_start_time
        if start <= 0.0:
            self.begin_final_vlm_refresh('FINAL_VLM_REFRESH_START')
            return
        if now - start > self.target_refresh_timeout_sec:
            with self.lock:
                self.state = 'RESUME_PATROL'
                self.last_event = 'FINAL_VLM_REFRESH_TIMEOUT_RESUME'
                self.target_refresh_start_time = 0.0
            return

        if self.is_final_vlm_grasp_ready():
            self.publish_stop(repeat=8)
            self.trigger_grasp('FINAL_VLM_GRASP_READY')
            return

        if self.has_vlm_camera_after_refresh_start():
            with self.lock:
                self.state = 'LOCAL_APPROACH'
                self.local_approach_start_time = time.time()
                self.last_event = 'FINAL_VLM_TARGET_START_LOCAL_APPROACH'
            return

        no_target, reason = self.vlm_no_target_after_refresh_start()
        if no_target:
            with self.lock:
                self.state = 'RESUME_PATROL'
                self.last_event = f'FINAL_VLM_NO_TARGET_RESUME {reason}'
                self.target_refresh_start_time = 0.0
            return

        self.publish_stop(repeat=3)
        with self.lock:
            self.last_event = 'FINAL_VLM_WAIT_TARGET'

    def visual_candidate_approach_tick(self) -> None:
        if not self.visual_approach_enabled:
            self.publish_stop()
            return

        now = time.time()
        with self.lock:
            candidate = dict(self.local_candidate)
            age = now - self.local_candidate_stamp
        if not bool(candidate.get('has_candidate', False)) or age > self.local_candidate_max_age_sec:
            self.publish_stop()
            with self.lock:
                self.last_event = 'VISUAL_APPROACH_WAIT_CANDIDATE'
            return

        center = candidate.get('center_norm') or [0.5, 0.5]
        bbox = candidate.get('bbox_norm') or [0.0, 0.0, 0.0, 0.0]
        try:
            cx = float(center[0])
            cy = float(center[1])
            area = max(0.0, float(bbox[2]) - float(bbox[0])) * max(0.0, float(bbox[3]) - float(bbox[1]))
        except (TypeError, ValueError, IndexError):
            self.publish_stop()
            with self.lock:
                self.last_event = 'VISUAL_APPROACH_BAD_CANDIDATE'
            return

        twist = Twist()
        x_error = cx - 0.5
        if abs(x_error) > self.visual_approach_center_deadband_norm:
            twist.angular.z = clamp(
                -self.visual_approach_angular_gain * x_error,
                -self.visual_approach_max_angular,
                self.visual_approach_max_angular,
            )

        # Without depth we only creep forward while the blob is still visually
        # high/small. Once it is near the lower image or large, wait for VLM+depth
        # to produce a real camera/arm point before grasping.
        if cy < self.visual_approach_stop_center_y_norm and area < self.visual_approach_stop_area_norm:
            twist.linear.x = max(0.0, self.visual_approach_linear_speed)

        self.cmd_vel_pub.publish(twist)
        with self.lock:
            self.last_event = (
                f'VISUAL_APPROACH provider={candidate.get("provider", "")} '
                f'cx={cx:.2f} cy={cy:.2f} area={area:.3f}'
            )

    def tick(self) -> None:
        with self.lock:
            state = self.state
            should_repeat_stop = time.time() < self.stop_until_time

        if should_repeat_stop:
            self.cmd_vel_pub.publish(Twist())

        self.strategy_scheduler.tick(self, state)

        self.publish_status()

    def status_dict(self) -> dict[str, Any]:
        now = time.time()
        with self.lock:
            arm = None if self.arm_target is None else self.arm_target.copy()
            cam = None if self.camera_target is None else self.camera_target.copy()
            target_map = None if self.target_map is None else self.target_map.copy()
            target_approach_goal = self.target_approach_goal
            label = self.label
            local_candidate = dict(self.local_candidate)
            local_candidate_age = None if not local_candidate else round(now - self.local_candidate_stamp, 3)
            vlm_visual_candidate = dict(self.vlm_visual_candidate)
            vlm_visual_candidate_age = (
                None if not vlm_visual_candidate else round(now - self.vlm_visual_candidate_stamp, 3)
            )
            vlm_result = dict(self.vlm_result)
            vlm_result_age = None if not vlm_result else round(now - self.vlm_result_stamp, 3)
            visual_camera_age = (
                None if self.visual_camera_target_msg is None else round(now - self.visual_camera_target_stamp, 3)
            )
            dwell_remaining = 0.0
            if self.state == 'PATROL_DWELL':
                dwell_remaining = max(0.0, self.patrol_dwell_sec - (now - self.patrol_dwell_start_time))
            data = {
                'state': self.state,
                'last_event': self.last_event,
                'active_route': self.active_route,
                'route_mode': self.route_mode,
                'patrol_goal_yaw_mode': self.patrol_goal_yaw_mode,
                'waypoint_index': self.waypoint_index,
                'waypoint_count': len(self.waypoints),
                'loop_route': self.loop_route,
                'patrol_direction': self.patrol_direction,
                'patrol_start_skip_radius_m': round(float(self.patrol_start_skip_radius_m), 3),
                'waypoint_dwell_sec': round(float(self.default_waypoint_dwell_sec), 2),
                'dwell_waypoint': self.patrol_dwell_waypoint,
                'dwell_remaining_sec': round(dwell_remaining, 2),
                'grasp_enabled': self.grasp_enabled,
                'target_nav_enabled': self.target_nav_enabled,
                'target_nav_standoff_m': round(self.target_nav_standoff_m, 3),
                'nav_goal_kind': self.nav_goal_kind,
                'resume_waypoint_index': self.resume_waypoint_index,
                'label': label,
                'label_age_sec': None if not label else round(now - self.label_stamp, 3),
                'local_candidate_enabled': self.local_candidate_enabled,
                'local_candidate': local_candidate if local_candidate_age is not None else None,
                'local_candidate_age_sec': local_candidate_age,
                'vlm_visual_candidate_enabled': self.vlm_visual_candidate_enabled,
                'vlm_visual_candidate': vlm_visual_candidate if vlm_visual_candidate_age is not None else None,
                'vlm_visual_candidate_age_sec': vlm_visual_candidate_age,
                'vlm_result_has_target': None if vlm_result_age is None else bool(vlm_result.get('has_target', False)),
                'vlm_result_age_sec': vlm_result_age,
                'visual_camera_target_age_sec': visual_camera_age,
                'nav_goal_active': self.nav_goal_active,
            }
        data.update(self.strategy_scheduler.snapshot())
        if arm is not None:
            data['arm_target_mm'] = [round(float(v * 1000.0), 1) for v in arm]
            data['arm_target_age_sec'] = round(now - self.arm_target_stamp, 3)
            data['safe'] = self.is_arm_safe(arm)
        else:
            data['arm_target_mm'] = None
            data['arm_target_age_sec'] = None
            data['safe'] = False
        if cam is not None:
            data['camera_target_m'] = [round(float(v), 4) for v in cam]
            data['camera_target_age_sec'] = round(now - self.camera_target_stamp, 3)
        else:
            data['camera_target_m'] = None
            data['camera_target_age_sec'] = None
        if target_map is not None:
            data['target_map_m'] = [round(float(v), 3) for v in target_map]
        else:
            data['target_map_m'] = None
        if target_approach_goal is not None:
            data['target_approach_goal'] = {
                'x': round(float(target_approach_goal[0]), 3),
                'y': round(float(target_approach_goal[1]), 3),
                'yaw_deg': round(math.degrees(float(target_approach_goal[2])), 1),
            }
        else:
            data['target_approach_goal'] = None
        return data

    def publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps(self.status_dict(), ensure_ascii=False)))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionSupervisor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
