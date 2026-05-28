from __future__ import annotations

import json
import time
import threading
import math
from pathlib import Path
from typing import Any, Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from roarm_moveit.srv import GetPoseCmd, MovePointCmd

try:
    from trash_robot_interfaces.action import SortGrasp
except Exception:  # pragma: no cover - generated action exists after colcon build on RDK.
    SortGrasp = None


DEFAULT_CONFIG_FILE = '/home/sunrise/trash_robot_v3/config/grasp/trash_sort_params.yaml'

DEFAULT_DROP_MM = {
    'GARBAGE_RECYCLE': [-142.7245785, -39.02620123, -110.379442],
    'GARBAGE_OTHER': [-142.1888055, 62.767168, -117.2972858],
    'GARBAGE_HAZARD': [-144.1961226, -138.5567872, -111.390525],
    'GARBAGE_KITCHEN': [-131.2546903, 157.9479884, -127.6844559],
}

DEFAULT_ALIASES = {
    'recycle': 'GARBAGE_RECYCLE',
    'other': 'GARBAGE_OTHER',
    'hazard': 'GARBAGE_HAZARD',
    'kitchen': 'GARBAGE_KITCHEN',
    '可回收': 'GARBAGE_RECYCLE',
    '其他': 'GARBAGE_OTHER',
    '有害': 'GARBAGE_HAZARD',
    '厨余': 'GARBAGE_KITCHEN',
}

ROARM_L2_MM = math.hypot(236.82, 30.00)
ROARM_L3_MM = math.hypot(280.15, 1.73)
DEPTH_HARD_REJECT_REASONS = (
    'DEPTH_COLOR_SIZE_MISMATCH',
    'DEPTH_CAMERA_INFO_FRAME_MISMATCH',
)


class RoArmSortGrasper(Node):
    def __init__(self) -> None:
        super().__init__('roarm_sort_grasper')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('enabled', True)
        self.declare_parameter('auto_grasp', False)
        self.declare_parameter('auto_execute', False)
        self.declare_parameter('offset_file', '/home/sunrise/trash_robot_v3/config/grasp/grasp_offset.yaml')
        self.declare_parameter('sort_config_file', DEFAULT_CONFIG_FILE)
        self.declare_parameter('target_max_age_sec', 1.5)
        self.declare_parameter('label_max_age_sec', 1.5)
        self.declare_parameter('max_grasp_plan_age_sec', 2.0)
        self.declare_parameter('max_camera_point_age_sec', 1.0)
        self.declare_parameter('max_arm_point_age_sec', 1.0)
        self.declare_parameter('cooldown_sec', 5.0)
        self.declare_parameter('clear_vlm_cache_after_sort', True)
        self.declare_parameter('clear_vlm_cache_before_manual_grasp', False)
        self.declare_parameter('vlm_clear_cache_service', '/trash_vlm/clear_cache')

        self.last_target: Optional[np.ndarray] = None
        self.target_history: list[tuple[float, np.ndarray]] = []
        self.last_label = ''
        self.last_raw_label = ''
        self.last_grasp_plan: dict[str, Any] = {}
        self.last_grasp_plan_error = ''
        self.last_grasp_plan_error_stamp = 0.0
        self.last_stamp = 0.0
        self.last_label_stamp = 0.0
        self.last_raw_label_stamp = 0.0
        self.last_grasp_plan_stamp = 0.0
        self.busy = False
        self.cancel_requested = False
        self.busy_lock = threading.Lock()
        self.target_lock = threading.Lock()
        self.last_grasp_time = 0.0
        self.last_target_trace_time = 0.0
        self.last_label_trace_time = 0.0
        self.last_plan_trace_time = 0.0
        self.trace_file = Path('/home/sunrise/trash_robot_v3/runtime/logs/grasp_target_trace.log')
        try:
            self.trace_file.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # noqa: BLE001 - tracing must never block startup
            self.get_logger().warning(f'failed to create grasp trace directory: {exc}')

        self.offset_m = self.load_offset()
        self.load_sort_config()

        self.action_group = ReentrantCallbackGroup()
        self.move_cli = self.create_client(MovePointCmd, '/move_point_cmd', callback_group=self.action_group)
        self.pose_cli = self.create_client(GetPoseCmd, '/get_pose_cmd', callback_group=self.action_group)
        self.vlm_clear_cli = self.create_client(
            Trigger,
            str(self.get_parameter('vlm_clear_cache_service').value),
            callback_group=self.action_group,
        )
        self.gripper_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, '/trash_grasp_status', status_qos)

        self.create_subscription(PointStamped, '/trash_target_point_arm', self.target_callback, 10)
        self.create_subscription(String, '/trash_target_label', self.label_callback, 10)
        self.create_subscription(String, '/trash_target_raw_label', self.raw_label_callback, 10)
        self.create_subscription(String, '/trash_grasp_plan', self.grasp_plan_callback, 10)
        self.create_service(Trigger, '/trash_grasp_once', self.grasp_once_callback)
        self.action_server = None
        if SortGrasp is not None:
            self.action_server = ActionServer(
                self,
                SortGrasp,
                '/trash_grasp_sort',
                execute_callback=self.execute_sort_action,
                goal_callback=self.sort_goal_callback,
                cancel_callback=self.sort_cancel_callback,
                callback_group=self.action_group,
            )
        self.timer = self.create_timer(0.2, self.tick)

        self.publish_status(f'CONFIG_LOADED {self.sort_config_file}')

    def parse_grasp_profile(
        self,
        profile: Any,
        name: str,
        defaults: dict[str, Any],
    ) -> dict[str, Any]:
        if not isinstance(profile, dict):
            profile = {}

        descent_default = defaults.get('descent_offset_m', np.zeros(3, dtype=np.float64)) * 1000.0
        target_default = defaults.get('target_offset_m', np.zeros(3, dtype=np.float64)) * 1000.0
        return {
            'approach_m': self.get_number(profile, 'approach_mm', defaults.get('approach_m', self.approach_m) * 1000.0)
            / 1000.0,
            'descent_offset_m': self.mm_to_m(
                profile.get('descent_offset_mm', profile.get('grasp_descent_offset_mm', descent_default)),
                f'grasp_profiles.{name}.descent_offset_mm',
            ),
            'target_offset_m': self.mm_to_m(
                profile.get('target_offset_mm', target_default),
                f'grasp_profiles.{name}.target_offset_mm',
            ),
            'descent_steps': max(
                1,
                int(
                    self.get_number(
                        profile,
                        'descent_steps',
                        profile.get('grasp_descent_steps', defaults.get('descent_steps', self.grasp_descent_steps)),
                    )
                ),
            ),
            'descent_lock_xy': bool(profile.get('descent_lock_xy', defaults.get('descent_lock_xy', self.descent_lock_xy))),
            'min_descent_m': self.get_number(
                profile,
                'min_descent_mm',
                defaults.get('min_descent_m', self.min_grasp_descent_m) * 1000.0,
            )
            / 1000.0,
            'name': name,
        }

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))
        try:
            stamp = time.strftime('%Y-%m-%d %H:%M:%S')
            with self.trace_file.open('a', encoding='utf-8') as f:
                f.write(f'[{stamp}] {text}\n')
        except Exception:
            pass

    @staticmethod
    def mm_to_m(values: Any, name: str, length: int = 3) -> np.ndarray:
        arr = np.array(values, dtype=np.float64)
        if arr.shape != (length,):
            raise ValueError(f'{name} must contain {length} numbers')
        return arr / 1000.0

    @staticmethod
    def get_number(data: dict[str, Any], key: str, default: float) -> float:
        value = data.get(key, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    @staticmethod
    def parameter_bool(value: object) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def load_offset(self) -> np.ndarray:
        path = Path(str(self.get_parameter('offset_file').value))
        if not path.exists():
            return np.zeros(3, dtype=np.float64)
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        return np.array(data.get('offset_mm', [0, 0, 0]), dtype=np.float64) / 1000.0

    def load_sort_config(self) -> None:
        self.sort_config_file = str(self.get_parameter('sort_config_file').value)
        path = Path(self.sort_config_file)
        data: dict[str, Any] = {}
        if path.exists():
            loaded = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            if isinstance(loaded, dict):
                data = loaded
            else:
                self.get_logger().warning(f'ignored invalid sort config: {path}')
        else:
            self.get_logger().warning(f'sort config not found, using defaults: {path}')

        self.drop_points: dict[str, np.ndarray] = {}
        drop_points = data.get('drop_points_mm', DEFAULT_DROP_MM)
        if not isinstance(drop_points, dict):
            drop_points = DEFAULT_DROP_MM
        for label, point_mm in drop_points.items():
            self.drop_points[str(label)] = self.mm_to_m(point_mm, f'drop_points_mm.{label}')

        aliases = data.get('label_aliases', DEFAULT_ALIASES)
        if not isinstance(aliases, dict):
            aliases = DEFAULT_ALIASES
        self.aliases = {str(k): str(v) for k, v in aliases.items()}

        self.init_pose = self.mm_to_m(data.get('init_pose_mm', [320.0, -1.5, 225.0]), 'init_pose_mm')
        self.safe_drop_pose = self.mm_to_m(data.get('safe_drop_pose_mm', [180.0, 0.0, -50.0]), 'safe_drop_pose_mm')
        self.approach_m = self.get_number(data, 'approach_mm', 60.0) / 1000.0
        self.lift_m = self.get_number(data, 'lift_mm', 80.0) / 1000.0
        self.drop_safe_z_m = self.get_number(data, 'drop_safe_z_mm', 20.0) / 1000.0
        self.grasp_descent_offset = self.mm_to_m(
            data.get('grasp_descent_offset_mm', [0.0, 0.0, 0.0]),
            'grasp_descent_offset_mm',
        )
        self.grasp_descent_steps = max(1, int(self.get_number(data, 'grasp_descent_steps', 1)))
        self.descent_lock_xy = bool(data.get('descent_lock_xy', True))
        self.min_grasp_descent_m = self.get_number(data, 'min_grasp_descent_mm', 20.0) / 1000.0

        global_profile = {
            'approach_m': self.approach_m,
            'descent_offset_m': self.grasp_descent_offset,
            'target_offset_m': np.zeros(3, dtype=np.float64),
            'descent_steps': self.grasp_descent_steps,
            'descent_lock_xy': self.descent_lock_xy,
            'min_descent_m': self.min_grasp_descent_m,
        }
        profiles = data.get('grasp_profiles', {})
        if not isinstance(profiles, dict):
            profiles = {}
        self.grasp_profile_default = self.parse_grasp_profile(
            profiles.get('default', {}),
            'default',
            global_profile,
        )
        by_label = profiles.get('by_label', {})
        if not isinstance(by_label, dict):
            by_label = {}
        self.grasp_profiles_by_label = {
            str(label): self.parse_grasp_profile(profile, f'by_label.{label}', self.grasp_profile_default)
            for label, profile in by_label.items()
        }
        by_raw_keyword = profiles.get('by_raw_keyword', {})
        if not isinstance(by_raw_keyword, dict):
            by_raw_keyword = {}
        self.grasp_profiles_by_raw_keyword = {
            str(keyword).lower(): self.parse_grasp_profile(
                profile,
                f'by_raw_keyword.{keyword}',
                self.grasp_profile_default,
            )
            for keyword, profile in by_raw_keyword.items()
        }
        by_strategy = profiles.get('by_strategy', {})
        if not isinstance(by_strategy, dict):
            by_strategy = {}
        self.grasp_profiles_by_strategy = {
            str(strategy).lower(): self.parse_grasp_profile(
                profile,
                f'by_strategy.{strategy}',
                self.grasp_profile_default,
            )
            for strategy, profile in by_strategy.items()
        }

        window = data.get('pickup_window_mm', data.get('grasp_window_mm', {}))
        if not isinstance(window, dict):
            window = {}
        self.grasp_min = np.array([
            self.mm_to_m(window.get('x', [240.0, 440.0]), 'pickup_window_mm.x', 2)[0],
            self.mm_to_m(window.get('y', [-180.0, 160.0]), 'pickup_window_mm.y', 2)[0],
            self.mm_to_m(window.get('z', [-340.0, -110.0]), 'pickup_window_mm.z', 2)[0],
        ], dtype=np.float64)
        self.grasp_max = np.array([
            self.mm_to_m(window.get('x', [240.0, 440.0]), 'pickup_window_mm.x', 2)[1],
            self.mm_to_m(window.get('y', [-180.0, 160.0]), 'pickup_window_mm.y', 2)[1],
            self.mm_to_m(window.get('z', [-340.0, -110.0]), 'pickup_window_mm.z', 2)[1],
        ], dtype=np.float64)

        gripper = data.get('gripper', {})
        if not isinstance(gripper, dict):
            gripper = {}
        self.open_value = self.get_number(gripper, 'open', 0.8)
        self.close_value = self.get_number(gripper, 'close', 0.0)
        self.gripper_repeat = max(1, int(self.get_number(gripper, 'repeat', 3)))
        self.gripper_repeat_interval_sec = self.get_number(gripper, 'repeat_interval_sec', 0.1)
        self.gripper_hold_sec = self.get_number(gripper, 'hold_sec', 0.8)

        timing = data.get('timing', {})
        if not isinstance(timing, dict):
            timing = {}
        self.target_max_age_sec = self.get_number(
            timing, 'target_max_age_sec', float(self.get_parameter('target_max_age_sec').value)
        )
        self.label_max_age_sec = self.get_number(
            timing, 'label_max_age_sec', float(self.get_parameter('label_max_age_sec').value)
        )
        self.max_grasp_plan_age_sec = self.get_number(
            timing,
            'max_grasp_plan_age_sec',
            float(self.get_parameter('max_grasp_plan_age_sec').value),
        )
        self.max_camera_point_age_sec = self.get_number(
            timing,
            'max_camera_point_age_sec',
            float(self.get_parameter('max_camera_point_age_sec').value),
        )
        self.max_arm_point_age_sec = self.get_number(
            timing,
            'max_arm_point_age_sec',
            float(self.get_parameter('max_arm_point_age_sec').value),
        )
        self.grasp_plan_max_age_sec = self.max_grasp_plan_age_sec
        self.cooldown_sec = self.get_number(timing, 'cooldown_sec', float(self.get_parameter('cooldown_sec').value))
        self.move_service_timeout_sec = self.get_number(timing, 'move_service_timeout_sec', 2.0)
        self.motion_timeout_sec = self.get_number(timing, 'motion_timeout_sec', 25.0)
        self.manual_trigger_wait_sec = self.get_number(timing, 'manual_trigger_wait_sec', 2.0)
        self.manual_trigger_wait_interval_sec = self.get_number(timing, 'manual_trigger_wait_interval_sec', 0.1)

        stability = data.get('target_stability', {})
        if not isinstance(stability, dict):
            stability = {}
        self.target_stability_enabled = bool(stability.get('enabled', True))
        self.target_stability_window_sec = self.get_number(stability, 'window_sec', 1.2)
        self.target_stability_min_samples = max(1, int(self.get_number(stability, 'min_samples', 3)))
        self.target_stability_max_delta_m = self.get_number(stability, 'max_delta_mm', 80.0) / 1000.0
        self.target_stability_jump_reset_m = self.get_number(stability, 'jump_reset_mm', 120.0) / 1000.0

        feedback = data.get('motion_feedback', {})
        if not isinstance(feedback, dict):
            feedback = {}
        self.feedback_enforce_pickup = bool(feedback.get('enforce_pickup', True))
        self.feedback_pickup_xy_tolerance_m = self.get_number(feedback, 'pickup_xy_tolerance_mm', 30.0) / 1000.0
        self.feedback_pickup_z_tolerance_m = self.get_number(feedback, 'pickup_z_tolerance_mm', 45.0) / 1000.0

        compensation = data.get('motion_command_compensation', {})
        if not isinstance(compensation, dict):
            compensation = {}
        self.pickup_command_compensation_enabled = bool(compensation.get('pickup_enabled', False))
        self.pickup_command_compensation_m = self.mm_to_m(
            compensation.get('pickup_offset_mm', [0.0, 0.0, 0.0]),
            'motion_command_compensation.pickup_offset_mm',
        )

        contact_workspace = data.get('contact_workspace', {})
        if not isinstance(contact_workspace, dict):
            contact_workspace = {}
        self.contact_workspace_enabled = bool(contact_workspace.get('enabled', True))
        self.contact_workspace_reject_on_violation = bool(contact_workspace.get('reject_on_violation', False))
        self.contact_workspace_clamp_on_violation = bool(contact_workspace.get('clamp_on_violation', False))
        points = contact_workspace.get('lowest_reliable_z_by_x_mm', [])
        self.contact_workspace_points: list[tuple[float, float]] = []
        if isinstance(points, list):
            for index, item in enumerate(points):
                try:
                    arr = np.array(item, dtype=np.float64)
                except (TypeError, ValueError):
                    continue
                if arr.shape == (2,):
                    self.contact_workspace_points.append((float(arr[0]) / 1000.0, float(arr[1]) / 1000.0))
                else:
                    self.get_logger().warning(f'ignored invalid contact_workspace point #{index}: {item}')
        if not self.contact_workspace_points:
            self.contact_workspace_points = [
                (0.24, -0.38),
                (0.28, -0.35),
                (0.31, -0.315),
                (0.34, -0.300),
                (0.38, -0.285),
            ]
        self.contact_workspace_points.sort(key=lambda item: item[0])
        self.contact_workspace_margin_m = self.get_number(contact_workspace, 'z_margin_mm', 8.0) / 1000.0

        ik = data.get('ik_precheck', {})
        if not isinstance(ik, dict):
            ik = {}
        self.ik_precheck_enabled = bool(ik.get('enabled', True))
        self.ik_reach_margin_m = self.get_number(ik, 'reach_margin_mm', 5.0) / 1000.0

        self.get_logger().info(
            'sort config loaded: '
            f'drops={len(self.drop_points)}, init_mm={self.init_pose * 1000.0}, '
            f'pickup_window_min_mm={self.grasp_min * 1000.0}, pickup_window_max_mm={self.grasp_max * 1000.0}'
        )

    def normalize_label(self, text: str) -> str:
        raw = (text or '').strip()
        if raw in self.drop_points:
            return raw
        return self.aliases.get(raw, self.aliases.get(raw.lower(), raw))

    def label_callback(self, msg: String) -> None:
        label = self.normalize_label(msg.data)
        if label:
            with self.target_lock:
                self.last_label = label
                self.last_label_stamp = time.time()
            now = time.time()
            if now - self.last_label_trace_time > 0.5:
                self.last_label_trace_time = now
                self.publish_status(f'LABEL_UPDATE raw={msg.data} label={label}')

    def raw_label_callback(self, msg: String) -> None:
        raw_label = (msg.data or '').strip()
        with self.target_lock:
            self.last_raw_label = raw_label
            self.last_raw_label_stamp = time.time()

    def grasp_plan_callback(self, msg: String) -> None:
        now = time.time()
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            with self.target_lock:
                if self.last_grasp_plan:
                    self.last_grasp_plan_error = ''
                    self.last_grasp_plan_error_stamp = 0.0
                    message = f'GRASP_PLAN_JSON_INVALID_KEEP_LAST error={exc}'
                else:
                    self.last_grasp_plan = {}
                    self.last_grasp_plan_stamp = 0.0
                    self.last_grasp_plan_error = f'GRASP_PLAN_JSON_INVALID error={exc}'
                    self.last_grasp_plan_error_stamp = now
                    message = self.last_grasp_plan_error
            self.publish_status(message)
            return
        if not isinstance(plan, dict):
            with self.target_lock:
                if self.last_grasp_plan:
                    self.last_grasp_plan_error = ''
                    self.last_grasp_plan_error_stamp = 0.0
                    message = 'GRASP_PLAN_JSON_INVALID_KEEP_LAST reason=not_object'
                else:
                    self.last_grasp_plan = {}
                    self.last_grasp_plan_stamp = 0.0
                    self.last_grasp_plan_error = 'GRASP_PLAN_JSON_INVALID reason=not_object'
                    self.last_grasp_plan_error_stamp = now
                    message = self.last_grasp_plan_error
            self.publish_status(message)
            return
        with self.target_lock:
            if bool(plan.get('has_target', False)):
                self.last_grasp_plan = plan
                self.last_grasp_plan_stamp = now
                self.last_grasp_plan_error = ''
                self.last_grasp_plan_error_stamp = 0.0
            else:
                if self.last_grasp_plan:
                    self.last_grasp_plan_error = ''
                    self.last_grasp_plan_error_stamp = 0.0
                else:
                    self.last_grasp_plan = {}
                    self.last_grasp_plan_stamp = 0.0
                    self.last_grasp_plan_error = 'GRASP_PLAN_NO_TARGET'
                    self.last_grasp_plan_error_stamp = now
        if now - self.last_plan_trace_time > 0.8:
            self.last_plan_trace_time = now
            self.publish_status(
                'GRASP_PLAN_UPDATE '
                f'strategy={plan.get("grasp_strategy") or "NO"} '
                f'type={plan.get("grasp_type") or "NO"} '
                f'shape={plan.get("object_shape") or "NO"} '
                f'width={plan.get("grasp_width_hint") or "NO"} '
                f'raw={plan.get("raw_label") or plan.get("object_name") or "NO"}'
            )

    def target_callback(self, msg: PointStamped) -> None:
        raw = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        target = raw + self.offset_m
        now = time.time()
        with self.target_lock:
            if self.target_history:
                last_history_target = self.target_history[-1][1]
                jump = float(np.linalg.norm(target - last_history_target))
                if jump > self.target_stability_jump_reset_m:
                    self.publish_status(
                        'TARGET_HISTORY_RESET '
                        f'jump_mm={jump*1000.0:.1f} '
                        f'limit_mm={self.target_stability_jump_reset_m*1000.0:.1f} '
                        f'old_mm={last_history_target[0]*1000:.1f},{last_history_target[1]*1000:.1f},{last_history_target[2]*1000:.1f} '
                        f'new_mm={target[0]*1000:.1f},{target[1]*1000:.1f},{target[2]*1000:.1f}'
                    )
                    self.target_history = []
            self.last_target = target
            self.last_stamp = now
            self.target_history.append((now, target.copy()))
            cutoff = now - max(0.1, self.target_stability_window_sec)
            self.target_history = [(stamp, item) for stamp, item in self.target_history if stamp >= cutoff]
        if now - self.last_target_trace_time > 0.5:
            self.last_target_trace_time = now
            self.publish_status(
                'TARGET_UPDATE '
                f'frame={msg.header.frame_id or "NO_FRAME"} '
                f'raw_mm={raw[0]*1000:.1f},{raw[1]*1000:.1f},{raw[2]*1000:.1f} '
                f'offset_mm={self.offset_m[0]*1000:.1f},{self.offset_m[1]*1000:.1f},{self.offset_m[2]*1000:.1f} '
                f'target_mm={target[0]*1000:.1f},{target[1]*1000:.1f},{target[2]*1000:.1f}'
            )

    def is_grasp_safe(self, point: np.ndarray) -> bool:
        return bool(np.all(point >= self.grasp_min) and np.all(point <= self.grasp_max))

    def stable_target_from_history(
        self,
        now: float,
        fallback: np.ndarray,
    ) -> tuple[bool, str, np.ndarray]:
        if not self.target_stability_enabled:
            return True, 'TARGET_STABILITY_DISABLED', fallback

        cutoff = now - max(0.1, self.target_stability_window_sec)
        history = [(stamp, item.copy()) for stamp, item in self.target_history if stamp >= cutoff]
        if len(history) < self.target_stability_min_samples:
            return (
                False,
                f'TARGET_NOT_STABLE samples={len(history)}/{self.target_stability_min_samples}',
                fallback,
            )

        samples = np.array([item for _, item in history], dtype=np.float64)
        latest_deltas = np.linalg.norm(samples - fallback, axis=1)
        cluster_mask = latest_deltas <= self.target_stability_max_delta_m
        if int(np.count_nonzero(cluster_mask)) >= self.target_stability_min_samples:
            samples = samples[cluster_mask]
        elif len(history) >= self.target_stability_min_samples:
            return (
                False,
                (
                    f'TARGET_NOT_CLUSTERED near_samples={int(np.count_nonzero(cluster_mask))}/'
                    f'{self.target_stability_min_samples} '
                    f'limit_mm={self.target_stability_max_delta_m*1000.0:.1f} '
                    f'last_mm={fallback[0]*1000:.1f},{fallback[1]*1000:.1f},{fallback[2]*1000:.1f}'
                ),
                fallback,
            )
        median = np.median(samples, axis=0)
        deltas = np.linalg.norm(samples - median, axis=1)
        max_delta = float(np.max(deltas))
        if max_delta > self.target_stability_max_delta_m:
            return (
                False,
                (
                    f'TARGET_UNSTABLE max_delta_mm={max_delta*1000.0:.1f} '
                    f'limit_mm={self.target_stability_max_delta_m*1000.0:.1f} '
                    f'median_mm={median[0]*1000:.1f},{median[1]*1000:.1f},{median[2]*1000:.1f} '
                    f'last_mm={fallback[0]*1000:.1f},{fallback[1]*1000:.1f},{fallback[2]*1000:.1f}'
                ),
                fallback,
            )

        return True, 'TARGET_STABLE', median

    def ik_precheck(self, point: np.ndarray, name: str) -> tuple[bool, str]:
        if not self.ik_precheck_enabled:
            return True, 'IK_PRECHECK_DISABLED'

        planar_m = math.hypot(float(point[0]), float(point[1]))
        distance_m = math.hypot(planar_m, float(point[2]))
        max_reach_m = (ROARM_L2_MM + ROARM_L3_MM) / 1000.0 - self.ik_reach_margin_m
        min_reach_m = abs(ROARM_L3_MM - ROARM_L2_MM) / 1000.0 + self.ik_reach_margin_m
        if distance_m > max_reach_m or distance_m < min_reach_m:
            return (
                False,
                (
                    f'IK_PRECHECK_FAILED stage={name} '
                    f'point_mm={point[0]*1000:.1f},{point[1]*1000:.1f},{point[2]*1000:.1f} '
                    f'distance_mm={distance_m*1000.0:.1f} '
                    f'allowed_mm={min_reach_m*1000.0:.1f}-{max_reach_m*1000.0:.1f}'
                ),
            )
        return True, 'IK_PRECHECK_OK'

    def precheck_motion_points(self, points: list[tuple[str, np.ndarray]]) -> tuple[bool, str]:
        for name, point in points:
            # For pickup contact, report the more actionable workspace reason
            # first. By default this is diagnostic only: the real execution
            # guard is the /get_pose_cmd feedback check before gripper close.
            if name == 'grasp' or name.startswith('grasp_'):
                ok, reason = self.contact_workspace_precheck(point, name)
                if not ok:
                    return False, reason
            ok, reason = self.ik_precheck(point, name)
            if not ok:
                return False, reason
            if name != 'grasp' and not name.startswith('grasp_'):
                ok, reason = self.contact_workspace_precheck(point, name)
                if not ok:
                    return False, reason
            command_point = self.command_point_for_stage(point, name)
            if not np.allclose(command_point, point):
                ok, reason = self.ik_precheck(command_point, f'{name}_command')
                if not ok:
                    return False, reason
        return True, 'IK_PRECHECK_OK'

    def lowest_reliable_contact_z(self, x_m: float) -> float:
        points = self.contact_workspace_points
        if not points:
            return -1.0
        x = float(x_m)
        if x <= points[0][0]:
            return points[0][1]
        if x >= points[-1][0]:
            return points[-1][1]
        for (x0, z0), (x1, z1) in zip(points, points[1:]):
            if x0 <= x <= x1:
                alpha = (x - x0) / max(1e-6, x1 - x0)
                return z0 + (z1 - z0) * alpha
        return points[-1][1]

    def max_reliable_x_for_contact_z(self, z_m: float) -> float:
        points = self.contact_workspace_points
        if not points:
            return 0.0
        z = float(z_m)
        # Lower Z is harder. Return the farthest X whose calibrated low-Z limit
        # is still at or below the requested contact Z.
        candidates = [x for x, limit_z in points if limit_z <= z]
        if candidates:
            return max(candidates)
        return points[0][0]

    def contact_workspace_precheck(self, point: np.ndarray, name: str) -> tuple[bool, str]:
        if not self.contact_workspace_enabled:
            return True, 'CONTACT_WORKSPACE_DISABLED'
        if name != 'grasp' and not name.startswith('grasp_'):
            return True, 'CONTACT_WORKSPACE_SKIPPED'
        x = float(point[0])
        z = float(point[2])
        lowest_z = self.lowest_reliable_contact_z(x)
        allowed_z = lowest_z - self.contact_workspace_margin_m
        if z < allowed_z:
            suggested_x = self.max_reliable_x_for_contact_z(z)
            approach_mm = max(0.0, (x - suggested_x) * 1000.0)
            reason = (
                f'PICKUP_CONTACT_WORKSPACE_WARN stage={name} '
                f'point_mm={point[0]*1000:.1f},{point[1]*1000:.1f},{point[2]*1000:.1f} '
                f'lowest_reliable_z_mm={lowest_z*1000:.1f} '
                f'margin_mm={self.contact_workspace_margin_m*1000.0:.1f} '
                f'suggested_arm_x_max_mm={suggested_x*1000.0:.1f} '
                f'need_base_approach_mm={approach_mm:.1f}'
            )
            self.publish_status(reason)
            if self.contact_workspace_reject_on_violation:
                return False, reason.replace('PICKUP_CONTACT_WORKSPACE_WARN', 'PICKUP_CONTACT_WORKSPACE_FAILED', 1)
            return True, reason
        return True, 'CONTACT_WORKSPACE_OK'

    def clamp_contact_workspace_z(self, point: np.ndarray, name: str) -> np.ndarray:
        """Keep pickup contact poses inside the empirically reliable low-Z workspace.

        RoArm can accept a deep command near the front reach and still solve to a
        forward/up pose. Clamping the contact Z before execution avoids that
        unsafe drift while preserving the VLM/depth X/Y target.
        """
        clamped = np.array(point, dtype=np.float64).copy()
        if (
            not self.contact_workspace_enabled
            or not self.contact_workspace_clamp_on_violation
            or (name != 'grasp' and not name.startswith('grasp_'))
        ):
            return clamped

        lowest_z = self.lowest_reliable_contact_z(float(clamped[0]))
        # Z is positive upward in this arm frame. Keep a margin above the lowest
        # reliable value; going more negative is where the SDK/IK starts to drift.
        min_reliable_z = lowest_z + self.contact_workspace_margin_m
        if float(clamped[2]) < min_reliable_z:
            old_z = float(clamped[2])
            clamped[2] = min_reliable_z
            self.publish_status(
                (
                    f'PICKUP_CONTACT_WORKSPACE_CLAMP stage={name} '
                    f'x_mm={clamped[0]*1000.0:.1f} '
                    f'old_z_mm={old_z*1000.0:.1f} '
                    f'new_z_mm={clamped[2]*1000.0:.1f} '
                    f'lowest_reliable_z_mm={lowest_z*1000.0:.1f} '
                    f'margin_mm={self.contact_workspace_margin_m*1000.0:.1f}'
                )
            )
        return clamped

    def publish_gripper(self, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        if self.parameter_bool(self.get_parameter('dry_run').value):
            self.publish_status(f'DRY_GRIPPER_CMD value={float(value):.3f} repeat={self.gripper_repeat}')
            return
        self.publish_status(f'GRIPPER_CMD value={float(value):.3f} repeat={self.gripper_repeat}')
        for _ in range(self.gripper_repeat):
            self.gripper_pub.publish(msg)
            time.sleep(self.gripper_repeat_interval_sec)

    @staticmethod
    def is_pickup_motion_stage(name: str) -> bool:
        return (
            name == 'pre_grasp'
            or name == 'grasp'
            or name.startswith('grasp_')
        )

    def command_point_for_stage(self, expected_point: np.ndarray, name: str) -> np.ndarray:
        command_point = np.array(expected_point, dtype=np.float64).copy()
        if self.pickup_command_compensation_enabled and self.is_pickup_motion_stage(name):
            command_point += self.pickup_command_compensation_m
        return command_point

    def clear_vlm_cache_after_success(self) -> None:
        if not bool(self.get_parameter('clear_vlm_cache_after_sort').value):
            return
        if not self.vlm_clear_cli.service_is_ready() and not self.vlm_clear_cli.wait_for_service(timeout_sec=0.1):
            self.publish_status('VLM_CACHE_CLEAR_SKIPPED service_not_ready')
            return

        future = self.vlm_clear_cli.call_async(Trigger.Request())

        def done(done_future) -> None:
            try:
                result = done_future.result()
                ok = bool(result and result.success)
                message = getattr(result, 'message', '')
                self.publish_status(f'VLM_CACHE_CLEAR_DONE ok={ok} msg={message}')
            except Exception as exc:  # noqa: BLE001 - cache cleanup must not affect arm recovery
                self.publish_status(f'VLM_CACHE_CLEAR_FAILED error={exc}')

        future.add_done_callback(done)

    def reset_target_state_for_new_grasp(self) -> None:
        with self.target_lock:
            self.last_target = None
            self.last_stamp = 0.0
            self.target_history = []
            self.last_label = ''
            self.last_label_stamp = 0.0
            self.last_raw_label = ''
            self.last_raw_label_stamp = 0.0
            self.last_grasp_plan = {}
            self.last_grasp_plan_stamp = 0.0
            self.last_grasp_plan_error = ''
            self.last_grasp_plan_error_stamp = 0.0
        self.publish_status('TARGET_STATE_CLEARED_FOR_NEW_GRASP')

    def clear_vlm_cache_before_manual_grasp(self) -> None:
        if not bool(self.get_parameter('clear_vlm_cache_before_manual_grasp').value):
            return
        self.reset_target_state_for_new_grasp()
        if not self.vlm_clear_cli.service_is_ready() and not self.vlm_clear_cli.wait_for_service(timeout_sec=0.5):
            self.publish_status('VLM_CACHE_CLEAR_BEFORE_GRASP_SKIPPED service_not_ready')
            return
        future = self.vlm_clear_cli.call_async(Trigger.Request())
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if future.done():
                try:
                    result = future.result()
                    ok = bool(result and result.success)
                    message = getattr(result, 'message', '')
                    self.publish_status(f'VLM_CACHE_CLEAR_BEFORE_GRASP_DONE ok={ok} msg={message}')
                except Exception as exc:  # noqa: BLE001
                    self.publish_status(f'VLM_CACHE_CLEAR_BEFORE_GRASP_FAILED error={exc}')
                return
            time.sleep(0.02)
        self.publish_status('VLM_CACHE_CLEAR_BEFORE_GRASP_TIMEOUT')

    def move_to(self, point: np.ndarray, name: str) -> bool:
        expected_point = np.array(point, dtype=np.float64).copy()
        command_point = self.command_point_for_stage(expected_point, name)
        command_base_yaw = math.atan2(-float(command_point[1]), float(command_point[0]))
        planar_radius = math.hypot(float(command_point[0]), float(command_point[1]))
        linkage_radius = math.hypot(planar_radius, float(command_point[2]))
        compensation_mm = (command_point - expected_point) * 1000.0
        self.publish_status(
            (
                f'MOVE_CMD {name}: '
                f'expected_mm={expected_point[0]*1000:.1f},{expected_point[1]*1000:.1f},{expected_point[2]*1000:.1f} '
                f'command_mm={command_point[0]*1000:.1f},{command_point[1]*1000:.1f},{command_point[2]*1000:.1f} '
                f'compensation_mm={compensation_mm[0]:.1f},{compensation_mm[1]:.1f},{compensation_mm[2]:.1f} '
                f'cmd_base_yaw_deg={math.degrees(command_base_yaw):.1f} '
                f'linkage_radius_mm={linkage_radius*1000.0:.1f}'
            )
        )

        if self.parameter_bool(self.get_parameter('dry_run').value):
            self.publish_status(
                (
                    f'DRY {name}: '
                    f'expected_mm={expected_point[0]*1000:.1f},{expected_point[1]*1000:.1f},{expected_point[2]*1000:.1f} '
                    f'command_mm={command_point[0]*1000:.1f},{command_point[1]*1000:.1f},{command_point[2]*1000:.1f}'
                )
            )
            return True

        if not self.move_cli.wait_for_service(timeout_sec=self.move_service_timeout_sec):
            self.publish_status('MOVE_SERVICE_NOT_READY')
            return False

        req = MovePointCmd.Request()
        req.x = float(command_point[0])
        req.y = float(command_point[1])
        req.z = float(command_point[2])

        future = self.move_cli.call_async(req)
        start = time.time()
        while rclpy.ok() and not future.done() and time.time() - start < self.motion_timeout_sec:
            time.sleep(0.05)

        if not future.done():
            self.publish_status(f'MOVE_TIMEOUT {name}')
            return False
        result = future.result()
        ok = bool(result and result.success)
        self.publish_status(f'MOVE {name} ok={ok} msg={getattr(result, "message", "")}')
        if ok:
            ok = self.publish_move_feedback(name, expected_point, command_point)
        return ok

    def publish_move_feedback(self, name: str, expected_point: np.ndarray, command_point: np.ndarray) -> bool:
        if not self.pose_cli.wait_for_service(timeout_sec=0.15):
            self.publish_status(f'MOVE_FEEDBACK {name} pose_service_not_ready')
            return True

        future = self.pose_cli.call_async(GetPoseCmd.Request())
        start = time.time()
        while rclpy.ok() and not future.done() and time.time() - start < 1.5:
            time.sleep(0.02)

        if not future.done():
            self.publish_status(f'MOVE_FEEDBACK {name} timeout')
            return True

        try:
            pose = future.result()
        except Exception as exc:  # noqa: BLE001 - diagnostics must not break grasp flow
            self.publish_status(f'MOVE_FEEDBACK {name} error={exc}')
            return True

        actual = np.array([float(pose.x), float(pose.y), float(pose.z)], dtype=np.float64)
        error = actual - expected_point
        xy_error = math.hypot(float(error[0]), float(error[1]))
        z_error = abs(float(error[2]))
        self.publish_status(
            (
                f'MOVE_FEEDBACK {name}: '
                f'expected_mm={expected_point[0]*1000:.1f},{expected_point[1]*1000:.1f},{expected_point[2]*1000:.1f} '
                f'command_mm={command_point[0]*1000:.1f},{command_point[1]*1000:.1f},{command_point[2]*1000:.1f} '
                f'actual_mm={actual[0]*1000:.1f},{actual[1]*1000:.1f},{actual[2]*1000:.1f} '
                f'error_mm={error[0]*1000:.1f},{error[1]*1000:.1f},{error[2]*1000:.1f}'
            )
        )
        if (
            self.feedback_enforce_pickup
            and self.is_pickup_motion_stage(name)
            and (xy_error > self.feedback_pickup_xy_tolerance_m or z_error > self.feedback_pickup_z_tolerance_m)
        ):
            self.publish_status(
                (
                    f'MOVE_FEEDBACK_OUT_OF_TOLERANCE {name}: '
                    f'xy_error_mm={xy_error*1000.0:.1f} '
                    f'z_error_mm={z_error*1000.0:.1f} '
                    f'limits_mm={self.feedback_pickup_xy_tolerance_m*1000.0:.1f},'
                    f'{self.feedback_pickup_z_tolerance_m*1000.0:.1f}'
                )
            )
            return False
        return True

    def publish_stage(self, stage: str, message: str = '', feedback_cb=None) -> None:
        text = f'STAGE {stage}' if not message else f'STAGE {stage} {message}'
        self.publish_status(text)
        if feedback_cb is not None:
            feedback_cb(stage, message)

    def move_descent(self, start: np.ndarray, end: np.ndarray, name: str, steps: Optional[int] = None) -> bool:
        steps = max(1, int(self.grasp_descent_steps if steps is None else steps))
        self.publish_status(
            (
                f'DESCENT_PLAN {name}: '
                f'start_mm={start[0]*1000:.1f},{start[1]*1000:.1f},{start[2]*1000:.1f} '
                f'end_mm={end[0]*1000:.1f},{end[1]*1000:.1f},{end[2]*1000:.1f} '
                f'steps={steps}'
            )
        )
        for index in range(1, steps + 1):
            alpha = index / float(steps)
            point = start + (end - start) * alpha
            step_name = name if steps == 1 else f'{name}_{index}/{steps}'
            if not self.move_to(point, step_name):
                return False
        return True

    def drop_point(self) -> Optional[np.ndarray]:
        if self.last_label not in self.drop_points:
            return None
        return self.drop_points[self.last_label].copy()

    def validate_current_grasp_plan(self) -> tuple[bool, str, dict[str, Any]]:
        now = time.time()
        with self.target_lock:
            plan = dict(self.last_grasp_plan)
            plan_stamp = self.last_grasp_plan_stamp
            plan_error = self.last_grasp_plan_error

        if plan_error:
            return False, plan_error, {}
        if not plan or plan_stamp <= 0.0:
            return False, 'NO_GRASP_PLAN', {}
        plan_age = now - plan_stamp
        if plan_age > self.max_grasp_plan_age_sec:
            return (
                False,
                f'GRASP_PLAN_TOO_OLD age={plan_age:.2f}s max={self.max_grasp_plan_age_sec:.2f}s',
                plan,
            )
        if 'depth_ok' not in plan:
            return False, 'DEPTH_OK_MISSING', plan
        if plan.get('depth_ok') is not True:
            return False, f'DEPTH_NOT_OK reason={plan.get("depth_reason") or "NO_REASON"}', plan
        if plan.get('camera_point_average_ready') is False:
            samples = plan.get('camera_point_average_samples', 0)
            required = plan.get('camera_point_average_required_samples', 2)
            reason = plan.get('camera_point_average_reason') or 'WAIT_AVERAGE'
            return False, f'CAMERA_POINT_AVERAGE_WAIT samples={samples}/{required} reason={reason}', plan
        depth_reason = str(plan.get('depth_reason') or '')
        for token in DEPTH_HARD_REJECT_REASONS:
            if token in depth_reason:
                return False, f'DEPTH_HARD_REJECT reason={depth_reason}', plan
        depth_age = plan.get('depth_age_sec')
        try:
            depth_age_sec = float(depth_age)
        except (TypeError, ValueError):
            return False, 'CAMERA_POINT_AGE_UNKNOWN', plan
        camera_point_age_sec = plan_age + depth_age_sec
        if camera_point_age_sec > self.max_camera_point_age_sec:
            return (
                False,
                f'CAMERA_POINT_TOO_OLD age={camera_point_age_sec:.2f}s max={self.max_camera_point_age_sec:.2f}s',
                plan,
            )
        return True, 'OK', plan

    def validate_target(self) -> tuple[bool, str, str, Optional[np.ndarray], float, float, str]:
        now = time.time()
        if not self.parameter_bool(self.get_parameter('enabled').value):
            return False, 'GRASPER_DISABLED', '', None, 0.0, 0.0, ''
        plan_ok, plan_reason, _ = self.validate_current_grasp_plan()
        with self.target_lock:
            target = None if self.last_target is None else self.last_target.copy()
            target_stamp = self.last_stamp
            label = self.last_label
            label_stamp = self.last_label_stamp
            raw_label = self.last_raw_label

        if not plan_ok:
            return False, plan_reason, label, target, target_stamp, label_stamp, raw_label

        if target is None:
            return False, 'NO_TARGET', '', None, 0.0, 0.0, raw_label
        target_age = now - target_stamp
        if target_age > self.max_arm_point_age_sec:
            return (
                False,
                f'ARM_POINT_TOO_OLD age={target_age:.2f}s max={self.max_arm_point_age_sec:.2f}s',
                label,
                target,
                target_stamp,
                label_stamp,
                raw_label,
            )

        stable_ok, stable_reason, stable_target = self.stable_target_from_history(now, target)
        if not stable_ok:
            return False, stable_reason, label, target, target_stamp, label_stamp, raw_label
        target = stable_target

        if not label:
            return False, 'NO_LABEL', '', target, target_stamp, label_stamp, raw_label
        label_age = now - label_stamp
        if label_age > self.label_max_age_sec:
            return False, f'LABEL_TOO_OLD age={label_age:.2f}s label={label}', label, target, target_stamp, label_stamp, raw_label
        if label not in self.drop_points:
            return False, f'UNKNOWN_LABEL {label}', label, target, target_stamp, label_stamp, raw_label

        if not self.is_grasp_safe(target):
            return (
                False,
                f'OUT_OF_RANGE {target[0]*1000:.1f},'
                f'{target[1]*1000:.1f},{target[2]*1000:.1f}mm',
                label,
                target,
                target_stamp,
                label_stamp,
                raw_label,
            )

        return True, 'OK', label, target, target_stamp, label_stamp, raw_label

    @staticmethod
    def is_retryable_validation_reason(reason: str) -> bool:
        retry_prefixes = (
            'NO_TARGET',
            'NO_GRASP_PLAN',
            'GRASP_PLAN_NO_TARGET',
            'TARGET_TOO_OLD',
            'ARM_POINT_TOO_OLD',
            'TARGET_NOT_STABLE',
            'TARGET_NOT_CLUSTERED',
            'TARGET_UNSTABLE',
            'CAMERA_POINT_AVERAGE_WAIT',
            'NO_LABEL',
            'LABEL_TOO_OLD',
        )
        return any((reason or '').startswith(prefix) for prefix in retry_prefixes)

    def wait_for_valid_target(self) -> tuple[bool, str, str, Optional[np.ndarray], float, float, str]:
        deadline = time.time() + max(0.0, self.manual_trigger_wait_sec)
        last_result = self.validate_target()
        if last_result[0] or not self.is_retryable_validation_reason(last_result[1]):
            return last_result

        self.publish_status(
            (
                f'MANUAL_TRIGGER_WAIT_TARGET reason={last_result[1]} '
                f'timeout_sec={self.manual_trigger_wait_sec:.1f}'
            )
        )
        while time.time() < deadline:
            time.sleep(max(0.02, self.manual_trigger_wait_interval_sec))
            last_result = self.validate_target()
            if last_result[0]:
                self.publish_status('MANUAL_TRIGGER_TARGET_READY')
                return last_result
            if not self.is_retryable_validation_reason(last_result[1]):
                return last_result
        return last_result

    def tick(self) -> None:
        if (
            self.busy
            or not self.parameter_bool(self.get_parameter('auto_grasp').value)
            or not self.parameter_bool(self.get_parameter('auto_execute').value)
        ):
            return
        ok, reason, _, _, _, _, _ = self.validate_target()
        if not ok:
            self.status_pub.publish(String(data=reason))
            return
        if time.time() - self.last_grasp_time < self.cooldown_sec:
            return
        self.sort_once()

    def current_target_summary(self) -> str:
        with self.target_lock:
            target = None if self.last_target is None else self.last_target.copy()
            target_stamp = self.last_stamp
            label = self.last_label
            label_stamp = self.last_label_stamp
            raw_label = self.last_raw_label
        if target is None:
            return 'target=NO'
        return self.format_target_summary(label, target, target_stamp, label_stamp, raw_label)

    def format_target_summary(
        self,
        label: str,
        target: np.ndarray,
        target_stamp: float,
        label_stamp: float,
        raw_label: str = '',
    ) -> str:
        now = time.time()
        age = now - target_stamp
        label_age = now - label_stamp if label_stamp > 0.0 else -1.0
        return (
            f'label={label or "NO"} '
            f'raw={raw_label or "NO"} '
            f'target_mm={target[0]*1000:.1f},{target[1]*1000:.1f},{target[2]*1000:.1f} '
            f'age={age:.2f}s label_age={label_age:.2f}s safe={self.is_grasp_safe(target)}'
        )

    def current_grasp_plan_snapshot(self) -> dict[str, Any]:
        ok, _, plan = self.validate_current_grasp_plan()
        if not ok:
            return {}
        return plan

    @staticmethod
    def describe_grasp_plan(plan: dict[str, Any]) -> str:
        if not plan:
            return 'plan=NO'
        return (
            f'plan_strategy={plan.get("grasp_strategy") or "NO"} '
            f'plan_type={plan.get("grasp_type") or "NO"} '
            f'plan_shape={plan.get("object_shape") or "NO"} '
            f'plan_width={plan.get("grasp_width_hint") or "NO"}'
        )

    def select_grasp_profile(self, label: str, raw_label: str, plan: Optional[dict[str, Any]] = None) -> dict[str, Any]:
        plan = plan or {}
        strategy = str(plan.get('grasp_strategy') or '').strip().lower()
        if strategy and strategy in self.grasp_profiles_by_strategy:
            return self.grasp_profiles_by_strategy[strategy]
        raw_lower = (raw_label or '').lower()
        for keyword, profile in self.grasp_profiles_by_raw_keyword.items():
            if keyword and keyword in raw_lower:
                return profile
        if label in self.grasp_profiles_by_label:
            return self.grasp_profiles_by_label[label]
        return self.grasp_profile_default

    def precheck_sort_snapshot(
        self,
        label: str,
        target: np.ndarray,
        raw_label: str = '',
        plan: Optional[dict[str, Any]] = None,
    ) -> tuple[bool, str]:
        """Run the same no-motion checks used by the actual sort path.

        /trash_grasp_once returns before the worker thread moves the arm. Without
        this upfront check the WebUI can show STARTED even when the worker
        rejects the target immediately, which looks like a dead arm.
        """
        plan = dict(plan or {})
        target = np.array(target, dtype=np.float64).copy()
        if label not in self.drop_points:
            return False, f'UNKNOWN_LABEL {label}'
        if not self.is_grasp_safe(target):
            return False, f'OUT_OF_RANGE {target[0]*1000:.1f},{target[1]*1000:.1f},{target[2]*1000:.1f}mm'

        profile = self.select_grasp_profile(label, raw_label, plan)
        command_target = target.copy() + profile['target_offset_m']
        hover_target = command_target.copy()
        grasp_target = command_target.copy() + profile['descent_offset_m']
        if profile['descent_lock_xy']:
            grasp_target[0] = command_target[0]
            grasp_target[1] = command_target[1]

        min_descent = max(0.0, float(profile.get('min_descent_m', 0.0)))
        if min_descent > 0.0:
            highest_allowed_contact_z = target[2] - min_descent
            if grasp_target[2] > highest_allowed_contact_z:
                grasp_target[2] = highest_allowed_contact_z
        grasp_target = self.clamp_contact_workspace_z(grasp_target, 'grasp')

        if not self.is_grasp_safe(grasp_target):
            return (
                False,
                (
                    f'OUT_OF_RANGE_GRASP_FINAL '
                    f'{grasp_target[0]*1000:.1f},{grasp_target[1]*1000:.1f},{grasp_target[2]*1000:.1f}mm'
                ),
            )

        pre = hover_target.copy()
        pre[2] += profile['approach_m']
        lift = grasp_target.copy()
        lift[2] += self.lift_m
        return self.precheck_motion_points([
            ('pre_grasp', pre),
            ('grasp', grasp_target),
            ('lift', lift),
        ])

    def grasp_once_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self.try_mark_busy():
            response.success = False
            response.message = 'BUSY'
            return response

        self.clear_vlm_cache_before_manual_grasp()
        ok, reason, label, target, target_stamp, label_stamp, raw_label = self.wait_for_valid_target()
        if not ok:
            self.clear_busy()
            self.publish_status(f'MANUAL_TRIGGER_REJECT {reason}')
            response.success = False
            response.message = reason
            return response

        summary = self.format_target_summary(label, target, target_stamp, label_stamp, raw_label)
        plan_ok, plan_reason, plan = self.validate_current_grasp_plan()
        if not plan_ok:
            self.clear_busy()
            self.publish_status(f'MANUAL_TRIGGER_REJECT {plan_reason}')
            response.success = False
            response.message = plan_reason
            return response
        if plan:
            self.publish_status(f'MANUAL_TRIGGER_PLAN {self.describe_grasp_plan(plan)}')
        ok, reason = self.precheck_sort_snapshot(label, target, raw_label, plan)
        if not ok:
            self.clear_busy()
            self.publish_status(f'MANUAL_TRIGGER_REJECT {reason}')
            response.success = False
            response.message = reason
            return response

        self.publish_status(f'MANUAL_TRIGGER_START {summary}')
        threading.Thread(target=self.manual_sort_worker, args=(label, target.copy(), raw_label, plan), daemon=True).start()
        response.success = True
        response.message = f'STARTED {summary}'
        return response

    def sort_goal_callback(self, goal_request) -> GoalResponse:
        del goal_request
        if self.busy:
            return GoalResponse.REJECT
        ok, reason, label, target, _, _, raw_label = self.validate_target()
        if ok:
            plan_ok, plan_reason, plan = self.validate_current_grasp_plan()
            if not plan_ok:
                ok, reason = False, plan_reason
            else:
                ok, reason = self.precheck_sort_snapshot(label, target, raw_label, plan)
        if not ok:
            self.publish_status(f'ACTION_GOAL_REJECT {reason}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def sort_cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.cancel_requested = True
        self.publish_status('ACTION_CANCEL_REQUESTED')
        return CancelResponse.ACCEPT

    def execute_sort_action(self, goal_handle):
        command = getattr(goal_handle.request, 'command', '') or 'ACTION'

        def feedback(stage: str, message: str) -> None:
            if SortGrasp is None:
                return
            msg = SortGrasp.Feedback()
            msg.stage = stage
            msg.message = message
            goal_handle.publish_feedback(msg)

        self.publish_status(f'ACTION_TRIGGER_START command={command} {self.current_target_summary()}')
        self.cancel_requested = False
        ok, reason, label, target, _, _, raw_label = self.validate_target()
        if not ok:
            self.clear_busy()
            result = SortGrasp.Result()
            result.success = False
            result.message = reason
            goal_handle.abort()
            self.publish_status(f'ACTION_EXECUTE_REJECT {reason}')
            return result
        plan_ok, plan_reason, plan = self.validate_current_grasp_plan()
        if not plan_ok:
            self.clear_busy()
            result = SortGrasp.Result()
            result.success = False
            result.message = plan_reason
            goal_handle.abort()
            self.publish_status(f'ACTION_EXECUTE_REJECT {plan_reason}')
            return result
        ok = self.sort_once(
            mark_busy=True,
            feedback_cb=feedback,
            cancel_check=lambda: bool(goal_handle.is_cancel_requested or self.cancel_requested),
            snapshot=(label, target.copy(), raw_label, plan),
        )
        result = SortGrasp.Result()
        cancelled = bool(goal_handle.is_cancel_requested or self.cancel_requested)
        result.success = bool(ok)
        result.message = 'SORT_CANCELLED' if cancelled else ('SORT_DONE' if ok else 'SORT_FAILED')
        if ok:
            goal_handle.succeed()
        elif cancelled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def try_mark_busy(self) -> bool:
        with self.busy_lock:
            if self.busy:
                return False
            self.busy = True
            return True

    def clear_busy(self) -> None:
        with self.busy_lock:
            self.busy = False

    def manual_sort_worker(
        self,
        label: str,
        target: np.ndarray,
        raw_label: str = '',
        plan: Optional[dict[str, Any]] = None,
    ) -> None:
        ok = self.sort_once(mark_busy=False, snapshot=(label, target.copy(), raw_label, plan or {}))
        self.publish_status(f'MANUAL_TRIGGER_DONE ok={ok}')

    def recover_after_failure(self, reason: str, stage: str) -> None:
        self.publish_status(f'SORT_FAILED stage={stage} reason={reason}')
        if stage == 'validate':
            return
        try:
            if stage in ('pre_grasp', 'grasp'):
                self.publish_gripper(self.open_value)
            elif stage in ('leave_drop', 'init'):
                self.publish_gripper(self.open_value)
            else:
                self.publish_gripper(self.close_value)

            if stage != 'validate':
                self.move_to(self.init_pose, 'recovery_init')
        except Exception as exc:  # noqa: BLE001 - recovery must never crash the node
            self.publish_status(f'RECOVERY_FAILED stage={stage} error={exc}')

    def sort_once(self, mark_busy: bool = True, feedback_cb=None, cancel_check=None, snapshot=None) -> bool:
        if mark_busy and not self.try_mark_busy():
            self.status_pub.publish(String(data='BUSY'))
            return False

        def cancelled(stage_name: str) -> bool:
            if cancel_check is not None and bool(cancel_check()):
                self.publish_status(f'SORT_CANCELLED stage={stage_name}')
                self.recover_after_failure('CANCELLED', stage_name)
                return True
            return False

        if snapshot is None:
            ok, reason, label, target, _, _, raw_label = self.validate_target()
            if not ok:
                self.publish_status(reason)
                self.clear_busy()
                return False
            plan = self.current_grasp_plan_snapshot()
        else:
            if len(snapshot) >= 4:
                label, target, raw_label, plan = snapshot
            elif len(snapshot) >= 3:
                label, target, raw_label = snapshot
                plan = {}
            else:
                label, target = snapshot
                raw_label = ''
                plan = {}
            target = np.array(target, dtype=np.float64).copy()
            plan = dict(plan or {})
            if label not in self.drop_points:
                self.publish_status(f'UNKNOWN_LABEL {label}')
                self.clear_busy()
                return False
            if not self.is_grasp_safe(target):
                self.publish_status(f'OUT_OF_RANGE {target[0]*1000:.1f},{target[1]*1000:.1f},{target[2]*1000:.1f}mm')
                self.clear_busy()
                return False

        stage = 'validate'
        try:
            self.publish_stage(stage, f'label={label}', feedback_cb)
            drop = self.drop_points[label].copy()
            profile = self.select_grasp_profile(label, raw_label, plan)
            command_target = target.copy() + profile['target_offset_m']
            hover_target = command_target.copy()
            grasp_target = command_target.copy() + profile['descent_offset_m']
            if profile['descent_lock_xy']:
                grasp_target[0] = command_target[0]
                grasp_target[1] = command_target[1]
            min_descent = max(0.0, float(profile.get('min_descent_m', 0.0)))
            if min_descent > 0.0:
                highest_allowed_contact_z = target[2] - min_descent
                if grasp_target[2] > highest_allowed_contact_z:
                    self.publish_status(
                        (
                            f'GRASP_Z_CLAMP profile={profile["name"]} '
                            f'surface_z_mm={target[2]*1000:.1f} '
                            f'old_grasp_z_mm={grasp_target[2]*1000:.1f} '
                            f'new_grasp_z_mm={highest_allowed_contact_z*1000:.1f} '
                            f'min_descent_mm={min_descent*1000:.1f}'
                        )
                    )
                    grasp_target[2] = highest_allowed_contact_z
            grasp_target = self.clamp_contact_workspace_z(grasp_target, 'grasp')
            if not self.is_grasp_safe(grasp_target):
                self.publish_status(
                    (
                        f'OUT_OF_RANGE_GRASP_FINAL '
                        f'{grasp_target[0]*1000:.1f},{grasp_target[1]*1000:.1f},{grasp_target[2]*1000:.1f}mm'
                    )
                )
                return False
            pre = hover_target.copy()
            pre[2] += profile['approach_m']
            lift = grasp_target.copy()
            lift[2] += self.lift_m
            pre_drop = drop.copy()
            pre_drop[2] += self.drop_safe_z_m

            ok, reason = self.precheck_motion_points([
                ('pre_grasp', pre),
                ('grasp', grasp_target),
                ('lift', lift),
            ])
            if not ok:
                self.publish_status(reason)
                self.recover_after_failure(reason, 'validate')
                return False

            self.publish_status(
                (
                    f'SORT_START label={label} '
                    f'raw={raw_label or "NO"} '
                    f'{self.describe_grasp_plan(plan)} '
                    f'profile={profile["name"]} '
                    f'target_mm={target[0]*1000:.1f},{target[1]*1000:.1f},{target[2]*1000:.1f} '
                    f'command_target_mm={command_target[0]*1000:.1f},{command_target[1]*1000:.1f},{command_target[2]*1000:.1f} '
                    f'pre_mm={pre[0]*1000:.1f},{pre[1]*1000:.1f},{pre[2]*1000:.1f} '
                    f'grasp_mm={grasp_target[0]*1000:.1f},{grasp_target[1]*1000:.1f},{grasp_target[2]*1000:.1f} '
                    f'lift_mm={lift[0]*1000:.1f},{lift[1]*1000:.1f},{lift[2]*1000:.1f} '
                    f'min_descent_mm={min_descent*1000:.1f}'
                )
            )
            if cancelled(stage):
                return False
            self.publish_stage('open_gripper', '', feedback_cb)
            self.publish_gripper(self.open_value)
            stage = 'pre_grasp'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(pre, 'pre_grasp'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'grasp'
            if cancelled(stage):
                return False
            self.publish_stage(
                stage,
                (
                    f'final_mm={grasp_target[0]*1000:.1f},'
                    f'{grasp_target[1]*1000:.1f},{grasp_target[2]*1000:.1f} '
                    f'profile={profile["name"]} '
                    f'descent_offset_mm={profile["descent_offset_m"][0]*1000:.1f},'
                    f'{profile["descent_offset_m"][1]*1000:.1f},{profile["descent_offset_m"][2]*1000:.1f} '
                    f'min_descent_mm={min_descent*1000:.1f} '
                    f'lock_xy={profile["descent_lock_xy"]}'
                ),
                feedback_cb,
            )
            if not self.move_descent(pre, grasp_target, 'grasp', int(profile['descent_steps'])):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'close_gripper'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            self.publish_gripper(self.close_value)
            time.sleep(self.gripper_hold_sec)
            stage = 'lift'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(lift, 'lift'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'init_after_grasp'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(self.init_pose, 'init_after_grasp'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'safe_drop_pose'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(self.safe_drop_pose, 'safe_drop_pose'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'pre_drop'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(pre_drop, 'pre_drop'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'drop'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(drop, 'drop'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'open_at_drop'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            self.publish_gripper(self.open_value)
            time.sleep(0.5)
            stage = 'leave_drop'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            if not self.move_to(pre_drop, 'leave_drop'):
                self.recover_after_failure('MOVE_FAILED', stage)
                return False
            stage = 'init'
            if cancelled(stage):
                return False
            self.publish_stage(stage, '', feedback_cb)
            self.move_to(self.init_pose, 'init')
            self.last_grasp_time = time.time()
            self.clear_vlm_cache_after_success()
            self.publish_status('SORT_DONE')
            return True
        except Exception as exc:  # noqa: BLE001 - publish and recover instead of leaving the arm stranded
            self.recover_after_failure(str(exc), stage)
            return False
        finally:
            self.clear_busy()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RoArmSortGrasper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
