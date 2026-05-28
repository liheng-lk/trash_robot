from __future__ import annotations

import json
import os
import signal
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

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

from geometry_msgs.msg import Twist
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trash_robot_interfaces.srv import InitPose, SaveMap


class RobotManager(Node):
    def __init__(self) -> None:
        super().__init__('trash_robot_manager')
        self.declare_parameter('project_root', '/home/sunrise/trash_robot_v3')
        self.ws = Path(str(self.get_parameter('project_root').value))
        self.runtime = self.ws / 'runtime'
        self.log_dir = self.runtime / 'logs' / 'manager'
        self.registry_file = self.runtime / 'manager_processes.yaml'
        self.mode_file = Path('/tmp/trash_robot_v3_mode.lock')
        self.motion_lock_file = Path('/tmp/trash_robot_v3_motion.lock')
        self.estop_lock_file = Path('/tmp/trash_robot_v3_estop.lock')
        self.profile_file = self.ws / 'config' / 'system' / 'rdk_resource_profile.yaml'
        self.runtime.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.resource_profile = self.load_resource_profile()
        self.stop_all_running = False
        self.stop_all_lock = threading.Lock()

        self.status_pub = self.create_publisher(String, '/trash_system_status', 10)
        self.system_state_pub = self.create_publisher(String, '/trash_robot_v3/manager/system_state', 10)
        self.resource_pub = self.create_publisher(String, '/trash_resource_status', 10)
        self.cmd_zero_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.prefixed_cmd_zero_pub = self.create_publisher(Twist, '/trash_robot_v3/base/cmd_vel', 10)
        self.create_service(Trigger, '/trash_manager/start_base', self.start_base_cb)
        self.create_service(Trigger, '/trash_manager/stop_base', self.stop_base_cb)
        self.create_service(Trigger, '/trash_manager/start_camera', self.start_camera_cb)
        self.create_service(Trigger, '/trash_manager/stop_camera', self.stop_camera_cb)
        self.create_service(Trigger, '/trash_manager/start_arm', self.start_arm_cb)
        self.create_service(Trigger, '/trash_manager/stop_arm', self.stop_arm_cb)
        self.create_service(Trigger, '/trash_manager/start_handeye', self.start_handeye_cb)
        self.create_service(Trigger, '/trash_manager/stop_handeye', self.stop_handeye_cb)
        self.create_service(Trigger, '/trash_manager/start_video', self.start_video_cb)
        self.create_service(Trigger, '/trash_manager/stop_video', self.stop_video_cb)
        self.create_service(Trigger, '/trash_manager/start_mapping', self.start_mapping_cb)
        self.create_service(Trigger, '/trash_manager/stop_mapping', self.stop_mapping_cb)
        self.create_service(Trigger, '/trash_manager/start_navigation', self.start_navigation_cb)
        self.create_service(Trigger, '/trash_manager/stop_navigation', self.stop_navigation_cb)
        self.create_service(Trigger, '/trash_manager/start_grasp_dry', self.start_grasp_dry_cb)
        self.create_service(Trigger, '/trash_manager/start_grasp_live', self.start_grasp_live_cb)
        self.create_service(Trigger, '/trash_manager/start_grasp_vlm_dry', self.start_grasp_vlm_dry_cb)
        self.create_service(Trigger, '/trash_manager/start_grasp_vlm_live', self.start_grasp_vlm_live_cb)
        self.create_service(Trigger, '/trash_manager/stop_grasp', self.stop_grasp_cb)
        self.create_service(SaveMap, '/trash_manager/save_map', self.save_map_cb)
        self.create_service(InitPose, '/trash_manager/init_pose', self.init_pose_cb)
        self.create_service(Trigger, '/trash_manager/reclaim_resources', self.reclaim_resources_cb)
        self.create_service(Trigger, '/trash_manager/stop_all', self.stop_all_cb)
        self.create_service(Trigger, '/trash_robot_v3/manager/estop_trigger', self.estop_trigger_cb)
        self.create_service(Trigger, '/trash_robot_v3/manager/estop_reset', self.estop_reset_cb)
        self.create_service(Trigger, '/trash_manager/estop_trigger', self.estop_trigger_cb)
        self.create_service(Trigger, '/trash_manager/estop_reset', self.estop_reset_cb)
        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f'trash_robot_manager ready root={self.ws}')

    def load_resource_profile(self) -> dict[str, Any]:
        if not self.profile_file.exists():
            self.get_logger().warning(f'resource profile not found: {self.profile_file}')
            return {}
        data = yaml.safe_load(self.profile_file.read_text(encoding='utf-8')) or {}
        return data if isinstance(data, dict) else {}

    def read_registry(self) -> dict[str, dict[str, Any]]:
        if not self.registry_file.exists():
            return {}
        data = yaml.safe_load(self.registry_file.read_text(encoding='utf-8')) or {}
        return data if isinstance(data, dict) else {}

    def write_registry(self, data: dict[str, dict[str, Any]]) -> None:
        self.registry_file.parent.mkdir(parents=True, exist_ok=True)
        self.registry_file.write_text(yaml.safe_dump(data, sort_keys=True), encoding='utf-8')

    def default_mode_state(self) -> dict[str, Any]:
        return {
            'mode': 'IDLE',
            'owner': 'trash_robot_manager',
            'reason': 'default',
            'stamp': time.time(),
        }

    def read_mode(self) -> dict[str, Any]:
        if not self.mode_file.exists():
            return self.default_mode_state()
        try:
            data = yaml.safe_load(self.mode_file.read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            return self.default_mode_state()
        if not isinstance(data, dict):
            return self.default_mode_state()
        data.setdefault('mode', 'IDLE')
        data.setdefault('owner', 'unknown')
        data.setdefault('reason', '')
        data.setdefault('stamp', 0.0)
        return data

    def read_lock_file(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError):
            return {}
        return data if isinstance(data, dict) else {}

    def motion_lock_status_dict(self) -> dict[str, Any]:
        motion = self.read_lock_file(self.motion_lock_file)
        estop = self.read_lock_file(self.estop_lock_file)
        return {
            'motion_owner': str(motion.get('owner') or 'IDLE'),
            'motion_stamp': float(motion.get('stamp') or 0.0),
            'estop_active': self.estop_lock_file.exists(),
            'estop_reason': str(estop.get('reason') or ''),
            'estop_stamp': float(estop.get('stamp') or 0.0),
            'motion_lock_file': str(self.motion_lock_file),
            'estop_lock_file': str(self.estop_lock_file),
        }

    def dds_status_dict(self) -> dict[str, Any]:
        return {
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', ''),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
            'cyclonedds_uri': os.environ.get('CYCLONEDDS_URI', ''),
            'fastrtps_profile': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', ''),
        }

    def write_mode(self, mode: str, owner: str, reason: str) -> None:
        self.mode_file.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'mode': mode,
            'owner': owner,
            'reason': reason,
            'stamp': time.time(),
        }
        self.mode_file.write_text(yaml.safe_dump(data, sort_keys=True), encoding='utf-8')

    def clear_mode_if(self, *modes: str) -> None:
        current = str(self.read_mode().get('mode', 'IDLE'))
        if current in modes:
            self.write_mode('IDLE', 'trash_robot_manager', f'clear {current}')

    def mode_available(self, requested: str) -> tuple[bool, str]:
        current_state = self.read_mode()
        current = str(current_state.get('mode', 'IDLE'))
        motion = self.motion_lock_status_dict()
        if bool(motion.get('estop_active')) and requested != 'ESTOP':
            return False, f'ESTOP active: {motion.get("estop_reason") or "manual"}'
        conflicts = {
            'GRASP': {'ARM_MANUAL', 'CALIBRATION'},
            'ARM_MANUAL': {'GRASP', 'CALIBRATION'},
            'CALIBRATION': {'GRASP', 'NAVIGATION', 'VIDEO', 'ARM_MANUAL'},
            'NAVIGATION': {'CALIBRATION'},
            'VIDEO': {'CALIBRATION'},
        }
        if current in conflicts.get(requested, set()):
            return False, f'mode conflict: current={current} requested={requested}'
        return True, 'OK'

    def publish_zero_burst(self, reason: str) -> None:
        msg = Twist()
        for _ in range(10):
            self.cmd_zero_pub.publish(msg)
            self.prefixed_cmd_zero_pub.publish(msg)
            time.sleep(0.02)
        self.get_logger().warning(f'zero velocity burst published: {reason}')

    def write_estop_active(self, reason: str) -> None:
        self.estop_lock_file.parent.mkdir(parents=True, exist_ok=True)
        self.motion_lock_file.parent.mkdir(parents=True, exist_ok=True)
        stamp = time.time()
        self.estop_lock_file.write_text(
            yaml.safe_dump({'active': True, 'reason': reason, 'stamp': stamp}, sort_keys=False, allow_unicode=True),
            encoding='utf-8',
        )
        self.motion_lock_file.write_text(
            yaml.safe_dump({'owner': 'ESTOP', 'stamp': stamp}, sort_keys=False, allow_unicode=True),
            encoding='utf-8',
        )
        self.write_mode('ESTOP', 'trash_robot_manager', reason)

    def clear_estop_active(self) -> None:
        try:
            self.estop_lock_file.unlink()
        except OSError:
            pass
        motion = self.read_lock_file(self.motion_lock_file)
        if str(motion.get('owner') or '') == 'ESTOP':
            try:
                self.motion_lock_file.unlink()
            except OSError:
                pass
        if str(self.read_mode().get('mode') or '') == 'ESTOP':
            self.write_mode('IDLE', 'trash_robot_manager', 'estop reset')

    def is_pid_alive(self, pid: int) -> bool:
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def pid_cmdline(self, pid: int) -> str:
        try:
            raw = Path(f'/proc/{pid}/cmdline').read_bytes()
        except OSError:
            return ''
        return raw.replace(b'\x00', b' ').decode('utf-8', errors='replace').strip()

    def registry_entry_alive(self, name: str, entry: dict[str, Any]) -> bool:
        pid = int(entry.get('pid', 0) or 0)
        if not self.is_pid_alive(pid):
            return False
        cmdline = self.pid_cmdline(pid)
        command = entry.get('command', [])
        if not cmdline or not isinstance(command, list) or not command:
            return False

        executable = Path(str(command[0])).name
        if executable and executable in cmdline:
            return True

        component = str(entry.get('component', name))
        expected_tokens = {
            'camera': ('start_camera.sh', 'camera_realsense.launch.py', 'realsense2_camera_node'),
            'base': ('start_base.sh', 'serial_base_node', 'sllidar_node'),
            'arm': ('start_arm.sh', 'roarm_driver', 'command_control.launch.py'),
            'video': ('light_mjpeg_streamer',),
            'grasp_vlm_live': ('start_grasp.sh', 'perception_grasp.launch.py', 'roarm_sort_grasper'),
            'grasp_vlm_dry': ('start_grasp.sh', 'perception_grasp.launch.py', 'roarm_sort_grasper'),
            'navigation': ('start_navigation.sh', 'nav2_', 'amcl'),
            'mapping': ('start_mapping.sh', 'slam_toolbox'),
            'handeye': ('start_handeye.sh', 'handeye_web_calibrator'),
            'mission': ('mission_supervisor',),
        }.get(component, ())
        return any(token in cmdline for token in expected_tokens)

    def process_alive_by_name(self, name: str) -> bool:
        entry = self.read_registry().get(name, {})
        return self.registry_entry_alive(name, entry)

    def pattern_alive(self, pattern: str) -> bool:
        result = subprocess.run(
            ['pgrep', '-f', pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return result.returncode == 0

    def service_exists(self, name: str) -> bool:
        try:
            return any(service_name == name for service_name, _types in self.get_service_names_and_types())
        except Exception as exc:  # noqa: BLE001 - readiness is best-effort during startup/shutdown
            self.get_logger().debug(f'service_exists failed for {name}: {exc}')
            return False

    def tcp_port_ready(self, host: str, port: int, timeout_sec: float = 0.5) -> bool:
        try:
            with socket.create_connection((host, int(port)), timeout=timeout_sec):
                return True
        except OSError:
            return False

    def arm_services_ready(self) -> bool:
        return bool(self.arm_status_dict().get('ok'))

    def tail_text(self, path: Path, limit: int = 12000) -> str:
        try:
            if not path.exists():
                return ''
            with path.open('rb') as handle:
                size = path.stat().st_size
                handle.seek(max(0, size - limit))
                return handle.read().decode('utf-8', errors='replace')
        except OSError:
            return ''

    def arm_status_dict(self) -> dict[str, Any]:
        serial_port = os.environ.get('ROARM_SERIAL_PORT', '/dev/roarm')
        serial_path = Path(serial_port)
        serial_exists = serial_path.exists()
        resolved = ''
        if serial_exists:
            try:
                resolved = str(serial_path.resolve())
            except OSError:
                resolved = serial_port
        serial_rw = serial_exists and os.access(serial_port, os.R_OK | os.W_OK)
        driver_online = self.pattern_alive('roarm_driver')
        move_service = self.service_exists('/move_point_cmd')
        pose_service = self.service_exists('/get_pose_cmd')
        gripper_service = self.service_exists('/gripper_controller/get_parameters') or self.service_exists('/gripper_action_client/get_parameters')
        log_path = self.runtime / 'logs' / 'arm' / 'roarm_driver.log'
        log_tail = self.tail_text(log_path)
        error_patterns = (
            'Serial write failed',
            'Error communicating with serial port',
            'failed to open',
            'Permission denied',
            'Input/output error',
        )
        recent_error = ''
        for line in reversed(log_tail.splitlines()):
            if any(pattern in line for pattern in error_patterns):
                recent_error = line.strip()
                break
        opened_after_error = False
        if recent_error:
            last_error_idx = log_tail.rfind(recent_error)
            opened_after_error = 'Opened ' in log_tail[last_error_idx + len(recent_error):]
        active_error = bool(recent_error and not opened_after_error)

        ok = bool(serial_exists and serial_rw and driver_online and move_service and pose_service and not active_error)
        if not serial_exists:
            state = 'missing'
            message = f'{serial_port} not found'
        elif not serial_rw:
            state = 'permission'
            message = f'{serial_port} is not readable/writable by current user'
        elif active_error:
            state = 'serial_error'
            message = recent_error
        elif not driver_online:
            state = 'driver_offline'
            message = 'roarm_driver not running'
        elif not (move_service and pose_service):
            state = 'services_not_ready'
            message = 'MoveIt arm services not ready'
        else:
            state = 'ok'
            message = 'serial and arm services ready'
        return {
            'ok': ok,
            'state': state,
            'message': message,
            'serial_port': serial_port,
            'resolved': resolved,
            'serial_exists': serial_exists,
            'serial_rw': serial_rw,
            'driver_online': driver_online,
            'move_service': move_service,
            'pose_service': pose_service,
            'gripper_service': gripper_service,
            'recent_error': recent_error,
            'log_path': str(log_path),
        }

    def profile_get(self, *keys: str, default: Any = None) -> Any:
        value: Any = self.resource_profile
        for key in keys:
            if not isinstance(value, dict) or key not in value:
                return default
            value = value[key]
        return value

    def log_size_mb(self, path: Path) -> float:
        if not path.exists():
            return 0.0
        if path.is_file():
            return path.stat().st_size / (1024.0 * 1024.0)
        total = 0
        for child in path.rglob('*'):
            try:
                if child.is_file():
                    total += child.stat().st_size
            except OSError:
                pass
        return total / (1024.0 * 1024.0)

    def rotate_log_if_needed(self, path: Path) -> None:
        max_mb = float(self.profile_get('disk', 'component_log_max_mb', default=50) or 50)
        if path.exists() and path.stat().st_size > max_mb * 1024 * 1024:
            rotated = path.with_suffix(path.suffix + f'.{time.strftime("%Y%m%d_%H%M%S")}')
            try:
                path.rename(rotated)
            except OSError as exc:
                self.get_logger().warning(f'log rotate failed {path}: {exc}')

    def start_managed(self, name: str, args: list[str], log_name: str, exclusive: bool = True) -> tuple[bool, str]:
        registry = self.read_registry()
        entry = registry.get(name, {})
        pid = int(entry.get('pid', 0) or 0)
        if exclusive and self.registry_entry_alive(name, entry):
            return True, f'{name} already running pid={pid}'
        if exclusive and pid > 0 and not self.registry_entry_alive(name, entry):
            registry.pop(name, None)
            self.write_registry(registry)

        log_path = self.log_dir / log_name
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.rotate_log_if_needed(log_path)
        log_file = log_path.open('ab')
        child_env = os.environ.copy()
        child_env['TRASH_ROBOT_ROOT'] = str(self.ws)
        child_env['ROS_DOMAIN_ID'] = os.environ.get('TRASH_ROS_DOMAIN_ID') or '1'
        child_env['ROS_LOCALHOST_ONLY'] = '0'
        child_env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        child_env['CYCLONEDDS_URI'] = f'file://{self.ws}/config/dds/cyclonedds_unicast.xml'
        for key in (
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            'RMW_FASTRTPS_USE_QOS_FROM_XML',
            'ROS_DISABLE_LOANED_MESSAGES',
        ):
            child_env.pop(key, None)
        proc = subprocess.Popen(
            args,
            cwd=str(self.ws),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=child_env,
        )
        registry[name] = {
            'pid': proc.pid,
            'pgid': proc.pid,
            'component': name,
            'owner': 'trash_robot_manager',
            'started_at': time.time(),
            'command': args,
            'log_path': str(log_path),
        }
        self.write_registry(registry)
        return True, f'{name} started pid={proc.pid}'

    def run_script(self, name: str, *args: str, timeout: float = 20.0) -> tuple[bool, str]:
        script = self.ws / 'scripts' / name
        if not script.exists():
            return False, f'script not found: {script}'
        try:
            result = subprocess.run(
                [str(script), *args],
                cwd=str(self.ws),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            return False, f'{name} timeout after {timeout:.1f}s: {(exc.stdout or "")[-300:]}'
        return result.returncode == 0, (result.stdout or '')[-600:]

    def run_common_func(self, func_name: str, timeout: float = 20.0) -> tuple[bool, str]:
        command = f'source scripts/common.sh; {func_name}'
        try:
            result = subprocess.run(
                ['bash', '-lc', command],
                cwd=str(self.ws),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            return False, f'{func_name} timeout after {timeout:.1f}s: {(exc.stdout or "")[-300:]}'
        return result.returncode == 0, (result.stdout or '')[-600:]

    def disk_allows_start(self, component: str) -> tuple[bool, str]:
        resources = self.resource_status_dict()
        if resources.get('disk_block'):
            return False, f'disk usage {resources.get("disk_used_percent")}% blocks starting {component}'
        return True, 'OK'

    def sanitize_map_name(self, name: str) -> str:
        raw = (name or '').strip()
        if not raw:
            raw = f'map_{time.strftime("%Y%m%d_%H%M%S")}'
        cleaned = ''.join(ch if ch.isalnum() or ch in ('_', '-') else '_' for ch in raw)
        return cleaned.strip('_') or f'map_{time.strftime("%Y%m%d_%H%M%S")}'

    def start_base_process(self) -> tuple[bool, str]:
        ok, msg = self.disk_allows_start('base')
        if not ok:
            return ok, msg
        base_pattern = str(self.profile_get('manager', 'component_patterns', 'base', default='serial_base_node|sllidar_node'))
        return self.start_managed(
            'base',
            [str(self.ws / 'scripts' / 'start_base.sh'), 'start'],
            'base.log',
        )

    def start_camera_process(self, profile: str = 'full') -> tuple[bool, str]:
        ok, msg = self.disk_allows_start('camera')
        if not ok:
            return ok, msg
        if profile == 'handeye':
            self.stop_registered('camera')
            env_prefix = 'TRASH_CAMERA_MODE=handeye TRASH_CAMERA_FOREGROUND=1'
        else:
            env_prefix = 'TRASH_CAMERA_FOREGROUND=1'
        return self.start_managed(
            'camera',
            ['bash', '-lc', f'{env_prefix} scripts/start_camera.sh'],
            'camera.log',
        )

    def start_arm_process(self, owner_mode: str = 'ARM_MANUAL') -> tuple[bool, str]:
        ok, msg = self.mode_available(owner_mode)
        if not ok:
            return ok, msg
        ok, msg = self.disk_allows_start('arm')
        if not ok:
            return ok, msg
        if self.arm_services_ready():
            if owner_mode == 'GRASP':
                self.write_mode('GRASP', 'trash_robot_manager', 'grasp using existing arm services')
            return True, 'arm already ready services=/move_point_cmd,/get_pose_cmd'

        started_ok, started_msg = self.run_script('start_arm.sh', 'start', timeout=75.0)
        if not started_ok:
            if owner_mode == 'GRASP':
                self.clear_mode_if('GRASP')
            return False, started_msg or 'start_arm.sh failed'

        deadline = time.monotonic() + 20.0
        last_status = self.arm_status_dict()
        while time.monotonic() < deadline:
            last_status = self.arm_status_dict()
            if last_status.get('ok'):
                if owner_mode == 'GRASP':
                    self.write_mode('GRASP', 'trash_robot_manager', 'grasp started arm services')
                return True, (started_msg.strip() or 'arm services started')
            time.sleep(0.5)
        if owner_mode == 'GRASP':
            self.clear_mode_if('GRASP')
        return False, f"arm services not ready after start: {last_status.get('state')} {last_status.get('message')}; {started_msg[-300:]}"

    def start_grasp_process(self, name: str, mode: str, backend: str) -> tuple[bool, str]:
        if backend != 'vlm':
            return False, 'ERROR: 当前阶段只启用 VLM API；BPU/COCO 本地模型后续接入'
        ok, msg = self.mode_available('GRASP')
        if not ok:
            return ok, msg
        ok, msg = self.disk_allows_start(name)
        if not ok:
            return ok, msg
        self.stop_registered('handeye')
        # Grasp mode is exclusive. A stale dry-run pipeline can still provide
        # /trash_grasp_once and make the WebUI look alive while never moving the
        # arm. Always clear every grasp owner before starting the requested mode.
        for item in ('grasp_vlm_live', 'grasp_vlm_dry'):
            self.stop_registered(item)
        self.run_script('start_grasp.sh', 'stop', timeout=20.0)
        self.write_mode('GRASP', 'trash_robot_manager', f'start {name} mode={mode} backend={backend}')
        camera_ok, camera_msg = self.start_camera_process('full')
        messages = [camera_msg]
        if not camera_ok:
            return False, '; '.join(messages)
        if mode == 'live':
            arm_ok, arm_msg = self.start_arm_process('GRASP')
            messages.append(arm_msg)
            if not arm_ok:
                return False, '; '.join(messages)
        grasp_ok, grasp_msg = self.run_script('start_grasp.sh', mode, timeout=75.0)
        messages.append(grasp_msg.strip() or f'start_grasp.sh {mode} finished')
        if not grasp_ok:
            self.clear_mode_if('GRASP')
            return False, '; '.join(messages)

        deadline = time.monotonic() + 35.0
        last_detail = 'waiting grasp nodes/services'
        while time.monotonic() < deadline:
            vlm_ready = self.service_exists('/trash_vlm/refresh') and self.pattern_alive('vlm_trash_classifier')
            grasp_ready = self.service_exists('/trash_grasp_once') and self.pattern_alive('roarm_sort_grasper')
            handeye_ready = self.pattern_alive('handeye_target_transformer')
            if vlm_ready and grasp_ready and handeye_ready:
                return True, '; '.join(messages + ['grasp live services ready'])
            last_detail = f'vlm={vlm_ready} grasp={grasp_ready} handeye={handeye_ready}'
            time.sleep(0.5)
        self.clear_mode_if('GRASP')
        return False, '; '.join(messages + [f'timeout waiting grasp services: {last_detail}'])

    def start_base_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success, response.message = self.start_base_process()
        return response

    def stop_base_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('base')
        ok, msg = self.run_script('start_base.sh', 'stop', timeout=30.0)
        response.success = ok
        response.message = msg or 'base stopped'
        return response

    def start_camera_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success, response.message = self.start_camera_process('full')
        return response

    def stop_camera_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('camera')
        response.success, response.message = self.run_script('start_camera.sh', 'stop', timeout=20.0)
        return response

    def start_arm_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success, response.message = self.start_arm_process()
        return response

    def stop_arm_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('arm')
        ok, msg = self.run_script('start_arm.sh', 'stop', timeout=20.0)
        self.clear_mode_if('ARM_MANUAL')
        response.success = ok
        response.message = msg or 'arm stopped'
        return response

    def start_handeye_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        mode_ok, mode_msg = self.mode_available('CALIBRATION')
        if not mode_ok:
            response.success = False
            response.message = mode_msg
            return response
        ok, msg = self.disk_allows_start('handeye')
        if not ok:
            response.success = False
            response.message = msg
            return response
        self.write_mode('CALIBRATION', 'trash_robot_manager', 'handeye calibration')
        self.stop_registered('navigation')
        self.stop_registered('mapping')
        self.stop_registered('base')
        self.stop_registered('camera')
        self.stop_registered('video')
        self.stop_registered('grasp_vlm_dry')
        self.stop_registered('grasp_vlm_live')
        response.success, response.message = self.start_managed(
            'handeye',
            [str(self.ws / 'scripts' / 'start_handeye.sh'), 'start'],
            'handeye.log',
        )
        return response

    def stop_handeye_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('handeye')
        ok, msg = self.run_script('start_handeye.sh', 'stop', timeout=20.0)
        self.clear_mode_if('CALIBRATION')
        response.success = ok
        response.message = msg or 'handeye stopped'
        return response

    def start_video_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        ok, msg = self.start_camera_process('full')
        if not ok:
            response.success = False
            response.message = f'camera start failed before video: {msg}'
            return response
        video_cfg = self.profile_get('video', 'mjpeg', default={}) or {}
        runtime_video_profile = self.runtime / 'config' / 'video_profile.yaml'
        if runtime_video_profile.exists():
            try:
                override = yaml.safe_load(runtime_video_profile.read_text(encoding='utf-8')) or {}
            except (OSError, yaml.YAMLError):
                override = {}
            active = override.get('active', {}) if isinstance(override, dict) else {}
            if isinstance(active, dict):
                if 'fps' in active:
                    video_cfg['max_fps'] = active.get('fps')
                if 'width' in active:
                    video_cfg['max_width'] = active.get('width')
                if 'quality' in active:
                    video_cfg['jpeg_quality'] = active.get('quality')
                if 'max_clients' in active:
                    video_cfg['max_clients'] = active.get('max_clients')
        port = int(video_cfg.get('port', 8092))
        jpeg_quality = int(video_cfg.get('jpeg_quality', 55))
        max_width = int(video_cfg.get('max_width', 640))
        max_fps = float(video_cfg.get('max_fps', 8.0))
        idle_fps = max(1.0, float(video_cfg.get('idle_fps', 1.0)))
        max_clients = int(video_cfg.get('max_clients', 8))
        overlay_max_age = float(video_cfg.get('overlay_max_age_sec', 0.8))
        coord_max_age = float(video_cfg.get('coord_max_age_sec', 1.0))
        command = (
            'set +u; source scripts/source_v3.sh; set -u; '
            'exec ros2 run trash_robot_vision light_mjpeg_streamer --ros-args '
            '-p image_topic:=/camera/camera/color/image_raw '
            f'-p host:=0.0.0.0 -p port:={port} -p jpeg_quality:={jpeg_quality} '
            f'-p max_width:={max_width} -p max_fps:={max_fps} -p idle_fps:={idle_fps} '
            f'-p max_clients:={max_clients} '
            '-p show_detections:=true '
            '-p show_yolo_candidate:=true '
            '-p yolo_candidate_topic:=/trash_yolo_candidate '
            f'-p overlay_max_age_sec:={overlay_max_age} '
            '-p yolo_overlay_max_age_sec:=0.6 '
            f'-p coord_max_age_sec:={coord_max_age}'
        )
        ok, msg = self.start_managed(
            'video',
            ['bash', '-lc', command],
            'video.log',
        )
        (self.runtime / 'video_stream.pid').write_text(str(self.read_registry().get('video', {}).get('pid', '')), encoding='utf-8')
        if not ok:
            response.success = False
            response.message = msg
            return response
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if self.pattern_alive('light_mjpeg_streamer') and self.tcp_port_ready('127.0.0.1', port):
                response.success = True
                response.message = f'{msg}; video stream ready port={port}'
                return response
            time.sleep(0.5)
        response.success = False
        response.message = f'{msg}; video stream did not become ready on port={port}'
        return response

    def stop_video_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('video')
        ok, msg = self.run_script('start_camera.sh', 'stop-rtp', timeout=10.0)
        stop_ok, stop_msg = self.run_common_func('stop_video', timeout=10.0)
        ok = ok and stop_ok
        try:
            (self.runtime / 'video_stream.pid').unlink()
        except OSError:
            pass
        response.success = ok
        response.message = (msg or stop_msg) or 'video stopped'
        return response

    def start_mapping_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        mode_ok, mode_msg = self.mode_available('NAVIGATION')
        if not mode_ok:
            response.success = False
            response.message = mode_msg
            return response
        base_ok, base_msg = self.start_base_process()
        if not base_ok:
            response.success = False
            response.message = base_msg
            return response
        self.write_mode('NAVIGATION', 'trash_robot_manager', 'mapping')
        mapping_ok, mapping_msg = self.start_managed(
            'mapping',
            [str(self.ws / 'scripts' / 'start_mapping.sh')],
            'mapping.log',
        )
        response.success = mapping_ok
        response.message = f'{base_msg}; {mapping_msg}'
        return response

    def stop_mapping_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('mapping')
        ok, msg = self.run_script('start_navigation.sh', 'stop', timeout=20.0)
        self.clear_mode_if('NAVIGATION')
        response.success = ok
        response.message = msg or 'mapping stopped'
        return response

    def start_navigation_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        mode_ok, mode_msg = self.mode_available('NAVIGATION')
        if not mode_ok:
            response.success = False
            response.message = mode_msg
            return response
        messages: list[str] = []
        base_ok, base_msg = self.start_base_process()
        messages.append(base_msg)
        if not base_ok:
            response.success = False
            response.message = '; '.join(messages)
            return response
        camera_ok, camera_msg = self.start_camera_process('full')
        messages.append(camera_msg)
        if not camera_ok:
            response.success = False
            response.message = '; '.join(messages)
            return response
        arm_ok, arm_msg = self.run_script('prepare_navigation_arm.sh', 'start', timeout=90.0)
        messages.append(arm_msg)
        if not arm_ok:
            response.success = False
            response.message = '; '.join(messages)
            return response
        self.write_mode('NAVIGATION', 'trash_robot_manager', 'navigation')
        nav_ok, nav_msg = self.start_managed(
            'navigation',
            [str(self.ws / 'scripts' / 'start_navigation.sh'), 'start'],
            'navigation.log',
        )
        messages.append(nav_msg)
        response.success = nav_ok
        response.message = '; '.join(messages)
        return response

    def stop_navigation_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.stop_registered('navigation')
        ok, msg = self.run_script('start_navigation.sh', 'stop', timeout=20.0)
        self.clear_mode_if('NAVIGATION')
        response.success = ok
        response.message = msg or 'navigation stopped'
        return response

    def start_grasp_dry_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Legacy API alias -> VLM dry stack."""
        return self.start_grasp_vlm_dry_cb(request, response)

    def start_grasp_live_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Legacy API alias -> VLM live stack."""
        return self.start_grasp_vlm_live_cb(request, response)

    def start_grasp_vlm_dry_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success, response.message = self.start_grasp_process('grasp_vlm_dry', 'dry', 'vlm')
        return response

    def start_grasp_vlm_live_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success, response.message = self.start_grasp_process('grasp_vlm_live', 'live', 'vlm')
        return response

    def stop_grasp_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        for name in ('grasp_vlm_live', 'grasp_vlm_dry'):
            self.stop_registered(name)
        ok, msg = self.run_script('start_grasp.sh', 'stop', timeout=20.0)
        self.clear_mode_if('GRASP')
        response.success = ok
        response.message = msg or 'grasp stopped'
        return response

    def estop_trigger_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        reason = 'manager service trigger'
        self.write_estop_active(reason)
        self.publish_zero_burst(reason)
        response.success = True
        response.message = 'ESTOP active; zero velocity burst published'
        return response

    def estop_reset_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.publish_zero_burst('estop reset guard')
        self.clear_estop_active()
        response.success = True
        response.message = 'ESTOP reset; zero velocity guard published'
        return response

    def save_map_cb(self, request: SaveMap.Request, response: SaveMap.Response) -> SaveMap.Response:
        name = self.sanitize_map_name(request.name)
        response.success, response.message = self.run_script('save_map.sh', name, timeout=60.0)
        return response

    def init_pose_cb(self, request: InitPose.Request, response: InitPose.Response) -> InitPose.Response:
        response.success, response.message = self.run_script(
            'init_pose.sh',
            str(float(request.x)),
            str(float(request.y)),
            str(float(request.yaw_deg)),
            timeout=25.0,
        )
        return response

    def stop_all_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self.stop_all_lock:
            if self.stop_all_running:
                response.success = True
                response.message = 'stop_all already running'
                return response
            self.stop_all_running = True
        threading.Thread(target=self.stop_all_worker, daemon=True).start()
        response.success = True
        response.message = 'stop_all queued'
        return response

    def stop_all_worker(self) -> None:
        try:
            ok, msg = self.stop_all()
            level = self.get_logger().info if ok else self.get_logger().warning
            level(f'stop_all completed: {msg}')
        except Exception as exc:  # noqa: BLE001 - Manager must keep serving after cleanup failures
            self.get_logger().error(f'stop_all worker failed: {exc}')
        finally:
            with self.stop_all_lock:
                self.stop_all_running = False

    def reclaim_resources_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success, response.message = self.reclaim_resources(apply=True)
        return response

    def stop_registered(self, name: str) -> None:
        registry = self.read_registry()
        entry = registry.get(name, {})
        pid = int(entry.get('pid', 0) or 0)
        pgid = int(entry.get('pgid', pid) or pid)
        if self.registry_entry_alive(name, entry):
            try:
                os.killpg(pgid, signal.SIGTERM)
            except OSError:
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
            time.sleep(float(self.profile_get('process', 'graceful_stop_sec', default=1.2) or 1.2))
            if self.registry_entry_alive(name, entry):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except OSError:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except OSError:
                        pass
        registry.pop(name, None)
        self.write_registry(registry)

    def stop_all(self) -> tuple[bool, str]:
        order = self.profile_get('manager', 'stop_order', default=None)
        if not isinstance(order, list):
            order = ['mission', 'grasp_vlm_live', 'grasp_vlm_dry', 'navigation', 'mapping', 'video', 'base', 'arm', 'handeye', 'camera']
        for name in order:
            self.stop_registered(name)

        cleanup_steps = [
            ('grasp', 'start_grasp.sh stop', 20.0),
            ('nav', 'start_navigation.sh stop', 20.0),
            ('base', 'start_base.sh stop', 15.0),
            ('arm', 'start_arm.sh stop', 20.0),
            ('handeye', 'start_handeye.sh stop', 20.0),
            ('camera', 'start_camera.sh stop', 20.0),
        ]
        ok = True
        messages: list[str] = []
        for label, command, timeout in cleanup_steps:
            parts = command.split()
            step_ok, step_msg = self.run_script(parts[0], *parts[1:], timeout=timeout)
            ok = ok and step_ok
            if step_msg:
                messages.append(f'{label}: {step_msg[-120:]}')

        try:
            (self.runtime / 'video_stream.pid').unlink()
        except OSError:
            pass

        self.write_registry({})
        if not bool(self.motion_lock_status_dict().get('estop_active')):
            self.write_mode('IDLE', 'trash_robot_manager', 'stop_all')
        _, reclaim_msg = self.reclaim_resources(apply=True)
        msg = '; '.join(messages) or 'manager-owned components stopped'
        return ok, msg + '; ' + reclaim_msg

    def cleanup_dead_registry(self) -> dict[str, dict[str, Any]]:
        registry = self.read_registry()
        alive = {}
        for name, entry in registry.items():
            if self.registry_entry_alive(name, entry):
                alive[name] = entry
        if alive != registry:
            self.write_registry(alive)
        return alive

    def fastdds_shm_files(self) -> list[Path]:
        shm = Path('/dev/shm')
        if not shm.exists():
            return []
        patterns = ('fastrtps_*', 'sem.fastrtps_*', 'fastdds_*', 'sem.fastdds_*')
        files: list[Path] = []
        for pattern in patterns:
            files.extend(shm.glob(pattern))
        return [p for p in files if p.exists()]

    def reclaim_resources(self, apply: bool = False) -> tuple[bool, str]:
        removed = 0
        shm_skipped = 0
        stale = len(self.read_registry()) - len(self.cleanup_dead_registry())
        if apply and bool(self.profile_get('process', 'cleanup_fastdds_shm_on_stop', default=True)):
            # Do not delete FastDDS shared-memory files from inside a live ROS
            # service callback. The manager and the caller may be using those
            # same segments to deliver the service response on RDK/TROS.
            shm_skipped = len([path for path in self.fastdds_shm_files() if path.exists()])

        now = time.time()
        runtime_days = float(self.profile_get('disk', 'runtime_log_retention_days', default=14) or 14)
        report_days = float(self.profile_get('disk', 'test_report_retention_days', default=30) or 30)
        for base, days in ((self.runtime / 'logs', runtime_days), (self.runtime / 'test_reports', report_days)):
            if not apply or not base.exists():
                continue
            cutoff = now - days * 86400.0
            for path in base.rglob('*'):
                try:
                    if path.is_file() and path.stat().st_mtime < cutoff:
                        path.unlink()
                        removed += 1
                except OSError:
                    pass
        return True, f'reclaimed stale_registry={stale} removed_files={removed} shm_skipped_live={shm_skipped}'

    def status_dict(self) -> dict[str, Any]:
        registry = self.cleanup_dead_registry()
        processes = {}
        for name, entry in registry.items():
            pid = int(entry.get('pid', 0) or 0)
            processes[name] = {
                'pid': pid,
                'pgid': int(entry.get('pgid', pid) or pid),
                'alive': self.registry_entry_alive(name, entry),
                'age_sec': round(time.time() - float(entry.get('started_at', time.time())), 1),
                'log_path': entry.get('log_path', ''),
            }
        video_pid = 0
        video_pid_file = self.runtime / 'video_stream.pid'
        if video_pid_file.exists():
            try:
                video_pid = int(video_pid_file.read_text(encoding='utf-8').strip() or 0)
            except ValueError:
                video_pid = 0
        patterns = self.profile_get('manager', 'component_patterns', default={}) or {}
        components = {
            'camera': self.pattern_alive(str(patterns.get('camera', 'realsense2_camera_node'))),
            'video': self.pattern_alive(str(patterns.get('video', 'light_mjpeg_streamer'))),
            'base': self.pattern_alive(str(patterns.get('base', 'serial_base_node|sllidar_node'))),
            'nav': self.pattern_alive(str(patterns.get('nav', 'nav2_bt_navigator|controller_server|planner_server|amcl'))),
            'mission': self.pattern_alive(str(patterns.get('mission', 'trash_mission_supervisor|/lib/trash_robot_mission/mission_supervisor'))),
            'vlm': self.pattern_alive(str(patterns.get('vlm', 'vlm_trash_classifier'))),
            'depth_locator': self.pattern_alive(str(patterns.get('depth_locator', 'pixel_depth_locator'))),
            'handeye_transformer': self.pattern_alive(str(patterns.get('handeye_transformer', 'handeye_target_transformer'))),
            'grasp': self.pattern_alive(str(patterns.get('grasp', 'roarm_sort_grasper|pixel_depth_locator|vlm_trash_classifier'))),
            'arm': bool(self.arm_status_dict().get('ok')),
            'handeye': self.pattern_alive(str(patterns.get('handeye', 'handeye_web_calibrator'))),
        }
        resources = self.resource_status_dict()
        mode = self.read_mode()
        motion_lock = self.motion_lock_status_dict()
        dds = self.dds_status_dict()
        system_state = {
            'current_state': 'ESTOP_LOCKED' if motion_lock.get('estop_active') else str(mode.get('mode') or 'IDLE'),
            'previous_state': '',
            'active_mode_lock': str(mode.get('mode') or 'IDLE'),
            'active_motion_lock': str(motion_lock.get('motion_owner') or 'IDLE'),
            'estop_active': bool(motion_lock.get('estop_active')),
            'fault_level': 'ESTOP_REQUIRED' if motion_lock.get('estop_active') else 'INFO',
            'fault_code': 'E9001' if motion_lock.get('estop_active') else 'E0000',
            'fault_message': str(motion_lock.get('estop_reason') or ''),
            'last_transition_time': float(mode.get('stamp') or 0.0),
            'can_reset': bool(motion_lock.get('estop_active')),
            'can_start_navigation': not bool(motion_lock.get('estop_active')) and str(mode.get('mode') or 'IDLE') != 'CALIBRATION',
            'can_start_grasp': not bool(motion_lock.get('estop_active')) and str(mode.get('mode') or 'IDLE') not in ('CALIBRATION', 'ARM_MANUAL'),
            'can_start_calibration': not bool(motion_lock.get('estop_active')) and str(mode.get('mode') or 'IDLE') not in ('GRASP', 'NAVIGATION', 'VIDEO', 'ARM_MANUAL'),
        }
        return {
            'stamp': time.time(),
            'root': str(self.ws),
            'processes': processes,
            'components': components,
            'arm': self.arm_status_dict(),
            'mode': mode,
            'motion_lock': motion_lock,
            'dds': dds,
            'system_state': system_state,
            'resources': resources,
            'video_pid': video_pid,
        }

    def resource_status_dict(self) -> dict[str, Any]:
        disk_path = Path('/')
        try:
            usage = shutil.disk_usage(disk_path)
            disk_used_percent = round((usage.used / usage.total) * 100.0, 1)
            disk_free_mb = round(usage.free / (1024.0 * 1024.0), 1)
        except OSError:
            disk_used_percent = None
            disk_free_mb = None

        mem_available_mb = None
        try:
            values: dict[str, int] = {}
            for line in Path('/proc/meminfo').read_text(encoding='utf-8').splitlines():
                parts = line.split()
                if len(parts) >= 2:
                    values[parts[0].rstrip(':')] = int(parts[1])
            mem_available_mb = round(values.get('MemAvailable', 0) / 1024.0, 1)
        except OSError:
            pass

        temp_c = None
        for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
            try:
                temp_c = round(float(path.read_text(encoding='utf-8').strip()) / 1000.0, 1)
                break
            except (OSError, ValueError):
                continue

        root_warn = float(self.profile_get('disk', 'root_warn_percent', default=90) or 90)
        root_block = float(self.profile_get('disk', 'root_block_percent', default=95) or 95)
        runtime_log_mb = round(self.log_size_mb(self.runtime / 'logs'), 1)
        shm_files = self.fastdds_shm_files()
        return {
            'disk_used_percent': disk_used_percent,
            'disk_free_mb': disk_free_mb,
            'disk_warn': bool(disk_used_percent is not None and disk_used_percent >= root_warn),
            'disk_block': bool(disk_used_percent is not None and disk_used_percent >= root_block),
            'mem_available_mb': mem_available_mb,
            'temp_c': temp_c,
            'runtime_log_mb': runtime_log_mb,
            'fastdds_shm_count': len(shm_files),
            'profile_file': str(self.profile_file),
        }

    def publish_status(self) -> None:
        status = self.status_dict()
        payload = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        self.system_state_pub.publish(String(data=payload))
        self.resource_pub.publish(String(data=json.dumps(status.get('resources', {}), ensure_ascii=False)))

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    ensure_robot_dds_environment()
    rclpy.init(args=args)
    node = RobotManager()
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


if __name__ == '__main__':
    main()
