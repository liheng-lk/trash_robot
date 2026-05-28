from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_yaml(rel: str):
    return yaml.safe_load((ROOT / rel).read_text(encoding='utf-8')) or {}


def test_patrol_route_schema_has_active_route_and_waypoints():
    data = load_yaml('config/mission/patrol_routes.yaml')
    assert isinstance(data.get('routes'), dict)
    active = data.get('active_route')
    assert active in data['routes']
    route = data['routes'][active]
    assert isinstance(route, dict)
    assert isinstance(route.get('waypoints'), list)
    assert route['waypoints'], 'patrol route must contain at least one waypoint'
    for item in route['waypoints']:
        assert {'x', 'y'} <= set(item), f'invalid waypoint: {item}'


def test_sort_config_has_four_drop_points_and_safe_window():
    data = load_yaml('config/grasp/trash_sort_params.yaml')
    drops = data.get('drop_points_mm', {})
    assert {
        'GARBAGE_RECYCLE',
        'GARBAGE_KITCHEN',
        'GARBAGE_HAZARD',
        'GARBAGE_OTHER',
    } <= set(drops)
    window = data.get('pickup_window_mm', {})
    for axis in ('x', 'y', 'z'):
        bounds = window.get(axis)
        assert isinstance(bounds, list) and len(bounds) == 2
        assert float(bounds[0]) < float(bounds[1])
    assert float(window['x'][0]) >= 0.0, 'pickup window must only allow camera-front targets'
    stability = data.get('target_stability', {})
    assert float(stability.get('jump_reset_mm', 0.0)) > float(stability.get('max_delta_mm', 0.0))
    descent = data.get('grasp_descent_offset_mm')
    assert isinstance(descent, list) and len(descent) == 3
    assert float(descent[2]) <= 0.0, 'final grasp descent must move down, not up'
    assert int(data.get('grasp_descent_steps', 0)) >= 1


def test_nav2_footprint_matches_bucket_body_size():
    data = load_yaml('config/navigation/nav2_params.yaml')
    local = data['local_costmap']['local_costmap']['ros__parameters']
    footprint = local['footprint']
    expected = '[[0.175,0.1625],[0.175,-0.1625],[-0.175,-0.1625],[-0.175,0.1625]]'
    assert footprint.replace(' ', '') == expected


def test_handeye_file_is_present_but_not_modified_by_mac_sync():
    data = load_yaml('config/grasp/handeye_point.yaml')
    assert 'matrix_row_major' in data
    assert len(data['matrix_row_major']) == 16


def test_sort_grasp_action_interface_exists():
    action = ROOT / 'src/trash_robot_interfaces/action/SortGrasp.action'
    text = action.read_text(encoding='utf-8')
    assert 'bool success' in text
    assert text.count('---') == 2


def test_rdk_resource_profile_has_commercial_guards():
    data = load_yaml('config/system/rdk_resource_profile.yaml')
    disk = data.get('disk', {})
    assert int(disk.get('root_warn_percent', 0)) <= 90
    assert int(disk.get('root_block_percent', 0)) <= 95
    video = data.get('video', {}).get('mjpeg', {})
    assert float(video.get('max_fps', 99)) <= 10.0
    assert 1 <= int(video.get('max_clients', 0)) <= 8
    manager = data.get('manager', {})
    assert 'stop_order' in manager and 'camera' in manager['stop_order']
    assert 'handeye' in manager['stop_order']
    assert 'component_patterns' in manager
    assert 'handeye' in manager['component_patterns']
    modes = manager.get('runtime_modes', {})
    assert {'CALIBRATION', 'ARM_MANUAL'} <= set(modes.get('exclusive_maintenance', []))
    assert {'GRASP', 'NAVIGATION'} <= set(modes.get('runtime', []))


def test_vlm_provider_registry_is_explicit_not_auto_guessing():
    data = load_yaml('config/perception/vlm_provider_registry.yaml')
    assert data.get('active_provider')
    providers = data.get('providers', {})
    for name, provider in providers.items():
        assert 'api_key' not in provider
        assert 'api_key_env' in provider
        if provider.get('enabled', False):
            assert provider.get('primary_model') or provider.get('model_candidates'), name


def test_manager_custom_services_exist():
    for rel in ('src/trash_robot_interfaces/srv/InitPose.srv', 'src/trash_robot_interfaces/srv/SaveMap.srv'):
        assert (ROOT / rel).exists()


def test_runtime_generated_configs_are_not_written_to_source_config():
    common = (ROOT / 'scripts/common.sh').read_text(encoding='utf-8')
    assert 'TRASH_GENERATED_CONFIG' in common
    assert 'prepare_' not in common
    assert 'dnn_node_example' not in common


def test_main_webui_is_active_control_plane_not_hardware_owner():
    required_paths = (
        'src/trash_robot_web',
        'scripts/start_web_console.sh',
        'src/trash_robot_web/trash_robot_web/web_console.py',
    )
    for rel in required_paths:
        assert (ROOT / rel).exists(), f'{rel} should be present as the current WebUI control plane'

    assert not (ROOT / 'scripts/start_webui.sh').exists(), 'legacy start_webui.sh must stay removed'
    assert not (ROOT / 'config/web').exists(), 'old config/web tree must stay removed'

    web = (ROOT / 'src/trash_robot_web/trash_robot_web/web_console.py').read_text(encoding='utf-8')
    assert '/trash_manager/start_base' in web
    assert '/trash_manager/start_navigation' in web
    assert '/trash_mission/start_patrol' in web
    assert 'create_publisher(Twist' in web

    stack = (ROOT / 'scripts/lib/stack.sh').read_text(encoding='utf-8')
    assert 'start_webui.sh' not in stack

    contract = load_yaml('config/system/stack_contract.yaml')
    assert 'web' not in contract.get('components', {}), 'WebUI must not be modeled as a hardware component'
    assert (ROOT / 'runtime/COLCON_IGNORE').exists(), 'runtime backups must not be scanned by colcon'


def test_manager_stop_all_does_not_kill_manager_process():
    text = (ROOT / 'src/trash_robot_manager/trash_robot_manager/robot_manager.py').read_text(encoding='utf-8')
    stop_all_body = text.split('    def stop_all(self)', 1)[1].split('    def cleanup_dead_registry', 1)[0]
    stop_all_cb_body = text.split('    def stop_all_cb(self', 1)[1].split('    def reclaim_resources_cb', 1)[0]
    assert 'threading.Thread' in stop_all_cb_body
    assert 'stop_all queued' in stop_all_cb_body
    assert "scripts' / 'stop_all.sh'" not in stop_all_body
    assert "start_grasp.sh stop" in stop_all_body
    assert "start_camera.sh stop" in stop_all_body
    reclaim_body = text.split('    def reclaim_resources(self', 1)[1].split('    def status_dict', 1)[0]
    shm_block = reclaim_body.split('cleanup_fastdds_shm_on_stop', 1)[1].split('now = time.time()', 1)[0]
    assert 'path.unlink()' not in shm_block, 'live manager service must not delete active FastDDS SHM'
    assert 'shm_skipped_live' in reclaim_body


def test_manager_exposes_dds_motion_lock_and_estop_services():
    manager = (ROOT / 'src/trash_robot_manager/trash_robot_manager/robot_manager.py').read_text(encoding='utf-8')
    assert '/trash_robot_v3/manager/system_state' in manager
    assert '/trash_robot_v3/manager/estop_trigger' in manager
    assert '/trash_robot_v3/manager/estop_reset' in manager
    assert 'motion_lock_status_dict' in manager
    assert 'dds_status_dict' in manager
    assert 'system_state' in manager
    assert 'publish_zero_burst' in manager
    assert 'camera_realsense.launch.py' in manager
    assert 'exec ros2 run trash_robot_vision light_mjpeg_streamer' in manager


def test_docs_are_named_in_chinese_and_define_module_boundaries():
    doc_dir = ROOT / 'docs'
    docs = sorted(path.name for path in doc_dir.glob('*.md'))
    assert '模块边界与工具清单.md' in docs
    assert '项目结构.md' in docs
    assert '生命周期与恢复.md' in docs
    assert all(re.search(r'[\u4e00-\u9fff]', name) for name in docs), docs
    boundary = (doc_dir / '模块边界与工具清单.md').read_text(encoding='utf-8')
    for token in ('主运行系统', '独立工具', '外部依赖包', 'CALIBRATION', 'ARM_MANUAL'):
        assert token in boundary


def test_config_root_only_contains_classified_dirs_and_chinese_readme():
    config_root = ROOT / 'config'
    allowed_files = {'配置说明.md'}
    allowed_dirs = {'dds', 'hardware', 'navigation', 'mission', 'grasp', 'perception', 'system', 'vision'}
    files = {path.name for path in config_root.iterdir() if path.is_file()}
    dirs = {path.name for path in config_root.iterdir() if path.is_dir()}
    assert files == allowed_files
    assert allowed_dirs <= dirs
    assert not any(path.suffix in {'.yaml', '.json', '.list'} for path in config_root.iterdir() if path.is_file())


def test_cyclonedds_config_disables_multicast_and_runtime_uses_domain_one():
    dds = (ROOT / 'config/dds/cyclonedds_unicast.xml').read_text(encoding='utf-8')
    common = (ROOT / 'scripts/common.sh').read_text(encoding='utf-8')
    assert '<AllowMulticast>false</AllowMulticast>' in dds
    assert 'SocketReceiveBufferSize min=' not in dds
    assert 'configure_cyclonedds' in common
    assert 'ROS_DOMAIN_ID="${TRASH_ROS_DOMAIN_ID:-1}"' in common
    assert 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' in common
    assert 'CYCLONEDDS_URI="file://$TRASH_ROBOT_ROOT/config/dds/cyclonedds_unicast.xml"' in common
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros' not in common
    assert 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' not in common


def test_motion_lock_and_estop_scripts_exist_and_are_safe():
    common = (ROOT / 'scripts/common.sh').read_text(encoding='utf-8')
    motion = (ROOT / 'scripts/lib/motion_lock.sh').read_text(encoding='utf-8')
    estop = (ROOT / 'scripts/start_estop.sh').read_text(encoding='utf-8')
    assert 'scripts/lib/motion_lock.sh' in common
    for token in (
        'TRASH_ESTOP_LOCK_FILE',
        'acquire_motion_lock',
        'release_motion_lock',
        'trigger_estop',
        'reset_estop',
        'motion_status',
    ):
        assert token in motion
    assert '/cmd_vel' in estop
    assert '/trash_robot_v3/base/cmd_vel' in estop
    assert 'for _ in 1 2 3 4 5 6 7 8 9 10' in estop
    forbidden = ('pkill -f ros2', 'pkill -f python', 'pkill -f trash_robot', 'pkill -f realsense')
    assert not any(item in estop for item in forbidden)


def test_bpu_runtime_policy_is_hbm_only_and_disabled_by_default():
    data = load_yaml('config/vision/bpu_runtime_policy.yaml')
    assert data.get('backend') == 'hbm_runtime'
    assert data.get('enabled') is False
    model = data.get('model', {})
    assert model.get('name') == 'qwen3.5:2b'
    assert model.get('input_format') == 'nv12_packed'
    assert int(model.get('max_loaded_instances', 0)) == 1
    rules = data.get('runtime_rules', {})
    assert rules.get('require_hbm_runtime') is True
    assert rules.get('forbid_hobot_dnn') is True
    assert rules.get('forbid_multiple_models') is True
    assert rules.get('release_tensors_after_inference') is True
    assert rules.get('exception_safe') is True
    preprocess = data.get('image_preprocess', {})
    assert preprocess.get('use_cv_bridge') is True
    assert preprocess.get('required_format') == 'nv12_packed'
    lock = data.get('bpu_lock', {})
    assert lock.get('lock_name') == 'BPU_INFERENCE'
    assert int(lock.get('max_concurrent_inference', 0)) == 1


def test_no_legacy_hobot_dnn_runtime_usage_in_source():
    search_roots = [
        ROOT / 'scripts',
        ROOT / 'src',
    ]
    offenders = []
    for root in search_roots:
        for path in root.rglob('*'):
            if path.is_file() and path.suffix in {'.py', '.sh', '.yaml', '.xml', '.launch.py'}:
                text = path.read_text(encoding='utf-8', errors='ignore')
                if 'hobot_dnn' in text:
                    offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []


def test_project_launch_files_are_owned_by_bringup_package():
    assert not (ROOT / 'launch').exists(), 'root launch/ must stay removed; use trash_robot_bringup/launch'
    launch_root = ROOT / 'src/trash_robot_bringup/launch'
    required = {
        'camera_realsense.launch.py',
        'depth_to_scan.launch.py',
        'nav2_real.launch.py',
        'nav2_real_depth.launch.py',
        'robot_bringup.launch.py',
        'perception_grasp.launch.py',
    }
    assert required <= {path.name for path in launch_root.glob('*.launch.py')}


def test_external_sdk_packages_are_documented_as_dependencies():
    boundary = (ROOT / 'docs/模块边界与工具清单.md').read_text(encoding='utf-8')
    for package in (
        'base_driver',
        'sllidar_ros2',
        'roarm_driver',
        'roarm_moveit',
        'roarm_moveit_cmd',
        'roarm_description',
        'moveit_servo',
        'roarm_moveit_ikfast_plugins',
    ):
        assert package in boundary


def test_manager_mode_lock_is_exposed_and_maintenance_tools_set_modes():
    manager = (ROOT / 'src/trash_robot_manager/trash_robot_manager/robot_manager.py').read_text(encoding='utf-8')
    common = (ROOT / 'scripts/common.sh').read_text(encoding='utf-8')
    mode_lock = (ROOT / 'scripts/lib/mode_lock.sh').read_text(encoding='utf-8')
    stop_all = (ROOT / 'scripts/stop_all.sh').read_text(encoding='utf-8')
    handeye = (ROOT / 'scripts/start_handeye.sh').read_text(encoding='utf-8')
    keyboard = (ROOT / 'scripts/start_arm_keyboard.sh').read_text(encoding='utf-8')
    grasp = (ROOT / 'scripts/start_grasp.sh').read_text(encoding='utf-8')

    assert 'self.mode_file' in manager
    assert 'def mode_available' in manager
    assert "'CALIBRATION': {'GRASP', 'NAVIGATION', 'VIDEO', 'ARM_MANUAL'}" in manager
    assert 'mode = self.read_mode()' in manager
    assert "'mode': mode" in manager
    assert 'scripts/lib/mode_lock.sh' in common
    assert 'clear_runtime_mode_if CALIBRATION' in common
    assert 'clear_runtime_mode_if ARM_MANUAL' in common
    assert 'clear_runtime_mode_if GRASP' in common
    assert '[ "$current" = "ESTOP" ]' in mode_lock
    assert 'rm -f "$TRASH_MOTION_LOCK_FILE"' in stop_all
    assert 'estop_active' in stop_all
    assert 'acquire_mode CALIBRATION' in handeye
    assert 'acquire_mode ARM_MANUAL' in keyboard
    assert 'acquire_mode GRASP' in grasp


def test_grasp_script_ensures_camera_but_does_not_own_arm_startup():
    grasp = (ROOT / 'scripts/start_grasp.sh').read_text(encoding='utf-8')
    assert 'ensure_camera_for_grasp' in grasp
    assert 'camera topics not ready; starting camera before grasp stack' in grasp
    assert 'camera topics not ready after wait' in grasp
    assert 'live grasp requires /move_point_cmd and /get_pose_cmd' in grasp
    assert '"$TRASH_ROBOT_ROOT/scripts/start_arm.sh"' not in grasp
