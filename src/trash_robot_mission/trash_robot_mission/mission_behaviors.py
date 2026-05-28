from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

from geometry_msgs.msg import Twist


class MissionNode(Protocol):
    """Runtime methods provided by MissionSupervisor.

    The behavior layer owns scheduling decisions. The supervisor owns ROS I/O,
    TF, action clients, and mutable mission data.
    """


BASE = 'base'
NAV = 'nav'
ARM = 'arm'
CAMERA = 'camera'
VLM = 'vlm'


@dataclass(frozen=True)
class MissionStrategy:
    state: str
    name: str
    resources: frozenset[str]

    def tick(self, node: MissionNode) -> None:
        raise NotImplementedError


class StrategyScheduler:
    def __init__(self, strategies: list[MissionStrategy]) -> None:
        self.strategies = {strategy.state: strategy for strategy in strategies}
        self.active_strategy = ''
        self.resource_owners: dict[str, str] = {}

    def snapshot(self) -> dict[str, object]:
        return {
            'active_strategy': self.active_strategy,
            'resource_owners': dict(self.resource_owners),
        }

    def tick(self, node: MissionNode, state: str) -> None:
        strategy = self.strategies.get(state)
        if strategy is None:
            self.active_strategy = ''
            self.resource_owners = {}
            return

        self.active_strategy = strategy.name
        self.resource_owners = {resource: strategy.name for resource in sorted(strategy.resources)}
        try:
            strategy.tick(node)
        except Exception as exc:  # pragma: no cover - runtime safety path.
            try:
                node.get_logger().error(f'mission strategy failed: {strategy.name}: {exc}')
            except Exception:
                pass
            node.begin_recovery(f'STRATEGY_ERROR {strategy.name} {exc}')


class PatrolNavigatingStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('PATROL_NAVIGATING', 'patrol_nav', frozenset({NAV, BASE, CAMERA}))

    def tick(self, node: MissionNode) -> None:
        with node.lock:
            enabled = node.grasp_enabled

        if enabled and node.is_target_fresh():
            start_refresh = False
            with node.lock:
                if node.detection_confirm_sec <= 0.0:
                    node.set_resume_after_target_locked()
                    start_refresh = True
                else:
                    node.state = 'TARGET_CONFIRMING'
                    node.target_confirm_start_time = time.time()
                    node.last_event = 'TARGET_SEEN_CONFIRMING'
            if start_refresh:
                node.begin_target_refresh('TARGET_SEEN_STOP_REFRESH')
        elif enabled and node.is_local_candidate_fresh():
            with node.lock:
                cls = str(node.local_candidate.get('class_name') or 'candidate')
                conf = float(node.local_candidate.get('confidence') or 0.0)
            node.begin_local_candidate_target_nav(f'YOLO_LOCK_TARGET class={cls} conf={conf:.2f}')
        elif enabled and node.is_vlm_visual_candidate_fresh():
            with node.lock:
                obj = str(node.vlm_visual_candidate.get('object_name') or 'vlm_target')
                label = str(node.vlm_visual_candidate.get('trash_label') or '')
                latency_ms = float(node.vlm_visual_candidate.get('latency_ms') or 0.0)
                node.set_resume_after_target_locked()
            node.begin_target_refresh(
                f'VLM_VISUAL_STOP_REFRESH object={obj} label={label} latency_ms={latency_ms:.0f}'
            )


class TargetConfirmingStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('TARGET_CONFIRMING', 'target_confirm', frozenset({BASE, CAMERA}))

    def tick(self, node: MissionNode) -> None:
        with node.lock:
            enabled = node.grasp_enabled
        if not enabled:
            with node.lock:
                node.state = 'PATROL_NAVIGATING'
                node.last_event = 'TARGET_CONFIRM_CANCELLED'
                node.target_confirm_start_time = 0.0
        elif not node.is_target_fresh():
            with node.lock:
                node.state = 'PATROL_NAVIGATING'
                node.last_event = 'TARGET_LOST_DURING_CONFIRM'
                node.target_confirm_start_time = 0.0
        elif time.time() - node.target_confirm_start_time >= node.detection_confirm_sec:
            with node.lock:
                node.set_resume_after_target_locked()
            node.begin_target_refresh('TARGET_CONFIRMED_STOP_REFRESH')


class TargetRefreshStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('TARGET_REFRESH', 'target_refresh', frozenset({BASE, CAMERA, VLM}))

    def tick(self, node: MissionNode) -> None:
        if node.has_target_after_refresh_start():
            with node.lock:
                node.state = 'STOP_NAV'
                node.last_event = 'TARGET_REFRESHED_STOP_NAV'
        elif time.time() - node.target_refresh_start_time > node.target_refresh_timeout_sec:
            with node.lock:
                node.state = 'RESUME_PATROL'
                node.last_event = 'TARGET_REFRESH_TIMEOUT_RESUME'
                node.target_refresh_start_time = 0.0
        else:
            node.publish_stop(repeat=3)


class StopNavStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('STOP_NAV', 'stop_nav', frozenset({BASE, NAV}))

    def tick(self, node: MissionNode) -> None:
        node.cancel_nav()
        node.publish_stop(repeat=8)
        if node.target_nav_enabled and node.start_target_nav_approach():
            return
        if node.is_final_vlm_grasp_ready():
            node.trigger_grasp('TARGET_SAFE_TRIGGER_GRASP')
            return
        with node.lock:
            node.state = 'LOCAL_APPROACH'
            node.last_event = 'TARGET_CONFIRMED_START_LOCAL_APPROACH'
            node.local_approach_start_time = time.time()


class TargetNavApproachStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('TARGET_NAV_APPROACH', 'target_nav_approach', frozenset({NAV, BASE}))

    def tick(self, node: MissionNode) -> None:
        if time.time() - node.target_nav_start_time > node.target_nav_timeout_sec:
            node.cancel_nav()
            node.begin_visual_align('TARGET_NAV_TIMEOUT_ALIGN')


class VisualAlignStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('VISUAL_ALIGN', 'visual_align', frozenset({BASE, CAMERA}))

    def tick(self, node: MissionNode) -> None:
        node.visual_align_tick()


class FinalVlmRefreshStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('FINAL_VLM_REFRESH', 'final_vlm_refresh', frozenset({BASE, CAMERA, VLM}))

    def tick(self, node: MissionNode) -> None:
        node.final_vlm_refresh_tick()


class LocalApproachStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('LOCAL_APPROACH', 'local_approach', frozenset({BASE, CAMERA, VLM}))

    def tick(self, node: MissionNode) -> None:
        node.local_approach_tick()


class GraspSortStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('GRASP_SORT', 'grasp_sort', frozenset({BASE, ARM, CAMERA, VLM}))

    def tick(self, node: MissionNode) -> None:
        if node.grasp_uses_action:
            if node.grasp_result_future is not None and node.grasp_result_future.done():
                return
            if time.time() - node.grasp_start_time > node.grasp_timeout_sec:
                node.begin_recovery('GRASP_ACTION_TIMEOUT')
            return

        if node.grasp_future is not None and node.grasp_future.done():
            try:
                result = node.grasp_future.result()
                if result is not None and not result.success:
                    node.begin_recovery(f'GRASP_REJECTED {result.message}')
            except Exception as exc:
                node.begin_recovery(f'GRASP_CALL_ERROR {exc}')
        if time.time() - node.grasp_start_time > node.grasp_timeout_sec:
            node.begin_recovery('GRASP_TIMEOUT')


class PatrolDwellStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('PATROL_DWELL', 'patrol_dwell', frozenset({BASE}))

    def tick(self, node: MissionNode) -> None:
        with node.lock:
            enabled = node.grasp_enabled
        if enabled and node.is_local_candidate_fresh():
            with node.lock:
                cls = str(node.local_candidate.get('class_name') or 'candidate')
                conf = float(node.local_candidate.get('confidence') or 0.0)
            node.begin_local_candidate_target_nav(f'YOLO_LOCK_TARGET_DWELL class={cls} conf={conf:.2f}')
            return

        node.publish_stop()
        send_next_patrol = False
        with node.lock:
            elapsed = time.time() - node.patrol_dwell_start_time
            if elapsed >= node.patrol_dwell_sec:
                if node.patrol_dwell_route_done:
                    node.state = 'IDLE'
                    node.last_event = 'NAV_ROUTE_DONE'
                    node.patrol_dwell_route_done = False
                else:
                    node.state = 'PATROL_NAVIGATING'
                    node.last_event = f'PATROL_DWELL_DONE {node.patrol_dwell_waypoint}'
                    send_next_patrol = True
                node.patrol_dwell_start_time = 0.0
                node.patrol_dwell_sec = 0.0
                node.patrol_dwell_waypoint = ''
        if send_next_patrol:
            node.send_current_nav_goal()


class RecoveryHomeStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('RECOVERY_HOME', 'recovery_home', frozenset({BASE}))

    def tick(self, node: MissionNode) -> None:
        node.publish_stop()
        if time.time() - node.recovery_start_time >= node.recovery_hold_sec:
            with node.lock:
                node.state = 'RESUME_PATROL'
                node.last_event = 'RECOVERY_DONE'


class ResumePatrolStrategy(MissionStrategy):
    def __init__(self) -> None:
        super().__init__('RESUME_PATROL', 'resume_patrol', frozenset({BASE, NAV}))

    def tick(self, node: MissionNode) -> None:
        node.publish_stop()
        with node.lock:
            if node.resume_route_done:
                node.state = 'IDLE'
                node.last_event = 'RESUME_ROUTE_DONE'
                node.target_confirm_start_time = 0.0
                node.resume_route_done = False
                node.target_map = None
                node.target_approach_goal = None
                return
            if node.waypoints:
                node.waypoint_index = node.resume_waypoint_index % len(node.waypoints)
            node.state = 'PATROL_NAVIGATING'
            node.last_event = 'RESUME_PATROL'
            node.target_confirm_start_time = 0.0
        node.send_current_nav_goal()


def create_default_scheduler() -> StrategyScheduler:
    return StrategyScheduler([
        PatrolNavigatingStrategy(),
        TargetConfirmingStrategy(),
        TargetRefreshStrategy(),
        StopNavStrategy(),
        TargetNavApproachStrategy(),
        VisualAlignStrategy(),
        FinalVlmRefreshStrategy(),
        LocalApproachStrategy(),
        GraspSortStrategy(),
        PatrolDwellStrategy(),
        RecoveryHomeStrategy(),
        ResumePatrolStrategy(),
    ])
