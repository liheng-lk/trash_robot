from __future__ import annotations

import asyncio
import base64
import copy
import json
import os
import re
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from ai_msgs.msg import PerceptionTargets
except ImportError:
    PerceptionTargets = None  # type: ignore[assignment]


DEFAULT_CONFIG_FILE = '/home/sunrise/trash_robot_v3/config/perception/vlm_trash_classifier.yaml'
CANONICAL_LABELS = {
    'GARBAGE_RECYCLE',
    'GARBAGE_OTHER',
    'GARBAGE_HAZARD',
    'GARBAGE_KITCHEN',
}

BATTERY_KEYWORDS = (
    'battery',
    'alkaline',
    'cell',
    'aa',
    'aaa',
    '电池',
    '干电池',
)

ALLOWED_OBJECT_SHAPES = {
    'unknown',
    'compact',
    'slender',
    'round',
    'cylindrical',
    'flat',
    'crumpled',
    'bag',
    'box',
    'soft_irregular',
    'fragile',
}

ALLOWED_GRASP_TYPES = {
    'unknown',
    'pinch',
    'top_down_pin',
    'side_pinch',
    'clamp_midbody',
    'scoop',
}

ALLOWED_GRASP_WIDTH_HINTS = {
    'unknown',
    'narrow',
    'medium',
    'wide',
}

LOCAL_HOBOT_IGNORE_CLASSES = {
    'person',
    'chair',
    'couch',
    'bed',
    'dining table',
    'toilet',
    'potted plant',
    'tv',
}

LOCAL_HOBOT_TRASH_KEYWORDS = (
    'trash',
    'garbage',
    'waste',
    'debris',
    'paper',
    'crumpled',
    'tissue',
    'napkin',
    'cardboard',
    'plastic',
    'bottle',
    'can',
    'cup',
    'battery',
    'peel',
    '纸',
    '纸团',
    '垃圾',
    '碎屑',
    '瓶',
    '罐',
    '电池',
    '果皮',
)

LOCAL_HOBOT_KITCHEN_CLASSES = {
    'banana',
    'apple',
    'orange',
    'broccoli',
    'carrot',
    'sandwich',
    'hot dog',
    'pizza',
    'donut',
    'cake',
}

LOCAL_HOBOT_RECYCLE_CLASSES = {
    'bottle',
    'wine glass',
    'cup',
    'book',
    'bowl',
    'scissors',
}

LOCAL_HOBOT_HAZARD_CLASSES = {
    'cell phone',
    'laptop',
    'mouse',
    'keyboard',
    'remote',
}


PROMPT = """你是垃圾分类机器人视觉抓取点规划模块。只识别当前画面里最适合机械臂抓取的一个垃圾目标，并给出夹爪最适合闭合的抓取点。

分类规则必须固定：
1. 水果、果皮、剩饭、食物残渣 => GARBAGE_KITCHEN
2. 纸张、纸团、纸盒、纸板、塑料瓶、易拉罐、玻璃瓶 => GARBAGE_RECYCLE
3. 电池、充电宝、电子小件、药品 => GARBAGE_HAZARD
4. 无法明确但确实是垃圾 => GARBAGE_OTHER
5. 人、桌椅、墙、地面、背景、非垃圾物体 => has_target=false

抓取点规则必须固定：
1. center_norm 不是 bbox 几何中心，而是夹爪闭合点/grasp point，必须落在目标可见表面上。
2. 纸团、纸张、果皮、软包装：抓取点选在物体可见区域的中央偏厚/偏皱处，避开边缘。
3. 瓶子、易拉罐等较粗圆柱：抓取点选在物体中段，不要选瓶口、边角或反光空洞。
4. 电池等细长硬物要单独处理：
   - 先给出紧贴电池本体的 bbox，不要把地面、夹爪或阴影放进 bbox。
   - 如果电池竖直/近似竖直站立，抓取点选在可见电池本体长轴中部附近，沿宽度居中，不要故意偏向电池前端或后端。
   - 如果电池横躺，抓取点选在长轴中段，沿宽度居中，避开正负极端头。
   - 不确定前后方向时，优先选择 bbox 内电池实体的几何中段；后端会用本地几何规则二次稳定抓取点。
5. 扁平物体：选上表面中心偏近相机的实物区域，避免把地面或阴影当成抓取点。
6. 如果物体被遮挡、太贴边、抓取点不明确，输出 has_target=false。

抓取策略规则必须固定：
1. object_shape 从 compact/slender/round/cylindrical/flat/crumpled/bag/box/soft_irregular/fragile/unknown 中选择。
2. grasp_type 从 pinch/top_down_pin/side_pinch/clamp_midbody/scoop/unknown 中选择。
3. grasp_strategy 必须用短英文 snake_case，优先使用：
   - slender_midbody：电池、笔状物、细长硬物，夹中段。
   - crumpled_center：纸团、皱纸、软包装，夹厚实中心。
   - flat_sheet_center：扁平纸张/纸片，压向上表面中心，避免边缘。
   - cylindrical_midbody：瓶子、易拉罐、圆柱物，夹中段。
   - soft_irregular_center：果皮、软垃圾，夹厚实皱褶区。
   - compact_top_pinch：其他小型垃圾，从上方夹中心。
4. risk_flags 是字符串数组，可包含 floor_contact/transparent/reflective/too_close_to_gripper/partially_occluded/edge_grasp/low_depth_confidence。
5. major_axis_angle_deg 是目标长轴在图像中的角度，无法判断填 0。

只输出一个 JSON 对象，不要 Markdown，不要解释文字。字段必须是：
{
  "has_target": true,
  "trash_label": "GARBAGE_KITCHEN",
  "object_name": "banana peel",
  "bbox_norm": [0.32, 0.40, 0.58, 0.72],
  "center_norm": [0.45, 0.56],
  "grasp_point_norm": [0.45, 0.56],
  "object_shape": "soft_irregular",
  "grasp_type": "pinch",
  "grasp_strategy": "soft_irregular_center",
  "grasp_width_hint": "medium",
  "major_axis_angle_deg": 20,
  "risk_flags": ["floor_contact"],
  "grasp_hint": "pinch visible thick middle of the peel",
  "confidence": 0.86,
  "reason": "fruit peel is kitchen waste"
}

bbox_norm、center_norm、grasp_point_norm 必须是 0 到 1 之间的归一化图像坐标，bbox_norm 顺序为 [x_min,y_min,x_max,y_max]。
center_norm 和 grasp_point_norm 必须表示同一个抓取点，且必须位于 bbox 内的实体目标区域，不要放在地面、阴影、空洞或背景上。
如果没有明确垃圾目标，输出：
{"has_target": false, "trash_label": "", "object_name": "", "bbox_norm": [0,0,0,0], "center_norm": [0,0], "grasp_point_norm": [0,0], "object_shape": "unknown", "grasp_type": "unknown", "grasp_strategy": "", "grasp_width_hint": "unknown", "major_axis_angle_deg": 0, "risk_flags": [], "grasp_hint": "", "confidence": 0, "reason": "no graspable trash"}
"""


@dataclass
class VlmProvider:
    name: str
    model: str
    model_candidates: list[str]
    base_url: str
    api_key_env: str
    enabled: bool = True


def strip_json_fence(text: str) -> str:
    text = (text or '').strip()
    if text.startswith('```'):
        text = re.sub(r'^```(?:json)?\s*', '', text, flags=re.IGNORECASE)
        text = re.sub(r'\s*```$', '', text)
    return text.strip()


def extract_json_object(text: str) -> dict[str, Any]:
    cleaned = strip_json_fence(text)
    try:
        value = json.loads(cleaned)
    except json.JSONDecodeError:
        start = cleaned.find('{')
        end = cleaned.rfind('}')
        if start < 0 or end <= start:
            raise
        value = json.loads(cleaned[start : end + 1])
    if not isinstance(value, dict):
        raise ValueError('VLM response is not a JSON object')
    return value


def _float_list(value: Any, length: int) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'expected list length {length}')
    return [float(v) for v in value]


def _inside_unit(values: list[float]) -> bool:
    return all(0.0 <= v <= 1.0 for v in values)


def _enum_value(value: Any, allowed: set[str], default: str) -> str:
    text = str(value or '').strip().lower()
    return text if text in allowed else default


def _string_list(value: Any, max_items: int = 8) -> list[str]:
    if not isinstance(value, list):
        return []
    items: list[str] = []
    for item in value:
        text = str(item or '').strip().lower()
        if text:
            items.append(text[:64])
        if len(items) >= max_items:
            break
    return items


def _axis_angle(value: Any) -> float:
    try:
        angle = float(value)
    except (TypeError, ValueError):
        return 0.0
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def rejected_vlm_result(raw: dict[str, Any], reason: str) -> dict[str, Any]:
    """Return a complete fail-closed VLM result payload.

    Keeping the full schema prevents downstream code from seeing partial
    `cleaned={}` dictionaries when a provider response is rejected.
    """
    try:
        confidence = float(raw.get('confidence', 0.0))
    except (TypeError, ValueError):
        confidence = 0.0
    try:
        bbox = _float_list(raw.get('bbox_norm'), 4)
    except (TypeError, ValueError):
        bbox = [0.0, 0.0, 0.0, 0.0]
    try:
        center = _float_list(raw.get('grasp_point_norm', raw.get('center_norm')), 2)
    except (TypeError, ValueError):
        center = [0.0, 0.0]
    return {
        'has_target': False,
        'trash_label': '',
        'object_name': str(raw.get('object_name') or ''),
        'bbox_norm': bbox,
        'center_norm': center,
        'grasp_point_norm': center,
        'object_shape': _enum_value(raw.get('object_shape'), ALLOWED_OBJECT_SHAPES, 'unknown'),
        'grasp_type': _enum_value(raw.get('grasp_type'), ALLOWED_GRASP_TYPES, 'unknown'),
        'grasp_strategy': str(raw.get('grasp_strategy') or '').strip()[:64],
        'grasp_width_hint': _enum_value(raw.get('grasp_width_hint'), ALLOWED_GRASP_WIDTH_HINTS, 'unknown'),
        'major_axis_angle_deg': _axis_angle(raw.get('major_axis_angle_deg')),
        'risk_flags': _string_list(raw.get('risk_flags')),
        'grasp_hint': str(raw.get('grasp_hint') or ''),
        'confidence': confidence,
        'reason': reason,
    }


def validate_vlm_result(raw: dict[str, Any], min_confidence: float) -> tuple[bool, str, dict[str, Any]]:
    has_target = bool(raw.get('has_target', False))
    if not has_target:
        reason = str(raw.get('reason') or 'NO_TARGET')
        return False, reason, rejected_vlm_result(raw, reason)

    label = str(raw.get('trash_label') or '').strip()
    if label not in CANONICAL_LABELS:
        reason = f'INVALID_LABEL {label}'
        return False, reason, rejected_vlm_result(raw, reason)

    confidence = float(raw.get('confidence', 0.0))
    if confidence < min_confidence:
        reason = f'LOW_CONFIDENCE {confidence:.3f}'
        return False, reason, rejected_vlm_result(raw, reason)

    bbox = _float_list(raw.get('bbox_norm'), 4)
    center = _float_list(raw.get('grasp_point_norm', raw.get('center_norm')), 2)
    if not _inside_unit(bbox) or not _inside_unit(center):
        reason = f'COORD_OUT_OF_RANGE bbox={bbox} center={center}'
        return False, reason, rejected_vlm_result(raw, reason)
    if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        reason = f'INVALID_BBOX bbox={bbox}'
        return False, reason, rejected_vlm_result(raw, reason)
    bbox_w = bbox[2] - bbox[0]
    bbox_h = bbox[3] - bbox[1]
    bbox_area = bbox_w * bbox_h
    if bbox_w < 0.045 or bbox_h < 0.045 or bbox_area < 0.003:
        reason = f'BBOX_TOO_SMALL bbox={bbox} size={bbox_w:.3f}x{bbox_h:.3f}'
        return False, reason, rejected_vlm_result(raw, reason)
    touches_edge = bbox[0] <= 0.025 or bbox[1] <= 0.025 or bbox[2] >= 0.975 or bbox[3] >= 0.975
    if touches_edge and (bbox_w < 0.18 or bbox_h < 0.18):
        reason = f'EDGE_PARTIAL_SMALL_BBOX bbox={bbox} size={bbox_w:.3f}x{bbox_h:.3f}'
        return False, reason, rejected_vlm_result(raw, reason)
    bbox_margin = 0.03
    if not (
        bbox[0] - bbox_margin <= center[0] <= bbox[2] + bbox_margin
        and bbox[1] - bbox_margin <= center[1] <= bbox[3] + bbox_margin
    ):
        reason = f'GRASP_POINT_OUTSIDE_BBOX bbox={bbox} grasp={center}'
        return False, reason, rejected_vlm_result(raw, reason)

    cleaned = {
        'has_target': True,
        'trash_label': label,
        'object_name': str(raw.get('object_name') or '').strip(),
        'bbox_norm': bbox,
        'center_norm': center,
        'grasp_point_norm': center,
        'object_shape': _enum_value(raw.get('object_shape'), ALLOWED_OBJECT_SHAPES, 'unknown'),
        'grasp_type': _enum_value(raw.get('grasp_type'), ALLOWED_GRASP_TYPES, 'unknown'),
        'grasp_strategy': str(raw.get('grasp_strategy') or '').strip().lower()[:64],
        'grasp_width_hint': _enum_value(raw.get('grasp_width_hint'), ALLOWED_GRASP_WIDTH_HINTS, 'unknown'),
        'major_axis_angle_deg': _axis_angle(raw.get('major_axis_angle_deg')),
        'risk_flags': _string_list(raw.get('risk_flags')),
        'grasp_hint': str(raw.get('grasp_hint') or '').strip(),
        'confidence': confidence,
        'reason': str(raw.get('reason') or '').strip(),
    }
    return True, 'OK', cleaned


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _is_battery_result(result: dict[str, Any]) -> bool:
    haystack = ' '.join(
        str(result.get(key) or '').lower()
        for key in ('object_name', 'grasp_hint', 'reason')
    )
    return any(keyword in haystack for keyword in BATTERY_KEYWORDS)


def infer_grasp_semantics(result: dict[str, Any]) -> dict[str, Any]:
    """Fill missing grasp strategy fields with deterministic local semantics."""
    adjusted = copy.deepcopy(result)
    if not bool(adjusted.get('has_target', False)):
        return adjusted

    haystack = ' '.join(
        str(adjusted.get(key) or '').lower()
        for key in ('object_name', 'grasp_hint', 'reason', 'trash_label')
    )
    strategy = str(adjusted.get('grasp_strategy') or '').strip().lower()
    shape = _enum_value(adjusted.get('object_shape'), ALLOWED_OBJECT_SHAPES, 'unknown')
    grasp_type = _enum_value(adjusted.get('grasp_type'), ALLOWED_GRASP_TYPES, 'unknown')
    width_hint = _enum_value(adjusted.get('grasp_width_hint'), ALLOWED_GRASP_WIDTH_HINTS, 'unknown')

    if _is_battery_result(adjusted) or any(word in haystack for word in ('pen', 'stick', 'cylindrical cell', '细长')):
        strategy = strategy or 'slender_midbody'
        shape = 'slender' if shape == 'unknown' else shape
        grasp_type = 'clamp_midbody' if grasp_type == 'unknown' else grasp_type
        width_hint = 'narrow' if width_hint == 'unknown' else width_hint
    elif any(word in haystack for word in ('crumpled', 'paper ball', 'tissue', 'napkin', '纸团', '纸巾')):
        strategy = strategy or 'crumpled_center'
        shape = 'crumpled' if shape == 'unknown' else shape
        grasp_type = 'pinch' if grasp_type == 'unknown' else grasp_type
        width_hint = 'medium' if width_hint == 'unknown' else width_hint
    elif any(word in haystack for word in ('paper', 'cardboard', 'sheet', '纸张', '纸片', '纸板')):
        strategy = strategy or 'flat_sheet_center'
        shape = 'flat' if shape == 'unknown' else shape
        grasp_type = 'top_down_pin' if grasp_type == 'unknown' else grasp_type
        width_hint = 'wide' if width_hint == 'unknown' else width_hint
    elif any(word in haystack for word in ('bottle', 'can', 'cup', '瓶', '易拉罐', '罐')):
        strategy = strategy or 'cylindrical_midbody'
        shape = 'cylindrical' if shape == 'unknown' else shape
        grasp_type = 'clamp_midbody' if grasp_type == 'unknown' else grasp_type
        width_hint = 'medium' if width_hint == 'unknown' else width_hint
    elif any(word in haystack for word in ('banana', 'peel', 'fruit', 'food', '果皮', '水果', '厨余')):
        strategy = strategy or 'soft_irregular_center'
        shape = 'soft_irregular' if shape == 'unknown' else shape
        grasp_type = 'pinch' if grasp_type == 'unknown' else grasp_type
        width_hint = 'medium' if width_hint == 'unknown' else width_hint
    else:
        strategy = strategy or 'compact_top_pinch'
        shape = 'compact' if shape == 'unknown' else shape
        grasp_type = 'pinch' if grasp_type == 'unknown' else grasp_type
        width_hint = 'medium' if width_hint == 'unknown' else width_hint

    adjusted['grasp_strategy'] = strategy[:64]
    adjusted['object_shape'] = shape
    adjusted['grasp_type'] = grasp_type
    adjusted['grasp_width_hint'] = width_hint
    adjusted['risk_flags'] = _string_list(adjusted.get('risk_flags'))
    adjusted['major_axis_angle_deg'] = _axis_angle(adjusted.get('major_axis_angle_deg'))
    return adjusted


def apply_geometric_grasp_rules(
    result: dict[str, Any],
    override_config: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    """Stabilize VLM grasp points for shape-specific manipulation.

    VLMs are useful for object identity and bbox, but thin batteries can drift
    along their long axis between API calls. For manipulation, use a
    deterministic mid-body grasp point and let local depth/hand-eye handle 3D.
    """
    adjusted = infer_grasp_semantics(result)
    if not bool(adjusted.get('has_target', False)):
        return adjusted

    cfg = override_config or {}
    battery_cfg = cfg.get('battery', {}) if isinstance(cfg.get('battery', {}), dict) else {}
    if not bool(battery_cfg.get('enabled', True)):
        return adjusted
    if not _is_battery_result(adjusted):
        return adjusted

    bbox = adjusted.get('bbox_norm')
    if not isinstance(bbox, list) or len(bbox) != 4:
        return adjusted
    x0, y0, x1, y1 = [float(v) for v in bbox]
    width = x1 - x0
    height = y1 - y0
    if width <= 0.0 or height <= 0.0:
        return adjusted

    aspect_threshold = float(battery_cfg.get('aspect_threshold', 1.35))
    vertical_y_ratio = float(battery_cfg.get('vertical_y_ratio', 0.52))
    horizontal_x_ratio = float(battery_cfg.get('horizontal_x_ratio', 0.50))
    margin_ratio = _clamp(float(battery_cfg.get('margin_ratio', 0.14)), 0.0, 0.35)
    inner_x0 = x0 + width * margin_ratio
    inner_x1 = x1 - width * margin_ratio
    inner_y0 = y0 + height * margin_ratio
    inner_y1 = y1 - height * margin_ratio

    if height >= width * aspect_threshold:
        grasp = [(x0 + x1) * 0.5, y0 + height * vertical_y_ratio]
        orientation = 'vertical'
    elif width >= height * aspect_threshold:
        grasp = [x0 + width * horizontal_x_ratio, (y0 + y1) * 0.5]
        orientation = 'horizontal'
    else:
        grasp = [(x0 + x1) * 0.5, (y0 + y1) * 0.5]
        orientation = 'compact'

    grasp = [
        _clamp(grasp[0], inner_x0, inner_x1),
        _clamp(grasp[1], inner_y0, inner_y1),
    ]
    adjusted['vlm_grasp_point_norm'] = list(adjusted.get('grasp_point_norm') or adjusted.get('center_norm') or grasp)
    adjusted['center_norm'] = grasp
    adjusted['grasp_point_norm'] = grasp
    adjusted['grasp_rule'] = f'battery_bbox_midbody_{orientation}'
    hint = str(adjusted.get('grasp_hint') or '').strip()
    adjusted['grasp_hint'] = (hint + '; ' if hint else '') + adjusted['grasp_rule']
    return adjusted


def normalized_bbox_to_pixels(
    bbox: list[float],
    center: list[float],
    width: int,
    height: int,
) -> tuple[float, float, float, float, float, float]:
    width = max(1, int(width))
    height = max(1, int(height))
    x0 = max(0.0, min(width - 1.0, bbox[0] * width))
    y0 = max(0.0, min(height - 1.0, bbox[1] * height))
    x1 = max(0.0, min(width - 1.0, bbox[2] * width))
    y1 = max(0.0, min(height - 1.0, bbox[3] * height))
    u = max(0.0, min(width - 1.0, center[0] * width))
    v = max(0.0, min(height - 1.0, center[1] * height))
    return x0, y0, x1, y1, u, v


def map_color_pixel_to_depth_pixel(
    u: float,
    v: float,
    bbox_px: tuple[float, float, float, float],
    color_w: int,
    color_h: int,
    depth_w: int,
    depth_h: int,
) -> tuple[float, float, tuple[float, float, float, float]]:
    if color_w <= 0 or color_h <= 0 or depth_w <= 0 or depth_h <= 0:
        raise ValueError(
            f'DEPTH_SIZE_MISMATCH color={color_w}x{color_h} depth={depth_w}x{depth_h}'
        )
    scale_x = float(depth_w) / float(color_w)
    scale_y = float(depth_h) / float(color_h)
    x0, y0, x1, y1 = bbox_px
    mapped_u = _clamp(float(u) * scale_x, 0.0, float(depth_w - 1))
    mapped_v = _clamp(float(v) * scale_y, 0.0, float(depth_h - 1))
    mapped_bbox = (
        _clamp(float(x0) * scale_x, 0.0, float(depth_w - 1)),
        _clamp(float(y0) * scale_y, 0.0, float(depth_h - 1)),
        _clamp(float(x1) * scale_x, 0.0, float(depth_w - 1)),
        _clamp(float(y1) * scale_y, 0.0, float(depth_h - 1)),
    )
    return mapped_u, mapped_v, mapped_bbox


class VlmTrashClassifier(Node):
    def __init__(self) -> None:
        super().__init__('vlm_trash_classifier')

        self.declare_parameter('enabled', True)
        self.declare_parameter('config_file', DEFAULT_CONFIG_FILE)
        self.declare_parameter('provider', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('image_qos_reliability', 'best_effort')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('target_frame', 'image_pixel')
        self.declare_parameter('depth_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('strict_depth_size_check', True)
        self.declare_parameter('publish_debug_coordinates', True)
        self.declare_parameter('call_interval_sec', 2.0)
        self.declare_parameter('api_timeout_sec', 8.0)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('max_image_width', 640)
        self.declare_parameter('max_frame_age_sec', 1.0)
        self.declare_parameter('cache_enabled', False)
        self.declare_parameter('cache_hold_sec', 6.0)
        self.declare_parameter('cache_republish_period_sec', 0.4)
        self.declare_parameter('cache_motion_check_period_sec', 1.0)
        self.declare_parameter('cache_image_change_threshold', 18.0)
        self.declare_parameter('cache_signature_width', 96)
        self.declare_parameter('depth_min_m', 0.12)
        self.declare_parameter('depth_max_m', 1.20)
        self.declare_parameter('depth_roi_radius_px', 12)
        self.declare_parameter('depth_valid_ratio_min', 0.20)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('depth_max_age_sec', 0.3)
        self.declare_parameter('grasp_refine_enabled', True)
        self.declare_parameter('fallback_to_vlm_grasp', False)
        self.declare_parameter('grasp_refine_min_mask_area_px', 80)
        self.declare_parameter('grasp_refine_depth_band_m', 0.06)
        self.declare_parameter('grasp_refine_edge_margin_px', 5)
        self.declare_parameter('grasp_refine_min_quality', 0.55)
        self.declare_parameter('grasp_refine_candidate_step_px', 3)
        self.declare_parameter('grasp_refine_depth_std_max_m', 0.035)
        self.declare_parameter('grasp_refine_hint_radius_ratio', 0.28)
        self.declare_parameter('grasp_refine_center_weight', 0.10)
        self.declare_parameter('grasp_refine_edge_weight', 0.35)
        self.declare_parameter('grasp_refine_depth_valid_weight', 0.25)
        self.declare_parameter('grasp_refine_depth_stability_weight', 0.20)
        self.declare_parameter('grasp_refine_vlm_hint_weight', 0.10)
        self.declare_parameter('plane_fallback_enabled', False)
        self.declare_parameter('plane_fallback_handeye_file', '/home/sunrise/trash_robot_v3/config/grasp/handeye_point.yaml')
        self.declare_parameter('plane_fallback_arm_z_m', -0.265)
        self.declare_parameter('plane_fallback_min_camera_z_m', 0.05)
        self.declare_parameter('plane_fallback_max_camera_z_m', 1.0)
        self.declare_parameter('plane_fallback_depth_disagreement_m', 0.20)
        self.declare_parameter('floor_contact_use_plane_fallback', True)
        self.declare_parameter('local_hobot_topic', '/perception/detection/dosod')
        self.declare_parameter('local_hobot_max_age_sec', 1.2)
        self.declare_parameter('local_hobot_min_confidence', 0.30)
        self.declare_parameter('local_candidate_enabled', True)
        self.declare_parameter('local_candidate_topic', '/trash_local_candidate')
        self.declare_parameter('local_candidate_min_center_y_norm', 0.30)
        self.declare_parameter('local_candidate_min_area_norm', 0.0004)
        self.declare_parameter('local_candidate_max_area_norm', 0.35)
        self.declare_parameter('local_image_candidate_enabled', True)
        self.declare_parameter('local_image_candidate_period_sec', 0.30)
        self.declare_parameter('local_image_candidate_min_center_y_norm', 0.32)
        self.declare_parameter('local_image_candidate_max_center_y_norm', 0.92)
        self.declare_parameter('local_image_candidate_min_area_norm', 0.0010)
        self.declare_parameter('local_image_candidate_max_area_norm', 0.06)
        self.declare_parameter('local_image_candidate_min_edge_density', 0.010)
        self.declare_parameter('local_image_candidate_min_contrast', 10.0)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.state_lock = threading.RLock()
        self.depth_lock = threading.Lock()
        self.local_hobot_lock = threading.Lock()
        self.latest_image: Optional[Image] = None
        self.latest_image_time = 0.0
        self.latest_depth: Optional[Image] = None
        self.latest_depth_time = 0.0
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_local_hobot: Optional[Any] = None
        self.latest_local_hobot_time = 0.0
        self.inflight = False
        self.worker_thread: Optional[threading.Thread] = None
        self.shutdown_event = threading.Event()
        self.last_call_time = 0.0
        self.last_status_time = 0.0
        self.config = self.load_config(str(self.get_parameter('config_file').value))
        self.provider_registry = self.load_provider_registry()
        self.consecutive_failures = 0
        self.backoff_until = 0.0
        self.provider_health: dict[str, dict[str, Any]] = {}
        self.cached_result: Optional[dict[str, Any]] = None
        self.cached_stamp = 0.0
        self.cached_signature: Optional[Any] = None
        self.last_cache_publish_time = 0.0
        self.last_cache_motion_check_time = 0.0
        self.camera_point_history: list[tuple[float, np.ndarray, str]] = []
        self.camera_point_average_locked = False
        self.last_camera_average_status_time = 0.0
        self.last_local_image_candidate_time = 0.0

        self.pixel_pub = self.create_publisher(PointStamped, '/trash_target_pixel', 10)
        self.label_pub = self.create_publisher(String, '/trash_target_label', 10)
        self.raw_label_pub = self.create_publisher(String, '/trash_target_raw_label', 10)
        self.status_pub = self.create_publisher(String, '/trash_detection_status', 10)
        self.vlm_result_pub = self.create_publisher(String, '/trash_vlm_result', 10)
        self.grasp_plan_pub = self.create_publisher(String, '/trash_grasp_plan', 10)
        self.bbox_pub = self.create_publisher(String, '/trash_target_bbox', 10)
        self.local_candidate_pub = self.create_publisher(
            String,
            self.setting_str('local_candidate_topic', '/trash_local_candidate'),
            10,
        )
        self.camera_point_pub = self.create_publisher(PointStamped, '/trash_target_camera_point', 10)
        # Legacy alias for WebUI / mission subscribers (same message as camera_point).
        self.legacy_camera_point_pub = self.create_publisher(PointStamped, '/trash_target_point_camera', 10)
        self.depth_status_pub = self.create_publisher(String, '/trash_target_depth_status', 10)
        self.debug_image_pub = self.create_publisher(Image, '/trash_vlm_debug_image', 5)

        image_qos = self.image_qos_profile()
        self.create_subscription(
            Image,
            self.setting_str('image_topic', '/camera/camera/color/image_raw'),
            self.image_callback,
            image_qos,
        )
        self.create_subscription(
            Image,
            self.setting_str('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.setting_str('camera_info_topic', '/camera/camera/color/camera_info'),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        if PerceptionTargets is not None:
            self.create_subscription(
                PerceptionTargets,
                self.setting_str('local_hobot_topic', '/perception/detection/dosod'),
                self.local_hobot_callback,
                10,
            )
        else:
            self.get_logger().warning(
                'ai_msgs is not available in this runtime; local_hobot provider will be unavailable'
            )
        self.create_service(Trigger, '/trash_vlm/refresh', self.refresh_cache_callback)
        self.create_service(Trigger, '/trash_vlm/clear_cache', self.clear_cache_callback)
        self.timer = self.create_timer(0.2, self.maybe_call_vlm)
        self.get_logger().info(
            f'vlm_trash_classifier image={self.setting_str("image_topic", "/camera/camera/color/image_raw")} '
            f'depth={self.setting_str("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")} '
            f'camera_info={self.setting_str("camera_info_topic", "/camera/camera/color/camera_info")} '
            f'config={self.get_parameter("config_file").value} '
            f'image_qos={self.setting_str("image_qos_reliability", "best_effort")}'
        )

    def image_qos_profile(self):
        reliability = self.setting_str('image_qos_reliability', 'best_effort').strip().lower()
        if reliability in ('reliable', '1', 'true'):
            return QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
        return qos_profile_sensor_data

    def load_config(self, path_text: str) -> dict[str, Any]:
        path = Path(path_text)
        if not path.exists():
            self.get_logger().warning(f'VLM config not found, using built-in defaults: {path}')
            return {}
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        return data if isinstance(data, dict) else {}

    def load_provider_registry(self) -> dict[str, Any]:
        config_file = Path(str(self.get_parameter('config_file').value))
        registry_text = str(self.config.get('provider_registry_file') or '').strip()
        candidates: list[Path] = []
        if registry_text:
            registry_path = Path(registry_text)
            candidates.append(registry_path if registry_path.is_absolute() else config_file.parent / registry_path)
        candidates.append(config_file.parent / 'vlm_provider_registry.yaml')
        project_root = Path('/home/sunrise/trash_robot_v3')
        for parent in config_file.parents:
            if parent.name == 'config':
                project_root = parent.parent
                break
        runtime_override = project_root / 'runtime' / 'config' / 'vlm_provider_registry.override.yaml'
        base_registry: dict[str, Any] = {}
        for path in candidates:
            if path.exists():
                data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
                if isinstance(data, dict):
                    self.get_logger().info(f'loaded VLM provider registry: {path}')
                    base_registry = data
                    break
        apply_override = os.environ.get('TRASH_APPLY_VLM_OVERRIDE', '').strip() in ('1', 'true', 'yes')
        if base_registry and runtime_override.exists() and apply_override:
            try:
                override = yaml.safe_load(runtime_override.read_text(encoding='utf-8')) or {}
            except (OSError, yaml.YAMLError) as exc:
                self.get_logger().warning(f'VLM provider override load failed: {exc}')
                override = {}
            if isinstance(override, dict):
                self.get_logger().info(f'loaded VLM provider override: {runtime_override}')
                return self.deep_merge(base_registry, override)
        if base_registry and runtime_override.exists() and not apply_override:
            self.get_logger().info(
                f'ignoring VLM provider override (set TRASH_APPLY_VLM_OVERRIDE=1 to apply): {runtime_override}'
            )
        if base_registry:
            return base_registry
        self.get_logger().warning('VLM provider registry not found; using provider config only')
        return {}

    def deep_merge(self, base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
        result = copy.deepcopy(base)
        for key, value in override.items():
            if isinstance(value, dict) and isinstance(result.get(key), dict):
                result[key] = self.deep_merge(result[key], value)
            else:
                result[key] = copy.deepcopy(value)
        return result

    def setting_float(self, key: str, default: float) -> float:
        value = self.config.get(key, self.get_parameter(key).value if self.has_parameter(key) else default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def setting_int(self, key: str, default: int) -> int:
        value = self.config.get(key, self.get_parameter(key).value if self.has_parameter(key) else default)
        try:
            return int(value)
        except (TypeError, ValueError):
            return int(default)

    def setting_bool(self, key: str, default: bool) -> bool:
        value = self.config.get(key, self.get_parameter(key).value if self.has_parameter(key) else default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def setting_str(self, key: str, default: str) -> str:
        value = self.config.get(key, self.get_parameter(key).value if self.has_parameter(key) else default)
        text = str(value or '').strip()
        return text if text else str(default)

    def image_callback(self, msg: Image) -> None:
        with self.lock:
            self.latest_image = msg
            self.latest_image_time = time.time()
        self.maybe_publish_image_candidate(msg)

    def depth_callback(self, msg: Image) -> None:
        with self.depth_lock:
            self.latest_depth = msg
            self.latest_depth_time = time.time()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        with self.depth_lock:
            self.latest_camera_info = msg

    def local_hobot_callback(self, msg: Any) -> None:
        with self.local_hobot_lock:
            self.latest_local_hobot = msg
            self.latest_local_hobot_time = time.time()
        self.publish_local_candidate()

    def local_hobot_category(self, class_name: str) -> tuple[Optional[str], int]:
        text = str(class_name or '').strip().lower()
        if not text or text in LOCAL_HOBOT_IGNORE_CLASSES:
            return None, 0
        if 'battery' in text or '电池' in text:
            return 'GARBAGE_HAZARD', 4
        if any(word in text for word in ('banana', 'apple', 'orange', 'food', 'peel', '果皮', '食物')):
            return 'GARBAGE_KITCHEN', 3
        if any(word in text for word in ('paper', 'cardboard', 'bottle', 'can', 'cup', 'plastic', '纸', '瓶', '罐')):
            return 'GARBAGE_RECYCLE', 3
        if any(word in text for word in LOCAL_HOBOT_TRASH_KEYWORDS):
            return 'GARBAGE_OTHER', 2
        if text in LOCAL_HOBOT_HAZARD_CLASSES:
            return 'GARBAGE_HAZARD', 3
        if text in LOCAL_HOBOT_KITCHEN_CLASSES:
            return 'GARBAGE_KITCHEN', 2
        if text in LOCAL_HOBOT_RECYCLE_CLASSES:
            return 'GARBAGE_RECYCLE', 2
        return None, 0

    def local_hobot_no_target(self, reason: str) -> dict[str, Any]:
        return {
            'has_target': False,
            'trash_label': '',
            'object_name': '',
            'bbox_norm': [0.0, 0.0, 0.0, 0.0],
            'center_norm': [0.0, 0.0],
            'grasp_point_norm': [0.0, 0.0],
            'object_shape': 'unknown',
            'grasp_type': 'unknown',
            'grasp_strategy': '',
            'grasp_width_hint': 'unknown',
            'major_axis_angle_deg': 0.0,
            'risk_flags': [],
            'grasp_hint': '',
            'confidence': 0.0,
            'reason': reason,
        }

    def local_hobot_result(self, image_w: int, image_h: int) -> tuple[dict[str, Any], float]:
        if PerceptionTargets is None:
            raise RuntimeError('local_hobot provider requires ai_msgs, but ai_msgs is not installed')

        now = time.time()
        with self.local_hobot_lock:
            msg = self.latest_local_hobot
            stamp = self.latest_local_hobot_time
        if msg is None:
            return self.local_hobot_no_target('LOCAL_HOBOT_WAIT_TARGET_TOPIC'), 0.0

        age = now - stamp
        max_age = max(0.2, self.setting_float('local_hobot_max_age_sec', 1.2))
        if age > max_age:
            return self.local_hobot_no_target(f'LOCAL_HOBOT_STALE age={age:.2f}s'), 0.0

        width = max(1, int(image_w))
        height = max(1, int(image_h))
        min_conf = _clamp(self.setting_float('local_hobot_min_confidence', 0.30), 0.05, 0.95)
        best: Optional[dict[str, Any]] = None

        for target in list(getattr(msg, 'targets', []) or []):
            target_type = str(getattr(target, 'type', '') or '').strip()
            for roi in list(getattr(target, 'rois', []) or []):
                rect = getattr(roi, 'rect', None)
                if rect is None:
                    continue
                cls_name = str(getattr(roi, 'type', '') or target_type).strip()
                label, priority = self.local_hobot_category(cls_name)
                if label is None or priority <= 0:
                    continue
                conf = float(getattr(roi, 'confidence', 0.0) or 0.0)
                if conf < min_conf:
                    continue
                x0 = float(getattr(rect, 'x_offset', 0.0) or 0.0)
                y0 = float(getattr(rect, 'y_offset', 0.0) or 0.0)
                bw = float(getattr(rect, 'width', 0.0) or 0.0)
                bh = float(getattr(rect, 'height', 0.0) or 0.0)
                if bw <= 3.0 or bh <= 3.0:
                    continue
                x1 = x0 + bw
                y1 = y0 + bh
                bbox = [
                    _clamp(x0 / width, 0.0, 1.0),
                    _clamp(y0 / height, 0.0, 1.0),
                    _clamp(x1 / width, 0.0, 1.0),
                    _clamp(y1 / height, 0.0, 1.0),
                ]
                if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                    continue
                area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                rank = (priority, float(conf), float(area))
                if best is None or rank > best['rank']:
                    center = [(bbox[0] + bbox[2]) * 0.5, (bbox[1] + bbox[3]) * 0.5]
                    best = {
                        'rank': rank,
                        'label': label,
                        'class_name': cls_name,
                        'bbox': bbox,
                        'center': center,
                        'confidence': _clamp(conf, 0.0, 1.0),
                    }

        if best is None:
            return self.local_hobot_no_target('LOCAL_HOBOT_NO_GRASPABLE_TARGET'), age * 1000.0

        result = {
            'has_target': True,
            'trash_label': best['label'],
            'object_name': best['class_name'],
            'bbox_norm': best['bbox'],
            'center_norm': best['center'],
            'grasp_point_norm': best['center'],
            'object_shape': 'unknown',
            'grasp_type': 'unknown',
            'grasp_strategy': '',
            'grasp_width_hint': 'unknown',
            'major_axis_angle_deg': 0.0,
            'risk_flags': [],
            'grasp_hint': f'local_hobot_dosod class={best["class_name"]}',
            'confidence': float(best['confidence']),
            'reason': f'LOCAL_HOBOT_OK class={best["class_name"]}',
        }
        return result, age * 1000.0

    def latest_image_size(self) -> tuple[int, int]:
        with self.lock:
            msg = self.latest_image
        if msg is None:
            return 640, 480
        width = int(getattr(msg, 'width', 0) or 0)
        height = int(getattr(msg, 'height', 0) or 0)
        return max(1, width or 640), max(1, height or 480)

    def publish_local_candidate(self) -> None:
        if not self.setting_bool('local_candidate_enabled', True):
            return
        if PerceptionTargets is None:
            return
        try:
            width, height = self.latest_image_size()
            result, latency_ms = self.local_hobot_result(width, height)
        except Exception as exc:  # noqa: BLE001 - candidate stream is diagnostic, not mission-critical.
            payload = {
                'has_candidate': False,
                'provider': 'hobot_dosod',
                'reason': f'LOCAL_CANDIDATE_ERROR {exc}',
                'stamp': time.time(),
            }
            self.local_candidate_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            return

        now = time.time()
        has_candidate = bool(result.get('has_target', False))
        reason = str(result.get('reason') or 'LOCAL_CANDIDATE_NONE')
        bbox = list(result.get('bbox_norm') or [0.0, 0.0, 0.0, 0.0])
        center = list(result.get('center_norm') or [0.0, 0.0])
        if has_candidate and len(bbox) == 4 and len(center) == 2:
            area = max(0.0, float(bbox[2] - bbox[0])) * max(0.0, float(bbox[3] - bbox[1]))
            min_center_y = self.setting_float('local_candidate_min_center_y_norm', 0.30)
            min_area = self.setting_float('local_candidate_min_area_norm', 0.0004)
            max_area = self.setting_float('local_candidate_max_area_norm', 0.35)
            if float(center[1]) < min_center_y:
                has_candidate = False
                reason = f'LOCAL_CANDIDATE_ABOVE_FLOOR center_y={float(center[1]):.3f}'
            elif area < min_area:
                has_candidate = False
                reason = f'LOCAL_CANDIDATE_TOO_SMALL area={area:.4f}'
            elif area > max_area:
                has_candidate = False
                reason = f'LOCAL_CANDIDATE_TOO_LARGE area={area:.4f}'

        payload = {
            'has_candidate': has_candidate,
            'provider': 'hobot_dosod',
            'class_name': str(result.get('object_name') or ''),
            'trash_label': str(result.get('trash_label') or ''),
            'confidence': float(result.get('confidence') or 0.0),
            'bbox_norm': bbox,
            'center_norm': center,
            'age_ms': round(float(latency_ms), 1),
            'stamp': now,
            'reason': reason,
        }
        self.local_candidate_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def maybe_publish_image_candidate(self, msg: Image) -> None:
        if not self.setting_bool('local_image_candidate_enabled', True):
            return
        now = time.time()
        period = max(0.05, self.setting_float('local_image_candidate_period_sec', 0.30))
        if now - self.last_local_image_candidate_time < period:
            return
        self.last_local_image_candidate_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - candidate detector must not break image ingest.
            self.local_candidate_pub.publish(
                String(
                    data=json.dumps(
                        {
                            'has_candidate': False,
                            'provider': 'local_image_blob',
                            'reason': f'LOCAL_IMAGE_CONVERT_FAILED {exc}',
                            'stamp': now,
                        },
                        ensure_ascii=False,
                    )
                )
            )
            return

        candidate = self.detect_image_blob_candidate(image)
        if candidate is None:
            payload = {
                'has_candidate': False,
                'provider': 'local_image_blob',
                'reason': 'LOCAL_IMAGE_NO_FLOOR_BLOB',
                'stamp': now,
            }
        else:
            payload = {
                'has_candidate': True,
                'provider': 'local_image_blob',
                'class_name': 'floor_debris',
                'trash_label': 'GARBAGE_OTHER',
                'confidence': candidate['confidence'],
                'bbox_norm': candidate['bbox_norm'],
                'center_norm': candidate['center_norm'],
                'stamp': now,
                'reason': candidate['reason'],
            }
        self.local_candidate_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def detect_image_blob_candidate(self, image: np.ndarray) -> Optional[dict[str, Any]]:
        h, w = image.shape[:2]
        if h <= 0 or w <= 0:
            return None

        min_cy = _clamp(self.setting_float('local_image_candidate_min_center_y_norm', 0.32), 0.0, 1.0)
        max_cy = _clamp(self.setting_float('local_image_candidate_max_center_y_norm', 0.92), 0.0, 1.0)
        min_area = self.setting_float('local_image_candidate_min_area_norm', 0.0010)
        max_area = self.setting_float('local_image_candidate_max_area_norm', 0.06)
        min_edge_density = self.setting_float('local_image_candidate_min_edge_density', 0.010)
        min_contrast = self.setting_float('local_image_candidate_min_contrast', 10.0)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        white_mask = cv2.inRange(hsv, np.array([0, 0, 145], dtype=np.uint8), np.array([179, 95, 255], dtype=np.uint8))
        color_mask = cv2.inRange(hsv, np.array([0, 55, 65], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
        mask = cv2.bitwise_or(white_mask, color_mask)

        y0 = int(h * max(0.0, min_cy - 0.18))
        y1 = int(h * min(1.0, max_cy + 0.08))
        floor_mask = np.zeros_like(mask)
        floor_mask[y0:y1, :] = 255
        mask = cv2.bitwise_and(mask, floor_mask)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best: Optional[dict[str, Any]] = None
        image_area = float(w * h)
        edges = cv2.Canny(gray, 60, 140)

        for contour in contours:
            area_px = float(cv2.contourArea(contour))
            area_norm = area_px / image_area
            if area_norm < min_area or area_norm > max_area:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 8 or bh < 8:
                continue
            cx = (x + bw * 0.5) / w
            cy = (y + bh * 0.5) / h
            if cy < min_cy or cy > max_cy:
                continue
            aspect = max(float(bw) / max(1.0, float(bh)), float(bh) / max(1.0, float(bw)))
            if aspect > 4.2:
                continue
            fill_ratio = area_px / float(max(1, bw * bh))
            if fill_ratio < 0.16:
                continue

            patch_gray = gray[y : y + bh, x : x + bw]
            patch_edges = edges[y : y + bh, x : x + bw]
            patch_sat = saturation[y : y + bh, x : x + bw]
            patch_val = value[y : y + bh, x : x + bw]
            if patch_gray.size == 0:
                continue
            edge_density = float(np.count_nonzero(patch_edges)) / float(patch_edges.size)
            contrast = float(patch_gray.std())
            bright = float(patch_val.mean()) / 255.0
            colorful = float(patch_sat.mean()) / 255.0
            if edge_density < min_edge_density and contrast < min_contrast:
                continue

            score = (
                min(1.0, area_norm / max(min_area * 6.0, 1e-6)) * 0.35
                + min(1.0, edge_density / max(min_edge_density * 5.0, 1e-6)) * 0.25
                + min(1.0, contrast / max(min_contrast * 4.0, 1e-6)) * 0.20
                + max(bright, colorful) * 0.20
            )
            candidate = {
                'rank': score,
                'confidence': float(_clamp(0.35 + score * 0.55, 0.0, 0.92)),
                'bbox_norm': [
                    _clamp(x / w, 0.0, 1.0),
                    _clamp(y / h, 0.0, 1.0),
                    _clamp((x + bw) / w, 0.0, 1.0),
                    _clamp((y + bh) / h, 0.0, 1.0),
                ],
                'center_norm': [_clamp(cx, 0.0, 1.0), _clamp(cy, 0.0, 1.0)],
                'reason': (
                    f'LOCAL_IMAGE_BLOB_OK area={area_norm:.4f} edge={edge_density:.3f} '
                    f'contrast={contrast:.1f}'
                ),
            }
            if best is None or candidate['rank'] > best['rank']:
                best = candidate

        if best is None:
            return None
        best.pop('rank', None)
        return best

    def refresh_cache_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            with self.state_lock:
                self.config = self.load_config(str(self.get_parameter('config_file').value))
                self.provider_registry = self.load_provider_registry()
                self.backoff_until = 0.0
                self.consecutive_failures = 0
        except Exception as exc:  # noqa: BLE001 - expose config reload errors to WebUI
            response.success = False
            response.message = f'VLM config reload failed: {exc}'
            return response
        self.clear_cache('manual refresh requested')
        response.success = True
        response.message = 'VLM config reloaded; cache cleared; next frame will call API'
        return response

    def clear_cache_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.clear_cache('manual clear requested')
        response.success = True
        response.message = 'VLM cache cleared'
        return response

    def clear_cache(self, reason: str) -> None:
        with self.state_lock:
            self.cached_result = None
            self.cached_stamp = 0.0
            self.cached_signature = None
            self.last_cache_publish_time = 0.0
            self.last_cache_motion_check_time = 0.0
            self.camera_point_history = []
            self.camera_point_average_locked = False
        self.status_pub.publish(String(data=f'VLM_CACHE_CLEARED {reason}; camera_point_history_cleared'))

    def provider_names(self) -> list[str]:
        requested = str(self.get_parameter('provider').value or '').strip()
        if requested:
            return [requested]
        configured = str(self.config.get('active_provider') or self.provider_registry.get('active_provider') or '').strip()
        fallback = self.config.get('fallback_order', self.provider_registry.get('fallback_order', []))
        names = []
        if configured:
            names.append(configured)
        if isinstance(fallback, list):
            names.extend(str(v).strip() for v in fallback if str(v).strip())
        return list(dict.fromkeys(names or ['dashscope', 'zhipu']))

    def get_provider(self, name: str) -> Optional[VlmProvider]:
        registry_providers = self.provider_registry.get('providers', {})
        config_providers = self.config.get('providers', {})
        registry_item = registry_providers.get(name, {}) if isinstance(registry_providers, dict) else {}
        config_item = config_providers.get(name, {}) if isinstance(config_providers, dict) else {}
        if name == 'dashscope':
            defaults = {
                'model': 'qwen3.5-omni-flash',
                'primary_model': 'qwen3.5-omni-flash',
                'model_candidates': ['qwen3.5-omni-flash'],
                'base_url': 'https://dashscope.aliyuncs.com/compatible-mode/v1',
                'api_key_env': 'DASHSCOPE_API_KEY',
                'enabled': True,
            }
        elif name == 'zhipu':
            defaults = {
                'model': 'glm-4v-plus-0111',
                'primary_model': 'glm-4v-plus-0111',
                'model_candidates': ['glm-4v-plus-0111'],
                'base_url': 'https://open.bigmodel.cn/api/paas/v4',
                'api_key_env': 'ZHIPUAI_API_KEY',
                'enabled': True,
            }
        elif name == 'local_hobot':
            defaults = {
                'model': 'hobot_dosod',
                'primary_model': 'hobot_dosod',
                'model_candidates': ['hobot_dosod'],
                'base_url': '',
                'api_key_env': '',
                'enabled': True,
            }
        elif name == 'mimo':
            defaults = {
                'model': 'mimo-v2.5',
                'primary_model': 'mimo-v2.5',
                'model_candidates': ['mimo-v2.5', 'mimo-v2-omni'],
                'base_url': 'https://api.xiaomimimo.com/v1',
                'api_key_env': 'MIMO_API_KEY',
                'enabled': True,
            }
        else:
            defaults = {'model': '', 'primary_model': '', 'model_candidates': [], 'base_url': '', 'api_key_env': '', 'enabled': False}
        merged = {
            **defaults,
            **(registry_item if isinstance(registry_item, dict) else {}),
            **(config_item if isinstance(config_item, dict) else {}),
        }
        requested = str(self.get_parameter('provider').value or '').strip()
        if requested == name:
            merged['enabled'] = True
        if not bool(merged.get('enabled', True)):
            return None
        primary_model = str(merged.get('primary_model') or merged.get('model') or '').strip()
        raw_candidates = merged.get('model_candidates', [])
        candidates = [primary_model] if primary_model else []
        if isinstance(raw_candidates, list):
            candidates.extend(str(v).strip() for v in raw_candidates if str(v).strip())
        candidates = list(dict.fromkeys(candidates))
        return VlmProvider(
            name=name,
            model=primary_model,
            model_candidates=candidates,
            base_url=str(merged.get('base_url') or '').rstrip('/'),
            api_key_env=str(merged.get('api_key_env') or ''),
            enabled=True,
        )

    def maybe_call_vlm(self) -> None:
        if not self.setting_bool('enabled', True):
            self.publish_status_throttled('VLM_DISABLED')
            return

        now = time.time()
        with self.lock:
            msg = self.latest_image
            image_age = now - self.latest_image_time
        if msg is None:
            self.publish_status_throttled('WAIT_IMAGE')
            return
        if image_age > self.setting_float('max_frame_age_sec', 1.0):
            self.publish_status_throttled(f'IMAGE_TOO_OLD age={image_age:.2f}s')
            return

        if self.try_reuse_cache(msg, now):
            return

        with self.state_lock:
            backoff_until = self.backoff_until
            inflight = self.inflight
            last_call_time = self.last_call_time
            if now >= backoff_until and not inflight and now - last_call_time >= self.setting_float('call_interval_sec', 2.0):
                self.inflight = True
                self.last_call_time = now
                should_start = True
            else:
                should_start = False
        if now < backoff_until:
            self.publish_status_throttled(f'VLM_BACKOFF remaining={backoff_until - now:.1f}s')
            return
        if inflight or not should_start:
            return

        self.worker_thread = threading.Thread(target=self.worker, args=(msg,), daemon=True)
        self.worker_thread.start()

    def try_reuse_cache(self, msg: Image, now: float) -> bool:
        if not self.setting_bool('cache_enabled', True):
            return False
        with self.state_lock:
            result = copy.deepcopy(self.cached_result) if self.cached_result else None
            cached_stamp = self.cached_stamp
            cached_signature = None if self.cached_signature is None else self.cached_signature.copy()
            last_motion_check = self.last_cache_motion_check_time
            last_publish = self.last_cache_publish_time
            average_locked = self.camera_point_average_locked
        if not result:
            return False
        age = now - cached_stamp
        cache_hold = self.setting_float('cache_hold_sec', 6.0)
        if age > cache_hold:
            self.clear_cache(f'expired age={age:.1f}s')
            return False
        if average_locked and self.setting_bool('camera_point_average_lock_after_ready', True):
            publish_period = max(0.1, self.setting_float('cache_republish_period_sec', 0.4))
            if now - last_publish >= publish_period:
                self.publish_vlm(result, True, 'CAMERA_POINT_AVERAGE_LOCK_REUSE', cached=True)
                with self.state_lock:
                    self.last_cache_publish_time = now
                self.publish_status_throttled(
                    f'CAMERA_POINT_AVERAGE_LOCK_REUSE label={result.get("trash_label")} age={age:.1f}s no_api_call=true',
                    period=2.0,
            )
            return True

        check_period = self.setting_float('cache_motion_check_period_sec', 1.0)
        if now - last_motion_check >= check_period:
            with self.state_lock:
                self.last_cache_motion_check_time = now
            try:
                signature = self.image_signature(msg)
                if cached_signature is not None:
                    diff = float(cv2.absdiff(signature, cached_signature).mean())
                    if diff > self.setting_float('cache_image_change_threshold', 18.0):
                        self.clear_cache(f'image_changed diff={diff:.1f}')
                        return False
            except Exception as exc:  # noqa: BLE001 - cache reuse should fail open to API
                self.publish_status_throttled(f'VLM_CACHE_CHECK_FAILED {exc}', period=5.0)
                return False

        publish_period = max(0.1, self.setting_float('cache_republish_period_sec', 0.4))
        if now - last_publish >= publish_period:
            self.publish_vlm(result, True, 'CACHE_REUSE', cached=True)
            with self.state_lock:
                self.last_cache_publish_time = now
            self.publish_status_throttled(
                f'VLM_CACHE_REUSE label={result.get("trash_label")} '
                f'age={age:.1f}s no_api_call=true',
                period=2.0,
            )
        return True

    def publish_status_throttled(self, text: str, period: float = 2.0) -> None:
        now = time.time()
        if now - self.last_status_time >= period:
            self.status_pub.publish(String(data=text))
            self.last_status_time = now

    def encode_image(self, msg: Image) -> tuple[str, int, int]:
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        max_width = self.setting_int('max_image_width', 640)
        if max_width > 0 and image.shape[1] > max_width:
            scale = max_width / float(image.shape[1])
            image = cv2.resize(image, (max_width, int(round(image.shape[0] * scale))), interpolation=cv2.INTER_AREA)
        quality = self.setting_int('jpeg_quality', 70)
        ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            raise RuntimeError('failed to encode image as JPEG')
        b64 = base64.b64encode(encoded.tobytes()).decode('ascii')
        return f'data:image/jpeg;base64,{b64}', int(msg.width), int(msg.height)

    def image_signature(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        target_width = max(16, self.setting_int('cache_signature_width', 96))
        if gray.shape[1] != target_width:
            scale = target_width / float(gray.shape[1])
            target_height = max(12, int(round(gray.shape[0] * scale)))
            gray = cv2.resize(gray, (target_width, target_height), interpolation=cv2.INTER_AREA)
        return gray

    def depth_image_to_meters(self, depth_msg: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_np = np.asarray(depth)
        if depth_np.dtype == np.uint16 or depth_msg.encoding.upper() in ('16UC1', 'MONO16'):
            depth_m = depth_np.astype(np.float32) * 0.001
        elif depth_np.dtype == np.float32 or depth_msg.encoding.upper() == '32FC1':
            depth_m = depth_np.astype(np.float32, copy=False)
        else:
            raise ValueError(f'unsupported depth encoding={depth_msg.encoding} dtype={depth_np.dtype}')
        depth_m = np.array(depth_m, dtype=np.float32, copy=True)
        depth_m[~np.isfinite(depth_m)] = np.nan
        return depth_m

    def latest_depth_snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self.depth_lock:
            depth_msg = self.latest_depth
            depth_time = self.latest_depth_time
            camera_info = self.latest_camera_info
        if depth_msg is None:
            return {'ok': False, 'reason': 'WAIT_DEPTH'}
        if camera_info is None:
            return {'ok': False, 'reason': 'WAIT_CAMERA_INFO'}
        depth_age = now - depth_time
        max_age = self.setting_float('depth_max_age_sec', 0.5)
        if depth_age > max_age:
            return {'ok': False, 'reason': f'DEPTH_TOO_OLD age={depth_age:.2f}s max={max_age:.2f}s'}
        try:
            depth_m = self.depth_image_to_meters(depth_msg)
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'reason': f'DEPTH_CONVERT_ERROR {exc}', 'depth_age_sec': round(depth_age, 3)}
        return {
            'ok': True,
            'depth_m': depth_m,
            'depth_msg': depth_msg,
            'camera_info': camera_info,
            'depth_age_sec': round(depth_age, 3),
        }

    def build_object_mask_from_depth(
        self,
        depth_m: np.ndarray,
        bbox_px: tuple[float, float, float, float],
        hint_u: float,
        hint_v: float,
    ) -> dict[str, Any]:
        if depth_m.ndim != 2:
            return {'ok': False, 'reason': f'DEPTH_NOT_2D shape={depth_m.shape}'}
        height, width = depth_m.shape
        x0, y0, x1, y1 = bbox_px
        bx0 = max(0, int(np.floor(min(x0, x1))))
        by0 = max(0, int(np.floor(min(y0, y1))))
        bx1 = min(width, int(np.ceil(max(x0, x1))) + 1)
        by1 = min(height, int(np.ceil(max(y0, y1))) + 1)
        bbox_clip = [int(bx0), int(by0), int(bx1), int(by1)]
        if bx1 - bx0 < 3 or by1 - by0 < 3:
            return {'ok': False, 'reason': f'BBOX_TOO_SMALL bbox={bbox_clip}', 'bbox_clip': bbox_clip}

        roi = depth_m[by0:by1, bx0:bx1]
        min_m = self.setting_float('depth_min_m', 0.12)
        max_m = self.setting_float('depth_max_m', 1.20)
        valid_mask = np.isfinite(roi) & (roi >= min_m) & (roi <= max_m)
        valid_depths = roi[valid_mask]
        if valid_depths.size <= 0:
            return {'ok': False, 'reason': f'NO_VALID_DEPTH_IN_BBOX bbox={bbox_clip}', 'bbox_clip': bbox_clip}

        band_m = max(0.005, self.setting_float('grasp_refine_depth_band_m', 0.06))
        foreground_depth = float(np.percentile(valid_depths, 25.0))
        low = max(min_m, foreground_depth - band_m * 0.35)
        high = min(max_m, foreground_depth + band_m)
        fg_mask = valid_mask & (roi >= low) & (roi <= high)

        kernel = np.ones((3, 3), dtype=np.uint8)
        cleaned = cv2.morphologyEx(fg_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel, iterations=1)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=2)
        if int(cleaned.sum()) <= 0:
            cleaned = fg_mask.astype(np.uint8)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
        if num_labels <= 1:
            return {
                'ok': False,
                'reason': 'NO_FOREGROUND_COMPONENT',
                'bbox_clip': bbox_clip,
                'foreground_depth_m': foreground_depth,
            }

        hint_x = int(round(hint_u)) - bx0
        hint_y = int(round(hint_v)) - by0
        chosen = 0
        if 0 <= hint_x < labels.shape[1] and 0 <= hint_y < labels.shape[0]:
            label_at_hint = int(labels[hint_y, hint_x])
            if label_at_hint > 0:
                chosen = label_at_hint
        if chosen <= 0:
            best_dist = float('inf')
            for label_id in range(1, num_labels):
                area = int(stats[label_id, cv2.CC_STAT_AREA])
                if area <= 0:
                    continue
                cx, cy = centroids[label_id]
                dist = float(np.hypot(cx - hint_x, cy - hint_y))
                if dist < best_dist:
                    best_dist = dist
                    chosen = label_id
        if chosen <= 0:
            return {
                'ok': False,
                'reason': 'NO_COMPONENT_NEAR_HINT',
                'bbox_clip': bbox_clip,
                'foreground_depth_m': foreground_depth,
            }

        component = labels == chosen
        area = int(component.sum())
        min_area = max(1, self.setting_int('grasp_refine_min_mask_area_px', 80))
        if area < min_area:
            return {
                'ok': False,
                'reason': f'MASK_AREA_TOO_SMALL area={area} min={min_area}',
                'mask_area_px': area,
                'bbox_clip': bbox_clip,
                'foreground_depth_m': foreground_depth,
            }

        full_mask = np.zeros(depth_m.shape, dtype=bool)
        full_mask[by0:by1, bx0:bx1] = component
        return {
            'ok': True,
            'object_mask': full_mask,
            'mask_area_px': area,
            'foreground_depth_m': foreground_depth,
            'bbox_clip': bbox_clip,
            'reason': f'OK foreground_depth={foreground_depth:.3f} band={band_m:.3f}',
        }

    def local_depth_score(
        self,
        depth_m: np.ndarray,
        x: int,
        y: int,
        bbox_clip: list[int],
    ) -> dict[str, Any]:
        radius = max(2, int(round(self.setting_int('depth_roi_radius_px', 12) * 0.5)))
        height, width = depth_m.shape
        bx0, by0, bx1, by1 = bbox_clip
        rx0 = max(0, bx0, x - radius)
        ry0 = max(0, by0, y - radius)
        rx1 = min(width, bx1, x + radius + 1)
        ry1 = min(height, by1, y + radius + 1)
        if rx1 <= rx0 or ry1 <= ry0:
            return {'valid_ratio': 0.0, 'std_m': float('inf'), 'valid_count': 0}
        roi = depth_m[ry0:ry1, rx0:rx1]
        min_m = self.setting_float('depth_min_m', 0.12)
        max_m = self.setting_float('depth_max_m', 1.20)
        valid = roi[np.isfinite(roi) & (roi >= min_m) & (roi <= max_m)]
        valid_count = int(valid.size)
        valid_ratio = float(valid_count / roi.size) if roi.size else 0.0
        std_m = float(np.std(valid)) if valid_count >= 3 else float('inf')
        return {
            'valid_ratio': valid_ratio,
            'std_m': std_m,
            'valid_count': valid_count,
            'roi': [int(rx0), int(ry0), int(rx1), int(ry1)],
        }

    def mask_pca_angle_deg(self, mask: np.ndarray) -> dict[str, Any]:
        ys, xs = np.nonzero(mask)
        if len(xs) < 8:
            return {'ok': False, 'reason': 'PCA_NOT_ENOUGH_POINTS'}
        pts = np.column_stack([xs.astype(np.float32), ys.astype(np.float32)])
        mean = pts.mean(axis=0)
        centered = pts - mean
        cov = np.cov(centered, rowvar=False)
        try:
            values, vectors = np.linalg.eigh(cov)
        except np.linalg.LinAlgError:
            return {'ok': False, 'reason': 'PCA_FAILED'}
        axis = vectors[:, int(np.argmax(values))]
        angle = float(np.degrees(np.arctan2(axis[1], axis[0])))
        while angle > 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
        projections = centered @ axis
        return {
            'ok': True,
            'angle_deg': angle,
            'axis': axis,
            'mean': mean,
            'projection_min': float(projections.min()),
            'projection_max': float(projections.max()),
        }

    def refine_grasp_point_by_local_geometry(
        self,
        result: dict[str, Any],
        depth_m: np.ndarray,
        color_w: int,
        color_h: int,
    ) -> dict[str, Any]:
        bbox = result.get('bbox_norm') if isinstance(result.get('bbox_norm'), list) else [0.0, 0.0, 0.0, 0.0]
        vlm_center = result.get('vlm_grasp_point_norm') or result.get('grasp_point_norm') or result.get('center_norm') or [0.0, 0.0]
        x0, y0, x1, y1, hint_u_color, hint_v_color = normalized_bbox_to_pixels(
            [float(v) for v in bbox],
            [float(v) for v in vlm_center],
            color_w,
            color_h,
        )
        depth_h, depth_w = depth_m.shape[:2]
        try:
            hint_u, hint_v, depth_bbox = map_color_pixel_to_depth_pixel(
                hint_u_color,
                hint_v_color,
                (x0, y0, x1, y1),
                color_w,
                color_h,
                depth_w,
                depth_h,
            )
        except ValueError as exc:
            return {
                'ok': False,
                'reason': str(exc),
                'vlm_u': hint_u_color,
                'vlm_v': hint_v_color,
                'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
            }

        mask_result = self.build_object_mask_from_depth(depth_m, depth_bbox, hint_u, hint_v)
        if not bool(mask_result.get('ok', False)):
            return {
                'ok': False,
                'reason': str(mask_result.get('reason') or 'MASK_FAILED'),
                'vlm_u': hint_u_color,
                'vlm_v': hint_v_color,
                'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
                'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
                'object_mask': mask_result.get('object_mask'),
            }

        object_mask = mask_result['object_mask']
        bbox_clip = mask_result['bbox_clip']
        bx0, by0, bx1, by1 = bbox_clip
        crop_mask = object_mask[by0:by1, bx0:bx1].astype(np.uint8)
        dist = cv2.distanceTransform(crop_mask, cv2.DIST_L2, 3)
        max_dist = float(dist.max()) if dist.size else 0.0
        ys, xs = np.nonzero(crop_mask)
        if len(xs) <= 0 or max_dist <= 0.0:
            return {
                'ok': False,
                'reason': 'MASK_DISTANCE_EMPTY',
                'object_mask': object_mask,
                'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
            }

        step = max(1, self.setting_int('grasp_refine_candidate_step_px', 3))
        grid_keep = ((xs + bx0) % step == 0) & ((ys + by0) % step == 0)
        if not bool(grid_keep.any()):
            grid_keep = np.ones_like(xs, dtype=bool)
        xs = xs[grid_keep] + bx0
        ys = ys[grid_keep] + by0

        shape = str(result.get('object_shape') or 'unknown').lower()
        strategy = str(result.get('grasp_strategy') or '').lower()
        pca = self.mask_pca_angle_deg(object_mask) if shape in ('slender', 'cylindrical') or 'midbody' in strategy else {'ok': False}
        risk_flags = _string_list(result.get('risk_flags'))
        gripper_yaw_deg = 0.0
        if bool(pca.get('ok', False)):
            axis_angle = float(pca['angle_deg'])
            gripper_yaw_deg = axis_angle + 90.0
            vlm_axis = _axis_angle(result.get('major_axis_angle_deg'))
            axis_diff = abs(((axis_angle - vlm_axis + 90.0) % 180.0) - 90.0)
            if axis_diff > 35.0 and 'axis_uncertain' not in risk_flags:
                risk_flags.append('axis_uncertain')
        elif shape in ('slender', 'cylindrical') or 'midbody' in strategy:
            gripper_yaw_deg = _axis_angle(result.get('major_axis_angle_deg')) + 90.0

        edge_weight = self.setting_float('grasp_refine_edge_weight', 0.35)
        valid_weight = self.setting_float('grasp_refine_depth_valid_weight', 0.25)
        stable_weight = self.setting_float('grasp_refine_depth_stability_weight', 0.20)
        center_weight = self.setting_float('grasp_refine_center_weight', 0.10)
        hint_weight = self.setting_float('grasp_refine_vlm_hint_weight', 0.10)
        std_max = max(0.001, self.setting_float('grasp_refine_depth_std_max_m', 0.035))
        diag = max(1.0, float(np.hypot(bx1 - bx0, by1 - by0)))
        half_min_extent = max(1.0, min(bx1 - bx0, by1 - by0) * 0.5)
        hint_radius_ratio = _clamp(self.setting_float('grasp_refine_hint_radius_ratio', 0.28), 0.05, 1.0)
        hint_radius_px = max(12.0, diag * hint_radius_ratio)
        hint_distances = np.hypot(xs.astype(np.float32) - float(hint_u), ys.astype(np.float32) - float(hint_v))
        near_hint = hint_distances <= hint_radius_px
        # Depth-only masks often merge paper/fruit with the floor. If that
        # happens, keep the final point local to the VLM's semantic hint
        # instead of letting distanceTransform drift to the center of the floor
        # component.
        local_hint_required = not (
            shape in ('slender', 'cylindrical') or 'midbody' in strategy
        )
        if local_hint_required:
            if not bool(near_hint.any()):
                return {
                    'ok': False,
                    'reason': (
                        f'NO_LOCAL_GRASP_CANDIDATE near_hint_radius_px={hint_radius_px:.1f} '
                        f'hint_depth_px={hint_u:.1f},{hint_v:.1f}'
                    ),
                    'object_mask': object_mask,
                    'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
                }
            xs = xs[near_hint]
            ys = ys[near_hint]

        best: Optional[dict[str, Any]] = None
        for x, y in zip(xs.tolist(), ys.tolist()):
            crop_x = int(x - bx0)
            crop_y = int(y - by0)
            edge_distance = float(dist[crop_y, crop_x])
            edge_score = _clamp(edge_distance / max_dist, 0.0, 1.0)
            local = self.local_depth_score(depth_m, int(x), int(y), bbox_clip)
            valid_score = _clamp(float(local['valid_ratio']), 0.0, 1.0)
            std_m = float(local['std_m'])
            stable_score = 0.0 if not np.isfinite(std_m) else _clamp(1.0 - std_m / std_max, 0.0, 1.0)
            edge_to_bbox = min(float(x - bx0), float(bx1 - 1 - x), float(y - by0), float(by1 - 1 - y))
            center_score = _clamp(edge_to_bbox / half_min_extent, 0.0, 1.0)
            if bool(pca.get('ok', False)):
                axis = pca['axis']
                mean = pca['mean']
                proj = float((np.array([x, y], dtype=np.float32) - mean) @ axis)
                span = max(1.0, float(pca['projection_max'] - pca['projection_min']))
                axis_mid_score = _clamp(1.0 - abs(proj) / (span * 0.5), 0.0, 1.0)
                center_score = 0.5 * center_score + 0.5 * axis_mid_score
            hint_score = _clamp(1.0 - float(np.hypot(x - hint_u, y - hint_v)) / diag, 0.0, 1.0)
            score = (
                edge_weight * edge_score
                + valid_weight * valid_score
                + stable_weight * stable_score
                + center_weight * center_score
                + hint_weight * hint_score
            )
            candidate = {
                'score': float(score),
                'x': float(x),
                'y': float(y),
                'edge_distance_px': edge_distance,
                'local_depth_std_m': None if not np.isfinite(std_m) else std_m,
                'local_depth_valid_ratio': float(local['valid_ratio']),
                'local_depth_valid_count': int(local['valid_count']),
                'local_depth_roi': local.get('roi', []),
            }
            if best is None or candidate['score'] > best['score']:
                best = candidate

        if best is None:
            return {
                'ok': False,
                'reason': 'NO_GRASP_CANDIDATE',
                'object_mask': object_mask,
                'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
            }

        if shape == 'flat' or 'flat' in strategy:
            if 'floor_contact' not in risk_flags:
                risk_flags.append('floor_contact')
            if best['local_depth_std_m'] is not None and best['local_depth_std_m'] < 0.010:
                best['score'] *= 0.85

        min_quality = self.setting_float('grasp_refine_min_quality', 0.55)
        edge_margin = self.setting_float('grasp_refine_edge_margin_px', 5.0)
        if best['edge_distance_px'] < edge_margin:
            return {
                'ok': False,
                'reason': f'EDGE_DISTANCE_LOW edge={best["edge_distance_px"]:.1f} min={edge_margin:.1f}',
                'object_mask': object_mask,
                'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
                **best,
            }
        if best['score'] < min_quality:
            return {
                'ok': False,
                'reason': f'GRASP_QUALITY_LOW quality={best["score"]:.3f} min={min_quality:.3f}',
                'object_mask': object_mask,
                'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
                **best,
            }

        scale_x = float(color_w) / float(depth_w)
        scale_y = float(color_h) / float(depth_h)
        refined_u = _clamp(best['x'] * scale_x, 0.0, float(color_w - 1))
        refined_v = _clamp(best['y'] * scale_y, 0.0, float(color_h - 1))
        refined_norm = [
            _clamp(refined_u / float(max(1, color_w)), 0.0, 1.0),
            _clamp(refined_v / float(max(1, color_h)), 0.0, 1.0),
        ]
        return {
            'ok': True,
            'refined_u': float(refined_u),
            'refined_v': float(refined_v),
            'refined_depth_u': float(best['x']),
            'refined_depth_v': float(best['y']),
            'refined_center_norm': refined_norm,
            'vlm_u': float(hint_u_color),
            'vlm_v': float(hint_v_color),
            'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
            'grasp_quality': float(best['score']),
            'grasp_refine_method': f'depth_mask_distance_transform_{shape or "unknown"}',
            'mask_area_px': int(mask_result.get('mask_area_px', 0) or 0),
            'edge_distance_px': float(best['edge_distance_px']),
            'local_depth_std_m': best['local_depth_std_m'],
            'local_depth_valid_ratio': float(best['local_depth_valid_ratio']),
            'local_depth_valid_count': int(best['local_depth_valid_count']),
            'local_depth_roi': best.get('local_depth_roi', []),
            'refine_reason': str(mask_result.get('reason') or 'OK'),
            'foreground_depth_m': float(mask_result.get('foreground_depth_m', 0.0) or 0.0),
            'object_mask': object_mask,
            'depth_bbox_px': [float(v) for v in depth_bbox],
            'color_bbox_px': [float(v) for v in (x0, y0, x1, y1)],
            'depth_size': [int(depth_w), int(depth_h)],
            'color_size': [int(color_w), int(color_h)],
            'risk_flags': risk_flags,
            'gripper_yaw_deg': float(gripper_yaw_deg),
        }

    def refine_grasp_point_by_rgb_texture(
        self,
        result: dict[str, Any],
        color_w: int,
        color_h: int,
    ) -> dict[str, Any]:
        shape = str(result.get('object_shape') or 'unknown').lower()
        strategy = str(result.get('grasp_strategy') or '').lower()
        if shape in ('slender', 'cylindrical') or 'midbody' in strategy:
            return {'ok': False, 'reason': 'RGB_REFINE_SKIPPED_SLENDER'}

        bbox = result.get('bbox_norm') if isinstance(result.get('bbox_norm'), list) else [0.0, 0.0, 0.0, 0.0]
        vlm_center = result.get('vlm_grasp_point_norm') or result.get('grasp_point_norm') or result.get('center_norm') or [0.0, 0.0]
        x0, y0, x1, y1, hint_u, hint_v = normalized_bbox_to_pixels(
            [float(v) for v in bbox],
            [float(v) for v in vlm_center],
            color_w,
            color_h,
        )

        with self.lock:
            image_msg = self.latest_image
        if image_msg is None:
            return {'ok': False, 'reason': 'RGB_REFINE_WAIT_IMAGE'}

        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'reason': f'RGB_REFINE_IMAGE_CONVERT_ERROR {exc}'}

        image_h, image_w = image.shape[:2]
        if image_w <= 0 or image_h <= 0:
            return {'ok': False, 'reason': 'RGB_REFINE_EMPTY_IMAGE'}

        bx0 = max(0, int(np.floor(min(x0, x1))))
        by0 = max(0, int(np.floor(min(y0, y1))))
        bx1 = min(image_w, int(np.ceil(max(x0, x1))) + 1)
        by1 = min(image_h, int(np.ceil(max(y0, y1))) + 1)
        if bx1 - bx0 < 12 or by1 - by0 < 12:
            return {'ok': False, 'reason': f'RGB_REFINE_BBOX_TOO_SMALL bbox={[bx0, by0, bx1, by1]}'}

        roi = image[by0:by1, bx0:bx1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY).astype(np.float32)
        blur = cv2.GaussianBlur(gray, (0, 0), 1.2)
        grad_x = cv2.Sobel(blur, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(blur, cv2.CV_32F, 0, 1, ksize=3)
        grad = cv2.magnitude(grad_x, grad_y)
        mean = cv2.GaussianBlur(gray, (0, 0), 4.0)
        mean_sq = cv2.GaussianBlur(gray * gray, (0, 0), 4.0)
        texture = np.sqrt(np.maximum(mean_sq - mean * mean, 0.0))

        def normalize_score(arr: np.ndarray) -> np.ndarray:
            high = float(np.percentile(arr, 98.0)) if arr.size else 0.0
            if high <= 1e-6:
                return np.zeros_like(arr, dtype=np.float32)
            return np.clip(arr / high, 0.0, 1.0).astype(np.float32)

        score = 0.65 * normalize_score(grad) + 0.35 * normalize_score(texture)
        score = cv2.GaussianBlur(score, (0, 0), 1.0)
        threshold = max(0.18, float(np.percentile(score, 72.0)))
        mask = (score >= threshold).astype(np.uint8)

        margin = max(3, int(round(min(bx1 - bx0, by1 - by0) * 0.06)))
        if mask.shape[0] > margin * 2 and mask.shape[1] > margin * 2:
            mask[:margin, :] = 0
            mask[-margin:, :] = 0
            mask[:, :margin] = 0
            mask[:, -margin:] = 0

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=1)
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if num_labels <= 1:
            return {'ok': False, 'reason': 'RGB_REFINE_NO_TEXTURE_COMPONENT'}

        hint_x = float(hint_u) - float(bx0)
        hint_y = float(hint_v) - float(by0)
        roi_diag = max(1.0, float(np.hypot(bx1 - bx0, by1 - by0)))
        min_area = max(24, int(round((bx1 - bx0) * (by1 - by0) * 0.01)))
        best_label = 0
        best_component_score = -1.0
        for label_id in range(1, num_labels):
            area = int(stats[label_id, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            component = labels == label_id
            ys, xs = np.nonzero(component)
            if len(xs) <= 0:
                continue
            centroid_x = float(xs.mean())
            centroid_y = float(ys.mean())
            hint_score = 1.0 - min(1.0, float(np.hypot(centroid_x - hint_x, centroid_y - hint_y)) / roi_diag)
            component_score = float(score[component].sum()) * (0.75 + 0.25 * hint_score)
            if component_score > best_component_score:
                best_component_score = component_score
                best_label = label_id

        if best_label <= 0:
            return {'ok': False, 'reason': 'RGB_REFINE_NO_COMPONENT_ABOVE_AREA'}

        component_u8 = (labels == best_label).astype(np.uint8)
        dist = cv2.distanceTransform(component_u8, cv2.DIST_L2, 3)
        if not np.isfinite(dist).any() or float(dist.max()) <= 0.0:
            return {'ok': False, 'reason': 'RGB_REFINE_DISTANCE_EMPTY'}
        combined = 0.72 * normalize_score(dist) + 0.28 * score
        _, max_val, _, max_loc = cv2.minMaxLoc(combined.astype(np.float32), mask=component_u8)
        refined_x = float(max_loc[0] + bx0)
        refined_y = float(max_loc[1] + by0)
        refined_norm = [
            _clamp(refined_x / float(max(1, image_w)), 0.0, 1.0),
            _clamp(refined_y / float(max(1, image_h)), 0.0, 1.0),
        ]
        risk_flags = _string_list(result.get('risk_flags'))
        if 'rgb_grasp_refine' not in risk_flags:
            risk_flags.append('rgb_grasp_refine')
        return {
            'ok': True,
            'refined_u': refined_x,
            'refined_v': refined_y,
            'refined_center_norm': refined_norm,
            'vlm_u': float(hint_u),
            'vlm_v': float(hint_v),
            'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
            'grasp_quality': float(max(0.35, min(0.95, max_val))),
            'grasp_refine_method': f'rgb_texture_inside_bbox_{shape or "unknown"}',
            'mask_area_px': int(component_u8.sum()),
            'edge_distance_px': float(dist[max_loc[1], max_loc[0]]),
            'local_depth_std_m': None,
            'local_depth_valid_ratio': 0.0,
            'refine_reason': 'RGB_TEXTURE_FALLBACK_AFTER_DEPTH_MASK',
            'gripper_yaw_deg': 0.0,
            'risk_flags': risk_flags,
        }

    def estimate_depth_from_roi(
        self,
        depth_m: np.ndarray,
        u: float,
        v: float,
        bbox_px: tuple[float, float, float, float],
    ) -> dict[str, Any]:
        if depth_m.ndim != 2:
            return {'ok': False, 'reason': f'DEPTH_NOT_2D shape={depth_m.shape}'}
        height, width = depth_m.shape
        if width <= 0 or height <= 0:
            return {'ok': False, 'reason': 'DEPTH_EMPTY'}

        x0, y0, x1, y1 = bbox_px
        radius = max(1, self.setting_int('depth_roi_radius_px', 12))
        u_i = int(round(u))
        v_i = int(round(v))
        roi_x0 = max(0, int(np.floor(x0)), u_i - radius)
        roi_y0 = max(0, int(np.floor(y0)), v_i - radius)
        roi_x1 = min(width, int(np.ceil(x1)) + 1, u_i + radius + 1)
        roi_y1 = min(height, int(np.ceil(y1)) + 1, v_i + radius + 1)
        roi_info = [int(roi_x0), int(roi_y0), int(roi_x1), int(roi_y1)]
        if roi_x1 <= roi_x0 or roi_y1 <= roi_y0:
            return {'ok': False, 'reason': f'DEPTH_ROI_EMPTY roi={roi_info}', 'roi': roi_info}

        roi = depth_m[roi_y0:roi_y1, roi_x0:roi_x1]
        total_count = int(roi.size)
        min_m = self.setting_float('depth_min_m', 0.12)
        max_m = self.setting_float('depth_max_m', 1.20)
        valid = roi[np.isfinite(roi) & (roi >= min_m) & (roi <= max_m)]
        valid_count = int(valid.size)
        valid_ratio = float(valid_count / total_count) if total_count else 0.0
        min_ratio = self.setting_float('depth_valid_ratio_min', 0.20)
        if valid_count <= 0:
            return {
                'ok': False,
                'reason': f'NO_VALID_DEPTH roi={roi_info}',
                'valid_ratio': valid_ratio,
                'valid_count': valid_count,
                'roi': roi_info,
            }
        if valid_ratio < min_ratio:
            return {
                'ok': False,
                'reason': f'DEPTH_VALID_RATIO_LOW ratio={valid_ratio:.3f} min={min_ratio:.3f}',
                'valid_ratio': valid_ratio,
                'valid_count': valid_count,
                'roi': roi_info,
            }
        percentile = _clamp(self.setting_float('depth_percentile', 25.0), 0.0, 100.0)
        depth_value = float(np.percentile(valid, percentile))
        return {
            'ok': True,
            'depth_m': depth_value,
            'valid_ratio': valid_ratio,
            'valid_count': valid_count,
            'roi': roi_info,
            'reason': f'OK percentile={percentile:.1f}',
        }

    def pixel_depth_to_camera_point(self, u: float, v: float, depth_m: float, camera_info: CameraInfo) -> tuple[float, float, float]:
        if len(camera_info.k) < 6:
            raise ValueError('camera_info.k is incomplete')
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError(f'invalid camera intrinsics fx={fx} fy={fy}')
        z = float(depth_m)
        x = (float(u) - cx) * z / fx
        y = (float(v) - cy) * z / fy
        return x, y, z

    def average_camera_point_if_ready(
        self,
        camera_point: list[float] | tuple[float, float, float],
        frame_id: str,
    ) -> tuple[bool, list[float], dict[str, Any]]:
        if not self.setting_bool('camera_point_average_enabled', True):
            point = [float(v) for v in camera_point]
            return True, point, {
                'enabled': False,
                'ready': True,
                'samples': 1,
                'required_samples': 1,
                'reason': 'CAMERA_POINT_AVERAGE_DISABLED',
            }

        required = max(1, self.setting_int('camera_point_average_samples', 2))
        window_sec = max(0.2, self.setting_float('camera_point_average_window_sec', 8.0))
        max_delta_m = max(0.001, self.setting_float('camera_point_average_max_delta_mm', 100.0) / 1000.0)
        frame = str(frame_id or self.setting_str('depth_frame_id', 'camera_color_optical_frame'))
        now = time.time()
        point = np.array(camera_point, dtype=np.float64)
        reset_reason = ''

        with self.state_lock:
            history = [
                (stamp, item.copy(), item_frame)
                for stamp, item, item_frame in self.camera_point_history
                if now - stamp <= window_sec and item_frame == frame
            ]
            if history:
                jump = float(np.linalg.norm(point - history[-1][1]))
                if jump > max_delta_m:
                    reset_reason = f'CAMERA_POINT_AVERAGE_RESET jump_mm={jump*1000.0:.1f} limit_mm={max_delta_m*1000.0:.1f}'
                    history = []
            history.append((now, point.copy(), frame))
            max_keep = max(required * 3, required, 2)
            history = history[-max_keep:]
            self.camera_point_history = history
            selected = np.array([item for _, item, _ in history[-required:]], dtype=np.float64)
            sample_count = int(selected.shape[0])
            ready = sample_count >= required
            averaged = np.mean(selected, axis=0) if ready else point.copy()

        reason = reset_reason or ('CAMERA_POINT_AVERAGED' if ready else 'CAMERA_POINT_AVERAGE_WAIT')
        if now - self.last_camera_average_status_time > 0.5:
            self.last_camera_average_status_time = now
            avg_mm = averaged * 1000.0
            raw_mm = point * 1000.0
            self.status_pub.publish(
                String(
                    data=(
                        f'{reason} samples={sample_count}/{required} '
                        f'raw_mm={raw_mm[0]:.1f},{raw_mm[1]:.1f},{raw_mm[2]:.1f} '
                        f'avg_mm={avg_mm[0]:.1f},{avg_mm[1]:.1f},{avg_mm[2]:.1f}'
                    )
                )
            )

        return ready, [float(v) for v in averaged], {
            'enabled': True,
            'ready': bool(ready),
            'samples': sample_count,
            'required_samples': required,
            'window_sec': window_sec,
            'max_delta_mm': max_delta_m * 1000.0,
            'reason': reason,
            'raw_camera_point_m': [float(v) for v in point],
        }

    def load_handeye_for_plane_fallback(self) -> tuple[np.ndarray, np.ndarray]:
        path = Path(self.setting_str('plane_fallback_handeye_file', '/home/sunrise/trash_robot_v3/config/grasp/handeye_point.yaml'))
        if not path.exists():
            raise FileNotFoundError(f'handeye file not found: {path}')
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        if 'matrix_row_major' in data:
            mat = np.array(data['matrix_row_major'], dtype=np.float64).reshape(4, 4)
            return mat[:3, :3], mat[:3, 3]
        if 'rotation_matrix' in data and 'translation_m' in data:
            rot = np.array(data['rotation_matrix'], dtype=np.float64).reshape(3, 3)
            trans = np.array(data['translation_m'], dtype=np.float64)
            return rot, trans
        raise ValueError(f'invalid handeye file for plane fallback: {path}')

    def estimate_camera_point_from_arm_plane(
        self,
        u: float,
        v: float,
        camera_info: CameraInfo,
    ) -> dict[str, Any]:
        if not self.setting_bool('plane_fallback_enabled', False):
            return {'ok': False, 'reason': 'PLANE_FALLBACK_DISABLED'}
        if len(camera_info.k) < 6:
            return {'ok': False, 'reason': 'PLANE_FALLBACK_BAD_CAMERA_INFO'}
        try:
            rot, trans = self.load_handeye_for_plane_fallback()
            fx = float(camera_info.k[0])
            fy = float(camera_info.k[4])
            cx = float(camera_info.k[2])
            cy = float(camera_info.k[5])
            if fx <= 0.0 or fy <= 0.0:
                return {'ok': False, 'reason': f'PLANE_FALLBACK_BAD_INTRINSICS fx={fx} fy={fy}'}
            ray = np.array([(float(u) - cx) / fx, (float(v) - cy) / fy, 1.0], dtype=np.float64)
            plane_z = self.setting_float('plane_fallback_arm_z_m', -0.265)
            denom = float(rot[2, :] @ ray)
            if abs(denom) < 1e-6:
                return {'ok': False, 'reason': 'PLANE_FALLBACK_RAY_PARALLEL'}
            camera_z = float((plane_z - trans[2]) / denom)
            min_z = self.setting_float('plane_fallback_min_camera_z_m', 0.05)
            max_z = self.setting_float('plane_fallback_max_camera_z_m', 1.0)
            if camera_z < min_z or camera_z > max_z:
                return {
                    'ok': False,
                    'reason': f'PLANE_FALLBACK_Z_OUT_OF_RANGE z={camera_z:.3f} range={min_z:.3f}-{max_z:.3f}',
                }
            camera_point = ray * camera_z
            arm_point = rot @ camera_point + trans
            return {
                'ok': True,
                'depth_m': camera_z,
                'camera_point_m': [float(camera_point[0]), float(camera_point[1]), float(camera_point[2])],
                'plane_arm_point_m': [float(arm_point[0]), float(arm_point[1]), float(arm_point[2])],
                'plane_arm_z_m': float(plane_z),
                'reason': f'PLANE_FALLBACK arm_z={plane_z:.3f}',
            }
        except Exception as exc:  # noqa: BLE001
            return {'ok': False, 'reason': f'PLANE_FALLBACK_ERROR {exc}'}

    def estimate_current_camera_point(
        self,
        u: float,
        v: float,
        bbox_px: tuple[float, float, float, float],
        color_w: int,
        color_h: int,
        prefer_plane_fallback: bool = False,
    ) -> dict[str, Any]:
        snapshot = self.latest_depth_snapshot()
        if not bool(snapshot.get('ok', False)):
            return snapshot
        try:
            depth_m = snapshot['depth_m']
            depth_msg = snapshot['depth_msg']
            camera_info = snapshot['camera_info']
            depth_h, depth_w = depth_m.shape[:2]

            configured_depth_frame_id = self.setting_str('depth_frame_id', 'camera_color_optical_frame')
            depth_frame_id = str(getattr(depth_msg.header, 'frame_id', '') or '').strip()
            camera_info_frame_id = str(getattr(camera_info.header, 'frame_id', '') or '').strip()
            camera_frame = depth_frame_id or configured_depth_frame_id
            debug_meta = {
                'depth_age_sec': snapshot.get('depth_age_sec'),
                'depth_stamp': depth_msg.header.stamp,
                'depth_size': [int(depth_w), int(depth_h)],
                'color_size': [int(color_w), int(color_h)],
                'depth_width': int(depth_w),
                'depth_height': int(depth_h),
                'color_width': int(color_w),
                'color_height': int(color_h),
                'depth_frame_id': depth_frame_id,
                'camera_info_frame_id': camera_info_frame_id,
                'configured_depth_frame_id': configured_depth_frame_id,
                'camera_frame': camera_frame,
            }

            if depth_frame_id and camera_info_frame_id and depth_frame_id != camera_info_frame_id:
                return {
                    **debug_meta,
                    'ok': False,
                    'reason': (
                        'DEPTH_CAMERA_INFO_FRAME_MISMATCH '
                        f'depth_frame={depth_frame_id} camera_info_frame={camera_info_frame_id}'
                    ),
                }

            if self.setting_bool('strict_depth_size_check', True) and (int(depth_w) != int(color_w) or int(depth_h) != int(color_h)):
                return {
                    **debug_meta,
                    'ok': False,
                    'reason': f'DEPTH_COLOR_SIZE_MISMATCH color={int(color_w)}x{int(color_h)} depth={int(depth_w)}x{int(depth_h)}',
                }

            depth_u, depth_v, depth_bbox_px = map_color_pixel_to_depth_pixel(
                u,
                v,
                bbox_px,
                int(color_w),
                int(color_h),
                int(depth_w),
                int(depth_h),
            )
            estimate = self.estimate_depth_from_roi(depth_m, depth_u, depth_v, depth_bbox_px)
            plane = self.estimate_camera_point_from_arm_plane(u, v, camera_info)
            if prefer_plane_fallback and bool(plane.get('ok', False)):
                return {
                    **debug_meta,
                    **estimate,
                    'ok': True,
                    'depth_m': float(plane['depth_m']),
                    'valid_ratio': float(estimate.get('valid_ratio', 0.0) or 0.0),
                    'valid_count': int(estimate.get('valid_count', 0) or 0),
                    'camera_point_m': plane['camera_point_m'],
                    'camera_frame': camera_frame,
                    'depth_method': 'handeye_arm_plane_fallback_floor_contact',
                    'depth_pixel': [float(depth_u), float(depth_v)],
                    'plane_arm_point_m': plane.get('plane_arm_point_m'),
                    'plane_arm_z_m': plane.get('plane_arm_z_m'),
                    'reason': f'{plane.get("reason")} floor_contact',
                }
            if not bool(estimate.get('ok', False)):
                plane = self.estimate_camera_point_from_arm_plane(u, v, camera_info)
                if bool(plane.get('ok', False)):
                    return {
                        **debug_meta,
                        **estimate,
                        'ok': True,
                        'depth_m': float(plane['depth_m']),
                        'valid_ratio': float(estimate.get('valid_ratio', 0.0) or 0.0),
                        'valid_count': int(estimate.get('valid_count', 0) or 0),
                        'camera_point_m': plane['camera_point_m'],
                        'camera_frame': camera_frame,
                        'depth_method': 'handeye_arm_plane_fallback',
                        'depth_pixel': [float(depth_u), float(depth_v)],
                        'plane_arm_point_m': plane.get('plane_arm_point_m'),
                        'plane_arm_z_m': plane.get('plane_arm_z_m'),
                        'reason': plane.get('reason'),
                    }
                estimate.update({k: v for k, v in debug_meta.items() if k not in estimate})
                estimate.setdefault('depth_pixel', [float(depth_u), float(depth_v)])
                return estimate
            point = self.pixel_depth_to_camera_point(u, v, float(estimate['depth_m']), camera_info)
            plane = self.estimate_camera_point_from_arm_plane(u, v, camera_info)
            if bool(plane.get('ok', False)):
                disagreement = abs(float(estimate['depth_m']) - float(plane['depth_m']))
                if disagreement > self.setting_float('plane_fallback_depth_disagreement_m', 0.20):
                    return {
                        **debug_meta,
                        **estimate,
                        'ok': True,
                        'depth_m': float(plane['depth_m']),
                        'camera_point_m': plane['camera_point_m'],
                        'camera_frame': camera_frame,
                        'depth_method': 'handeye_arm_plane_fallback_depth_disagreement',
                        'depth_pixel': [float(depth_u), float(depth_v)],
                        'plane_arm_point_m': plane.get('plane_arm_point_m'),
                        'plane_arm_z_m': plane.get('plane_arm_z_m'),
                        'reason': f'{plane.get("reason")} depth_disagreement={disagreement:.3f}',
                    }
            return {
                **debug_meta,
                **estimate,
                'camera_point_m': [float(point[0]), float(point[1]), float(point[2])],
                'camera_frame': camera_frame,
                'depth_method': 'aligned_depth_roi_percentile',
                'depth_pixel': [float(depth_u), float(depth_v)],
            }
        except Exception as exc:  # noqa: BLE001 - publish clear failure status instead of crashing
            return {'ok': False, 'reason': f'DEPTH_ESTIMATE_ERROR {exc}', 'depth_age_sec': snapshot.get('depth_age_sec')}

    def call_provider(
        self,
        provider: VlmProvider,
        model: str,
        data_url: str,
        image_w: Optional[int] = None,
        image_h: Optional[int] = None,
    ) -> tuple[str, float]:
        if provider.name == 'local_hobot':
            result, latency_ms = self.local_hobot_result(int(image_w or 640), int(image_h or 480))
            return json.dumps(result, ensure_ascii=False), latency_ms
        api_key = os.environ.get(provider.api_key_env, '').strip()
        if not api_key:
            raise RuntimeError(f'missing API key env {provider.api_key_env}')
        if not provider.base_url or not model:
            raise RuntimeError(f'invalid provider config: {provider.name}')
        if provider.name == 'dashscope' and self.is_dashscope_realtime_model(model):
            return self.call_dashscope_realtime(provider, model, data_url)

        content = [
            {'type': 'text', 'text': PROMPT},
            {'type': 'image_url', 'image_url': {'url': data_url}},
        ]
        if provider.name == 'mimo':
            content = [
                {'type': 'image_url', 'image_url': {'url': data_url}},
                {'type': 'text', 'text': PROMPT},
            ]

        payload = {
            'model': model,
            'messages': [{'role': 'user', 'content': content}],
            'temperature': 0.0,
        }
        if provider.name == 'dashscope' and model.startswith('qwen3'):
            payload['enable_thinking'] = False
        if provider.name == 'mimo':
            payload['max_completion_tokens'] = 512
            payload['thinking'] = {'type': 'disabled'}
        url = provider.base_url + '/chat/completions'
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        req = urllib.request.Request(
            url,
            data=body,
            headers={
                'Authorization': f'Bearer {api_key}',
                'Content-Type': 'application/json',
            },
            method='POST',
        )
        start = time.time()
        try:
            with urllib.request.urlopen(req, timeout=self.setting_float('api_timeout_sec', 8.0)) as resp:
                response = json.loads(resp.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = ''
            try:
                detail = exc.read().decode('utf-8', errors='ignore')
            except Exception:
                detail = ''
            detail = re.sub(r'\s+', ' ', detail).strip()
            if detail:
                raise RuntimeError(f'HTTP {exc.code} {exc.reason}: {detail[:260]}') from exc
            raise RuntimeError(f'HTTP {exc.code} {exc.reason}') from exc
        latency_ms = (time.time() - start) * 1000.0
        content = response['choices'][0]['message']['content']
        if isinstance(content, list):
            content = ''.join(str(item.get('text', '')) if isinstance(item, dict) else str(item) for item in content)
        return str(content), latency_ms

    @staticmethod
    def is_dashscope_realtime_model(model: str) -> bool:
        text = str(model or '').strip().lower()
        return text.endswith('-realtime') or '-realtime-' in text

    def dashscope_realtime_url(self, provider: VlmProvider) -> str:
        if provider.base_url.startswith('wss://'):
            return provider.base_url.rstrip('/')
        if 'dashscope-intl.aliyuncs.com' in provider.base_url:
            return 'wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime'
        return 'wss://dashscope.aliyuncs.com/api-ws/v1/realtime'

    def call_dashscope_realtime(
        self,
        provider: VlmProvider,
        model: str,
        data_url: str,
    ) -> tuple[str, float]:
        try:
            import websockets
        except ImportError as exc:
            raise RuntimeError('dashscope realtime requires python package websockets') from exc

        api_key = os.environ.get(provider.api_key_env, '').strip()
        if not api_key:
            raise RuntimeError(f'missing API key env {provider.api_key_env}')
        if ',' not in data_url:
            raise RuntimeError('invalid image data URL for dashscope realtime')
        image_b64 = data_url.split(',', 1)[1]
        try:
            image_bytes = base64.b64decode(image_b64)
        except Exception as exc:
            raise RuntimeError(f'invalid base64 image for dashscope realtime: {exc}') from exc

        timeout_sec = max(30.0, self.setting_float('api_timeout_sec', 8.0) * 4.0)
        start = time.time()
        text = asyncio.run(
            self.call_dashscope_realtime_async(
                websockets,
                api_key,
                self.dashscope_realtime_url(provider),
                model,
                image_bytes,
                timeout_sec,
            )
        )
        return text, (time.time() - start) * 1000.0

    async def call_dashscope_realtime_async(
        self,
        websockets_module: Any,
        api_key: str,
        base_url: str,
        model: str,
        image_bytes: bytes,
        timeout_sec: float,
    ) -> str:
        url = f'{base_url}?model={model}'

        async def send_event(ws: Any, event: dict[str, Any]) -> None:
            event['event_id'] = f'event_{int(time.time() * 1000)}'
            await ws.send(json.dumps(event, ensure_ascii=False))

        text_parts: list[str] = []
        done_text = ''
        async with websockets_module.connect(
            url,
            additional_headers={'Authorization': f'Bearer {api_key}'},
            ping_interval=None,
            open_timeout=min(15.0, timeout_sec),
        ) as ws:
            await send_event(
                ws,
                {
                    'type': 'session.update',
                    'session': {
                        'modalities': ['text'],
                        'instructions': PROMPT,
                        'input_audio_format': 'pcm',
                        'output_audio_format': 'pcm',
                        'turn_detection': None,
                    },
                },
            )
            # Qwen-Omni-Realtime requires at least one audio append before image input.
            silence_pcm_16k_mono = b'\x00\x00' * 16000
            await send_event(
                ws,
                {
                    'type': 'input_audio_buffer.append',
                    'audio': base64.b64encode(silence_pcm_16k_mono).decode('ascii'),
                },
            )
            await send_event(
                ws,
                {
                    'type': 'input_image_buffer.append',
                    'image': base64.b64encode(image_bytes).decode('ascii'),
                },
            )
            await send_event(ws, {'type': 'input_audio_buffer.commit'})
            await send_event(ws, {'type': 'response.create'})

            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                try:
                    message = await asyncio.wait_for(ws.recv(), timeout=min(8.0, max(0.2, deadline - time.time())))
                except asyncio.TimeoutError:
                    continue
                event = json.loads(message)
                event_type = str(event.get('type') or '')
                if event_type == 'error':
                    raise RuntimeError(f'dashscope realtime error: {event.get("error", event)}')
                if event_type in ('response.text.delta', 'response.audio_transcript.delta'):
                    text_parts.append(str(event.get('delta') or ''))
                elif event_type in ('response.text.done', 'response.audio_transcript.done'):
                    done_text = str(event.get('text') or event.get('transcript') or done_text)
                elif event_type == 'response.done':
                    result = done_text or ''.join(text_parts)
                    if not result.strip():
                        raise RuntimeError('dashscope realtime returned empty text')
                    return result
            result = done_text or ''.join(text_parts)
            if result.strip():
                return result
            raise RuntimeError('dashscope realtime timeout waiting for response')

    def worker(self, msg: Image) -> None:
        try:
            if self.shutdown_event.is_set():
                return
            data_url, image_w, image_h = self.encode_image(msg)
            errors = []
            for name in self.provider_names():
                if self.shutdown_event.is_set():
                    return
                provider = self.get_provider(name)
                if provider is None:
                    errors.append(f'{name}:disabled')
                    continue
                for model in (provider.model_candidates or [provider.model]):
                    if self.shutdown_event.is_set():
                        return
                    try:
                        text, latency_ms = self.call_provider(
                            provider,
                            model,
                            data_url,
                            image_w=image_w,
                            image_h=image_h,
                        )
                        if self.shutdown_event.is_set():
                            return
                        raw = extract_json_object(text)
                        min_confidence = self.setting_float('min_confidence', 0.55)
                        if provider.name == 'local_hobot':
                            min_confidence = self.setting_float('local_hobot_min_confidence', min_confidence)
                        ok, reason, cleaned = validate_vlm_result(
                            raw,
                            min_confidence,
                        )
                        if ok:
                            cleaned = apply_geometric_grasp_rules(
                                cleaned,
                                self.config.get('grasp_point_overrides', {}),
                            )
                        cleaned.update({
                            'provider': provider.name,
                            'model': model,
                            'latency_ms': round(latency_ms, 1),
                            'image_width': image_w,
                            'image_height': image_h,
                            'stamp': time.time(),
                        })
                        signature = None
                        if ok:
                            try:
                                signature = self.image_signature(msg)
                            except Exception as exc:  # noqa: BLE001
                                self.publish_status_throttled(f'VLM_CACHE_SIGNATURE_FAILED {exc}', period=5.0)
                        with self.state_lock:
                            self.consecutive_failures = 0
                            self.backoff_until = 0.0
                            self.provider_health[provider.name] = {
                                'ok': True,
                                'model': model,
                                'latency_ms': round(latency_ms, 1),
                                'stamp': time.time(),
                            }
                            if ok:
                                self.cached_result = copy.deepcopy(cleaned)
                                self.cached_stamp = time.time()
                                self.cached_signature = signature
                                self.last_cache_publish_time = 0.0
                                self.last_cache_motion_check_time = 0.0
                            else:
                                self.cached_result = None
                                self.cached_stamp = 0.0
                                self.cached_signature = None
                                self.last_cache_publish_time = 0.0
                                self.last_cache_motion_check_time = 0.0
                        if not ok:
                            self.clear_cache(reason)
                        self.publish_vlm(cleaned, ok, reason, cached=False)
                        return
                    except Exception as exc:  # noqa: BLE001 - provider fallback must be robust
                        errors.append(f'{name}/{model}:{exc}')
                        with self.state_lock:
                            self.provider_health[name] = {
                                'ok': False,
                                'model': model,
                                'error': str(exc)[-200:],
                                'stamp': time.time(),
                            }
            with self.state_lock:
                self.consecutive_failures += 1
                failures = self.consecutive_failures
            max_failures = max(1, self.setting_int('max_consecutive_failures', 5))
            if failures >= max_failures:
                initial = self.setting_float('backoff_initial_sec', 2.0)
                max_backoff = self.setting_float('backoff_max_sec', 60.0)
                backoff = min(max_backoff, initial * (2 ** min(5, failures - max_failures)))
                with self.state_lock:
                    self.backoff_until = time.time() + backoff
            self.publish_failed('NO_VLM_RESULT ' + '; '.join(errors))
        finally:
            with self.state_lock:
                self.inflight = False

    def publish_failed(self, status: str) -> None:
        with self.state_lock:
            provider_health = copy.deepcopy(self.provider_health)
            consecutive_failures = self.consecutive_failures
            backoff_remaining = max(0.0, round(self.backoff_until - time.time(), 2))
        result = {
            'has_target': False,
            'trash_label': '',
            'object_name': '',
            'bbox_norm': [0.0, 0.0, 0.0, 0.0],
            'center_norm': [0.0, 0.0],
            'grasp_point_norm': [0.0, 0.0],
            'object_shape': 'unknown',
            'grasp_type': 'unknown',
            'grasp_strategy': '',
            'grasp_width_hint': 'unknown',
            'major_axis_angle_deg': 0.0,
            'risk_flags': [],
            'grasp_hint': '',
            'confidence': 0.0,
            'reason': status,
            'provider_health': provider_health,
            'consecutive_failures': consecutive_failures,
            'backoff_remaining_sec': backoff_remaining,
            'stamp': time.time(),
        }
        self.vlm_result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self.grasp_plan_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self.status_pub.publish(String(data=status[:900]))

    def publish_debug_image(
        self,
        bbox_px: tuple[float, float, float, float],
        vlm_point_px: tuple[float, float],
        refined_point_px: tuple[float, float],
        depth_fields: dict[str, Any],
        refine_result: dict[str, Any],
        publish_result: dict[str, Any],
    ) -> None:
        with self.lock:
            image_msg = self.latest_image
        if image_msg is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.publish_status_throttled(f'VLM_DEBUG_IMAGE_CONVERT_FAILED {exc}', period=5.0)
            return

        image_h, image_w = image.shape[:2]
        x0, y0, x1, y1 = [int(round(v)) for v in bbox_px]
        x0 = max(0, min(image_w - 1, x0))
        x1 = max(0, min(image_w - 1, x1))
        y0 = max(0, min(image_h - 1, y0))
        y1 = max(0, min(image_h - 1, y1))
        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 0), 2)

        object_mask = refine_result.get('object_mask')
        if isinstance(object_mask, np.ndarray) and object_mask.ndim == 2 and object_mask.any():
            depth_h, depth_w = object_mask.shape[:2]
            mask_u8 = object_mask.astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            scale_x = float(image_w) / float(max(1, depth_w))
            scale_y = float(image_h) / float(max(1, depth_h))
            for contour in contours:
                if contour.size == 0:
                    continue
                contour = contour.astype(np.float32)
                contour[:, :, 0] *= scale_x
                contour[:, :, 1] *= scale_y
                cv2.drawContours(image, [contour.astype(np.int32)], -1, (255, 180, 0), 1)

        vu, vv = int(round(vlm_point_px[0])), int(round(vlm_point_px[1]))
        ru, rv = int(round(refined_point_px[0])), int(round(refined_point_px[1]))
        cv2.circle(image, (vu, vv), 6, (0, 255, 255), -1)
        cv2.circle(image, (ru, rv), 6, (0, 255, 0), -1)
        cv2.line(image, (vu, vv), (ru, rv), (0, 200, 255), 1)

        roi = depth_fields.get('depth_roi') or refine_result.get('local_depth_roi') or []
        depth_size = depth_fields.get('depth_size') or refine_result.get('depth_size') or [image_w, image_h]
        if isinstance(roi, list) and len(roi) == 4:
            scale_x = float(image_w) / float(max(1, int(depth_size[0])))
            scale_y = float(image_h) / float(max(1, int(depth_size[1])))
            rx0, ry0, rx1, ry1 = [int(round(float(v))) for v in roi]
            cv2.rectangle(
                image,
                (int(rx0 * scale_x), int(ry0 * scale_y)),
                (int(rx1 * scale_x), int(ry1 * scale_y)),
                (255, 255, 0),
                1,
            )

        label = str(publish_result.get('object_name') or publish_result.get('trash_label') or '')
        text_lines = [
            f'{label[:24]} conf={float(publish_result.get("confidence", 0.0) or 0.0):.2f}',
            f'quality={float(publish_result.get("grasp_quality", 0.0) or 0.0):.2f} depth={depth_fields.get("depth_m")}',
            f'depth_ok={bool(depth_fields.get("depth_ok", False))} risks={",".join(publish_result.get("risk_flags", [])[:3])}',
        ]
        y_text = 24
        for line in text_lines:
            cv2.putText(image, line, (8, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, line, (8, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y_text += 22

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            debug_msg.header = image_msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as exc:  # noqa: BLE001
            self.publish_status_throttled(f'VLM_DEBUG_IMAGE_PUBLISH_FAILED {exc}', period=5.0)

    def publish_vlm(self, result: dict[str, Any], accepted: bool, reason: str, cached: bool = False) -> None:
        with self.state_lock:
            cached_stamp = self.cached_stamp
        publish_result = dict(result)
        publish_result['cached'] = bool(cached)
        publish_result['cache_age_sec'] = round(time.time() - cached_stamp, 2) if cached and cached_stamp else 0.0
        publish_result['api_call_skipped'] = bool(cached)
        publish_result['stamp'] = time.time()
        if not accepted:
            self.vlm_result_pub.publish(String(data=json.dumps(publish_result, ensure_ascii=False)))
            self.grasp_plan_pub.publish(String(data=json.dumps(publish_result, ensure_ascii=False)))
            self.status_pub.publish(String(data=f'VLM_REJECT {reason}'))
            return

        bbox = publish_result['bbox_norm']
        vlm_center = publish_result.get('vlm_grasp_point_norm') or publish_result.get('grasp_point_norm') or publish_result['center_norm']
        center = publish_result['center_norm']
        image_w = int(publish_result.get('image_width') or 640)
        image_h = int(publish_result.get('image_height') or 480)
        x0, y0, x1, y1, vlm_u, vlm_v = normalized_bbox_to_pixels(bbox, vlm_center, image_w, image_h)

        refine_enabled = self.setting_bool('grasp_refine_enabled', True)
        fallback_to_vlm = self.setting_bool('fallback_to_vlm_grasp', False)
        risk_flags = _string_list(publish_result.get('risk_flags'))
        gripper_yaw_deg = 0.0
        if str(publish_result.get('grasp_strategy') or '').lower() in ('slender_midbody', 'cylindrical_midbody'):
            gripper_yaw_deg = _axis_angle(publish_result.get('major_axis_angle_deg')) + 90.0
        refine_result: dict[str, Any] = {
            'ok': False,
            'reason': 'REFINE_DISABLED',
            'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
            'vlm_u': vlm_u,
            'vlm_v': vlm_v,
        }
        if refine_enabled:
            snapshot = self.latest_depth_snapshot()
            if bool(snapshot.get('ok', False)):
                refine_result = self.refine_grasp_point_by_local_geometry(
                    publish_result,
                    snapshot['depth_m'],
                    image_w,
                    image_h,
                )
            else:
                refine_result = {
                    'ok': False,
                    'reason': str(snapshot.get('reason') or 'WAIT_DEPTH_FOR_REFINE'),
                    'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
                    'vlm_u': vlm_u,
                    'vlm_v': vlm_v,
                }
            if not bool(refine_result.get('ok', False)):
                depth_refine_reason = str(refine_result.get('reason') or 'DEPTH_REFINE_FAILED')
                rgb_refine_result = self.refine_grasp_point_by_rgb_texture(
                    publish_result,
                    image_w,
                    image_h,
                )
                if bool(rgb_refine_result.get('ok', False)):
                    rgb_refine_result['depth_refine_reason'] = depth_refine_reason
                    refine_result = rgb_refine_result

        if bool(refine_result.get('ok', False)):
            center = list(refine_result['refined_center_norm'])
            publish_result['center_norm'] = center
            publish_result['grasp_point_norm'] = center
            risk_flags = _string_list(refine_result.get('risk_flags', risk_flags))
            gripper_yaw_deg = float(refine_result.get('gripper_yaw_deg', gripper_yaw_deg) or 0.0)
        elif refine_enabled:
            if fallback_to_vlm or self.setting_bool('plane_fallback_enabled', False):
                if 'grasp_refine_failed' not in risk_flags:
                    risk_flags.append('grasp_refine_failed')
                if self.setting_bool('plane_fallback_enabled', False) and 'plane_depth_fallback' not in risk_flags:
                    risk_flags.append('plane_depth_fallback')
                center = [float(vlm_center[0]), float(vlm_center[1])]
                publish_result['center_norm'] = center
                publish_result['grasp_point_norm'] = center
            else:
                reject_fields = {
                    'depth_ok': False,
                    'depth_m': None,
                    'depth_reason': f'GRASP_REFINE_FAILED {refine_result.get("reason")}',
                    'depth_valid_ratio': 0.0,
                    'depth_valid_count': 0,
                    'depth_roi': [],
                    'camera_point_m': None,
                    'camera_frame': self.setting_str('depth_frame_id', 'camera_color_optical_frame'),
                    'depth_method': 'grasp_refine_rejected',
                    'depth_age_sec': None,
                }
                refine_fields = {
                    'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
                    'refined_grasp_point_norm': None,
                    'grasp_refine_enabled': True,
                    'grasp_refine_method': 'depth_mask_distance_transform',
                    'grasp_quality': 0.0,
                    'mask_area_px': int(refine_result.get('mask_area_px', 0) or 0),
                    'edge_distance_px': refine_result.get('edge_distance_px'),
                    'local_depth_std_m': refine_result.get('local_depth_std_m'),
                    'local_depth_valid_ratio': float(refine_result.get('local_depth_valid_ratio', 0.0) or 0.0),
                    'refine_reason': str(refine_result.get('reason') or 'REFINE_FAILED'),
                    'gripper_yaw_deg': float(gripper_yaw_deg),
                    'risk_flags': risk_flags + ['grasp_refine_failed'],
                    'graspable': False,
                }
                publish_result.update(reject_fields)
                publish_result.update(refine_fields)
                publish_result['has_target'] = False
                self.vlm_result_pub.publish(String(data=json.dumps(publish_result, ensure_ascii=False)))
                self.grasp_plan_pub.publish(String(data=json.dumps(publish_result, ensure_ascii=False)))
                bbox_payload = {
                    'stamp': time.time(),
                    'image_width': image_w,
                    'image_height': image_h,
                    'u0': x0,
                    'v0': y0,
                    'u1': x1,
                    'v1': y1,
                    'center_u': vlm_u,
                    'center_v': vlm_v,
                    'grasp_u': vlm_u,
                    'grasp_v': vlm_v,
                    'confidence': float(publish_result.get('confidence', 0.0) or 0.0),
                    'label': str(publish_result.get('trash_label') or ''),
                    'raw_label': str(publish_result.get('object_name') or ''),
                    **reject_fields,
                    **refine_fields,
                }
                self.bbox_pub.publish(String(data=json.dumps(bbox_payload, ensure_ascii=False)))
                self.depth_status_pub.publish(String(data=reject_fields['depth_reason']))
                self.publish_debug_image(
                    (x0, y0, x1, y1),
                    (vlm_u, vlm_v),
                    (vlm_u, vlm_v),
                    reject_fields,
                    refine_result,
                    publish_result,
                )
                self.status_pub.publish(String(data=f'GRASP_REFINE_REJECT {refine_fields["refine_reason"]}'))
                return

        x0, y0, x1, y1, u, v = normalized_bbox_to_pixels(bbox, center, image_w, image_h)

        prefer_plane_fallback = (
            self.setting_bool('floor_contact_use_plane_fallback', True)
            and 'floor_contact' in risk_flags
        )
        depth_result = self.estimate_current_camera_point(
            u,
            v,
            (x0, y0, x1, y1),
            image_w,
            image_h,
            prefer_plane_fallback=prefer_plane_fallback,
        )
        depth_ok = bool(depth_result.get('ok', False))
        camera_point_ready = False
        averaged_camera_point = None
        camera_average_fields: dict[str, Any] = {
            'camera_point_average_enabled': self.setting_bool('camera_point_average_enabled', True),
            'camera_point_average_ready': False,
            'camera_point_average_samples': 0,
            'camera_point_average_required_samples': max(1, self.setting_int('camera_point_average_samples', 2)),
            'camera_point_average_reason': 'NO_DEPTH_POINT',
        }
        if depth_ok and depth_result.get('camera_point_m') is not None:
            camera_point_ready, averaged_camera_point, average_meta = self.average_camera_point_if_ready(
                depth_result['camera_point_m'],
                str(depth_result.get('camera_frame') or self.setting_str('depth_frame_id', 'camera_color_optical_frame')),
            )
            camera_average_fields.update(
                {
                    'camera_point_average_enabled': bool(average_meta.get('enabled', True)),
                    'camera_point_average_ready': bool(average_meta.get('ready', camera_point_ready)),
                    'camera_point_average_samples': int(average_meta.get('samples', 0) or 0),
                    'camera_point_average_required_samples': int(average_meta.get('required_samples', 1) or 1),
                    'camera_point_average_window_sec': average_meta.get('window_sec'),
                    'camera_point_average_max_delta_mm': average_meta.get('max_delta_mm'),
                    'camera_point_average_reason': str(average_meta.get('reason') or ''),
                    'camera_point_raw_m': average_meta.get('raw_camera_point_m'),
                    'camera_point_average_lock_after_ready': self.setting_bool('camera_point_average_lock_after_ready', True),
                }
            )
            if camera_point_ready and self.setting_bool('camera_point_average_lock_after_ready', True):
                with self.state_lock:
                    self.camera_point_average_locked = True
        plane_depth_ok = str(depth_result.get('depth_method') or '').startswith('handeye_arm_plane_fallback')
        refine_fields = {
            'vlm_grasp_point_norm': [float(vlm_center[0]), float(vlm_center[1])],
            'refined_grasp_point_norm': [float(center[0]), float(center[1])],
            'grasp_refine_enabled': bool(refine_enabled),
            'grasp_refine_method': str(refine_result.get('grasp_refine_method') or ('vlm_fallback' if fallback_to_vlm else 'disabled')),
            'grasp_quality': float(refine_result.get('grasp_quality', 0.35 if refine_enabled and fallback_to_vlm else 0.0) or 0.0),
            'mask_area_px': int(refine_result.get('mask_area_px', 0) or 0),
            'edge_distance_px': refine_result.get('edge_distance_px'),
            'local_depth_std_m': refine_result.get('local_depth_std_m'),
            'local_depth_valid_ratio': float(refine_result.get('local_depth_valid_ratio', 0.0) or 0.0),
            'refine_reason': str(refine_result.get('refine_reason') or refine_result.get('reason') or ''),
            'gripper_yaw_deg': float(gripper_yaw_deg),
            'risk_flags': risk_flags,
            'graspable': bool(
                depth_ok
                and camera_point_ready
                and (
                    not refine_enabled
                    or bool(refine_result.get('ok', False))
                    or fallback_to_vlm
                    or plane_depth_ok
                )
            ),
        }
        depth_fields = {
            'depth_ok': depth_ok,
            'depth_m': float(depth_result['depth_m']) if depth_ok else None,
            'depth_reason': str(depth_result.get('reason') or ''),
            'depth_valid_ratio': float(depth_result.get('valid_ratio', 0.0) or 0.0),
            'depth_valid_count': int(depth_result.get('valid_count', 0) or 0),
            'depth_roi': depth_result.get('roi', []),
            'camera_point_m': averaged_camera_point if (depth_ok and camera_point_ready) else None,
            'camera_frame': str(depth_result.get('camera_frame') or self.setting_str('depth_frame_id', 'camera_color_optical_frame')),
            'depth_method': str(depth_result.get('depth_method') or 'aligned_depth_roi_percentile'),
            'depth_age_sec': depth_result.get('depth_age_sec'),
            'depth_pixel': depth_result.get('depth_pixel'),
            'depth_size': depth_result.get('depth_size'),
            'color_size': depth_result.get('color_size'),
            'color_width': int(depth_result.get('color_width') or image_w),
            'color_height': int(depth_result.get('color_height') or image_h),
            'depth_width': depth_result.get('depth_width'),
            'depth_height': depth_result.get('depth_height'),
            'depth_frame_id': str(depth_result.get('depth_frame_id') or ''),
            'camera_info_frame_id': str(depth_result.get('camera_info_frame_id') or ''),
            'configured_depth_frame_id': str(
                depth_result.get('configured_depth_frame_id')
                or self.setting_str('depth_frame_id', 'camera_color_optical_frame')
            ),
            'publish_debug_coordinates': self.setting_bool('publish_debug_coordinates', True),
            'plane_arm_point_m': depth_result.get('plane_arm_point_m'),
            'plane_arm_z_m': depth_result.get('plane_arm_z_m'),
        }
        depth_fields.update(camera_average_fields)
        publish_result.update(depth_fields)
        publish_result.update(refine_fields)
        publish_result['risk_flags'] = risk_flags
        self.vlm_result_pub.publish(String(data=json.dumps(publish_result, ensure_ascii=False)))

        stamp = self.get_clock().now().to_msg()
        pixel = PointStamped()
        pixel.header.stamp = stamp
        pixel.header.frame_id = self.setting_str('target_frame', 'image_pixel')
        pixel.point.x = float(u)
        pixel.point.y = float(v)
        pixel.point.z = 0.0
        self.pixel_pub.publish(pixel)

        if depth_ok and camera_point_ready and averaged_camera_point is not None:
            camera_point = averaged_camera_point
            point_msg = PointStamped()
            point_msg.header.stamp = depth_result.get('depth_stamp') or stamp
            point_msg.header.frame_id = depth_fields['camera_frame']
            point_msg.point.x = float(camera_point[0])
            point_msg.point.y = float(camera_point[1])
            point_msg.point.z = float(camera_point[2])
            self.camera_point_pub.publish(point_msg)
            self.legacy_camera_point_pub.publish(point_msg)
            self.depth_status_pub.publish(
                String(
                    data=(
                        f'DEPTH_OK z={float(depth_fields["depth_m"]):.3f}m '
                        f'ratio={depth_fields["depth_valid_ratio"]:.2f} '
                        f'count={depth_fields["depth_valid_count"]} roi={depth_fields["depth_roi"]} '
                        f'camera_avg={depth_fields["camera_point_average_samples"]}/'
                        f'{depth_fields["camera_point_average_required_samples"]}'
                    )
                )
            )
        elif depth_ok:
            self.depth_status_pub.publish(
                String(
                    data=(
                        f'CAMERA_POINT_AVERAGE_WAIT samples={depth_fields["camera_point_average_samples"]}/'
                        f'{depth_fields["camera_point_average_required_samples"]} '
                        f'reason={depth_fields["camera_point_average_reason"]}'
                    )
                )
            )
        else:
            self.depth_status_pub.publish(String(data=f'DEPTH_REJECT {depth_fields["depth_reason"]}'))

        label = str(publish_result['trash_label'])
        raw = str(publish_result.get('object_name') or label)
        grasp_plan = {
            'stamp': time.time(),
            'has_target': True,
            'label': label,
            'raw_label': raw,
            'object_name': raw,
            'confidence': float(publish_result['confidence']),
            'bbox_norm': bbox,
            'grasp_point_norm': center,
            'grasp_u': u,
            'grasp_v': v,
            'image_width': image_w,
            'image_height': image_h,
            'object_shape': str(publish_result.get('object_shape') or 'unknown'),
            'grasp_type': str(publish_result.get('grasp_type') or 'unknown'),
            'grasp_strategy': str(publish_result.get('grasp_strategy') or ''),
            'grasp_width_hint': str(publish_result.get('grasp_width_hint') or 'unknown'),
            'major_axis_angle_deg': float(publish_result.get('major_axis_angle_deg', 0.0) or 0.0),
            'risk_flags': publish_result.get('risk_flags') if isinstance(publish_result.get('risk_flags'), list) else [],
            'grasp_hint': str(publish_result.get('grasp_hint') or ''),
            'cached': bool(cached),
            'cache_age_sec': publish_result['cache_age_sec'],
            'provider': publish_result.get('provider'),
            'model': publish_result.get('model'),
            **depth_fields,
            **refine_fields,
        }
        self.grasp_plan_pub.publish(String(data=json.dumps(grasp_plan, ensure_ascii=False)))
        self.label_pub.publish(String(data=label))
        self.raw_label_pub.publish(String(data=raw))
        bbox_payload = {
            'stamp': time.time(),
            'image_width': image_w,
            'image_height': image_h,
            'u0': x0,
            'v0': y0,
            'u1': x1,
            'v1': y1,
            'center_u': u,
            'center_v': v,
            'grasp_u': u,
            'grasp_v': v,
            'confidence': float(publish_result['confidence']),
            'label': label,
            'raw_label': raw,
            'object_shape': grasp_plan['object_shape'],
            'grasp_type': grasp_plan['grasp_type'],
            'grasp_strategy': grasp_plan['grasp_strategy'],
            'grasp_width_hint': grasp_plan['grasp_width_hint'],
            'major_axis_angle_deg': grasp_plan['major_axis_angle_deg'],
            'grasp_hint': str(publish_result.get('grasp_hint') or ''),
            'cached': bool(cached),
            'cache_age_sec': publish_result['cache_age_sec'],
            **depth_fields,
            **refine_fields,
        }
        self.bbox_pub.publish(
            String(data=json.dumps(bbox_payload, ensure_ascii=False))
        )
        self.publish_debug_image(
            (x0, y0, x1, y1),
            (vlm_u, vlm_v),
            (u, v),
            depth_fields,
            refine_result,
            publish_result,
        )
        self.status_pub.publish(
            String(
                data=(
                    f'VLM provider={publish_result.get("provider")} model={publish_result.get("model")} '
                    f'raw={raw} mapped={label} score={float(publish_result["confidence"]):.3f} '
                    f'grasp_pixel=({u:.1f},{v:.1f}) cached={bool(cached)} '
                    f'depth_ok={depth_ok} depth_m={depth_fields["depth_m"]} '
                    f'quality={refine_fields["grasp_quality"]:.2f} '
                    f'strategy={grasp_plan["grasp_strategy"]} type={grasp_plan["grasp_type"]} '
                    f'hint={str(publish_result.get("grasp_hint") or "")[:80]} '
                    f'cache_age_sec={publish_result["cache_age_sec"]:.1f} '
                    f'latency_ms={float(publish_result.get("latency_ms", 0.0)):.1f}'
                )
            )
        )

    def destroy_node(self) -> bool:
        self.shutdown_event.set()
        thread = self.worker_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = VlmTrashClassifier()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
