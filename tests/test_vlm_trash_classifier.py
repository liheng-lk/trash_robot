import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'trash_robot_vision'))

from trash_robot_vision.vlm_trash_classifier import (  # noqa: E402
    apply_geometric_grasp_rules,
    extract_json_object,
    infer_grasp_semantics,
    normalized_bbox_to_pixels,
    validate_vlm_result,
)


def test_vlm_config_has_no_inline_secret_values():
    config_file = ROOT / 'config/perception/vlm_trash_classifier.yaml'
    data = yaml.safe_load(config_file.read_text(encoding='utf-8')) or {}
    registry_file = data.get('provider_registry_file')
    assert registry_file
    registry = yaml.safe_load((config_file.parent / registry_file).read_text(encoding='utf-8')) or {}
    providers = data.get('providers', {})
    assert providers
    for name, provider in providers.items():
        assert 'api_key' not in provider, f'{name} must use env vars, not inline keys'
        assert 'api_key_env' in provider
    for name, provider in registry.get('providers', {}).items():
        assert 'api_key' not in provider, f'{name} registry must use env vars, not inline keys'
        assert 'api_key_env' in provider
        if provider.get('enabled', False) and name != 'local_hobot':
            assert provider.get('primary_model') or provider.get('model_candidates')


def test_extract_json_object_accepts_markdown_fence():
    data = extract_json_object('```json\n{"has_target": false, "reason": "no trash"}\n```')
    assert data['has_target'] is False


def test_validate_accepts_four_canonical_labels():
    for label in ['GARBAGE_RECYCLE', 'GARBAGE_OTHER', 'GARBAGE_HAZARD', 'GARBAGE_KITCHEN']:
        ok, reason, cleaned = validate_vlm_result(
            {
                'has_target': True,
                'trash_label': label,
                'object_name': 'test',
                'bbox_norm': [0.1, 0.2, 0.4, 0.6],
                'center_norm': [0.25, 0.4],
                'grasp_point_norm': [0.27, 0.43],
                'grasp_hint': 'pinch middle',
                'confidence': 0.88,
                'reason': 'test',
            },
            min_confidence=0.55,
        )
        assert ok, reason
        assert cleaned['trash_label'] == label
        assert cleaned['center_norm'] == [0.27, 0.43]
        assert cleaned['grasp_point_norm'] == [0.27, 0.43]
        assert cleaned['grasp_hint'] == 'pinch middle'
        assert cleaned['object_shape'] == 'unknown'
        assert cleaned['grasp_type'] == 'unknown'


@pytest.mark.parametrize(
    'payload,reason_prefix',
    [
        ({'has_target': True, 'trash_label': 'chair', 'bbox_norm': [0.1, 0.2, 0.4, 0.6], 'center_norm': [0.2, 0.3], 'confidence': 0.9}, 'INVALID_LABEL'),
        ({'has_target': True, 'trash_label': 'GARBAGE_OTHER', 'bbox_norm': [0.1, 0.2, 0.4, 0.6], 'center_norm': [0.2, 0.3], 'confidence': 0.1}, 'LOW_CONFIDENCE'),
        ({'has_target': True, 'trash_label': 'GARBAGE_OTHER', 'bbox_norm': [0.5, 0.2, 0.4, 0.6], 'center_norm': [0.2, 0.3], 'confidence': 0.9}, 'INVALID_BBOX'),
        ({'has_target': True, 'trash_label': 'GARBAGE_OTHER', 'bbox_norm': [0.1, 0.2, 1.4, 0.6], 'center_norm': [0.2, 0.3], 'confidence': 0.9}, 'COORD_OUT_OF_RANGE'),
    ],
)
def test_validate_fail_closed(payload, reason_prefix):
    ok, reason, _ = validate_vlm_result(payload, min_confidence=0.55)
    assert not ok
    assert reason.startswith(reason_prefix)


def test_non_trash_does_not_become_target():
    ok, reason, cleaned = validate_vlm_result(
        {
            'has_target': False,
            'trash_label': '',
            'object_name': 'chair',
            'bbox_norm': [0, 0, 0, 0],
            'center_norm': [0, 0],
            'confidence': 0,
            'reason': 'background furniture',
        },
        min_confidence=0.55,
    )
    assert not ok
    assert cleaned['has_target'] is False
    assert cleaned['grasp_point_norm'] == [0.0, 0.0]


def test_normalized_bbox_to_pixels_uses_image_dimensions():
    x0, y0, x1, y1, u, v = normalized_bbox_to_pixels(
        [0.25, 0.25, 0.75, 0.75],
        [0.5, 0.5],
        width=640,
        height=480,
    )
    assert (x0, y0, x1, y1, u, v) == (160.0, 120.0, 480.0, 360.0, 320.0, 240.0)


def test_battery_grasp_point_is_stabilized_to_bbox_midbody():
    result = {
        'has_target': True,
        'trash_label': 'GARBAGE_HAZARD',
        'object_name': 'battery',
        'bbox_norm': [0.40, 0.20, 0.50, 0.80],
        'center_norm': [0.45, 0.26],
        'grasp_point_norm': [0.45, 0.26],
        'grasp_hint': 'pinch upper body',
        'confidence': 0.9,
        'reason': 'battery is hazardous waste',
    }
    adjusted = apply_geometric_grasp_rules(
        result,
        {'battery': {'enabled': True, 'vertical_y_ratio': 0.52}},
    )
    assert adjusted['grasp_rule'] == 'battery_bbox_midbody_vertical'
    assert adjusted['grasp_strategy'] == 'slender_midbody'
    assert adjusted['grasp_type'] == 'clamp_midbody'
    assert adjusted['vlm_grasp_point_norm'] == [0.45, 0.26]
    assert adjusted['grasp_point_norm'][0] == pytest.approx(0.45)
    assert adjusted['grasp_point_norm'][1] == pytest.approx(0.512)


def test_non_battery_grasp_point_is_not_overridden():
    result = {
        'has_target': True,
        'trash_label': 'GARBAGE_RECYCLE',
        'object_name': 'paper ball',
        'bbox_norm': [0.20, 0.20, 0.60, 0.70],
        'center_norm': [0.30, 0.40],
        'grasp_point_norm': [0.30, 0.40],
    }
    adjusted = apply_geometric_grasp_rules(result, {'battery': {'enabled': True}})
    assert adjusted['grasp_point_norm'] == [0.30, 0.40]
    assert 'grasp_rule' not in adjusted
    assert adjusted['grasp_strategy'] == 'crumpled_center'


def test_grasp_semantics_infers_shape_strategy_for_common_trash():
    battery = infer_grasp_semantics({'has_target': True, 'object_name': 'AA battery', 'trash_label': 'GARBAGE_HAZARD'})
    assert battery['grasp_strategy'] == 'slender_midbody'
    assert battery['object_shape'] == 'slender'

    paper = infer_grasp_semantics({'has_target': True, 'object_name': 'crumpled paper ball', 'trash_label': 'GARBAGE_RECYCLE'})
    assert paper['grasp_strategy'] == 'crumpled_center'
    assert paper['grasp_type'] == 'pinch'

    bottle = infer_grasp_semantics({'has_target': True, 'object_name': 'plastic bottle', 'trash_label': 'GARBAGE_RECYCLE'})
    assert bottle['grasp_strategy'] == 'cylindrical_midbody'
    assert bottle['grasp_type'] == 'clamp_midbody'


def test_vlm_runtime_limits_are_configured_for_rdk():
    data = yaml.safe_load((ROOT / 'config/perception/vlm_trash_classifier.yaml').read_text(encoding='utf-8')) or {}
    assert 0.2 <= float(data['call_interval_sec']) <= 10.0
    assert 1.0 <= float(data['api_timeout_sec']) <= 15.0
    assert int(data['max_image_width']) <= 640
    assert int(data['max_consecutive_failures']) >= 1
