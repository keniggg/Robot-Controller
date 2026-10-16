"""Configuration identity survives measured camera diagnostics, not policy changes.

Only the configuration revalidation boundary is exercised here. A successful
case does not certify a trajectory or grant any execution authority.
"""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from alicia_flexible_grasp.robot import observation_view_gateway as view
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from test_observation_view_gateway import configured


RECORD_ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records')
RECORDS = [
    ('gen34', '20261008T124106.426913Z_8690434647cf4b4e838df3f34e6b250c', 96),
    ('oct09_execution', '20261009T004422.530234Z_2ca7b5431ccf4c3f93c20132672c5172', 69),
]


@pytest.fixture(scope='module', params=RECORDS, ids=[row[0] for row in RECORDS])
def recorded_cameras(request):
    _, directory, expected_count = request.param
    bag_path = RECORD_ROOT / directory / 'keyframes.bag'
    if not bag_path.is_file():
        pytest.skip('sealed camera metadata is an external local-only experiment fixture')
    import rosbag
    rows = []
    with rosbag.Bag(str(bag_path), 'r') as bag:
        for _, message, _ in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']):
            row = json.loads(message.data)
            camera = row['parameters']
            assert isinstance(camera['runtime_profile']['stationary_temporal'], dict)
            rows.append(camera)
    assert len(rows) == expected_count
    base = deepcopy(rows[0])
    base['runtime_profile'].pop('stationary_temporal')
    for camera in rows:
        stable = deepcopy(camera)
        stable['runtime_profile'].pop('stationary_temporal')
        assert stable == base, 'fixture must isolate diagnostics from camera reconfiguration'
    assert any(row != rows[0] for row in rows[1:]), 'real metadata must reproduce old equality failure'
    return rows


def _audit(monkeypatch, camera):
    gateway, scene, params, _, _ = configured(monkeypatch)
    params['/camera'] = deepcopy(camera)
    monkeypatch.setattr(view, 'validate_observation_target_trajectory',
                        lambda *args, **kwargs: {'ok': True, 'test_scope': 'configuration_only'})
    audit = gateway._unknown_observation_view_audit(scene, None, None, None)
    return gateway, scene, params, audit


def test_real_diagnostic_changes_do_not_invalidate_configuration(monkeypatch, recorded_cameras):
    gateway, scene, params, audit = _audit(monkeypatch, recorded_cameras[0])
    frozen = deepcopy(audit)
    assert audit['configuration']['camera_configuration_snapshot'] == recorded_cameras[0]
    for camera in recorded_cameras[1:]:
        params['/camera'] = deepcopy(camera)
        gateway._revalidate_unknown_observation_view(scene, audit)
    # Canonical comparison must not discard or mutate the original evidence.
    assert audit == frozen


@pytest.mark.parametrize('mutation', [
    'fx', 'depth_scale', 'activation', 'preset', 'filter_alpha', 'owner_schema',
    'region_bounds', 'visual_threshold', 'unknown_camera', 'unknown_runtime',
    'unknown_temporal', 'unknown_region', 'unknown_visual',
])
def test_real_metadata_keeps_configuration_and_unknown_changes_blocked(monkeypatch, recorded_cameras, mutation):
    camera = next(row for row in recorded_cameras
                  if row['runtime_profile']['stationary_temporal'].get('regions'))
    gateway, scene, params, audit = _audit(monkeypatch, camera)
    current = params['/camera']
    runtime = current['runtime_profile']
    temporal = runtime['stationary_temporal']
    if mutation == 'fx':
        current['fx'] += .1
    elif mutation == 'depth_scale':
        current['depth_scale'] *= 1.01
    elif mutation == 'activation':
        runtime['geometry_activation_stamp_ns'] = str(int(runtime['geometry_activation_stamp_ns']) + 1)
    elif mutation == 'preset':
        runtime['mode_depth_preset']['preset'] = 'test_changed_preset'
    elif mutation == 'filter_alpha':
        temporal['alpha'] = .5
    elif mutation == 'owner_schema':
        temporal['owner_schema'] += '_changed'
    elif mutation == 'region_bounds':
        temporal['regions'][0]['bounds_xyxy'][0] += 1
    elif mutation == 'visual_threshold':
        temporal['regions'][0]['visual']['max_motion_px'] *= 2
    elif mutation == 'unknown_camera':
        current['unrecognized_geometry_policy'] = 1
    elif mutation == 'unknown_runtime':
        runtime['unrecognized_geometry_policy'] = 1
    elif mutation == 'unknown_temporal':
        temporal['unrecognized_filter_policy'] = 1
    elif mutation == 'unknown_region':
        temporal['regions'][0]['unrecognized_region_policy'] = 1
    elif mutation == 'unknown_visual':
        temporal['regions'][0]['visual']['unrecognized_visual_policy'] = 1
    with pytest.raises(ObservationPathError):
        gateway._revalidate_unknown_observation_view(scene, audit)


def test_real_regional_active_to_no_output_is_diagnostic(monkeypatch, recorded_cameras):
    camera = next(row for row in recorded_cameras
                  if row['runtime_profile']['stationary_temporal'].get('regions'))
    gateway, scene, params, audit = _audit(monkeypatch, camera)
    temporal = params['/camera']['runtime_profile']['stationary_temporal']
    temporal.update(active=False, regional_output_active=False, fully_active=False,
                    filtered_pixel_count=0, filtered_fraction=0., history_warming_up=False,
                    regions=[], history_reset=True)
    gateway._revalidate_unknown_observation_view(scene, audit)


def _legacy_camera(monkeypatch):
    _, _, params, _, _ = configured(monkeypatch)
    camera = deepcopy(params['/camera'])
    camera['runtime_profile'] = dict(source='realsense_active_profile', stationary_temporal=dict(
        enabled=True, active=False, history_warming_up=True,
        alpha=.4, delta_depth_units=100, persistence=0,
        stationary_window_sec=.35, evidence_max_age_sec=.25,
        visual_stationarity_required=True, visual_window_sec=.35,
        visual_max_motion_full_resolution_px=.25,
        scope='unknown_only; encoder AND continuous SDK RGB evidence; reset on motion or missing evidence'))
    return camera


def test_legacy_active_warming_changes_without_external_fixture(monkeypatch):
    gateway, scene, params, audit = _audit(monkeypatch, _legacy_camera(monkeypatch))
    params['/camera']['runtime_profile']['stationary_temporal'].update(active=True, history_warming_up=False)
    gateway._revalidate_unknown_observation_view(scene, audit)


@pytest.mark.parametrize('key,value', [
    ('enabled', False), ('alpha', .5), ('delta_depth_units', 101),
    ('persistence', 1), ('stationary_window_sec', .2), ('evidence_max_age_sec', .5),
    ('visual_stationarity_required', False), ('visual_window_sec', .2),
    ('visual_max_motion_full_resolution_px', .5), ('scope', 'changed'),
    ('unknown_future_policy', 'changed'),
])
def test_legacy_policy_changes_block_without_external_fixture(monkeypatch, key, value):
    gateway, scene, params, audit = _audit(monkeypatch, _legacy_camera(monkeypatch))
    params['/camera']['runtime_profile']['stationary_temporal'][key] = value
    with pytest.raises(ObservationPathError):
        gateway._revalidate_unknown_observation_view(scene, audit)
