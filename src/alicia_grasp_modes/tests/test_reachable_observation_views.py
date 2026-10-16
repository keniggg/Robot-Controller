"""Frozen-record endpoint reproduction and fault injection; never robot I/O."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
import alicia_grasp_modes.reachable_observation_views as module


ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')


@pytest.fixture(scope='module')
def recorded_inputs():
    review = ROOT / 'user_grasp_20261008T090751Z/source_snapshot_review'
    source_root = ROOT / 'supplement_grasp_20261008T103117Z/frozen_view_source'
    if not (source_root / 'source.npz').is_file():
        pytest.skip('sealed local gen28 source is not distributed with source code')
    source = json.loads((source_root / 'source.json').read_text())
    with np.load(str(source_root / 'source.npz')) as arrays:
        source.update({key: arrays[key].copy() for key in arrays.files})
    raw = (review / 'gen28_observation_frozen_audit.json').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == '554fffc15e60d2cbda26b11c4d1cdc8319e3e91441f4180cebb196e5542acb5f'
    audit = json.loads(raw)
    prepared = json.loads((review / 'gen28_endpoints_prepared.jsonl').read_text().splitlines()[0])
    fk = SerialUrdfFk((review / 'gen28_fk_ik_100ms.model.urdf').read_text(), ['Joint%d' % i for i in range(1, 7)])
    contract = json.loads((ROOT / 'supplement_grasp_20261008T103117Z/strict_candidate/candidate0_following_path.json').read_text())
    assert fk.model_sha256 == prepared['robot_description_sha256']
    return dict(source=source, current_joint_positions=prepared['seed']['position'][:6], fk=fk,
        gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003),
        opening_width_m=.05, gaps=audit['observe_return']['measured_missing_sides'], depth_uncertainty_m=.003,
        joint_error_bounds_rad=contract['goal_support']['joint_error_bounds_rad'], max_seconds=2., max_directions=1)


def test_recorded_noncentered_view_preserves_original_endpoint_gates_and_inputs(recorded_inputs):
    arguments = deepcopy(recorded_inputs)
    before = deepcopy(arguments)
    rows, audit = module.generate_reachable_observation_views(**arguments)
    assert audit['status'] == 'complete', audit
    assert len(rows) == 2 and audit['candidate_count'] == 2
    assert {row['exposure_target_noise_multiplier'] for row in rows} == {1., 2.}
    row = next(row for row in rows if row['exposure_target_noise_multiplier'] == 1.)
    assert not row['execution_authority'] and not row['complete_path_checked']
    assert not row['whole_arm_collision_checked'] and row['actual_arrival_measurement_required']
    assert row['binding'] == arguments['source']['binding']
    checks = row['endpoint_checks']
    for name in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                 'conditional_endpoint_support', 'measured_support_projection'):
        assert checks[name]['ok'], checks[name]
    assert checks['working_volume']['required_image_margin_px'] == 36.
    assert checks['working_volume']['minimum_image_edge_distance_px'] > 39.9
    assert checks['conditional_endpoint_support']['minimum_support_clearance_lower_bound_m'] >= .003
    assert np.allclose(arguments['fk'](row['q']), row['T_base_tool0'], atol=1e-12, rtol=0.)
    assert arguments['gaps'] == before['gaps']
    assert arguments['current_joint_positions'] == before['current_joint_positions']
    for name in ('points_base', 'corners_base', 'tool_from_camera'):
        assert np.array_equal(arguments['source'][name], before['source'][name])


def test_nested_frozen_binding_and_gaps_preserve_values_and_output_isolation(recorded_inputs):
    from test_remote_modes import base
    from types import MappingProxyType
    arguments = deepcopy(recorded_inputs)
    nested = {'tuple': ({'value': 1791509854735741138},), 'list': [1., 2.]}
    arguments['source']['binding']['extra_evidence'] = nested
    arguments['source'] = base._freeze_request_evidence(arguments['source'])
    arguments['gaps'] = base._freeze_request_evidence(arguments['gaps'])
    assert isinstance(arguments['gaps'][0], MappingProxyType)
    rows, audit = module.generate_reachable_observation_views(**arguments)
    assert rows and audit['status'] == 'complete', audit
    binding = arguments['source']['binding']
    assert rows[0]['binding'] == audit['binding'] == binding
    assert type(rows[0]['binding']['snapshot_stamp_ns']) is int
    assert isinstance(rows[0]['binding']['extra_evidence']['tuple'], tuple)
    rows[0]['binding']['extra_evidence']['tuple'][0]['value'] = -1
    assert binding['extra_evidence']['tuple'][0]['value'] == 1791509854735741138
    assert audit['binding']['extra_evidence']['tuple'][0]['value'] == 1791509854735741138
    with pytest.raises(TypeError):
        binding['extra_evidence']['tuple'][0]['value'] = 0


@pytest.mark.parametrize('fault', ['foreign_track', 'future_gap', 'bad_stamp'])
def test_actual_frozen_invalid_evidence_still_fails_before_solver(recorded_inputs, monkeypatch, fault):
    from test_remote_modes import base
    arguments = deepcopy(recorded_inputs)
    if fault == 'foreign_track':
        arguments['gaps'][0]['target_track_id'] = 'foreign'
    elif fault == 'future_gap':
        arguments['gaps'][0]['source_view_stamps_ns'] = [arguments['source']['binding']['snapshot_stamp_ns'] + 1]
    else:
        arguments['source']['binding']['snapshot_stamp_ns'] = True
    arguments['source'] = base._freeze_request_evidence(arguments['source'])
    arguments['gaps'] = base._freeze_request_evidence(arguments['gaps'])
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('invalid frozen source reached solver'))
    rows, audit = module.generate_reachable_observation_views(**arguments)
    assert not rows and audit['status'] == 'invalid_input'
    assert audit['execution_authority'] is False and audit['original_candidates_removed'] == 0


@pytest.mark.parametrize('fault', ['nan_q', 'outside_limit', 'nan_support', 'foreign_track',
    'future_gap', 'empty_errors', 'zero_errors', 'negative_budget', 'bool_directions',
    'bad_iterations', 'relaxed_fov', 'negative_reserve'])
def test_invalid_or_foreign_inputs_never_enter_solver(recorded_inputs, monkeypatch, fault):
    arguments = deepcopy(recorded_inputs)
    if fault == 'nan_q': arguments['current_joint_positions'][0] = float('nan')
    elif fault == 'outside_limit': arguments['current_joint_positions'][0] = 10.
    elif fault == 'nan_support': arguments['source']['points_base'][0, 0] = float('nan')
    elif fault == 'foreign_track': arguments['gaps'][0]['target_track_id'] = 'foreign'
    elif fault == 'future_gap': arguments['gaps'][0]['source_view_stamps_ns'] = [arguments['source']['binding']['snapshot_stamp_ns'] + 1]
    elif fault == 'empty_errors': arguments['joint_error_bounds_rad'] = []
    elif fault == 'zero_errors': arguments['joint_error_bounds_rad'] = [0.] * 6
    elif fault == 'negative_budget': arguments['max_seconds'] = -1.
    elif fault == 'bool_directions': arguments['max_directions'] = True
    elif fault == 'bad_iterations': arguments['max_iterations'] = 161
    elif fault == 'relaxed_fov': arguments['source']['view_limits']['effective_visibility_limits']['roi_margin_px'] = 23.
    elif fault == 'negative_reserve': arguments['proposal_image_reserve_px'] = -1.
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('invalid input reached backend'))
    rows, audit = module.generate_reachable_observation_views(**arguments)
    assert rows == () and audit['status'] == 'invalid_input'
    assert audit['execution_authority'] is False


def test_missing_solver_backend_is_diagnostic_only(recorded_inputs, monkeypatch):
    def missing(): raise ImportError('backend unavailable')
    monkeypatch.setattr(module, '_backend', missing)
    rows, audit = module.generate_reachable_observation_views(**recorded_inputs)
    assert rows == () and audit['status'] == 'backend_unavailable'
    assert audit['original_candidates_removed'] == 0


def test_expired_budget_cannot_start_solver(recorded_inputs, monkeypatch):
    times = iter([0., 3., 3.])
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('late backend start'))
    rows, audit = module.generate_reachable_observation_views(**recorded_inputs, clock=lambda: next(times))
    assert rows == () and audit['status'] == 'budget_exhausted'


def test_solver_success_cannot_replace_physical_endpoint_checks(recorded_inputs, monkeypatch):
    def fake_solver(_objective, q, **kwargs):
        return SimpleNamespace(x=q, nit=1, success=True, message='untrusted solver claims success')
    monkeypatch.setattr(module, '_backend', lambda: fake_solver)
    rows, audit = module.generate_reachable_observation_views(**recorded_inputs)
    assert rows == () and audit['candidate_count'] == 0
    assert audit['directions'][0]['accepted'] is False


def test_late_solver_result_cannot_become_a_proposal(recorded_inputs, monkeypatch):
    now = [0.]
    def late(_objective, q, **kwargs):
        now[0] = 3.
        return SimpleNamespace(x=q, nit=1, success=True, message='late')
    monkeypatch.setattr(module, '_backend', lambda: late)
    rows, audit = module.generate_reachable_observation_views(**recorded_inputs, clock=lambda: now[0])
    assert rows == () and audit['status'] == 'budget_exhausted'
    assert audit['directions'][0]['reason'] == 'shared wall-clock budget exhausted'
