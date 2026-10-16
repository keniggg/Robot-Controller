"""Independent repeated live guards: mutable owner inputs never become cache authority."""
from copy import deepcopy
from dataclasses import replace
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import Header

from test_locked_rgb_hold import locked, module
from test_observation_measurement_unknown import frozen, arrival, send, ack
from test_measured_face_source_binding import request_for


def warm(n):
    pending = n._observation_measurement
    for _ in range(3):
        n._check_measurement_binding(pending)
    return pending


@pytest.mark.parametrize('field', ['mask', 'position_camera', 'position_base', 'extent_m',
    'support_footprint_base', 'bbox', 'center_uv', 'depth_m', 'quality',
    'support_plane_base', 'valid_points'])
@pytest.mark.parametrize('owner', ['anchor', 'last'])
def test_warmed_candidate_guard_detects_same_object_in_place_value_changes(monkeypatch, field, owner):
    n, _, _, _, _, _ = frozen(monkeypatch)
    pending = warm(n)
    candidate = getattr(n.tracker, owner)
    identity = id(candidate)
    if field in ('support_plane_base', 'valid_points'):
        if field == 'valid_points':
            candidate.metrics[field] += 1
        else:
            values = candidate.metrics[field]
            values[0] += .001
    elif field == 'bbox':
        candidate.bbox = (candidate.bbox[0]+1, *candidate.bbox[1:])
    elif field in ('depth_m', 'quality'):
        setattr(candidate, field, getattr(candidate, field)+.001)
    else:
        value = getattr(candidate, field)
        assert isinstance(value, np.ndarray)
        if field == 'support_footprint_base':
            # This array is already immutable; replacing the field must still
            # invalidate the same mutable Candidate object.
            with pytest.raises(ValueError):
                value.flat[0] += .001
            value = value.copy()
            candidate.support_footprint_base = value
        if field == 'mask':
            value.flat[0] = 1 if value.flat[0] == 0 else 0
        else:
            value.flat[0] += .001
    assert id(getattr(n.tracker, owner)) == identity
    with pytest.raises(ValueError, match='identity_changed'):
        n._check_measurement_binding(pending)


@pytest.mark.parametrize('change', ['config_value', 'config_type', 'plane', 'perception_nested',
    'camera_fx', 'camera_scalar_type', 'camera_profile_geometry', 'camera_rectification',
    'camera_activation', 'camera_unknown_setting', 'handeye_nested', 'selection', 'model',
    'gui_manual', 'epoch', 'lost', 'first_loss', 'explicit_invalidated'])
def test_warmed_guard_rechecks_mutable_configuration_and_owner_every_time(monkeypatch, change):
    n, _, _, _, params, _ = frozen(monkeypatch, camera_profile=True)
    pending = warm(n)
    if change == 'config_value': n.config.support_threshold_m += .0001
    elif change == 'config_type': n.config.support_min_points = float(n.config.support_min_points)
    elif change == 'plane': n.tracker.plane_base[3] += .001
    elif change == 'perception_nested': params['/unknown_perception']['new'] = {'values': [1., 2.]}
    elif change == 'camera_fx': params['/camera']['fx'] += .01
    elif change == 'camera_scalar_type': params['/camera']['width'] = float(params['/camera']['width'])
    elif change == 'camera_profile_geometry': params['/camera']['runtime_profile']['depth_to_color_extrinsics'] = {'translation': [0., 0., .001]}
    elif change == 'camera_rectification': params['/camera']['pair_rectification'] = True
    elif change == 'camera_activation': params['/camera']['runtime_profile']['geometry_activation_stamp_ns'] = 123
    elif change == 'camera_unknown_setting': params['/camera']['runtime_profile']['stationary_temporal']['new_setting'] = 1
    elif change == 'handeye_nested': params['/handeye']['matrix'] = [[1., .01], [0., 1.]]
    elif change == 'selection': params['/grasp_mode/selection']['generation'] += 1
    elif change == 'model': params['/robot_description'] = '<robot name="different"/>'
    elif change == 'gui_manual': params['/gui/joint_direct_mode'] = True
    elif change == 'epoch': n._rgb_interest_epoch += 1
    elif change == 'lost': n.tracker.lost = True
    elif change == 'first_loss': n.first_loss = {'reason': 'confirmed prior loss'}
    else: pending['invalidated_reason'] = 'independent_explicit_revoke'
    with pytest.raises(ValueError):
        n._check_measurement_binding(pending)


@pytest.mark.parametrize('which', ['source_tf', 'current_tf', 'both_tf'])
def test_warmed_guard_repeats_source_and_latest_mount_queries(monkeypatch, which):
    n, _, _, _, _, _ = frozen(monkeypatch)
    pending = warm(n)
    original = module.transform_matrix
    def changed(args):
        matrix = original(args)
        if args[0] == 'tool0' and (which == 'both_tf' or
                (args[2].to_nsec() == 0) == (which == 'current_tf')):
            matrix[0, 3] += .001
        return matrix
    monkeypatch.setattr(module, 'transform_matrix', changed)
    with pytest.raises(ValueError, match='(?:mount|configuration)_changed'):
        n._check_measurement_binding(pending)


def test_every_unchanged_checkpoint_reads_gui_model_params_and_both_tf(monkeypatch):
    n, _, _, _, params, _ = frozen(monkeypatch)
    pending = warm(n)
    get = Mock(side_effect=lambda key, default=None: params.get(key, default))
    lookup = Mock(side_effect=n.buffer.lookup_transform)
    monkeypatch.setattr(module.rospy, 'get_param_cached', get)
    monkeypatch.setattr(n.buffer, 'lookup_transform', lookup)
    before = pending['deadline_monotonic']
    for _ in range(7):
        n._check_measurement_binding(pending)
    keys = [c.args[0] for c in get.call_args_list]
    for key in ('/gui/joint_direct_mode', '/robot_description', '/camera',
                '/grasp_mode/selection', '/handeye', '/unknown_perception'):
        assert keys.count(key) >= 7, (key, keys)
    assert lookup.call_count == 14
    assert sum(c.args[2].to_nsec() == 0 for c in lookup.call_args_list) == 7
    assert pending['deadline_monotonic'] == before


@pytest.mark.parametrize('clock_kind', ['ros_only', 'monotonic_only'])
def test_warmed_guard_keeps_both_original_absolute_deadlines(monkeypatch, clock_kind):
    n, clock, _, _, _, request = frozen(monkeypatch)
    pending = warm(n)
    start = clock[0]
    if clock_kind == 'ros_only':
        monkeypatch.setattr(module.time, 'monotonic', lambda: start)
        clock[0] = request['deadline_ns']*1e-9
    else:
        monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(start)))
        clock[0] = pending['deadline_monotonic']
    with pytest.raises(ValueError, match='deadline_exceeded'):
        n._check_measurement_binding(pending)


def test_warmed_arrival_guard_preserves_sdk_out_and_back_invalidation(monkeypatch):
    n, clock, _, joint, _, request = frozen(monkeypatch)
    arrival(n, clock, joint, request)
    pending = warm(n)
    pending['arrival_q'] = np.zeros(6)
    joint(clock[0]+.01, [.005, 0., 0., 0., 0., 0.])
    joint(clock[0]+.02, [0.]*6)
    assert np.array_equal(n._rgb_interest_feedback[-1][1], np.zeros(6))
    with pytest.raises(ValueError, match='movement_after_stationary_arrival'):
        n._check_measurement_binding(pending)


@pytest.mark.parametrize('change', ['source_age', 'monotonic_source', 'sdk_out_and_back'])
def test_pre_freeze_repeated_checkpoints_cannot_admit_expired_or_moved_source(monkeypatch, change):
    n, clock, _, joint, params = locked(monkeypatch)
    request = request_for(n, clock, params)
    start = clock[0]
    before = n.last_good_source_stamp_ns
    seen = []
    def freeze_then_change(*args, **kwargs):
        check = kwargs['check_current']
        check(); check()
        seen.append(True)
        if change == 'source_age':
            monkeypatch.setattr(module.time, 'monotonic', lambda: start)
            clock[0] += .801
        elif change == 'monotonic_source':
            monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(start)))
            clock[0] += .801
        else:
            joint(clock[0]-.02, [.005, 0., 0., 0., 0., 0.])
            joint(clock[0]-.01, [0.]*6)
        check()
        pytest.fail('old source was admitted after changed checkpoint')
    monkeypatch.setattr(module, 'freeze_measured_face', freeze_then_change)
    send(n, request)
    assert seen == [True]
    assert not ack(n)['accepted']
    assert getattr(n, '_observation_measurement', None) is None
    assert n.last_good_source_stamp_ns == before
    assert 'face-lease' not in getattr(n, '_observation_measurement_seen', ())


@pytest.mark.parametrize('field', ['mask', 'position_base', 'valid_points'])
def test_warmed_candidate_guard_preserves_dtype_sensitive_original_fingerprint(monkeypatch, field):
    n, _, _, _, _, _ = frozen(monkeypatch)
    pending = warm(n)
    candidate = n.tracker.anchor
    if field == 'valid_points':
        candidate.metrics[field] = float(candidate.metrics[field])
    else:
        old = getattr(candidate, field)
        converted = old.astype((np.uint8 if old.dtype == np.bool_ else bool) if field == 'mask' else np.float32)
        assert converted.dtype != old.dtype
        # Mask has the same numeric values; original dtype remains part of its fingerprint.
        setattr(candidate, field, converted)
    with pytest.raises(ValueError, match='identity_changed'):
        n._check_measurement_binding(pending)


@pytest.mark.parametrize('before,after', [
    (1, 1.), (True, 1), (0., -0.), ([1, 2], (1, 2)),
    (np.array([1], np.int32), np.array([1], np.int64)),
    (np.array([1., 2.]), np.array([[1., 2.]])),
    (np.array([0.]), np.array([-0.])),
    ({'nested': [1, {'value': 2}]}, {'nested': [1, {'value': 3}]}),
])
def test_pure_memo_requires_exact_types_shape_dtype_signed_zero_and_nested_values(before, after):
    memo = module._ExactValueMemo()
    calls = []
    def compute(value):
        calls.append(deepcopy(value))
        return str(len(calls))
    assert memo.get(before, compute) == '1'
    assert memo.get(deepcopy(before), compute) == '1'
    assert memo.get(after, compute) == '2'
    assert len(calls) == 2


def test_pure_memo_owns_nested_snapshot_and_rechecks_in_place_array_changes():
    raw = {'items': [{'array': np.arange(8, dtype=np.float64)}]}
    memo = module._ExactValueMemo()
    calls = []
    def compute(snapshot):
        assert snapshot is not raw and snapshot['items'] is not raw['items']
        assert not np.shares_memory(snapshot['items'][0]['array'], raw['items'][0]['array'])
        calls.append(snapshot['items'][0]['array'].copy())
        return str(float(snapshot['items'][0]['array'].sum()))
    assert memo.get(raw, compute) == '28.0'
    assert memo.get(raw, compute) == '28.0'
    raw['items'][0]['array'][0] = .5
    assert memo.get(raw, compute) == '28.5'
    assert len(calls) == 2 and calls[0][0] == 0.


def test_pure_memo_rejects_input_changes_during_actual_miss_compute_without_poisoning_old_entry():
    raw = {'values': [1, 2]}
    memo = module._ExactValueMemo()
    assert memo.get(raw, lambda value: str(value)) == "{'values': [1, 2]}"
    raw['values'][1] = 3
    def change_while_computing(snapshot):
        raw['values'][1] = 4
        return str(snapshot)
    with pytest.raises(ValueError, match='changed_during_snapshot'):
        memo.get(raw, change_while_computing)
    calls = []
    assert memo.get(raw, lambda snapshot: calls.append(1) or str(snapshot)) == "{'values': [1, 4]}"
    assert calls == [1]


@pytest.mark.parametrize('unsupported', [np.float64(1.), {1: 'integer-key'}, np.array([object()], dtype=object)])
def test_unsupported_values_always_execute_original_pure_validator(unsupported):
    memo = module._ExactValueMemo()
    calls = []
    for index in range(3):
        assert memo.get(unsupported, lambda snapshot: calls.append(snapshot) or 'original') == 'original'
    assert len(calls) == 3


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_original_nonfinite_json_rejection_cannot_be_cached_as_success(value):
    memo = module._ExactValueMemo()
    for _ in range(2):
        with pytest.raises(ValueError):
            memo.get({'value': value}, module.digest)


def test_compute_owner_exception_is_exact_and_cannot_populate_a_success_entry():
    memo = module._ExactValueMemo()
    error = ValueError('real original rejection')
    calls = []
    def fail(snapshot):
        calls.append(snapshot)
        raise error
    for _ in range(2):
        with pytest.raises(ValueError) as caught:
            memo.get({'v': 1}, fail)
        assert caught.value is error
    assert len(calls) == 2


def test_source_context_export_does_not_alias_private_cache_or_current_parameters(monkeypatch):
    n, _, _, _, params, _ = frozen(monkeypatch, camera_profile=True)
    pending = warm(n)
    stamp = module.rospy.Time.from_sec(pending['reference_stamp_ns']*1e-9)
    first = n._rgb_interest_source_context(stamp)
    before = module.digest(first)
    first['camera']['fx'] += 1
    first['selection']['generation'] += 1
    first['perception']['injected'] = [1, 2, 3]
    first['tool_camera'][0][3] += .01
    second = n._rgb_interest_source_context(stamp)
    assert module.digest(second) == before
    assert params['/camera']['fx'] == second['camera']['fx']
    assert params['/grasp_mode/selection']['generation'] == second['selection']['generation']
    n._check_measurement_binding(pending)
