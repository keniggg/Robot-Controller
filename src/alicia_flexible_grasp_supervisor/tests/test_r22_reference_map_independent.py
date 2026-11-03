"""Independent source-bias boundaries; no ROS transport or device access.

Synthetic source pairs are explicitly labelled. Recorded endpoint vectors are
copied verbatim from the immutable r21 endpoint comparison, SHA256
0fa73d9fe71c5cd4674366354c6494e21e3dc80b1f3eb67043f4bc3baf2e06bc.
Using its frozen bias in a synthetic pair tests arithmetic, not source admission.
Only a separate live lease can admit a next episode or authorize motion.
"""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
import math

import numpy as np
import pytest

from alicia_flexible_grasp.robot.following_model import (
    ARM_NAMES, SDK_QUANTUM_RAD as Q, SourceBiasModel,
    following_evidence, following_offset, source_bias_model,
    validate_source_bias_model,
)

REAL_ENCODER = np.array([-1.8453788878264263, -0.5123495831538042,
    0.9924855697620099, -0.07516505860639641, -1.0891263593988052,
    -1.9726992932209346])
REAL_WIRE = np.array([-1.8438449070385408, -0.46172821715357804,
    0.9970875121256668, -0.06902913545485385, -1.0875923786109196,
    -1.9803691971603627])
RECORDED_BIAS_COUNTS = np.array([-6, 13, 11, -5, -4, -4])
MODEL_SHA = '7812d6b69e854ed9f37b4e3cbd813ecd53dab3504065b2d04d287abc6fd852e0'


def synthetic_source():
    """A synthetic, quantized source pair with the recorded offset vector."""
    actual = (np.array([2000, 1800, 2300, 2050, 1700, 1900])-2048)*Q
    sdk = actual+RECORDED_BIAS_COUNTS*Q
    return dict(measurement_kind='stationary_sdk_following_not_executed_endpoint',
        model_sha256=MODEL_SHA, epoch_ns=1_000_000_000, joint_names=list(ARM_NAMES),
        stamp_sec=10., sdk_stamp_ns=9_990_000_000, accepted_stamp_ns=9_991_000_000,
        stationary_window_start_ns=9_690_000_000, stationary_window_end_ns=9_990_000_000,
        sdk_positions_rad=sdk.tolist(), accepted_positions_rad=actual.tolist())


def model():
    return source_bias_model(synthetic_source(), binding_id='independent-synthetic-episode-1',
                             source_bias_limit_rad=(.09,)*6)


def test_r22_real_raw_33_count_endpoint_still_fails_legacy():
    before = REAL_ENCODER.copy()
    with pytest.raises(ValueError, match='FOLLOWING_OUTSIDE_CONTRACT'):
        following_offset(REAL_WIRE, REAL_ENCODER)
    np.testing.assert_array_equal(REAL_ENCODER, before)


def test_r22_recorded_offset_arithmetic_keeps_twenty_count_residual():
    m = model()
    frozen = following_offset(REAL_WIRE, REAL_ENCODER, following_model=m)
    np.testing.assert_allclose(frozen/Q, RECORDED_BIAS_COUNTS, atol=1e-12)
    e = following_evidence(REAL_WIRE, REAL_ENCODER, following_model=m)
    np.testing.assert_allclose(np.array(e['raw_sdk_minus_encoder_rad'])/Q,
        [1, 33, 3, 4, 1, -5], atol=1e-12)
    np.testing.assert_allclose(np.array(e['physical_reference_minus_encoder_rad'])/Q,
        [7, 20, -8, 9, 5, -1], atol=1e-12)
    assert e['domain'] == 'mapped_physical'
    assert e['source_bias_sha256'] == m.digest
    assert not e['execution_authorized'] and not e['future_following_verified']


def test_r22_later_pose_drift_cannot_silently_reestimate_frozen_bias():
    m = model()
    saved = m.digest
    actual = REAL_ENCODER.copy(); actual[1] -= 3*Q
    with pytest.raises(ValueError, match='FOLLOWING_OUTSIDE_CONTRACT'):
        following_offset(REAL_WIRE, actual, following_model=m)
    assert m.digest == saved
    np.testing.assert_allclose(np.array(m.bias_rad)/Q, RECORDED_BIAS_COUNTS, atol=1e-12)


@pytest.mark.parametrize('residual_counts,qualified', [(22, True), (23, False), (-22, True), (-23, False)])
def test_r22_quantized_residual_original_point035_boundary(residual_counts, qualified):
    m = model(); source = synthetic_source()
    sdk = np.array(source['sdk_positions_rad'])
    actual = np.array(source['accepted_positions_rad']); actual[1] -= residual_counts*Q
    if qualified:
        np.testing.assert_allclose(following_offset(sdk, actual, following_model=m), m.bias_rad)
    else:
        with pytest.raises(ValueError, match='FOLLOWING_OUTSIDE_CONTRACT'):
            following_offset(sdk, actual, following_model=m)


@pytest.mark.parametrize('bad', [.035000001, .09, 0., -1., float('nan'), float('inf'), True])
def test_r22_residual_tolerance_cannot_be_relaxed(bad):
    with pytest.raises(ValueError, match='TOLERANCE_CANNOT_BE_RELAXED'):
        following_offset(REAL_WIRE, REAL_ENCODER, following_model=model(), max_error_rad=bad)


def test_r22_model_copies_input_and_remains_immutable():
    sample = synthetic_source(); m = source_bias_model(sample, binding_id='synthetic-copy',
                                                      source_bias_limit_rad=(.09,)*6)
    saved = m.digest
    sample['sdk_positions_rad'][0] += Q
    assert m.digest == saved and type(m.bias_rad) is tuple
    with pytest.raises(FrozenInstanceError):
        m.bias_rad = (0.,)*6
    with pytest.raises(ValueError, match='MUTABLE_VECTOR'):
        validate_source_bias_model(replace(m, bias_rad=list(m.bias_rad)))


@pytest.mark.parametrize('change', ['epoch', 'model', 'names', 'pre_source', 'same_stamp_other_value'])
def test_r22_live_sample_cannot_change_provenance_or_reuse_stamp(change):
    sample = synthetic_source(); m = model(); later = deepcopy(sample)
    if change == 'epoch': later['epoch_ns'] += 1
    elif change == 'model': later['model_sha256'] = 'f'*64
    elif change == 'names': later['joint_names'][0],later['joint_names'][1] = later['joint_names'][1],later['joint_names'][0]
    elif change == 'pre_source': later['accepted_stamp_ns'] -= 1
    else: later['accepted_positions_rad'][0] += Q
    with pytest.raises(ValueError): validate_source_bias_model(m, later)


@pytest.mark.parametrize('key,value', [
    ('epoch_ns', True), ('sdk_stamp_ns', 0), ('accepted_stamp_ns', 500_000_000),
    ('model_sha256', 'not-a-model'), ('joint_names', list(reversed(ARM_NAMES))),
    ('sdk_positions_rad', [float('nan')]*6), ('accepted_positions_rad', [float('inf')]*6),
    ('sdk_positions_rad', [True]*6), ('sdk_positions_rad', [.5*Q]*6),
    ('sdk_positions_rad', [math.pi]*6), ('accepted_positions_rad', [math.pi+Q]*6),
])
def test_r22_invalid_or_non_wire_source_is_rejected(key, value):
    sample = synthetic_source(); sample[key] = value
    with pytest.raises(ValueError): source_bias_model(sample, binding_id='synthetic-invalid',
                                                       source_bias_limit_rad=(.09,)*6)


def test_r22_source_bias_must_match_both_frozen_sources():
    m = model(); forged = list(m.bias_rad); forged[1] += Q
    with pytest.raises(ValueError, match='SOURCE_BIAS_MISMATCH'):
        validate_source_bias_model(replace(m, bias_rad=tuple(forged)))


def test_r22_distinct_next_episode_model_does_not_rewrite_old_residual_or_grant_authority():
    old = model(); old_digest = old.digest
    old_report = following_evidence(REAL_WIRE, REAL_ENCODER, following_model=old)
    # Pure construction is NOT admission: the gateway must prove normal prior
    # completion, fresh FK/CAD, ownership and a same-wire transaction separately.
    next_source = synthetic_source()
    next_source.update(sdk_positions_rad=REAL_WIRE.tolist(), accepted_positions_rad=REAL_ENCODER.tolist(),
        sdk_stamp_ns=10_990_000_000, accepted_stamp_ns=10_991_000_000, stamp_sec=11.)
    new = source_bias_model(next_source, binding_id='synthetic-unadmitted-next-episode',
                            source_bias_limit_rad=(.09,)*6)
    assert new.digest != old_digest and old.digest == old_digest
    assert following_evidence(REAL_WIRE, REAL_ENCODER, following_model=old) == old_report
    new_report = following_evidence(REAL_WIRE, REAL_ENCODER, following_model=new)
    assert not new_report['execution_authorized']
    assert old_report['physical_reference_minus_encoder_rad'][1] == pytest.approx(20*Q)


def test_r22_model_validator_is_not_a_live_lease_or_freshness_gate():
    m = model(); later = synthetic_source()
    later['sdk_stamp_ns'] += 1_000_000_000; later['accepted_stamp_ns'] += 1_000_000_000
    assert validate_source_bias_model(m, later) is m
    # This pure value object carries no action ID, task owner, deadline or CAD.
    # The independent integration contract requires those checks before motion.
    assert not hasattr(m, 'execution_authorized')


@pytest.mark.parametrize('cap', [[.090000001]*6, [.001]*6, [0.]*6,
    [float('nan')]*6, [True]*6, [.09]*5])
def test_r22_source_bias_cap_is_explicit_finite_and_cannot_be_raised(cap):
    with pytest.raises(ValueError):
        source_bias_model(synthetic_source(), binding_id='synthetic-cap', source_bias_limit_rad=cap)


def test_r22_real_pre_action_source_model_preserves_real_terminal_error():
    # Exact fields from runtime_reference_map_source_sample.json, SHA256
    # bd01db59f5d870d0b417a35c1e227d6e8ff7018fc3acb45a89d475cc42ba77a8.
    # The initial source is different from the terminal encoder sample.
    source = dict(model_sha256=MODEL_SHA, epoch_ns=1793163486501992378,
        joint_names=list(ARM_NAMES), sdk_stamp_ns=1793164115727613176,
        accepted_stamp_ns=1793164115690343736,
        sdk_positions_rad=[-2.092349794676015, -0.5491651220630596, 1.1550875332778878,
            -0.5721748338813442, -1.500233210552157, -0.39883500485026674],
        accepted_positions_rad=[-2.0831459099487004, -0.5691068723055729, 1.1382137446111458,
            -0.5645049299419159, -1.4940972874006146, -0.39269908169872414])
    m = source_bias_model(source, binding_id='r21-real-frozen-source-replay',
                          source_bias_limit_rad=(.09,)*6)
    terminal = dict(model_sha256=MODEL_SHA, epoch_ns=source['epoch_ns'],
        joint_names=list(ARM_NAMES), sdk_stamp_ns=1793164250188864444,
        accepted_stamp_ns=1793164250190299665,
        sdk_positions_rad=REAL_WIRE.tolist(), accepted_positions_rad=REAL_ENCODER.tolist())
    assert validate_source_bias_model(m, terminal) is m
    np.testing.assert_allclose(following_offset(REAL_WIRE, REAL_ENCODER, following_model=m)/Q,
        RECORDED_BIAS_COUNTS, atol=1e-12)
    e = following_evidence(REAL_WIRE, REAL_ENCODER, following_model=m)
    assert e['raw_sdk_minus_encoder_rad'][1] == pytest.approx(33*Q)
    assert e['physical_reference_minus_encoder_rad'][1] == pytest.approx(20*Q)
    assert not e['execution_authorized'] and not e['future_following_verified']


@pytest.mark.parametrize('above_last_code',[.25*Q,.5*Q,Q])
def test_r22_encoder_source_cannot_enter_upper_sdk_clipping_region(above_last_code):
    sample=synthetic_source();maximum=(4095-2048)*Q
    sample['sdk_positions_rad']=[maximum]*6
    sample['accepted_positions_rad']=[maximum+above_last_code]*6
    with pytest.raises(ValueError):
        source_bias_model(sample,binding_id='synthetic-upper-clipping',source_bias_limit_rad=(.09,)*6)


def test_r22_last_decodable_sdk_count_remains_supported():
    sample=synthetic_source();maximum=(4095-2048)*Q
    sample['sdk_positions_rad']=sample['accepted_positions_rad']=[maximum]*6
    m=source_bias_model(sample,binding_id='synthetic-last-word',source_bias_limit_rad=(.09,)*6)
    assert m.bias_rad==(0.,)*6
