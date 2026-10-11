"""Restation wire proofs cannot revive geometry or extend registered clocks."""
from copy import deepcopy

import pytest

from alicia_grasp_modes import observe_return_measurement as measurement
from alicia_grasp_modes.station_compute_reacquisition import build_contract


DRIFT_NS = 1791688198501070048
STATION_Q = [-2.11075756, -.61819426, 1.27013609, -.17027187,
             -1.43427204, .016873788666742055]
DRIFT_Q = STATION_Q[:5] + [.0107378655151991]


@pytest.fixture
def parent():
    # Measured GEN8 drift; source geometry/digests are synthetic local records.
    source = DRIFT_NS - 300_000_000
    binding = dict(selection=dict(mode='unknown', strategy='observe_return',
        generation=8, stamp_ns=1791688120029028892), lease_id='old:observation:2',
        session_id='session', view_plan_id='521ae69d8d91f44c0d9521f0', view_index=2,
        target_epoch=4, target_track_id='g8-t4', control_epoch_ns=1791677188993077035,
        phase_id=6, phase_started_ns=source-2_000_000_000,
        phase_reference_source_ns=source-3_000_000_000,
        reference_binding='synthetic_frozen_reference', source_id='a'*64,
        camera_configuration_sha256='b'*64, config_sha256='c'*64,
        station_q_rad=list(STATION_Q))
    return build_contract(source_ns=source, maximum_frame_age_sec=.8,
        session_deadline_ns=source+100_000_000_000,
        phase_deadline_ns=source+20_000_000_000, binding=binding)


def witness(parent):
    return measurement.station_requalification_witness(parent, DRIFT_NS,
        list(DRIFT_Q), DRIFT_NS+1_000_000)


def executing(parent):
    binding = parent['binding']
    return dict(version=1, state='executing', lease_id='new:remeasure:2',
        request_id='new:remeasure:2:executing', selection=deepcopy(binding['selection']),
        session_id=binding['session_id'], target_epoch=binding['target_epoch'],
        target_track_id=binding['target_track_id'], origin_plan_id='origin',
        origin_snapshot_stamp_ns=1791688121029028892,
        view_plan_id=binding['view_plan_id'], view_source_stamp_ns=parent['source_ns'],
        view_index=2, plan_sha256='d'*64, control_epoch_ns=binding['control_epoch_ns'],
        deadline_ns=parent['session_deadline_ns'], stamp_ns=DRIFT_NS+1_000_000,
        motion_kind='observation', arrival_stamp_ns=0, capture_deadline_ns=0,
        metric_geometry_authority=False, motion_authorized=False,
        **{measurement.STATION_REQUALIFICATION: witness(parent)})


def parse(request, previous=None, now=None):
    return measurement.parse_measurement_request(request, request['selection'],
        request['stamp_ns'] if now is None else now, previous)


def test_gen8_witness_is_only_local_identity_evidence(parent):
    before = deepcopy(parent)
    value = witness(parent)
    assert measurement.validate_station_requalification(value, parent=parent,
        feedback=[(DRIFT_NS, DRIFT_Q)], now_ns=DRIFT_NS+2_000_000) == value
    assert value['deadline_ns'] == parent['identity_deadline_ns']
    assert parent == before
    result = parse(executing(parent))
    assert result['motion_authorized'] is False
    assert result['metric_geometry_authority'] is False


@pytest.mark.parametrize('feedback', [[], [(DRIFT_NS-1, DRIFT_Q)],
    [(DRIFT_NS, STATION_Q)], [(DRIFT_NS, DRIFT_Q[:5]+[.01227184630308513])]])
def test_command_or_different_local_sdk_sample_is_not_drift_proof(parent, feedback):
    with pytest.raises(ValueError, match='FEEDBACK_UNPROVEN'):
        measurement.validate_station_requalification(witness(parent), parent=parent,
            feedback=feedback, now_ns=DRIFT_NS+2_000_000)


@pytest.mark.parametrize('fault', ['source', 'phase', 'station', 'deadline'])
def test_wire_parent_cannot_replace_independently_registered_parent(parent, fault):
    value = witness(parent)
    if fault == 'source': value['parent_contract']['source_ns'] += 1
    elif fault == 'phase': value['parent_contract']['binding']['phase_id'] += 1
    elif fault == 'station': value['parent_contract']['binding']['station_q_rad'][5] += .001
    else: value['parent_contract']['identity_deadline_ns'] += 1
    with pytest.raises(ValueError, match='PARENT_UNREGISTERED'):
        measurement.validate_station_requalification(value, parent=parent,
            feedback=[(DRIFT_NS, DRIFT_Q)], now_ns=DRIFT_NS+2_000_000)


@pytest.mark.parametrize('offset', [0, 1, 1_000_000_000])
def test_fixed_identity_deadline_applies_to_proof_request_and_ack(parent, offset):
    value = witness(parent)
    request = parse(executing(parent))
    now = value['deadline_ns']+offset
    with pytest.raises(ValueError):
        measurement.validate_station_requalification(value, parent=parent,
            feedback=[(DRIFT_NS, DRIFT_Q)], now_ns=now)
    with pytest.raises(ValueError):
        parse(request, now=now)
    for participant in measurement.PARTICIPANTS:
        ack = measurement.measurement_ack(request, participant, accepted=True)
        with pytest.raises(ValueError):
            measurement.validate_measurement_ack(ack, request, participant, now)


def test_once_per_view_key_survives_new_phase_lease_and_source(parent):
    modified = deepcopy(parent)
    modified['source_ns'] += 100_000_000
    modified['binding']['phase_id'] += 1
    modified['binding']['lease_id'] = 'new_lease'
    assert measurement.station_requalification_key(modified) == measurement.station_requalification_key(parent)
    modified['binding']['view_index'] += 1
    assert measurement.station_requalification_key(modified) != measurement.station_requalification_key(parent)


@pytest.mark.parametrize('fault', ['renewed_witness', 'old_lease', 'wrong_view', 'motion',
                                 'geometry', 'return', 'renewed_session'])
def test_new_request_cannot_authorize_motion_or_change_original_ownership(parent, fault):
    request = executing(parent)
    if fault == 'renewed_witness': request[measurement.STATION_REQUALIFICATION]['deadline_ns'] += 1
    elif fault == 'old_lease': request['lease_id'] = parent['binding']['lease_id']
    elif fault == 'wrong_view': request['view_index'] += 1
    elif fault == 'motion': request['motion_authorized'] = True
    elif fault == 'geometry': request['metric_geometry_authority'] = True
    elif fault == 'return': request['motion_kind'] = 'return'
    else: request['deadline_ns'] += 1
    with pytest.raises(ValueError):
        parse(request)


def test_requalifying_requires_new_stillness_and_preserves_witness(parent):
    first = parse(executing(parent))
    later = dict(first, state='requalifying', request_id='new:remeasure:2:requalifying',
        stamp_ns=DRIFT_NS+1_200_000_000, arrival_stamp_ns=DRIFT_NS+1_100_000_000,
        capture_deadline_ns=parent['identity_deadline_ns'],
        settle=dict(epoch_ns=first['control_epoch_ns'],
            stationary_window_start_ns=DRIFT_NS+100_000_000,
            stationary_window_end_ns=DRIFT_NS+1_000_000_000))
    assert parse(later, first)[measurement.STATION_REQUALIFICATION] == witness(parent)
    changed = deepcopy(later)
    changed['settle']['stationary_window_start_ns'] = first['stamp_ns']
    with pytest.raises(ValueError, match='SETTLE_PREDATES'):
        parse(changed, first)
    changed = deepcopy(later)
    changed['capture_deadline_ns'] += 1
    with pytest.raises(ValueError):
        parse(changed, first)


@pytest.mark.parametrize('q', [STATION_Q, STATION_Q[:5]+[STATION_Q[5]+.0039],
    STATION_Q[:5]+[float('nan')], STATION_Q[:5]+[float('inf')]])
def test_no_restoration_without_finite_measured_out_of_station_drift(parent, q):
    with pytest.raises(ValueError, match='DRIFT_REQUIRED'):
        measurement.station_requalification_witness(parent, DRIFT_NS, q, DRIFT_NS+1_000_000)
