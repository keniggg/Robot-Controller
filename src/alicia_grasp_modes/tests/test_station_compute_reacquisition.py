"""Clock and binding faults for identity-only waiting; no robot authority."""
from copy import deepcopy

import pytest

from alicia_grasp_modes.station_compute_reacquisition import (
    build_contract, validate_contract, validate_registration, validate_recovery_parent)


@pytest.fixture
def contract():
    # gen31 actual previous-good and failing source times. The geometry and
    # identity bindings are deliberately synthetic and self-contained.
    source = 1791552437092216491
    binding = dict(selection=dict(mode='unknown', strategy='observe_return',
        generation=31, stamp_ns=1791552361682603597), lease_id='session:observation:2',
        session_id='session', view_plan_id='plan', view_index=2, target_epoch=4,
        target_track_id='g1-t4', control_epoch_ns=1791506524580754980,
        phase_id=2, phase_started_ns=1791552426268758535,
        phase_reference_source_ns=1791552424760397672,
        reference_binding='{"frozen_reference":"synthetic"}', source_id='a'*64,
        camera_configuration_sha256='b'*64, config_sha256='c'*64,
        station_q_rad=[0., .1, -.2, .3, -.4, .5])
    return build_contract(source_ns=source, maximum_frame_age_sec=.8,
        session_deadline_ns=1791552971798227548,
        phase_deadline_ns=1791552516268740177, binding=binding)


def test_gen31_late_calculation_is_discarded_but_new_fresh_measurement_may_recover(contract):
    reference = contract['source_ns']
    registered = validate_registration(contract, binding=contract['binding'], now_ns=reference+50_000_000)
    failed_source = 1791552437453025102
    failed_now = 1791552440750702381
    assert validate_contract(registered, binding=contract['binding'], now_ns=failed_now) == 'PENDING'
    with pytest.raises(ValueError, match='recovery_source_not_fresh'):
        validate_contract(registered, binding=contract['binding'], now_ns=failed_now, source_ns=failed_source)
    assert validate_contract(registered, binding=contract['binding'], now_ns=failed_now+200_000_000,
        source_ns=failed_now+100_000_000) == 'PENDING'
    assert registered == contract
    assert all(contract[k] is False for k in ('metric_geometry_authority', 'motion_authorized', 'reference_refreshed'))


@pytest.mark.parametrize('offset', [0, 1, 1_000_000_000])
def test_late_receipt_cannot_register_or_renew_identity(contract, offset):
    with pytest.raises(ValueError, match='registration_too_late'):
        validate_registration(contract, binding=contract['binding'],
            now_ns=contract['geometry_deadline_ns']+offset)


@pytest.mark.parametrize('key', ['phase_deadline_ns', 'session_deadline_ns'])
def test_original_owner_deadline_caps_wait_and_is_not_extended(contract, key):
    kwargs = {k: contract[k] for k in ('source_ns', 'maximum_frame_age_sec',
        'session_deadline_ns', 'phase_deadline_ns', 'binding')}
    kwargs[key] = contract['source_ns']+300_000_000
    bounded = build_contract(**kwargs)
    assert bounded['identity_deadline_ns'] == kwargs[key]
    assert bounded['geometry_deadline_ns'] == contract['geometry_deadline_ns']
    assert validate_contract(bounded, binding=bounded['binding'], now_ns=kwargs[key]-1) == 'ARMED'
    with pytest.raises(ValueError, match='expired_or_future'):
        validate_contract(bounded, binding=bounded['binding'], now_ns=kwargs[key])


@pytest.mark.parametrize('offset', [0, 1, 50_000_000_000])
def test_five_second_identity_expiry_is_absolute_even_with_fresh_input(contract, offset):
    now = contract['source_ns']+5_000_000_000+offset
    with pytest.raises(ValueError, match='expired_or_future'):
        validate_contract(contract, binding=contract['binding'], now_ns=now, source_ns=now-10_000_000)


@pytest.mark.parametrize('key', ['lease_id', 'session_id', 'view_plan_id', 'view_index',
    'target_epoch', 'target_track_id', 'control_epoch_ns', 'phase_id', 'phase_started_ns',
    'phase_reference_source_ns', 'reference_binding', 'source_id',
    'camera_configuration_sha256', 'config_sha256', 'station_q_rad', 'selection'])
def test_changed_station_or_target_context_cannot_use_old_contract(contract, key):
    binding = deepcopy(contract['binding'])
    if key == 'selection':
        binding[key]['generation'] += 1
    elif key == 'station_q_rad':
        binding[key][0] += .001
    elif type(binding[key]) is int:
        binding[key] += 1
    elif key in ('source_id', 'camera_configuration_sha256', 'config_sha256'):
        binding[key] = 'd'*64
    else:
        binding[key] += '-changed'
    with pytest.raises(ValueError, match='contract_changed'):
        validate_contract(contract, binding=binding, now_ns=contract['source_ns']+1)


@pytest.mark.parametrize('key,value', [('version', True), ('metric_geometry_authority', 0),
    ('motion_authorized', True), ('reference_refreshed', True),
    ('geometry_deadline_ns', 1), ('identity_deadline_ns', 9999999999999999999)])
def test_forged_clock_or_authority_rejected(contract, key, value):
    altered = deepcopy(contract); altered[key] = value
    with pytest.raises(ValueError):
        validate_contract(altered, binding=contract['binding'], now_ns=contract['source_ns']+1)


@pytest.mark.parametrize('relative', [-1, 0, 1_000_000_001])
def test_recovery_must_be_new_and_not_future(contract, relative):
    now = contract['source_ns']+1_000_000_000
    with pytest.raises(ValueError, match='recovery_source_not_fresh'):
        validate_contract(contract, binding=contract['binding'], now_ns=now,
            source_ns=contract['source_ns']+relative)


def test_source_at_original_freshness_boundary_still_rejected(contract):
    now = contract['source_ns']+1_000_000_000
    with pytest.raises(ValueError, match='recovery_source_not_fresh'):
        validate_contract(contract, binding=contract['binding'], now_ns=now,
            source_ns=now-800_000_000)


@pytest.mark.parametrize('maximum', [True, 0., -.1, .80000001, float('nan'), float('inf')])
def test_identity_wait_cannot_widen_current_frame_age(contract, maximum):
    kwargs = {k: contract[k] for k in ('source_ns', 'maximum_frame_age_sec',
        'session_deadline_ns', 'phase_deadline_ns', 'binding')}
    kwargs['maximum_frame_age_sec'] = maximum
    with pytest.raises(ValueError, match='invalid_frame_age'):
        build_contract(**kwargs)


def test_caller_mutation_cannot_rewrite_registered_contract(contract):
    registered = validate_registration(contract, binding=contract['binding'], now_ns=contract['source_ns']+1)
    contract['binding']['station_q_rad'][0] = 99.
    assert registered['binding']['station_q_rad'][0] == 0.
    assert validate_contract(registered, binding=registered['binding'], now_ns=registered['source_ns']+1) == 'ARMED'


@pytest.mark.parametrize('malformed', [None, [], {}, {'source_ns': True}])
def test_malformed_receipt_has_no_wait_authority(contract, malformed):
    with pytest.raises(ValueError):
        validate_contract(malformed, binding=contract['binding'], now_ns=contract['source_ns']+1)


def later_contract(value, offset):
    fields = {k: deepcopy(value[k]) for k in ('source_ns', 'maximum_frame_age_sec',
        'session_deadline_ns', 'phase_deadline_ns', 'binding')}
    fields['source_ns'] += offset
    return build_contract(**fields)


def test_missed_ready_is_identity_bridge_without_deadline_or_geometry_renewal(contract):
    before = deepcopy(contract)
    intermediate = later_contract(contract, 400_000_000)
    now = contract['source_ns']+3_500_000_000
    for source in (None, now-50_000_000):
        validate_recovery_parent(contract, parent=intermediate, chain=[contract, intermediate],
            binding=contract['binding'], now_ns=now, source_ns=source)
    assert contract == before
    assert contract['identity_deadline_ns'] < intermediate['identity_deadline_ns']
    assert contract['metric_geometry_authority'] is False


def test_newer_parent_cannot_extend_original_consumer_wait(contract):
    parent = later_contract(contract, 400_000_000)
    now = contract['identity_deadline_ns']+1
    assert validate_contract(parent, binding=parent['binding'], now_ns=now) == 'PENDING'
    with pytest.raises(ValueError, match='expired_or_future'):
        validate_recovery_parent(contract, parent=parent, chain=[contract, parent],
            binding=contract['binding'], now_ns=now, source_ns=now-1)


@pytest.mark.parametrize('fault', ['missing_pin', 'duplicate', 'reverse', 'wrong_tail',
    'changed_owner', 'changed_age', 'authority', 'oversized', 'missing_chain'])
def test_invalid_history_cannot_replace_registered_parent(contract, fault):
    parent = later_contract(contract, 400_000_000)
    chain = [deepcopy(contract), deepcopy(parent)]
    if fault == 'missing_pin': chain = [parent]
    if fault == 'duplicate': chain.insert(0, deepcopy(contract))
    if fault == 'reverse': chain = [parent, contract, parent]
    if fault == 'wrong_tail': chain = [parent, contract]
    if fault == 'changed_owner': chain[0]['binding']['phase_id'] += 1
    if fault == 'changed_age': chain.insert(1, later_contract(contract, 200_000_000)); chain[1]['maximum_frame_age_sec'] = .6
    if fault == 'authority': chain[0]['motion_authorized'] = True
    if fault == 'oversized': chain = [deepcopy(contract)]*32+[parent]
    if fault == 'missing_chain': chain = None
    with pytest.raises(ValueError):
        validate_recovery_parent(contract, parent=parent, chain=chain, binding=contract['binding'],
            now_ns=contract['source_ns']+1_000_000_000, source_ns=contract['source_ns']+900_000_000)


def test_direct_parent_does_not_require_historical_bridge(contract):
    validate_recovery_parent(contract, parent=deepcopy(contract), binding=contract['binding'],
        now_ns=contract['source_ns']+1_000_000_000, source_ns=contract['source_ns']+900_000_000)


def test_recovery_cannot_precede_its_actual_parent(contract):
    parent = later_contract(contract, 400_000_000)
    with pytest.raises(ValueError, match='parent_not_before_source'):
        validate_recovery_parent(contract, parent=parent, chain=[contract, parent],
            binding=contract['binding'], now_ns=contract['source_ns']+500_000_000,
            source_ns=parent['source_ns'])
