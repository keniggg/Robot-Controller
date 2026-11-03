"""The doubled software window still validates raw time and source ownership."""
from copy import deepcopy
import pytest

from alicia_grasp_modes.runtime_gate_policy import SOURCE_MAX_AGE_SEC, STATION_IDENTITY_WAIT_NS
from alicia_grasp_modes.rgb_observation_source import require_fresh_source
from alicia_grasp_modes.station_compute_reacquisition import build_contract, validate_contract, validate_registration
from test_station_compute_reacquisition import contract


@pytest.mark.parametrize('age_ns', [800_000_001, 1_200_000_000, 1_600_000_000])
def test_default_source_window_accepts_the_authorized_second_interval(age_ns):
    assert require_fresh_source(10_000_000_000, 10_000_000_000+age_ns) == pytest.approx(age_ns/1e9)


@pytest.mark.parametrize('age_ns', [-1, 1_600_000_001, 2_000_000_000])
def test_source_still_rejects_future_and_expired_input(age_ns):
    with pytest.raises(ValueError):
        require_fresh_source(10_000_000_000, 10_000_000_000+age_ns)


def test_an_explicit_stricter_source_window_is_preserved():
    with pytest.raises(ValueError):
        require_fresh_source(10_000_000_000, 11_000_000_000, .8)


def test_registered_frame_and_identity_have_independent_doubled_deadlines(contract):
    fields={k:deepcopy(contract[k]) for k in ('source_ns','maximum_frame_age_sec',
        'session_deadline_ns','phase_deadline_ns','binding')}
    fields['maximum_frame_age_sec']=SOURCE_MAX_AGE_SEC
    current=build_contract(**fields);ns=current['source_ns']
    assert current['geometry_deadline_ns']==ns+1_600_000_000
    assert current['identity_deadline_ns']==ns+STATION_IDENTITY_WAIT_NS
    assert validate_registration(current,binding=current['binding'],now_ns=ns+1_000_000_000)==current
    assert validate_contract(current,binding=current['binding'],now_ns=ns+6_000_000_000)=='PENDING'
    with pytest.raises(ValueError,match='registration_too_late'):
        validate_registration(current,binding=current['binding'],now_ns=ns+1_600_000_000)
    with pytest.raises(ValueError,match='expired_or_future'):
        validate_contract(current,binding=current['binding'],now_ns=current['identity_deadline_ns'])
    assert current['motion_authorized'] is False and current['metric_geometry_authority'] is False



def _actual_window_initial(monkeypatch, maximum):
    """Real producer segmentation and both consumers; only I/O is synthetic."""
    import test_locked_rgb_hold as fixture_module
    from test_initial_preflight_remote import initial
    original = fixture_module.rgb_interest_node
    def configured(mp):
        return original(mp, maximum_frame_age_sec=maximum)
    monkeypatch.setattr(fixture_module, 'rgb_interest_node', configured)
    return initial.__wrapped__(monkeypatch)


@pytest.mark.parametrize('maximum', [.8, 1.6])
@pytest.mark.parametrize('order', [
    ('object','mask','status'), ('object','status','mask'),
    ('mask','object','status'), ('mask','status','object'),
    ('status','object','mask'), ('status','mask','object')])
def test_real_current_join_commits_inside_configured_second_window(monkeypatch, maximum, order):
    from test_gen33_initial_preflight_commit import handoff, commit_time
    from test_initial_preflight_remote import request, deliver
    s = _actual_window_initial(monkeypatch, maximum)
    lease, original, row = handoff(s)
    assert original['maximum_ns'] == round(maximum*1e9)
    identity = s.n._current_stream_target_identity()
    deliver(s.n, row, order)
    commit_time(s, original, age=maximum+.037253809)
    assert s.n._ros_source_clock_ns() >= original['source_ns']+original['maximum_ns']
    assert s.n._ros_source_clock_ns() < row['status']['source_stamp_ns']+original['maximum_ns']
    reply = request(s, 'commit')
    assert reply['accepted'], reply
    assert reply['commit_reference']['source_ns'] == row['status']['source_stamp_ns']
    assert reply['commit_reference']['admission_source_ns'] == original['source_ns']
    assert reply['commit_reference']['refreshed'] is True
    assert s.n._measurement_remote_reference == original
    assert s.n._measurement_remote_request == lease
    assert s.n._current_stream_target_identity() == identity
    assert not request(s, 'commit')['accepted']


@pytest.mark.parametrize('maximum', [.8, 1.6])
@pytest.mark.parametrize('missing', ['object', 'mask', 'status'])
def test_real_current_window_requires_complete_independent_join(monkeypatch, maximum, missing):
    from test_gen33_initial_preflight_commit import handoff, commit_time
    from test_initial_preflight_remote import request, deliver
    s = _actual_window_initial(monkeypatch, maximum)
    lease, original, row = handoff(s)
    deliver(s.n, row, tuple(k for k in ('object','mask','status') if k != missing))
    commit_time(s, original, age=maximum+.037253809)
    reply = request(s, 'commit')
    assert not reply['accepted'] and reply['reason'] == 'TARGET_MEASUREMENT_PENDING', reply
    assert s.n._measurement_remote_reference == original and s.n._measurement_remote_request == lease


@pytest.mark.parametrize('fault', ['confidence', 'depth_stamp', 'template', 'expired'])
def test_real_doubled_window_does_not_relax_source_or_quality(monkeypatch, fault):
    from test_gen33_initial_preflight_commit import handoff, commit_time
    from test_initial_preflight_remote import request, deliver, feedback
    s = _actual_window_initial(monkeypatch, 1.6)
    lease, original, row = handoff(s)
    if fault == 'confidence':
        row['object'].confidence = .01
    elif fault == 'depth_stamp':
        row['status']['depth_stamp_ns'] += 1
    elif fault == 'template':
        row['status']['initial_station_identity_template']['source_id'] = 'f'*64
    deliver(s.n, row)
    if fault == 'expired':
        s.clock[0] = (row['status']['source_stamp_ns']+1_600_000_001)/1e9
        feedback(s)
    else:
        commit_time(s, original, age=1.637253809)
    reply = request(s, 'commit')
    assert not reply['accepted'], reply
    assert reply['reason'] != 'initial_preflight_current_maximum_invalid'
    if fault == 'confidence':
        assert reply['reason'] == 'initial_preflight_current_object_confidence'
    elif fault == 'expired':
        assert reply['reason'] == 'TARGET_MEASUREMENT_PENDING'
    assert s.n._measurement_remote_reference == original and s.n._measurement_remote_request == lease


@pytest.mark.parametrize('count', [33, 64, 65])
def test_ten_second_recovery_chain_has_bounded_capacity_without_new_authority(contract, count):
    from alicia_grasp_modes.station_compute_reacquisition import validate_recovery_parent
    fields = {k:deepcopy(contract[k]) for k in (
        'maximum_frame_age_sec','session_deadline_ns','phase_deadline_ns','binding')}
    first = contract['source_ns']
    chain = [build_contract(source_ns=first+i*100_000_000, **fields) for i in range(count)]
    original = deepcopy(chain[0])
    now = chain[-1]['source_ns']+50_000_000
    kwargs = dict(parent=chain[-1], chain=chain, binding=original['binding'],
                  now_ns=now, source_ns=now-10_000_000)
    if count <= 64:
        validate_recovery_parent(original, **kwargs)
        assert original == chain[0]
        assert original['identity_deadline_ns'] == first+10_000_000_000
        assert original['motion_authorized'] is False
    else:
        with pytest.raises(ValueError, match='chain_unbound'):
            validate_recovery_parent(original, **kwargs)

