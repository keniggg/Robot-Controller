"""Real rebound handoff checks with deterministic RPC/feedback interleavings.

The live record did not capture the task's local clock value. These tests
reproduce the source ordering defect, not unrecorded real callback timing.
Only clocks, parameter transport and the existing offline node fixture are
controlled; no motion or ROS service is invoked.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import station_compute_wait_task as wait
from test_station_compute_wait_task import configured, station, feedback, module
from test_gen10_station_handoff_authority import handoff, rebind


@pytest.fixture
def rebound(handoff):
    c = handoff
    result, frozen = rebind(c)
    assert result.ok, (result.code, result.reason)
    h = c.node._station_task_handoff
    assert h['rebound'] is True and h['cutoff_ns'] is None
    assert frozen.plan_id == c.next_plan.plan_id
    c.deadlines = {k: h[k] for k in (
        'phase_deadline_ns', 'session_deadline_ns', 'phase_mono', 'session_mono')}
    return c


def during_camera_rpc(c, monkeypatch, event):
    original = wait.rospy.get_param
    fired = []

    def get_param(name, *args):
        if name == '/camera' and not fired:
            fired.append(True)
            event()
        return original(name, *args)

    monkeypatch.setattr(wait.rospy, 'get_param', get_param)
    return fired


def check_deadlines_unchanged(c):
    h = c.node._station_task_handoff
    assert {k: h[k] for k in c.deadlines} == c.deadlines
    assert h['cutoff_ns'] is None


def test_rebound_accepts_real_feedback_arriving_during_camera_rpc(rebound, monkeypatch):
    c = rebound
    start = c.now[0]

    def incoming():
        c.now[0] += 20_000_000
        c.mono[0] += .02
        feedback(c, c.now[0])

    fired = during_camera_rpc(c, monkeypatch, incoming)
    assert c.node._station_handoff_live() is c.node._station_task_handoff
    assert fired == [True] and c.now[0] > start
    check_deadlines_unchanged(c)


@pytest.mark.parametrize('age_ns,accepted', [
    (0, True), (799_999_999, True), (800_000_000, False),
    (800_000_001, False), (-1, False), (-20_000_000, False),
])
def test_rebound_keeps_exact_stale_and_future_feedback_boundaries(
        rebound, monkeypatch, age_ns, accepted):
    c = rebound

    def incoming():
        c.now[0] += 20_000_000
        c.mono[0] += .02
        c.node._following_accepted_history.clear()
        feedback(c, c.now[0] - age_ns)

    during_camera_rpc(c, monkeypatch, incoming)
    if accepted:
        assert c.node._station_handoff_live() is c.node._station_task_handoff
    else:
        with pytest.raises(ValueError, match='station_handoff_feedback_stale'):
            c.node._station_handoff_live()
    check_deadlines_unchanged(c)


@pytest.mark.parametrize('fault', [
    'wall_deadline', 'monotonic_deadline', 'epoch', 'revoked', 'retired',
    'owner_session', 'owner_plan', 'stopped', 'manual', 'selection',
])
def test_rebound_rechecks_original_authority_after_camera_rpc(rebound, monkeypatch, fault):
    c = rebound
    h = c.node._station_task_handoff

    def incoming():
        if fault == 'wall_deadline':
            c.now[0] = h['phase_deadline_ns']
        elif fault == 'monotonic_deadline':
            c.mono[0] = h['phase_mono']
        elif fault == 'epoch':
            c.node._following_epoch_ns += 1
        elif fault == 'revoked':
            c.node._execution_authority_revoked = True
        elif fault == 'retired':
            c.node._station_task_retired = True
        elif fault == 'owner_session':
            c.node._observe_return_session = dict(c.node._observe_return_session, view_index=99)
        elif fault == 'owner_plan':
            c.node._bound_execution_plan = deepcopy(c.node._bound_execution_plan)
            c.node._bound_execution_plan.poses[0].position.x += .001
        elif fault == 'stopped':
            c.node.active = False
        elif fault == 'manual':
            c.params['/gui/joint_direct_mode'] = True
        elif fault == 'selection':
            c.params[module.SELECTION_PARAM] = dict(c.selection, generation=c.selection['generation'] + 1)
        feedback(c, c.now[0])

    during_camera_rpc(c, monkeypatch, incoming)
    with pytest.raises(ValueError):
        c.node._station_handoff_live()
    check_deadlines_unchanged(c)


@pytest.mark.parametrize('fault', ['camera', 'pose', 'phase'])
def test_rebound_preserves_camera_pose_and_phase_checks(rebound, monkeypatch, fault):
    c = rebound
    if fault == 'phase':
        c.node._near_field_phase_id += 1

    def incoming():
        c.now[0] += 20_000_000
        c.mono[0] += .02
        if fault == 'camera':
            c.params['/camera']['fx'] += 1.
        feedback(c, c.now[0], .005 if fault == 'pose' else 0.)

    during_camera_rpc(c, monkeypatch, incoming)
    with pytest.raises(ValueError, match='station_handoff_(station|pose)_changed'):
        c.node._station_handoff_live()
    check_deadlines_unchanged(c)


@pytest.mark.parametrize('reject', [None, 'unknown', 'remote'])
def test_real_frozen_checkpoint_then_dual_ack_is_required_to_retire_station(
        station, monkeypatch, reject):
    from test_gen10_handoff_authority_independent import rebound as bind_next
    from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
    c, _, candidate = bind_next(station, monkeypatch)
    h = c.node._station_task_handoff
    deadlines = {k: h[k] for k in (
        'phase_deadline_ns', 'session_deadline_ns', 'phase_mono', 'session_mono')}

    def incoming():
        c.now[0] += 20_000_000
        c.mono[0] += .02
        feedback(c, c.now[0])

    fired = during_camera_rpc(c, monkeypatch, incoming)
    assert c.node._diagnostic_observation_checkpoint(
        candidate, {}, 'supplemental observation frozen')
    assert fired == [True] and h['cutoff_ns'] is None
    requests = []

    def publish(message):
        request = json.loads(message.data)
        requests.append(request)
        assert h['cutoff_ns'] is None
        for participant in PARTICIPANTS:
            ack = measurement_ack(request, participant, accepted=participant != reject)
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(ack)))

    c.node._observation_measurement_pub = NS(publish=publish)
    if reject is None:
        prepared = c.node._observation_measurement_prepare(candidate)
        assert len(prepared['acks']) == 2 and not prepared['consumed']
        assert h['cutoff_ns'] == c.now[0]
        with pytest.raises(ValueError, match='PREPARATION_ALREADY_PENDING'):
            c.node._observation_measurement_prepare(candidate)
    else:
        with pytest.raises(ValueError, match='ACK_REJECTED'):
            c.node._observation_measurement_prepare(candidate)
        assert h['cutoff_ns'] is None
    assert len(requests) == 1 and requests[0]['state'] == 'executing'
    assert requests[0]['motion_authorized'] is False
    assert {k: h[k] for k in deadlines} == deadlines
