"""Independent frozen-checkpoint and producer-retirement timing boundaries.

The real task methods and callbacks run with the existing inert ROS fixture.
Only lock scheduling, clocks and the measurement topic transport are controlled;
these tests do not submit motion or certify the upstream collision plan.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import observe_return_task as task
from alicia_grasp_modes import station_compute_wait_task as station_task
from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
from test_station_compute_wait_task import configured, station, feedback, module
from test_gen10_handoff_authority_independent import rebound
from test_gen10_station_handoff_authority import handoff


class FeedbackAcquisition:
    """Deliver one callback before the next real feedback lock is acquired."""
    def __init__(self, original, event):
        self.original, self.event = original, event
        self.fired = False

    def __enter__(self):
        if not self.fired:
            self.fired = True
            self.event()
        return self.original.__enter__()

    def __exit__(self, *args):
        return self.original.__exit__(*args)


def original_deadlines(h):
    return {key: h[key] for key in (
        'phase_deadline_ns', 'session_deadline_ns', 'phase_mono', 'session_mono')}


@pytest.mark.parametrize('clock', ['phase_wall', 'phase_mono'])
def test_unrebound_handoff_also_rechecks_original_deadline_after_camera_rpc(
        handoff, monkeypatch, clock):
    c = handoff
    h = c.node._station_task_handoff
    before = original_deadlines(h)
    assert h['rebound'] is False
    original = station_task.rospy.get_param
    fired = []

    def get_param(name, *args):
        if name == '/camera' and not fired:
            fired.append(True)
            if clock == 'phase_wall':
                c.now[0] = h['phase_deadline_ns']
            else:
                c.mono[0] = h['phase_mono']
            feedback(c, c.now[0])
        return original(name, *args)

    monkeypatch.setattr(station_task.rospy, 'get_param', get_param)
    with pytest.raises(ValueError, match='station_handoff_expired_or_revoked'):
        c.node._station_handoff_live()
    assert fired == [True] and h['cutoff_ns'] is None
    assert original_deadlines(h) == before


@pytest.mark.parametrize('fault', [None, 'future', 'phase_wall', 'phase_mono', 'epoch'])
def test_frozen_checkpoint_reads_clock_after_feedback_lock_without_renewal(
        station, monkeypatch, fault):
    c, _, candidate = rebound(station, monkeypatch)
    h = c.node._station_task_handoff
    before = original_deadlines(h)
    states = []
    c.node.set_state = lambda *args, **kwargs: states.append(args)

    def callback():
        c.now[0] += 20_000_000
        c.mono[0] += .02
        if fault == 'phase_wall':
            c.now[0] = h['phase_deadline_ns']
        elif fault == 'phase_mono':
            c.mono[0] = h['phase_mono']
        elif fault == 'epoch':
            c.node._following_epoch_ns += 1
        feedback(c, c.now[0] + (1 if fault == 'future' else 0))

    barrier = FeedbackAcquisition(c.node._following_evidence_lock, callback)
    c.node._following_evidence_lock = barrier
    accepted = c.node._diagnostic_observation_checkpoint(
        candidate, {}, 'supplemental observation frozen')
    assert barrier.fired
    assert accepted is (fault is None), states
    assert h['cutoff_ns'] is None and original_deadlines(h) == before
    assert c.node._observation_measurement_prepared is None
    if fault is not None:
        assert any('STATION_HANDOFF_INVALID' in row[1] for row in states)
    else:
        assert not states


@pytest.mark.parametrize('fault', [None, 'phase_wall', 'phase_mono', 'epoch', 'selection'])
def test_executing_ack_logging_cannot_retire_expired_or_changed_frozen_station(
        station, monkeypatch, fault):
    c, _, candidate = rebound(station, monkeypatch)
    h = c.node._station_task_handoff
    before = original_deadlines(h)
    assert c.node._diagnostic_observation_checkpoint(
        candidate, {}, 'supplemental observation frozen')
    requests, logged = [], []

    def publish(message):
        request = json.loads(message.data)
        requests.append(deepcopy(request))
        assert h['cutoff_ns'] is None
        for participant in PARTICIPANTS:
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(
                measurement_ack(request, participant, accepted=True))))

    def logging(message, *args):
        if 'measurement acknowledged:' not in message or logged:
            return
        logged.append(True)
        assert h['cutoff_ns'] is None
        if fault == 'phase_wall':
            c.now[0] = h['phase_deadline_ns']
        elif fault == 'phase_mono':
            c.mono[0] = h['phase_mono']
        elif fault == 'epoch':
            c.node._following_epoch_ns += 1
        elif fault == 'selection':
            c.params[module.SELECTION_PARAM] = dict(
                c.selection, generation=c.selection['generation'] + 1)
        else:
            c.now[0] += 20_000_000
            c.mono[0] += .02
        feedback(c, c.now[0])

    c.node._observation_measurement_pub = NS(publish=publish)
    monkeypatch.setattr(task.rospy, 'loginfo', logging)
    if fault is None:
        prepared = c.node._observation_measurement_prepare(candidate)
        assert h['cutoff_ns'] == c.now[0] and not prepared['consumed']
        assert len(prepared['acks']) == 2
        with pytest.raises(ValueError, match='PREPARATION_ALREADY_PENDING'):
            c.node._observation_measurement_prepare(candidate)
    else:
        with pytest.raises(ValueError):
            c.node._observation_measurement_prepare(candidate)
        assert h['cutoff_ns'] is None
    assert logged == [True] and len(requests) == 1
    assert requests[0]['state'] == 'executing'
    assert requests[0]['motion_authorized'] is False
    assert requests[0]['metric_geometry_authority'] is False
    assert original_deadlines(h) == before
