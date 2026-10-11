"""Task recovery retains its old deadline through actual new phase/source join."""
from copy import deepcopy
from itertools import permutations
import json
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import observe_return_measurement as measurement
from alicia_grasp_modes.station_compute_reacquisition import build_contract
from test_station_compute_wait_task import station, configured, feedback, frame, deliver, wait
from test_observe_return_task import at_ns


DELTA = .006135923151543


def enrolled(c):
    n = c.node
    n._observation_measurement_request.update(version=1, request_id='lease:qualified',
        origin_plan_id='origin', origin_snapshot_stamp_ns=19_500_000_000,
        view_source_stamp_ns=20_000_000_000, plan_sha256='a'*64,
        stamp_ns=99_300_000_000, arrival_stamp_ns=98_900_000_000,
        capture_deadline_ns=102_000_000_000, metric_geometry_authority=False,
        motion_authorized=False)
    deliver(c, frame(c))
    assert n._station_task_record is not None
    c.now[0] = 100_100_000_000; c.mono[0] += .05
    feedback(c, 100_080_000_000, DELTA)
    assert n._station_requalification_task is not None
    assert n._station_task_record is None and n.latest_obj is None
    return n._station_requalification_task


def clock_step(c, delta):
    c.now[0] += int(delta*1e9); c.mono[0] += delta
    feedback(c, c.now[0], DELTA)


def measurement_transport(c, monkeypatch):
    """Exercise task publish/ACK, settle, finish and phase methods; no robot I/O."""
    n = c.node
    monkeypatch.setattr(n, '_observe_return_live', lambda: None)
    monkeypatch.setattr(wait.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(wait.rospy, 'sleep', lambda seconds: clock_step(c, seconds))
    phases, requests = [], []
    n.near_field_phase_pub = NS(publish=lambda phase: phases.append(deepcopy(phase)))
    def publish(msg):
        request = json.loads(msg.data)
        requests.append(request)
        sources = []
        if request['state'] == 'requalifying':
            for _ in range(3):
                clock_step(c, .1)
                sources.append(c.now[0])
                f = frame(c, c.now[0])
                # These are actual fresh objects, not a status-only substitute.
                deliver(c, f)
        for participant in measurement.PARTICIPANTS:
            ack = measurement.measurement_ack(request, participant, accepted=True,
                qualified=request['state']=='requalifying', source_stamps_ns=sources)
            n._observation_measurement_ack_cb(NS(data=json.dumps(ack)))
    n._observation_measurement_pub = NS(publish=publish)
    return phases, requests


def finalizing(c, monkeypatch):
    hold = enrolled(c)
    phases, requests = measurement_transport(c, monkeypatch)
    c.node._station_requalification_run(c.plan, {})
    assert [r['state'] for r in requests] == ['executing', 'requalifying', 'qualified']
    assert len(phases) == 1 and phases[0].phase_id == 3
    assert phases[0].deadline.to_nsec() == hold['witness']['parent_contract']['phase_deadline_ns']
    assert c.node._station_requalification_task is hold
    assert hold['finalizing_phase'] is not None and not hold['completed']
    assert c.node._station_task_record is None
    return hold


def new_station_frame(c):
    clock_step(c, .1)
    source = c.now[0]
    f = frame(c, source)
    n = c.node; phase = n._published_near_field_phase
    b = f['status']['station_compute_wait']['binding']
    b.update(lease_id=n._observation_measurement_request['lease_id'], phase_id=phase.phase_id,
        phase_started_ns=phase.header.stamp.to_nsec(),
        phase_reference_source_ns=phase.reference_source_stamp.to_nsec(),
        station_q_rad=[DELTA]+[0.]*5)
    f['status']['station_compute_wait'] = build_contract(source_ns=source,
        maximum_frame_age_sec=.8, session_deadline_ns=200_000_000_000,
        phase_deadline_ns=phase.deadline.to_nsec(), binding=b)
    clock_step(c, .02)  # Actual source bracket, not a payload joint claim.
    return f


@pytest.mark.parametrize('order', list(permutations(('object', 'mask', 'status'))))
def test_real_task_finish_and_new_phase_join_preserve_once_only_budget(station, monkeypatch, order):
    c = station; hold = finalizing(c, monkeypatch)
    n = c.node; new = new_station_frame(c)
    deliver(c, new, order)
    assert n._station_requalification_task is None and hold['completed']
    assert n._station_task_record['contract'] == new['status']['station_compute_wait']
    assert n._unknown_target_loss is None and not n._execution_authority_revoked
    assert n._station_task_record['contract']['identity_deadline_ns'] > hold['witness']['deadline_ns']
    # Normal newly measured geometry gets its ordinary source clock, while
    # another drift at this same view cannot create another recovery attempt.
    c.now[0] += 20_000_000
    feedback(c, c.now[0], 2*DELTA)
    assert n._station_task_retired and n._station_requalification_task is None


@pytest.mark.parametrize('kind', ['ros', 'monotonic'])
def test_first_registration_cannot_arrive_after_original_recovery_deadline(station, monkeypatch, kind):
    c = station; hold = finalizing(c, monkeypatch)
    if kind == 'ros':
        clock_step(c, (hold['witness']['deadline_ns']-c.now[0])/1e9)
    else:
        c.mono[0] = hold['monotonic_deadline']
    deliver(c, new_station_frame(c))
    assert c.node._station_task_record is None and c.node._station_task_retired
    assert not hold['completed']


def test_local_sdk_witness_survives_natural_history_eviction(station):
    c = station; hold = enrolled(c)
    proof = hold['local_feedback_proof']
    for _ in range(140):
        clock_step(c, .005)
    assert all(m.header.stamp.to_nsec() != proof[0] for m in c.node._following_accepted_history)
    assert c.node._station_requalification_live() is hold
    assert hold['local_feedback_proof'] == proof
    assert c.node.latest_obj is None and not hold['completed']


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'hard_loss', 'camera', 'contact'])
def test_finalizing_never_revives_a_hard_fault(station, monkeypatch, fault):
    c = station; hold = finalizing(c, monkeypatch)
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': c.node._following_epoch_ns += 1
    elif fault == 'hard_loss': c.node._unknown_target_loss = dict(reason='identity_conflict')
    elif fault == 'camera': c.params['/camera']['fx'] += 1.
    else: c.node._bound_execution_plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    c.node._station_compute_task_tick()
    assert c.node._station_task_retired and c.node._station_task_record is None
    assert not hold['completed']
