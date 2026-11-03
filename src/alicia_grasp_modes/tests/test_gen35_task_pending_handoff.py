"""Late waiting notices cannot undo completed task-owned receipt progress."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from test_station_compute_wait_task import (
    station, configured, frame, deliver, advance, feedback, at_ns, base)
from test_station_contact_admission import computed
from test_gen19_measurement_ack_clock import capture, prepared_task, schedule, publish
from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack


def historical_pending(c, contract):
    msg = base.Grasp6DPlan()
    msg.header.stamp = at_ns(contract['geometry_deadline_ns'])
    msg.header.frame_id = 'base_link'
    msg.valid = False
    msg.candidate_source = 'near_field_pending'
    msg.target_track_id = c.plan.target_track_id
    msg.diagnostic = 'TARGET_PROCESSING_PENDING: '+json.dumps(dict(
        station_compute_wait=contract, station_compute_recovery_chain=None, clock_only=True))
    return msg


def test_known_older_clock_notice_keeps_computed_contact_for_original_provenance(station):
    c = station
    plan, receipt, current = computed(c)
    parent = deepcopy(c.node._station_contact_sources[100_800_000_000]['contract'])
    before = deepcopy(c.node._station_task_record)
    c.node.grasp6d_preview_plan_cb(plan)
    c.node.grasp6d_preview_plan_cb(historical_pending(c, parent))
    assert c.node._station_contact_proposal == plan
    assert c.node._station_contact_provenance(plan).ok
    assert c.node._station_task_record == before
    assert c.node._station_contact_admitted is None
    assert receipt['motion_authorized'] is False
    assert plan.header.stamp.to_nsec() == 100_800_000_000


@pytest.mark.parametrize('fault', ['loss', 'foreign', 'false_clock', 'unknown_source', 'changed_parent'])
def test_waiting_cache_exception_never_ignores_loss_or_unbound_notice(station, fault):
    c = station
    plan, _, _ = computed(c)
    parent = deepcopy(c.node._station_contact_sources[100_800_000_000]['contract'])
    msg = historical_pending(c, parent)
    if fault == 'loss': msg.candidate_source = 'near_field_terminal'; msg.diagnostic = 'TARGET_LOST: target retired'
    else:
        payload = json.loads(msg.diagnostic.partition(':')[2])
        if fault == 'foreign': payload['station_compute_wait']['binding']['lease_id'] = 'foreign'
        elif fault == 'false_clock': payload['clock_only'] = False
        elif fault == 'unknown_source': c.node._station_contact_sources.pop(parent['source_ns'])
        elif fault == 'changed_parent': payload['station_compute_wait']['binding']['source_id'] = 'f'*64
        msg.diagnostic = 'TARGET_PROCESSING_PENDING: '+json.dumps(payload)
    c.node.grasp6d_preview_plan_cb(plan)
    c.node.grasp6d_preview_plan_cb(msg)
    assert c.node._station_contact_proposal is None
    assert c.node.latest_grasp6d_preview_plan == msg


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'geometry_deadline', 'phase_deadline', 'source_conflict'])
def test_retained_computation_cannot_bypass_a_new_real_source_or_owner_failure(station, fault):
    c = station
    plan, _, current = computed(c)
    parent = deepcopy(c.node._station_contact_sources[100_800_000_000]['contract'])
    c.node.grasp6d_preview_plan_cb(plan)
    c.node.grasp6d_preview_plan_cb(historical_pending(c, parent))
    assert c.node._station_contact_proposal == plan
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': c.node._following_epoch_ns += 1
    elif fault == 'geometry_deadline': advance(c, current['geometry_deadline_ns'])
    elif fault == 'phase_deadline': advance(c, current['phase_deadline_ns'])
    else:
        conflicting = frame(c, current['source_ns'])
        conflicting['object'].pose_base.pose.position.x += .001
        deliver(c, conflicting, ('object',))
    assert not c.node._station_contact_provenance(plan).ok
    assert c.node._station_contact_admitted is None
    assert plan.header.stamp.to_nsec() == 100_800_000_000


class ExpireOnWait:
    def __init__(self, condition, c): self.condition, self.c = condition, c
    def __enter__(self): return self.condition.__enter__()
    def __exit__(self, *args): return self.condition.__exit__(*args)
    def notify_all(self): return self.condition.notify_all()
    def wait(self, timeout):
        self.c.now[0] = self.c.request['capture_deadline_ns']
        self.c.mono[0] += 6.


def test_real_qualified_ack_then_delayed_pending_completes_same_single_request(capture):
    c = capture
    c.node._observe_return_condition = ExpireOnWait(c.node._observe_return_condition, c)
    def incoming():
        c.now[0] = c.sources[-1]+1
        c.mono[0] += .3
        for participant in PARTICIPANTS:
            complete = measurement_ack(c.request, participant, accepted=True,
                qualified=True, source_stamps_ns=c.sources)
            pending = measurement_ack(c.request, participant, accepted=True, qualified=False)
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(complete)))
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(pending)))
    schedule(c, incoming)
    result = publish(c)
    assert all(row['qualified'] and row['source_stamps_ns'] == c.sources for row in result)
    assert c.sent == [c.original_request]
    assert c.request == c.original_request


def test_real_ack_rejection_still_overrides_qualified_completion(capture):
    c = capture
    def incoming():
        c.now[0] = c.sources[-1]+1
        for participant in PARTICIPANTS:
            for accepted in (True, False):
                value = measurement_ack(c.request, participant, accepted=accepted,
                    qualified=accepted, source_stamps_ns=c.sources if accepted else [])
                c.node._observation_measurement_ack_cb(NS(data=json.dumps(value)))
    schedule(c, incoming)
    with pytest.raises(ValueError, match='ACK_REJECTED'):
        publish(c)


@pytest.mark.parametrize('fault', ['lease', 'digest', 'state', 'authority',
    'capture_deadline', 'request_deadline', 'future_completion', 'changed_source_set'])
def test_pending_retention_cannot_hide_changed_binding_or_expired_completion(capture, fault):
    c = capture
    c.node._observe_return_condition = ExpireOnWait(c.node._observe_return_condition, c)
    def incoming():
        c.now[0] = c.sources[-1]+1
        for participant in PARTICIPANTS:
            complete = measurement_ack(c.request, participant, accepted=True,
                qualified=True, source_stamps_ns=c.sources)
            pending = measurement_ack(c.request, participant, accepted=True, qualified=False)
            if fault == 'lease': pending['lease_id'] = 'foreign'
            elif fault == 'digest': pending['request_sha256'] = 'f'*64
            elif fault == 'state': pending['state'] = 'executing'
            elif fault == 'authority': pending['motion_authorized'] = True
            elif fault == 'capture_deadline': c.now[0] = c.request['capture_deadline_ns']
            elif fault == 'request_deadline': c.now[0] = c.request['deadline_ns']
            elif fault == 'future_completion': complete['source_stamps_ns'][-1] = c.now[0]+1
            elif fault == 'changed_source_set':
                pending['qualified'] = True
                pending['source_stamps_ns'] = [c.sources[0]+1, *c.sources[1:]] if participant == PARTICIPANTS[0] else c.sources
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(complete)))
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(pending)))
            assert c.node._observation_measurement_acks[(c.request['request_id'], participant)] == pending
    schedule(c, incoming)
    expected = 'SOURCE_ACK_MISMATCH' if fault == 'changed_source_set' else ''
    with pytest.raises(ValueError, match=expected):
        publish(c)
    assert c.sent == [c.original_request]
    assert c.request == c.original_request
