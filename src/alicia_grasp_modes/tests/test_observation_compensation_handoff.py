"""Observation correction stays inside one motion lease, before new frames."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes import observe_return_task as task
from test_observe_return_measurement import inert_node, request


@pytest.fixture
def scenario(monkeypatch):
    node = inert_node()
    lease = request()
    node._observe_return_session = dict(phase='observing')
    node._observation_measurement_request = deepcopy(lease)
    node._observation_measurement_live = lambda req: node.events.append('live')
    node._last_motion_settle_sample = dict(epoch_ns=lease['control_epoch_ns'],
        stationary_window_start_ns=9_000_000_000, stationary_window_end_ns=9_900_000_000)
    node.set_state = lambda *a: None
    now = [10_000_000_000]
    pose = NS(position=NS(x=.1, y=.2, z=.3), orientation=NS(x=0., y=0., z=0., w=1.))
    plan = NS(plan_id='view', poses=[pose])
    monkeypatch.setattr(task, 'committed_plan_digest', lambda plan: lease['plan_sha256'])
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: NS(to_nsec=lambda: now[0])))
    monkeypatch.setattr(task.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(task.rospy, 'loginfo', lambda *a, **k: None)
    response = dict(code='OBSERVATION_ENDPOINT_READY', plan_id='view',
        plan_digest=lease['plan_sha256'], fixed_goal_pose=[.1,.2,.3,0.,0.,0.,1.],
        control_epoch_ns=lease['control_epoch_ns'], session_id=lease['session_id'],
        lease_id=lease['lease_id'], view_index=lease['view_index'],
        physical_motion_submitted=False, real_grasp_success=False,
        completion_stamp_ns=10_100_000_000, accepted_stamp_ns=10_050_000_000,
        view_audit=dict(minimum_edge_margin_px=31.14, roi_margin_px=24.))
    behavior = dict(success=True, mutate=None, settled=True, old_settle=False)

    def invoke(request_plan, execute):
        assert request_plan.plan_id == 'view' and execute is True
        assert node._observation_measurement_request['state'] == 'executing'
        node.events.append('compensation_service')
        now[0] = 10_200_000_000
        if behavior['mutate']:
            behavior['mutate']()
        return NS(success=behavior['success'], message=json.dumps(response))

    monkeypatch.setattr(task.rospy, 'ServiceProxy', lambda name, typ: invoke)
    node._invoke_observation_compensation_action = lambda plan, cfg, action, **kw: (NS(ok=True), action())

    def settle(reason):
        node.events.append('correction_settle')
        if not behavior['old_settle']:
            node._last_motion_settle_sample = dict(epoch_ns=lease['control_epoch_ns'],
                stationary_window_start_ns=10_200_000_000, stationary_window_end_ns=10_600_000_000)
        now[0] = 10_600_000_000
        return behavior['settled']

    node._wait_for_motion_settle = settle
    return NS(node=node, plan=plan, response=response, behavior=behavior, now=now,
              cfg=dict(observation_endpoint_compensation_enabled=True))


def run(s):
    def strict(*args):
        s.node.events.append('strict_motion')
        return NS(success=True)
    return s.node._plan_and_execute_pose(1, 'view', object(), lambda *a: None, 'view',
        execution_plan=s.plan, execute_pose=strict, gcfg=s.cfg)


def test_in_margin_receipt_never_adds_a_motion_or_skips_new_frames(scenario):
    s = scenario
    assert run(s)
    assert 'correction_settle' not in s.node.events
    assert s.node.events.index('original_settle') < s.node.events.index('compensation_service')
    assert s.node.events.index('compensation_service') < s.node.events.index('fresh_original_multiframe')
    assert s.node._observation_measurement_request['state'] == 'executing'
    assert s.node._observation_measurement_request['deadline_ns'] == request()['deadline_ns']


def test_corrected_view_requalifies_only_after_new_motion_settle(scenario):
    s = scenario
    s.response.update(physical_motion_submitted=True, accepted_stamp_ns=10_150_000_000)
    assert run(s)
    assert s.node.events.index('compensation_service') < s.node.events.index('correction_settle')
    assert s.node.events.index('correction_settle') < s.node.events.index('fresh_original_multiframe')
    assert s.node._last_motion_settle_sample['stationary_window_start_ns'] > s.response['completion_stamp_ns']


@pytest.mark.parametrize('field,value', [
    ('code', 'PREGRASP_MEASURED_CONVERGED'), ('plan_id', 'old'),
    ('plan_digest', 'b'*64), ('control_epoch_ns', 1), ('session_id', 'old'),
    ('lease_id', 'old'), ('view_index', 2), ('fixed_goal_pose', [.2,.2,.3,0,0,0,1]),
    ('physical_motion_submitted', 'false'), ('real_grasp_success', True),
    ('accepted_stamp_ns', 9_600_000_000), ('accepted_stamp_ns', 10_300_000_000),
    ('completion_stamp_ns', 9_900_000_000), ('completion_stamp_ns', 10_300_000_000),
    ('view_audit', None),
])
def test_wrong_or_stale_compensation_receipt_cannot_release_station(scenario, field, value):
    s = scenario
    s.response[field] = value
    assert run(s) is False
    assert 'revoke' in s.node.events and 'fresh_original_multiframe' not in s.node.events


@pytest.mark.parametrize('fault', ['rpc_failure', 'expired', 'lease_changed', 'old_settle', 'not_settled'])
def test_failed_or_unsettled_compensation_cannot_continue(scenario, fault):
    s = scenario
    if fault == 'rpc_failure':
        s.behavior['success'] = False
    elif fault == 'expired':
        s.behavior['mutate'] = lambda: s.now.__setitem__(0, request()['deadline_ns'])
    elif fault == 'lease_changed':
        s.behavior['mutate'] = lambda: s.node._observation_measurement_request.update(lease_id='new')
    else:
        s.response.update(physical_motion_submitted=True, accepted_stamp_ns=10_150_000_000)
        s.behavior['old_settle' if fault == 'old_settle' else 'settled'] = (fault == 'old_settle')
    assert run(s) is False
    assert 'revoke' in s.node.events and 'fresh_original_multiframe' not in s.node.events


@pytest.mark.parametrize('phase', ['returning', 'finished'])
def test_origin_return_does_not_use_observation_pose_compensation(scenario, phase):
    s = scenario
    s.node._observe_return_session['phase'] = phase
    assert run(s)
    assert 'compensation_service' not in s.node.events


def test_failed_original_motion_never_retries_through_compensation(scenario):
    s = scenario
    s.node.backend_result = False
    assert run(s) is False
    assert 'compensation_service' not in s.node.events


def test_revoked_plan_cannot_submit_correction(scenario):
    s = scenario
    s.node._invoke_observation_compensation_action = lambda *a, **kw: (NS(ok=False, code='TARGET_LOST', reason='revoked'), None)
    assert run(s) is False
    assert 'compensation_service' not in s.node.events
