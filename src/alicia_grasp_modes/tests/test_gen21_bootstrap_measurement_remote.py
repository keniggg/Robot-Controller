"""Pre-session bootstrap receipts through real consumer callbacks, without ROS."""
from copy import deepcopy
import itertools
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from std_msgs.msg import Header, String

from alicia_grasp_modes import observe_return_measurement as contract
from alicia_grasp_modes import observe_return_measurement_remote as remote
from test_observation_measurement_remote import (
    context, at, request, arrival, fresh, deliver_one, qualified_ack)
from test_remote_modes import base, Publisher
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest


@pytest.fixture
def bootstrap(context):
    c = context
    n = c.node
    n._observe_return_session = None
    n._observe_return_phase = 'observation_prefix'
    n._observe_return_paused = False
    n.latest_rich_plan = deepcopy(n._observe_return_prefix)
    n._promotion_controller().execution_plan_id = n.latest_rich_plan.plan_id
    n.execution_plan_validity_sec = 120.
    deadline = c.request['view_source_stamp_ns'] + 120_000_000_000
    c.request.update(motion_kind=contract.ACTUATION_BOOTSTRAP,
        session_id='bootstrap:one', deadline_ns=deadline,
        actuation_bootstrap=dict(version=1, source_deadline_ns=deadline))
    return c


def test_bootstrap_ack_precedes_sync_and_pauses_worker_without_origin_or_geometry(bootstrap):
    c = bootstrap
    n = c.node
    old = deepcopy(n.latest_object)
    count = len(n.geometry_pub.messages)
    assert request(n, c.request)['accepted']
    assert n._observe_return_session is None
    assert n._observe_return_phase == 'observation_prefix' and n._observe_return_paused
    assert n._poll_stream_snapshot() is False
    assert n._stream_ticket_cancellation_code_locked(None) == 'OBSERVE_RETURN_PHASE_PAUSED'
    c.clock[0] += 4.
    assert n._observation_measurement_poll() is True
    assert n.latest_object == old and len(n.geometry_pub.messages) == count
    assert n._current_stream_target_identity() == c.identity
    assert n._measurement_remote_request['deadline_ns'] == c.request['deadline_ns']


def test_completed_bootstrap_enters_real_session_callback_then_first_view_measurement(bootstrap, monkeypatch):
    c = bootstrap
    n = c.node
    for name in ('_observe_return_prefix', '_observe_return_origin_prefix', 'latest_rich_plan'):
        getattr(n, name).poses = [base.make_pose_stamped('base_link', [.2, 0., .2], [0., 0., 0., 1.]).pose]
    c.request['plan_sha256'] = committed_plan_digest(n.latest_rich_plan)
    req = arrival(c)
    row, sources = fresh(c, req)
    for key in ('object', 'mask', 'status', 'ack'):
        deliver_one(c, row, key)
    assert qualified_ack(c)[-1]['source_stamps_ns'] == sources
    c.clock[0] += .01
    final = dict(req, state='qualified', request_id='complete', stamp_ns=n._ros_source_clock_ns(),
                 source_stamps_ns=sources)
    assert request(n, final)['accepted']
    surface = NS(identity=c.identity)
    n._observe_return_saved = (surface, NS())
    n._near_field_fused_surface = None
    n._observe_return_ack_pub = Publisher()
    monkeypatch.setattr(n, '_tool_from_camera_matrix', lambda: np.eye(4))
    c.clock[0] += .01
    session = {k: deepcopy(final[k]) for k in (
        'selection', 'target_epoch', 'target_track_id', 'origin_plan_id',
        'origin_snapshot_stamp_ns', 'view_plan_id', 'view_source_stamp_ns', 'view_index')}
    session.update(version=2, session_id='actual-origin-session', phase='observing',
        stamp_ns=n._ros_source_clock_ns(), deadline_ns=n._ros_source_clock_ns()+60_000_000_000,
        max_observations=3, return_verified_stamp_ns=0, contact_deadline_ns=0,
        view_request_id='initial:'+session['view_plan_id'], previous_gain_sha256='0'*64)
    n._observe_return_session_cb(String(data=json.dumps(session)))
    ack = json.loads(n._observe_return_ack_pub.messages[-1].data)
    assert ack['accepted'], ack
    assert n._observe_return_phase == 'observing' and n._observe_return_paused
    assert len(n._observe_return_visited_camera_origins) == 1
    assert n._current_stream_target_identity() == c.identity
    assert n._observe_return_saved[0] is surface
    c.clock[0] += .01
    ordinary = dict(c.request, motion_kind='observation', session_id=session['session_id'],
        deadline_ns=session['deadline_ns'], lease_id='normal-view', request_id='normal-executing',
        stamp_ns=n._ros_source_clock_ns())
    ordinary.pop('actuation_bootstrap')
    ack = request(n, ordinary)
    assert ack['accepted'], ack
    assert n._measurement_remote_request['motion_kind'] == 'observation'


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status', 'ack'))))
def test_bootstrap_requires_three_frame_join_and_stays_paused_until_formal_session(bootstrap, order):
    c = bootstrap
    n = c.node
    req = arrival(c)
    row, sources = fresh(c, req)
    for key in order[:-1]:
        deliver_one(c, row, key)
        assert not qualified_ack(c)
    deliver_one(c, row, order[-1])
    assert qualified_ack(c)[-1]['source_stamps_ns'] == sources
    c.clock[0] += .01
    final = dict(req, state='qualified', request_id='complete', stamp_ns=n._ros_source_clock_ns(),
                 source_stamps_ns=sources)
    assert request(n, final)['accepted']
    assert not n._measurement_remote_active()
    assert n._observe_return_session is None and n._observe_return_paused
    assert n._current_stream_target_identity() == c.identity
    assert n.latest_object.header.stamp.to_nsec() == sources[-1]
    assert n._poll_stream_snapshot() is False
    # A new ordinary lease belongs to the real session, not the bootstrap id.
    session = {k: deepcopy(final[k]) for k in (
        'selection', 'target_epoch', 'target_track_id', 'origin_plan_id',
        'origin_snapshot_stamp_ns', 'view_plan_id', 'view_source_stamp_ns', 'view_index')}
    session.update(version=2, session_id='actual-gateway-origin', phase='observing',
                   deadline_ns=n._ros_source_clock_ns() + 60_000_000_000)
    n._observe_return_session = session
    n._observe_return_phase = 'observing'
    c.clock[0] += .01
    ordinary = dict(c.request, motion_kind='observation', session_id=session['session_id'],
        deadline_ns=session['deadline_ns'], lease_id='view-one', request_id='view-executing',
        stamp_ns=n._ros_source_clock_ns())
    ordinary.pop('actuation_bootstrap')
    ack = request(n, ordinary)
    assert ack['accepted'], ack
    assert n._measurement_remote_request['motion_kind'] == 'observation'


@pytest.mark.parametrize('fault', ['kind', 'metadata', 'namespace', 'view_index', 'origin_plan',
    'origin_source', 'deadline', 'restation', 'version_bool', 'null_on_ordinary'])
def test_bootstrap_wire_contract_cannot_borrow_a_session_or_extend_source(bootstrap, fault):
    c = bootstrap
    r = deepcopy(c.request)
    if fault == 'kind': r['motion_kind'] = 'observation'
    elif fault == 'metadata': r.pop('actuation_bootstrap')
    elif fault == 'namespace': r['session_id'] = 'real-session'
    elif fault == 'view_index': r['view_index'] = 2
    elif fault == 'origin_plan': r['origin_plan_id'] = 'different'
    elif fault == 'origin_source': r['origin_snapshot_stamp_ns'] -= 1
    elif fault == 'deadline': r['actuation_bootstrap']['source_deadline_ns'] += 1
    elif fault == 'restation': r[contract.STATION_REQUALIFICATION] = {}
    elif fault == 'version_bool': r['actuation_bootstrap']['version'] = True
    else: r.update(motion_kind='observation', actuation_bootstrap=None)
    with pytest.raises(ValueError):
        contract.parse_measurement_request(r, c.request['selection'], r['stamp_ns'])


@pytest.mark.parametrize('fault', ['session', 'phase', 'prefix', 'execution', 'digest', 'controller',
    'epoch', 'manual', 'selection', 'retired', 'pending', 'deadline_extended', 'source_old'])
def test_bootstrap_admission_requires_local_execution_and_current_identity(bootstrap, fault):
    c = bootstrap
    n = c.node
    r = deepcopy(c.request)
    if fault == 'session': n._observe_return_session = {'version': 2}
    elif fault == 'phase': n._observe_return_phase = 'initial_contact'
    elif fault == 'prefix': n._observe_return_prefix = None
    elif fault == 'execution': n.latest_rich_plan = None
    elif fault == 'digest': n.latest_rich_plan.header.frame_id = 'changed'
    elif fault == 'controller': n._promotion_controller().clear_execution()
    elif fault == 'epoch': n._measurement_remote_epoch_ns += 1
    elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'selection': c.params['/grasp_mode/selection'] = dict(n._mode_selection, generation=99)
    elif fault == 'retired': n._unknown_identity_retired_selection = deepcopy(n._mode_selection)
    elif fault == 'pending': n._unknown_identity_pending = {'pending': True}
    elif fault == 'deadline_extended':
        r['deadline_ns'] += 1
        r['actuation_bootstrap']['source_deadline_ns'] += 1
    else: c.clock[0] += 1.
    assert not request(n, r)['accepted']
    assert not n._measurement_remote_active()


@pytest.mark.parametrize('fault', ['object_false', 'producer_lost', 'epoch', 'expiry', 'wall_expiry'])
def test_true_loss_revokes_the_gateway_plan_even_while_task_execution_is_frozen(bootstrap, monkeypatch, fault):
    c = bootstrap
    n = c.node
    n.robot_execution_active = True
    assert request(n, c.request)['accepted']
    before = len(n.rich_plan_pub.messages)
    if fault == 'object_false':
        obj = deepcopy(c.ready['object']); obj.detected = False; n.object_cb(obj)
    elif fault == 'producer_lost':
        status = deepcopy(c.ready['status']); status['target_lost'] = True
        n._silhouette_source_status_cb(String(data=json.dumps(status)))
    elif fault == 'epoch':
        n._measurement_remote_epoch_cb(Header(stamp=at(c.request['control_epoch_ns'] + 1),
                                              frame_id='sdk_reference_epoch'))
    elif fault == 'expiry':
        c.clock[0] = c.request['deadline_ns'] / 1e9 + .001; n._observation_measurement_poll()
    else:
        deadline = n._measurement_bootstrap_deadline_mono
        monkeypatch.setattr(remote.time, 'monotonic', lambda: deadline)
        n._observation_measurement_poll()
    assert n._unknown_identity_retired_selection == n._mode_selection
    assert not n._promotion_controller().has_execution
    assert len(n.rich_plan_pub.messages) > before
    assert n.rich_plan_pub.messages[-1].valid is False
    assert not n._measurement_remote_active()
