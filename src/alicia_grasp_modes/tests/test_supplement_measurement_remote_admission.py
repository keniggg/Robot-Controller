"""Actual paused-session and remote input callbacks; no live ROS or hardware."""
from copy import deepcopy
import itertools
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from std_msgs.msg import String

from test_observation_measurement_remote import context, at, request
from test_station_compute_wait_remote import station, arm, row_at
from test_unknown_pending_identity import deliver
from test_remote_modes import base, Publisher
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest


def next_session(c, monkeypatch):
    n = c.node
    previous = deepcopy(n._observe_return_session)
    # Freeze the replacement session while its old stationary source still
    # satisfies .8 seconds; the delayed new join crosses that old deadline.
    c.clock[0] += .7
    now = n._ros_source_clock_ns()
    previous.update(stamp_ns=previous['origin_snapshot_stamp_ns'],
        max_observations=2, view_request_id='initial', previous_gain_sha256='0' * 64,
        return_verified_stamp_ns=0, contact_deadline_ns=0)
    n._observe_return_session = previous
    n._observe_return_phase = 'observation_prefix'
    view = deepcopy(n._observe_return_prefix)
    view.plan_id = 'next-measured-view'
    view.header.stamp = at(now)
    view.poses = [base.make_pose_stamped('base_link', [.2, 0., .2], [0., 0., 0., 1.]).pose]
    n._observe_return_prefix = view
    session = dict(previous, view_index=2, view_plan_id=view.plan_id,
        view_source_stamp_ns=now, view_request_id='next-request',
        previous_gain_sha256='a' * 64, stamp_ns=now)
    n._observe_return_pending_supplement = {key: session[key] for key in (
        'session_id', 'view_index', 'view_request_id', 'previous_gain_sha256',
        'deadline_ns', 'max_observations', 'origin_plan_id', 'origin_snapshot_stamp_ns',
        'target_epoch', 'target_track_id')}
    n._observe_return_pending_supplement.update(previous_view_index=1,
        contact_or_motion_authorized=False)
    # Surface geometry is not used by this handoff. Preserve the exact identity
    # through the real phase transition; no planner/transport gate is replaced.
    surface = NS(identity=c.identity)
    n._observe_return_saved = (surface, NS())
    n._near_field_fused_surface = None
    n._observe_return_ack_pub = Publisher()
    monkeypatch.setattr(n, '_tool_from_camera_matrix', lambda: np.eye(4))
    n._observe_return_session_cb(String(data=json.dumps(session)))
    ack = json.loads(n._observe_return_ack_pub.messages[-1].data)
    assert ack['accepted'], ack
    assert n._observe_return_paused and n._observe_return_phase == 'observing'
    assert n._current_stream_target_identity() == c.identity
    assert n._observe_return_session['deadline_ns'] == c.session_deadline
    executing = dict(c.request, lease_id='second-lease', request_id='second-executing',
        stamp_ns=now, view_index=2, view_plan_id=view.plan_id,
        view_source_stamp_ns=view.header.stamp.to_nsec(), plan_sha256=committed_plan_digest(view))
    return session, executing


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_paused_next_session_receives_fresh_full_join_before_executing_ack(station, monkeypatch, order):
    c = station
    arm(c)
    session, executing = next_session(c, monkeypatch)
    n = c.node
    n._measurement_remote_ack_pub.messages.clear()
    n._measurement_remote_request_cb(String(data=json.dumps(executing)))
    assert not n._measurement_remote_ack_pub.messages
    assert n._measurement_remote_admission == executing
    assert not n._measurement_remote_active()
    source = executing['stamp_ns'] + 200_000_000
    c.clock[0] = (source + 10_000_000) / 1e9
    assert n._ros_source_clock_ns() - c.ready['object'].header.stamp.to_nsec() > 800_000_000
    current = row_at(c, source)
    for key in order[:-1]:
        deliver(n, current, (key,))
        assert not n._measurement_remote_ack_pub.messages
    deliver(n, current, order[-1:])
    ack = json.loads(n._measurement_remote_ack_pub.messages[-1].data)
    assert ack['accepted'] and not ack['qualified'], ack
    assert n._measurement_remote_reference['source_ns'] == source
    assert n._measurement_remote_request == executing
    assert n._observe_return_paused and not n._station_compute_scope()
    assert n._poll_stream_snapshot() is False
    assert n._current_stream_target_identity() == c.identity
    assert n._observe_return_session['deadline_ns'] == session['deadline_ns']


@pytest.mark.parametrize('fault', ['pending', 'missing_mask', 'foreign_source', 'expired_request',
    'expired_frame', 'manual', 'epoch', 'selection', 'session', 'retired'])
def test_paused_next_session_cannot_ack_without_original_join_or_owner(station, monkeypatch, fault):
    c = station
    arm(c)
    session, executing = next_session(c, monkeypatch)
    n = c.node
    n._measurement_remote_ack_pub.messages.clear()
    n._measurement_remote_request_cb(String(data=json.dumps(executing)))
    source = executing['stamp_ns'] + 200_000_000
    c.clock[0] = (source + 10_000_000) / 1e9
    current = row_at(c, source)
    if fault == 'pending':
        current['status']['state'] = 'processing_deadline_pending'
    elif fault == 'foreign_source':
        current['mask'].header.stamp = at(source - 1)
    elif fault == 'expired_request':
        c.clock[0] = (executing['stamp_ns'] + 800_000_000) / 1e9
    elif fault == 'expired_frame':
        source = executing['stamp_ns'] - 700_000_000
        for key in ('object', 'mask'):
            current[key].header.stamp = at(source)
        for key in ('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'):
            current['status'][key] = source
    elif fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch':
        n._measurement_remote_epoch_ns += 1
    elif fault == 'selection':
        c.params['/grasp_mode/selection'] = dict(n._mode_selection, generation=99)
    elif fault == 'session':
        n._observe_return_session = dict(n._observe_return_session, view_plan_id='changed')
    elif fault == 'retired':
        n._unknown_identity_retired_selection = deepcopy(n._mode_selection)
    deliver(n, current, ('object', 'status') if fault == 'missing_mask' else ('object', 'mask', 'status'))
    n._observation_measurement_poll()
    acks = [json.loads(msg.data) for msg in n._measurement_remote_ack_pub.messages]
    assert not any(ack['accepted'] for ack in acks)
    assert not n._measurement_remote_active()
    assert n._observe_return_session is None or n._observe_return_session['deadline_ns'] == session['deadline_ns']
