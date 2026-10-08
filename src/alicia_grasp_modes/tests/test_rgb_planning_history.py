"""Initial same-hold RGB evidence survives freeze, never a physical movement."""
from copy import deepcopy
import json

import numpy as np
import pytest
from std_msgs.msg import Header, String

from test_unknown_stream import module, rgb_interest_node
from test_rgb_handoff_stream import frozen_node, settled
from test_rgb_set_stream import freeze_set, ready_set


def ready_single(monkeypatch):
    n, clock, feed, _, params = rgb_interest_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    return n, clock, feed, params


def freeze(n, clock, params):
    interest = n._rgb_observation_interest
    request = dict(version=1, request_id='history-offline-test', source_id=interest['source_id'],
        selection=params['/grasp_mode/selection'], source_stamp_ns=str(interest['source_stamp_ns']),
        deadline_ns=str(int((clock[0]+100)*1e9)), max_observations=3,
        **{key:interest[key] for key in ('region_mask_sha256','source_rgb_sha256',
            'source_depth_sha256','observation_set_sha256')})
    n._rgb_reference_request_cb(String(data=json.dumps(request)))
    n._service_rgb_reference_request()
    return json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)


@pytest.mark.parametrize('multiple', [False, True])
def test_first_fresh_frame_reuses_real_initial_history_and_publishes_ready(monkeypatch, multiple):
    n, clock, feed, params = (ready_set(monkeypatch) if multiple else ready_single(monkeypatch))
    old = n._rgb_observation_interest
    old_stamps = old['rgb_track']['source_stamps_ns']
    rows = tuple(n._rgb_initial_tracking_history)
    assert len(rows) == len(set(row['source_stamp_ns'] for row in rows)) == 3
    assert [row['source_stamp_ns'] for row in rows] == list(old_stamps)
    assert all(not region.mask.flags.writeable for row in rows for region in row['regions'])
    assert freeze(n, clock, params)['status'] == 'frozen'
    receipt = n._rgb_acquisition_active['initial_rgb_history_receipt']
    assert receipt['inherited'] and receipt['source_stamps_ns'] == list(old_stamps)
    assert not n._rgb_initial_tracking_history and not n._rgb_reference_cache
    clock[0] += .2; feed()
    published = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert published['detail'] == 'frozen_reference_fresh_scene'
    assert published['observation_interest']['observation_interest_ready']
    assert published['observation_interest']['source_stamp_ns'] > old_stamps[-1]
    assert published['rgb_acquisition_handoff']['rgb_consistent_frames'] == 3
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is None and n._rgb_handoff.acquisition.samples == []


def test_history_is_bounded_and_initial_tracker_reset_discards_prior_sources(monkeypatch):
    n, clock, feed, _ = ready_single(monkeypatch)
    for _ in range(8):
        clock[0] += .6; feed()
        assert len(n._rgb_initial_tracking_history) <= 3
        assert len(n._rgb_reference_cache) <= 2
        assert [row['source_stamp_ns'] for row in n._rgb_initial_tracking_history] == list(
            n._rgb_observation_interest['rgb_track']['source_stamps_ns'])
    clock[0] += 1.2; feed()
    assert len(n._rgb_initial_tracking_history) == 1
    assert not n._rgb_reference_cache


@pytest.mark.parametrize('field', ['source_id','epoch_ns','generation','selection_stamp_ns','q_source',
    'source_stamp_ns','mask','digest','missing'])
def test_corrupted_or_incompatible_old_evidence_cannot_seed_history(monkeypatch, field):
    n, clock, feed, params = ready_single(monkeypatch)
    row = n._rgb_reference_cache[-1]
    history = list(deepcopy(row['tracking_history']))
    item = history[0]
    if field == 'source_id': item[field] = 'different-camera-model-handeye-source'
    elif field in ('epoch_ns','generation','selection_stamp_ns'): item[field] += 1
    elif field == 'q_source': item[field] = (.005,0.,0.,0.,0.,0.)
    elif field == 'source_stamp_ns':
        item[field] -= 2_000_000_000
        row['tracking_source_stamps_ns'] = tuple(h['source_stamp_ns'] for h in history)
    elif field == 'mask':
        region = item['regions'][0]
        region.mask.setflags(write=True)
        region.mask[0,0] = True
        # Retain old metadata/digest: actual pixel changes must be detected.
    elif field == 'digest': item['evidence_sha256'] = '0'*64
    elif field == 'missing': history = history[1:]
    if field not in ('mask','digest','missing'):
        item['evidence_sha256'] = n._rgb_tracking_evidence_digest(item)
    row['tracking_history'] = tuple(history)
    assert freeze(n, clock, params)['status'] == 'frozen'
    assert not n._rgb_acquisition_active['initial_rgb_history_receipt']['inherited']
    clock[0] += .2; feed()
    assert n._rgb_handoff_status['rgb_consistent_frames'] == 1
    assert not n._rgb_observation_interest['observation_interest_ready']


@pytest.mark.parametrize('change', ['epoch','selection','pose','camera_source'])
def test_freeze_current_binding_changes_never_inherit_old_evidence(monkeypatch, change):
    n, clock, feed, params = ready_single(monkeypatch)
    old = deepcopy(n._rgb_observation_interest)
    if change == 'epoch':
        n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
    elif change == 'selection':
        n.selection(String(data=json.dumps(dict(mode='unknown',generation=n.generation+1,
            stamp_ns=int(clock[0]*1e9)))))
    elif change == 'pose':
        n._rgb_interest_feedback.append((int(clock[0]*1e9), np.array([.005,0.,0.,0.,0.,0.])))
    else:
        params['/handeye']['mount_changed'] = True
    # A stale request carries its original source; it cannot rename the cache.
    n._rgb_observation_interest = old
    response = freeze(n, clock, params)
    assert response['status'] == 'rejected'
    assert n._rgb_acquisition_active is None


@pytest.mark.parametrize('multiple', [False,True])
def test_real_execution_and_settle_require_three_entirely_new_frames(monkeypatch, multiple):
    if multiple:
        n, clock, feed, _, _ = freeze_set(monkeypatch)
    else:
        n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    original = n._rgb_acquisition_active['reference']
    clock[0] += .2; feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    settled(n, clock, phase='executing')
    clock[0] += .2; feed()
    assert not n._rgb_observation_interest['observation_interest_ready']
    settled(n, clock)
    settled_ns = n._rgb_acquisition_active['settled_ns']
    for index in range(3):
        clock[0] += .2; feed()
        assert n._rgb_handoff_status['rgb_consistent_frames'] == index+1
        assert n._rgb_observation_interest['observation_interest_ready'] is (index == 2)
    assert min(n._rgb_observation_interest['rgb_track']['source_stamps_ns']) > settled_ns
    assert n._rgb_acquisition_active['reference'] is original


def test_fresh_gap_or_actual_rgb_mismatch_still_discards_inherited_confirmation(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    clock[0] += 1.2; feed()
    assert n._rgb_handoff_status['rgb_consistent_frames'] == 1
    assert not n._rgb_observation_interest['observation_interest_ready']
    original = module.propose_rgb_observation_regions
    def no_regions(*args, **kwargs):
        result = original(*args, **kwargs)
        return module.replace(result,candidates=(),selected_index=None,total_candidate_count=0,
                              reason='no_rgb_observation_region')
    monkeypatch.setattr(module,'propose_rgb_observation_regions',no_regions)
    clock[0] += .2; feed()
    assert n._rgb_handoff_status['reason'] == 'no_cross_view_rgb_match'
    assert not n._rgb_observation_interest['observation_interest_ready']


def test_fresh_feedback_failure_cannot_publish_old_ready_source(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    n._rgb_interest_feedback.clear()
    clock[0] += .2; feed(bracket=False)
    assert n._rgb_handoff_status['reason'] == 'accepted_feedback_does_not_bracket_source'
    assert not n._rgb_observation_interest['observation_interest_ready']
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_measured_out_and_back_pose_excursion_retires_inherited_history(monkeypatch):
    n, clock, feed, joint, _, _ = frozen_node(monkeypatch)
    # The arm may return before the next image. Matching the latest q alone
    # must not hide the accepted samples that already invalidated the hold.
    joint(clock[0]+.01, [.005,0.,0.,0.,0.,0.])
    joint(clock[0]+.02, [0.,0.,0.,0.,0.,0.])
    assert n._rgb_acquisition_active['initial_rgb_history_invalidated']
    clock[0] += .2; feed()
    assert n._rgb_handoff_status['rgb_consistent_frames'] == 1
    assert not n._rgb_observation_interest['observation_interest_ready']
