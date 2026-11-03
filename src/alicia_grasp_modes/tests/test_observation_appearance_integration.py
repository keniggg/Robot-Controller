"""Real producer/geometry lifecycle with an explicitly stubbed appearance verdict.

These tests isolate ownership, timing and stage routing. Separate recorded
entry replays and pure-module tests exercise real ORB and dense photometry.
"""
from copy import deepcopy
from dataclasses import replace
from unittest.mock import Mock
import numpy as np
import pytest

from test_observation_measurement_unknown import frozen, arrival, next_frame, send, ack
from test_locked_rgb_hold import locked, custom_frame, latest_status, module
from test_arrival_fragment_group import fragment_frame
from test_arrival_same_instance_confirmation import matching_frame
from alicia_grasp_modes import observation_appearance as appearance
from alicia_grasp_modes import arrival_dense_identity as dense
from alicia_grasp_modes import arrival_orb_proposal as orb


def force_colour_failure(monkeypatch):
    original = appearance._appearance
    def measured(rgb, mask):
        hist, contour = original(rgb, mask)
        # Current-only synthetic luminance-bin change. AB and contour remain
        # unchanged; reference extraction uses the real original function.
        return np.roll(hist, 3, axis=0), contour
    monkeypatch.setattr(appearance, '_appearance', measured)


def typed_texture_stub(monkeypatch, effect=None):
    calls = []
    def proposal(reference, image, mask, valid, source, **kwargs):
        calls.append((reference, source, kwargs['deadline_monotonic']))
        if effect is not None:
            effect()
        return None, dict(code='TEST_PROPOSAL_ONLY', appearance_match=False)
    def evaluate(reference, image, mask, valid, source, proposal, **kwargs):
        return dense.AppearanceEvidence(True, True, 'TEST_DENSE_MATCH', reference.digest,
            reference.source_stamp_ns, source,
            dict(holdout_global_ncc=.8, unique_test_pixels=300,
                 test_cell_metrics=[dict(large_diagnostic='must_not_reach_ros')]))
    monkeypatch.setattr(orb, 'propose_orb_homography', proposal)
    monkeypatch.setattr(dense, 'evaluate_appearance', evaluate)
    return calls


def complete(node, clock, request):
    pending = node._observation_measurement
    stamps = list(pending['qualified_stamps'])
    terminal = deepcopy(request)
    terminal.update(state='qualified', request_id='completed',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=stamps)
    send(node, terminal)
    assert ack(node)['qualified'] and node._observation_measurement is None
    return node._locked_rgb_hold_reference['observation_arrival_retry']


def test_original_three_frame_path_never_freezes_texture_or_proposes_h(monkeypatch):
    node, clock, _, joint, _, request = frozen(monkeypatch)
    reference = node._observation_measurement['confirmation'].appearance_reference
    monkeypatch.setattr(dense, 'freeze_reference', Mock(side_effect=AssertionError('unexpected freeze')))
    monkeypatch.setattr(orb, 'propose_orb_homography', Mock(side_effect=AssertionError('unexpected ORB')))
    request = arrival(node, clock, joint, request)
    for _ in range(3):
        next_frame(node, clock, joint)
    assert ack(node)['qualified']
    assert reference._features is None
    assert latest_status(node)['observation_measurement']['appearance_identity'][-1]['code']=='ORIGINAL_HISTOGRAM_PASS'


@pytest.mark.parametrize('frame', [next_frame, fragment_frame, matching_frame],
                         ids=['single', 'zero_match_group', 'every_matching_fragment'])
def test_typed_appearance_follows_real_three_frame_and_qualified_fragment_chain(monkeypatch, frame):
    node, clock, _, joint, _, request = frozen(monkeypatch)
    reference = node._observation_measurement['confirmation'].appearance_reference
    source = reference.source_ns
    freeze = Mock(wraps=dense.freeze_reference)
    monkeypatch.setattr(dense, 'freeze_reference', freeze)
    force_colour_failure(monkeypatch); calls = typed_texture_stub(monkeypatch)
    request = arrival(node, clock, joint, request)
    count = node.pubs['object'].publish.call_count
    for index in range(3):
        frame(node, clock, joint)
        assert node.pubs['object'].publish.call_count == count+(index==2),latest_status(node)
    assert ack(node)['qualified'], latest_status(node)
    marker = complete(node,clock,request)
    assert marker['fragment_appearance_reference'] is reference
    assert reference.source_ns == source
    assert freeze.call_count == 1
    confirmation_calls = len(calls)
    assert all(ref is reference._features and stamp>source for ref,stamp,_ in calls)
    station = marker['station_appearance']
    assert station.source_ns == marker['qualified_source_stamps_ns'][-1] > source
    assert station._features is None
    frame(node,clock,joint)
    assert latest_status(node)['state']=='ready',latest_status(node)
    assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert marker['fragment_appearance_reference'] is reference
    assert all(ref is reference._features and stamp>source
               for ref,stamp,_ in calls[:confirmation_calls])
    assert marker['station_appearance'] is station
    if frame is next_frame:
        assert len(calls)==confirmation_calls and freeze.call_count==1
    else:
        assert len(calls)>confirmation_calls and freeze.call_count==2
        assert all(ref is station._features and stamp>station.source_ns
                   for ref,stamp,_ in calls[confirmation_calls:])
    # Fragment audit must name the alternative evidence without claiming the
    # actual color distance passed or publishing large per-cell diagnostics.
    if frame is not next_frame:
        audit=node.tracker.last.metrics['observation_fragment_group']
        assert audit['appearance_identity']['code']=='TEST_DENSE_MATCH'
        assert audit['colour_distance']>.4
        assert 'test_cell_metrics' not in audit['appearance_identity']['dense_metrics']


def test_failed_appearance_preserves_current_scores_but_resets_three_frames(monkeypatch):
    node,clock,_,joint,_,request=frozen(monkeypatch)
    force_colour_failure(monkeypatch); typed_texture_stub(monkeypatch)
    request=arrival(node,clock,joint,request)
    next_frame(node,clock,joint)
    pending=node._observation_measurement
    assert len(pending['confirmation'].source_stamps_ns)==1
    positive=dense.evaluate_appearance
    def rejected(*args,**kwargs):
        return replace(positive(*args,**kwargs),appearance_match=False,code='HOLDOUT_REJECTED')
    with monkeypatch.context() as change:
        change.setattr(dense,'evaluate_appearance',rejected)
        next_frame(node,clock,joint)
    assert pending['confirmation'].source_stamps_ns==[]
    report=latest_status(node)['observation_measurement']['appearance_identity'][-1]
    assert report['code']=='HOLDOUT_REJECTED' and report['colour_distance']>.4
    assert report['reference_source_stamp_ns']<report['current_source_stamp_ns']
    assert not report['motion_authorized'] and not report['metric_geometry_authority']
    next_frame(node,clock,joint)
    assert len(pending['confirmation'].source_stamps_ns)==1 and not ack(node)['qualified']


@pytest.mark.parametrize('mutation',['source_expired','epoch','manual','deadline'])
def test_late_proposal_cannot_publish_or_qualify_after_owner_or_deadline_change(monkeypatch,mutation):
    node,clock,_,joint,params,request=frozen(monkeypatch)
    force_colour_failure(monkeypatch)
    def effect():
        if mutation=='source_expired':clock[0]+=1.
        elif mutation=='epoch':node._rgb_interest_epoch+=1
        elif mutation=='manual':params['/gui/joint_direct_mode']=True
        else:node._observation_measurement['deadline_monotonic']=0.
    typed_texture_stub(monkeypatch,effect)
    arrival(node,clock,joint,request)
    previous=node.last_good_source_stamp_ns
    count=node.pubs['object'].publish.call_count
    next_frame(node,clock,joint)
    assert node.last_good_source_stamp_ns==previous
    assert not ack(node).get('qualified')
    assert node.pubs['object'].publish.call_count==count or not node.pubs['object'].publish.call_args[0][0].detected


def test_reference_freeze_overrun_is_rejected_without_caching_or_renewing_source(monkeypatch):
    node,clock,_,joint,_,request=frozen(monkeypatch)
    ref=node._observation_measurement['confirmation'].appearance_reference
    original=dense.freeze_reference
    def late(*args):
        result=original(*args);clock[0]+=1.;return result
    monkeypatch.setattr(dense,'freeze_reference',late)
    force_colour_failure(monkeypatch);calls=typed_texture_stub(monkeypatch)
    arrival(node,clock,joint,request)
    next_frame(node,clock,joint)
    assert ref._features is None and not calls
    assert not ack(node).get('qualified')
    assert latest_status(node)['observation_measurement']['appearance_identity'][-1]['appearance_match'] is False


def test_no_bound_reference_never_uses_texture(monkeypatch):
    node,clock,_,joint,_,request=frozen(monkeypatch)
    node._observation_measurement['confirmation'].appearance_reference=None
    force_colour_failure(monkeypatch);calls=typed_texture_stub(monkeypatch)
    arrival(node,clock,joint,request);next_frame(node,clock,joint)
    assert not calls and not ack(node).get('qualified')
    assert latest_status(node)['observation_measurement']['appearance_identity'][-1]['code']=='BOUND_OBSERVATION_REFERENCE_UNAVAILABLE'


def test_snapshot_cannot_be_rebound_or_mutated(monkeypatch):
    node,_,_,_,_,_=frozen(monkeypatch)
    ref=node._observation_measurement['confirmation'].appearance_reference
    with pytest.raises(AttributeError):ref.source_ns+=1
    with pytest.raises(AttributeError):ref.bgr=ref.bgr.copy()
    for value in (ref.bgr,ref.mask,ref.valid,ref.histogram,ref.contour):
        with pytest.raises(ValueError):value.setflags(write=True)


def test_original_reference_histogram_binding_mismatch_fails_closed(monkeypatch):
    node,clock,_,joint,_,request=frozen(monkeypatch)
    confirmation=node._observation_measurement['confirmation']
    confirmation.histogram=np.roll(confirmation.histogram,1,axis=0)
    force_colour_failure(monkeypatch);calls=typed_texture_stub(monkeypatch)
    arrival(node,clock,joint,request);next_frame(node,clock,joint)
    assert not calls and not ack(node).get('qualified')
    assert latest_status(node)['observation_measurement']['appearance_identity'][-1]['code']=='REFERENCE_APPEARANCE_BINDING_MISMATCH'


def test_station_identity_reacquisition_uses_same_bound_template_after_old_geometry_expires(monkeypatch):
    from test_station_compute_wait_producer import armed,overrun
    force_colour_failure(monkeypatch);calls=typed_texture_stub(monkeypatch)
    node,clock,joint,params,marker,_=armed(monkeypatch)
    fixed=marker['fragment_appearance_reference'];origin=fixed.source_ns
    overrun(monkeypatch,node,clock,joint)
    assert latest_status(node)['state']=='processing_deadline_pending'
    clock[0]+=.2
    custom_frame(node,clock,joint,target_depth=.35)
    assert latest_status(node)['state']=='ready',latest_status(node)
    assert node._locked_rgb_hold_reference['observation_arrival_retry']['fragment_appearance_reference'] is fixed
    assert fixed.source_ns==origin and calls
    assert latest_status(node)['station_compute_recovery']['source_ns']>origin


@pytest.mark.parametrize('change',['manual','epoch','motion'])
def test_retired_station_marker_cannot_reuse_cached_texture(monkeypatch,change):
    node,clock,_,joint,params,request=frozen(monkeypatch)
    force_colour_failure(monkeypatch);calls=typed_texture_stub(monkeypatch)
    request=arrival(node,clock,joint,request)
    for _ in range(3):fragment_frame(node,clock,joint)
    marker=complete(node,clock,request);before=len(calls)
    if change=='manual':params['/gui/joint_direct_mode']=True
    elif change=='epoch':node._rgb_interest_epoch+=1
    else:marker['arrival_q']=marker['arrival_q']+.02
    fragment_frame(node,clock,joint)
    assert len(calls)==before
    assert latest_status(node)['state']!='ready'


@pytest.mark.parametrize('rejected_call',[2,3],ids=['second_native_match','final_consensus'])
def test_all_matching_fragments_must_match_no_best_of_positive_texture(monkeypatch,rejected_call):
    node,clock,_,joint,_,request=frozen(monkeypatch)
    force_colour_failure(monkeypatch);typed_texture_stub(monkeypatch)
    original=dense.evaluate_appearance;calls=[]
    def evaluate(*args,**kwargs):
        calls.append(args[4])
        result=original(*args,**kwargs)
        return replace(result,appearance_match=False,code='ONE_REQUIRED_MEMBER_REJECTED') if len(calls)==rejected_call else result
    monkeypatch.setattr(dense,'evaluate_appearance',evaluate)
    arrival(node,clock,joint,request)
    count=node.pubs['object'].publish.call_count
    matching_frame(node,clock,joint)
    assert len(calls)==rejected_call
    assert node.pubs['object'].publish.call_count==count
    assert node._observation_measurement['confirmation'].source_stamps_ns==[]
    report=latest_status(node)['observation_measurement']['appearance_identity']
    assert report[-1]['code']=='ONE_REQUIRED_MEMBER_REJECTED'
    assert report[0]['appearance_match'] is True


@pytest.mark.parametrize('change',['source_expired','manual','epoch'])
def test_executing_snapshot_creation_rechecks_fresh_source_and_owner_before_ack(monkeypatch,change):
    node,clock,_,_,params=locked(monkeypatch)
    node._observation_measurement_ack_pub=Mock()
    request=dict(version=1,state='executing',motion_kind='observation',lease_id='lease1',request_id='request1',session_id='session1',
        selection=deepcopy(params['/grasp_mode/selection']),target_epoch=1,target_track_id='target1',origin_plan_id='plan1',
        origin_snapshot_stamp_ns=node.anchor_source_stamp_ns,view_plan_id='plan2',view_source_stamp_ns=node.last_good_source_stamp_ns,
        view_index=1,plan_sha256='a'*64,control_epoch_ns=node._rgb_interest_epoch,deadline_ns=int((clock[0]+60)*1e9),
        stamp_ns=module.rospy.Time.now().to_nsec(),metric_geometry_authority=False,motion_authorized=False)
    original=module.FrozenObservationAppearance
    def snapshot(*args,**kwargs):
        result=original(*args,**kwargs)
        if change=='source_expired':clock[0]+=1.
        elif change=='manual':params['/gui/joint_direct_mode']=True
        else:node._rgb_interest_epoch+=1
        return result
    monkeypatch.setattr(module,'FrozenObservationAppearance',snapshot)
    previous=node.last_good_source_stamp_ns
    send(node,request)
    assert ack(node)['accepted'] is False
    assert getattr(node,'_observation_measurement',None) is None
    assert node.last_good_source_stamp_ns==previous


def test_late_qualified_notification_cannot_replace_current_frozen_template(monkeypatch):
    node,clock,_,joint,_,request=frozen(monkeypatch)
    force_colour_failure(monkeypatch);typed_texture_stub(monkeypatch)
    request=arrival(node,clock,joint,request)
    for _ in range(3):fragment_frame(node,clock,joint)
    stamps=list(node._observation_measurement['qualified_stamps'])
    marker=complete(node,clock,request);reference=marker['fragment_appearance_reference']
    fragment_frame(node,clock,joint)
    lastgood=node.last_good_source_stamp_ns
    old=deepcopy(request)
    old.update(state='qualified',request_id='late-completed',stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=stamps)
    receipt_count=node._observation_measurement_ack_pub.publish.call_count
    send(node,old)
    # Parser rejects this before binding a new request; it emits no new ACK.
    assert node._observation_measurement_ack_pub.publish.call_count==receipt_count
    assert node.last_good_source_stamp_ns==lastgood and not node.tracker.lost
    assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert marker['fragment_appearance_reference'] is reference
