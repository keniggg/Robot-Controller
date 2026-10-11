"""Qualified observation station only: multiple native pieces, one current RGB.

All pixel geometry and RGB processing are real; clocks and robot feedback use
existing offline fixtures. No initial/one-shot ambiguity rule is changed.
"""
import inspect

import numpy as np
import pytest

from test_observation_arrival_station import completed
from test_arrival_same_instance_confirmation import completed_matching_arrival, diagonal_depth, matching_frame
from test_locked_rgb_hold import custom_frame, latest_status, locked, module
from test_metric_ambiguity_hold import frame as ambiguity_frame
from test_support_measurement_hold import support_gap_depth
from alicia_grasp_modes.tabletop import _matching_candidates, _project



def split_frame(n,clock,joint):
    return matching_frame(n,clock,joint)


def inspect_matches(monkeypatch,n):
    counts=[];original=module.segment_with_support_fallback
    def segment(*args,**kwargs):
        result=original(*args,**kwargs)
        counts.append(len(_matching_candidates(result.candidates,n.tracker.anchor,n.tracker.last,n.config)))
        return result
    monkeypatch.setattr(module,'segment_with_support_fallback',segment)
    return counts


def test_qualified_station_three_normal_multimatch_frames_keep_fixed_owner_and_real_geometry(monkeypatch):
    n,clock,_,joint,params,marker=completed_matching_arrival(monkeypatch)
    counts=inspect_matches(monkeypatch,n)
    anchor=n.tracker.anchor;plane=n.tracker.plane_base.copy()
    q=marker['arrival_q'].copy();spatial=marker['spatial_reference']
    stamps=[]
    for _ in range(3):
        pair=split_frame(n,clock,joint)
        assert latest_status(n)['state']=='ready',latest_status(n)
        assert counts[-1]>1
        assert n._metric_ambiguity_pending is None
        assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
        assert marker['spatial_reference'] is spatial
        assert n.tracker.anchor is anchor
        np.testing.assert_array_equal(n.tracker.plane_base,plane)
        np.testing.assert_array_equal(marker['arrival_q'],q)
        raw=n.tracker.last;measured=raw.mask>0
        depth=n.bridge.imgmsg_to_cv2(pair[1],desired_encoding='passthrough')
        assert np.all(np.isfinite(depth[measured]) & (depth[measured]>.03))
        assert raw.metrics['valid_points']==np.count_nonzero(measured)
        K=[params['/camera'][k] for k in ('fx','fy','cx','cy')]
        np.testing.assert_allclose(raw.position_camera,np.median(_project(depth,K)[measured],axis=0),atol=1e-8,rtol=0)
        stamps.append(n.last_good_source_stamp_ns)
    assert stamps==sorted(set(stamps))


@pytest.mark.parametrize('kind',['support','metric'])
def test_existing_pending_recovers_with_same_exact_pair_and_all_original_identity_gates(monkeypatch,kind):
    n,clock,_,joint,_,marker=completed(monkeypatch)
    clock[0]+=.1
    if kind=='support':custom_frame(n,clock,joint,depth_image=support_gap_depth())
    else:ambiguity_frame(n,clock,joint)
    assert not n.tracker.lost
    assert n._locked_source_pending is not None or n._metric_ambiguity_pending is not None
    split_frame(n,clock,joint)
    assert latest_status(n)['state']=='ready',latest_status(n)
    assert n._locked_source_pending is None and n._metric_ambiguity_pending is None
    assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker


@pytest.mark.parametrize('fault',['post_rgb_stale','post_rgb_out_back','commit_stale'])
def test_multimatch_proposal_cannot_commit_after_current_or_station_invalidation(monkeypatch,fault):
    n,clock,_,joint,_,marker=completed(monkeypatch)
    oldlast=n.tracker.last;oldhold=n._locked_rgb_hold_reference;oldstamp=n.last_good_source_stamp_ns
    count=n.pubs['object'].publish.call_count
    original=module.recover_arrival_matching_fragments
    def finish(*a,**kw):
        result=original(*a,**kw)
        assert result[1] is not None
        assert n.tracker.last is oldlast
        if fault=='post_rgb_stale':clock[0]+=.81
        if fault=='post_rgb_out_back':
            joint(clock[0],[.005,0,0,0,0,0]);joint(clock[0]+.01,[0.]*6)
        return result
    monkeypatch.setattr(module,'recover_arrival_matching_fragments',finish)
    if fault=='commit_stale':
        lock=n.input_lock;stage=n._frame_timing_stage;ready=[False]
        def staged(name,*a,**kw):
            if name=='post_refine':ready[0]=True
            return stage(name,*a,**kw)
        monkeypatch.setattr(n,'_frame_timing_stage',staged)
        class DelayedLock:
            def __enter__(self):
                if ready[0] and inspect.currentframe().f_back.f_code.co_name=='process':
                    clock[0]+=.81;ready[0]=False
                return lock.__enter__()
            def __exit__(self,*a):return lock.__exit__(*a)
        n.input_lock=DelayedLock()
    split_frame(n,clock,joint)
    assert n.tracker.last is oldlast and n.last_good_source_stamp_ns==oldstamp
    assert n._locked_rgb_hold_reference is None or n._locked_rgb_hold_reference is oldhold
    assert all(not row[0][0].detected for row in n.pubs['object'].publish.call_args_list[count:])


def test_failed_proposal_preserves_original_distinct_object_ambiguity_hold_with_reason(monkeypatch):
    n,clock,_,joint,_,marker=completed(monkeypatch)
    oldlast=n.tracker.last;oldstamp=n.last_good_source_stamp_ns
    clock[0]+=.1;ambiguity_frame(n,clock,joint)
    status=latest_status(n)
    assert not n.tracker.lost and n._metric_ambiguity_pending is not None,status
    assert n.tracker.last is oldlast and n.last_good_source_stamp_ns==oldstamp
    assert not n.pubs['object'].publish.call_args[0][0].detected
    proposal=status['frame_timing']['matching_fragment_proposal']
    assert proposal['accepted'] is False and proposal['reason'].startswith('matching_fragment')
    assert proposal['metric_geometry_authority'] is False


@pytest.mark.parametrize('scope',['initial','single'])
def test_original_initial_and_single_match_paths_never_call_new_proposal(monkeypatch,scope):
    if scope=='initial':n,clock,_,joint,_=locked(monkeypatch,initial_station=True)
    else:n,clock,_,joint,_,_=completed(monkeypatch)
    calls=[]
    monkeypatch.setattr(module,'recover_arrival_matching_fragments',lambda *a,**kw:calls.append(True))
    if scope=='initial':split_frame(n,clock,joint)
    else:
        clock[0]+=.2;custom_frame(n,clock,joint,target_depth=.35)
        assert latest_status(n)['state']=='ready'
    assert not calls


def test_matching_station_full_rgb_wait_discards_slow_frame_then_fresh_pair_recovers(monkeypatch):
    from copy import deepcopy
    from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
    n,clock,_,joint,params,marker=completed_matching_arrival(monkeypatch)
    anchor=n.tracker.anchor;plane=n.tracker.plane_base.copy()
    phase=NearFieldPlanningPhase()
    phase.active=True;phase.phase_id=7;phase.planning_stage='contact_replan'
    phase.header.stamp=module.rospy.Time.now()
    phase.reference_source_stamp=module.rospy.Time.from_sec(n.last_good_source_stamp_ns*1e-9)
    phase.reference_target_track_id='target1'
    phase.deadline=module.rospy.Time.from_sec(clock[0]+30)
    n._station_compute_phase_cb(phase)
    n._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
    ready=split_frame(n,clock,joint)
    status=latest_status(n)
    assert status['state']=='ready',status
    contract=deepcopy(status['station_compute_wait'])
    assert contract['source_ns']==ready[0].header.stamp.to_nsec()
    registration=n._station_compute_registration
    mono=registration['monotonic_deadline']
    oldlast=n.tracker.last;oldhold=n._locked_rgb_hold_reference
    original=module.recover_arrival_matching_fragments
    calls=[]
    def expensive(*a,**kw):
        result=original(*a,**kw)
        assert result[1] is not None
        calls.append(result[1])
        assert n.tracker.last is oldlast
        clock[0]+=3.197677
        joint(clock[0]-.01)
        return result
    with monkeypatch.context() as change:
        change.setattr(module,'recover_arrival_matching_fragments',expensive)
        split_frame(n,clock,joint)
    assert len(calls)==1
    status=latest_status(n)
    assert status['state']=='processing_deadline_pending',status
    assert status['station_compute_wait']==contract
    assert n._station_compute_registration is registration
    assert registration['monotonic_deadline']==mono
    assert not status['target_lost'] and n.first_loss is None
    assert n.tracker.last is oldlast and n._locked_rgb_hold_reference is oldhold
    assert not n.pubs['object'].publish.call_args[0][0].detected
    fresh=split_frame(n,clock,joint)
    status=latest_status(n)
    assert status['state']=='ready',status
    assert status['station_compute_recovery']==contract
    assert status['station_compute_wait']['source_ns']==fresh[0].header.stamp.to_nsec()
    assert n.last_good_source_stamp_ns==fresh[0].header.stamp.to_nsec()
    assert (module.rospy.Time.now()-fresh[0].header.stamp).to_sec()<n.max_age
    assert n.tracker.anchor is anchor and n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    np.testing.assert_array_equal(n.tracker.plane_base,plane)
    assert n._station_compute_frame is None
