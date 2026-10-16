"""Scheduling facts retain all current candidates and grant no path authority."""
from copy import deepcopy
import pytest
from alicia_grasp_modes.observation_search_progress import SearchProgress, family_key


def bound():
    p=SearchProgress(); token=p.bind({'station':'a','geometry_reference':'g'},remaining_sec=90.,monotonic_now=10.)
    return p,token


def test_fresh_request_preserves_count_and_retains_failed_family():
    p,t=bound();p.start(t,'a');p.finish(t,'a',elapsed_sec=1.63,code='OBSERVATION_PLANNING_TIMEOUT')
    assert p.bind({'station':'a','geometry_reference':'g'},remaining_sec=80.,monotonic_now=20.)==t
    assert p.order([('a',True,0,'old'),('b',False,1,'new')])==('new','old')
    assert p.families['a']==dict(attempts=1,timeouts=1,last_code='OBSERVATION_PLANNING_TIMEOUT')


@pytest.mark.parametrize('changed',['station','geometry_reference','model','camera','control_epoch','phase','session'])
def test_owner_change_clears_all_facts_and_late_result_is_ignored(changed):
    p,t=bound();p.start(t,'a')
    owner={'station':'a','geometry_reference':'g',changed:'changed'}
    new=p.bind(owner,remaining_sec=70.,monotonic_now=30.)
    assert new!=t and p.count('a')==0
    assert not p.finish(t,'a',elapsed_sec=100.,code='MOVEIT_UNREACHABLE')
    assert p.maximum_check_sec==0. and p.families=={}


def test_all_current_positive_endpoints_precede_unknown_without_removing_them():
    p,t=bound()
    rows=[('unknown_strong_support',False,0,0),('ik_weak_support',True,2,2),('fk',True,1,1)]
    assert p.order(rows)==(1,2,0)
    p.start(t,'fk')
    assert p.order(rows)==(2,0,1)


def test_call_wall_cost_includes_gateway_overhead_and_deadline_cannot_renew():
    p,t=bound();p.start(t,'a');p.finish(t,'a',elapsed_sec=1.64,code='OBSERVATION_PLANNING_TIMEOUT')
    assert p.bind({'station':'a','geometry_reference':'g'},remaining_sec=900.,monotonic_now=99.)==t
    r=p.reserve(remaining_sec=900.,monotonic_now=99.,planning_sec=1.,publication_sec=.85)
    assert not r['can_start'] and r['remaining_sec']==1. and r['estimated_strict_sec']==1.64
    assert r['attempted_count']==1 and r['timeout_count']==1


def test_exact_constructor_coordinates_survive_pixel_noise_without_pose_cache():
    variant={'observation_view':dict(orientation_source='measured_missing_side_geodesic',
        camera_distance_preference_index=2,camera_roll_preference_index=1,camera_tilt_preference_index=3)}
    assert family_key(variant,'pose1')==family_key(variant,'pose2')
    changed=deepcopy(variant);changed['observation_view']['camera_tilt_preference_index']=4
    assert family_key(variant,'pose1')!=family_key(changed,'pose1')
    assert family_key({},'pose1')!=family_key({},'pose2')


def test_budget_uses_existing_planning_plus_service_wait_before_first_measurement():
    p,t=bound()
    r=p.reserve(remaining_sec=2.09,monotonic_now=97.91,planning_sec=1.,publication_sec=.85)
    assert not r['can_start'] and r['estimated_strict_sec']==1.25
    r=p.reserve(remaining_sec=2.11,monotonic_now=97.89,planning_sec=1.,publication_sec=.85)
    assert r['can_start']


# Actual three-topic admission, rolling five-source snapshot, station certificate
# and producer publication. Only geometric view-source construction is stubbed;
# separate source/view suites retain its pixel and full-path tests.
from types import SimpleNamespace as NS
import json
import numpy as np
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, clock, row_at
from test_station_compute_work_handoff import configure, next_row, prepared_for
from test_remote_modes import base
from alicia_grasp_modes.observation_search_failure import validate_search_budget_failure


def real_station_prepared(c, monkeypatch):
    n=c.node;t=configure(c)
    n._observe_return_phase='observation_prefix';n.near_field_planning_active=False
    n._observe_return_trigger='BILATERAL_SURFACE_EVIDENCE_MISSING'
    n._observe_return_diagnostic_observation_only=False
    n._observe_return_pending_supplement=dict(session_id=c.binding['session_id'],
        target_epoch=c.binding['target_epoch'],target_track_id=c.binding['target_track_id'],
        previous_view_index=1,view_index=2,deadline_ns=c.session_deadline,
        contact_or_motion_authorized=False)
    n.grasp_config={};n.execution_plan_validity_sec=50.
    n._observe_return_view_gaps=[{'source_view_stamps_ns':[c.binding['phase_reference_source_ns']]}]
    n._observe_return_view_uncertainty_m=.002
    previous=None
    for i in range(5):
        row=next_row(c,t+i*400000000,previous);previous=row['status']['station_compute_wait']
        admitted=base.RemoteGrasp6DNode._poll_stream_snapshot(n)
        assert admitted is (i==4)
    assert admitted
    ticket=n._stream_worker_ticket
    assert n._station_compute_work_certificate(ticket)
    assert n._station_compute_work[ticket.request_id]['snapshot_policy']['purpose']=='supplementary_observation'
    assert len(ticket.payload[0].sample_stamp_ns)==5
    prepared=prepared_for(n,ticket)
    prepared.near_field=False;prepared.model_choice='unknown_tabletop';prepared.remote_failure_code=''
    source=dict(tool_from_camera=np.eye(4),camera={'width':24,'height':24},view_limits={},binding={})
    monkeypatch.setattr(n,'_require_diagnostic_observation_view_source',lambda _:source)
    n._station_observation_search_clock=lambda:c.clock[0]
    return prepared,row


def test_actual_station_certificate_enters_context_and_bound_failure_publishes_while_pending(station,monkeypatch):
    c=station;n=c.node;prepared,row=real_station_prepared(c,monkeypatch)
    context=n._station_observation_search_context(prepared)
    assert context is not None and context['contract']['binding']==c.binding
    assert prepared.ticket.target_epoch==c.binding['target_epoch']
    # Reach the last 1.9 seconds by continuing original independent source
    # joins. No lifetime/deadline/source is renewed from a callback timestamp.
    deadline=c.phase.deadline.to_nsec();source=deadline-2_000_000_000
    while row['status']['source_stamp_ns'] < source:
        ns=min(source,row['status']['source_stamp_ns']+400_000_000)
        row=next_row(c,ns,row['status']['station_compute_wait'])
    context['progress'].start(context['token'],'view')
    context['progress'].finish(context['token'],'view',elapsed_sec=1.64,code='OBSERVATION_PLANNING_TIMEOUT')
    # Current metric geometry may expire, while the unchanged identity contract
    # still permits a non-executable failure result.
    clock(c,source+850_000_000)
    assert n._station_compute_poll() and n._station_compute_pending
    budget=n._station_observation_search_reserve(context)
    assert not budget['station_can_start']
    context.update(budget=budget,station_budget_exhausted=True)
    runtime={'station_observation_search':context,'observation_variants':(object(),object())}
    result=n._publish_station_observation_search_exhausted(prepared,runtime)
    assert result['status']=='OBSERVATION_SEARCH_BUDGET_EXHAUSTED'
    plan=n.preview_rich_plan_pub.messages[-1]
    assert not plan.valid and not plan.poses and not plan.plan_id
    value=json.loads(plan.diagnostic.partition(': ')[2])
    assert validate_search_budget_failure(value,context['contract'],now_ns=n._ros_source_clock_ns(),
        header_ns=plan.header.stamp.to_nsec())==value
    assert value['request']['target_epoch']==value['binding']['target_epoch']
    assert n._station_compute_pending and not n._station_compute_work_published


def constructor_rows(prepared, shift=0.):
    rows=[]
    for i in range(2):
        pose=base.make_pose_stamped('base_link',[.2+shift,.02*i,.15],[0.,0.,0.,1.],stamp=prepared.stamp)
        audit=dict(orientation_source='measured_missing_side_geodesic',camera_distance_preference_index=i,
            camera_roll_preference_index=0,camera_tilt_preference_index=0,information_utility={'utility':.5})
        rows.append(dict(sequence=NS(pregrasp=pose),observation_view=audit))
    return rows


def test_real_new_source_and_geometry_noise_keep_only_family_scheduling_facts(station,monkeypatch):
    c=station;n=c.node;prepared,row=real_station_prepared(c,monkeypatch)
    first=n._station_observation_search_context(prepared)
    monkeypatch.setattr(n,'_order_unknown_observation_checks',lambda variants:tuple(variants))
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_strict_moveit_evaluation',lambda *a,**k:
        (base.MoveItResult(False,0.,0.,'one-second planner timed out',failure_code='OBSERVATION_PLANNING_TIMEOUT'),{},''))
    rows=constructor_rows(prepared)
    runtime={'prepared':prepared,'unknown_observation_scheduling':{}}
    n._active_unknown_observation_check_runtime=runtime
    iterator=n._station_progress_observation_checks(rows,first)
    selected=next(iterator);assert selected is rows[0]
    result=n._strict_moveit_evaluation(selected['sequence'].pregrasp,observation=True)
    assert not result[0].reachable and first['progress'].count(n._station_observation_family(rows[0]))==1
    iterator.close()
    ns=row['status']['source_stamp_ns']+400_000_000
    row=next_row(c,ns,row['status']['station_compute_wait'])
    assert base.RemoteGrasp6DNode._poll_stream_snapshot(n)
    # Drive the original coordinator's worker handoff without a ROS worker.
    completion=n.inference_coordinator.complete(prepared.ticket,now_sec=c.clock[0],target_epoch=n.target_instance_epoch)
    assert completion.accepted and completion.next_ticket is not None
    n._stream_worker_ticket=completion.next_ticket
    second_prepared=prepared_for(n,completion.next_ticket)
    second_prepared.near_field=False;second_prepared.model_choice='unknown_tabletop';second_prepared.remote_failure_code=''
    second_prepared.geometry.center_base[0]+=.0007
    second=n._station_observation_search_context(second_prepared)
    assert second_prepared.snapshot.stamp_ns!=prepared.snapshot.stamp_ns
    assert second['token']==first['token'] and second['progress'] is first['progress']
    current_rows=constructor_rows(second_prepared,shift=.0007)
    current_runtime={'prepared':second_prepared,'unknown_observation_scheduling':{}}
    n._active_unknown_observation_check_runtime=current_runtime
    new_iterator=n._station_progress_observation_checks(current_rows,second)
    assert next(new_iterator) is current_rows[1]
    new_iterator.close()
    assert set(second['progress'].families[n._station_observation_family(current_rows[0])])=={'attempts','timeouts','last_code'}
    assert not any(p.valid for p in n.preview_rich_plan_pub.messages)


def test_mixin_does_not_emit_budget_receipt_when_only_monotonic_reserve_expired(station,monkeypatch):
    c=station;n=c.node;prepared,_=real_station_prepared(c,monkeypatch)
    scoped=n._station_observation_search_context(prepared)
    # Simulate monotonic time advancing while the source clock remains fixed.
    scoped['progress'].deadline_monotonic=c.clock[0]+.01
    with pytest.raises(base.StreamResultCancelled,match='OBSERVATION_SEARCH_CLOCK_CHANGED'):
        n._station_observation_search_reserve(scoped)
    assert not any(p.candidate_source=='observation_search_budget_exhausted'for p in n.preview_rich_plan_pub.messages)


def test_source_reserve_cancels_request_without_becoming_invalid_reference_or_station_failure(station,monkeypatch):
    c=station;n=c.node;prepared,_=real_station_prepared(c,monkeypatch)
    scoped=n._station_observation_search_context(prepared)
    n.execution_plan_validity_sec=2.4
    with pytest.raises(base.StreamResultCancelled,match='OBSERVATION_SOURCE_RESERVE_REACHED') as caught:
        n._station_observation_search_checkpoint(scoped,'endpoint_ordering')
    assert not isinstance(caught.value,ValueError)
    assert not scoped['station_budget_exhausted']
    assert not any(p.candidate_source=='observation_search_budget_exhausted'for p in n.preview_rich_plan_pub.messages)


@pytest.mark.parametrize('entry',['ready','pending','pending_wait'])
def test_actual_accept_budget_stops_before_generation_and_during_original_pending_wait(station,monkeypatch,entry):
    c=station;n=c.node;prepared,row=real_station_prepared(c,monkeypatch)
    n._observe_return_view_gaps[0].update(target_epoch=c.identity.epoch,target_track_id=c.identity.track_id)
    prepared.candidates=();prepared.tabletop_candidates=();prepared.graspnet_input=None
    prepared.remote_diagnostics=dict(candidate_role='measured_observation',contact_candidates_requested=False,
        contact_geometry_checked=False)
    deadline=c.phase.deadline.to_nsec()
    source=deadline-{'ready':1_900_000_000,'pending':2_700_000_000,'pending_wait':3_200_000_000}[entry]
    while row['status']['source_stamp_ns'] < source:
        ns=min(source,row['status']['source_stamp_ns']+400_000_000)
        row=next_row(c,ns,row['status']['station_compute_wait'])
    waits=[]
    if entry!='ready':
        clock(c,source+850_000_000)
        assert n._station_compute_poll() and n._station_compute_pending
        def wait_original_interval(timeout):
            assert timeout==.05 and len(waits)<20
            waits.append(timeout)
            clock(c,n._ros_source_clock_ns()+50_000_000)
        monkeypatch.setattr(n._stream_condition,'wait',wait_original_interval)
    reference=base.make_pose_stamped('base_link',[.2,0.,.15],[0.,0.,0.,1.],stamp=prepared.stamp)
    monkeypatch.setattr(n,'_frozen_observation_reference_pose',lambda _:reference)
    monkeypatch.setattr(n,'_far_field_observation_variants',lambda *a,**k:pytest.fail('expensive generation started without remaining budget'))
    result=n._accept_diagnostic_observation_prediction(prepared)
    assert result['status']=='OBSERVATION_SEARCH_BUDGET_EXHAUSTED'
    assert not n.preview_rich_plan_pub.messages[-1].valid
    assert getattr(n,'_station_observation_generation_context',None) is None
    assert n._station_observation_activation_context is None
    assert bool(waits) is (entry=='pending_wait')
    if entry!='ready':
        assert n._station_compute_pending and not n._station_compute_work_published
        assert not n.latest_object_geometry.valid
