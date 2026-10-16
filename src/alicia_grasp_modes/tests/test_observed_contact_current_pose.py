"""Offline internal handoff: real audit code/receipt binding, inert transport."""
from copy import deepcopy
from types import SimpleNamespace as NS
import threading

import pytest

from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
from alicia_grasp_modes.observe_return_session import parse_session
from alicia_grasp_modes.runner_contract import observed_contact_audit_error, observe_return_audit_error
from test_observe_return_task import context, begin, evidence, audit_reference, at_ns
from test_runner_observe_return import runner_for
from test_remote_modes import configured


@pytest.fixture
def observed(context):
    c = context
    begin(c)
    session = c.node._observe_return_session
    _, report, plan = evidence('initial_contact')
    stamp = 25_000_000_000
    plan.plan_id = report['plan_id'] = 'e'*24
    plan.header.stamp = at_ns(stamp)
    plan.refinement_status = 'VALID_3D'
    report['snapshot_stamp_ns'] = stamp
    report['selected']['final_strict_plan_id'] = plan.plan_id
    reached = 24_000_000_000
    phase = NS(active=True, phase_id=4, header=NS(stamp=at_ns(reached+100), frame_id='base_link'),
        deadline=at_ns(80_000_000_000), reference_center_valid=True,
        reference_target_track_id=plan.target_track_id, reference_source_stamp=at_ns(reached),
        planning_stage='contact_replan')
    measurement = dict(version=1, state='qualified', lease_id='lease', request_id='lease:qualified',
        selection=deepcopy(c.selection), session_id=session['session_id'],
        target_epoch=session['target_epoch'], target_track_id=session['target_track_id'],
        origin_plan_id=session['origin_plan_id'], origin_snapshot_stamp_ns=int(session['origin_snapshot_stamp_ns']),
        view_plan_id=session['view_plan_id'], view_source_stamp_ns=int(session['view_source_stamp_ns']),
        view_index=session['view_index'], plan_sha256='a'*64, control_epoch_ns=20_000_000_000,
        deadline_ns=int(session['deadline_ns']), stamp_ns=reached-50, motion_kind='observation',
        arrival_stamp_ns=23_000_000_000, capture_deadline_ns=26_000_000_000,
        source_stamps_ns=[23_100_000_000,23_200_000_000,23_300_000_000],
        metric_geometry_authority=False, motion_authorized=False)
    fused = [19_500_000_000,reached,stamp]
    report['multiview_surface'] = dict(active=True,phase_id=12,
        target_epoch=session['target_epoch'],target_track_id=session['target_track_id'],
        source_stamps_ns=fused,latest_registration=dict(ok=True,code='REGISTERED',
            source_clipped=False,source_stamp_ns=stamp))
    remote = NS(_observe_return_selected=lambda:True, _observe_return_phase='observing_contact',
        _current_stream_target_identity=lambda:NS(epoch=session['target_epoch'],track_id=session['target_track_id']),
        _observe_return_initial_stamp=fused[0],_observe_return_identity_transition=None,
        _observe_return_diagnostic_observation_only=False,_observe_return_trigger_evidence=None,
        _observe_return_view_gaps=(),_observe_return_trigger='BILATERAL_SURFACE_EVIDENCE_MISSING',
        _observe_return_session=parse_session(session,c.selection,22_000_000_000),
        _active_multiview_surface=lambda:(NS(view_stamps_ns=fused),None),
        _observe_return_reached_contact_context=dict(phase_id=phase.phase_id,
            start_stamp_ns=phase.header.stamp.to_nsec(),deadline_ns=phase.deadline.to_nsec(),
            source_stamp_ns=reached,registration_phase_id=12),
        _measurement_remote_request=measurement)
    # Actual production metadata builder. Synthetic already-gated contact
    # geometry/registration isolates this handoff, not a physical success.
    ObserveReturnRemoteMixin._observe_return_audit(remote,report,NS(snapshot=NS(stamp_ns=stamp)))
    c.now[0]=stamp+1
    c.node._published_near_field_phase=phase
    c.node._near_field_phase_id=phase.phase_id
    c.node._near_field_active=True
    c.node._near_field_phase_started_ns=phase.header.stamp.to_nsec()
    c.node._near_field_phase_deadline_ns=phase.deadline.to_nsec()
    c.node._near_field_phase_started_sec=phase.header.stamp.to_sec()
    c.node._near_field_phase_deadline_sec=phase.deadline.to_sec()
    c.node._observation_measurement_request=measurement
    c.node._following_evidence_lock=threading.RLock()
    c.node._following_epoch_ns=measurement['control_epoch_ns']
    c.contact,c.contact_report,c.reached_phase,c.measurement=plan,report,phase,measurement
    c.remote=remote
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path,report))
    return c


def error(c):
    return observed_contact_audit_error(c.contact_report,session=c.node._observe_return_session,
        reached_phase=c.reached_phase,measurement=c.measurement,now_ns=c.now[0])


@pytest.mark.parametrize('cfg',[{}, {'unknown_observe_return_return_to_origin':False}])
def test_real_remote_audit_file_task_callback_default_continues_without_return(observed,cfg):
    c=observed
    assert not error(c)
    assert c.node._before_near_field_contact_freeze(c.contact,c.plan,cfg) is None,c.states
    assert c.node._observe_return_session['phase']=='observing'
    assert [p['phase'] for p in c.publications]==['observing']
    assert c.gateways==[('/supervisor/capture_observe_return_origin',c.plan.plan_id,True)]
    assert not c.replans
    assert c.node._observe_return_reports[c.contact.plan_id]==c.contact_report
    # Internal continuation cannot become a new externally startable plan.
    assert 'not an initial attempt entry' in observe_return_audit_error(c.contact_report)
    assert runner_for(c.selection).audit_error(c.contact_report)
    assert not c.node._observation_measurement_scope(c.contact)


@pytest.mark.parametrize('fault', ['session','phase','view','view_plan','view_source','epoch','track',
    'origin','selection','deadline','reached_id','reached_start','reached_source','reached_deadline',
    'reached_invalid','registration','clipped','registration_source','registration_phase','fused_initial',
    'fused_reached','fused_current','receipt','receipt_source','receipt_view','receipt_state',
    'contact_stage','contact_collision','strict_plan','initial_phase','diagnostic'])
def test_foreign_stale_or_incomplete_context_cannot_release_contact(observed,fault):
    c=observed;r=c.contact_report;m=r['observe_return'];s=c.node._observe_return_session
    if fault=='session':s['session_id']='foreign'
    elif fault=='phase':s['phase']='returning'
    elif fault=='view':s['view_index']+=1
    elif fault=='view_plan':s['view_plan_id']='b'*24
    elif fault=='view_source':m['view_source_stamp_ns']=int(m['view_source_stamp_ns'])+1
    elif fault=='epoch':r['target_epoch']+=1
    elif fault=='track':m['target_track_id']='foreign'
    elif fault=='origin':m['origin_plan_id']='f'*24
    elif fault=='selection':r['grasp_mode_selection']['generation']+=1
    elif fault=='deadline':c.now[0]=c.reached_phase.deadline.to_nsec()
    elif fault=='reached_id':c.reached_phase.phase_id+=1
    elif fault=='reached_start':c.reached_phase.header.stamp=at_ns(c.reached_phase.header.stamp.to_nsec()+1)
    elif fault=='reached_source':c.reached_phase.reference_source_stamp=at_ns(24_000_000_001)
    elif fault=='reached_deadline':c.reached_phase.deadline=at_ns(80_000_000_001)
    elif fault=='reached_invalid':c.reached_phase.reference_center_valid=False
    elif fault=='registration':r['multiview_surface']['latest_registration']['ok']=False
    elif fault=='clipped':r['multiview_surface']['latest_registration']['source_clipped']=None
    elif fault=='registration_source':r['multiview_surface']['latest_registration']['source_stamp_ns']-=1
    elif fault=='registration_phase':r['multiview_surface']['phase_id']+=1
    elif fault.startswith('fused_'):
        idx={'fused_initial':0,'fused_reached':1,'fused_current':2}[fault]
        m['fused_source_stamps_ns'].pop(idx)
        r['multiview_surface']['source_stamps_ns']=m['fused_source_stamps_ns']
    elif fault=='receipt':m['measurement_receipt_sha256']='0'*64
    elif fault=='receipt_source':m['qualified_source_stamps_ns'][-1]+=1
    elif fault=='receipt_view':c.measurement['view_index']+=1
    elif fault=='receipt_state':c.measurement['state']='requalifying'
    elif fault=='contact_stage':r['selected']['execution_sequence']['stages'].pop()
    elif fault=='contact_collision':r['selected']['moveit']['collision_free']=False
    elif fault=='strict_plan':r['selected']['final_strict_plan_id']='b'*24
    elif fault=='initial_phase':m['phase']='initial_contact'
    else:m['diagnostic_observation_only']=True
    assert error(c)
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path,r))
    assert c.node._before_near_field_contact_freeze(c.contact,c.plan,{})=={'failed':True}
    assert not [g for g in c.gateways if 'return_observe' in g[0]]


@pytest.mark.parametrize('value',[0,1,'false','true',None])
def test_return_option_must_be_explicit_boolean(observed,value):
    c=observed
    assert c.node._before_near_field_contact_freeze(c.contact,c.plan,
        {'unknown_observe_return_return_to_origin':value})=={'failed':True}
    assert 'RETURN_OPTION_MUST_BE_BOOL' in c.states[-1][1]


def test_control_epoch_change_rejects_internal_contact_despite_valid_audit(observed):
    c=observed;c.node._following_epoch_ns+=1
    assert not error(c)
    assert c.node._before_near_field_contact_freeze(c.contact,c.plan,{})=={'failed':True}
    assert 'CONTROL_EPOCH_CHANGED' in c.states[-1][1]


def test_successfully_registered_clipped_source_keeps_original_contact_contract(observed):
    c=observed
    c.contact_report['multiview_surface']['latest_registration']['source_clipped']=True
    assert not error(c)  # Original VALID_3D/complete contact gates decide it.


@pytest.mark.parametrize('fault',['phase_replaced','phase_stopped','deadline_replaced','unregistered_plan'])
def test_actual_task_phase_cannot_be_replaced_after_audit_build(observed,fault):
    c=observed
    if fault=='phase_replaced':c.node._near_field_phase_id+=1
    elif fault=='phase_stopped':c.node._near_field_active=False
    elif fault=='deadline_replaced':c.node._near_field_phase_deadline_sec+=1
    else:c.contact.refinement_status='INVALID_3D'
    assert not error(c)
    assert c.node._before_near_field_contact_freeze(c.contact,c.plan,{})=={'failed':True}


def test_actual_registration_callback_records_exact_task_phase_without_reseeding(configured,monkeypatch):
    from test_observe_return_remote import start_session, direct_observation, module
    node,_,clock,_,initial=start_session(configured)
    clock[0]=21_000_000_000
    obs=direct_observation(node,19_000_000_037)
    monkeypatch.setattr(node,'_resolve_reached_view_observation',lambda msg:obs)
    phase=NS(active=True,phase_id=7,header=NS(stamp=at_ns(20_000_000_073),frame_id='base_link'),
        deadline=at_ns(70_000_000_089))
    node.near_field_state_cb(phase)
    assert node._latest_registration_evidence.result.ok
    assert node._active_multiview_surface()[0].view_stamps_ns==(initial.view_stamps_ns[0],obs.stamp_ns)
    assert node._observe_return_reached_contact_context==dict(phase_id=7,
        start_stamp_ns=20_000_000_073,deadline_ns=70_000_000_089,
        source_stamp_ns=obs.stamp_ns,registration_phase_id=node._near_field_phase_id)
    node._observe_return_advance('observation_prefix',22_000_000_000,80_000_000_000,
        active=False,paused=False)
    assert node._observe_return_reached_contact_context is None


def test_supplemental_view_contact_uses_current_view_without_returning_to_origin(observed):
    c=observed;previous=deepcopy(c.node._observe_return_session)
    updated=dict(previous,view_index=2,view_plan_id='2'*24,
        view_source_stamp_ns=22_500_000_000,stamp_ns=22_600_000_000,
        view_request_id='b'*64,previous_gain_sha256='c'*64)
    session=parse_session(updated,c.selection,c.now[0],previous)
    c.node._observe_return_session=session
    current=deepcopy(c.plan);current.plan_id=session['view_plan_id']
    current.header.stamp=at_ns(session['view_source_stamp_ns'])
    c.measurement.update(view_index=2,view_plan_id=current.plan_id,
        view_source_stamp_ns=session['view_source_stamp_ns'])
    c.remote._observe_return_session=deepcopy(session)
    ObserveReturnRemoteMixin._observe_return_audit(c.remote,c.contact_report,
        NS(snapshot=NS(stamp_ns=c.contact.header.stamp.to_nsec())))
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path,c.contact_report))
    assert not error(c)
    assert c.node._before_near_field_contact_freeze(c.contact,current,{}) is None,c.states
    assert c.node._observe_return_session['view_index']==2
    assert c.node._observe_return_origin_plan.plan_id==previous['origin_plan_id']
    assert c.node._observe_return_session['phase']=='observing'
    assert not [g for g in c.gateways if 'return_observe' in g[0]]
