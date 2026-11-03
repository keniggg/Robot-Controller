"""Independent episode lifetime tests with explicit synthetic task/feedback doubles."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest
import rospy

from alicia_flexible_grasp.robot import mapped_contact_execution as module
from alicia_flexible_grasp.robot.mapped_contact_execution import MappedContactEpisode as Episode
from alicia_flexible_grasp.robot.stationary_following import stationary_following_error
from alicia_flexible_grasp.robot.physical_reference_contract import message_bytes
from test_stationary_following import fixture as following_fixture, XML
from test_contracted_trajectory import example


def case(monkeypatch):
    clock=NS(mono=100.,ros=10.05)
    monkeypatch.setattr(module.time,'monotonic',lambda:clock.mono)
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(clock.ros)))
    cfg={module.ENABLED_KEY:True}
    selection={'generation':1,'mode':'unknown','strategy':'observe_return','stamp_ns':9_000_000_000}
    scene=NS(digest='d'*64,track_id='target-1')
    context=dict(config=cfg,plan=NS(model_choice='unknown_tabletop'),epoch=9_000_000_000,
        revocation=0,selection=selection,scene=scene,model=XML,deadline=160.)
    session={'session_id':'synthetic-session','deadline_ns':110_000_000_000}
    origin=dict(selection=deepcopy(selection),epoch=context['epoch'],revocation=0,
        model=XML,config=deepcopy(cfg),scene=scene,identity='synthetic-origin',
        deadline=200.,view_session=session)
    node=NS(_physical_reference_lease=None,_observe_return_origin=origin)
    node._validated_observe_return_view_session=lambda captured,value:value
    constraints={n:dict(trajectory=.09,goal=.035) for n in ('Joint%d'%i for i in range(1,7))}
    constraints.update(goal_time=1.,stopped_velocity_tolerance=.03)
    params={'/alicia_controller/constraints':constraints,'/alicia_controller/stop_trajectory_duration':.5}
    monkeypatch.setattr(rospy,'get_param',lambda key,default=None:deepcopy(params.get(key,default)))
    fk,sdk,accepted=following_fixture()
    sample=stationary_following_error(fk,sdk,accepted,now_sec=clock.ros,epoch_ns=context['epoch'])
    validations=[]
    def validate():validations.append(True)
    c=NS(clock=clock,node=node,context=context,sample=sample,validate=validate,
         params=params,validations=validations)
    return c


def capture(c,phase='pregrasp-local',context=None,sample=None):
    return Episode.capture(c.node,c.context if context is None else context,
        c.sample if sample is None else sample,phase,c.validate)


def without_cached_episode(c):
    result=dict(c.context);result.pop('physical_reference_episode',None);return result


def test_r22_capture_keeps_original_clocks_and_fixed_source(monkeypatch):
    c=case(monkeypatch);e=capture(c);before=e.model.digest
    assert e.local_deadline==160. and e.outer_deadline==200.
    assert e.outer_ros_deadline==110_000_000_000 and not e.evidence['execution_authorized']
    assert capture(c) is e and e.model.digest==before
    c.context['deadline']=199.
    assert capture(c).local_deadline==160.


@pytest.mark.parametrize('fault',['stale','future','wrong_kind','short_window','one_sample','fake_capture_time'])
def test_r22_live_capture_requires_actual_fresh_stationary_source(monkeypatch,fault):
    c=case(monkeypatch);s=deepcopy(c.sample)
    if fault in ('stale','fake_capture_time'):
        for k in ('sdk_stamp_ns','accepted_stamp_ns','stationary_window_start_ns','stationary_window_end_ns'):
            s[k]-=600_000_000
        if fault=='fake_capture_time':s['stamp_sec']=c.clock.ros
    elif fault=='future':s['accepted_stamp_ns']=int((c.clock.ros+.1)*1e9)
    elif fault=='wrong_kind':s['measurement_kind']='predicted_encoder'
    elif fault=='short_window':s['stationary_window_start_ns']=s['stationary_window_end_ns']-299_999_999
    else:s['accepted_sample_count']=1
    with pytest.raises(ValueError):capture(c,sample=s)


def test_r22_failed_without_submission_cannot_reopen_same_phase(monkeypatch):
    c=case(monkeypatch);e=capture(c);e.finish(False)
    again=without_cached_episode(c);again['deadline']=170.
    with pytest.raises(ValueError):capture(c,context=again)


def test_r22_successful_no_motion_phase_is_consumed_without_forging_release(monkeypatch):
    c=case(monkeypatch);e=capture(c);result=e.finish(True)
    assert result['no_motion_submitted'] and not result['release_verified']
    assert e.chain['previous_binding_id']==''
    with pytest.raises(ValueError):capture(c,context=without_cached_episode(c))


def test_r22_invalid_capture_cannot_reset_phase_clock_after_model_failure(monkeypatch):
    c=case(monkeypatch);bad=deepcopy(c.sample);bad['sdk_positions_rad'][1]+=.2
    with pytest.raises(ValueError):capture(c,sample=bad)
    again=without_cached_episode(c);again['deadline']=170.
    with pytest.raises(ValueError):capture(c,context=again)


def test_r22_outer_deadline_or_constraints_change_invalidates_episode(monkeypatch):
    c=case(monkeypatch);e=capture(c)
    c.node._observe_return_origin['deadline']=201.
    with pytest.raises(ValueError):e.check()
    c.node._observe_return_origin['deadline']=200.
    c.params['/alicia_controller/constraints']['Joint1']['trajectory']=.12
    with pytest.raises(ValueError):e.check()


def test_r22_elapsed_local_time_cannot_be_replaced_by_longer_context_deadline(monkeypatch):
    c=case(monkeypatch);e=capture(c);c.context['deadline']=200.;c.clock.mono=160.001
    with pytest.raises(ValueError):e.check()


def test_r22_no_motion_success_allows_different_phase_without_renewing_outer(monkeypatch):
    c=case(monkeypatch);first=capture(c);first.finish(True)
    nxt=without_cached_episode(c);nxt['deadline']=161.
    second=capture(c,phase='approach-local',context=nxt)
    assert second.outer_deadline==first.outer_deadline
    assert second.outer_ros_deadline==first.outer_ros_deadline
    assert second.previous_binding_id==''


def test_r22_schedule_records_real_cost_and_changes_only_timing(monkeypatch):
    c=case(monkeypatch);e=capture(c);plan,_,_=example();old=deepcopy(plan)
    seen=[]
    def proof(p,*unused):
        seen.append(message_bytes(p));c.clock.mono+=.2;c.clock.ros+=.2;return {}
    planner=NS(observation_path_validator=proof)
    c.node._observation_controller_hold=lambda names:NS(positions=(0.,)*6,velocities=(0.,)*6,accelerations=(0.,)*6)
    e.install(planner,physical_prove=proof)
    result=e._schedule(plan)
    assert result is plan and len(seen)==2
    assert e.evidence['schedule']['proof_measurement_sec']==pytest.approx(.4)
    assert e.evidence['schedule']['start_lead_sec']==pytest.approx(2.8)
    assert result.joint_trajectory.header.stamp.to_nsec()==e.evidence['schedule']['scheduled_start_ns']
    old.joint_trajectory.header.stamp=result.joint_trajectory.header.stamp
    assert message_bytes(old)==message_bytes(result)
    assert e.local_deadline==160.
    e.restore();assert not hasattr(planner,'observation_trajectory_schedule')


def test_r22_schedule_that_cannot_fit_original_budget_is_rejected(monkeypatch):
    c=case(monkeypatch);c.context['deadline']=105.;e=capture(c);plan,_,_=example()
    planner=NS(observation_path_validator=lambda p:{})
    c.node._observation_controller_hold=lambda names:NS(positions=(0.,)*6,velocities=(0.,)*6,accelerations=(0.,)*6)
    e.install(planner,physical_prove=lambda p,r:{})
    with pytest.raises(ValueError):e._schedule(plan)
    assert e.local_deadline==105.


def test_r22_model_alone_does_not_install_or_dispatch_a_motion_path(monkeypatch):
    c=case(monkeypatch);e=capture(c)
    with pytest.raises(ValueError):e.install(NS())
    assert e.lease is None and not e.submissions and not e.evidence['execution_authorized']


def attach_released_motion(c, episode, *, fail_release=False, confirm=True):
    """Explicit trusted-transport double; it cannot certify real SDK I/O."""
    events=[]
    lease=NS(released=False,check_deadline=lambda:None)
    episode.lease=lease
    c.node._physical_reference_lease=lease
    c.node._endpoint_following_snapshot=lambda *a,**kw:deepcopy(c.sample)
    def release(value):
        events.append(('release',value,episode.model.digest))
        if fail_release:raise ValueError('synthetic unconfirmed same-wire release')
        value.released=confirm
        if confirm:c.node._physical_reference_lease=None
    c.node._release_physical_reference_lease=release
    return lease,events


def test_r22_finish_requires_confirmed_release_before_success_and_preserves_residual(monkeypatch):
    c=case(monkeypatch);e=capture(c);lease,events=attach_released_motion(c,e)
    old=e.model.digest
    c.sample['accepted_positions_rad'][1]-=.01
    c.sample['accepted_stamp_ns']+=1;c.sample['sdk_stamp_ns']+=1
    c.sample['opening_stamp_ns']=c.sample['accepted_stamp_ns']
    c.sample['stationary_window_end_ns']=min(c.sample['accepted_stamp_ns'],c.sample['sdk_stamp_ns'])
    result=e.finish(True)
    assert len(events)==1 and events[0][0]=='release'
    assert lease.released and result['release_verified'] and result['episode_success']
    assert result['final_following']['physical_reference_minus_encoder_rad'][1]==pytest.approx(.01)
    assert e.model.digest==old and e.chain['previous_binding_id']==e.model.binding_id


@pytest.mark.parametrize('fault',['exception','unconfirmed'])
def test_r22_release_failure_closes_entire_chain_and_never_reports_success(monkeypatch,fault):
    c=case(monkeypatch);e=capture(c)
    lease,_=attach_released_motion(c,e,fail_release=fault=='exception',confirm=fault!='unconfirmed')
    with pytest.raises(ValueError):e.finish(True)
    assert not e.evidence['release_verified'] and not e.evidence['episode_success']
    assert e.chain['failed'] and not e.chain['previous_binding_id']
    c.node._physical_reference_lease=None
    with pytest.raises(ValueError):capture(c,phase='contact-descent',context=without_cached_episode(c))


def test_r22_same_episode_cannot_reestimate_bias_after_successful_intermediate_motion(monkeypatch):
    c=case(monkeypatch);e=capture(c);old=e.model.digest
    moved=deepcopy(c.sample);moved['accepted_positions_rad'][1]-=.01
    moved['accepted_stamp_ns']+=1;moved['sdk_stamp_ns']+=1
    moved['opening_stamp_ns']=moved['accepted_stamp_ns']
    moved['stationary_window_end_ns']=min(moved['accepted_stamp_ns'],moved['sdk_stamp_ns'])
    again=capture(c,sample=moved)
    assert again is e and again.model.digest==old


def test_r22_next_successful_episode_can_capture_new_bias_with_same_outer_clock(monkeypatch):
    c=case(monkeypatch);first=capture(c);attach_released_motion(c,first)
    first.finish(True)
    old=first.model.digest
    c.clock.mono+=.1;c.clock.ros+=.1
    nxt=without_cached_episode(c);nxt['deadline']=170.
    sample=deepcopy(c.sample)
    for k in ('sdk_stamp_ns','accepted_stamp_ns','stationary_window_start_ns',
              'stationary_window_end_ns','opening_stamp_ns'):sample[k]+=100_000_000
    sample['accepted_positions_rad'][1]-=.01
    second=capture(c,phase='contact-descent',context=nxt,sample=sample)
    assert second.previous_binding_id==first.model.binding_id
    assert second.model.digest!=old and first.model.digest==old
    assert second.model.source_bias_limit_rad==(.09,)*6
    assert second.outer_deadline==first.outer_deadline==200.
    assert second.outer_ros_deadline==first.outer_ros_deadline==110_000_000_000
    assert second.local_deadline==170.


def test_r22_failed_finish_without_lease_cannot_advance_a_different_phase(monkeypatch):
    c=case(monkeypatch);e=capture(c)
    c.node._observe_return_origin['deadline']=201.
    with pytest.raises(ValueError):e.finish(True)
    c.node._observe_return_origin['deadline']=200.
    with pytest.raises(ValueError):capture(c,phase='contact-descent',context=without_cached_episode(c))


def test_r22_cached_capture_cannot_label_a_stale_sample_as_live(monkeypatch):
    c=case(monkeypatch);e=capture(c);c.clock.ros+=.6;c.clock.mono+=.6
    with pytest.raises(ValueError):capture(c)
    assert e.model.source_sdk_stamp_ns==c.sample['sdk_stamp_ns']


@pytest.mark.parametrize('fault',['source','caps'])
def test_r22_capture_failure_closes_owner_chain_across_different_phases(monkeypatch,fault):
    c=case(monkeypatch);bad=deepcopy(c.sample)
    if fault=='source':bad['sdk_positions_rad'][1]+=.2
    else:c.params['/alicia_controller/constraints']['Joint1']['trajectory']=None
    with pytest.raises(ValueError):capture(c,sample=bad)
    c.params['/alicia_controller/constraints']['Joint1']['trajectory']=.09
    with pytest.raises(ValueError):capture(c,phase='contact-descent',context=without_cached_episode(c))


def test_r22_actual_start_cad_failure_cannot_reach_typed_lease_prepare(monkeypatch):
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk,validate_observation_trajectory
    from test_observation_path_guard import scene,gripper
    c=case(monkeypatch);e=capture(c);wire,_,_=example()
    reference=NS(positions=(0.,)*6,velocities=(0.,)*6,accelerations=(0.,)*6)
    attempts=[]
    def forbidden(*a,**kw):attempts.append(True);raise AssertionError('CAD failure must precede transport')
    monkeypatch.setattr(module.PhysicalReferenceLease,'connect',forbidden)
    c.node._prepare_physical_reference_lease=forbidden
    def physical_proof(plan,ref):
        return validate_observation_trajectory(plan,scene(),SerialUrdfFk(XML,module.ARM_NAMES),
            gripper(),.05,ref)
    e.install(NS(observation_path_validator=lambda p:{}),physical_prove=physical_proof)
    with pytest.raises(ValueError,match='support clearance'):
        e._submission(wire,{},reference)
    assert not attempts and e.lease is None and not e.submissions


def released_with_refresh(c,*,fault=None):
    e=capture(c);attach_released_motion(c,e);e.finish(True)
    expected=deepcopy(c.sample);seen=[]
    def snapshot(*a,**kw):
        seen.append(kw['after_ns']);kw['context_validator']()
        fresh=deepcopy(expected);after=kw['after_ns']
        if fault in ('old_packets','fake_capture_stamp'):
            if fault=='fake_capture_stamp':fresh['stamp_sec']=c.clock.ros+.01
            return fresh
        c.clock.ros+=.4;c.clock.mono+=.4
        end=after+300_000_002
        fresh.update(sdk_stamp_ns=end+1,accepted_stamp_ns=end,opening_stamp_ns=end,
            stationary_window_start_ns=after+1,stationary_window_end_ns=end)
        if fault=='old_window':fresh['stationary_window_start_ns']=after-1
        elif fault=='sdk_changed':fresh['sdk_positions_rad'][0]+=2*np.pi/4096
        elif fault=='encoder_changed':fresh['accepted_positions_rad'][1]+=.001
        elif fault=='opening_changed':fresh['accepted_gripper_opening_m']-=.0001
        elif fault=='deadline':c.clock.mono=e.local_deadline+.001
        elif fault=='outer_changed':c.node._observe_return_origin['deadline']+=1.
        elif fault=='new_lease':c.node._physical_reference_lease=object()
        kw['context_validator']()
        return fresh
    c.node._endpoint_following_snapshot=snapshot
    return e,expected,seen


def test_r22_post_release_refresh_uses_real_new_equal_packets_without_rewriting_old_sample(monkeypatch):
    c=case(monkeypatch);e,expected,seen=released_with_refresh(c)
    before=deepcopy(expected);model=e.model.digest;deadline=(e.local_deadline,e.outer_deadline)
    result=e.refresh_completed_sample(expected)
    assert expected==before and result is not expected
    assert result['stationary_window_start_ns']>seen[0]
    assert result['sdk_stamp_ns']>expected['sdk_stamp_ns']
    assert result['accepted_stamp_ns']>expected['accepted_stamp_ns']
    assert result['accepted_positions_rad']==expected['accepted_positions_rad']
    assert (e.local_deadline,e.outer_deadline)==deadline and e.model.digest==model


@pytest.mark.parametrize('fault',['old_packets','fake_capture_stamp','old_window','sdk_changed',
    'encoder_changed','opening_changed','deadline','outer_changed','new_lease'])
def test_r22_post_release_refresh_rejects_reused_changed_or_expired_endpoint(monkeypatch,fault):
    c=case(monkeypatch);e,expected,seen=released_with_refresh(c,fault=fault)
    with pytest.raises(ValueError):e.refresh_completed_sample(expected)
    assert e.failed and e.chain['failed'] and e.evidence['episode_success'] is False


def r26_mapped_admission(monkeypatch, *, fault=None, local_budget_sec=None, outer_budget_sec=None):
    """Enabled real episode -> real connect -> real lease, without ROS I/O.

    New SDK/encoder messages are explicit simulated packets, consumed by the
    real stationary-window/FK helper. Geometry proof numbers remain the existing
    synthetic fixture; the real dual-contract validators check their binding.
    Only service discovery/transport and message delivery are substituted.
    """
    from test_r22_typed_client_independent import Clock,r26_transport
    from test_r22_dual_reference_independent import fixture as dual_fixture
    from alicia_flexible_grasp.robot.physical_reference_contract import (
        physical_plan_from_wire,physical_reference_from_wire,commanded_reference_bounds)
    from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
    c=case(monkeypatch);clock=Clock();clock.ros=10_050_000_000;clock.mono=100.
    c.clock=clock
    monkeypatch.setattr(module,'time',NS(monotonic=clock.now))
    monkeypatch.setattr(rospy.Time,'now',staticmethod(
        lambda:rospy.Time(clock.ros//10**9,clock.ros%10**9)))
    if local_budget_sec is not None:c.context['deadline']=clock.mono+local_budget_sec
    if outer_budget_sec is not None:
        c.node._observe_return_origin['deadline']=clock.mono+outer_budget_sec
        c.node._observe_return_origin['view_session']['deadline_ns']=clock.ros+int(outer_budget_sec*1e9)
    f=dual_fixture()
    c.params['/alicia_controller/constraints']=deepcopy(f['wire_audit']['controller_constraints_snapshot'])
    e=capture(c,phase='nominal_pregrasp')
    f['following_model']=e.model
    f['wire_plan'].joint_trajectory.header.stamp=rospy.Time(14,0)
    f['physical_plan']=physical_plan_from_wire(f['wire_plan'],e.model)
    f['physical_reference']=physical_reference_from_wire(f['wire_reference'],e.model)
    for domain in ('wire','physical'):
        audit=f[domain+'_audit'];plan=f[domain+'_plan'];reference=f[domain+'_reference']
        audit.update(model_sha256=e.model.model_sha256,scene_sha256=c.context['scene'].digest,
            trajectory_sha256=wire_digest(plan),bridge_start_joint_positions_rad=list(reference.positions))
    f['wire_audit']['wire_reference_contract']=commanded_reference_bounds(
        f['wire_plan'],f['wire_reference'],.5)
    calls=[];samples=[];packets=[]
    def after_validation():
        if fault=='owner':c.node._observe_return_origin['identity']='other-owner'
        elif fault=='epoch_context':c.node._observe_return_origin['epoch']+=1
        elif fault=='deadline':clock.mono=e.local_deadline+.000001
        elif fault=='ros_deadline':clock.ros=e.local_ros_deadline+1
    io=r26_transport(monkeypatch,clock,after_validation=after_validation)
    c.node._trajectory_controller_state=NS(header=NS(stamp=rospy.Time(10,0)))
    def packet(stamp,frame,positions,measured=False):
        return NS(header=NS(stamp=rospy.Time(stamp//10**9,stamp%10**9),frame_id=frame),
            name=list(module.ARM_NAMES)+(['right_finger'] if measured else []),
            position=list(positions)+([c.sample['accepted_gripper_opening_m']] if measured else []))
    def snapshot(fk,epoch,context,*,context_validator):
        context_validator();calls.append('snapshot')
        assert len(io.waits)==2 and len(io.validations)==1
        io.order.append(('source',clock.ros))
        if fault=='old_packets':return deepcopy(c.sample)
        sdk=list(e.model.source_sdk_positions_rad)
        accepted=list(e.model.source_accepted_positions_rad)
        if fault=='sdk_vector':sdk[0]+=2*np.pi/4096
        elif fault=='accepted_vector':accepted[1]+=.001
        sdk_packets=[packet(clock.ros-age,'sdk_transmitted',sdk)
            for age in (450_000_000,340_000_000,230_000_000,120_000_000,10_000_000)]
        accepted_packets=[packet(clock.ros-age,'sdk_measured',accepted,True)
            for age in (451_000_000,341_000_000,231_000_000,121_000_000,11_000_000)]
        result=stationary_following_error(fk,sdk_packets,accepted_packets,
            now_sec=clock.ros*1e-9,epoch_ns=epoch+(1 if fault=='epoch_sample' else 0))
        packets.append(deepcopy((sdk_packets,accepted_packets)))
        samples.append(deepcopy(result))
        if fault!='controller_stale':
            stamp=clock.ros-2_000_000
            c.node._trajectory_controller_state=NS(header=NS(stamp=rospy.Time(stamp//10**9,stamp%10**9)))
        context_validator()
        return result
    c.node._endpoint_following_snapshot=snapshot
    def physical_proof(plan,reference):
        calls.append('physical_proof')
        assert message_bytes(plan)==message_bytes(f['physical_plan'])
        assert tuple(reference.positions)==tuple(f['physical_reference'].positions)
        return deepcopy(f['physical_audit'])
    e.install(NS(observation_path_validator=lambda p:deepcopy(f['wire_audit'])),
              physical_prove=physical_proof)
    def prepare(lease):
        calls.append('prepare')
        for peer in io.peers.values():peer.lease=lease
        c.node._physical_reference_lease=lease
        lease.prepare();lease.commit()
    c.node._prepare_physical_reference_lease=prepare
    return NS(case=c,episode=e,proof=f,io=io,calls=calls,samples=samples,packets=packets)


def test_r26_enabled_nearfield_episode_admits_new_packets_after_transport_and_proof_cost(monkeypatch):
    c=r26_mapped_admission(monkeypatch);e=c.episode;f=c.proof;clock=c.case.clock
    original=deepcopy(c.case.sample);model=e.model.digest
    deadlines=(e.outer_deadline,e.local_deadline,e.outer_ros_deadline,e.local_ros_deadline)
    wire=message_bytes(f['wire_plan']);physical=message_bytes(f['physical_plan'])
    submission=e._submission(f['wire_plan'],deepcopy(f['wire_audit']),f['wire_reference'])
    assert len(c.samples)==len(c.packets)==1
    assert c.calls==['physical_proof','snapshot','prepare']
    sample=c.samples[0];sdk,accepted=c.packets[0];lease=submission.lease
    assert sample['sdk_stamp_ns']==sdk[-1].header.stamp.to_nsec()>original['sdk_stamp_ns']
    assert sample['accepted_stamp_ns']==accepted[-1].header.stamp.to_nsec()>original['accepted_stamp_ns']
    assert lease.template.source_sdk_stamp_ns==sample['sdk_stamp_ns']
    assert lease.template.source_encoder_stamp_ns==sample['accepted_stamp_ns']
    assert lease.template.controller_stamp_ns==c.case.node._trajectory_controller_state.header.stamp.to_nsec()
    assert lease.template.issued_ns==clock.ros
    assert lease.template.deadline_ns==e.outer_ros_deadline
    assert lease.template.episode_deadline_ns==e.local_ros_deadline
    expected_outer=min(e.outer_deadline,clock.mono+(e.outer_ros_deadline-clock.ros)*1e-9)
    assert lease.outer_deadline==pytest.approx(expected_outer)
    assert lease.local_deadline==pytest.approx(e.local_deadline)
    assert lease.template.maximum_duration_sec==pytest.approx(expected_outer-clock.mono)
    assert lease.template.episode_duration_sec==pytest.approx(e.local_deadline-clock.mono)
    assert (e.outer_deadline,e.local_deadline,e.outer_ros_deadline,e.local_ros_deadline)==deadlines
    assert c.case.sample==original and e.model.digest==model
    assert message_bytes(f['wire_plan'])==wire and message_bytes(f['physical_plan'])==physical
    assert message_bytes(submission.contract.wire_plan())==wire
    assert message_bytes(submission.contract.physical_plan())==physical
    assert [op for _,op,_ in c.io.events]==[lease.PREPARE,lease.PREPARE,lease.COMMIT,lease.COMMIT]
    assert all(r.issued_ns==clock.ros for _,_,r in c.io.events)
    assert len(e.submissions)==1 and not e.evidence['execution_authorized']
    assert e.chain['phases'][e.episode_id]=='consumed_before_prepare'
    e.restore()


@pytest.mark.parametrize('fault',['sdk_vector','accepted_vector','epoch_sample','epoch_context',
                                  'owner','deadline','ros_deadline','old_packets','controller_stale'])
def test_r26_nearfield_source_or_owner_change_after_setup_cannot_prepare(monkeypatch,fault):
    c=r26_mapped_admission(monkeypatch,fault=fault);e=c.episode;f=c.proof
    model=e.model.digest;wire=message_bytes(f['wire_plan']);original=deepcopy(c.case.sample)
    with pytest.raises(ValueError):
        e._submission(f['wire_plan'],deepcopy(f['wire_audit']),f['wire_reference'])
    assert 'prepare' not in c.calls and not c.io.events and not e.submissions and e.lease is None
    assert c.case.node._physical_reference_lease is None
    assert e.model.digest==model and c.case.sample==original and message_bytes(f['wire_plan'])==wire
    result=e.finish(False)
    assert result['episode_success'] is False and not result['execution_authorized']
    assert e.failed and e.chain['failed']
    e.restore()


def test_r26_mapped_snapshot_cost_is_included_before_issue_and_prepare(monkeypatch):
    from test_r22_typed_client_independent import r26_advance
    c=r26_mapped_admission(monkeypatch);e=c.episode;f=c.proof;clock=c.case.clock
    original=deepcopy(c.case.sample);model=e.model.digest;bias=e.model.bias_rad
    deadlines=(e.outer_deadline,e.local_deadline,e.outer_ros_deadline,e.local_ros_deadline)
    wire=message_bytes(f['wire_plan']);physical=message_bytes(f['physical_plan'])
    setup_ros=clock.ros;setup_mono=clock.mono;timeline=[]
    original_snapshot=c.case.node._endpoint_following_snapshot
    collect_ns=410_000_000;finish_ns=190_000_000
    def timed_snapshot(*args,**kwargs):
        timeline.append(('entered',clock.ros,clock.mono))
        r26_advance(clock,collect_ns)
        result=original_snapshot(*args,**kwargs)
        r26_advance(clock,finish_ns)
        timeline.append(('returned',clock.ros,clock.mono))
        return result
    monkeypatch.setattr(c.case.node,'_endpoint_following_snapshot',timed_snapshot)
    submission=e._submission(f['wire_plan'],deepcopy(f['wire_audit']),f['wire_reference'])
    lease=submission.lease;admission=lease.source_admission;elapsed_ns=1_830_000_000
    assert len(timeline)==2 and len(c.samples)==len(c.packets)==1
    assert lease.template.issued_ns==timeline[1][1]==setup_ros+elapsed_ns
    assert lease.template.issued_ns>timeline[0][1]>setup_ros
    sample=c.samples[0];sdk,accepted=c.packets[0]
    assert sample['sdk_stamp_ns']==sdk[-1].header.stamp.to_nsec()>original['sdk_stamp_ns']
    assert sample['accepted_stamp_ns']==accepted[-1].header.stamp.to_nsec()>original['accepted_stamp_ns']
    assert lease.template.source_sdk_stamp_ns==sample['sdk_stamp_ns']
    assert lease.template.source_encoder_stamp_ns==sample['accepted_stamp_ns']
    assert clock.ros-sample['sdk_stamp_ns']==finish_ns+10_000_000
    assert clock.ros-sample['accepted_stamp_ns']==finish_ns+11_000_000
    assert clock.ros-lease.template.controller_stamp_ns==finish_ns+2_000_000
    initial_outer_budget=min(e.outer_deadline-setup_mono,(e.outer_ros_deadline-setup_ros)*1e-9)
    initial_episode_budget=min(e.local_deadline-setup_mono,(e.local_ros_deadline-setup_ros)*1e-9)
    assert lease.template.maximum_duration_sec==pytest.approx(initial_outer_budget-elapsed_ns*1e-9)
    assert lease.template.episode_duration_sec==pytest.approx(initial_episode_budget-elapsed_ns*1e-9)
    assert lease.template.deadline_ns==e.outer_ros_deadline
    assert lease.template.episode_deadline_ns==e.local_ros_deadline
    assert lease.outer_deadline==pytest.approx(setup_mono+initial_outer_budget)
    assert lease.local_deadline==pytest.approx(setup_mono+initial_episode_budget)
    assert admission['service_wait_elapsed_sec']==pytest.approx(.62)
    assert admission['contract_validation_elapsed_sec']==pytest.approx(.61)
    assert admission['source_snapshot_elapsed_sec']==pytest.approx(.6)
    assert admission['setup_issued_ns']==setup_ros
    assert admission['issued_ns']==admission['checked_ns']==timeline[1][1]
    assert admission['passed'] is True and e.evidence['source_admission']==admission
    assert c.calls==['physical_proof','snapshot','prepare']
    assert [op for _,op,_ in c.io.events]==[lease.PREPARE,lease.PREPARE,lease.COMMIT,lease.COMMIT]
    assert all(request.issued_ns==timeline[1][1] for _,_,request in c.io.events)
    assert (e.outer_deadline,e.local_deadline,e.outer_ros_deadline,e.local_ros_deadline)==deadlines
    assert c.case.sample==original and e.model.digest==model and tuple(lease.template.bias_rad)==bias
    assert message_bytes(submission.contract.wire_plan())==wire
    assert message_bytes(submission.contract.physical_plan())==physical
    assert len(e.submissions)==1 and not e.evidence['execution_authorized']
    e.restore()


@pytest.mark.parametrize('deadline_kind',['episode','outer'])
@pytest.mark.parametrize('clock_kind',['ros','monotonic'])
def test_r26_mapped_snapshot_crossing_original_deadline_never_prepares(monkeypatch,deadline_kind,clock_kind):
    budgets=({'local_budget_sec':1.5} if deadline_kind=='episode' else {'outer_budget_sec':1.5})
    c=r26_mapped_admission(monkeypatch,**budgets);e=c.episode;f=c.proof;clock=c.case.clock
    original=deepcopy(c.case.sample);model=e.model.digest;wire=message_bytes(f['wire_plan'])
    deadlines=(e.outer_deadline,e.local_deadline,e.outer_ros_deadline,e.local_ros_deadline)
    ros_deadline=e.local_ros_deadline if deadline_kind=='episode' else e.outer_ros_deadline
    mono_deadline=e.local_deadline if deadline_kind=='episode' else e.outer_deadline
    setup_ros=clock.ros;timeline=[];original_snapshot=c.case.node._endpoint_following_snapshot
    def crossing_snapshot(*args,**kwargs):
        assert clock.ros<ros_deadline and clock.mono<mono_deadline
        timeline.append(('entered',clock.ros,clock.mono))
        result=original_snapshot(*args,**kwargs)
        if clock_kind=='ros':clock.ros=ros_deadline+1
        else:clock.mono=mono_deadline+.000001
        timeline.append(('returned',clock.ros,clock.mono))
        return result
    monkeypatch.setattr(c.case.node,'_endpoint_following_snapshot',crossing_snapshot)
    with pytest.raises(ValueError) as rejected:
        e._submission(f['wire_plan'],deepcopy(f['wire_audit']),f['wire_reference'])
    assert len(timeline)==2 and len(c.samples)==len(c.packets)==1
    sample=c.samples[0];controller=c.case.node._trajectory_controller_state.header.stamp.to_nsec()
    assert all(0<=clock.ros-stamp<=500_000_000
        for stamp in (sample['sdk_stamp_ns'],sample['accepted_stamp_ns'],controller))
    admission=e.evidence['source_admission']
    assert admission==rejected.value.source_admission
    assert admission['passed'] is False and admission['source_snapshot_used'] is True
    assert admission['setup_issued_ns']==setup_ros
    assert admission['issued_ns']==admission['checked_ns']==timeline[1][1]
    assert admission['source_stamps_ns']==dict(sdk_stamp_ns=sample['sdk_stamp_ns'],
        accepted_stamp_ns=sample['accepted_stamp_ns'],controller_stamp_ns=controller)
    assert admission['source_ages_ns']=={key:clock.ros-stamp
        for key,stamp in admission['source_stamps_ns'].items()}
    assert admission['maximum_source_age_ns']==500_000_000
    assert admission['original_outer_ros_deadline_ns']==e.outer_ros_deadline
    assert admission['original_episode_ros_deadline_ns']==e.local_ros_deadline
    assert admission['original_outer_monotonic_deadline']==e.outer_deadline
    assert admission['original_episode_monotonic_deadline']==e.local_deadline
    assert admission['service_wait_elapsed_sec']==pytest.approx(.62)
    assert admission['contract_validation_elapsed_sec']==pytest.approx(.61)
    assert admission['source_snapshot_elapsed_sec']==pytest.approx(timeline[1][2]-timeline[0][2])
    assert c.calls==['physical_proof','snapshot']
    assert not c.io.events and not e.submissions and e.lease is None
    assert c.case.node._physical_reference_lease is None
    assert (e.outer_deadline,e.local_deadline,e.outer_ros_deadline,e.local_ros_deadline)==deadlines
    assert c.case.sample==original and e.model.digest==model and message_bytes(f['wire_plan'])==wire
    result=e.finish(False)
    assert result['episode_success'] is False and not result['execution_authorized']
    assert e.failed and e.chain['failed']
    e.restore()
