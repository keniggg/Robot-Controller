"""Independent injected-transport lease and exact-cancel tests; no ROS I/O."""
from copy import deepcopy
from types import SimpleNamespace as NS
import math

import numpy as np
import pytest
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header

from alicia_flexible_grasp.robot.following_model import source_bias_model
from alicia_flexible_grasp.robot.physical_reference_contract import bind_dual_reference,commanded_reference_bounds
from alicia_flexible_grasp.robot.physical_reference_client import PhysicalReferenceLease, MappedTrajectorySubmission
from alicia_flexible_grasp.robot import physical_reference_client as client_module
from alicia_flexible_grasp.robot.physical_reference_contract import message_bytes
from alicia_flexible_grasp.robot.contracted_trajectory import ContractedTrajectoryExecutor
from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
from test_r22_dual_reference_independent import fixture as proof_fixture


class Clock:
    ros=10_000_000_000
    mono=100.
    def now(self):return self.mono
    def ros_now(self):return self.ros


class Peer:
    def __init__(self, name, clock, events):
        self.name,self.clock,self.events=name,clock,events
        self.lease=None;self.phase=0;self.goal_terminal=False;self.fail_op=None;self.overrides={}
    def __call__(self,r):
        self.events.append((self.name,r.operation,deepcopy(r)))
        states={1:1,2:2,3:3,4:2,5:5,6:0}
        if r.operation in states:self.phase=states[r.operation]
        l=self.lease;s=NS(**deepcopy(vars(l.template)))
        s.header=Header(stamp=rospy.Time(self.clock.ros//10**9,self.clock.ros%10**9))
        s.phase=self.phase;s.evidence_valid=True;s.output_gated=self.phase in (1,3,5)
        s.pending_complete=s.stationary=True;s.last_command_sequence=7
        s.monotonic_remaining_sec=l.local_deadline-self.clock.mono
        s.outer_monotonic_remaining_sec=l.outer_deadline-self.clock.mono
        s.sdk_counts=list(l.template.baseline_sdk_counts)
        s.goal_terminal=self.goal_terminal;s.goal_status=3;s.goal_error_code=0
        s.goal_observed=l.armed_envelope is not None
        s.action_goal_id=l.armed_envelope.goal_id.id if l.armed_envelope is not None else ''
        s.action_goal_stamp_ns=l.armed_envelope.goal_id.stamp.to_nsec() if l.armed_envelope is not None else 0
        s.request_sequence=r.request_sequence
        s.sdk_stamp_ns=s.encoder_stamp_ns=self.clock.ros-1_000_000
        s.manual_reference=False
        for k,v in self.overrides.get(r.operation,{}).items():setattr(s,k,v)
        if r.operation==self.fail_op:return NS(success=False,code='INJECTED_REJECT',state=s)
        return NS(success=True,code='INJECTED_OK',state=s)


def case(*,clock=None,episode_duration=8.):
    clock=clock or Clock();f=proof_fixture();old=f['following_model']
    sample=dict(model_sha256=old.model_sha256,epoch_ns=old.epoch_ns,joint_names=old.joint_names,
        sdk_stamp_ns=9_990_000_000,accepted_stamp_ns=9_991_000_000,
        sdk_positions_rad=[0.]*6,accepted_positions_rad=(-np.array(old.bias_rad)).tolist())
    f['following_model']=source_bias_model(sample,binding_id='typed-synthetic-lease',source_bias_limit_rad=(.09,)*6)
    for side in ('wire_plan','physical_plan'):f[side].joint_trajectory.header.stamp=rospy.Time.from_sec(10.2)
    for side,p in (('wire_audit','wire_plan'),('physical_audit','physical_plan')):
        f[side]['trajectory_sha256']=wire_digest(f[p])
        f[side]['controller_constraints_snapshot']['goal_time']=1.
    f['wire_audit']['wire_reference_contract']=commanded_reference_bounds(f['wire_plan'],f['wire_reference'],.5)
    contract=bind_dual_reference(**f);events=[]
    driver=Peer('driver',clock,events);hardware=Peer('hardware',clock,events)
    lease=PhysicalReferenceLease(contract,request_factory=lambda:NS(action_goal=None),
        driver_call=driver,hardware_call=hardware,owner_token='owner',goal_token='goal',
        selection_generation=1,issued_ns=9_999_000_000,deadline_ns=30_000_000_000,
        maximum_duration_sec=20.,fresh_sample=sample,controller_stamp_ns=9_992_000_000,
        episode_id='typed-synthetic-episode',episode_deadline_ns=25_000_000_000,
        episode_duration_sec=episode_duration,clock=clock.now,ros_now_ns=clock.ros_now)
    driver.lease=hardware.lease=lease
    envelope=FollowJointTrajectoryActionGoal(header=Header(stamp=rospy.Time.from_sec(10.)),
        goal_id=GoalID(id='exact-armed-new-goal',stamp=rospy.Time.from_sec(10.)),goal=contract.action_goal())
    return NS(lease=lease,contract=contract,driver=driver,hardware=hardware,events=events,
              clock=clock,envelope=envelope)


def prepared(c):c.lease.prepare();c.lease.commit();return c


def releasing(c):
    prepared(c);c.lease.arm(c.envelope)
    for p in (c.driver,c.hardware):p.phase=c.lease.EXECUTING;p.goal_terminal=True
    c.lease.release_prepare()
    return c


def test_r22_prepare_commit_order_and_arm_queue_barrier():
    c=prepared(case());c.lease.arm(c.envelope)
    assert [(n,op) for n,op,_ in c.events]==[
        ('driver',1),('hardware',1),('driver',2),('hardware',2),
        ('hardware',3),('driver',0),('driver',3)]
    with pytest.raises(ValueError):c.lease.arm(c.envelope)


def test_r22_prepare_partial_failure_keeps_consumed_sequence_and_never_commits():
    c=case();c.hardware.fail_op=c.lease.PREPARE
    with pytest.raises(ValueError):c.lease.prepare()
    assert c.driver.phase==c.lease.PREPARED and not c.lease.released
    assert [(n,o) for n,o,_ in c.events]==[('driver',1),('hardware',1)]


def test_r22_query_never_renews_outer_or_episode_deadline():
    c=prepared(case());old=(c.lease.outer_deadline,c.lease.local_deadline)
    c.clock.mono+=.5;c.clock.ros+=500_000_000
    c.lease.query()
    assert (c.lease.outer_deadline,c.lease.local_deadline)==old
    req=c.events[-1][2]
    assert req.maximum_duration_sec<=19.5 and req.episode_duration_sec<=7.5


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_r22_nonfinite_monotonic_clock_cannot_create_lease(bad):
    clock=Clock();clock.mono=bad
    with pytest.raises(ValueError):case(clock=clock)


def test_r22_goal_must_fit_earlier_effective_monotonic_episode_budget():
    c=prepared(case(episode_duration=1.))
    with pytest.raises(ValueError):c.lease.arm(c.envelope)
    assert not any(op==c.lease.ARM for _,op,_ in c.events)


@pytest.mark.parametrize('fault',['evidence','stale','epoch'])
def test_r22_release_ack_requires_fresh_valid_bound_state(fault):
    c=releasing(case())
    if fault=='evidence':mutation={'evidence_valid':False}
    elif fault=='stale':mutation={'header':Header(stamp=rospy.Time.from_sec(1.))}
    else:mutation={'epoch_ns':c.lease.model.epoch_ns+1}
    c.hardware.overrides[c.lease.RELEASE_COMMIT]=mutation
    with pytest.raises(ValueError):c.lease.release_commit()
    assert not c.lease.released


def test_r22_successful_same_wire_release_finishes_lease():
    c=releasing(case());states=c.lease.release_commit()
    assert c.lease.released and all(s.phase==c.lease.LEGACY for s in states)
    with pytest.raises(ValueError):c.lease.query()


def test_r22_partial_publish_cancels_exact_armed_id_and_restores_hook(monkeypatch):
    c=prepared(case());events=[]
    monkeypatch.setattr(rospy,'get_time',lambda:c.clock.ros*1e-9)
    def publish(envelope):events.append(('publish',deepcopy(envelope)));raise RuntimeError('after publish')
    manager=NS(send_goal_fn=publish)
    manager.register_send_goal_fn=lambda fn:setattr(manager,'send_goal_fn',fn)
    cancelled=[]
    client=NS(action_client=NS(manager=manager,pub_cancel=NS(publish=lambda msg:cancelled.append(deepcopy(msg)))))
    client.send_goal=lambda goal:manager.send_goal_fn(deepcopy(c.envelope))
    client.cancel_goal=lambda:events.append(('wrong_old_handle_cancel',None))
    holds=[];hold=NS(get_num_connections=lambda:1,publish=lambda msg:holds.append(msg))
    submission=MappedTrajectorySubmission(c.contract,c.lease,revalidate=lambda:None)
    executor=ContractedTrajectoryExecutor(client,hold,clock=c.clock.now)
    with pytest.raises(RuntimeError,match='after publish'):
        executor.execute(c.contract.action_goal(),before_send=lambda:None,still_authorized=lambda:True,
                         mapped_submission=submission)
    assert [k for k,_ in events]==['publish']
    assert len(cancelled)==1 and cancelled[0].id=='exact-armed-new-goal' and cancelled[0].stamp.to_nsec()==0
    assert len(holds)==1 and not holds[0].points
    assert manager.send_goal_fn is publish and not executor.active


def test_r22_arm_transports_complete_original_wire_bytes_and_exact_inner_goal():
    c=prepared(case());c.lease.arm(c.envelope)
    armed=[r for _,op,r in c.events if op==c.lease.ARM]
    assert len(armed)==2
    for r in armed:
        assert message_bytes(r.action_goal)==message_bytes(c.envelope)
        assert message_bytes(r.wire_trajectory)==message_bytes(c.contract.wire_plan().joint_trajectory)
        assert r.wire_trajectory_sha256==c.contract.wire_trajectory_sha256
        assert r.stop_duration_sec==.5


@pytest.mark.parametrize('field',['goal_id','goal_stamp','proof','wire_digest','physical_digest','stop'])
def test_r22_typed_arm_ack_binds_exact_goal_and_proof(field):
    c=prepared(case())
    mapping={'goal_id':('action_goal_id','old-other-goal'),
        'goal_stamp':('action_goal_stamp_ns',1),'proof':('proof_sha256','f'*64),
        'wire_digest':('wire_trajectory_sha256','f'*64),
        'physical_digest':('physical_trajectory_sha256','f'*64),'stop':('stop_duration_sec',.4)}
    key,value=mapping[field];c.hardware.overrides[c.lease.ARM]={key:value}
    with pytest.raises(ValueError):c.lease.arm(c.envelope)
    assert not any(name=='driver' and op==c.lease.ARM for name,op,_ in c.events)


def test_r22_arm_waits_for_driver_queue_receipt_and_never_publishes_on_timeout(monkeypatch):
    c=prepared(case());c.hardware.overrides[c.lease.ARM]={'last_command_sequence':8}
    c.driver.overrides[c.lease.QUERY]={'last_command_sequence':7}
    def advance(dt):c.clock.mono+=dt;c.clock.ros+=round(dt*1e9)
    monkeypatch.setattr(client_module.time,'sleep',advance)
    old=(c.lease.outer_deadline,c.lease.local_deadline)
    with pytest.raises(ValueError,match='QUEUE_BARRIER_TIMEOUT'):c.lease.arm(c.envelope)
    assert not any(name=='driver' and op==c.lease.ARM for name,op,_ in c.events)
    assert c.lease.armed_envelope is not None and (c.lease.outer_deadline,c.lease.local_deadline)==old


def test_r22_post_arm_revalidation_cannot_slide_expired_future_start():
    c=prepared(case());calls=[];start=c.envelope.goal.trajectory.header.stamp.to_nsec()
    def revalidate():
        calls.append(True)
        if len(calls)==2:c.clock.ros+=250_000_000;c.clock.mono+=.25
    submission=MappedTrajectorySubmission(c.contract,c.lease,revalidate=revalidate)
    with pytest.raises(ValueError,match='ACTION_START_OR_DEADLINE'):submission.arm(c.envelope)
    assert len(calls)==2 and c.envelope.goal.trajectory.header.stamp.to_nsec()==start


def expired_manual(c=None):
    c=c or prepared(case());c.clock.mono+=21.;c.clock.ros+=21_000_000_000
    words=[2049,2048,2048,2048,2048,2048]
    for peer in (c.driver,c.hardware):
        peer.phase=c.lease.MANUAL
        for op in (c.lease.QUERY,c.lease.RELEASE_PREPARE,c.lease.RELEASE_COMMIT):
            peer.overrides[op]={'manual_reference':True,'sdk_counts':words,
                'sdk_stamp_ns':c.clock.ros-1,'encoder_stamp_ns':c.clock.ros-1}
    return c


def test_r22_expired_manual_cleanup_only_releases_software_with_same_words():
    c=expired_manual();old=(c.lease.outer_deadline,c.lease.local_deadline,
        c.lease.template.deadline_ns,c.lease.template.episode_deadline_ns)
    c.events.clear()
    c.lease.manual_cleanup_prepare();c.lease.manual_cleanup_commit()
    assert [(n,op) for n,op,_ in c.events]==[
        ('driver',0),('hardware',0),('hardware',5),('driver',5),('driver',6),('hardware',6)]
    assert c.lease.released and c.lease.release_counts==(2049,2048,2048,2048,2048,2048)
    assert (c.lease.outer_deadline,c.lease.local_deadline,
        c.lease.template.deadline_ns,c.lease.template.episode_deadline_ns)==old
    with pytest.raises(ValueError):c.lease.arm(c.envelope)
    with pytest.raises(ValueError):c.lease.manual_cleanup_query()


@pytest.mark.parametrize('fault',['manual_proof','stale_sdk','stale_encoder','epoch',
    'owner','binding','active','pending','stationary','different_words'])
def test_r22_manual_cleanup_rejects_wrong_or_stale_owner_evidence(fault):
    c=expired_manual();over=c.hardware.overrides[c.lease.QUERY]
    if fault=='manual_proof':over['manual_reference']=False
    elif fault=='stale_sdk':over['sdk_stamp_ns']=c.clock.ros-500_000_001
    elif fault=='stale_encoder':over['encoder_stamp_ns']=c.clock.ros-500_000_001
    elif fault=='epoch':over['epoch_ns']=c.lease.model.epoch_ns+1
    elif fault=='owner':over['owner_token']='different-owner'
    elif fault=='binding':over['binding_id']='new-active-binding'
    elif fault=='active':c.hardware.phase=c.lease.ACTIVE_IDLE
    elif fault=='pending':over['pending_complete']=False
    elif fault=='stationary':over['stationary']=False
    else:over['sdk_counts']=[2048]*6
    c.events.clear()
    with pytest.raises(ValueError):c.lease.manual_cleanup_prepare()
    assert not any(op==c.lease.RELEASE_PREPARE for _,op,_ in c.events)
    assert not c.lease.released


def test_r22_manual_cleanup_final_ack_cannot_change_the_manual_sdk_words():
    c=expired_manual();c.lease.manual_cleanup_prepare()
    c.hardware.overrides[c.lease.RELEASE_COMMIT]['sdk_counts']=[2048]*6
    with pytest.raises(ValueError,match='NOT_CONTINUOUS'):c.lease.manual_cleanup_commit()
    assert not c.lease.released


def r26_advance(clock, nanoseconds):
    """Advance an explicit synthetic clock; never sleep or contact ROS."""
    clock.ros += nanoseconds
    clock.mono += nanoseconds*1e-9


def r26_source_sample(c):
    model=c.contract.following_model
    return dict(model_sha256=model.model_sha256,epoch_ns=model.epoch_ns,
        joint_names=list(model.joint_names),
        sdk_stamp_ns=c.clock.ros-1_000_000,accepted_stamp_ns=c.clock.ros-2_000_000,
        sdk_positions_rad=list(model.source_sdk_positions_rad),
        accepted_positions_rad=list(model.source_accepted_positions_rad))


def r26_lease_inputs(c):
    return dict(owner_token='r26-owner',goal_token='r26-goal',selection_generation=1,
        issued_ns=c.clock.ros,deadline_ns=c.clock.ros+20_000_000_000,
        maximum_duration_sec=20.,fresh_sample=None,controller_stamp_ns=None,
        episode_id='r26-synthetic-episode',episode_deadline_ns=c.clock.ros+8_000_000_000,
        episode_duration_sec=8.,clock=c.clock.now,
        outer_monotonic_deadline=c.clock.mono+20.,
        episode_monotonic_deadline=c.clock.mono+8.)


class R26Peer(Peer):
    """Reuse the protocol double with actual slotted ROS request messages."""
    def __call__(self, request):
        original=self.lease
        if hasattr(original.template,'__dict__'):
            return super().__call__(request)
        # Only the fake peer's read view is adapted. The real lease/template
        # and the actual request passed to its transport remain untouched.
        self.lease=NS(template=NS(**{name:getattr(original.template,name)
            for name in original.template.__slots__}),
            local_deadline=original.local_deadline,outer_deadline=original.outer_deadline,
            armed_envelope=original.armed_envelope)
        try:
            return super().__call__(request)
        finally:
            self.lease=original


def r26_transport(monkeypatch, clock, *, wait_ns=310_000_000,
                  proof_ns=610_000_000, after_validation=None):
    """Use real connect/contract validation with only ROS transport doubled.

    Geometry proof values are the existing explicit synthetic fixture. The
    complete contract validator still runs; its first call then consumes a
    controlled amount of time. Later validations also run, without adding a
    second unrelated delay. No service proxy can reach a ROS master/device.
    """
    order=[];events=[];waits=[];validations=[]
    peers={path:R26Peer(name,clock,events) for path,name in (
        ('/alicia_d/physical_reference','driver'),
        ('/bessica_d_hw_interface/physical_reference','hardware'))}
    monkeypatch.setattr(client_module,'time',NS(monotonic=clock.now,perf_counter=clock.now))
    monkeypatch.setattr(rospy.Time,'now',staticmethod(
        lambda:rospy.Time(clock.ros//10**9,clock.ros%10**9)))
    def wait(path,timeout):
        assert path in peers and 0.<timeout<=2.
        waits.append(path);order.append(('wait',path));r26_advance(clock,wait_ns)
    monkeypatch.setattr(rospy,'wait_for_service',wait)
    monkeypatch.setattr(rospy,'ServiceProxy',lambda path,*a,**kw:peers[path])
    original=client_module.validate_dual_reference
    def validate(contract,*args,**kwargs):
        result=original(contract,*args,**kwargs)
        validations.append(contract.proof_sha256)
        if len(validations)==1:
            r26_advance(clock,proof_ns)
            if after_validation is not None:after_validation()
        order.append(('validated',clock.ros))
        return result
    monkeypatch.setattr(client_module,'validate_dual_reference',validate)
    return NS(order=order,events=events,peers=peers,waits=waits,validations=validations)


@pytest.mark.parametrize('wait_ns,proof_ns',[
    (310_000_000,0),(0,610_000_000),(310_000_000,610_000_000)])
def test_r26_connect_samples_after_discovery_and_complete_proof_without_renewal(
        monkeypatch,wait_ns,proof_ns):
    c=case();inputs=r26_lease_inputs(c);old_inputs=dict(inputs)
    old_contract=c.contract;old_wire=message_bytes(c.contract.wire_plan())
    old_model=c.contract.following_model.digest;captured=[]
    io=r26_transport(monkeypatch,c.clock,wait_ns=wait_ns,proof_ns=proof_ns)
    def snapshot():
        assert len(io.waits)==2 and len(io.validations)==1
        io.order.append(('source',c.clock.ros))
        sample=r26_source_sample(c);controller=c.clock.ros-3_000_000
        captured.append(deepcopy((sample,controller)))
        return sample,controller
    lease=PhysicalReferenceLease.connect(c.contract,source_snapshot=snapshot,**inputs)
    assert len(captured)==1 and io.order[-1][0]=='source'
    sample,controller=captured[0]
    assert lease.template.source_sdk_stamp_ns==sample['sdk_stamp_ns']
    assert lease.template.source_encoder_stamp_ns==sample['accepted_stamp_ns']
    assert lease.template.controller_stamp_ns==controller
    assert lease.template.issued_ns==c.clock.ros>inputs['issued_ns']
    assert 0<=c.clock.ros-lease.template.issued_ns<=500_000_000
    assert lease.template.deadline_ns==inputs['deadline_ns']
    assert lease.template.episode_deadline_ns==inputs['episode_deadline_ns']
    assert lease.outer_deadline==pytest.approx(inputs['outer_monotonic_deadline'])
    assert lease.local_deadline==pytest.approx(inputs['episode_monotonic_deadline'])
    elapsed=(2*wait_ns+proof_ns)*1e-9
    assert lease.template.maximum_duration_sec==pytest.approx(20.-elapsed)
    assert lease.template.episode_duration_sec==pytest.approx(8.-elapsed)
    assert c.contract==old_contract and c.contract.following_model.digest==old_model
    assert message_bytes(c.contract.wire_plan())==old_wire
    assert inputs==old_inputs and not io.events


@pytest.mark.parametrize('wait_ns,proof_ns',[
    (310_000_000,0),(0,610_000_000),(310_000_000,610_000_000)])
def test_r26_explicit_old_source_is_not_relabelled_after_connect_cost(
        monkeypatch,wait_ns,proof_ns):
    c=case();inputs=r26_lease_inputs(c)
    inputs.update(fresh_sample=r26_source_sample(c),controller_stamp_ns=c.clock.ros-3_000_000)
    old=deepcopy(inputs['fresh_sample'])
    io=r26_transport(monkeypatch,c.clock,wait_ns=wait_ns,proof_ns=proof_ns)
    with pytest.raises(ValueError,match='PHYSICAL_LEASE_SOURCE_STALE'):
        PhysicalReferenceLease.connect(c.contract,**inputs)
    assert inputs['fresh_sample']==old and not io.events


@pytest.mark.parametrize('mode',['explicit','snapshot'])
@pytest.mark.parametrize('field',['sdk_stamp_ns','accepted_stamp_ns','controller_stamp_ns'])
@pytest.mark.parametrize('age_ns',[499_999_999,500_000_000,500_000_001,-1])
def test_r26_each_lease_source_preserves_exact_half_second_boundary(mode,field,age_ns):
    c=case();r26_advance(c.clock,1_000_000_000)
    inputs=r26_lease_inputs(c);sample=r26_source_sample(c);controller=c.clock.ros-1
    inputs['issued_ns']-=3_000_000
    if field=='controller_stamp_ns':controller=c.clock.ros-age_ns
    else:sample[field]=c.clock.ros-age_ns
    calls=[];transport=[]
    def snapshot():calls.append(True);return deepcopy(sample),controller
    if mode=='snapshot':inputs['source_snapshot']=snapshot
    else:inputs.update(fresh_sample=sample,controller_stamp_ns=controller)
    def forbidden(request):transport.append(request);raise AssertionError('constructor must not send')
    def construct():
        return PhysicalReferenceLease(c.contract,request_factory=lambda:NS(action_goal=None),
            driver_call=forbidden,hardware_call=forbidden,ros_now_ns=c.clock.ros_now,**inputs)
    if 0<=age_ns<=500_000_000:
        lease=construct()
        assert lease.template.source_sdk_stamp_ns==sample['sdk_stamp_ns']
        assert lease.template.source_encoder_stamp_ns==sample['accepted_stamp_ns']
        assert lease.template.controller_stamp_ns==controller
        assert lease.template.issued_ns==(c.clock.ros if mode=='snapshot' else inputs['issued_ns'])
    else:
        with pytest.raises(ValueError,match='PHYSICAL_LEASE_SOURCE_STALE'):construct()
    assert len(calls)==int(mode=='snapshot') and not transport


@pytest.mark.parametrize('fault',['explicit_sample','explicit_controller','not_callable',
                                  'missing_outer_clock','missing_episode_clock'])
def test_r26_late_source_requires_exclusive_provider_and_original_clocks(fault):
    c=case();inputs=r26_lease_inputs(c);calls=[];transport=[]
    def snapshot():calls.append(True);return r26_source_sample(c),c.clock.ros
    inputs['source_snapshot']=snapshot
    if fault=='explicit_sample':inputs['fresh_sample']=r26_source_sample(c)
    elif fault=='explicit_controller':inputs['controller_stamp_ns']=c.clock.ros
    elif fault=='not_callable':inputs['source_snapshot']=True
    elif fault=='missing_outer_clock':inputs['outer_monotonic_deadline']=None
    else:inputs['episode_monotonic_deadline']=None
    with pytest.raises(ValueError):
        PhysicalReferenceLease(c.contract,request_factory=lambda:NS(action_goal=None),
            driver_call=lambda r:transport.append(r),hardware_call=lambda r:transport.append(r),
            ros_now_ns=c.clock.ros_now,**inputs)
    assert not calls and not transport


@pytest.mark.parametrize('fault',['sdk_vector','accepted_vector','epoch','model','provider_exception'])
def test_r26_new_timestamps_do_not_replace_frozen_source_or_authorize_prepare(monkeypatch,fault):
    c=case();inputs=r26_lease_inputs(c);before=c.contract.following_model.digest;calls=[]
    io=r26_transport(monkeypatch,c.clock)
    def snapshot():
        calls.append(True);sample=r26_source_sample(c)
        if fault=='provider_exception':raise RuntimeError('synthetic source collection failure')
        if fault=='sdk_vector':sample['sdk_positions_rad'][0]+=2*math.pi/4096
        elif fault=='accepted_vector':sample['accepted_positions_rad'][1]+=.001
        elif fault=='epoch':sample['epoch_ns']+=1
        elif fault=='model':sample['model_sha256']='f'*64
        return sample,c.clock.ros-1
    with pytest.raises((ValueError,RuntimeError)):
        PhysicalReferenceLease.connect(c.contract,source_snapshot=snapshot,**inputs)
    assert calls==[True] and c.contract.following_model.digest==before and not io.events


def test_r26_nonzero_snapshot_cost_precedes_issuance_and_consumes_original_budgets(monkeypatch):
    c=case();inputs=r26_lease_inputs(c);original_inputs=dict(inputs)
    model=c.contract.following_model.digest;bias=c.contract.following_model.bias_rad
    wire=message_bytes(c.contract.wire_plan());timeline=[];captured=[]
    wait_ns=310_000_000;proof_ns=610_000_000;collect_ns=410_000_000;finish_ns=190_000_000
    io=r26_transport(monkeypatch,c.clock,wait_ns=wait_ns,proof_ns=proof_ns)
    def snapshot():
        assert len(io.waits)==2 and len(io.validations)==1
        timeline.append(('entered',c.clock.ros,c.clock.mono))
        r26_advance(c.clock,collect_ns)
        sample=r26_source_sample(c);controller=c.clock.ros-3_000_000
        captured.append(deepcopy((sample,controller)))
        r26_advance(c.clock,finish_ns)
        timeline.append(('returned',c.clock.ros,c.clock.mono))
        return sample,controller
    lease=PhysicalReferenceLease.connect(c.contract,source_snapshot=snapshot,**inputs)
    elapsed_ns=2*wait_ns+proof_ns+collect_ns+finish_ns
    assert len(captured)==1 and len(timeline)==2
    assert lease.template.issued_ns==timeline[1][1]==inputs['issued_ns']+elapsed_ns
    assert lease.template.issued_ns>timeline[0][1]>inputs['issued_ns']
    sample,controller=captured[0]
    assert lease.template.source_sdk_stamp_ns==sample['sdk_stamp_ns']
    assert lease.template.source_encoder_stamp_ns==sample['accepted_stamp_ns']
    assert lease.template.controller_stamp_ns==controller
    assert c.clock.ros-sample['sdk_stamp_ns']==finish_ns+1_000_000
    assert c.clock.ros-sample['accepted_stamp_ns']==finish_ns+2_000_000
    assert c.clock.ros-controller==finish_ns+3_000_000
    assert lease.template.deadline_ns==inputs['deadline_ns']
    assert lease.template.episode_deadline_ns==inputs['episode_deadline_ns']
    assert lease.outer_deadline==pytest.approx(inputs['outer_monotonic_deadline'])
    assert lease.local_deadline==pytest.approx(inputs['episode_monotonic_deadline'])
    assert lease.template.maximum_duration_sec==pytest.approx(20.-elapsed_ns*1e-9)
    assert lease.template.episode_duration_sec==pytest.approx(8.-elapsed_ns*1e-9)
    admission=lease.source_admission
    assert admission['service_wait_elapsed_sec']==pytest.approx(2*wait_ns*1e-9)
    assert admission['contract_validation_elapsed_sec']==pytest.approx(proof_ns*1e-9)
    assert admission['source_snapshot_elapsed_sec']==pytest.approx((collect_ns+finish_ns)*1e-9)
    assert admission['setup_issued_ns']==inputs['issued_ns']
    assert admission['issued_ns']==admission['checked_ns']==timeline[1][1]
    assert admission['passed'] is True and not io.events
    assert inputs==original_inputs and c.contract.following_model.digest==model
    assert tuple(lease.template.bias_rad)==bias and message_bytes(c.contract.wire_plan())==wire
    for peer in io.peers.values():peer.lease=lease
    lease.prepare()
    assert [op for _,op,_ in io.events]==[lease.PREPARE,lease.PREPARE]
    assert all(request.issued_ns==timeline[1][1] for _,_,request in io.events)


@pytest.mark.parametrize('deadline_kind',['episode','outer'])
@pytest.mark.parametrize('clock_kind',['ros','monotonic'])
def test_r26_snapshot_crossing_original_deadline_cannot_construct_or_prepare(
        monkeypatch,deadline_kind,clock_kind):
    c=case();inputs=r26_lease_inputs(c);timeline=[];captured=[]
    inputs.update(episode_deadline_ns=c.clock.ros+1_500_000_000,
        episode_duration_sec=1.5,episode_monotonic_deadline=c.clock.mono+1.5)
    if deadline_kind=='outer':
        inputs.update(deadline_ns=inputs['episode_deadline_ns'],maximum_duration_sec=1.5,
            outer_monotonic_deadline=inputs['episode_monotonic_deadline'])
    original_inputs=dict(inputs);model=c.contract.following_model.digest
    wire=message_bytes(c.contract.wire_plan());io=r26_transport(monkeypatch,c.clock)
    ros_deadline=inputs['episode_deadline_ns' if deadline_kind=='episode' else 'deadline_ns']
    mono_deadline=inputs[deadline_kind+'_monotonic_deadline']
    def snapshot():
        assert len(io.waits)==2 and len(io.validations)==1
        assert c.clock.ros<ros_deadline and c.clock.mono<mono_deadline
        timeline.append(('entered',c.clock.ros,c.clock.mono))
        sample=r26_source_sample(c);controller=c.clock.ros-3_000_000
        captured.append(deepcopy((sample,controller)))
        if clock_kind=='ros':c.clock.ros=ros_deadline+1
        else:c.clock.mono=mono_deadline+.000001
        timeline.append(('returned',c.clock.ros,c.clock.mono))
        return sample,controller
    expected=('PHYSICAL_LEASE_SOURCE_STALE' if clock_kind=='ros'
        else 'PHYSICAL_ORIGINAL_MONOTONIC_DEADLINE_INVALID')
    with pytest.raises(ValueError,match=expected) as rejected:
        PhysicalReferenceLease.connect(c.contract,source_snapshot=snapshot,**inputs)
    assert len(timeline)==2 and len(captured)==1
    sample,controller=captured[0]
    assert all(0<=c.clock.ros-stamp<=500_000_000
        for stamp in (sample['sdk_stamp_ns'],sample['accepted_stamp_ns'],controller))
    admission=rejected.value.source_admission
    assert admission['passed'] is False and admission['source_snapshot_used'] is True
    assert admission['setup_issued_ns']==inputs['issued_ns']
    assert admission['issued_ns']==admission['checked_ns']==timeline[1][1]
    assert admission['source_stamps_ns']==dict(sdk_stamp_ns=sample['sdk_stamp_ns'],
        accepted_stamp_ns=sample['accepted_stamp_ns'],controller_stamp_ns=controller)
    assert admission['source_ages_ns']=={key:c.clock.ros-stamp
        for key,stamp in admission['source_stamps_ns'].items()}
    assert admission['maximum_source_age_ns']==500_000_000
    assert admission['original_outer_ros_deadline_ns']==inputs['deadline_ns']
    assert admission['original_episode_ros_deadline_ns']==inputs['episode_deadline_ns']
    assert admission['original_outer_monotonic_deadline']==inputs['outer_monotonic_deadline']
    assert admission['original_episode_monotonic_deadline']==inputs['episode_monotonic_deadline']
    assert admission['service_wait_elapsed_sec']==pytest.approx(.62)
    assert admission['contract_validation_elapsed_sec']==pytest.approx(.61)
    assert admission['source_snapshot_elapsed_sec']==pytest.approx(timeline[1][2]-timeline[0][2])
    assert not io.events and all(peer.lease is None for peer in io.peers.values())
    assert inputs==original_inputs and c.contract.following_model.digest==model
    assert message_bytes(c.contract.wire_plan())==wire
