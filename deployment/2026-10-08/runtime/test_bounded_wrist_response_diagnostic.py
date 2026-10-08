"""Offline wrist contracts and publication stub tests; never hardware."""
import importlib.util
import itertools
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import numpy as np
import pytest
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import bounded_wrist_response_diagnostic as m


def q0():return np.array([-.5,.4,-.3,.1,-.4,.03,.04975])
def contract(joint=4,direction=1):return m.WristContract(q0(),q0(),100,joint,direction)


@pytest.mark.parametrize('joint,direction',itertools.product((4,6),(-1,1)))
def test_two_commands_restore_exact_original_sdk_and_keep_initial_interval(joint,direction):
    c=contract(joint,direction);current=c.reference.copy();initial=None
    for i,target in enumerate(c.targets):
        c.begin(m.LABELS[i],target,current,current,100,1000+i*100)
        if initial is None:initial=(c.active['lower'].copy(),c.active['upper'].copy())
        assert np.array_equal(c.active['lower'],initial[0]) and np.array_equal(c.active['upper'],initial[1])
        c.observe_sdk(target,1001+i*100,100);c.complete(target,100,True);current=target.copy()
    assert c.count==2 and np.array_equal(current,c.reference)
    expected=np.zeros(6,dtype=int);expected[joint-1]=direction*22
    assert np.array_equal(np.array(m.words(c.targets[0]))-m.words(c.reference),expected)
    with pytest.raises(RuntimeError,match='exhausted'):c.begin('outbound',c.targets[0],current,current,100,2000)


@pytest.mark.parametrize('joint',(4,6))
def test_sdk_bias_on_unedited_axis_never_expands_actual_permission(joint):
    q=q0();sdk=q.copy();sdk[2]+=math.radians(.615234375)
    c=m.WristContract(q,sdk,100,joint)
    c.begin('outbound',c.targets[0],q,sdk,100,1000)
    c.observe_sdk(c.targets[0],1001,100)
    actual=q.copy();actual[joint-1]=c.targets[0][joint-1];c.complete(actual,100,True)
    assert c.active['upper'][2]==pytest.approx(q[2]+math.radians(.4))
    actual[2]=q[2]+math.radians(.401)
    with pytest.raises(RuntimeError,match='unedited'):c.assert_feedback(actual,100)
    assert c.failed


@pytest.mark.parametrize('joint',(4,6))
@pytest.mark.parametrize('failure',('edited','held','gripper','nan','epoch','reverse'))
def test_feedback_escapes_latch_without_recovery(joint,failure):
    c=contract(joint);c.begin('outbound',c.targets[0],c.origin,c.reference,100,1000)
    q=c.origin.copy();epoch=100
    if failure=='edited':q[joint-1]+=math.radians(4.801)
    if failure=='held':q[0]+=math.radians(.401)
    if failure=='gripper':q[6]-=.00001
    if failure=='nan':q[1]=np.nan
    if failure=='epoch':epoch=101
    if failure=='reverse':q[joint-1]-=math.radians(.501)
    with pytest.raises(RuntimeError):c.assert_feedback(q,epoch)
    with pytest.raises(RuntimeError):c.assert_feedback(c.origin,100)


@pytest.mark.parametrize('failure',('order','held_command','gripper','wrong_target','predecessor','no_settle'))
def test_command_contract_rejects_nonliteral_sequence(failure):
    c=contract();target=c.targets[0].copy();ref=c.reference.copy();label='outbound'
    if failure=='order':label='return_original_sdk'
    if failure=='held_command':target[2]+=m.QUANTUM
    if failure=='gripper':target[6]-=.001
    if failure=='wrong_target':target[3]+=m.QUANTUM
    if failure=='predecessor':ref[3]+=m.QUANTUM
    if failure=='no_settle':
        c.begin(label,target,c.origin,ref,100,1000);target=c.reference;ref=c.targets[0];label='return_original_sdk'
    with pytest.raises(RuntimeError):c.begin(label,target,c.origin,ref,100,1100)


@pytest.mark.parametrize('failure',('held_word','edited_overshoot','gripper','regression','precommand','changed_after_target'))
def test_sdk_monitor_exact_single_axis(failure):
    c=contract();c.begin('outbound',c.targets[0],c.origin,c.reference,100,1000)
    q=c.reference.copy();stamp=1001
    if failure=='held_word':q[2]+=m.QUANTUM
    if failure=='edited_overshoot':q[3]+=23*m.QUANTUM
    if failure=='gripper':q[6]-=.001
    if failure=='regression':c.observe_sdk(q,1002,100)
    if failure=='precommand':q=c.targets[0];stamp=999
    if failure=='changed_after_target':c.observe_sdk(c.targets[0],1001,100);stamp=1002
    with pytest.raises(RuntimeError):c.observe_sdk(q,stamp,100)
    assert c.failed


@pytest.mark.parametrize('joint',(4,6))
def test_full_physical_and_nominal_cells_cover_all_axes(joint):
    q=q0();sdk=q.copy();sdk[2]+=math.radians(.615234375);c=m.WristContract(q,sdk,100,joint)
    lo,hi,_=m.segment_rectangle(q,c.targets[0],sdk,q,joint)
    assert hi[joint-1]-lo[joint-1]==pytest.approx(math.radians(9.6))
    held=[j for j in range(6) if j!=joint-1]
    assert np.allclose((hi-lo)[held],math.radians(.8))
    for lower,upper in ((lo,hi),(np.vstack((q[:6],sdk[:6],c.targets[0][:6])).min(0),np.vstack((q[:6],sdk[:6],c.targets[0][:6])).max(0))):
        axes=m.rectangle_grid(lower,upper,joint)
        cells=list(m.rectangle_cells(axes))
        assert max(r for _,r in cells)<=math.radians(1.3)+1e-12
        for indices,(center,radius) in zip(itertools.product(*(range(len(a)-1) for a in axes)),cells):
            for bits in itertools.product((0,1),repeat=6):
                corner=np.array([a[i+b] for a,i,b in zip(axes,indices,bits)])
                assert np.sum(abs(corner-center))<=radius+1e-12


@pytest.mark.parametrize('argv',[[],['--joint','3'],['--joint','5'],['--joint','4','--step-deg','3'],['--joint','4','--bootstrap-pending'],['--joint','6','--plan','old'],['--joint','4','--enable'],['--joint','4','--direction','0']])
def test_cli_cannot_expand_scope(argv):
    with pytest.raises(SystemExit):m.parse_args(argv)


@pytest.mark.parametrize('joint',(4,6))
def test_default_prepare_has_no_joint_command_publisher(joint,tmp_path):
    class Base:
        class Diagnostic:
            def __init__(self,args,directory):self.args=args;self.publisher=object() if args.execute else None
    cls=m.make_diagnostic_class(Base);obj=cls(m.parse_args(['--joint',str(joint)]),tmp_path)
    assert obj.publisher is None
    with pytest.raises(RuntimeError,match='cannot publish'):obj.execute_step('outbound',q0())
    with pytest.raises(RuntimeError,match='no bootstrap'):obj.bootstrap_once()


@pytest.mark.parametrize('channel',('accepted','sdk','board'))
def test_all_sources_must_follow_epoch_and_original_age(channel):
    values=dict(accepted=1000,sdk=1000,board=1000);values[channel]=99
    with pytest.raises(RuntimeError):m.validate_source_binding(1100,100,values['accepted'],values['sdk'],values['board'])
    m.validate_source_binding(350001000,100,350001000,350001000,1000)
    with pytest.raises(RuntimeError):m.validate_source_binding(350001001,100,350001001,350001001,1000)


def test_fresh_scene_keeps_each_corner_identity():
    lower=np.array([[.2,-.01,-.02],[.3,.02,-.01],[.22,-.02,.03],[.32,.01,.04]])
    proof=dict(minimum_plane_clearance_lower_bound_m=.1,maximum_cad_distance_to_board_origin_m=1.,camera_link_corner_union_lower_m=lower,camera_link_corner_union_upper_m=lower+.001)
    old=np.eye(4);fresh=old.copy();a=math.radians(.1);fresh[:3,:3]=[[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]]
    model=SimpleNamespace(evidence={'color_intrinsics':dict(width=640,height=480)})
    observed=[]
    def project(model,lo,hi):observed.append((lo,hi));return np.array([[40,500],[40,400]])
    m.verify_fresh_scene(proof,old,fresh,model,project,(lower[None],(lower+.001)[None]))
    assert np.array_equal(observed[0][0][0,0],lower[0])
    assert np.all(observed[0][0][0,-1]<lower[-1])


def publication_fixture(monkeypatch,joint):
    import bounded_joint_response_diagnostic as base
    cls=m.make_diagnostic_class(base);obj=cls.__new__(cls)
    obj.args=m.parse_args(['--joint',str(joint),'--execute']);obj.contract=contract(joint)
    obj.epoch=obj.frozen_epoch=8_000_000_000;obj.started_ns=9_000_000_000
    obj.contract.epoch=obj.epoch;obj.lock=threading.RLock()
    obj.guard=lambda:None;obj.stationary=lambda _:{};obj.driver_connection=lambda:None
    def msg(q):
        a=base.JointState(name=base.NAMES,position=q.tolist());a.header.stamp=base.rospy.Time(9,990_000_000);return a
    obj.accepted=[msg(obj.contract.origin)];obj.reference=[msg(obj.contract.reference)]
    geo=dict(Y=np.eye(4),X=np.eye(4),model=object(),source_stamp_ns=9_990_000_000)
    obj.snapshot=lambda _:geo
    lower,upper,_=m.segment_rectangle(obj.contract.origin,obj.contract.targets[0],obj.contract.reference,obj.contract.origin,joint)
    obj.path=lambda *a:dict(lower_rad=lower.tolist(),upper_rad=upper.tolist(),cell_enclosure_archive='offline')
    obj.geometry_batches={'offline':(np.ones((1,4,3)),np.ones((1,4,3)))}
    obj.publish_count=0;obj.expected_words=m.words(obj.contract.reference)
    obj.event=lambda *a,**k:None
    monkeypatch.setattr(m,'verify_fresh_scene',lambda *a:{})
    monkeypatch.setattr(base.rospy.Time,'now',lambda:base.rospy.Time(10,0))
    return base,obj


@pytest.mark.parametrize('joint',(4,6))
def test_real_execute_code_only_constructs_one_hot_wrist_command(monkeypatch,joint):
    base,obj=publication_fixture(monkeypatch,joint);sent=[]
    class Published(Exception):pass
    def publish(msg):sent.append(msg);raise Published()
    obj.publisher=SimpleNamespace(get_num_connections=lambda:1,publish=publish)
    with pytest.raises(Published):obj.execute_step('outbound',obj.contract.targets[0])
    assert obj.publish_count==obj.contract.count==len(sent)==1
    msg=sent[0];effort=[0.]*7;effort[joint-1]=1.
    assert msg.header.frame_id=='gui_direct' and msg.effort==effort
    assert np.array_equal(msg.position,obj.contract.targets[0])
    assert msg.header.stamp.to_nsec()==obj.contract.active['command_stamp_ns']==10_000_000_000


def test_expiry_during_command_construction_publishes_nothing(monkeypatch):
    base,obj=publication_fixture(monkeypatch,4);sent=[]
    obj.publisher=SimpleNamespace(get_num_connections=lambda:1,publish=sent.append)
    geo=dict(Y=np.eye(4),X=np.eye(4),model=object(),source_stamp_ns=10_000_000_000)
    obj.snapshot=lambda _:geo
    obj.accepted[0].header.stamp=obj.reference[0].header.stamp=base.rospy.Time(10,300_000_000)
    now=iter((base.rospy.Time(10,349_000_000),base.rospy.Time(10,350_000_001)))
    monkeypatch.setattr(base.rospy.Time,'now',lambda:next(now))
    with pytest.raises(RuntimeError,match='board source'):obj.execute_step('outbound',obj.contract.targets[0])
    assert obj.contract.count==1 and obj.publish_count==0 and not sent


def test_run_failure_never_invokes_return(monkeypatch,tmp_path):
    import bounded_joint_response_diagnostic as base
    cls=m.make_diagnostic_class(base);obj=cls.__new__(cls);q=q0()
    obj.args=m.parse_args(['--joint','4','--execute']);obj.lock=threading.RLock();obj.guard=lambda:None
    obj.await_initial=lambda:(q,{});obj.snapshot=lambda _:{};obj.frozen_epoch=100
    msg=base.JointState(name=base.NAMES,position=q.tolist());obj.accepted=[msg];obj.out=tmp_path
    seen=[]
    def execute(label,target):seen.append(label);raise RuntimeError('outbound fault')
    obj.execute_step=execute
    with pytest.raises(RuntimeError,match='outbound fault'):obj.run()
    assert seen==['outbound']


@pytest.mark.parametrize('joint,direction',itertools.product((4,6),(-1,1)))
def test_publication_start_drift_keeps_independent_half_degree_reverse_limit(joint,direction):
    c=contract(joint,direction)
    start=c.origin.copy();start[joint-1]+=direction*math.radians(.1)
    c.begin('outbound',c.targets[0],start,c.reference,100,1000)
    # This point remains inside the INITIAL fixed absolute geometry box, but is
    # 0.6 degree reverse from the true command start, so must fail independently.
    q=c.origin.copy();q[joint-1]-=direction*math.radians(.5)
    assert np.all(q[:6]>=c.active['lower']) and np.all(q[:6]<=c.active['upper'])
    with pytest.raises(RuntimeError,match='reverse'):
        c.assert_feedback(q,100)
    assert c.failed
