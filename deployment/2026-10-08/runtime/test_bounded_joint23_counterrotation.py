"""Offline contract and full-cell coverage tests; no ROS imports or hardware."""
import importlib.util
import itertools
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import threading

import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
spec=importlib.util.spec_from_file_location('counterrotation',HERE/'bounded_joint23_counterrotation.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def baseline():
    return np.array([-.5,.4,-.3,.1,-.4,.03,.04975])


def contract(step=2.):
    q=baseline();return m.CounterrotationContract(q,q,100,step)


def send(c,index,measured=None,reference=None):
    measured=c.reference if measured is None else measured
    reference=c.reference if reference is None else reference
    return c.begin(m.LABELS[index],c.targets[index],measured,reference,c.epoch,1000+100*index)


def reserve(c, axis, measured=None, reference=None, stamp=1001):
    measured=c.active['start'] if measured is None else measured
    reference=c.reference if reference is None else reference
    return c.reserve_wire(axis,reference,measured,c.epoch,stamp,stamp-1,stamp-1,stamp-1)


def finish_leg(c):
    current=c.active['start'].copy()
    sdk=c.reference.copy() if c.count==1 else c.targets[c.count-2].copy()
    for part,axis in enumerate(m.EDITED):
        stamp=c.active['command_stamp_ns']+10*part+1
        payload,effort=reserve(c,axis,current,sdk,stamp)
        assert sum(effort)==1. and effort[axis]==1.
        current[axis]=payload[axis];sdk=payload.copy()
        c.observe_sdk(sdk,stamp+1,c.epoch)
        c.confirm_wire_stationary(current,c.epoch,True)
    c.complete(current,c.epoch,True)
    return current


@pytest.mark.parametrize('step,count',[(2.,22),(3.,34),(4.,45)])
def test_quantized_four_command_sequence_returns_exact_sdk(step,count):
    c=contract(step);current=c.reference.copy()
    for index,target in enumerate(c.targets):
        previous=current.copy()
        send(c,index,current,previous)
        current=finish_leg(c)
        c.assert_feedback(target,c.epoch)
    assert c.step_words==count and c.count==4 and c.wire_count==8 and c.maximum_commands==8
    np.testing.assert_array_equal(current,c.reference)
    with pytest.raises(RuntimeError,match='exhausted'):
        c.begin('positive',c.targets[0],current,current,c.epoch,2000)
    assert c.failed


@pytest.mark.parametrize('direction,index',[(1,0),(-1,2)])
def test_sdk_word_changes_equal_and_opposite(direction,index):
    c=contract();delta=np.array(m.words(c.targets[index]))-m.words(c.reference)
    np.testing.assert_array_equal(delta,[0,direction*22,-direction*22,0,0,0])
    assert c.targets[index][6]==c.reference[6]


def test_actual_rotation_does_not_follow_sdk_claim():
    def fk(q):
        theta=q[1]+q[2];c,s=np.cos(theta),np.sin(theta)
        result=np.eye(4);result[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];return result
    before=baseline();after=before.copy();after[1]+=.03;after[2]-=.02
    result=m.actual_motion_metrics(fk,before,after)
    assert result['actual_fk_rotation_residual_deg']==pytest.approx(math.degrees(.01))
    assert result['actual_joint23_sum_delta_deg']==pytest.approx(math.degrees(.01))
    assert result['actual_fk_rotation_within_one_encoder_word'] is False
    assert result['pure_translation_proven'] is False


@pytest.mark.parametrize('name',m.LABELS[1:])
def test_out_of_order_command_latches_failure(name):
    c=contract()
    with pytest.raises(RuntimeError,match='out of order'):
        c.begin(name,c.targets[0],c.reference,c.reference,c.epoch,1000)
    with pytest.raises(RuntimeError):send(c,0)


def test_epoch_change_precludes_return_even_after_recovery():
    c=contract();send(c,0)
    with pytest.raises(RuntimeError,match='epoch changed'):c.assert_feedback(c.reference,101)
    with pytest.raises(RuntimeError,match='epoch changed'):
        c.begin(m.LABELS[1],c.reference,c.targets[0],c.targets[0],100,1200)


@pytest.mark.parametrize('index',[0,3,4,5])
def test_every_unedited_axis_feedback_is_latched(index):
    c=contract();send(c,0);q=c.reference.copy();q[index]+=math.radians(.401)
    with pytest.raises(RuntimeError,match='unedited axis'):c.assert_feedback(q,100)
    with pytest.raises(RuntimeError):c.assert_feedback(c.reference,100)


@pytest.mark.parametrize('index,direction',[(1,-1),(2,1)])
def test_wrong_direction_over_half_degree_latches(index,direction):
    c=contract();send(c,0);q=c.reference.copy();q[index]+=direction*math.radians(.501)
    with pytest.raises(RuntimeError,match='directional'):c.assert_feedback(q,100)


def test_absolute_reservation_does_not_recenter_on_return():
    c=contract();send(c,0)
    q=finish_leg(c)
    send(c,1,q,q)
    assert c.active['lower'][1]>=c.origin[1]-m.ABSOLUTE_RAD
    assert c.active['upper'][2]<=c.origin[2]+m.ABSOLUTE_RAD
    escaped=c.origin.copy();escaped[2]+=math.radians(4.801)
    with pytest.raises(RuntimeError,match='initial absolute'):c.assert_feedback(escaped,100)


def test_sdk_bias_cannot_clamp_four_degree_target_to_fit():
    q=baseline();sdk=q.copy();sdk[2]+=math.radians(.9)
    with pytest.raises(RuntimeError,match='never clamp'):
        m.CounterrotationContract(q,sdk,100,4.)


@pytest.mark.parametrize('index',[0,3,4,5,6])
def test_command_cannot_edit_other_axis_or_gripper(index):
    c=contract();target=c.targets[0].copy();target[index]+=.0001
    with pytest.raises(RuntimeError,match='exact original'):
        c.begin(m.LABELS[0],target,c.reference,c.reference,100,1000)


def test_second_sdk_axis_waits_for_its_onehot_intent_and_first_stationarity():
    c=contract();send(c,0)
    first,effort=reserve(c,1)
    c.observe_sdk(first,1002,100)
    assert not c.target_seen and c.expected_words==m.words(first)
    c.confirm_wire_stationary(first,100,True)
    final,effort=reserve(c,2,first,first,stamp=1011)
    c.observe_sdk(final,1012,100)
    assert c.target_seen
    with pytest.raises(RuntimeError,match='changed after'):
        c.observe_sdk(first,1013,100)


@pytest.mark.parametrize('index,delta',[(0,m.QUANTUM),(1,23*m.QUANTUM),(2,m.QUANTUM),(6,-.001)])
def test_unexpected_sdk_is_latched(index,delta):
    c=contract();send(c,0);reserve(c,1);q=c.reference.copy();q[index]+=delta
    with pytest.raises(RuntimeError):c.observe_sdk(q,1001,100)
    assert c.failed


def test_no_return_without_independent_stationary_completion():
    c=contract();send(c,0);reserve(c,1)
    with pytest.raises(RuntimeError,match='stationary'):
        c.begin(m.LABELS[1],c.reference,c.targets[0],c.targets[0],100,1100)


def test_one_edited_axis_no_response_cannot_complete():
    c=contract();send(c,0);finish_leg(c)
    q=c.reference.copy();q[1]=c.targets[0][1]
    with pytest.raises(RuntimeError,match='both edited axes'):c.complete(q,100,True)


def test_existing_sdk_bias_is_separate_from_feedback_rectangle():
    q=baseline();target=contract().targets[0];sdk=q.copy();sdk[1]-=math.radians(.6)
    lower,upper,_=m.segment_rectangle(q,target,sdk,q)
    assert sdk[1]<lower[1]
    assert lower[1]==pytest.approx(q[1]-math.radians(.5))


@pytest.mark.parametrize('index,delta',[(1,-4.801),(0,.401)])
def test_nominal_sdk_bridge_still_requires_original_absolute_and_held_bounds(index,delta):
    q=baseline();sdk=q.copy();sdk[index]+=math.radians(delta)
    with pytest.raises(RuntimeError,match='held SDK bridge'):
        m.segment_rectangle(q,contract().targets[0],sdk,q)


def test_cells_cover_full_independent_box_without_held_double_counting():
    c=contract();lo,hi,_=m.segment_rectangle(c.reference,c.targets[0],c.reference,c.origin)
    axes=m.rectangle_grid(lo,hi);cells=list(m.rectangle_cells(axes))
    assert len(cells)==16*(len(axes[1])-1)*(len(axes[2])-1)
    assert len(cells)<=1296
    assert max(radius for _,radius in cells)<=math.radians(1.4)+1e-12
    # Every corner of every six-dimensional cell fits its own certified L1 ball.
    for indices,(center,radius) in zip(itertools.product(*(range(len(a)-1) for a in axes)),cells):
        for bits in itertools.product((0,1),repeat=6):
            corner=np.array([a[i+b] for a,i,b in zip(axes,indices,bits)])
            assert np.sum(abs(corner-center))<=radius+1e-12
    for index in m.HELD:
        assert len(axes[index])==3
        centers=sorted(set(round(float(q[index]-c.origin[index]),12) for q,_ in cells))
        assert centers==pytest.approx([-math.radians(.2),math.radians(.2)],abs=1e-12)


class FakeBase:
    class Diagnostic:
        def guard(self):pass
    NAMES=['Joint%d'%i for i in range(1,7)]+['right_finger']
    GetStateValidityRequest=lambda **kwargs:SimpleNamespace(**kwargs)
    RobotState=lambda:SimpleNamespace()
    JointState=lambda **kwargs:SimpleNamespace(**kwargs)
    positions=staticmethod(lambda value:np.array(value))
    write_json=staticmethod(lambda *args:None)


def test_off_diagonal_moveit_rejection_prevents_authorization(monkeypatch):
    fakeproof=lambda *args:dict(minimum_plane_clearance_lower_bound_m=.1,
        minimum_full_board_edge_lower_bound_px=50.,minimum_board_depth_lower_bound_m=.2,
        camera_coordinate_taylor_bound=dict(corner_lower_xyz_m=[[.2,-.02,-.02]]*4,corner_upper_xyz_m=[[.3,.02,.02]]*4))
    monkeypatch.setitem(sys.modules,'bootstrap_mixed_geometry',SimpleNamespace(certify_local_cell_geometry=fakeproof,projection_corner_box_batch=lambda *args:np.array([[50,500],[50,400]])))
    cls=m.make_diagnostic_class(FakeBase);d=cls.__new__(cls)
    d.contract=contract();d.guard=lambda:None;d.lo=np.full(6,-math.pi);d.hi=np.full(6,math.pi)
    class Fk:
        def __call__(self,q):return np.eye(4)
        def corner_motion_radii(self,p):return [.1]*6
    d.fk=Fk();d.gripper=object();visited=[]
    monkeypatch.setattr(FakeBase,'_stage_boxes',staticmethod(lambda *args:[('box',np.zeros(3),np.ones(3))]),raising=False)
    monkeypatch.setattr(FakeBase,'_box_corners',staticmethod(lambda *args:np.array([[0.,0.,0.]])),raising=False)
    def validity(request):
        q=np.array(request.robot_state.joint_state.position);visited.append(q)
        # Both joints positive cannot occur on the positive command diagonal,
        # but can occur inside the independent rebound rectangle.
        delta=q-d.contract.origin
        return SimpleNamespace(valid=not(delta[1]>.02 and delta[2]>.001))
    d.validity=validity
    with pytest.raises(RuntimeError,match='MoveIt'):
        d.path(d.contract.origin,d.contract.targets[0],{'Y':np.eye(4)},d.contract.reference)
    assert any((q-d.contract.origin)[1]>.02 and (q-d.contract.origin)[2]>.001 for q in visited)


def test_preparation_cannot_enter_execute_or_bootstrap():
    cls=m.make_diagnostic_class(FakeBase);d=cls.__new__(cls)
    d.args=SimpleNamespace(execute=False);d.publisher=None
    with pytest.raises(RuntimeError,match='cannot publish'):d.execute_step('positive',baseline())
    with pytest.raises(RuntimeError,match='no bootstrap'):d.bootstrap_once()


def test_run_failure_never_sends_automatic_return(tmp_path):
    cls=m.make_diagnostic_class(FakeBase);d=cls.__new__(cls);q=baseline()
    d.args=SimpleNamespace(execute=True,step_deg=2.);d.await_initial=lambda:(q,{'stationary':True})
    d.snapshot=lambda label:{};d.lock=threading.RLock();d.guard=lambda:None;d.accepted=[q]
    d.frozen_epoch=100;d.out=tmp_path;seen=[]
    def execute(label,target):
        seen.append(label);raise RuntimeError('fault after positive publication')
    d.execute_step=execute
    with pytest.raises(RuntimeError,match='fault after positive'):d.run()
    assert seen==['positive']


@pytest.mark.parametrize('bad',['--bootstrap-pending','--plan','--enable','--sync'])
def test_cli_does_not_expose_recovery_or_old_plan(bad):
    with pytest.raises(SystemExit):m.parse_args([bad])


def test_cli_defaults_to_readonly_two_degree():
    args=m.parse_args([])
    assert args.execute is False and args.step_deg==2. and args.bootstrap_pending is False


def test_actual_gripper_cannot_escape_proved_frozen_interval():
    c=contract();send(c,0);q=c.reference.copy();q[6]-=.00001
    with pytest.raises(RuntimeError,match='gripper escaped'):c.assert_feedback(q,100)
    assert c.failed


@pytest.mark.parametrize('channel',['accepted','sdk','board'])
def test_fresh_sources_must_follow_frozen_epoch(channel):
    stamps=dict(accepted=500,sdk=500,board=500);stamps[channel]=99
    with pytest.raises(RuntimeError,match='before frozen epoch'):
        m.validate_source_binding(1000,100,stamps['accepted'],stamps['sdk'],stamps['board'])


def test_publish_time_recheck_rejects_crossing_350ms():
    epoch=100;source=1_000_000_000
    m.validate_source_binding(source+350_000_000,epoch,source+100_000_000,source+100_000_000,source)
    with pytest.raises(RuntimeError,match='board source'):
        m.validate_source_binding(source+350_000_001,epoch,source+100_000_000,source+100_000_000,source)


def scene_fixture(clearance=.1):
    proof=dict(minimum_plane_clearance_lower_bound_m=clearance,
        maximum_cad_distance_to_board_origin_m=1.,
        camera_link_cell_corner_lower_m=[[[.2,-.02,-.02]]*4],camera_link_cell_corner_upper_m=[[[.3,.02,.02]]*4])
    model=SimpleNamespace(evidence={'color_intrinsics':{'width':640,'height':480}})
    return proof,np.eye(4),model


def test_fresh_plane_change_cannot_spend_old_clearance_margin():
    proof,old,model=scene_fixture(.0401);fresh=old.copy();fresh[2,3]+=.001
    with pytest.raises(RuntimeError,match='40 mm'):
        m.verify_fresh_scene(proof,old,fresh,model,lambda *args:np.array([[50,500],[50,400]]))


def test_fresh_plane_tilt_uses_full_cell_cad_radius():
    proof,old,model=scene_fixture(.0401);fresh=old.copy();a=math.radians(.1)
    fresh[:3,:3]=[[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]]
    with pytest.raises(RuntimeError,match='40 mm'):
        m.verify_fresh_scene(proof,old,fresh,model,lambda *args:np.array([[50,500],[50,400]]))


def test_fresh_board_change_reproves_projection_instead_of_accepting_three_mm():
    proof,old,model=scene_fixture();fresh=old.copy();fresh[1,3]+=.001
    def projection(model,lower,upper):
        edge=20.1-1000*(.2-np.min(lower[:,:,0]));return np.array([[edge,600],[30,430]])
    assert m.verify_fresh_scene(proof,old,old,model,projection)['minimum_full_board_edge_lower_bound_px']>20
    with pytest.raises(RuntimeError,match='20 px'):
        m.verify_fresh_scene(proof,old,fresh,model,projection)


def test_fresh_scene_reproof_allows_only_remaining_proved_margin():
    proof,old,model=scene_fixture();fresh=old.copy();fresh[2,3]+=.001
    result=m.verify_fresh_scene(proof,old,fresh,model,lambda *args:np.array([[50,500],[50,400]]))
    assert result['minimum_plane_clearance_lower_bound_m']==pytest.approx(.099)
    assert result['observed_board_point_displacement_bound_m']==pytest.approx(.001)


def test_realistic_j3_sdk_bias_does_not_expand_physical_reverse_response():
    q=baseline();sdk=q.copy();sdk[2]+=math.radians(.615234375)
    c=m.CounterrotationContract(q,sdk,100,2.)
    c.begin('positive',c.targets[0],q,sdk,100,1000)
    assert c.active['upper'][2]-q[2]==pytest.approx(math.radians(.5))
    c.observe_sdk(sdk,1001,100)
    changed=q.copy();changed[2]+=math.radians(.501)
    with pytest.raises(RuntimeError,match='directional'):c.assert_feedback(changed,100)


def test_nominal_sdk_box_contains_independent_axis_states():
    q=baseline();sdk=q.copy();sdk[2]+=math.radians(.615234375)
    c=m.CounterrotationContract(q,sdk,100,2.)
    endpoints=np.vstack((q[:6],sdk[:6],c.targets[0][:6]))
    axes=m.rectangle_grid(endpoints.min(axis=0),endpoints.max(axis=0))
    centers=[center for center,radius in m.rectangle_cells(axes)]
    assert any(center[1]>q[1]+.02 and center[2]>q[2] for center in centers)
    assert all(np.all(center>=endpoints.min(axis=0)) and np.all(center<=endpoints.max(axis=0)) for center in centers)


def test_fresh_scene_keeps_per_corner_depth_and_displacement_binding():
    proof,old,model=scene_fixture();fresh=old.copy();a=math.radians(.1)
    fresh[:3,:3]=[[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]]
    proof['camera_link_cell_corner_lower_m']=[[[.2,-.03,-.04],[.23,-.02,.01],[.28,.04,-.02],[.31,.05,.02]]]
    proof['camera_link_cell_corner_upper_m']=(np.asarray(proof['camera_link_cell_corner_lower_m'])+.01).tolist()
    seen=[]
    def projection(model,lower,upper):
        seen.append((lower.copy(),upper.copy()))
        return np.array([[50,500],[50,400]])
    m.verify_fresh_scene(proof,old,fresh,model,projection)
    lower,upper=seen[0]
    assert lower.shape==(1,4,3) and upper.shape==(1,4,3)
    np.testing.assert_array_equal(lower[0,0],proof['camera_link_cell_corner_lower_m'][0][0])
    assert np.all(lower[0,2]<np.asarray(proof['camera_link_cell_corner_lower_m'][0][2]))
    assert lower[0,2,0]>lower[0,0,0]


def test_positive_only_is_explicit_default_stays_four_commands():
    args=m.parse_args([])
    assert args.positive_only is False and len(contract().targets)==4
    args=m.parse_args(['--positive-only'])
    assert args.positive_only is True and args.execute is False


def test_positive_only_allows_exactly_two_commands_and_no_negative_fallback():
    q=baseline();c=m.CounterrotationContract(q,q,100,2.,positive_only=True)
    assert c.labels==m.LABELS[:2] and len(c.targets)==2
    current=q.copy()
    for index,target in enumerate(c.targets):
        c.begin(c.labels[index],target,current,current,100,1000+index*100)
        current=finish_leg(c)
    np.testing.assert_array_equal(current,q)
    with pytest.raises(RuntimeError,match='2-leg sequence exhausted'):
        c.begin('negative',q,q,q,100,1300)
    assert c.count==2 and c.wire_count==4 and c.maximum_commands==4


def test_positive_only_failure_never_turns_into_return_or_negative():
    q=baseline();c=m.CounterrotationContract(q,q,100,2.,positive_only=True)
    c.begin('positive',c.targets[0],q,q,100,1000)
    wrong=q.copy();wrong[1]-=math.radians(.501)
    with pytest.raises(RuntimeError,match='directional'):c.assert_feedback(wrong,100)
    with pytest.raises(RuntimeError,match='directional'):
        c.begin('return_after_positive',q,c.targets[0],c.targets[0],100,1100)
    assert c.count==1


def test_positive_only_keeps_same_full_physical_reservation():
    q=baseline();full=m.CounterrotationContract(q,q,100,2.)
    short=m.CounterrotationContract(q,q,100,2.,positive_only=True)
    a=full.begin('positive',full.targets[0],q,q,100,1000)
    b=short.begin('positive',short.targets[0],q,q,100,1000)
    for name in ('lower','upper','directions'):
        np.testing.assert_array_equal(a[name],b[name])


def test_positive_only_does_not_read_or_prepare_negative_branch(tmp_path):
    cls=m.make_diagnostic_class(FakeBase);d=cls.__new__(cls);q=baseline()
    d.args=SimpleNamespace(execute=False,step_deg=2.,positive_only=True)
    d.await_initial=lambda:(q,{'stationary':True});d.snapshot=lambda label:{}
    d.lock=threading.RLock();d.guard=lambda:None;d.accepted=[q];d.reference=[q]
    d.frozen_epoch=100;d.epoch=100;d.started_ns=101;d.out=tmp_path
    d.stationary=lambda *args:{};d.event=lambda *args,**kwargs:None;calls=[]
    d.path=lambda measured,target,geometry,reference:(calls.append(target.copy()) or {})
    d.run()
    assert len(calls)==1
    np.testing.assert_array_equal(calls[0],d.contract.targets[0])


def test_planning_gap_intersection_cannot_expand_publish_start_reverse_limit():
    q=baseline();c=m.CounterrotationContract(q,q,100,2.,positive_only=True)
    old_lower,old_upper,_=m.segment_rectangle(q,c.targets[0],q,q)
    current=q.copy();current[1]+=math.radians(.1)
    c.begin('positive',c.targets[0],current,q,100,1000)
    c.intersect_proved_rectangle(old_lower,old_upper)
    assert c.active['lower'][1]==pytest.approx(current[1]-math.radians(.5))
    wrong=q.copy();wrong[1]-=math.radians(.45)
    with pytest.raises(RuntimeError,match='directional'):c.assert_feedback(wrong,100)


def test_onehot_payload_matches_driver_hold_semantics_for_both_legs():
    q=baseline();c=m.CounterrotationContract(q,q,100,2.,positive_only=True)
    manual=q.copy();current=q.copy();transmitted=[]
    for index,target in enumerate(c.targets):
        c.begin(c.labels[index],target,current,manual,100,1000+100*index)
        for part,axis in enumerate(m.EDITED):
            stamp=1001+100*index+10*part
            payload,effort=reserve(c,axis,current,manual,stamp)
            # driver.cpp: exactly one effort >.5; only that named axis is
            # changed, all other values are taken from its retained manual goal.
            marked=[j for j,value in enumerate(effort) if value>.5]
            assert marked==[axis] and len(effort)==7
            updated=manual.copy();updated[marked[0]]=payload[marked[0]]
            np.testing.assert_array_equal(updated,payload)
            assert all(payload[j]==manual[j] for j in range(7) if j!=axis)
            c.observe_sdk(updated,stamp+1,100)
            current[axis]=payload[axis]
            c.confirm_wire_stationary(current,100,True)
            manual=updated;transmitted.append(marked[0])
        c.complete(current,100,True)
    assert transmitted==[1,2,1,2] and c.wire_count==c.maximum_commands==4
    np.testing.assert_array_equal(manual,q)


@pytest.mark.parametrize('second_first',[False,True])
def test_next_axis_needs_current_sdk_confirmation_and_stationarity(second_first):
    c=contract();send(c,0)
    first,_=reserve(c,1)
    if second_first:
        c.observe_sdk(first,1002,100)
    with pytest.raises(RuntimeError,match='confirmation and independent stationary'):
        reserve(c,2,first,first,1011)
    assert c.failed and c.wire_count==1
    with pytest.raises(RuntimeError):reserve(c,2,first,first,1020)


def test_j3_cannot_update_before_its_onehot_message_even_inside_old_box():
    c=contract();send(c,0);reserve(c,1)
    early=c.reference.copy();early[2]-=m.QUANTUM
    with pytest.raises(RuntimeError,match='active one-hot'):
        c.observe_sdk(early,1002,100)


@pytest.mark.parametrize('axis',[0,2,3,4,5,6])
def test_j2_sdk_ack_does_not_permit_any_other_axis_word_change(axis):
    c=contract();send(c,0);first,_=reserve(c,1)
    changed=first.copy();changed[axis]+=(-m.QUANTUM if axis in (2,6) else m.QUANTUM)
    with pytest.raises(RuntimeError):c.observe_sdk(changed,1002,100)


@pytest.mark.parametrize('stamp',[99,1000,1001])
def test_old_epoch_or_non_new_sdk_cannot_ack_this_publication(stamp):
    c=contract();send(c,0);first,_=reserve(c,1,stamp=1001)
    with pytest.raises(RuntimeError):c.observe_sdk(first,stamp,100)
    assert c.failed and not c.active['wire_acknowledged']


def test_second_payload_preserves_the_newly_confirmed_j2_word():
    c=contract();send(c,0);first,_=reserve(c,1)
    c.observe_sdk(first,1002,100);c.confirm_wire_stationary(first,100,True)
    final,effort=reserve(c,2,first,first,1011)
    assert final[1]==first[1] and final[1]!=c.reference[1] and effort==[0,0,1,0,0,0,0]
    stale=c.reference.copy();stale[2]=c.targets[0][2]
    with pytest.raises(RuntimeError,match='one-hot wire segment'):
        c.observe_sdk(stale,1012,100)


@pytest.mark.parametrize('bad', ['accepted','sdk','board'])
def test_second_axis_rejects_a_fresh_but_preceding_publication_source(bad):
    c=contract();send(c,0);first,_=reserve(c,1)
    c.observe_sdk(first,1002,100);c.confirm_wire_stationary(first,100,True)
    stamps={'accepted':1010,'sdk':1010,'board':1010};stamps[bad]=1000
    with pytest.raises(RuntimeError,match='precedes first-axis'):
        c.reserve_wire(2,first,first,100,1011,stamps['accepted'],stamps['sdk'],stamps['board'])
    assert c.failed and c.wire_count==1


@pytest.mark.parametrize('changed', ['epoch','stale_board','old_sdk'])
def test_second_axis_fault_latches_before_any_second_reservation(changed):
    c=contract();send(c,0);first,_=reserve(c,1)
    c.observe_sdk(first,1002,100);c.confirm_wire_stationary(first,100,True)
    epoch=101 if changed=='epoch' else 100
    sdk=c.reference if changed=='old_sdk' else first
    now=400_001_003 if changed=='stale_board' else 1011
    with pytest.raises(RuntimeError):c.reserve_wire(2,sdk,first,epoch,now,now-1,now-1,1002)
    assert c.failed and c.wire_count==1
    with pytest.raises(RuntimeError):c.begin('return_after_positive',c.reference,first,first,100,now+1)


def test_second_axis_intersects_current_start_reverse_bound_without_expansion():
    c=contract();send(c,0);first,_=reserve(c,1)
    c.observe_sdk(first,1002,100)
    actual=first.copy();actual[2]-=math.radians(.1)
    c.confirm_wire_stationary(actual,100,True)
    old_lower=c.active['lower'].copy();old_upper=c.active['upper'].copy()
    reserve(c,2,actual,first,1011)
    assert c.active['upper'][2]==pytest.approx(actual[2]+m.REVERSE_RAD)
    assert np.all(c.active['lower']>=old_lower) and np.all(c.active['upper']<=old_upper)
    wrong=actual.copy();wrong[2]+=math.radians(.501)
    with pytest.raises(RuntimeError,match='directional'):c.assert_feedback(wrong,100)


def test_first_sdk_endpoint_without_actual_response_cannot_authorize_second():
    c=contract();send(c,0);first,_=reserve(c,1)
    c.observe_sdk(first,1002,100)
    with pytest.raises(RuntimeError,match='actual response'):
        c.confirm_wire_stationary(c.reference,100,True)
    assert c.failed and c.wire_count==1


class Stamp:
    def __init__(self,ns):self.ns=int(ns)
    def to_nsec(self):return self.ns


class WireBase(FakeBase):
    positions=staticmethod(lambda msg:np.array(msg.position))
    JointState=staticmethod(lambda **kwargs:SimpleNamespace(header=SimpleNamespace(),**kwargs))


def wire_client(monkeypatch,tmp_path):
    proof,old,model=scene_fixture()
    proof['lower_rad']=(baseline()[:6]-m.ABSOLUTE_RAD).tolist()
    proof['upper_rad']=(baseline()[:6]+m.ABSOLUTE_RAD).tolist()
    proof['scene_enclosures_key']='test'
    monkeypatch.setitem(sys.modules,'bootstrap_mixed_geometry',SimpleNamespace(
        projection_corner_box_batch=lambda *args:np.array([[50,500],[50,400]])))
    clock=SimpleNamespace(ns=1_000_000_000)
    WireBase.rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda:Stamp(clock.ns)))
    cls=m.make_diagnostic_class(WireBase);d=cls.__new__(cls)
    q=baseline();d.args=SimpleNamespace(execute=True,step_deg=2.,positive_only=True)
    d.contract=m.CounterrotationContract(q,q,100,2.,positive_only=True)
    d.lock=threading.RLock();d.epoch=d.frozen_epoch=100;d.started_ns=101
    d.deadline=float('inf');d.publish_count=0;d.saved_rgb=set();d.out=tmp_path
    def message(q):return SimpleNamespace(position=q.copy(),header=SimpleNamespace(stamp=Stamp(clock.ns-1)))
    d.reference=[message(q)];d.accepted=[message(q)];d.expected_words=m.words(q)
    d.scene_enclosures={'test':(proof['camera_link_cell_corner_lower_m'],proof['camera_link_cell_corner_upper_m'])}
    d.guard=lambda:d.contract.assert_feedback(WireBase.positions(d.accepted[-1]),d.epoch)
    d.stationary=lambda after:{'after':after,'stationary_window_start_ns':clock.ns-800_000_000}
    d.await_initial=lambda:(q,{'stationary':True})
    d.snapshot=lambda label:dict(X=np.eye(4),Y=np.eye(4),model=model,source_stamp_ns=clock.ns-1,q=WireBase.positions(d.accepted[-1]))
    d.path=lambda *args,**kwargs:proof.copy();d.driver_connection=lambda:None
    d.fk=lambda q:np.eye(4);d.stable_hold_recording=lambda seconds:None
    events=[];d.event=lambda name,**values:events.append((name,values))
    publications=[]
    def publish(msg):
        assert d.publish_count==len(publications)+1
        assert d.contract.wire_count==d.publish_count
        publications.append(msg)
    d.publisher=SimpleNamespace(get_num_connections=lambda:1,publish=publish)
    def confirm(label,axis,sent,deadline):
        clock.ns+=30_000_000
        payload=np.asarray(publications[-1].position)
        sdk=message(payload);sdk.header.stamp=Stamp(clock.ns)
        d.reference.append(sdk);d.accepted.append(message(payload))
        d.contract.observe_sdk(payload,clock.ns,d.epoch)
        d.contract.confirm_wire_stationary(payload,d.epoch,True)
        return payload,{'stationary':True}
    d.await_onehot_stationary=confirm
    return d,clock,publications,events


def test_actual_publisher_path_uses_four_onehot_messages_no_driver_rejection(monkeypatch,tmp_path):
    d,clock,publications,events=wire_client(monkeypatch,tmp_path)
    d.run()
    assert len(publications)==4 and d.publish_count==4
    assert [[i for i,e in enumerate(msg.effort) if e>.5] for msg in publications]==[[1],[2],[1],[2]]
    assert all(msg.header.frame_id=='gui_direct' for msg in publications)
    assert [[name for name,effort in zip(msg.name,msg.effort) if effort>.5] for msg in publications]==[['Joint2'],['Joint3'],['Joint2'],['Joint3']]
    assert all(len(msg.name)==len(msg.effort)==7 for msg in publications)
    original=baseline();first=np.asarray(publications[0].position);second=np.asarray(publications[1].position)
    assert first[2]==original[2] and second[1]==first[1]
    np.testing.assert_array_equal(publications[-1].position,original)
    commands=[values for name,values in events if name=='command']
    assert [row['one_hot_part'] for row in commands]==[1,2,1,2]
    assert all(row['precommand_source_age_ms']<=350 for row in commands)


def test_first_onehot_timeout_never_publishes_second_or_return(monkeypatch,tmp_path):
    d,clock,publications,events=wire_client(monkeypatch,tmp_path)
    def missing(*args):raise RuntimeError('SDK never confirms first one-hot')
    d.await_onehot_stationary=missing
    with pytest.raises(RuntimeError,match='SDK never confirms'):d.run()
    assert len(publications)==1 and d.publish_count==1


def test_second_fresh_source_failure_never_reuses_first_source_or_returns(monkeypatch,tmp_path):
    d,clock,publications,events=wire_client(monkeypatch,tmp_path)
    snapshot=d.snapshot
    def missing(label):
        if '_j3_precommand_' in label:raise RuntimeError('fresh paired board unavailable')
        return snapshot(label)
    d.snapshot=missing
    with pytest.raises(RuntimeError,match='fresh paired board'):d.run()
    assert len(publications)==1 and d.contract.wire_count==1


def test_second_sdk_changed_after_confirmation_never_reaches_publisher(monkeypatch,tmp_path):
    d,clock,publications,events=wire_client(monkeypatch,tmp_path)
    snapshot=d.snapshot
    def changed(label):
        if '_j3_precommand_' in label:
            d.reference[-1].position[1]-=m.QUANTUM
        return snapshot(label)
    d.snapshot=changed
    with pytest.raises(RuntimeError,match='predecessor'):d.run()
    assert len(publications)==1 and d.contract.failed
