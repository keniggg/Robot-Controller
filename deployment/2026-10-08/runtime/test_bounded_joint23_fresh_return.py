"""Offline one-shot return/lineage/adaptive-box contracts; no ROS control."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np
import pytest
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import bounded_joint23_fresh_return as m
import test_bounded_joint23_counterrotation as original
cr=m.cr


def parent_identity():
    origin=original.baseline();goal=origin.copy();goal[1]+=3*cr.QUANTUM;goal[2]+=7*cr.QUANTUM
    current=goal.copy();current[1]+=22*cr.QUANTUM;current[2]-=22*cr.QUANTUM
    actual=origin.copy();actual[1]+=22*cr.QUANTUM;actual[2]-=22*cr.QUANTUM
    return dict(origin=origin,original_sdk=goal,expected_current_sdk=current,expected_actual=actual,epoch=100)


def test_new_return_keeps_parent_origin_and_exact_target_with_two_publications():
    p=parent_identity();current=p['expected_actual'].copy();sdk=p['expected_current_sdk'].copy()
    c=m.ReturnContract(p,current,sdk,100)
    np.testing.assert_array_equal(c.origin,p['origin']);np.testing.assert_array_equal(c.targets[0],p['original_sdk'])
    assert len(c.targets)==1 and c.maximum_commands==2
    c.begin(c.labels[0],c.targets[0],current,sdk,100,1000)
    # These are fresh actual->goal directions, opposite to outbound.
    assert c.active['directions'][1:3].tolist()==[-1.,1.]
    for part,axis in enumerate(cr.EDITED):
        stamp=1001+10*part
        payload,effort=c.reserve_wire(axis,sdk,current,100,stamp,stamp-1,stamp-1,stamp-1)
        assert [j for j,e in enumerate(effort) if e>.5]==[axis]
        sdk=payload.copy();current[axis]=p['origin'][axis]
        c.observe_sdk(sdk,stamp+1,100);c.confirm_wire_stationary(current,100,True)
    c.complete(current,100,True)
    assert c.wire_count==2
    np.testing.assert_array_equal(sdk,p['original_sdk'])
    with pytest.raises(RuntimeError,match='exhausted'):c.begin(c.labels[0],c.targets[0],current,sdk,100,1100)


@pytest.mark.parametrize('kind',['epoch','sdk','actual'])
def test_parent_endpoint_change_prevents_new_return_contract(kind):
    p=parent_identity();q=p['expected_actual'].copy();sdk=p['expected_current_sdk'].copy();epoch=100
    if kind=='epoch':epoch=101
    elif kind=='sdk':sdk[1]+=cr.QUANTUM
    else:q[1]+=math.radians(.151)
    with pytest.raises(RuntimeError):m.ReturnContract(p,q,sdk,epoch)


def test_new_return_never_recenters_initial_absolute_or_held_limits():
    p=parent_identity();c=m.ReturnContract(p,p['expected_actual'],p['expected_current_sdk'],100)
    c.begin(c.labels[0],c.targets[0],p['expected_actual'],p['expected_current_sdk'],100,1000)
    assert np.all(c.active['lower']>=p['origin'][:6]-cr.ABSOLUTE_RAD)
    assert np.all(c.active['upper']<=p['origin'][:6]+cr.ABSOLUTE_RAD)
    wrong=p['expected_actual'].copy();wrong[0]=p['origin'][0]+math.radians(.401)
    with pytest.raises(RuntimeError,match='unedited'):c.assert_feedback(wrong,100)


def budget(maximum=20):return dict(checks=0,maximum_checks=maximum,rejected_cells=0,diagnostics=[])


def inspect_good(*args):return dict(exact_center_edge_px=40.,exact_center_plane_clearance_m=.08,exact_center_depth_m=.2)


def test_failed_cell_bisects_without_shrinking_original_box_or_changing_threshold():
    lo=np.zeros(6);hi=np.zeros(6);lo[1:3]=-.01;hi[1:3]=.01;calls=[];b=budget()
    def prove(center,radius,half):
        calls.append((center.copy(),radius))
        if radius>.0150000001:raise RuntimeError('continuous mixed initial sync violates 40 mm plane / 20 px board margin')
    cells=list(m.partition_cell(lo,hi,prove,inspect_good,lambda:None,b))
    assert b['checks']==3 and len(cells)==2
    np.testing.assert_array_equal(cells[0][0]+cells[1][0],np.zeros(6))
    assert cells[0][0][1]==pytest.approx(-.005) and cells[1][0][1]==pytest.approx(.005)
    assert all(radius==pytest.approx(.015) for _,radius in cells)
    # All 4 original edited-axis box corners retain coverage by a proved leaf.
    for first in (-.01,.01):
        for second in (-.01,.01):
            point=np.zeros(6);point[1:3]=[first,second]
            assert any(np.sum(abs(point-center))<=radius+1e-12 for center,radius in cells)


@pytest.mark.parametrize('field,value',[('exact_center_edge_px',19.999),('exact_center_plane_clearance_m',.039999),('exact_center_depth_m',.119999)])
def test_exact_outside_center_stops_subdivision_immediately(field,value):
    b=budget();report=inspect_good();report[field]=value
    def fail(*args):raise RuntimeError('continuous mixed initial sync violates threshold')
    with pytest.raises(RuntimeError,match='exact return cell center'):
        list(m.partition_cell(np.zeros(6),np.full(6,.001),fail,lambda *args:report,lambda:None,b))
    assert b['checks']==1


def test_non_geometry_fault_is_not_retried_or_subdivided():
    b=budget()
    def fail(*args):raise RuntimeError('epoch changed')
    with pytest.raises(RuntimeError,match='epoch changed'):
        list(m.partition_cell(np.zeros(6),np.full(6,.001),fail,inspect_good,lambda:None,b))
    assert b['checks']==1


def test_subdivision_has_fixed_depth_and_shared_total_budget():
    def fail(*args):raise RuntimeError('continuous mixed initial sync violates threshold')
    b=budget()
    with pytest.raises(RuntimeError,match='four-level'):
        list(m.partition_cell(np.zeros(6),np.full(6,.01),fail,inspect_good,lambda:None,b))
    assert b['checks']==5
    b=budget(2)
    with pytest.raises(RuntimeError,match='budget exhausted'):
        list(m.partition_cell(np.zeros(6),np.full(6,.01),fail,inspect_good,lambda:None,b))
    assert b['checks']==3


def parent_archive(monkeypatch,tmp_path):
    monkeypatch.setattr(cr,'ARCHIVE',tmp_path)
    path=tmp_path/'bounded_joint23_counterrotation_fixture';path.mkdir();p=parent_identity()
    cmd=[];confirm=[];current=p['expected_current_sdk']
    for part,axis in enumerate((2,3),1):
        wire=p['original_sdk'].copy() if part==1 else current.copy()
        if part==1:wire[1]=current[1]
        cmd.append(dict(event='command',label='positive',edited_joint=axis,one_hot_part=part,target_rad=wire.tolist()))
        confirm.append(dict(event='one_hot_sdk_confirmed_and_stationary',label='positive',edited_joint=axis,
            actual_sdk_words=cr.words(wire),measured_rad=p['expected_actual'].tolist()))
    events=cmd+confirm+[dict(event='post_hold_observation',label='positive')]
    values={'plan.json':dict(schema=2,maximum_commands=4,initial_measured_rad=p['origin'].tolist(),starting_sdk_rad=p['original_sdk'].tolist(),step_words=22,frozen_epoch_ns=100),
        'terminal.json':dict(status='failed',commands_published=2),'baseline_geometry.json':{},'positive_post_hold_geometry.json':{},'source_sha256.json':{}}
    for name,value in values.items():(path/name).write_text(json.dumps(value))
    (path/'robot_description.urdf').write_text('<robot name="fixture"/>')
    (path/'events.jsonl').write_text(''.join(json.dumps(event)+'\n' for event in events))
    return path


def test_parent_digest_binds_closed_identity_records(monkeypatch,tmp_path):
    path=parent_archive(monkeypatch,tmp_path);first=m.load_parent(path)
    assert m.load_parent(path,first['digest'])['digest']==first['digest']
    (path/'terminal.json').write_text(json.dumps(dict(status='failed',commands_published=2,note='changed')))
    with pytest.raises(RuntimeError,match='digest changed'):m.load_parent(path,first['digest'])


@pytest.mark.parametrize('count',[0,1,3,4])
def test_parent_with_different_number_of_real_publications_is_rejected(monkeypatch,tmp_path,count):
    path=parent_archive(monkeypatch,tmp_path)
    (path/'terminal.json').write_text(json.dumps(dict(status='failed',commands_published=count)))
    with pytest.raises(RuntimeError,match='zero return commands'):m.load_parent(path)


def test_execute_requires_reviewed_parent_digest_but_readonly_does_not():
    assert m.parse_args(['--parent','/tmp/parent']).execute is False
    with pytest.raises(SystemExit):m.parse_args(['--parent','/tmp/parent','--execute'])


def test_default_cell_partition_hook_keeps_original_contract():
    cls=cr.make_diagnostic_class(original.FakeBase);d=cls.__new__(cls)
    lo=np.zeros(6);hi=np.full(6,.003);axes=cr.rectangle_grid(lo,hi)
    before=list(cr.rectangle_cells(axes));after=list(d.path_cell_partition(axes,{},[.05]))
    assert len(before)==len(after)
    for (a,ra),(b,rb) in zip(before,after):np.testing.assert_array_equal(a,b);assert ra==rb


def test_three_or_four_degree_parent_is_outside_this_explicit_return_scope(monkeypatch,tmp_path):
    path=parent_archive(monkeypatch,tmp_path)
    value=json.loads((path/'plan.json').read_text());value['step_words']=34
    (path/'plan.json').write_text(json.dumps(value))
    with pytest.raises(RuntimeError,match='SDK pair'):m.load_parent(path)


def test_complete_return_runtime_reuses_two_onehot_publishers(monkeypatch,tmp_path):
    d,clock,publications,events=original.wire_client(monkeypatch,tmp_path)
    d.__class__=m.make_diagnostic_class(original.WireBase)
    p=parent_identity();geometry=d.snapshot('test');geometry['model'].fingerprint='same'
    p.update(directory='/closed_parent',digest='parentdigest',hashes={'robot_description.urdf':'urdf'},
        baseline={'quality':{'camera_model_fingerprint':'same'},'T_tool_camera':np.eye(4).tolist()},
        post_hold={'T_base_board':np.eye(4).tolist()})
    d.fk.model_sha256='urdf';d.args.parent_identity=p
    d.reference[-1].position=p['expected_current_sdk'].copy();d.accepted[-1].position=p['expected_actual'].copy()
    d.contract=m.ReturnContract(p,p['expected_actual'],p['expected_current_sdk'],100)
    d.await_initial=lambda:(p['expected_current_sdk'],{'stationary':True})
    d.run()
    assert len(publications)==2 and d.contract.maximum_commands==2
    assert [[name for name,e in zip(msg.name,msg.effort) if e>.5] for msg in publications]==[['Joint2'],['Joint3']]
    np.testing.assert_array_equal(publications[-1].position,p['original_sdk'])
    assert events[-1][0]=='completed_original_sdk_return'


def test_current_urdf_change_prevents_any_return_publication(monkeypatch,tmp_path):
    d,clock,publications,events=original.wire_client(monkeypatch,tmp_path)
    d.__class__=m.make_diagnostic_class(original.WireBase)
    p=parent_identity();p['hashes']={'robot_description.urdf':'old'}
    d.fk.model_sha256='new';d.args.parent_identity=p
    with pytest.raises(RuntimeError,match='URDF changed'):d.run()
    assert publications==[]


def test_partition_transmits_exact_per_axis_half_widths_to_certificate():
    lo=np.array([-.002,-.007,0.,-.001,0.,0.]);hi=-lo;seen=[]
    def prove(center,radius,half):seen.append((center.copy(),radius,half.copy()))
    cells=list(m.partition_cell(lo,hi,prove,inspect_good,lambda:None,budget()))
    assert len(cells)==1
    np.testing.assert_array_equal(seen[0][2],.5*(hi-lo))
    assert seen[0][1]==pytest.approx(sum(seen[0][2]))


def test_equal_center_radius_never_reuses_a_different_axis_box(monkeypatch):
    from types import SimpleNamespace
    cls=m.make_diagnostic_class(original.FakeBase);d=cls.__new__(cls)
    d.return_cell_cache={};d.return_partition=budget();d.guard=lambda:None
    d.fk=d.gripper=object();d.lo=np.full(6,-1.);d.hi=np.ones(6)
    calls=[]
    def prove(fk,gripper,center,geometry,lo,hi,half):
        calls.append(half.copy());return {'joint_half_width_rad':half.tolist()}
    monkeypatch.setitem(sys.modules,'bootstrap_mixed_geometry',SimpleNamespace(
        certify_joint_box_geometry=prove,board_camera_box_taylor_bound=object()))
    geometry={'Y':np.eye(4),'X':np.eye(4),'model':SimpleNamespace(evidence={'color_intrinsics':{'width':640,'height':480}})}
    for axis in (0,5):
        half=np.zeros(6);half[axis]=.003
        axes=[np.array([-h,h]) for h in half]
        cells=list(d.path_cell_partition(axes,geometry,[.05]));assert len(cells)==1
        center,radius=cells[0];proof=d.path_cell_geometry(center,radius,geometry,.05)
        np.testing.assert_array_equal(proof['joint_half_width_rad'],half)
    assert len(calls)==2 and not np.array_equal(calls[0],calls[1])
