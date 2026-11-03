"""Independent original-GEN53 collision counterexamples; no hardware/ROS I/O."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import hashlib
import json
import math

import numpy as np
import pytest
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.grasp import gripper_geometry as gg
from alicia_flexible_grasp.robot import observation_path_guard as guard

FIXTURE=Path(__file__).parent/'fixtures/gen54_gen53_collision_contract.json'


def original_case(index):
    data=json.loads(FIXTURE.read_text());row=data['cases'][index]
    assert hashlib.sha256(data['model']['xml'].encode()).hexdigest()==data['model']['sha256']
    fk=guard.SerialUrdfFk(data['model']['xml'],row['checked_curve']['joint_names'])
    g=gg.GripperGeometry(**{k:np.array(v) if isinstance(v,list) else v for k,v in row['evaluated_gripper'].items()})
    scene=guard.FrozenObservationScene(**{k:np.array(v) if k in ('normal','center','rotation','size') else v for k,v in row['scene'].items()})
    transforms=[]
    for vals in row['plan_data']['poses_xyz_xyzw']:
        T=quaternion_matrix(vals[3:]);T[:3,3]=vals[:3];transforms.append(T)
    evidence=json.loads(row['plan_data']['contact_geometry_evidence'])
    return row,fk,g,scene,transforms,evidence


def physical_inputs(g,s,transforms,evidence):
    T=transforms[2]
    return dict(gripper=g,candidate_center_base=s.center,candidate_tool0_base=T[:3,3],R_base_tool=T[:3,:3],
        required_width_m=evidence['required_open_width_m'],obb_center_base=s.center,R_base_obb=s.rotation,obb_size_xyz_m=s.size,
        support_normal_base=s.normal,support_offset_m=s.offset,pregrasp_T_base_tool=transforms[0],approach_T_base_tool=transforms[1],
        grasp_T_base_tool=transforms[2],lift_T_base_tool=transforms[3],opening_fit_clearance_each_side_m=.0005,
        contact_height_bounds_m=evidence['contact_height_bounds_m'],contact_height_axis_base_override=T[:3,2],minimum_contact_patch_overlap_m=.002)


@pytest.mark.parametrize('index',range(7))
def test_original_frozen_fk_and_same_branch_terminal_rejection_unchanged(index):
    row,fk,g,s,transforms,evidence=original_case(index)
    original_row=deepcopy(row)
    T=fk(row['checked_curve']['points'][-1]['positions'])
    np.testing.assert_allclose(T,row['first_failed_sample']['T_base_tool0'],atol=1e-14,rtol=0)
    assert np.linalg.norm(T[:3,3]-transforms[2][:3,3])<6e-10
    # Extract only the actual pure production screen; importing the service
    # owner is unnecessary and would require generated ROS service modules.
    source=Path(guard.__file__).with_name('contact_sequence_precheck.py')
    function=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='contact_curve_terminal_collision')
    env=dict(guard.__dict__);exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),env)
    curve=NS(joint_trajectory=NS(joint_names=row['checked_curve']['joint_names'],points=[NS(positions=p['positions'],time_from_start=NS(to_nsec=lambda p=p:p['time_from_start_ns']))for p in row['checked_curve']['points']]))
    # GEN54's exact old-box rejection remains a regression requirement. The
    # existing testcase identity is retained, with GEN55's source-bound CAD
    # result explicitly versioned below for the very same FK/scene/opening.
    cad_model=fk.finger_cad_model
    assert cad_model is not None
    fk.finger_cad_model=None
    with pytest.raises(guard.ObservationPathError) as caught:
        env['contact_curve_terminal_collision'](curve,fk,s,g,row['opening_m'])
    assert 'nominal continuous geometry: '+str(caught.value)==row['failure']['message']
    assert caught.value.contact_replay_sample['body']==row['first_failed_sample']['body']
    assert caught.value.contact_replay_sample['endpoint_screen_only'] is True
    assert caught.value.contact_replay_sample['first_collision_along_curve_claimed'] is False
    fk.finger_cad_model=cad_model

    # Independent precomputed complete active+reference hull vertices already
    # include every +/-0.6 mm tool-axis padding corner. Project these explicit
    # world vertices, not the production model's support implementation.
    hull_fixture=Path(__file__).parent/'fixtures/gen55_finger_cad_actual_receipts.json'
    assert hashlib.sha256(hull_fixture.read_bytes()).hexdigest()=='f4c4e8f3f6624d85427de54026fc7dfc925fe14a6f104dd1b255d6efed0d12da'
    hull_data=json.loads(hull_fixture.read_text())
    assert row['opening_m']==hull_data['opening_m']
    report=env['contact_curve_terminal_collision'](curve,fk,s,g,row['opening_m'])
    assert report['checked'] is True and report['collision_at_terminal'] is False
    assert report['scope']=='same_proposed_curve_endpoint_only'
    assert report['path_verified'] is False
    proof=next(p['proof'] for p in report['cad_target_projection_proofs']
               if p['body']==row['first_failed_sample']['body'])
    vertices=np.asarray(hull_data['hulls'][proof['body']])@T[:3,:3].T+T[:3,3]
    axes=list(T[:3,:3].T)+list(s.rotation.T)
    axes += [np.cross(u,v) for u in T[:3,:3].T for v in s.rotation.T]
    gaps=[]
    for axis in axes:
        norm=np.linalg.norm(axis)
        if norm<=1e-12:continue
        axis=axis/norm;projection=vertices@axis
        mid=float(s.center@axis);radius=float(np.abs(s.rotation.T@axis)@(s.size/2.))
        gaps.append(max(mid-radius-projection.max(),projection.min()-mid-radius))
    expected_gap_m=(.0008485069763596909,.0008485069944922453,.000694771564196878,
        .0011626555735114116,.0011626548869737693,.0012184573611177772,.0006947714040360975)[index]
    assert max(gaps)==pytest.approx(expected_gap_m,abs=1e-12)
    assert proof['separated'] is True and proof['gap_m']==pytest.approx(max(gaps),abs=1e-12)
    assert proof['padding_each_tool_axis_m']==.0006
    assert proof['displacement_each_tool_axis_m']==0.
    assert proof['provenance']['robot_description_sha256']==fk.model_sha256
    assert proof['execution_authorized'] is False
    assert row==original_row  # Original recorded failure is never rewritten.


@pytest.mark.parametrize('index',range(7))
def test_original_failed_open_descent_is_rejected_by_candidate_before_strict_service(index):
    row,fk,g,s,transforms,evidence=original_case(index)
    kwargs=physical_inputs(g,s,transforms,evidence)
    before=deepcopy(kwargs)
    result=gg._evaluate_physical_candidate(**kwargs)
    assert not result.ok and result.failure_code=='GRIPPER_SWEEP_COLLISION'
    assert 'finger intrudes into target OBB' in result.failure_reason
    for key,value in kwargs.items():
        if isinstance(value,np.ndarray):np.testing.assert_array_equal(value,before[key])
    assert row['opening_m']==pytest.approx(.0498)
    # Neither calling the contact-sized opening 38 mm nor allowing another
    # .2 mm of actuator opening may erase the original padded collision.
    for opening in (row['opening_m'],.05):
        assert not gg._check_boxes(transforms[2],g,opening,'y','z',s.normal,s.offset,s.center,s.rotation,s.size,False,.0005)[0]
        assert gg._check_boxes(transforms[2],g,opening,'y','z',s.normal,s.offset,s.center,s.rotation,s.size,True,.0005)[0]


def synthetic_case(size,obb_rotation):
    g=gg.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
    s=NS(center=np.array([0.,0.,.025]),rotation=obb_rotation,size=np.array(size),normal=np.array([0.,0.,1.]),offset=0.)
    T=np.eye(4);T[:3,:3]=np.diag([-1.,1.,-1.]);T[:3,3]=[0.,0.,.015]
    transforms=[T.copy()for _ in range(4)]
    for tt,dz in zip(transforms,[.06,.03,0.,.04]):tt[2,3]+=dz
    evidence=dict(required_open_width_m=.004,contact_height_bounds_m=[-.01,.01])
    return g,s,transforms,evidence


def test_clear_preclosure_and_carried_lift_remain_valid():
    g,s,Ts,ev=synthetic_case([.04,.03,.02],np.eye(3));ev['required_open_width_m']=.031
    result=gg._evaluate_physical_candidate(**physical_inputs(g,s,Ts,ev))
    assert result.ok and result.passed_gate_count==6
    assert result.support_clearance_m>=.003


def test_large_complete_projection_is_not_a_necessary_finite_finger_collision():
    a=math.pi/4;R=np.array([[math.cos(a),-math.sin(a),0.],[math.sin(a),math.cos(a),0.],[0.,0.,1.]])
    g,s,Ts,ev=synthetic_case([.1,.001,.02],R)
    # The long thin object's distant corners exceed total jaw projection,
    # while every finite finger body remains disjoint along the open path.
    projection=float(np.abs(s.rotation.T@Ts[2][:3,1])@s.size)
    assert projection>.07 and projection>g.max_inner_gap_m
    result=gg._evaluate_physical_candidate(**physical_inputs(g,s,Ts,ev))
    assert result.ok
    for T in Ts[:3]:
        assert gg._check_boxes(T,g,.05,'y','z',s.normal,s.offset,s.center,s.rotation,s.size,False,.0005)[0]


def test_fixture_does_not_promote_failure_to_success_or_borrow_box_evidence():
    data=json.loads(FIXTURE.read_text())
    assert len(data['cases'])==7
    assert data['original_physical_grasp_success'] is False
    assert data['execution_authorized'] is False
    assert data['silhouette_or_inferred_box_evidence_included'] is False
    assert data['source_record']['sha256']=='da973f4282bc8dffcb304323e932abe346a71b700053ef9691f51d6f8e67050c'
    for case in data['cases']:
        evidence=json.loads(case['plan_data']['contact_geometry_evidence'])
        assert evidence['schema_version']==2 and evidence['measurement_source']=='fused_measured_bilateral_surface'
