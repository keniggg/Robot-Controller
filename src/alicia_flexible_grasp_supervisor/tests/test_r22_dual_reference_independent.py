"""Independent dual-proof binding negatives using explicit synthetic proofs.

The proof numbers come from the existing mock bound-action fixture. They are
not real CAD results and do not authorize hardware execution.
"""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace as NS
import math

import pytest
from alicia_flexible_grasp.robot.physical_reference_contract import (
    bind_dual_reference, physical_plan_from_wire, physical_reference_from_wire,
    validate_dual_reference, message_bytes, message_sha256,
    commanded_reference_bounds, WIRE_KIND,
)
from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
from test_contracted_trajectory import example
from test_r22_reference_map_independent import model


def fixture():
    wire, audit, cfg = example()
    m = model()
    wr = NS(positions=tuple(wire.joint_trajectory.points[0].positions),
            velocities=(0.,)*6, accelerations=(0.,)*6)
    pr = physical_reference_from_wire(wr, m)
    physical = physical_plan_from_wire(wire, m)
    audit.update(model_sha256=m.model_sha256, scene_sha256='c'*64,
        plan_id='explicit-synthetic-plan', opening_width_m=.0498,
        bridge='stationary_desired_to_first_positive_time_waypoint',
        bridge_start_joint_positions_rad=list(wr.positions),
        target_cad_projection=dict(target_envelope_unchanged=True,
            support_and_palm_checks_unchanged=True),
        controller_constraints_snapshot=cfg,
        gripper_geometry_snapshot={'max_inner_gap_m':.05,'tool_jaw_axis':'y'})
    pa = deepcopy(audit)
    pa.update(trajectory_sha256=wire_digest(physical),
        bridge_start_joint_positions_rad=list(pr.positions))
    # The geometry numbers remain explicitly synthetic. The commanded curve,
    # stop and quantization bounds use the real pure checker, never a fictional
    # raw SDK-versus-encoder enforcement contract for the wire reference.
    bounds=commanded_reference_bounds(wire,wr,.5)
    del audit['execution_tracking_contract']
    audit.update(kind=WIRE_KIND,reference_domain='wire',wire_reference_contract=bounds,
        raw_tracking_enforced=False,target_obb_tracking_uncertainty_checked=False,
        joint_limit_uncertainty_checked=False)
    audit['following_support'].update(joint_error_bounds_rad=bounds['joint_error_bounds_rad'],
        target_obb_tracking_uncertainty_checked=False)
    return dict(wire_plan=wire, physical_plan=physical, wire_audit=audit,
        physical_audit=pa, following_model=m, wire_reference=wr, physical_reference=pr)


def test_r22_two_synthetic_complete_proofs_bind_exact_message_bytes():
    f = fixture(); c = bind_dual_reference(**f)
    assert validate_dual_reference(c) is c
    goal = c.action_goal()
    assert message_bytes(goal.trajectory) == message_bytes(f['physical_plan'].joint_trajectory)
    assert c.wire_trajectory_sha256 == message_sha256(f['wire_plan'].joint_trajectory)
    assert c.physical_trajectory_sha256 == message_sha256(f['physical_plan'].joint_trajectory)
    assert c.wire_trajectory_sha256 != c.physical_trajectory_sha256
    assert [v.position for v in goal.goal_tolerance] == [.035]*6
    assert not hasattr(c, 'execution_authorized')


@pytest.mark.parametrize('field', ['positions','velocities','accelerations','effort','time'])
def test_r22_physical_curve_cannot_diverge_from_wire_curve(field):
    f=fixture(); p=f['physical_plan'].joint_trajectory.points[-1]
    if field=='time': p.time_from_start.nsecs += 1
    elif field=='effort': p.effort=[0.]*6
    else:
        row=list(getattr(p,field));row[0]+=1e-9;setattr(p,field,row)
    with pytest.raises(ValueError): bind_dual_reference(**f)


@pytest.mark.parametrize('side', ['wire_audit','physical_audit'])
@pytest.mark.parametrize('fault', ['missing_proof','changed_digest','no_bridge','clearance',
    'target_shrunk','support_relaxed','model','reference','tracking_missing'])
def test_r22_each_curve_needs_its_own_complete_proof(side, fault):
    f=fixture(); a=f[side]
    if fault=='missing_proof': f[side]={}
    elif fault=='changed_digest': a['trajectory_sha256']='0'*64
    elif fault=='no_bridge': a['following_support']['includes_controller_bridge']=False
    elif fault=='clearance': a['following_support']['minimum_support_clearance_lower_bound_m']=.002999
    elif fault=='target_shrunk': a['target_cad_projection']['target_envelope_unchanged']=False
    elif fault=='support_relaxed': a['target_cad_projection']['support_and_palm_checks_unchanged']=False
    elif fault=='model': a['model_sha256']='f'*64
    elif fault=='reference': a['bridge_start_joint_positions_rad'][0]+=1e-9
    else: del a['wire_reference_contract' if side=='wire_audit' else 'execution_tracking_contract']
    with pytest.raises(ValueError): bind_dual_reference(**f)


@pytest.mark.parametrize('fault', ['scene','plan','opening','gripper'])
def test_r22_two_domains_cannot_use_different_frozen_geometry(fault):
    f=fixture(); a=f['physical_audit']
    if fault=='scene': a['scene_sha256']='d'*64
    elif fault=='plan': a['plan_id']='different-plan'
    elif fault=='opening': a['opening_width_m']=.0497
    else: a['gripper_geometry_snapshot']['max_inner_gap_m']=.06
    with pytest.raises(ValueError): bind_dual_reference(**f)


@pytest.mark.parametrize('missing', ['opening_width_m','plan_id','gripper_geometry_snapshot'])
def test_r22_equal_missing_geometry_fields_do_not_make_a_complete_proof(missing):
    f=fixture()
    for side in ('wire_audit','physical_audit'):del f[side][missing]
    with pytest.raises(ValueError): bind_dual_reference(**f)


def test_r22_scene_digest_must_be_hex_not_just_sixty_four_characters():
    f=fixture()
    for side in ('wire_audit','physical_audit'):f[side]['scene_sha256']='z'*64
    with pytest.raises(ValueError): bind_dual_reference(**f)


def test_r22_source_bias_capture_cap_cannot_exceed_controller_configuration():
    f=fixture()
    from alicia_flexible_grasp.robot.observation_tracking_contract import tracking_contract_candidates
    cfg=deepcopy(f['wire_audit']['controller_constraints_snapshot'])
    for name in f['wire_plan'].joint_trajectory.joint_names:cfg[name]['trajectory']=.08
    tracking=tracking_contract_candidates(cfg,f['wire_plan'].joint_trajectory.joint_names,.5)[1]
    for side in ('wire_audit','physical_audit'):
        f[side]['controller_constraints_snapshot']=deepcopy(cfg)
        if side=='physical_audit':
            f[side]['execution_tracking_contract']=deepcopy(tracking)
            f[side]['following_support']['joint_error_bounds_rad']=tracking['joint_error_bounds_rad']
    with pytest.raises(ValueError,match='SOURCE_CAP_CONFIG_MISMATCH'):bind_dual_reference(**f)


def test_r22_frozen_contract_does_not_alias_mutable_input_or_returned_messages():
    f=fixture(); c=bind_dual_reference(**f); saved=c.proof_sha256
    f['wire_plan'].joint_trajectory.points[0].positions[0]+=1.
    f['wire_audit']['opening_width_m']=.03
    copy=c.physical_plan();positions=list(copy.joint_trajectory.points[0].positions)
    positions[0]+=1.;copy.joint_trajectory.points[0].positions=positions
    assert c.proof_sha256==saved and validate_dual_reference(c) is c
    with pytest.raises(FrozenInstanceError):c.plan_sha256='f'*64


@pytest.mark.parametrize('field',['wire_trajectory_sha256','physical_trajectory_sha256','plan_sha256'])
def test_r22_replaced_digest_cannot_validate(field):
    c=bind_dual_reference(**fixture())
    with pytest.raises(ValueError):validate_dual_reference(replace(c,**{field:'0'*64}))


def test_r22_live_wire_or_physical_plan_change_invalidates_frozen_contract():
    c=bind_dual_reference(**fixture());wire=c.wire_plan();physical=c.physical_plan()
    positions=list(wire.joint_trajectory.points[-1].positions)
    positions[0]+=1e-12;wire.joint_trajectory.points[-1].positions=positions
    with pytest.raises(ValueError):validate_dual_reference(c,wire_plan=wire)
    positions=list(physical.joint_trajectory.points[-1].positions)
    positions[0]+=1e-12;physical.joint_trajectory.points[-1].positions=positions
    with pytest.raises(ValueError):validate_dual_reference(c,physical_plan=physical)


@pytest.mark.parametrize('fault',['raw_tracking','old_execution_contract','target_uncertainty',
    'joint_limit_uncertainty','support_target_scope','changed_stop','changed_quantization'])
def test_r22_wire_reference_cannot_claim_physical_tracking_or_larger_geometry_scope(fault):
    f=fixture();a=f['wire_audit']
    if fault=='raw_tracking':a['raw_tracking_enforced']=True
    elif fault=='old_execution_contract':a['execution_tracking_contract']=deepcopy(f['physical_audit']['execution_tracking_contract'])
    elif fault=='target_uncertainty':a['target_obb_tracking_uncertainty_checked']=True
    elif fault=='joint_limit_uncertainty':a['joint_limit_uncertainty_checked']=True
    elif fault=='support_target_scope':a['following_support']['target_obb_tracking_uncertainty_checked']=True
    elif fault=='changed_stop':a['wire_reference_contract']['commanded_stop_reserve_rad']=0.
    else:a['wire_reference_contract']['sdk_quantization_bound_rad']=0.
    with pytest.raises(ValueError):bind_dual_reference(**f)


def test_r22_wire_support_bounds_are_recomputed_and_not_raw_following_tolerance():
    f=fixture();c=bind_dual_reference(**f)
    wire=f['wire_audit']['wire_reference_contract']
    physical=f['physical_audit']['execution_tracking_contract']
    assert wire['raw_tracking_enforced'] is False
    assert wire['target_obb_tracking_uncertainty_checked'] is False
    assert wire['joint_limit_uncertainty_checked'] is False
    assert wire['sdk_quantization_bound_rad']==math.pi/4096
    assert all(a<b for a,b in zip(wire['joint_error_bounds_rad'],physical['joint_error_bounds_rad']))
    assert c.proof_kind=='observation'


@pytest.mark.parametrize('value',[math.pi,math.pi-math.pi/4096,-math.pi-1e-12])
def test_r22_constant_curve_in_sdk_clipping_or_outside_domain_is_rejected(value):
    f=fixture();p=f['wire_plan'];ref=f['wire_reference']
    for point in p.joint_trajectory.points:point.positions=[value]*6
    ref.positions=(value,)*6
    with pytest.raises(ValueError,match='CONTINUOUS_RANGE_OR_STOP'):commanded_reference_bounds(p,ref,.5)


def test_r22_continuous_sdk_domain_checks_bridge_and_stop_not_only_waypoints():
    f=fixture();p=f['wire_plan'];ref=f['wire_reference']
    # Entire nominal curve stays below the last SDK position, but its
    # commanded-stop reserve exceeds the remaining domain at the endpoint.
    hi=math.pi-2*math.pi/4096
    ref.positions=(hi-.055,)*6
    p.joint_trajectory.points[0].positions=list(ref.positions)
    p.joint_trajectory.points[1].positions=[hi-.005]*6
    assert all(max(pt.positions)<hi for pt in p.joint_trajectory.points)
    with pytest.raises(ValueError,match='CONTINUOUS_RANGE_OR_STOP'):commanded_reference_bounds(p,ref,.5)


def test_r22_complete_original_wire_survives_one_ulp_inverse_roundtrip():
    f=fixture();p=f['wire_plan'];m=f['following_model']
    # The historical problematic number is a closed-record point. This test
    # only verifies representation; no geometry or execution is inferred.
    p.joint_trajectory.points[0].positions[1]=-.49451705649463357
    physical=physical_plan_from_wire(p,m)
    back=physical.joint_trajectory.points[0].positions[1]+m.bias_rad[1]
    original=p.joint_trajectory.points[0].positions[1]
    import numpy as np
    assert back!=original and abs(back-original)<=abs(np.spacing(original))
    assert physical.joint_trajectory.points[0].positions[1]==original-m.bias_rad[1]
