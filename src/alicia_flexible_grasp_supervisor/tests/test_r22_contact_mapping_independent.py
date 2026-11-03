"""Independent mapped-contact checks with synthetic offline message histories."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest
import rospy

from alicia_flexible_grasp.robot.contact_tracking_contract import (
    _microstep, build_contact_tracking_contract, revalidate_contact_tracking_contract,
    validate_contact_tracking_contract_binding,
)
from alicia_flexible_grasp.robot.following_model import source_bias_model
from alicia_flexible_grasp.robot.physical_reference_contract import (
    physical_plan_from_wire, physical_reference_from_wire,
)
from alicia_flexible_grasp.robot.stationary_following import stationary_following_error, SDK_QUANTUM_RAD as Q
from test_contact_tracking_contract import inputs


def fixture():
    wire=inputs()
    sample=stationary_following_error(wire[2],wire[3],wire[4],now_sec=10.05,epoch_ns=9_000_000_000)
    m=source_bias_model(sample,binding_id='synthetic-contact-model',source_bias_limit_rad=(.09,)*6)
    mapped=list(wire)
    mapped[0]=physical_plan_from_wire(wire[0],m)
    mapped[1]=physical_reference_from_wire(wire[1],m)
    return wire,mapped,m


def build(args,m):
    return build_contact_tracking_contract(*args,now_sec=10.05,epoch_ns=9_000_000_000,following_model=m)


def fresh(c,args,m,now=10.05):
    return revalidate_contact_tracking_contract(c,*args,now_sec=now,epoch_ns=9_000_000_000,following_model=m)


def binding(c,args):
    return validate_contact_tracking_contract_binding(c,args[0],args[1],args[5],args[6])


def test_r22_mapped_microstep_preserves_wire_budget_and_separate_residual():
    wire,mapped,m=fixture();c=build(mapped,m)
    assert c['reference_domain']=='physical' and c['source_bias_sha256']==m.digest
    assert binding(c,mapped) and fresh(c,mapped,m)
    assert not c['motion_authorized'] and c['geometry_proof_required']
    assert c['continuous_accepted_feedback_monitor_required']
    assert c['raw_sdk_minus_encoder_rad'][1]==pytest.approx(8*Q)
    assert c['physical_reference_minus_encoder_rad'][1]==pytest.approx(0.)
    restored=physical_plan_from_wire(wire[0],m)
    assert restored==mapped[0]
    assert max(c['goal_position_tolerance_rad'])<=.035


@pytest.mark.parametrize('fault',['five_counts','unquantized_wire','start_changed','held_j2_spline','held_j5_spline'])
def test_r22_mapping_does_not_relax_wire_count_or_held_spline_checks(fault):
    _,args,m=fixture();p=args[0].joint_trajectory.points
    if fault=='five_counts':p[-1].positions[5]+=Q
    elif fault=='unquantized_wire':p[-1].positions[5]+=.25*Q
    elif fault=='start_changed':p[0].positions[0]+=Q
    else:p[-1].velocities[1 if fault=='held_j2_spline' else 4]=.01
    with pytest.raises(ValueError):build(args,m)


def test_r22_physical_target_need_not_be_on_sdk_grid_but_wire_must_be():
    wire,_,old=fixture()
    # Explicit synthetic analog source: .25-count bias verifies that the SDK
    # grid applies to restored wire, not to the physical coordinate system.
    actual=list(old.source_accepted_positions_rad);actual[0]-=.25*Q
    m=source_bias_model(dict(model_sha256=old.model_sha256,epoch_ns=old.epoch_ns,
        joint_names=old.joint_names,sdk_stamp_ns=old.source_sdk_stamp_ns,
        accepted_stamp_ns=old.source_accepted_stamp_ns,
        sdk_positions_rad=old.source_sdk_positions_rad,accepted_positions_rad=actual),
        binding_id='synthetic-fractional-bias',source_bias_limit_rad=(.09,)*6)
    p=physical_plan_from_wire(wire[0],m);h=physical_reference_from_wire(wire[1],m)
    q=_microstep(p,h,np.asarray(wire[3][-1].position),following_model=m)
    np.testing.assert_allclose(q,h.positions)
    assert abs(p.joint_trajectory.points[-1].positions[0]/Q-round(p.joint_trajectory.points[-1].positions[0]/Q))>.1


@pytest.mark.parametrize('model_change',['missing','new_id','new_cap'])
def test_r22_submission_cannot_replace_or_omit_bound_model(model_change):
    _,args,m=fixture();c=build(args,m)
    if model_change=='missing':other=None
    elif model_change=='new_id':other=replace(m,binding_id='different-episode')
    else:other=replace(m,source_bias_limit_rad=(.08,)*6)
    with pytest.raises(ValueError):fresh(c,args,other)


@pytest.mark.parametrize('field',['raw_sdk_minus_encoder_rad','physical_reference_minus_encoder_rad','source_bias_sha256','reference_domain'])
def test_r22_frozen_entry_model_or_residual_cannot_be_overwritten(field):
    _,args,m=fixture();c=build(args,m)
    if field=='source_bias_sha256':c[field]='0'*64
    elif field=='reference_domain':c[field]='legacy'
    else:c[field][1]+=Q
    with pytest.raises(ValueError):binding(c,args)


def test_r22_legacy_contract_cannot_silently_become_mapped():
    wire,_,m=fixture();c=build(wire,None)
    assert 'reference_domain' not in c and 'source_bias_model' not in c
    with pytest.raises(ValueError):fresh(c,wire,m)


def test_r22_fresh_drift_cannot_widen_contact_envelope_or_reestimate_bias():
    _,args,m=fixture();c=build(args,m);saved=deepcopy(c)
    args=deepcopy(args)
    for msg in args[3]+args[4]:msg.header.stamp+=rospy.Duration.from_sec(.02)
    for msg in args[4]:msg.position[1]-=2*Q
    with pytest.raises(ValueError):fresh(c,args,m,now=10.07)
    assert c==saved and c['source_bias_sha256']==m.digest


def test_r22_mapped_contract_still_rejects_stale_history():
    _,args,m=fixture();c=build(args,m)
    with pytest.raises(ValueError):fresh(c,args,m,now=11.)
