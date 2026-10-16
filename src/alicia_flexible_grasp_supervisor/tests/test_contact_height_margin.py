"""Offline finite-pad sensitivity; no ROS node, parameter, or hardware I/O."""
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
import pytest
from genpy.message import fill_message_args
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.contact_height_margin import (
    evaluate_contact_height_margin, plan_contact_height_margin,
)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M as PAIR,
)
from alicia_flexible_grasp.grasp.measured_contact import evaluate_measured_contact


@pytest.fixture(scope='module')
def fixture():
    return json.loads((Path(__file__).parent/'fixtures/contact_height_margin_20260927.json').read_text())


def geometry(fixture):
    return dict(deepcopy(fixture['geometry']),
        gripper=GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),
                                np.array([.1175,.155,.0774]),.003))


def inside(value, intervals):
    return any((a is None or value >= a-1e-10) and (b is None or value <= b+1e-10)
               for a,b in intervals)


def test_recorded_nominal_contact_is_not_robust_to_historical_translation_evidence(fixture):
    args = geometry(fixture)
    before = deepcopy(fixture['geometry'])
    result = evaluate_contact_height_margin(translation_evidence_limit_m=fixture['translation_evidence_limit_m'], **args)
    assert result['evaluated'] and not result['ok']
    assert result['nominal']['ok']
    assert '4.606164 mm' in result['reason'] and '0.758767 mm' in result['reason']
    assert 'Registration calibration' in result['reason']
    np.testing.assert_allclose(result['allowed_normal_translation_intervals_m'],
                               [[-.0007587669858157035,.003820525895443185]],atol=1e-12)
    assert result['maximum_symmetric_normal_translation_m'] == pytest.approx(.0007587669858157035)
    assert result['minimum_contact_overlap_m'] == .002
    assert result['minimum_support_clearance_m'] == .003
    assert result['endpoint_diagnostics'][0]['code'] == 'MEASURED_CONTACT_SUPPORT_CLEARANCE'
    assert result['endpoint_diagnostics'][1]['actual_overlap_m'] == pytest.approx(.001214361731383199)
    assert result['scope'] == 'support_normal_translation_1d_necessary_condition'
    for flag in ('motion_authorized','physical_grip_verified','full_3d_robustness_proven','uncertainty_is_certified_bound'):
        assert result[flag] is False
    assert fixture['geometry'] == before
    json.dumps(result,allow_nan=False)


def test_signed_plan_adapter_checks_planned_pose_and_does_not_apply_centering_override(fixture):
    plan=Grasp6DPlan();fill_message_args(plan,[fixture['plan']])
    for override in (False,True):
        report=plan_contact_height_margin(plan,{'calibration_centering_margin_override':override},
            .04975,fixture['translation_evidence_limit_m'])
        assert report['plan_id'] == plan.plan_id and not report['ok']
        np.testing.assert_allclose(report['allowed_normal_translation_intervals_m'],
            [[-.0005000009236437756,.0043634793816350334]],atol=1e-12)
    plan.poses[2].position.z += .001
    with pytest.raises(ValueError,match='PLAN_CHANGED'):
        plan_contact_height_margin(plan,{},.04975,.0046)


@pytest.mark.parametrize('radius,passed',[(0.,True),(.0005,True),(.00075,True),(.0008,False),(.00460616416406,False)])
def test_symmetric_margin_accounts_for_support_and_pad_contact(fixture,radius,passed):
    result=evaluate_contact_height_margin(translation_evidence_limit_m=radius,**geometry(fixture))
    assert result['ok'] is passed
    assert result['covers_entire_normal_translation_interval'] is passed


@pytest.mark.parametrize('direction',['support_normal','actual_tool_z','actual_jaw_axis'])
def test_recorded_sensitivity_including_ignored_jaw_direction_is_reproducible(fixture,direction):
    args=geometry(fixture)
    actual=np.array(args['actual_T_base_tool0'])
    vector={'support_normal':np.array(args['support_normal_base']),
            'actual_tool_z':actual[:3,2], 'actual_jaw_axis':actual[:3,1]}[direction]
    for row in fixture['counterfactual_sensitivity']:
        if row['direction'] != direction:continue
        shifted=actual.copy();shifted[:3,3]+=row['translation_m']*vector
        result=asdict(evaluate_measured_contact(**dict(args,actual_T_base_tool0=shifted)))
        assert result['ok'] is row['ok']
        assert result['actual_overlap_m'] == pytest.approx(row['actual_overlap_m'],abs=1e-12)
        assert result['minimum_support_clearance_m'] == pytest.approx(row['minimum_support_clearance_m'],abs=1e-12)
    if direction=='actual_jaw_axis':
        # Preserve this as evidence of the existing 1D gate's blind direction,
        # not as a desirable production contact behavior or 3D certificate.
        assert all(row['ok'] for row in fixture['counterfactual_sensitivity'] if row['direction']==direction)


def artificial_geometry(fixture,point,normal):
    args=geometry(fixture)
    pose=np.eye(4);pose[:3,3]=[0,0,.2]
    args.update(actual_T_base_tool0=pose,planned_T_base_tool0=pose.copy(),
        contact_center_base=pose[:3,3]+PAIR+np.array([point[0],0,point[1]]),
        contact_height_axis_base=np.array([0.,0.,1.]),
        contact_height_bounds_m=[-.001,.0015],
        support_normal_base=np.asarray(normal)/np.linalg.norm(normal),support_offset_m=1.)
    return args


@pytest.mark.parametrize('point,normal',[
    ((0.,0.),(.3,.2,1.)), ((-.012,-.028),(1.,0.,0.)),
    ((.012,-.028),(-1.,.3,.4)), ((0.,.025),(.1,1.,.1)),
])
def test_analytic_intervals_match_independent_existing_finite_patch_evaluator(fixture,point,normal):
    args=artificial_geometry(fixture,point,normal)
    report=evaluate_contact_height_margin(translation_evidence_limit_m=.005,**args)
    assert report['evaluated'],report
    actual=np.asarray(args['actual_T_base_tool0'])
    for delta in np.linspace(-.05,.05,401):
        moved=actual.copy();moved[:3,3]+=delta*np.asarray(args['support_normal_base'])
        exact=evaluate_measured_contact(**dict(args,actual_T_base_tool0=moved))
        expected=exact.actual_overlap_m >= args['minimum_contact_overlap_m']-1e-9
        expected &= exact.minimum_support_clearance_m >= args['gripper'].support_clearance_m-1e-9
        assert inside(delta,report['allowed_normal_translation_intervals_m'])==expected,(point,normal,delta,exact,report)


def test_concave_notch_is_not_filled_by_two_passing_translation_endpoints(fixture):
    args=artificial_geometry(fixture,(-.012,-.028),(1.,0.,0.))
    report=evaluate_contact_height_margin(translation_evidence_limit_m=0.,**args)
    intervals=report['allowed_normal_translation_intervals_m']
    assert len(intervals)==2
    assert inside(-.024,intervals) and inside(0.,intervals)
    assert not inside(-.012,intervals)


@pytest.mark.parametrize('value',[True,-1.,float('nan'),float('inf'),'0.0046'])
def test_invalid_evidence_limit_is_unavailable_not_a_safety_pass(fixture,value):
    result=evaluate_contact_height_margin(translation_evidence_limit_m=value,**geometry(fixture))
    assert not result['evaluated'] and not result['ok']
    assert result['code']=='CONTACT_HEIGHT_MARGIN_UNAVAILABLE'
    json.dumps(result,allow_nan=False)
