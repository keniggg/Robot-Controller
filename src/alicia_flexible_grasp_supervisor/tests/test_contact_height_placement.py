"""Measured contact placement never moves the target or weakens its budget."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from tf.transformations import rotation_matrix
from alicia_flexible_grasp.grasp import contact_height_placement as placement
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def case():
    fixture=json.loads((Path(__file__).parent/'fixtures/contact_height_margin_20260927.json').read_text())
    geometry = deepcopy(fixture['geometry'])
    # This helper places a proposal before execution; the recorded measured
    # FK belongs to the execution-margin tests, not this planning input.
    geometry['actual_T_base_tool0'] = deepcopy(geometry['planned_T_base_tool0'])
    return dict(geometry, gripper=GripperGeometry(.05,.002,
        np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003))


def test_balances_finite_measured_window_without_changing_target_or_budget():
    g=case();before=deepcopy(g)
    T,a=place_contact_height(translation_evidence_limit_m=.001,**g)
    assert T is not None and not a['before']['ok'] and a['after']['ok']
    assert a['after']['translation_evidence_limit_m']==.001
    assert a['after']['minimum_contact_overlap_m']==.002
    assert a['after']['minimum_support_clearance_m']==.003
    delta=T[:3,3]-np.array(g['actual_T_base_tool0'])[:3,3]
    np.testing.assert_allclose(delta,a['normal_translation_m']*np.array(g['support_normal_base']),atol=1e-14)
    np.testing.assert_allclose(T[:3,:3],np.array(g['actual_T_base_tool0'])[:3,:3])
    for name in g:
        if name!='gripper':np.testing.assert_array_equal(g[name],before[name])
    assert a['motion_authorized'] is False and a['target_relocated'] is False


def test_already_robust_pose_is_preserved_exactly():
    g=case();T,a=place_contact_height(translation_evidence_limit_m=.0005,**g)
    np.testing.assert_array_equal(T,g['actual_T_base_tool0'])
    assert a['normal_translation_m']==0.


def test_narrow_window_cannot_be_fixed_by_relabeling_or_larger_shift():
    T,a=place_contact_height(translation_evidence_limit_m=.00460616416406,**case())
    assert T is None and not a['before']['ok']
    assert a['before']['evaluated'] and a['before']['nominal']['ok']
    lo,hi=next((lo,hi) for lo,hi in a['before']['allowed_normal_translation_intervals_m']
               if lo is not None and hi is not None and lo <= 0. <= hi)
    assert hi-lo < 2.*a['before']['translation_evidence_limit_m']
    assert a['normal_translation_m']==0. and 'after' not in a
    assert 'unchanged translation evidence limit' in a['reason']


@pytest.mark.parametrize('nominal_margin_m',[.000000057,.0005])
def test_small_nominal_margin_does_not_discard_a_wide_connected_window(nominal_margin_m):
    # R4 analytic boundaries, not an R6 cloud/path replay. Move the proposed
    # tool near the existing support endpoint; retain its measured target band.
    row=json.loads((Path(__file__).parent/'fixtures/gen16_r4_final_height_boundaries.json').read_text())['rows'][0]
    g=dict(deepcopy(row['geometry']),gripper=case()['gripper'])
    before=deepcopy(g)
    limit=.0026767676132463574
    lo,hi=row['original_report']['allowed_normal_translation_intervals_m'][0]
    assert hi-lo > 2.*limit
    normal=np.asarray(g['support_normal_base'])
    shifted=np.array(g['actual_T_base_tool0'],dtype=float)
    shift=lo+nominal_margin_m
    shifted[:3,3]+=shift*normal
    g.update(actual_T_base_tool0=shifted,planned_T_base_tool0=shifted.copy())

    T,a=place_contact_height(translation_evidence_limit_m=limit,**g)

    assert a['before']['nominal']['ok'] and not a['before']['ok']
    assert a['before']['maximum_symmetric_normal_translation_m']==pytest.approx(nominal_margin_m,abs=1e-12)
    assert T is not None and a['after']['ok']
    assert a['after']['translation_evidence_limit_m']==limit
    assert a['after']['minimum_contact_overlap_m']==.002
    assert a['after']['minimum_support_clearance_m']==.003
    assert lo+limit <= shift+a['normal_translation_m'] <= hi-limit
    np.testing.assert_allclose(T[:3,3]-shifted[:3,3],a['normal_translation_m']*normal,atol=1e-14)
    np.testing.assert_array_equal(T[:3,:3],shifted[:3,:3])
    for name in ('contact_center_base','contact_height_axis_base','contact_height_bounds_m',
                 'support_normal_base','support_offset_m'):
        np.testing.assert_array_equal(g[name],before[name])
    assert a['motion_authorized'] is False and a['target_relocated'] is False


def test_nominal_support_collision_cannot_be_repaired_by_placement():
    g=case();limit=.001
    report=evaluate_contact_height_margin(translation_evidence_limit_m=limit,**g)
    lo,_=next((lo,hi) for lo,hi in report['allowed_normal_translation_intervals_m']
              if lo is not None and hi is not None and lo <= 0. <= hi)
    shifted=np.array(g['actual_T_base_tool0'],dtype=float)
    shifted[:3,3]+=(lo-.0001)*np.asarray(g['support_normal_base'])
    g.update(actual_T_base_tool0=shifted,planned_T_base_tool0=shifted.copy())

    T,a=place_contact_height(translation_evidence_limit_m=limit,**g)

    assert T is None and a['before']['evaluated']
    assert not a['before']['nominal']['ok']
    assert a['before']['nominal']['code']=='MEASURED_CONTACT_SUPPORT_CLEARANCE'
    assert a['normal_translation_m']==0. and 'after' not in a
    np.testing.assert_array_equal(g['actual_T_base_tool0'],shifted)
    assert a['motion_authorized'] is False


def test_recenter_rechecks_changed_measurement_instead_of_reusing_first_margin(monkeypatch):
    g=case();original=deepcopy(g);calls=[]
    real=evaluate_contact_height_margin
    def changed_measurement(*,translation_evidence_limit_m,**geometry):
        calls.append(np.array(geometry['actual_T_base_tool0'],copy=True))
        if len(calls)==2:
            lower=geometry['contact_height_bounds_m'][0]
            geometry=dict(geometry,contact_height_bounds_m=[lower,lower+.0001])
        return real(translation_evidence_limit_m=translation_evidence_limit_m,**geometry)
    monkeypatch.setattr(placement,'evaluate_contact_height_margin',changed_measurement)

    T,a=place_contact_height(translation_evidence_limit_m=.001,**g)

    assert len(calls)==2 and not np.array_equal(calls[0],calls[1])
    assert a['before']['nominal']['ok'] and a['normal_translation_m']!=0.
    assert T is None and not a['after']['ok']
    assert a['after']['nominal']['code']=='FROZEN_CONTACT_EVIDENCE_INVALID'
    assert a['after']['translation_evidence_limit_m']==.001
    assert a['after']['minimum_contact_overlap_m']==.002
    assert a['after']['minimum_support_clearance_m']==.003
    for name in g:
        if name!='gripper':np.testing.assert_array_equal(g[name],original[name])
    assert a['motion_authorized'] is False and a['target_relocated'] is False


def test_result_is_covariant_with_rigid_base_frame_change():
    g=case();T,a=place_contact_height(translation_evidence_limit_m=.001,**g)
    Q=rotation_matrix(.63,[.4,.2,.7]);Q[:3,3]=[.12,-.07,.3];h=deepcopy(g)
    for key in ('actual_T_base_tool0','planned_T_base_tool0'):h[key]=Q@np.array(g[key])
    h['contact_center_base']=Q[:3,:3]@g['contact_center_base']+Q[:3,3]
    for key in ('contact_height_axis_base','support_normal_base'):h[key]=Q[:3,:3]@g[key]
    h['support_offset_m']=g['support_offset_m']-h['support_normal_base']@Q[:3,3]
    U,b=place_contact_height(translation_evidence_limit_m=.001,**h)
    np.testing.assert_allclose(U,Q@T,atol=1e-12)
    assert b['normal_translation_m']==pytest.approx(a['normal_translation_m'])


@pytest.mark.parametrize('limit',[True,'0.001',float('nan'),float('inf'),-.001])
def test_invalid_evidence_cannot_create_a_pose(limit):
    T,a=place_contact_height(translation_evidence_limit_m=limit,**case())
    assert T is None and a['motion_authorized'] is False
