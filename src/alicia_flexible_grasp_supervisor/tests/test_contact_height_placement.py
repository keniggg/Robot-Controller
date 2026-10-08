"""Measured contact placement never moves the target or weakens its budget."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from tf.transformations import rotation_matrix
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
    assert 'unchanged translation evidence limit' in a['reason']


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
