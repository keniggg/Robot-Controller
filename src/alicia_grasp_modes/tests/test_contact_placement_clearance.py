"""Positive placement clearance is retained before frozen contact publication."""
from types import SimpleNamespace
import numpy as np
import pytest
from test_remote_modes import configured, base
from test_contact_tilt_replay import configured_replay
from alicia_grasp_modes.contact_probe import contact_tilt_probe
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    generate_tabletop_proposals, materialize_tabletop_candidates, _open_finger_corners)


@pytest.mark.parametrize('mode,near,explicit,expected',[
    ('unknown',True,None,.0035),('unknown',False,None,.003),
    ('unknown',False,True,.0035),('unknown',True,False,.003),
    ('carton',True,None,.003),('carton',False,True,.003)])
def test_placement_margin_is_contact_unknown_only_and_never_mutates_gate(configured,mode,near,explicit,expected):
    node,*_=configured_replay(configured,'gen3_contact_tilt_20260926.json')
    physical=node.gripper_geometry
    node._mode_selection={'mode':mode};node.near_field_planning_active=near
    placement=node._tabletop_placement_gripper(explicit)
    assert placement.support_clearance_m == pytest.approx(expected)
    assert node.gripper_geometry is physical and physical.support_clearance_m==.003
    np.testing.assert_array_equal(placement.finger_size_xyz_m,physical.finger_size_xyz_m)
    np.testing.assert_array_equal(placement.palm_size_xyz_m,physical.palm_size_xyz_m)
    assert base.RemoteGrasp6DNode._tabletop_placement_gripper(node,True) is physical


def test_generated_boundary_union_probe_and_reprojection_keep_same_positive_clearance(configured):
    node,geometry,snapshot,frozen,_=configured_replay(configured,'gen3_contact_tilt_20260926.json')
    candidates,audit=node._generate_tabletop_candidates(geometry,snapshot)
    assert candidates
    normal=geometry.support_normal_base;point=-geometry.support_offset_m*normal
    def clearance(t):
        return float(np.min((_open_finger_corners(t,node.gripper_geometry,'y','z')-point)@normal))
    for item in candidates:
        assert clearance(item.T_base_tool0)==pytest.approx(.0035,abs=1e-9)
    proposal=generate_tabletop_proposals(geometry.object_points_base,geometry.center_base,
        geometry.axes_base,geometry.size_xyz_m,point,normal,node.tabletop_geometry_config,
        fused_surface=frozen[0],finger_geometry=node.gripper_geometry).proposals[0]
    placement=node._tabletop_placement_gripper()
    probe=contact_tilt_probe(proposal,point,normal,placement,'y','z')
    item=probe(5.625,(0,-1.))
    materialized=materialize_tabletop_candidates(proposal,point,normal,placement,'y','z',
        approach_tilt_degrees=(5.625,),branch_key=(0,-1.))
    selected=next(c for c in materialized if c.audit['approach_tilt_deg']==5.625)
    np.testing.assert_allclose(item.T_base_tool0,selected.T_base_tool0,atol=1e-12)
    c=candidates[0]
    pose=base.make_pose_stamped('base_link',c.T_base_tool0[:3,3],
        base.quaternion_from_matrix(c.T_base_tool0),stamp=base.rospy.Time(1))
    pose.pose.position.z -= .001
    stable=SimpleNamespace(approach_base_xyz=c.insertion_axis_base,center_base_xyz=c.contact_center_base)
    reprojected=node._reproject_tabletop_stable_pose_for_support_clearance(pose,stable,geometry)
    assert clearance(base.pose_matrix(reprojected))==pytest.approx(.0035,abs=1e-9)
    assert node.gripper_geometry.support_clearance_m==.003
