"""Equivalent concave CAD intersections; never produces execution authority."""
from dataclasses import replace
import types
import numpy as np
import pytest
from alicia_flexible_grasp.grasp import gripper_geometry as reference
from alicia_grasp_modes.contact_intervals import finger_contact_patch_overlap_m, contact_intervals_2d
from test_tabletop_materialization_reuse import proposal_and_node, streaming
from alicia_grasp_modes.contact_probe import contact_tilt_probe


def test_random_rigid_frames_and_height_lines_match_original():
    rng = np.random.RandomState(739)
    for i in range(800):
        rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        rotation[:, 0] *= np.linalg.det(rotation)
        tool0 = rng.uniform(-.7, .7, 3)
        center = tool0 + rotation @ (reference.ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M
                                    + rng.uniform(-.04, .04, 3))
        axis = rng.normal(size=3); axis /= np.linalg.norm(axis)
        lower, upper = sorted(rng.uniform(-.06, .06, 2))
        args = center, tool0, rotation, axis, lower, upper
        assert finger_contact_patch_overlap_m(*args) == pytest.approx(
            reference.finger_contact_patch_overlap_m(*args), abs=1e-12, rel=0.)


def test_concavity_vertices_collinear_edges_and_zero_direction_match_original():
    polygon = reference.ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        for offset in (0., 1e-11, -1e-11, 1e-9, -1e-9):
            for direction in (b-a, np.array([1., 0.]), np.array([0., 1.]), np.zeros(2)):
                point = a + offset
                norm = np.linalg.norm(direction)
                direction = direction / norm if norm else direction
                axis = np.array([direction[0], 0. if norm else 1., direction[1]])
                center = reference.ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M + [point[0], 0., point[1]]
                expected = reference.finger_contact_patch_height_intervals_m(center, np.zeros(3), np.eye(3), axis, -.06, .06)
                actual = contact_intervals_2d(
                    (center - reference.ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M)[[0, 2]],
                    direction, -.06, .06)
                assert len(actual) == len(expected)
                np.testing.assert_allclose(np.array(actual), np.array(expected), atol=1e-12, rtol=0.)


@pytest.mark.parametrize('field,value', [(0,[float('nan'),0,0]), (1,[0,0]),
    (2,np.zeros((3,3))), (2,np.diag([1.,1.,-1.])), (3,[0,0,0]),
    (3,[0,0,1.01]), (4,float('nan')), (5,float('inf')), (4,.02)])
def test_same_invalid_inputs_rejected(field, value):
    args=[np.zeros(3),np.zeros(3),np.eye(3),np.array([0.,0.,1.]),-.01,.01]
    args[field]=value
    for function in (reference.finger_contact_patch_overlap_m, finger_contact_patch_overlap_m):
        with pytest.raises(ValueError): function(*args)


def test_boundary_search_still_returns_identical_tilts_and_evidence():
    node, proposal = proposal_and_node()
    point, normal = np.zeros(3), np.array([0.,0.,1.])
    variants = streaming.materialize_tabletop_candidates(proposal, point, normal,
        node.gripper_geometry, approach_tilt_degrees=(10.,25.,45.))
    probe = contact_tilt_probe(proposal, point, normal, node.gripper_geometry, 'y', 'z')
    original_overlap = node._tabletop_candidate_contact_overlap_m
    def fast(proposal, candidate, support, contact_height_bounds_m=None, contact_height_axis_base_override=None):
        lower, upper = node._proposal_contact_height_bounds(proposal) if contact_height_bounds_m is None else contact_height_bounds_m
        t = candidate.T_base_tool0
        axis = reference.contact_height_axis_base(t[:3,1], support) if contact_height_axis_base_override is None else contact_height_axis_base_override
        return finger_contact_patch_overlap_m(candidate.contact_center_base,t[:3,3],t[:3,:3],axis,lower,upper)
    for overlap in (.001,.002,.003,.006,.01):
        node._tabletop_candidate_contact_overlap_m = original_overlap
        expected=node._contact_boundary_tilts(proposal,point,normal,variants,45.,overlap,contact_probe=probe)
        node._tabletop_candidate_contact_overlap_m = fast
        actual=node._contact_boundary_tilts(proposal,point,normal,variants,45.,overlap,contact_probe=probe)
        assert actual[0] == expected[0]
        for got,wanted in zip(actual[1],expected[1]): assert got == pytest.approx(wanted,abs=1e-12)
