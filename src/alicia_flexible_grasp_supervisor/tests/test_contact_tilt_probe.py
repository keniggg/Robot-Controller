"""Unknown-only cached probes must preserve the authoritative CAD boundaries."""
from dataclasses import replace
import numpy as np
import pytest

from test_tabletop_materialization_reuse import proposal_and_node, streaming
from alicia_grasp_modes.contact_probe import contact_tilt_probe


@pytest.mark.parametrize('seed', range(12))
def test_probe_matches_full_cad_in_arbitrary_rigid_frame(seed):
    node, proposal = proposal_and_node()
    rng = np.random.RandomState(seed)
    rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    rotation[:, 0] *= np.linalg.det(rotation)
    point = rng.uniform(-.5, .5, 3)
    normal = rotation[:, 2]
    proposal = replace(proposal,
        contact_center_base=rotation @ proposal.contact_center_base + point,
        jaw_axis_base=rotation @ proposal.jaw_axis_base,
        insertion_axis_base=rotation @ proposal.insertion_axis_base)
    probe = contact_tilt_probe(proposal, point, normal, node.gripper_geometry, 'y', 'z')
    for angle in (1e-4, 12.317, 25., 44.999, 45.):
        for branch in ((0, -1.), (0, 1.), (1, -1.), (1, 1.)):
            actual = probe(angle, branch)
            full = streaming.materialize_tabletop_candidates(proposal, point, normal,
                node.gripper_geometry, approach_tilt_degrees=(angle,), branch_key=branch)[0]
            np.testing.assert_allclose(actual.T_base_tool0, full.T_base_tool0, rtol=0., atol=1e-12)
            np.testing.assert_allclose(actual.contact_center_base, full.contact_center_base, rtol=0., atol=1e-12)
            assert node._tabletop_candidate_contact_overlap_m(proposal, actual, normal) == pytest.approx(
                node._tabletop_candidate_contact_overlap_m(proposal, full, normal), abs=1e-12)


@pytest.mark.parametrize('overlap', [.001, .003, .006, .010])
def test_identical_bisection_boundaries_and_audit(overlap):
    node, proposal = proposal_and_node()
    point, normal = np.zeros(3), np.array([0., 0., 1.])
    variants = streaming.materialize_tabletop_candidates(proposal, point, normal,
        node.gripper_geometry, approach_tilt_degrees=(10., 25., 45.))
    args = (proposal, point, normal, variants, 45., overlap)
    expected = node._contact_boundary_tilts(*args)
    probe = contact_tilt_probe(proposal, point, normal, node.gripper_geometry, 'y', 'z')
    actual = node._contact_boundary_tilts(*args, contact_probe=probe)
    assert actual[0] == expected[0]
    for got, wanted in zip(actual[1], expected[1]):
        assert got == pytest.approx(wanted, abs=1e-12)


@pytest.mark.parametrize('angle,branch', [(0.,(0,1.)), (46.,(0,1.)), (float('nan'),(0,1.)),
    (25.,(True,1.)), (25.,(2,1.)), (25.,(0,0.)), (25.,(0,float('nan')))])
def test_invalid_probe_branch_or_angle_rejected(angle, branch):
    node, proposal = proposal_and_node()
    probe = contact_tilt_probe(proposal, np.zeros(3), np.array([0.,0.,1.]), node.gripper_geometry, 'y', 'z')
    with pytest.raises(streaming.remote_node.TabletopCandidateContractError):
        probe(angle, branch)


@pytest.mark.parametrize('invalid', ['normal', 'height', 'jaw', 'clearance'])
def test_invalid_geometry_still_rejected_by_authoritative_materializer(invalid):
    node, proposal = proposal_and_node()
    point, normal = np.zeros(3), np.array([0.,0.,1.])
    if invalid == 'normal': normal[2] = 0.
    if invalid == 'height': proposal = replace(proposal, contact_center_base=np.array([0.,0.,-.1]))
    if invalid == 'jaw': proposal = replace(proposal, jaw_axis_base=np.array([0.,0.,1.]))
    if invalid == 'clearance': node.gripper_geometry = replace(node.gripper_geometry, jaw_clearance_each_side_m=.003)
    with pytest.raises(streaming.remote_node.TabletopCandidateContractError):
        contact_tilt_probe(proposal, point, normal, node.gripper_geometry, 'y', 'z')


def test_alternative_axis_convention_retains_original_path():
    assert contact_tilt_probe(None, None, None, None, 'x', 'z') is None
