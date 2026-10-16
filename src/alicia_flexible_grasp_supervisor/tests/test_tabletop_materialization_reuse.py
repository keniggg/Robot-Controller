"""Extending a CAD grid preserves the full materializer's geometry and IDs."""
from dataclasses import fields, replace

import numpy as np
import pytest

import test_remote_grasp6d_streaming as streaming


def proposal_and_node():
    module = streaming.remote_node
    node = module.RemoteGrasp6DNode.__new__(module.RemoteGrasp6DNode)
    node.gripper_geometry = streaming.tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    geometry = streaming.tabletop_geometry((.040, .035, .021))
    generation = streaming.generate_tabletop_proposals(
        object_points_base=geometry.object_points_base,
        obb_center_base=geometry.center_base, R_base_obb=geometry.axes_base,
        obb_size_xyz_m=geometry.size_xyz_m, support_point_base=np.zeros(3),
        support_normal_base=geometry.support_normal_base,
        config=streaming.TabletopGeometryConfig(max_candidates=8),
    )
    return node, replace(generation.proposals[0], source_index=7)


@pytest.mark.parametrize('probe_tilts, union_tilts', [
    ((), (12.5,)),
    ((10., 25.), (10., 25.)),
    ((10., 25.), (10., 17.3, 25.)),
    ((10., 25.), (5., 10., 17.3, 25., 37.)),
    ((25., 10.), (10., 25.)),
])
def test_extended_grid_equals_complete_cad_materialization(monkeypatch, probe_tilts, union_tilts):
    node, proposal = proposal_and_node()
    original = streaming.remote_node.materialize_tabletop_candidates
    args = dict(proposal=proposal, support_point_base=np.zeros(3),
                support_normal_base=np.array([0., 0., 1.]), gripper=node.gripper_geometry)
    probes = original(**args, approach_tilt_degrees=probe_tilts)
    expected = original(**args, approach_tilt_degrees=union_tilts)
    calls = []
    def materialize(**kwargs):
        calls.append(kwargs['approach_tilt_degrees'])
        return original(**kwargs)
    monkeypatch.setattr(streaming.remote_node, 'materialize_tabletop_candidates', materialize)

    result = node._materialize_tabletop_union(
        proposal, args['support_point_base'], args['support_normal_base'],
        probes, probe_tilts, union_tilts)

    assert len(result) == len(expected)
    for actual, wanted in zip(result, expected):
        for field in fields(wanted):
            first, second = getattr(actual, field.name), getattr(wanted, field.name)
            if isinstance(first, np.ndarray):
                np.testing.assert_array_equal(first, second)
                assert not first.flags.writeable
            else:
                assert first == second, field.name
    additional = tuple(t for t in union_tilts if t not in probe_tilts)
    assert calls == ([additional] if additional else [])


@pytest.mark.parametrize('union_tilts', [(10., 46.), tuple(float(i) for i in range(1, 10))])
def test_extended_grid_retains_angle_and_grid_size_rejection(union_tilts):
    node, proposal = proposal_and_node()
    point, normal = np.zeros(3), np.array([0., 0., 1.])
    probes = streaming.materialize_tabletop_candidates(
        proposal, point, normal, node.gripper_geometry, approach_tilt_degrees=(10.,))
    with pytest.raises(streaming.remote_node.TabletopCandidateContractError) as rejected:
        node._materialize_tabletop_union(proposal, point, normal, probes, (10.,), union_tilts)
    assert rejected.value.code == 'TABLETOP_APPROACH_INVALID'
