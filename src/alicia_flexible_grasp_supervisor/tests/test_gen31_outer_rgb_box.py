"""Current RGB outer contour replaces absent depth-edge support explicitly."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from test_gen26_regular_box_inference_independent import ray_scene, qualify
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top


def visual_edge_gap_scene():
    scene = ray_scene(width=.035, length=.065)
    actual = scene.mask > 0
    scene.color = np.full(scene.depth.shape+(3,), 20, np.uint8)
    scene.color[actual] = [70, 140, 220]
    # Actual RGB remains complete. Segmentation is clipped at its corners,
    # and one depth edge is absent; neither creates artificial measured data.
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    corners = (np.abs(x) > .029) & (np.abs(y) > .014)
    scene.mask[corners] = 0
    missing = actual & (y > .012)
    scene.depth[missing] = 0
    points = scene.geometry.object_points_base
    keep = (points@scene.geometry.axes_base)[:, 1] <= .012
    scene.geometry = replace(scene.geometry, object_points_base=points[keep])
    return scene


def qualify_visual_scene(scene, **changes):
    return qualify(scene, color_bgr=scene.color, **changes)


def test_full_rgb_outer_edges_qualify_explicit_v2_with_depth_gaps_audited():
    scene = visual_edge_gap_scene()
    original = scene.geometry.object_points_base.copy()
    result = qualify_visual_scene(scene)
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == 2
    assert result.model.qualification['contour_source'] == 'rgb_outer_joint_interval_on_measured_top_plane'
    assert min(result.model.qualification['corner_coverage']) < .5 or min(result.model.qualification['edge_coverage']) < .7
    np.testing.assert_array_equal(scene.geometry.object_points_base, original)
    np.testing.assert_allclose(sorted(result.model.size_xyz_m[:2]), [.035, .065], atol=.0015)


@pytest.mark.parametrize('fault', ['missing_rgb_edge', 'large_chamfer', 'crop', 'internal_hole', 'external_top'])
def test_incomplete_or_contradictory_rgb_cannot_supply_box_qualification(fault):
    scene = visual_edge_gap_scene()
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    if fault == 'missing_rgb_edge':
        scene.color[y > .01] = 20
    elif fault == 'large_chamfer':
        cut = (x > .018) & (y > .003) & (x+y > .033)
        scene.color[cut] = 20
        scene.mask[cut] = 0
    elif fault == 'crop':
        scene.mask[0, 120:140] = 255
    elif fault == 'internal_hole':
        scene.depth[(abs(x)<.009)&(abs(y)<.008)] = 0
    else:
        # Extend the measured top beyond the outer color boundary while the
        # original object mask remains narrow: this is an internal print edge.
        same = (abs(x)<.04)&(abs(y)<.026)
        scene.depth[same] = 3250
    result = qualify_visual_scene(scene)
    assert not result.ok, (result.code, result.audit)


def test_unchanged_complete_depth_path_keeps_schema_v1():
    result = qualify(ray_scene())
    assert result.ok
    assert result.model.schema_version == 1
