"""A qualified v1 box must not pay its projected width uncertainty twice.

The original pixel qualifier supplies every model. These tests cover the
previously rejected extra-clearance interval, retaining the full U already
charged to opening; they do not assert an executable path or grip.
"""
import numpy as np
import pytest

from test_gen26_regular_box_inference_independent import ray_scene, qualify
from test_gen26_contact_provenance_independent import physical_gripper
from alicia_flexible_grasp.grasp.gripper_geometry import evaluate_inferred_box_contact
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (
    TabletopGeometryConfig, generate_tabletop_proposals)
from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry


def contact(model, gap=.05, fit=.0005, center=None):
    return evaluate_inferred_box_contact(model,
        model.center_base if center is None else center,
        model.axes_base[:, 1], -model.support_normal_base,
        physical_gripper(gap), opening_fit_clearance_each_side_m=fit)


def candidates(scene, model, gap, fit):
    geometry, _ = enclose_inferred_box_geometry(scene.geometry, model)
    return generate_tabletop_proposals(geometry.object_points_base,
        geometry.center_base, geometry.axes_base, geometry.size_xyz_m,
        np.zeros(3), model.support_normal_base,
        TabletopGeometryConfig(max_inner_gap_m=gap,
            opening_fit_clearance_each_side_m=fit),
        finger_geometry=physical_gripper(), inferred_box=model)


@pytest.mark.parametrize('fit', [.0005, .002])
def test_v1_candidates_keep_full_uncertainty_without_requiring_a_second_copy(fit):
    scene = ray_scene(width=.030)
    result = qualify(scene)
    assert result.ok, result.audit
    model = result.model
    assert model.schema_version == 1
    full = contact(model, fit=fit)
    assert full.ok
    reserve = physical_gripper().jaw_clearance_each_side_m
    assert full.width_uncertainty_m > reserve
    remaining = (reserve + full.width_uncertainty_m) / 2.
    required = full.inferred_width_m + full.width_uncertainty_m + 2.*fit
    gap = required + remaining
    assert gap < .05
    assert reserve < remaining < full.width_uncertainty_m
    projected = contact(model, gap=gap, fit=fit)
    generated = candidates(scene, model, gap, fit)
    assert projected.ok, projected
    assert generated.ok, generated
    assert generated.proposals[0].required_open_width_m == pytest.approx(required)
    assert projected.targeted_view['required_open_upper_m'] == pytest.approx(required)
    assert projected.width_uncertainty_m == pytest.approx(full.width_uncertainty_m)


@pytest.mark.parametrize('remaining, accepted', [
    (.00040664592985334425, True), (.0019, True), (.0021, True)])
def test_v1_reports_extra_clearance_without_rejecting_physical_fit(remaining, accepted):
    scene = ray_scene(width=.030)
    model = qualify(scene).model
    full = contact(model)
    gap = full.targeted_view['required_open_upper_m'] + remaining
    projected = contact(model, gap=gap)
    generated = candidates(scene, model, gap, .0005)
    assert projected.ok is accepted
    assert generated.ok is accepted
    assert projected.targeted_view['precision_reserve_m'] == pytest.approx(.002)
    assert projected.targeted_view['precision_reserve_is_hard_requirement'] is False
    assert projected.targeted_view['precision_preference_met'] is (remaining >= .002)
    if not accepted:
        assert projected.code == generated.failure_code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'


def test_v1_commanded_center_offset_still_consumes_two_sides_of_opening():
    scene = ray_scene(width=.030)
    model = qualify(scene).model
    full = contact(model)
    gap = full.targeted_view['required_open_upper_m'] + .0003
    centered = contact(model, gap=gap)
    displaced = contact(model, gap=gap,
        center=model.center_base + model.axes_base[:, 1]*.0002)
    assert centered.ok
    assert not displaced.ok
    assert displaced.code == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    assert displaced.width_uncertainty_m-centered.width_uncertainty_m == pytest.approx(.0004)
    assert displaced.targeted_view['commanded_center_jaw_offset_m'] == pytest.approx(.0002)
