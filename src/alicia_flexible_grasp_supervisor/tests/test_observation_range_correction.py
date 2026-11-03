"""A range preference never substitutes for the post-move acceptance gate."""
import math

import pytest

from alicia_flexible_grasp.grasp.observation_range_correction import (
    select_observation_camera_correction_distance as select,
)


def inputs(**updates):
    values = dict(selected_distance_m=.22, minimum_distance_m=.22,
        maximum_distance_m=.30, position_error_vector_xyz=[.003, .004, 0.],
        orientation_error_rad=.04, handeye_translation_xyz=[.03, .04, .12],
        depth_mad_m=.001)
    values.update(updates)
    return values


@pytest.mark.parametrize('selected', [.22, .26, .30])
def test_selected_preference_projected_into_interior_without_changing_band(selected):
    source = inputs(selected_distance_m=selected)
    target, audit = select(**source)
    margin = .005 + 2. * .13 * math.sin(.04 / 2.) + .001
    assert target == pytest.approx(min(.30 - margin, max(.22 + margin, selected)))
    assert audit['range_margin_m'] == pytest.approx(margin)
    assert source['selected_distance_m'] == selected
    assert source['minimum_distance_m'] == .22 and source['maximum_distance_m'] == .30
    assert audit['range_margin_is_future_error_bound'] is False
    assert audit['post_move_measurement_required'] is True
    assert audit['range_margin_policy'].endswith('_heuristic')


@pytest.mark.parametrize('selected', [.22, .26, .30])
def test_complete_zero_error_evidence_preserves_preference(selected):
    target, audit = select(**inputs(selected_distance_m=selected,
        position_error_vector_xyz=[0., 0., 0.], orientation_error_rad=0., depth_mad_m=0.))
    assert target == selected and audit['range_margin_m'] == 0.
    assert audit['post_move_measurement_required'] is True


@pytest.mark.parametrize('angle,offset,expected', [
    (math.pi, [0., 0., 0.], 0.),
    (0., [.5, .5, .5], 0.),
    (math.pi, [.005, 0., 0.], .01),
])
def test_rotation_lever_term_endpoints(angle, offset, expected):
    _, audit = select(**inputs(orientation_error_rad=angle, handeye_translation_xyz=offset))
    assert audit['range_margin_rotation_lever_m'] == pytest.approx(expected)


@pytest.mark.parametrize('field,bad', [
    ('selected_distance_m', .219999), ('selected_distance_m', .300001),
    ('selected_distance_m', float('nan')), ('selected_distance_m', True),
    ('minimum_distance_m', 0.), ('minimum_distance_m', .31),
    ('maximum_distance_m', .21), ('maximum_distance_m', float('inf')),
    ('position_error_vector_xyz', [0., 0.]),
    ('position_error_vector_xyz', [0., 0., 0., 0.]),
    ('position_error_vector_xyz', [[0.], [0.], [0.]]),
    ('position_error_vector_xyz', [float('nan'), 0., 0.]),
    ('position_error_vector_xyz', [True, 0., 0.]),
    ('position_error_vector_xyz', None),
    ('handeye_translation_xyz', [0., 0.]),
    ('handeye_translation_xyz', [0., float('inf'), 0.]),
    ('handeye_translation_xyz', ['0', 0., 0.]),
    ('orientation_error_rad', -.001), ('orientation_error_rad', math.pi + .001),
    ('orientation_error_rad', float('nan')), ('orientation_error_rad', True),
    ('depth_mad_m', -.001), ('depth_mad_m', float('inf')),
    ('depth_mad_m', None), ('depth_mad_m', False),
])
def test_missing_or_invalid_evidence_never_becomes_zero_margin(field, bad):
    with pytest.raises(ValueError):
        select(**inputs(**{field: bad}))


@pytest.mark.parametrize('error', [.04, .040001, .10])
def test_exhausted_margin_refuses_target(error):
    with pytest.raises(ValueError, match='no positive-width'):
        select(**inputs(minimum_distance_m=.20, maximum_distance_m=.28,
            position_error_vector_xyz=[error, 0., 0.], orientation_error_rad=0., depth_mad_m=0.))


def test_norm_overflow_is_not_silently_projected():
    with pytest.raises(ValueError, match='finite'):
        select(**inputs(position_error_vector_xyz=[1.7e308, 1.7e308, 1.7e308]))
