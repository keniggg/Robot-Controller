"""Choose an interior observation correction target without relaxing acceptance.

The margin summarizes the current endpoint residual and frozen depth dispersion.
It is a heuristic for choosing one command, not a bound on future tracking error.
Only a fresh post-move measurement can establish that the reached range is valid.
"""

import math
from numbers import Real


def _finite_scalar(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('%s must be a finite number' % label)
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('%s must be a finite number' % label)
    return result


def _finite_xyz(values, label):
    try:
        values = tuple(values)
    except TypeError as exc:
        raise ValueError('%s must contain three finite numbers' % label) from exc
    if len(values) != 3:
        raise ValueError('%s must contain three finite numbers' % label)
    return tuple(_finite_scalar(value, label) for value in values)


def select_observation_camera_correction_distance(
    selected_distance_m,
    minimum_distance_m,
    maximum_distance_m,
    position_error_vector_xyz,
    orientation_error_rad,
    handeye_translation_xyz,
    depth_mad_m,
):
    """Project the selected preference into a measured-margin interior band.

    The immutable selected distance is relative to the frozen OBB centre. The
    returned command preference is relative to the live point used by the range
    gate; it does not assert that the two points coincide. The original gate,
    one-correction budget and required post-move measurement remain unchanged.
    """
    selected = _finite_scalar(selected_distance_m, 'selected distance')
    minimum = _finite_scalar(minimum_distance_m, 'minimum distance')
    maximum = _finite_scalar(maximum_distance_m, 'maximum distance')
    position_error = _finite_xyz(position_error_vector_xyz, 'endpoint position error')
    offset = _finite_xyz(handeye_translation_xyz, 'handeye translation')
    angle = _finite_scalar(orientation_error_rad, 'endpoint orientation error')
    depth_mad = _finite_scalar(depth_mad_m, 'frozen depth MAD')
    if minimum <= 0. or not minimum <= selected <= maximum:
        raise ValueError('selected distance must be inside the ordered positive range')
    if not 0. <= angle <= math.pi:
        raise ValueError('endpoint orientation error must be in [0, pi]')
    if depth_mad < 0.:
        raise ValueError('frozen depth MAD must be nonnegative')
    position_margin = math.hypot(*position_error)
    lever_length = math.hypot(*offset)
    rotation_margin = 2. * lever_length * math.sin(angle / 2.)
    margin = position_margin + rotation_margin + depth_mad
    if not all(math.isfinite(value) for value in (
            position_margin, lever_length, rotation_margin, margin)):
        raise ValueError('observation correction margin must be finite')
    lower = minimum + margin
    upper = maximum - margin
    if not lower < upper:
        raise ValueError('observation correction margin leaves no positive-width interior range')
    target = min(upper, max(lower, selected))
    return target, {
        'correction_target_camera_distance_m': target,
        'range_margin_policy': 'current_endpoint_and_frozen_depth_dispersion_heuristic',
        'range_margin_is_future_error_bound': False,
        'post_move_measurement_required': True,
        'range_margin_m': margin,
        'range_margin_position_residual_m': position_margin,
        'range_margin_rotation_lever_m': rotation_margin,
        'range_margin_frozen_depth_mad_m': depth_mad,
        'range_margin_handeye_lever_length_m': lever_length,
        'range_margin_orientation_error_rad': angle,
        'correction_target_min_distance_m': lower,
        'correction_target_max_distance_m': upper,
        'correction_target_policy': 'project_frozen_selected_preference_into_margin_interior',
    }
