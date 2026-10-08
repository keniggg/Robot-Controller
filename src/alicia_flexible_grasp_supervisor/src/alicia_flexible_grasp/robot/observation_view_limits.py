"""Shared projection limits for unknown-object auxiliary camera observations.

Configuration may tighten the original image/depth bounds, never relax them.
This parser does not grant motion authority or replace input-snapshot binding.
"""
from collections.abc import Mapping
import math


DEFAULT_OBSERVATION_VIEW_PARAMETERS = {
    'camera_visibility_margin_px': 36.,
    'camera_visibility_min_depth_m': .070,
    'camera_visibility_max_depth_m': .500,
}


def effective_observation_view_limits(configuration):
    if not isinstance(configuration, Mapping):
        raise ValueError('observation visibility configuration must be a mapping')
    values = {}
    for key, baseline in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items():
        value = configuration.get(key, baseline)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError('observation visibility limit must be finite numeric: ' + key)
        values[key] = float(value)
    margin = values['camera_visibility_margin_px']
    near = values['camera_visibility_min_depth_m']
    far = values['camera_visibility_max_depth_m']
    if margin < 36. or near < .070 or far > .500:
        raise ValueError('observation visibility limits cannot relax 36 px / 0.070 m / 0.500 m bounds')
    if not 0. < near < far:
        raise ValueError('observation visibility minimum depth must be below maximum depth')
    return dict(roi_margin_px=margin, minimum_depth_m=near, maximum_depth_m=far)
