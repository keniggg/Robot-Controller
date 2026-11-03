"""Shared projection limits for unknown-object auxiliary camera observations.

Configuration may tighten the original image/depth bounds, never relax them.
This parser does not grant motion authority or replace input-snapshot binding.
"""
from collections.abc import Mapping
import math


MINIMUM_OBSERVATION_IMAGE_MARGIN_PX = 24.


DEFAULT_OBSERVATION_VIEW_PARAMETERS = {
    'observation_camera_visibility_margin_px': MINIMUM_OBSERVATION_IMAGE_MARGIN_PX,
    'camera_visibility_min_depth_m': .070,
    'camera_visibility_max_depth_m': .500,
}


def read_observation_view_parameters(get_param):
    """Read the observation-only margin; generic contact camera limits are separate.

    Return canonical requested fields so frozen audits detect runtime changes.
    The margin deliberately never falls back to the generic contact parameter.
    """
    return {key: get_param('/grasp_6d/remote/' + key, default)
            for key, default in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()}


def effective_observation_view_limits(configuration):
    if not isinstance(configuration, Mapping):
        raise ValueError('observation visibility configuration must be a mapping')
    values = {}
    for key, baseline in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items():
        # Preserve the old pure geometry API's explicit margin spelling. Runtime
        # readers above always provide the observation-specific canonical key.
        fallback = (configuration.get('camera_visibility_margin_px', baseline)
                    if key == 'observation_camera_visibility_margin_px' else baseline)
        value = configuration.get(key, fallback)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            raise ValueError('observation visibility limit must be finite numeric: ' + key)
        values[key] = float(value)
    margin = values['observation_camera_visibility_margin_px']
    near = values['camera_visibility_min_depth_m']
    far = values['camera_visibility_max_depth_m']
    if margin < MINIMUM_OBSERVATION_IMAGE_MARGIN_PX or near < .070 or far > .500:
        raise ValueError('observation visibility limits cannot relax 24 px / 0.070 m / 0.500 m bounds')
    if not 0. < near < far:
        raise ValueError('observation visibility minimum depth must be below maximum depth')
    return dict(roi_margin_px=margin, minimum_depth_m=near, maximum_depth_m=far)
