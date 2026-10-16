"""Canonical image attention from projected frozen reference vertices.

The resulting policy only bounds a search for RGB evidence. It establishes no
object identity, measured geometry, free space, or movement authority. Both
producer and independent gateway use the same arithmetic before exact digest
binding; algebraically equivalent floating-point formulas are not substituted.
"""
from dataclasses import replace

import numpy as np

from .rgb_observation_region import CenterSearchPolicy


def projected_rgb_search_policy(policy, projected_pixels, image_shape):
    """Keep the original search constraints and canonical projection arithmetic."""
    if (type(policy) is not CenterSearchPolicy
            or not isinstance(image_shape, (tuple, list)) or len(image_shape) != 2
            or any(type(size) is not int or size <= 0 for size in image_shape)):
        raise ValueError('invalid_projected_rgb_search_policy_input')
    projected = np.asarray(projected_pixels, dtype=float)
    if projected.shape != (8, 2) or not np.all(np.isfinite(projected)):
        raise ValueError('invalid_projected_rgb_search_vertices')
    middle = (projected.min(axis=0)+projected.max(axis=0))/2.
    radius = float(np.max(np.linalg.norm(projected-middle, axis=1)))/min(image_shape)
    return replace(policy,
        center_xy=(float(middle[0]/image_shape[1]), float(middle[1]/image_shape[0])),
        radius_fraction=min(1., max(radius, .01)))
