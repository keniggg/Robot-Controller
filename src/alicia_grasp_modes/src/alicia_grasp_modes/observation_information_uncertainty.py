"""Separate visual resolution from contact-stage execution allowances.

Adaptive contact profiles contain perception + measured endpoint residual.
The latter remains in all contact distances and physical motion contracts;
it is not additional depth noise after the actual camera pose is measured.
This helper supplies observation information predictions and new measured
coverage after successful registration. It never changes registration gates.
"""
from collections.abc import Mapping
import math
from numbers import Real


def observation_information_uncertainty_m(profiles):
    """Return the largest explicit perception term without modifying profiles.

    Missing execution fields must fail, not silently treat a combined contact
    allowance as optical resolution. Both quantities come from the same
    immutable current-source profile; no parameter or old task sample is read.
    """
    values = []
    for profile in profiles:
        def field(name):
            value = profile[name] if isinstance(profile, Mapping) else getattr(profile, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
                raise ValueError('observation information uncertainty requires finite explicit profile fields')
            return float(value)
        try:
            combined = field('depth_uncertainty_m')
            execution = field('execution_position_error_m')
        except (AttributeError, KeyError) as exc:
            raise ValueError('observation information uncertainty profile split is missing') from exc
        perception = combined-execution
        if execution < 0. or perception <= 0.:
            raise ValueError('observation information uncertainty split is invalid')
        values.append(perception)
    if not values:
        raise ValueError('observation information uncertainty requires current-source profiles')
    return max(values)
