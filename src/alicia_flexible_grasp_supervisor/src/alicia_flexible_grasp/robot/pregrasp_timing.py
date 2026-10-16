"""Unknown pregrasp speed preference; no trajectory or motion authority."""
from contextlib import contextmanager
import math


PREGRASP_LIMITED_JOINTS = ('Joint3', 'Joint4', 'Joint6')


def unknown_pregrasp_velocity_limits(selection, config, global_limit):
    if not isinstance(selection, dict) or selection.get('mode') != 'unknown':
        return None
    if not isinstance(config, dict):
        raise ValueError('invalid unknown pregrasp timing configuration')
    enabled = config.get('enabled', False)
    if type(enabled) is not bool:
        raise ValueError('unknown pregrasp timing enabled must be boolean')
    if not enabled:
        return None
    value = config.get('joint_velocity_rad_s')
    for number in (value, global_limit):
        if (isinstance(number, bool) or not isinstance(number, (int, float))
                or not math.isfinite(number) or number <= 0.0):
            raise ValueError('invalid unknown pregrasp joint velocity')
    if value > min(global_limit, .08):
        raise ValueError('pregrasp velocity exceeds the existing global ceiling')
    return {name: float(value) for name in PREGRASP_LIMITED_JOINTS}


@contextmanager
def temporary_attribute(instance, name, value):
    existed, previous = hasattr(instance, name), getattr(instance, name, None)
    setattr(instance, name, value)
    try:
        yield
    finally:
        if existed:
            setattr(instance, name, previous)
        else:
            delattr(instance, name)


@contextmanager
def pregrasp_velocity_scope(planner, limits):
    if limits is None:
        yield
        return
    name = 'strict_execution_joint_velocity_limits_rad_s'
    values = dict(getattr(planner, name, {}) or {})
    values.update(limits)
    with temporary_attribute(planner, name, values):
        yield
