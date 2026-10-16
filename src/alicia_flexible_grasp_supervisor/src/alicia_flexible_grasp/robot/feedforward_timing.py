"""Pure timing cap for existing unknown-object pose-0/1 feedforward paths.

No path, response, tracking, stopping or execution authority is supplied here.
The planner still proves its retimed complete curve against every joint limit.
"""
import math
from numbers import Real

from .stationary_following import ARM_NAMES


LEGACY_FEEDFORWARD_VELOCITY_RAD_S = .002
MAX_FEEDFORWARD_VELOCITY_RAD_S = .020
DEFAULT_ENABLED_FEEDFORWARD_VELOCITY_RAD_S = .008


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_INVALID: '+label)
    value = float(value)
    if not math.isfinite(value) or value <= 0.:
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_INVALID: '+label)
    return value


def resolve_unknown_feedforward_timing(config, model_choice, pose_index,
                                      planner_max_velocity_rad_s,
                                      joint_velocity_limits_rad_s=None):
    """Return the temporary global cap and explicit existing per-axis caps.

    A missing/disabled switch retains 0.002 rad/s. Invalid configuration never
    silently falls back. Per-axis planner settings are read without mutation;
    the existing timing/continuous derivative proof enforces their minima.
    """
    if model_choice != 'unknown_tabletop' or type(pose_index) is not int or pose_index not in (0, 1):
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_SCOPE_INVALID')
    if not isinstance(config, dict):
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_INVALID: configuration')
    enabled = config.get('unknown_feedforward_timing_enabled', False)
    if type(enabled) is not bool:
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_INVALID: enabled must be boolean')
    configured = _positive(config.get('unknown_feedforward_max_joint_velocity_rad_s',
                                     DEFAULT_ENABLED_FEEDFORWARD_VELOCITY_RAD_S), 'configured velocity')
    if configured > MAX_FEEDFORWARD_VELOCITY_RAD_S:
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_INVALID: configured velocity exceeds 0.020 rad/s')
    planner_limit = _positive(planner_max_velocity_rad_s, 'planner velocity')
    limits = {} if joint_velocity_limits_rad_s is None else joint_velocity_limits_rad_s
    if not isinstance(limits, dict):
        raise ValueError('UNKNOWN_FEEDFORWARD_TIMING_INVALID: per-joint limits')
    requested = configured if enabled else LEGACY_FEEDFORWARD_VELOCITY_RAD_S
    cap = min(requested, planner_limit)
    effective = {name:min(cap, _positive(limits.get(name, planner_limit), name+' velocity'))
                 for name in ARM_NAMES}
    return cap, dict(policy='existing_strict_collinear_curve_with_configured_feedforward_cap',
        enabled=enabled, pose_index=pose_index, configured_velocity_rad_s=configured,
        requested_velocity_rad_s=requested, previous_planner_max_velocity_rad_s=planner_limit,
        effective_global_velocity_rad_s=cap, effective_joint_velocity_limits_rad_s=effective,
        path_changed=False, response_and_settle_gates_changed=False,
        continuous_derivative_and_stop_proof_required=True)
