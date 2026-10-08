"""One fixed-position close target for simulation and the real task."""
from copy import deepcopy
import math

from alicia_flexible_grasp.grasp.rich_plan_integrity import float32_wire_value


def _number(config, key, default, code):
    value = config.get(key, default)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(code + ': ' + key + ' must be finite')
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(code + ': ' + key + ' must be finite')
    return result


def fixed_close_contract(plan, config):
    if bool(config.get('use_compliant_close', True)):
        return None  # Adaptive force control retains its separate contract.
    code = 'PLAN_GRIPPER_WIDTH_INVALID'
    close_limit = _number(config, 'close_limit_m', 0., code)
    opened = _number(config, 'open_position_m', .05, code)
    target = _number(config, 'simple_close_position_m',
                     config.get('close_limit_m', .05), code)
    if plan is not None and bool(config.get('use_plan_bound_close_position', True)):
        width = _number({'width': getattr(plan, 'required_open_width_m', None)},
                        'width', None, code)
        clearance = _number(config, 'plan_bound_opening_clearance_each_side_m', .002,
                            'PLAN_GRIPPER_CLEARANCE_INVALID')
        if clearance < 0.:
            raise ValueError('PLAN_GRIPPER_CLEARANCE_INVALID: negative opening clearance')
        preload = max(0., _number(config, 'plan_bound_close_preload_m', .002, code))
        lower, upper = min(close_limit, opened), max(close_limit, opened)
        # The preview holds a Python float; the task receives this plan field
        # as ROS float32. Derive both close targets from that same wire value.
        # Compare limits in its representation, then clamp any upper-end
        # rounding (e.g. the float32 representation of 50 mm) to the physical
        # configured limit. This does not change the configured opening.
        try:
            width = float32_wire_value(width)
            lower_wire = float32_wire_value(lower)
            upper_wire = float32_wire_value(upper)
        except (OverflowError, ValueError) as exc:
            raise ValueError(code + ': required opening outside gripper range') from exc
        if not lower_wire < width <= upper_wire:
            raise ValueError(code + ': required opening outside gripper range')
        width = min(width, upper)
        target = max(lower, min(upper, width - 2.*clearance - preload))
    wait = max(0., _number(config, 'simple_close_wait_sec', .8, code))
    if not 0. <= target <= .05:
        raise ValueError(code + ': close target outside 50 mm range')
    return dict(close_target_inner_gap_m=target, close_settle_sec=wait)


def bind_simulated_fixed_close(plan, simulation_config, gripper_config):
    contract = fixed_close_contract(plan, gripper_config)
    if contract is None:
        return simulation_config
    result = deepcopy(simulation_config)
    model = dict(result.get('gripper_model', {}) or {})
    model.update(contract)
    result['gripper_model'] = model
    return result
