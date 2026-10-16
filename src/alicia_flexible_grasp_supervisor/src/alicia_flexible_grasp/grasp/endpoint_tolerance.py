"""Measured arrival policy shared by the task and its correction services."""
import math


def measured_position_tolerance(config, model_choice):
    value = config.get('measured_endpoint_position_tolerance_m', .006)
    if model_choice == 'unknown_tabletop':
        value = config.get('unknown_measured_endpoint_position_tolerance_m', value)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('MEASURED_ENDPOINT_POSITION_TOLERANCE_INVALID')
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= .010:
        raise ValueError('MEASURED_ENDPOINT_POSITION_TOLERANCE_INVALID')
    return value
