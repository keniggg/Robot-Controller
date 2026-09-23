"""Exact selection contract shared by mode-aware planner and executor."""

import json


SELECTION_PARAM = '/grasp_mode/selection'


def parse_selection(value):
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict):
        raise ValueError('mode selection must be an object')
    mode = value.get('mode')
    strategy = value.get('strategy', 'two_stage')
    generation = value.get('generation')
    stamp_ns = value.get('stamp_ns')
    # XML-RPC integers are int32; exact nanoseconds travel as decimal text.
    if isinstance(stamp_ns, str) and stamp_ns.isdecimal():
        stamp_ns = int(stamp_ns)
    if mode not in ('carton', 'unknown'):
        raise ValueError('mode must be carton or unknown')
    if strategy not in ('two_stage', 'direct'):
        raise ValueError('strategy must be two_stage or direct')
    if type(generation) is not int or generation < 1:
        raise ValueError('generation must be a positive integer')
    if type(stamp_ns) is not int or stamp_ns < 1:
        raise ValueError('stamp_ns must be a positive integer or decimal string')
    return {'mode': mode, 'strategy': strategy,
            'generation': generation, 'stamp_ns': stamp_ns}
