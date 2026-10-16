"""Exact selection contract shared by mode-aware planner and executor."""

import json


SELECTION_PARAM = '/grasp_mode/selection'
STRATEGIES = ('two_stage', 'direct', 'observe_return')


def read_authoritative_selection(master):
    """Read the master directly at a reset barrier, bypassing rospy's cache.

    Once get_param_cached has subscribed, ordinary get_param also reads that
    process-local cache. Its asynchronous update is not a reset acknowledgement.
    MasterProxy.getParam supplies caller_id itself; it takes only the key here.
    """
    code, message, value = master.getParam(SELECTION_PARAM)
    if type(code) is not int or code != 1:
        raise ValueError('authoritative mode selection unavailable: %s' % message)
    return parse_selection(value)


def validate_mode_strategy(mode, strategy):
    if mode not in ('carton', 'unknown'):
        raise ValueError('mode must be carton or unknown')
    if strategy not in STRATEGIES:
        raise ValueError('strategy must be two_stage, direct or observe_return')
    if strategy == 'observe_return' and mode != 'unknown':
        raise ValueError('observe_return strategy requires unknown mode')


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
    validate_mode_strategy(mode, strategy)
    if type(generation) is not int or generation < 1:
        raise ValueError('generation must be a positive integer')
    if type(stamp_ns) is not int or stamp_ns < 1:
        raise ValueError('stamp_ns must be a positive integer or decimal string')
    return {'mode': mode, 'strategy': strategy,
            'generation': generation, 'stamp_ns': stamp_ns}
