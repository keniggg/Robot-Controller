"""Exact mode/strategy/generation identity for the observation-return option."""
from copy import deepcopy
import json

import pytest

from alicia_grasp_modes.selection import parse_selection


@pytest.mark.parametrize('mode,strategy', [
    ('unknown', 'observe_return'), ('unknown', 'direct'), ('unknown', 'two_stage'),
    ('carton', 'direct'), ('carton', 'two_stage'),
])
def test_supported_strategy_preserves_exact_selection(mode, strategy):
    value = dict(mode=mode, strategy=strategy, generation=4, stamp_ns='1790040471603499175')
    before = deepcopy(value)
    expected = dict(value, stamp_ns=1790040471603499175)
    assert parse_selection(value) == expected
    assert parse_selection(json.dumps(value)) == expected
    assert value == before


@pytest.mark.parametrize('changes', [dict(mode='carton'), dict(generation=True),
    dict(generation=0), dict(stamp_ns='-1'), dict(stamp_ns=0), dict(strategy='observe_return_direct')])
def test_new_strategy_does_not_accept_invalid_identity_or_alias(changes):
    value = dict(mode='unknown', strategy='observe_return', generation=1, stamp_ns='1')
    value.update(changes)
    with pytest.raises(ValueError):
        parse_selection(value)
