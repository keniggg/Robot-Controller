"""The confirmed dispatcher cannot accidentally run a newly selected generation."""
import json

import pytest

from alicia_grasp_modes.selection import parse_selection
from test_mode_runner import fake_run


def test_expected_selection_accepts_exact_xmlrpc_decimal_stamp(monkeypatch):
    runner, params, calls = fake_run(monkeypatch, lambda *_: type('Result', (),
        {'success':True, 'message':'done'})())
    selected = dict(params['/grasp_mode/selection'])
    selected['stamp_ns'] = str(selected['stamp_ns'])
    runner.expected_selection = parse_selection(json.dumps(selected))
    assert runner.run() == 0
    assert any(call[0] == 'start' for call in calls)


@pytest.mark.parametrize('field,value', [('generation',3), ('stamp_ns',19000000001),
                                        ('strategy','observe_return')])
def test_expected_selection_mismatch_never_requests_plan_or_execution(monkeypatch, field, value):
    runner, params, calls = fake_run(monkeypatch, lambda *_: pytest.fail('unexpected execution'))
    runner.expected_selection = dict(params['/grasp_mode/selection'], **{field:value})
    with pytest.raises(ValueError, match='confirmed attempt'):
        runner.run()
    assert calls == []
