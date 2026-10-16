"""No ROS master: the helper has one bounded API and no implicit retries."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'tools/run_fixed_goal_endpoint_experiment.py'
spec = importlib.util.spec_from_file_location('fixed_goal_runner_tested', SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize('execute,previous,error', [
    (False, None, False), (True, None, False), (True, False, False),
    (True, True, False), (True, None, True), (True, False, True),
])
def test_single_explicit_call_restores_only_diagnostic_opt_in(
        monkeypatch, tmp_path, execute, previous, error):
    key = '/motion_gateway/enable_endpoint_correction_experiments'
    parameters = {} if previous is None else {key: previous}
    original = dict(parameters)
    output = tmp_path / 'report.json'
    calls, mutations = [], []
    monkeypatch.setattr(sys, 'argv', ['runner', '--output', str(output)]
                        + (['--execute'] if execute else []))
    monkeypatch.setattr(runner.rospy, 'init_node', lambda *a, **k: None)
    monkeypatch.setattr(runner.rospy, 'get_time', lambda: 10.)
    monkeypatch.setattr(runner.rospy, 'has_param', lambda k: k in parameters)
    monkeypatch.setattr(runner.rospy, 'get_param', lambda k, d=None: parameters.get(k, d))
    def set_param(k, value):
        mutations.append(k)
        parameters[k] = value
    monkeypatch.setattr(runner.rospy, 'set_param', set_param)
    monkeypatch.setattr(runner.rospy, 'delete_param', lambda k: parameters.pop(k))
    monkeypatch.setattr(runner.rospy, 'wait_for_service', lambda *a, **k: None)
    def proxy(name, service):
        assert name == '/supervisor/refine_endpoint_feedback'
        def request(trigger):
            calls.append(trigger)
            if execute:
                assert parameters[key] is True
            if error:
                raise RuntimeError('transport failure')
            return SimpleNamespace(success=False, message='diagnostic rejected')
        return request
    monkeypatch.setattr(runner.rospy, 'ServiceProxy', proxy)
    if error:
        with pytest.raises(RuntimeError, match='transport failure'):
            runner.main()
    else:
        runner.main()
    assert calls == [execute]
    assert parameters == original
    assert mutations == ([key, key] if execute and previous is not None else
                         [key] if execute else [])
    report = json.loads(output.read_text())
    assert report['grasp_success'] is False
    assert report['execution_requested'] is execute
    with pytest.raises(FileExistsError):
        runner.main()
    assert calls == [execute]  # existing output never causes a repeated action
