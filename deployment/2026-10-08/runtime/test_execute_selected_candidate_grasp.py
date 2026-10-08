"""Offline contracts: no ROS imports, nodes, parameters, services or motion."""
import ast
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

PATH = Path(__file__).with_name('execute_selected_candidate_grasp.py')
spec = importlib.util.spec_from_file_location('candidate_handoff_test', PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
SELECTION = dict(mode='unknown', strategy='observe_return', generation=6, stamp_ns='1790661700098336935')


def admission(**changes):
    data = dict(task=NS(active=False, state='IDLE', stage=0),
                router=dict(SELECTION, stamp_ns=int(SELECTION['stamp_ns']), active=False,
                            start_reserved=False, pending='', pending_strategy='', state='ready'),
                manual=False, actions=[([], 1_000_000), ([3], 1_000_000)],
                selection=SELECTION.copy(), expected=SELECTION)
    data.update(changes)
    return data


@pytest.mark.parametrize('state,stage', list(m.INACTIVE_STAGES.items()))
def test_real_inactive_stages_and_string_ns_parameter(state, stage):
    m.validate_admission(**admission(task=NS(active=False, state=state, stage=stage)))


@pytest.mark.parametrize('task', [NS(active=True, state='IDLE', stage=0), NS(active=False, state='LIFT_OBJECT', stage=5),
                                  NS(active=False, state='EMERGENCY_STOP', stage=8), NS(active=False, state='IDLE', stage=6)])
def test_active_nonterminal_and_inconsistent_task_rejected(task):
    with pytest.raises(RuntimeError):
        m.validate_admission(**admission(task=task))


@pytest.mark.parametrize('manual', [True, 0, None, 'false'])
def test_manual_ownership_is_not_overridden(manual):
    with pytest.raises(RuntimeError, match='manual'):
        m.validate_admission(**admission(manual=manual))


@pytest.mark.parametrize('delta', [dict(active=True), dict(start_reserved=True), dict(pending='unknown'),
                                   dict(pending_strategy='observe_return'), dict(state='error'), dict(generation=5)])
def test_router_conflict_rejected(delta):
    data = admission()
    data['router'].update(delta)
    with pytest.raises(RuntimeError):
        m.validate_admission(**data)


@pytest.mark.parametrize('status', [0, 1, 6, 7, 10, True])
def test_either_controller_pending_active_or_invalid_rejected(status):
    with pytest.raises(RuntimeError):
        m.validate_admission(**admission(actions=[([], 0), ([status], 0)]))


@pytest.mark.parametrize('age', [-1, 2_000_000_001])
def test_controller_source_must_be_current(age):
    with pytest.raises(RuntimeError):
        m.validate_admission(**admission(actions=[([], age), ([], 0)]))


def run_fixture(response=None, failure_at=None, second_context=None):
    events, records = [], []
    context = dict(selection=SELECTION, model_sha256=m.MODEL_SHA, handeye_sha256=m.HANDEYE_SHA)
    reads = 0
    def read():
        nonlocal reads
        reads += 1
        events.append(('guard', reads))
        if failure_at == reads:
            raise RuntimeError('manual/task changed')
        return context if reads == 1 or second_context is None else second_context
    def request(trigger):
        events.append(('request', trigger))
        if isinstance(response, Exception):
            raise response
        return response or NS(success=True, message='continuous remote 6D inference stopped')
    def launch(argv):
        events.append(('launch', argv))
        return 17
    def save(record):
        records.append(copy.deepcopy(record))
    return lambda: m.fresh_phase_handoff(read, request, launch, save, now_ns=lambda: 123), events, records


@pytest.mark.parametrize('message', sorted(m.STOPPED_MESSAGES))
def test_retire_once_then_child_owns_true_after_fresh_window(message):
    run, events, records = run_fixture(NS(success=True, message=message))
    assert run() == 17
    assert events == [('guard', 1), ('request', False), ('guard', 2), ('launch', list(m.RUNNER))]
    assert records[-1]['start_delegated_to_runner_after_minimum_stamp'] is True
    assert records[-1]['hardware_stop_or_disable_command_sent'] is False
    assert records[-1]['old_plan_reused'] is False
    assert records[-1]['runner_exit_code'] == 17


@pytest.mark.parametrize('response', [NS(success=False, message='no'), NS(success=True, message='continuous remote 6D inference already running'), RuntimeError('transport ambiguous')])
def test_no_child_no_restart_after_unacknowledged_stop(response):
    run, events, records = run_fixture(response)
    with pytest.raises(RuntimeError):
        run()
    assert events == [('guard', 1), ('request', False)]
    assert records[-1]['status'] == 'failed_no_further_action'


@pytest.mark.parametrize('failure_at', [1, 2])
def test_conflict_before_stop_or_after_stop_has_no_child(failure_at):
    run, events, records = run_fixture(failure_at=failure_at)
    with pytest.raises(RuntimeError):
        run()
    assert not any(e[0] == 'launch' for e in events)
    assert [e for e in events if e[0] == 'request'] == ([] if failure_at == 1 else [('request', False)])


def test_changed_bound_context_after_stop_has_no_child():
    run, events, records = run_fixture(second_context=dict(selection=dict(SELECTION, generation=7)))
    with pytest.raises(RuntimeError, match='context changed'):
        run()
    assert not any(e[0] == 'launch' for e in events)


def test_import_and_wrapper_have_no_ros_writes_or_publisher():
    tree = ast.parse(PATH.read_text())
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [x.name for x in node.names]
            assert 'rospy' not in names and getattr(node, 'module', '') != 'rospy'
        if isinstance(node, ast.Expr):
            assert not isinstance(node.value, ast.Call)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    assert not any(isinstance(n.func, ast.Attribute) and n.func.attr in ('set_param', 'Publisher') for n in calls)
    request_calls = [n for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'request_plan']
    assert len(request_calls) == 1 and ast.literal_eval(request_calls[0].args[0]) is False
    assert "user_change = context_data['user_change']" in PATH.read_text()
    assert 'authorization=user_change' in PATH.read_text()


def test_actual_child_establishes_time_and_subscribers_before_true():
    runner = PATH.parents[1] / 'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py'
    tree = ast.parse(runner.read_text())
    run = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'run')
    minimum = [n.lineno for n in ast.walk(run) if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Attribute) and t.attr == 'minimum_stamp_ns' for t in n.targets)]
    subscribers = [n.lineno for n in ast.walk(run) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'Subscriber']
    start = [n.lineno for n in ast.walk(run) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == 'request_plan' and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value is True]
    assert len(minimum) == len(start) == 1 and subscribers
    assert minimum[0] < min(subscribers) <= max(subscribers) < start[0]


@pytest.mark.parametrize('strategy', ['two_stage', 'direct'])
def test_selected_strategy_is_preserved_in_child_command(strategy):
    context = dict(selection=dict(SELECTION, strategy=strategy))
    commands = []
    result = m.fresh_phase_handoff(lambda: context,
        lambda trigger: NS(success=True, message='continuous remote 6D inference already stopped'),
        lambda argv: commands.append(argv) or 0, lambda record: None)
    assert result == 0
    assert commands == [list(m.RUNNER[:-1]) + [strategy]]


def test_invalid_strategy_rejected_before_retiring_computation():
    calls = []
    with pytest.raises(RuntimeError, match='strategy'):
        m.fresh_phase_handoff(lambda: dict(selection=dict(SELECTION, strategy='invalid')),
            lambda trigger: calls.append(trigger), lambda argv: calls.append(argv), lambda record: None)
    assert calls == []
