"""Exact GEN54 replay plus same-policy local FK-reuse comparisons.

The hash-pinned prior function still replays its original 10/6 mm policy.
A separate, explicitly derived oracle changes only its three approach caps
from 6 to 12 mm for same-policy comparisons with current production. Only
each function's AST is evaluated, with the existing production imports.
The eight recorded contact prechecks failed later; these tests certify endpoint
arithmetic equivalence, never whole-path or hardware success.
"""
import ast
from copy import deepcopy
import gzip
import hashlib
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.robot import held_pregrasp_selection as module
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q

FIXTURE_SHA = '34f65601583d1ed385ffc9492e8018153c55dad647b67ebfc53176b5d68712dd'
BASELINE_SHA = 'a55fcdc0c1c086cc822d3dd2991d2e0374b29942eae3b0374ad51661ee51fafc'


@pytest.fixture(scope='module')
def data():
    raw = (Path(__file__).parent/'fixtures/gen55_g54_held_pregrasp_inputs.json.gz').read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    value = json.loads(gzip.decompress(raw))
    assert len(value['cases']) == 8
    assert hashlib.sha256(value['baseline_source']['text'].encode()).hexdigest() == BASELINE_SHA
    assert value['baseline_source']['sha256'] == BASELINE_SHA
    assert hashlib.sha256(value['model']['xml'].encode()).hexdigest() == value['model']['sha256']
    return value


def baseline_function(data, *, current_approach_policy=True, artifact_dir=None):
    original = data['baseline_source']['text']
    assert hashlib.sha256(original.encode()).hexdigest() == BASELINE_SHA
    old_cap = 'min(position_tolerance_m, .006)'
    new_cap = 'min(position_tolerance_m, .012)'
    assert original.count(old_cap) == 3
    source = original.replace(old_cap, new_cap) if current_approach_policy else original
    source_sha = hashlib.sha256(source.encode()).hexdigest()
    provenance = dict(original_source_sha256=BASELINE_SHA, fixture_sha256=FIXTURE_SHA,
        derived_source_sha256=source_sha, original_policy_replayed=not current_approach_policy,
        normalization=([dict(before=old_cap, after=new_cap, occurrences=3)]
                       if current_approach_policy else []),
        scope='same approach cap for historical requests at 10 mm position and 5 degrees')
    if artifact_dir is not None:
        stem = 'gen54_current_approach_policy' if current_approach_policy else 'gen54_original_policy'
        source_path = artifact_dir/(stem+'.py')
        source_path.write_text(source)
        assert hashlib.sha256(source_path.read_bytes()).hexdigest() == source_sha
        (artifact_dir/(stem+'.json')).write_text(json.dumps(provenance, sort_keys=True))
    tree = ast.parse(source)
    functions = [item for item in tree.body if isinstance(item, ast.FunctionDef)]
    assert len(functions) == 1 and functions[0].name == 'select_held_pregrasp_endpoint'
    namespace = dict(vars(module))
    body = ast.Module(body=functions, type_ignores=[])
    label = ('GEN54 derived current-approach policy ' if current_approach_policy
             else 'GEN54 original frozen policy ')
    exec(compile(body, '<'+label+source_sha+'>', 'exec'), namespace)
    function = namespace['select_held_pregrasp_endpoint']
    function.frozen_source_provenance = provenance
    return function


class CountingFk(SerialUrdfFk):
    def __init__(self, xml):
        super().__init__(xml, ARM_NAMES)
        self.calls = dict(optimizer=0, verification=0)
        self.optimizing = False
        self.fail_verification = False

    def __call__(self, q):
        self.calls['optimizer' if self.optimizing else 'verification'] += 1
        if self.fail_verification and not self.optimizing:
            raise ValueError('GEN55_FINAL_REAL_FK_REJECTED')
        return super().__call__(q)


def select(function, fk, case, **kwargs):
    return function(fk, case['sample'], case['pregrasp_goal'], case['approach_goal'],
        case['nominal_command'], position_tolerance_m=case['position_tolerance_m'],
        orientation_tolerance_rad=case['orientation_tolerance_rad'], **kwargs)


def stable(value):
    """Elapsed time is the sole non-numerical solver-output comparison exception."""
    if isinstance(value, dict):
        return {key:stable(item) for key,item in value.items() if key != 'elapsed_sec'}
    if isinstance(value, list):
        return [stable(item) for item in value]
    return value


def run(function, data, case):
    fk = CountingFk(data['model']['xml'])
    assert fk.model_sha256 == case['model_sha256']
    calls = [0]
    def check():
        calls[0] += 1
    namespace = function.__globals__
    original_minimize = namespace['minimize']
    def minimize(*args, **kwargs):
        fk.optimizing = True
        try:
            return original_minimize(*args, **kwargs)
        finally:
            fk.optimizing = False
    before = deepcopy(case)
    namespace['minimize'] = minimize
    wall, cpu = time.perf_counter(), time.process_time()
    try:
        command, report = select(function, fk, case, deadline_check=check)
    finally:
        elapsed, cpu_sec = time.perf_counter()-wall, time.process_time()-cpu
        namespace['minimize'] = original_minimize
    assert case == before
    return dict(command=command.tolist(), report=report, wall_sec=elapsed,
        cpu_sec=cpu_sec, fk_calls=fk.calls, deadline_callbacks=calls[0])


@pytest.mark.parametrize('index', range(8))
def test_actual_gen54_request_preserves_historical_replay_and_same_policy_reuse(data, index, tmp_path):
    case = data['cases'][index]
    original_function = baseline_function(data, current_approach_policy=False, artifact_dir=tmp_path)
    original = run(original_function, data, case)
    historical = deepcopy(case['recorded_selection']); historical.pop('full_path')
    assert original['command'] == case['recorded_selection']['selected_command_rad']
    assert stable(original['report']) == stable(historical)
    normalized_function = baseline_function(data, artifact_dir=tmp_path)
    functions = [('baseline', normalized_function),
                 ('candidate', module.select_held_pregrasp_endpoint)]
    if os.environ.get('GEN55_REPLAY_ORDER') == 'reverse':
        functions.reverse()
    rows = {name:run(function, data, case) for name,function in functions}
    old, new = rows['baseline'], rows['candidate']
    assert old['command'] == new['command']
    assert stable(old['report']) == stable(new['report'])
    if old['fk_calls']['optimizer']:
        assert new['fk_calls']['optimizer'] < old['fk_calls']['optimizer']
    else:
        assert new['fk_calls']['optimizer'] == 0
    assert new['fk_calls']['verification'] == old['fk_calls']['verification'] > 0
    assert new['deadline_callbacks'] == old['deadline_callbacks']
    assert new['report']['success'] is True and case['recorded_precheck_success'] is False
    assert new['report']['trajectory_checked'] is False
    assert new['report']['execution_authorized'] is False
    np.testing.assert_allclose(np.array(new['command'])/Q,
        np.rint(np.array(new['command'])/Q), atol=1.e-12)
    (tmp_path/'replay.json').write_text(json.dumps(dict(index=index,
        diagnostic_call_id=case['diagnostic_call_id'], order=[name for name,_ in functions],
        outputs=rows, original_policy_output=original,
        original_policy_provenance=original_function.frozen_source_provenance,
        same_policy_provenance=normalized_function.frozen_source_provenance,
        exact_original_policy_recorded_report_equal=True,
        exact_current_policy_baseline_candidate_report_equal=True,
        current_policy_is_original_policy_replay=False,
        whole_contact_pipeline_replayed=False), sort_keys=True, allow_nan=False))


class ProbeComplete(Exception):
    pass


def test_exact_revisits_reuse_fk_without_rounding_or_skipping_callbacks(data, monkeypatch):
    fk = CountingFk(data['model']['xml']); checks = [0]
    def check(): checks[0] += 1
    def optimizer(fun, seed, **kwargs):
        constraints = kwargs['constraints'][0]['fun']
        first = constraints(seed)
        count, callbacks = sum(fk.calls.values()), checks[0]
        np.testing.assert_array_equal(constraints(seed.copy()), first)
        assert sum(fk.calls.values()) == count and checks[0] == callbacks+1
        fun(seed)
        assert sum(fk.calls.values()) == count and checks[0] == callbacks+2
        # A one-ULP change to a held joint must never hit a rounded cache key.
        seed[1] = np.nextafter(seed[1], np.inf)
        constraints(seed)
        assert sum(fk.calls.values()) > count
        raise ProbeComplete()
    monkeypatch.setattr(module, 'minimize', optimizer)
    with pytest.raises(ProbeComplete):
        select(module.select_held_pregrasp_endpoint, fk, data['cases'][0], deadline_check=check)


def test_reuse_has_bounded_storage_and_does_not_survive_another_request(data, monkeypatch):
    fk = CountingFk(data['model']['xml']); first_evaluations = []
    def optimizer(fun, seed, **kwargs):
        before = sum(fk.calls.values()); original = fun(seed)
        first_evaluations.append(sum(fk.calls.values())-before)
        for i in range(1, 130):
            probe = seed.copy(); probe[1] += i*1.e-5
            fun(probe)
        before = sum(fk.calls.values())
        assert fun(seed) == original
        assert sum(fk.calls.values()) > before
        raise ProbeComplete()
    monkeypatch.setattr(module, 'minimize', optimizer)
    for _ in range(2):
        with pytest.raises(ProbeComplete):
            select(module.select_held_pregrasp_endpoint, fk, data['cases'][0])
    assert first_evaluations == [1, 1]


@pytest.mark.parametrize('reason', ['deadline after delayed computation', 'ownership cancelled'])
@pytest.mark.parametrize('where', ['objective_cache_hit', 'constraint_cache_hit', 'post_quantization'])
def test_cached_computation_cannot_hide_deadline_or_cancellation(data, monkeypatch, where, reason):
    fk = CountingFk(data['model']['xml']); state = dict(expired=False, count=None)
    class Cancelled(ValueError): pass
    def check():
        if state['expired']:
            assert sum(fk.calls.values()) == state['count']
            raise Cancelled(reason)
    def optimizer(fun, seed, **kwargs):
        fun(seed); kwargs['constraints'][0]['fun'](seed)
        state.update(expired=True, count=sum(fk.calls.values()))
        if where == 'objective_cache_hit': fun(seed)
        if where == 'constraint_cache_hit': kwargs['constraints'][0]['fun'](seed)
        return NS(success=True, nit=1, message='synthetic solver return', x=seed)
    monkeypatch.setattr(module, 'minimize', optimizer)
    with pytest.raises(Cancelled, match=reason):
        select(module.select_held_pregrasp_endpoint, fk, data['cases'][0], deadline_check=check)


def test_solver_cache_does_not_replace_actual_quantized_fk_verification(data, monkeypatch):
    fk = CountingFk(data['model']['xml'])
    def optimizer(fun, seed, **kwargs):
        fun(seed); kwargs['constraints'][0]['fun'](seed)
        fk.fail_verification = True
        return NS(success=True, nit=1, message='synthetic solver return', x=seed)
    monkeypatch.setattr(module, 'minimize', optimizer)
    with pytest.raises(ValueError, match='GEN55_FINAL_REAL_FK_REJECTED'):
        select(module.select_held_pregrasp_endpoint, fk, data['cases'][0])


@pytest.mark.parametrize('fault', ['nonfinite', 'unqualified'])
def test_unsuccessful_solver_retains_exact_failure_and_evidence(data, monkeypatch, fault, tmp_path):
    def optimizer(fun, seed, **kwargs):
        fun(seed); kwargs['constraints'][0]['fun'](seed)
        result = seed.copy()
        if fault == 'nonfinite': result[0] = np.nan
        return NS(success=False, nit=1, message='frozen failure probe', x=result)
    failures = []
    for function in (baseline_function(data, artifact_dir=tmp_path), module.select_held_pregrasp_endpoint):
        monkeypatch.setitem(function.__globals__, 'minimize', optimizer)
        fk = CountingFk(data['model']['xml'])
        with pytest.raises(HeldAxesPredictionError) as caught:
            select(function, fk, data['cases'][0])
        evidence = deepcopy(caught.value.evidence)
        selection = evidence['pregrasp_endpoint_selection']
        if function is module.select_held_pregrasp_endpoint and fault == 'unqualified':
            # GEN60 extends the rejected finite-guide search with a bounded
            # integer-neighbour audit. Validate that addition explicitly;
            # every pre-existing rejection field must still compare exactly.
            audit = selection['quantized_neighborhood']
            assert set(audit) == {'schema_version', 'scope', 'step_counts',
                'max_full_checks', 'maximum_enumerated_count', 'enumerated_count',
                'duplicate_count', 'unique_candidate_count', 'pregrasp_screen_pass_count',
                'checked', 'eligible_seed_indices', 'execution_authorized',
                'trajectory_checked', 'success', 'elapsed_sec'}
            assert audit['schema_version'] == 1 and audit['step_counts'] == 1
            assert audit['scope'] == 'conditional_quantized_pregrasp_neighborhood'
            assert audit['max_full_checks'] == 16
            assert audit['maximum_enumerated_count'] == 3*3**6
            assert audit['eligible_seed_indices'] == [0, 1, 2]
            assert audit['enumerated_count'] == 3*3**6
            assert 0 <= audit['duplicate_count'] <= audit['enumerated_count']
            assert audit['unique_candidate_count']+audit['duplicate_count'] == audit['enumerated_count']
            assert 0 < audit['pregrasp_screen_pass_count'] <= audit['unique_candidate_count']
            assert len(audit['checked']) == min(16, audit['pregrasp_screen_pass_count'])
            assert np.isfinite(audit['elapsed_sec']) and audit['elapsed_sec'] >= 0
            assert audit['success'] is False and audit['execution_authorized'] is False
            assert audit['trajectory_checked'] is False
            guide = 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'
            assert len(selection['trials']) == 3
            assert all(t['rejection'] == guide and np.all(np.isfinite(t['solver_endpoint']))
                       for t in selection['trials'])
            checked_counts, order = [], []
            for item in audit['checked']:
                assert set(item) == {'seed_index', 'sdk_counts', 'command_rad',
                    'ranking_proxy_cost', 'qualified', 'rejection', 'rejection_evidence'}
                assert item['qualified'] is False and item['rejection'] == guide
                assert item['seed_index'] in audit['eligible_seed_indices']
                counts = np.asarray(item['sdk_counts'])
                assert counts.shape == (6,) and np.all(counts == counts.astype(int))
                assert np.all(counts >= 0) and np.all(counts <= 4095)
                np.testing.assert_array_equal(item['command_rad'], (counts-2048)*Q)
                centre = module.sdk_counts(selection['trials'][item['seed_index']]['quantized_command_rad'])
                assert np.max(np.abs(counts-centre)) <= 1
                assert np.isfinite(item['ranking_proxy_cost']) and item['ranking_proxy_cost'] >= 0
                rejection = item['rejection_evidence']
                assert rejection['code'] == guide and rejection['success'] is False
                assert rejection['execution_authorized'] is False
                assert rejection['predicted_position_error_m'] > rejection['position_tolerance_m'] or (
                    rejection['predicted_orientation_error_rad'] > rejection['orientation_tolerance_rad'])
                checked_counts.append(tuple(counts)); order.append((item['ranking_proxy_cost'],tuple(counts)))
            assert len(set(checked_counts)) == len(checked_counts) and order == sorted(order)
            del selection['quantized_neighborhood']
        else:
            # Nonfinite original failures retain the old evidence verbatim.
            assert 'quantized_neighborhood' not in selection
        evidence = stable(evidence)
        # NaN solver endpoints deliberately remain nonfinite in the original
        # report; JSON renders the same evidence while NaN != NaN in Python.
        failures.append((str(caught.value), json.dumps(evidence, sort_keys=True)))
    assert failures[0] == failures[1]
