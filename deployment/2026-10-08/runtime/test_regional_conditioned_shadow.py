"""Bounded scheduling and inherited fail-open lifetimes; entirely offline."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

import test_regional_shadow_lifecycle as lifecycle

spec = importlib.util.spec_from_file_location('conditioned_shadow_test',
    Path(__file__).with_name('regional_conditioned_shadow.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.fixture
def setup(tmp_path, monkeypatch):
    facade = SimpleNamespace(ab=module.ab, stage=module.stage,
                             RegionalShadow=module.RegionalConditionedShadow)
    monkeypatch.setattr(lifecycle, 'module', facade)
    fixture = lifecycle.setup.__wrapped__(tmp_path, monkeypatch)
    context = next(fixture)
    yield context
    try:
        next(fixture)
    except StopIteration:
        pass


# The exact original 10 lifecycle assertions run on the subclass too.
for _name, _test in vars(lifecycle).items():
    if _name.startswith('test_'):
        globals()['test_inherited_'+_name[5:]] = _test


@pytest.fixture
def conditioned(setup, monkeypatch):
    c = setup
    c.observer.samples = 8
    c.observer.baseline_interval = 3.
    c.audit = dict(active=True, fully_active=False,
        regions=[dict(name='center', qualified=True, filter_applied=True,
                      visual=dict(reason='stationary')),
                 dict(name='top_left', qualified=False, filter_applied=False,
                      visual=dict(reason='visual_motion'))])
    c.camera.runtime_profile['stationary_temporal']['active'] = False
    original_process = c.Regional.process
    def process(self, *args, **kwargs):
        result, _ = original_process(self, *args, **kwargs)
        return result, copy.deepcopy(c.audit)
    monkeypatch.setattr(c.Regional, 'process', process)
    original_filter = c.cls._filter_stationary_depth
    def filtered(self, frame, diagnostic=None, color=None, source=None):
        source['frame_number'] = self.calls
        return original_filter(self, frame, diagnostic, color, source)
    monkeypatch.setattr(c.cls, '_filter_stationary_depth', filtered)
    return c


def take(c, seconds=.5):
    c.clock.step(seconds)
    result = c.camera.read()
    rows = []
    while not c.observer.queue.empty():
        rows.append(c.observer.queue.get_nowait())
    return result, rows


def baseline(c):
    take(c, 0.)  # Establish one complete read, per the offline fixture.
    _, first = take(c, .5)
    _, second = take(c, 3.)
    assert len(first) == len(second) == 1
    assert first[0]['selection_kind'] == second[0]['selection_kind'] == 'uniform_baseline'
    return first+second


def test_previous_condition_selects_but_actual_failure_is_preserved(conditioned):
    c = conditioned
    baseline(c)
    previous = copy.deepcopy(c.observer.previous_condition)
    assert previous['global_active'] is False and previous['center_active'] is True
    c.audit['regions'][0].update(qualified=False, filter_applied=False,
                               visual=dict(reason='visual_motion'))
    c.camera.runtime_profile['stationary_temporal']['active'] = True
    c.camera.previous_published_metadata = dict(frame_number=999)
    returned, rows = take(c)
    assert returned is c.camera.last_result and len(rows) == 1
    row = rows[0]
    assert row['previous_condition'] == previous
    assert row['actual_condition']['center_active'] is False
    assert row['actual_condition']['global_active'] is True
    assert row['actual_condition']['center_reason'] == 'visual_motion'
    assert row['actual_condition']['read_complete'] is True
    assert row['shadow_current_sdk_color_source']['frame_number'] == c.camera.calls
    assert row['shadow_current_sdk_color_source']['frame_number'] != previous['sdk_color_source']['frame_number']
    assert row['published_ros_image_stamp_available_at_tap'] is False
    assert c.observer.condition_counts['conditional_selected'] == 1
    assert take(c)[1] == []  # Current failure prevents the next trigger only.


def test_uniform_baselines_then_at_most_six_conditioned_samples_and_minimum_interval(conditioned):
    c = conditioned
    baseline(c)
    for _ in range(6):
        assert take(c, .25)[1] == []
        assert len(take(c, .25)[1]) == 1
    obs = c.observer
    assert obs.closed and obs.close_reason == 'sample_limit' and obs.attempts == 8
    assert obs.condition_counts['baseline_selected'] == 2
    assert obs.condition_counts['conditional_selected'] == 6
    counts = copy.deepcopy(obs.condition_counts)
    assert take(c)[1] == [] and obs.condition_counts == counts
    assert c.cls.read is c.original and obs.inflight == 0


def test_no_matching_condition_expires_at_original_deadline_without_forced_fill(conditioned):
    c = conditioned
    c.camera.runtime_profile['stationary_temporal']['active'] = True
    baseline(c)
    assert take(c, 20.)[1] == []
    assert take(c, 6.5)[1] == []
    obs = c.observer
    assert obs.clock()-obs.first_read == 30.
    assert obs.closed and obs.close_reason == 'deadline' and obs.attempts == 2
    summary = obs.summary()
    assert summary['maximum_capture_seconds'] == 30.
    assert summary['conditional_samples_maximum'] == 6 and not summary['capture_complete']
    assert c.cls.read is c.original


def test_cross_table_counts_all_reads_and_separates_rejected_and_original_error(conditioned):
    c = conditioned
    baseline(c)
    c.camera.complete = False
    returned, rows = take(c)
    assert returned == (None, None) and len(rows) == 1 and not rows[0]['complete']
    assert rows[0]['actual_condition']['read_complete'] is False
    assert c.observer.previous_condition['read_complete'] is False
    c.camera.complete = True
    assert take(c)[1] == []  # Rejected read cannot trigger immediately.
    c.camera.failure = RuntimeError('original failure')
    with pytest.raises(RuntimeError) as error:
        take(c)
    assert error.value is c.camera.failure
    counts = c.observer.condition_counts
    assert counts['reads'] == 6 and counts['complete'] == 4
    assert counts['rejected'] == counts['original_error'] == counts['audit_unavailable'] == 1
    table = c.observer.condition_cross_table
    assert table['complete']['global_false_center_true'] == 4
    assert table['rejected']['global_false_center_true'] == 1
    assert table['original_error']['global_unavailable_center_unavailable'] == 1
    reasons = c.observer.region_reason_counts
    assert reasons['complete']['center']['qualified_true:stationary'] == 4
    assert reasons['rejected']['top_left']['qualified_false:visual_motion'] == 1
    assert not reasons['original_error']


def test_summary_waits_for_current_read_statistics_and_diagnostic_failure_is_fail_open(conditioned, monkeypatch):
    c = conditioned
    baseline(c)
    def broken_statistics(_):
        raise ValueError('statistics deliberately unavailable')
    monkeypatch.setattr(c.observer, '_record_completed_read', broken_statistics)
    result, _ = take(c)
    assert result is c.camera.last_result
    assert c.observer.closed and c.observer.close_reason == 'conditioned_read_statistics_error'
    assert c.observer.previous_condition is None
    assert c.observer.inflight == 0 and c.cls.read is c.original


def test_rejected_pair_does_not_reuse_previous_read_source(conditioned):
    c = conditioned
    baseline(c)
    c.camera.complete = False
    _, rows = take(c)
    assert rows[0]['actual_condition']['read_complete'] is False
    assert rows[0]['shadow_current_sdk_color_source']['frame_number'] == 4
    c.camera.failure = RuntimeError('failed before filter tap')
    with pytest.raises(RuntimeError):
        take(c)
    assert c.observer.latest_shadow_source is None
    assert c.observer.previous_condition['sdk_color_source'] is None
    assert c.observer.previous_condition['shadow_audit_available'] is False


def test_original_exception_restores_temporary_selection_gate_and_interval(conditioned):
    c = conditioned
    c.camera.runtime_profile['stationary_temporal']['active'] = True
    baseline(c)
    obs = c.observer
    old_stable, old_interval = obs.stable_reads, obs.interval
    c.camera.failure = RuntimeError('production read failed under closed selection gate')
    with pytest.raises(RuntimeError) as error:
        take(c)
    assert error.value is c.camera.failure
    assert obs.stable_reads == old_stable and obs.interval == old_interval
    assert obs.inflight == 0 and obs.current_row is None
    assert obs.condition_counts['conditional_selected'] == 0
    assert obs.previous_condition['read_complete'] is False
    assert '_filter_stationary_depth' not in c.camera.__dict__
