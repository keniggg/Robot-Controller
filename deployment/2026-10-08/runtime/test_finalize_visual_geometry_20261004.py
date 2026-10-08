"""Historical report admission and isolation from a newer session's progress."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest


spec = importlib.util.spec_from_file_location(
    'visual_finalizer', Path(__file__).with_name('finalize_visual_geometry_20261004.py'))
finalizer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(finalizer)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))
    return path


@pytest.fixture
def capture(tmp_path):
    root = tmp_path / 'historical_capture'
    runtime = write(root / 'result.json', {'visual_nodes_restored': True})
    sensor = write(root / 'stages/summary.json', {
        'counts': {'written': 3, 'complete': 2},
        'capture_complete': False, 'experiment_valid': False,
        'restoration': {'succeeded': True}})
    analysis = {
        'schema': 1, 'source_capture': str(root),
        'source_result_sha256': finalizer.sha(runtime),
        'source_summary_sha256': finalizer.sha(sensor),
        'all_source_files_and_array_hashes_verified': True,
        'protocol': {'counts': {'written': 3, 'complete': 2},
                     'capture_complete': False, 'experiment_valid': False,
                     'restoration_succeeded': True, 'visual_nodes_restored': True},
        'rows': [{'index': 1, 'complete': True}, {'index': 2, 'complete': False},
                 {'index': 3, 'complete': True}],
        'phase_summary': {
            'A1': {'attempted': 1, 'complete_pairs': 1},
            'B': {'attempted': 1, 'complete_pairs': 0},
            'A2': {'attempted': 1, 'complete_pairs': 1}},
        'limitations': ['Incomplete protocol; descriptive only.']}
    report = write(root / 'analysis.json', analysis)
    return runtime, sensor, report, analysis


def test_preserves_explicitly_incomplete_evidence(capture):
    runtime, sensor, report, _ = capture
    result = finalizer.validate_analysis(report, runtime, sensor)
    assert result['protocol']['experiment_valid'] is False
    assert result['protocol']['capture_complete'] is False


@pytest.mark.parametrize('change', [
    lambda a: a.update(source_capture='/another/target'),
    lambda a: a.update(source_result_sha256='0' * 64),
    lambda a: a.update(source_summary_sha256='0' * 64),
    lambda a: a.update(all_source_files_and_array_hashes_verified=False),
    lambda a: a['protocol'].update(experiment_valid=True),
    lambda a: a['protocol'].update(capture_complete=True),
    lambda a: a['protocol'].update(restoration_succeeded=False),
    lambda a: a['protocol'].update(visual_nodes_restored=False),
    lambda a: a['rows'].pop(),
    lambda a: a['rows'][1].update(index=1),
    lambda a: a['rows'][1].update(complete=True),
    lambda a: a['phase_summary']['B'].update(complete_pairs=1),
    lambda a: a.update(limitations=[]),
])
def test_rejects_wrong_capture_or_promoted_partial_record(capture, change):
    runtime, sensor, report, analysis = capture
    change(analysis)
    write(report, analysis)
    with pytest.raises(AssertionError):
        finalizer.validate_analysis(report, runtime, sensor)


def test_archiving_cannot_overwrite_new_object_progress(tmp_path, monkeypatch, capture):
    work, mount = tmp_path / 'work', tmp_path / 'mount'
    ext = mount / '实验记录/Robot-Controller'
    run = ext / 'ros_log/full_ros_resume_20261004T002930Z'
    monkeypatch.setattr(finalizer, 'WORK', work)
    monkeypatch.setattr(finalizer, 'MOUNT', mount)
    monkeypatch.setattr(finalizer, 'EXT', ext)
    monkeypatch.setattr(finalizer, 'RUN', run)
    monkeypatch.setattr(Path, 'is_mount', lambda path: path == mount)
    newer = {'generation': 29, 'target': 'new object', 'session': 'new ROS start'}
    current = write(run / 'current_progress.json', newer)
    latest = write(work / '.runtime/latest_full_ros_start.json', newer)
    original_current, original_latest = current.read_bytes(), latest.read_bytes()
    for name in ('analyze_camera_preset_ab_20261004.py', 'test_finalize_visual_geometry_20261004.py'):
        (work / '.runtime' / name).write_text('# Offline fixture source.\n')
    write(work / '.runtime/rgbd_rectification_validation_20261004.json', {
        'test_exit_code': 0, 'source_unchanged_during_test': True,
        'source_sha256': {}, 'test_sha256': {}, 'test_result': 'fixture pass',
        'additional_test_runs': []})
    log = run / 'fixture_test.log'
    log.write_text('fixture test passed\n')
    write(run / 'camera_persisted_rectification_supplement_20261004T145114.619461Z.json', {
        'passed': True, 'source_unchanged_during_test': True,
        'source_sha256_after': {}, 'test_log': str(log),
        'test_log_sha256': finalizer.sha(log), 'test_result': 'fixture supplement pass'})
    write(ext / 'ros_log/pair_rectification_resume_20261004T141101.307578Z/result.json',
          {'loaded': True})
    archived = ext / 'ros_log/camera_preset_ab_20261004T145322.001056Z'
    runtime = write(archived / 'result.json', {
        'visual_nodes_restored': True, 'motion_commands_sent': False,
        'torque_commands_sent': False, 'after': {'camera': 'historical PID'}})
    _, _, _, analysis = capture
    sensor = write(archived / 'stages/summary.json', {
        'counts': analysis['protocol']['counts'],
        'capture_complete': False, 'experiment_valid': False,
        'restoration': {'succeeded': True},
        'original_read_restored': True, 'original_sync_restored': True, 'files': []})
    analysis.update(source_capture=str(archived),
                    source_result_sha256=finalizer.sha(runtime),
                    source_summary_sha256=finalizer.sha(sensor))
    report = write(archived / 'analysis.json', analysis)
    doc = work / 'docs/verification/2026-10-04-thin-target-rgb-observation.md'
    doc.parent.mkdir(parents=True)
    doc.write_text('Historical diagnostics.\n')
    for name in (
        'gen13_rgb_support_exact_20261004T122355Z/rgb_rectified_pixel_counterfactual.json',
        'rgb_observation_bootstrap_20261004T141247Z/measurement_gate_diagnostic.json',
        'camera_preset_ab_20261004T144329.072192Z/result.json',
    ):
        write(ext / 'ros_log' / name, {})
    monkeypatch.setattr(sys, 'argv', ['finalizer', '--analysis', str(report)])
    finalizer.main()
    assert current.read_bytes() == original_current
    assert latest.read_bytes() == original_latest
    results = list(run.glob('visual_geometry_result_*.json'))
    results = [p for p in results if '_finalize_' not in p.name and '_rgbd_' not in p.name]
    assert len(results) == 1
    result = json.loads(results[0].read_text())
    assert result['current_progress_modified'] is False
    assert result['current_runtime_observed'] is False
    assert result['task_complete'] is False
    assert result['preset_comparison']['experiment_valid'] is False
