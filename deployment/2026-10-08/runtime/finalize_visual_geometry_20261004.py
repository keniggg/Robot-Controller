#!/usr/bin/env python3
"""Archive the completed historical visual diagnostics, without live-state updates.

This may run after a new object/session has started. Archived generation 13
receipts cannot describe the current camera, joint state, target or alignment.
No ROS imports, control calls, current-progress writes or storage fallback.
"""
import argparse
import hashlib
import json
import os
import shutil
from datetime import datetime, timezone
from pathlib import Path

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
RUN = EXT / 'ros_log/full_ros_resume_20261004T002930Z'


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evidence(path):
    return dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size)


def save(path, value):
    temporary = path.with_name(path.name + '.tmp-visual-finalization')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    temporary.replace(path)


def validate_analysis(path, runtime_path, sensor_path):
    """Bind the offline report to this exact capture, including partial results."""
    analysis, runtime, sensor = read(path), read(runtime_path), read(sensor_path)
    assert analysis['schema'] == 1
    assert Path(analysis['source_capture']).resolve() == runtime_path.parent.resolve()
    assert analysis['source_result_sha256'] == sha(runtime_path)
    assert analysis['source_summary_sha256'] == sha(sensor_path)
    assert analysis['all_source_files_and_array_hashes_verified'] is True
    protocol = analysis['protocol']
    for name in ('counts', 'capture_complete', 'experiment_valid'):
        assert protocol[name] == sensor[name], name
    assert protocol['restoration_succeeded'] == sensor['restoration']['succeeded']
    assert protocol['visual_nodes_restored'] == runtime['visual_nodes_restored']
    rows = analysis['rows']
    assert len(rows) == sensor['counts']['written']
    assert len({row['index'] for row in rows}) == len(rows)
    assert sum(bool(row['complete']) for row in rows) == sensor['counts']['complete']
    assert sum(v['attempted'] for v in analysis['phase_summary'].values()) == len(rows)
    assert sum(v['complete_pairs'] for v in analysis['phase_summary'].values()) == sensor['counts']['complete']
    # An incomplete A/B sequence can still be archived as descriptive evidence;
    # it must not become a qualified experiment or automatic preset promotion.
    assert isinstance(analysis['limitations'], list) and analysis['limitations']
    return analysis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--analysis', type=Path, required=True)
    args = parser.parse_args()
    assert MOUNT.is_mount() and RUN.is_dir(), 'SanStand required; no fallback'
    assert EXT in args.analysis.resolve().parents
    manifest_path = WORK / '.runtime/rgbd_rectification_validation_20261004.json'
    manifest = read(manifest_path)
    assert manifest['test_exit_code'] == 0 and manifest['source_unchanged_during_test']
    for group in ('source_sha256', 'test_sha256'):
        for name, digest in manifest[group].items():
            assert sha(WORK/name) == digest, name
    supplement_path = RUN/'camera_persisted_rectification_supplement_20261004T145114.619461Z.json'
    supplement = read(supplement_path)
    assert supplement['passed'] and supplement['source_unchanged_during_test']
    for name, digest in supplement['source_sha256_after'].items():
        assert sha(WORK/name) == digest, name
    assert sha(Path(supplement['test_log'])) == supplement['test_log_sha256']
    deployment_path = EXT/'ros_log/pair_rectification_resume_20261004T141101.307578Z/result.json'
    runtime_path = EXT/'ros_log/camera_preset_ab_20261004T145322.001056Z/result.json'
    runtime = read(runtime_path)
    assert read(deployment_path)['loaded'] and runtime['visual_nodes_restored']
    sensor_path = runtime_path.parent/'stages/summary.json'
    sensor = read(sensor_path)
    assert sensor['restoration']['succeeded'] and sensor['original_read_restored'] and sensor['original_sync_restored']
    assert not runtime['motion_commands_sent'] and not runtime['torque_commands_sent']
    for row in sensor['files']:
        path = sensor_path.parent/row['name']
        assert path.stat().st_size == row['bytes'] and sha(path) == row['sha256']
    analysis = validate_analysis(args.analysis, runtime_path, sensor_path)
    now = datetime.now(timezone.utc)
    output = RUN/('visual_geometry_result_'+now.strftime('%Y%m%dT%H%M%S.%fZ')+'.json')
    doc = WORK/'docs/verification/2026-10-04-thin-target-rgb-observation.md'
    mirror = EXT/'文档/verification'/doc.name
    mirror.parent.mkdir(parents=True, exist_ok=True)
    if mirror.exists() and sha(mirror) != sha(doc):
        shutil.copyfile(mirror, mirror.with_name(mirror.stem+'_before_'+now.strftime('%H%M%S')+'.md'))
    shutil.copyfile(doc, mirror); assert sha(doc) == sha(mirror)
    paths = [manifest_path, supplement_path, deployment_path, runtime_path, sensor_path,
             args.analysis, doc, mirror,
             WORK/'.runtime/analyze_camera_preset_ab_20261004.py',
             WORK/'.runtime/test_finalize_visual_geometry_20261004.py',
             EXT/'ros_log/gen13_rgb_support_exact_20261004T122355Z/rgb_rectified_pixel_counterfactual.json',
             EXT/'ros_log/rgb_observation_bootstrap_20261004T141247Z/measurement_gate_diagnostic.json',
             EXT/'ros_log/camera_preset_ab_20261004T144329.072192Z/result.json']
    result = dict(recorded_at_utc=now.isoformat(), task_complete=False,
        record_scope='Historical generation 13 and following visual maintenance only; not the current target or runtime.',
        current_runtime_observed=False, current_progress_modified=False,
        physical_grasp_success_verified=False, automatic_motion_started=False,
        completed_software_fixes=['SDK lens-model RGB-D rectification',
            'fixed original-volume support consistency domain with current coverage',
            'RGB valid-domain admission', 'visual-node recovery after stale registration'],
        software_loaded_at_archived_deployment=True, camera_configuration_persisted=True,
        main_regression=manifest['test_result'],
        additional_regressions=[r['test_result'] for r in manifest['additional_test_runs']],
        persisted_config_supplement=supplement['test_result'],
        regression_counts_must_not_be_summed_due_to_overlap=True,
        exact_archival_replay=dict(rgb_and_support_pass=88, measured_target_ready=0),
        remaining='Actual target metric depth remains insufficient or does not overlap the RGB target. '
                  'Historical 19–20 mm physical cross-view cause is not uniquely identified or globally resolved.',
        preset_comparison=dict(capture_complete=sensor['capture_complete'],
            experiment_valid=sensor['experiment_valid'], restoration_verified=True,
            phase_summary=analysis['phase_summary'], limitations=analysis['limitations'],
            interpretation='Use attached analysis with incomplete-frame/source limits; no automatic preset promotion.'),
        charuco=dict(code_verified=True, node_reloaded=False, new_handeye_calibration=False),
        selected_generation=13, old_generation_renewed=False,
        visual_processes_at_archived_maintenance=runtime['after'],
        source_manifest_scope='Frozen tested deployment set, not proof every module was imported at runtime.',
        source_sha256=manifest['source_sha256'],
        evidence=[evidence(p) for p in paths], finalizer=evidence(Path(__file__).resolve()),
        ros_or_robot_commands_sent_by_finalization=False, task_complete_marker_written=False)
    save(output, result)
    # Never overwrite current_progress.json or latest_full_ros_start.json here:
    # the user can have replaced the object/restarted ROS since these receipts.
    # A current-session owner may explicitly link this immutable historical result.
    for path in (manifest_path, Path(__file__).resolve(),
                 WORK/'.runtime/analyze_camera_preset_ab_20261004.py',
                 WORK/'.runtime/test_finalize_visual_geometry_20261004.py'):
        copy = RUN/(output.stem+'_'+path.name)
        shutil.copyfile(path,copy); assert sha(copy)==sha(path)
    print(json.dumps(dict(result=str(output), sha256=sha(output), task_complete=False,
                         physical_grasp_success_verified=False, document_mirror=str(mirror))))


if __name__ == '__main__':
    main()
