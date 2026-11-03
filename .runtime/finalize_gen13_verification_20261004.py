#!/usr/bin/env python3
"""Save the closed gen13 evidence and progress; no ROS or robot commands."""
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
RECORD = EXT / 'grasp_records/20261004T114247.984592Z_7c99754a8ef14082bf57c5417afb9499'


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    temp = path.with_name(path.name + '.tmp-gen13-finalization')
    with temp.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temp.replace(path)


def evidence(path):
    return {'path': str(path), 'sha256': sha(path)}


def main():
    if not os.path.ismount(MOUNT) or not RUN.is_dir():
        raise RuntimeError('SanStand is not mounted; no system-disk fallback')
    validation_path = WORK / '.runtime/rgb_acquisition_validation_20261004.json'
    validation = read(validation_path)
    assert validation['test_exit_code'] == 0
    assert validation['source_unchanged_during_test'] is True
    mismatches = []
    for group in ('source_sha256', 'test_sha256'):
        for rel, expected in validation[group].items():
            if sha(WORK / rel) != expected:
                mismatches.append(rel)
    if mismatches:
        raise RuntimeError('Frozen test/source mismatch: ' + repr(mismatches))

    deployment_path = RUN / 'rgb_acquisition_deployment_20261004T114018Z.json'
    deployment = read(deployment_path)
    assert deployment['loaded'] is True
    assert deployment['source_sha256'] == validation['source_sha256']
    dispatch = read(RECORD / 'dispatch.json')
    finished = next(e for e in dispatch['events'] if e['phase'] == 'rgb_finished')
    rgb_path = Path(finished['artifact']['path'])
    assert sha(rgb_path) == finished['artifact']['sha256']
    rgb = read(rgb_path)
    assert rgb['rpc_count'] == rgb['charged_attempts'] == len(rgb['requests']) == 3
    assert rgb['grasp_started'] is False and rgb['measured_target_available'] is False
    assert rgb['actuation_prefix_completed'] is True
    ledgers = [r['ledger'] for r in rgb['requests']]
    for i, (request, ledger) in enumerate(zip(rgb['requests'], ledgers), 1):
        assert request['success'] is True
        assert ledger['attempts'] == ledger['observation_index'] == i
        assert ledger['max_observations'] == 3 and ledger['duration_sec'] == 120.0
        for key in ('request_id', 'reference_id', 'source_id', 'created_ns', 'deadline_ns'):
            assert ledger[key] == ledgers[0][key]
    assert int(ledgers[0]['deadline_ns']) - int(ledgers[0]['created_ns']) == 120_000_000_000
    assert dispatch['events'][-1]['reason'] == 'dispatch_rgb_acquisition_not_measured'
    assert dispatch['events'][-1]['automatic_new_selection'] is False

    manifest = read(RECORD / 'manifest.json')
    recorder = read(RECORD / 'recorder_outcome.json')
    assert manifest['recorder_closed'] is True and recorder['bag_closed'] is True
    assert recorder['active_seen'] is False
    checked_small_files = []
    for item in manifest['files']:
        path = RECORD / item['path']
        assert path.stat().st_size == item['size_bytes']
        if item['path'] != 'keyframes.bag':
            assert sha(path) == item['sha256']
            checked_small_files.append(item['path'])

    expected_copies = {
        'gen13_native_depth_diagnostic_20261004.json':
            '6cbe642645d4f9977bc7af4468ad5ba1d00737102ae48782a954357575bb4957',
        'diagnose_gen13_native_depth_20261004.py':
            '7f4dd17ee4cf8b6d33bc0d53c7cbeeb4d267c8b2dfaee7aec5c02a73b2733104',
    }
    for name, expected in expected_copies.items():
        source = WORK / '.runtime' / name
        assert sha(source) == expected
        destination = RUN / name
        if destination.exists():
            assert sha(destination) == expected
        else:
            shutil.copyfile(source, destination)
        assert sha(destination) == expected

    diagnostic_path = RUN / 'gen13_native_depth_diagnostic_20261004.json'
    diagnostic = read(diagnostic_path)
    for item in diagnostic['source_reports']:
        assert sha(Path(item['path'])) == item['sha256']
    rows = []
    for row in diagnostic['rows']:
        assert sha(Path(row['source_npz'])) == row['source_sha256']
        outline = row['scopes']['rgb_outline']
        core = row['scopes']['rgb_core_erode3x3']
        assert outline['within_segmentation_roi'] == outline['pixels']
        assert row['after_original_deadline_sec'] > 0
        rows.append({
            'source_stamp_ns': row['source_stamp_ns'],
            'after_original_deadline_sec': row['after_original_deadline_sec'],
            'rgb_outline_pixels': outline['pixels'],
            'valid_native_depth': outline['valid_native_depth'],
            'missing_fraction': outline['missing_fraction'],
            'core_pixels': core['pixels'],
            'core_valid_native_depth': core['valid_native_depth'],
            'native_foreground_points': outline['raw_ge_4mm_inside_roi'],
            'opened_foreground_points': outline['production_opened_pixels'],
            'current_plane_height_median_mm': outline['height_to_current_plane_mm']['p50'],
        })
    runtime_path = RUN / 'gen13_final_runtime_state.json'
    runtime = read(runtime_path)
    pair_path = EXT / 'ros_log/unknown_first_loss_pair_20261004T114247Z_168778/report.json'
    pair = read(pair_path)
    now = datetime.now(timezone.utc).isoformat()
    result_path = RUN / 'gen13_validation_result.json'
    result = {
        'recorded_at_utc': now,
        'scope': 'closed_gen13_motion_and_observation_verification_with_later_native_depth_diagnostic',
        'physical_grasp_executed': False,
        'physical_grasp_success_verified': False,
        'global_cross_view_fusion_resolved': False,
        'bounded_prefix_actuation_verified_on_robot': True,
        'full_observation_moves_completed': 2,
        'no_submission_prefix_retry_verified_on_robot': False,
        'no_submission_prefix_retry_verification': 'offline regression only; not triggered in gen13',
        'record': str(RECORD),
        'record_classification_unchanged': manifest['result'],
        'record_failure_reason_unchanged': manifest['failure_reason'],
        'keyframe_count': recorder['keyframe_count'],
        'keyframes_bag_closed': recorder['bag_closed'],
        'keyframes_bag_integrity_scope': 'manifest SHA retained and size matched; not rehashed in finalization',
        'manifest_small_files_sha256_verified': checked_small_files,
        'selection': dispatch['selection'],
        'charged_attempts': 3,
        'original_created_ns': ledgers[0]['created_ns'],
        'original_deadline_ns': ledgers[0]['deadline_ns'],
        'original_duration_sec': 120.0,
        'request_messages': finished['request_messages'],
        'last_handoff': rgb['latest_handoff'],
        'nonready_reasons': rgb['status_diagnostics']['nonready_reasons'],
        'later_native_depth_summary': rows,
        'later_native_depth_limits': diagnostic['limits'],
        'pair_recorder': {
            'status_count': pair['status_count'],
            'paired_count': pair['source_evidence']['paired_count'],
            'evidence_complete': pair['evidence_complete'],
            'run_error': pair['run_error'],
            'archive_bag_closed': pair['archive_bag_closed'],
            'finalization_errors': pair['finalization_errors'],
            'static_handeye_and_urdf_unchanged': pair['handeye_unchanged'] and pair['robot_description_unchanged'],
            'source_context_scope': 'stationary temporal runtime active changes with motion; no static calibration change inferred',
        },
        'regression': validation['test_result'],
        'frozen_source_files_sha256_verified': len(validation['source_sha256']),
        'frozen_test_files_sha256_verified': len(validation['test_sha256']),
        'evidence': {
            'dispatch': evidence(RECORD / 'dispatch.json'),
            'manifest': evidence(RECORD / 'manifest.json'),
            'rgb_run': evidence(rgb_path),
            'deployment': evidence(deployment_path),
            'regression_manifest': evidence(RUN / 'rgb_prefix_validation_20261004T113628Z.json'),
            'regression_log': evidence(Path(validation['test_log'])),
            'native_depth_diagnostic': evidence(diagnostic_path),
            'native_depth_reproduction_script': evidence(RUN / 'diagnose_gen13_native_depth_20261004.py'),
            'pair_recorder': evidence(pair_path),
            'final_readonly_runtime': evidence(runtime_path),
            'verification_document': evidence(WORK / 'docs/verification/2026-10-04-thin-target-rgb-observation.md'),
        },
        'finalizer': evidence(Path(__file__).resolve()),
        'ros_or_robot_commands_sent_by_finalization': False,
        'temperature_queried_by_finalization': False,
        'new_generation_or_budget_created_by_finalization': False,
        'task_complete_marker_written': False,
    }
    save(result_path, result)
    updates = {
        'updated_at_utc': now,
        'work_status': 'real_observation_completed_dispatch_stopped_missing_measured_geometry',
        'pause_requested_by_user': False,
        'automatic_resume_allowed': False,
        'automatic_executor_running': False,
        'task_complete': False,
        'latest_verification_episode_complete': True,
        'physical_grasp_success_verified': False,
        'current_cross_view_fusion_resolved': False,
        'current_initial_target_acquired': False,
        'current_generation_target_lost': False,
        'current_generation_target_lost_scope': 'last dispatched handoff observing; lacks measured 3D, not an identity-lost event',
        'current_selection_generation': 13,
        'current_generation_motion_commands_sent': True,
        'grasp_commands_sent_this_turn': False,
        'completed_observation_count': 2,
        'completed_observation_count_scope': 'generation 13 full requested observation moves only; prefix counted separately',
        'bounded_prefix_actuation_verified_on_robot': True,
        'latest_measured_support_difference': {
            'height_mm': 1000 * rgb['latest_handoff']['support_audit']['support_reference_max_distance_m'],
            'angle_deg': rgb['latest_handoff']['support_audit']['support_reference_angle_deg'],
            'scope': 'gen13 final handoff fixed-footprint comparison only; not a global wide-pose validation',
        },
        'latest_alignment_receipt': str(RUN / 'alignment_receipt_20261004T113909Z.json'),
        'latest_rgb_acquisition_deployment': str(deployment_path),
        'latest_loaded_source_sha256': deployment['source_sha256'],
        'pending_new_prefix_repairs_loaded': True,
        'loaded_joint_regression': validation['test_result'],
        'latest_joint_regression': validation['test_result'],
        'final_offline_regression': validation['test_result'],
        'final_offline_regression_log': validation['test_log'],
        'latest_validation_result': str(result_path),
        'latest_native_depth_diagnostic': str(diagnostic_path),
        'latest_readonly_runtime': str(runtime_path),
        'driver_pid': runtime['nodes']['/alicia_d_driver_node']['pid'],
        'driver_feedback_ready': runtime['/alicia_d/feedback_ready'],
        'driver_motion_enabled': runtime['/alicia_d/motion_enabled'],
        'driver_actuation_status': runtime['/alicia_d/actuation_status'],
        'driver_actuation_status_observed_at_utc': runtime['observed_at_utc'],
        'actual_directional_actuation_confirmation_pending': False,
        'pair_recorder_gen13_complete': pair['evidence_complete'],
        'pair_recorder_gen13_report': str(pair_path),
        'current_blocker': 'Gen13 bounded actuation prefix and two full observation moves completed. The dispatcher stopped because no measured target was acquired; all three original attempts were consumed. Later four native frames show depth holes and fewer than 60 foreground points, but are not exact earlier handoff replay. RGB-match and support-consistency rejections also remain; physical grasp and global cross-view resolution are unverified.',
    }
    for path in (RUN / 'current_progress.json', WORK / '.runtime/latest_full_ros_start.json'):
        backup = path.with_name(path.stem + '_before_gen13_finalization_20261004.json')
        if not backup.exists():
            shutil.copyfile(path, backup)
            assert sha(backup) == sha(path)
        progress = read(path)
        progress.update(updates)
        progress.pop('pending_offline_regression', None)
        records = progress.setdefault('latest_live_verification_records', [])
        if str(RECORD) not in records:
            records.append(str(RECORD))
        attempts = progress.setdefault('latest_live_attempts', [])
        if str(rgb_path) not in attempts:
            attempts.append(str(rgb_path))
        save(path, progress)
    print(json.dumps({'saved_result': str(result_path), 'sha256': sha(result_path),
                      'regression': validation['test_result'], 'source_mismatches': [],
                      'native_depth_report': str(diagnostic_path), 'task_complete': False,
                      'robot_commands_sent': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
