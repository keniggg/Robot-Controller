"""Bind a verified GEN17 compute reload and reconcile current progress.

Metadata only: no ROS publications, node lifecycle or physical commands.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def save_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def current_identity(expected):
    data = (Path('/proc') / str(expected['pid']) / 'stat').read_text().rsplit(')', 1)[1].split()
    assert data[0] != 'Z' and data[19] == expected['start_ticks']
    assert int(data[1]) == expected['parent_pid']


def main():
    ap = argparse.ArgumentParser()
    for name in ('install', 'tests', 'release', 'reload', 'previous-deployment', 'deployment', 'output'):
        ap.add_argument('--' + name, type=Path, required=True)
    args = ap.parse_args()
    for name, value in vars(args).items():
        setattr(args, name, value.resolve())
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and all(mount in p.resolve().parents for p in (args.output, args.deployment))
    installed, tested, release, reload, previous = map(read,
        (args.install, args.tests, args.release, args.reload, args.previous_deployment))
    sources = installed['source_sha256']
    assert installed['passed'] and tested['passed'] and tested['source_unchanged']
    assert tested['prior_sha256'] == sha(args.install)
    assert release['passed'] and release['reviewed'] and release['installed']
    assert release['kind'].startswith('gen17_')
    assert all(release['evidence_sha256'][str(p)] == sha(p)
               for p in (args.install, args.tests))
    assert sources == tested['source_sha256'] == release['source_sha256'] == reload['source_sha256']
    assert release['test_summary'] == tested['summary']
    assert tested['summary']['tests'] > 0 and all(tested['summary'][k] == 0 for k in ('failures', 'errors', 'skipped'))
    assert release['test_scope'] == installed['test_scope']
    for name, digest in sources.items():
        assert sha(work / name) == digest, name
    assert previous['deployment_passed'] and previous['whole_patch_deployed']
    assert reload['deployment_passed'] and reload['before'] == previous['after']
    assert reload['preserved_before'] == reload['preserved_after'] == previous['preserved_after']
    assert len(reload['after']) == 3 and len(reload['preserved_after']) == 8
    assert all(reload['before'][name] != row for name, row in reload['after'].items())
    for row in (*reload['after'].values(), *reload['preserved_after'].values()):
        current_identity(row)
    for row in reload['before'].values():
        path = Path('/proc') / str(row['pid']) / 'stat'
        if path.exists():
            data = path.read_text().rsplit(')', 1)[1].split()
            assert data[0] == 'Z' or data[19] != row['start_ticks']
    acknowledgements = [json.loads(reload['reset_ack'][name]) for name in
        ('/grasp_mode/reset_remote', '/grasp_mode/reset_task')]
    assert all(ack['ready'] is True for ack in acknowledgements)
    selections = [{key: ack[key] for key in ('mode', 'strategy', 'generation', 'stamp_ns')}
                  for ack in acknowledgements]
    assert selections[0] == selections[1]
    selection = selections[0]
    manual_path = args.deployment.parent / 'gen17_before_reload_manual/result.json'
    manual = read(manual_path)
    assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
    assert manual['generation'] == selection['generation']
    evidence = {str(p.resolve()): sha(p) for p in (
        args.install, args.tests, args.release, args.reload, args.previous_deployment,
        work / '.runtime/load_gen16_consumers_20261013.py', manual_path, Path(__file__))}
    for p in (args.output.parent / 'gen16_compute_loader_review/result.json',):
        review = read(p)
        assert review['passed']
        helper = work / '.runtime/load_gen16_consumers_20261013.py'
        assert Path(review['helper_path']).resolve() == helper
        assert review['helper_sha256'] == sha(helper)
        evidence[str(p)] = sha(p)
    deployment = dict(reload, whole_patch_deployed=True, compute_reloaded=True,
        hardware_processes_preserved=True, source_manifest=str(args.release),
        source_manifest_sha256=sha(args.release), reload_evidence=str(args.reload),
        reload_sha256=sha(args.reload), previous_deployment=str(args.previous_deployment),
        previous_deployment_sha256=sha(args.previous_deployment), evidence_sha256=evidence,
        selection=selection, recorded_ns=time.time_ns(), physical_grasp_verified=False,
        deployment_kind='gen17_compute_only_reload_after_actual_integrated_regression')
    runtime_dir = args.deployment.parent
    paths = [runtime_dir / 'current_progress.json', work / '.runtime/latest_full_ros_start.json']
    before = {str(p): p.read_bytes() for p in paths}
    states = {name: json.loads(value) for name, value in before.items()}
    assert all(not value.get('automatic_grasp_running') and value.get('active_record') is None
               and value.get('physical_grasp_verified') is False for value in states.values())
    assert len({value['last_record'] for value in states.values()}) == 1
    args.output.mkdir(exist_ok=False)
    for index, p in enumerate(paths):
        (args.output / ('before_%d.json' % index)).write_bytes(before[str(p)])
    save_new(args.deployment, deployment)
    count = len(sources)
    common = dict(active_work='gen17_deployed_fresh_recorded_grasp_preflight',
        metadata_reconciliation_id='gen17_runtime_progress_' + str(time.time_ns()),
        current_regression=str(args.tests.parent), current_test_result=str(args.tests),
        current_test_result_sha256=sha(args.tests), current_regression_summary=tested['summary'],
        current_test_summary=tested['summary'], current_test_passed=True, current_test_returncode=0,
        current_regression_passed=True, current_regression_elapsed_sec=tested['elapsed_sec'],
        current_regression_state='passed', current_test_scope_files=len(installed['test_scope']),
        pending_combined_test_scope_files=0, installed_repair=str(args.install),
        installed_repair_sha256=sha(args.install), source_manifest=str(args.release),
        source_manifest_sha256=sha(args.release), runtime_source_manifest=str(args.release),
        runtime_source_manifest_sha256=sha(args.release), verified_disk_release_manifest=str(args.release),
        verified_disk_release_manifest_sha256=sha(args.release), sealed_source_manifest=str(args.release),
        sealed_source_manifest_sha256=sha(args.release), disk_source_evidence=str(args.install),
        disk_source_evidence_sha256=sha(args.install), source_count=count, current_source_count=count,
        disk_source_count=count, runtime_source_count=count, operational_source_sha256=sources,
        disk_source_sha256=sources, runtime_matches_disk=True, runtime_reload_pending=False,
        runtime_status='gen17_compute_loaded_hardware_preserved', installed_but_not_loaded_patch=None,
        installed_but_not_loaded_patch_sha256=None, whole_patch_deployed=True, compute_reloaded=True,
        hardware_processes_preserved=True, deployment_passed=True, startup_runtime_verified=True,
        deployment=str(args.deployment), current_deployment=str(args.deployment), latest_deployment=str(args.deployment),
        deployment_sha256=sha(args.deployment), runtime_receipt=str(args.deployment),
        runtime_receipt_sha256=sha(args.deployment), compute_unit=reload['unit'],
        reload_evidence=str(args.reload), reload_evidence_sha256=sha(args.reload),
        deployed_node_identities=dict(reload['after'], **reload['preserved_after']),
        runtime_identity_verified_ns=time.time_ns(),
        runtime_identity_verification='Bound reload plus live /proc PID/start ticks/parent; snapshot, not continuous monitoring.',
        selection=selection, current_selection=selection, current_generation=selection['generation'],
        expected_generation=selection['generation'], next_generation=selection['generation'] + 1,
        selection_evidence=str(args.deployment), selection_evidence_sha256=sha(args.deployment),
        selection_is_live_snapshot=False, selection_scope='Matching reload reset ACKs; fresh preflight still required.',
        candidate_delivery=release['candidate_delivery'], candidate_delivery_sha256=sha(release['candidate_delivery']),
        staged_algorithm_candidate=str(Path(release['candidate_delivery']).parent), candidate_installed=True,
        staged_candidate_installed=True, staged_candidate_test_result=str(args.tests),
        staged_candidate_test_result_sha256=sha(args.tests), staged_candidate_test_count=tested['summary']['tests'],
        staged_candidate_test_scope_note='Final actual installed combination; separate author evidence retained by release.',
        root_cause_confirmed=True,
        root_cause_evidence=str(runtime_dir / 'verification/gen17_station_contact_current_wait_delivery/diagnosis.json'),
        failure_stage_confirmed='R5 registered current geometry crossed its deadline between repeated admission checks and was misclassified as invalid registration; typed bounded waits also cover final metadata freeze and remote issuance.',
        algorithm_root_cause_status='GEN17 waits for fresh independently registered geometry at repeated admission boundaries while preserving candidate and owner contracts; clock pairing and atomic current-record checks close independently reproduced boundary defects. Physical grasp remains unverified.',
        metadata_scope='GEN17 actual regression and bound compute reload passed. R5 remains a sealed physical failure.',
        source_manifest_scope='GEN17 reviewed source bundle; current disk SHA and compute reload receipt agree.',
        source_count_scope='All counts describe the complete release bundle including tests/resources, not all loaded modules.',
        deployment_scope='Three compute consumers reloaded; eight hardware/GUI process identities preserved.',
        next_action='Fresh current scene/SDK/recording preflight, then continue authorized recorded grasp.',
        pending_work=['Fresh scene, SDK and recording preflight', 'Recorded physical grasp with actual outcome retained'],
        last_completed_regression=dict(path=str(args.tests), sha256=sha(args.tests), summary=tested['summary'],
            test_scope_files=len(installed['test_scope'])),
        manual_mode=True, gui_manual_restored=True, manual_state_is_live_snapshot=False,
        verified_manual_state_evidence=str(manual_path), manual_restore_receipt=str(manual_path),
        manual_evidence=str(args.reload), manual_restore_attempt=str(manual_path.parent),
        manual_mode_scope='Manual mode checked by successful reload; fresh execution preflight still required.',
        physical_grasp_success=False, physical_grasp_verified=False, updated_ns=time.time_ns(),
        metadata_update_audit=str(args.output / 'result.json'))
    common['root_cause_evidence_sha256'] = sha(common['root_cause_evidence'])
    common['verified_manual_state_evidence_sha256'] = sha(common['verified_manual_state_evidence'])
    common['manual_evidence_sha256'] = sha(common['manual_evidence'])
    common['failure_stage_evidence'] = common['root_cause_evidence']
    common['failure_stage_evidence_sha256'] = common['root_cause_evidence_sha256']
    after = {}
    for index, p in enumerate(paths):
        value = deepcopy(states[str(p)])
        historical_keys = [key for key in value if key.startswith(
            ('three_owner_replay_', 'latest_three_owner_'))]
        historical = {key: value.pop(key) for key in historical_keys}
        value.setdefault('metadata_history', []).append(dict(recorded_ns=time.time_ns(),
            reason='GEN17 actual verified deployment replaces previous current software claims',
            historical_replay_fields=historical,
            backup=str(args.output / ('before_%d.json' % index)),
            backup_sha256=hashlib.sha256(before[str(p)]).hexdigest()))
        value.update(common)
        after[str(p)] = (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode()
    assert all(Path(name).read_bytes() == value for name, value in before.items())
    changed = []
    try:
        for name, value in after.items():
            p = Path(name)
            assert p.read_bytes() == before[name]
            temporary = p.with_name(p.name + '.gen17-reconcile.tmp')
            temporary.write_bytes(value)
            temporary.replace(p)
            changed.append(name)
    except BaseException:
        for name in reversed(changed):
            assert Path(name).read_bytes() == after[name]
            Path(name).write_bytes(before[name])
        raise
    save_new(args.output / 'result.json', dict(passed=True, deployment=str(args.deployment),
        deployment_sha256=sha(args.deployment), source_count=count,
        test_scope_files=len(installed['test_scope']), summary=tested['summary'],
        metadata_sha256={name: sha(name) for name in after}, motion_commands_sent=False))
    print(json.dumps(dict(passed=True, deployment=str(args.deployment), sha256=sha(args.deployment),
                         sources=count, generation=selection['generation'])))


if __name__ == '__main__':
    main()
