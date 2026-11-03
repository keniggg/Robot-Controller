"""Install only the complete reviewed GEN57 bytes with the ROS stack stopped.

This does not start services or authorize motion. Live feedback, GUI/manual,
camera and current remote-service acceptance are required after full startup.
"""
import argparse
import fcntl
import os
from pathlib import Path
import time

from gen57_cold_install_guard_20261020 import *
from gen57_offline_result_verifier_r2_20261020 import validate_result

CANDIDATE = W/'.runtime/gen57_candidate_r1'
RELEASE = N/'verification/gen57_release_manifest.json'
ALLOWED = frozenset((
    'src/alicia_grasp_modes/src/alicia_grasp_modes/source_connected_owner.py',
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'))
ADDED = frozenset('src/alicia_grasp_modes/tests/'+name for name in (
    'test_gen57_source_component_measurement.py',
    'test_gen57_source_component_pipeline.py', 'test_gen57_task_feedback_ingress.py'))
HELPERS = (Path(__file__).resolve(), W/'.runtime/gen57_cold_install_guard_20261020.py',
    W/'.runtime/gen57_offline_result_verifier_r2_20261020.py')


def reference(args, key):
    path = getattr(args, key).resolve(strict=True)
    require(sha(path) == getattr(args, key+'_sha256'), key+' input changed')
    return path


def review(path, source):
    value = read(path)
    require(value['passed'] is True and value['source_sha256'] == source
        and value['full_checks_retained'] is True
        and value['physical_grasp_verified'] is False, 'independent candidate review incomplete')
    require(all(sha(p) == h for p, h in value['dependency_sha256'].items()),
        'independent candidate review dependency changed')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('manifest', 'runner', 'focus', 'full', 'review', 'helpers_review'):
        parser.add_argument('--'+key.replace('_', '-'), type=Path, required=True)
        parser.add_argument('--'+key.replace('_', '-')+'-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    refs = {k: reference(args, k) for k in
        ('manifest', 'runner', 'focus', 'full', 'review', 'helpers_review')}
    storage(args.output)
    control = control_check()
    initial_cold = cold_pair()
    manifest, context, full, evidence = validate_result(refs['full'], refs['manifest'],
        args.manifest_sha256, refs['runner'], args.runner_sha256, 'full')
    focus_receipt = read(refs['focus'])
    focus_manifest, focus_context, focus, focus_evidence = validate_result(refs['focus'],
        focus_receipt['candidate_manifest_path'], focus_receipt['candidate_manifest_sha256'],
        refs['runner'], args.runner_sha256, 'focus')
    require(focus_context['sources'] == context['sources']
        and focus_manifest['focus_scope'] == manifest['focus_scope'],
        'focus and full candidate/scope differ')
    require(Path(manifest['workspace_root']).resolve(strict=True) == CANDIDATE,
        'unexpected candidate workspace')
    require(sha(BASE_RELEASE) == BASE_SHA and sha(PREVIOUS) == PREVIOUS_SHA,
        'historical baseline/deployment changed')
    baseline = read(BASE_RELEASE)['source_sha256']
    source = context['sources']
    require(len(baseline) == 1085 and len(source) == 1088
        and source == manifest['source_sha256'] and set(baseline) <= set(source),
        'complete frozen source map required')
    require(all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in source),
        'unsafe source path')
    changed = {p for p in baseline if baseline[p] != source[p]}
    added = set(source)-set(baseline)
    require(changed == ALLOWED == set(manifest['allowed_production_paths'])
        and added == ADDED, 'unreviewed source delta')
    require(full['summary'] == dict(tests=len(context['case_identities']), failures=0, errors=0, skipped=0)
        and full['test_scope_files'] == len(context['scope']), 'complete frozen case scope required')
    reviewed = review(refs['review'], source)
    helpers = read(refs['helpers_review'])
    require(helpers['passed'] is True and helpers['cold_installation_reviewed'] is True
        and all(sha(p) == h for p, h in helpers['dependency_sha256'].items())
        and all(str(p) in helpers['dependency_sha256'] for p in HELPERS),
        'independent cold installer review incomplete')
    require(all(sha(W/p) == h for p, h in baseline.items()), 'live baseline differs')
    require(all(not (W/p).exists() for p in added), 'new source destination already exists')
    require(all(sha(CANDIDATE/p) == h for p, h in source.items()), 'tested candidate changed')
    evidence.update(focus_evidence)
    evidence.update(reviewed['dependency_sha256'])
    evidence.update(helpers['dependency_sha256'])
    evidence.update({str(p): sha(p) for p in refs.values()})
    evidence.update({str(BASE_RELEASE): BASE_SHA, str(PREVIOUS): PREVIOUS_SHA,
        str(CONTROL_SNAPSHOT): CONTROL_SHA, str(REQUEST): REQUEST_SHA})
    for filename, digest in evidence.items():
        path = Path(filename).resolve(strict=True)
        require(sha(path) == digest, 'release evidence already differs: '+filename)
        try:
            relative = str(path.relative_to(W))
        except ValueError:
            relative = None
        require(relative not in changed or digest == source[relative],
            'immutable review points to a soon-replaced baseline source: '+filename)
    output = args.output.resolve()
    require(not output.exists() and not RELEASE.exists(), 'installation/release already exists')
    pinned = {str(p): sha(p) for p in (*refs.values(), BASE_RELEASE, PREVIOUS,
        CONTROL, CONTROL_SNAPSHOT, REQUEST, *HELPERS)}
    output.mkdir()
    write(output/'plan.json', dict(changed_paths=sorted(changed), added_paths=sorted(added),
        baseline_sha256=baseline, source_sha256=source, input_sha256=pinned,
        initial_cold_scan=initial_cold, full_result=str(refs['full']),
        full_result_sha256=args.full_sha256, recorded_ns=time.time_ns(),
        runtime_reloaded=False, live_feedback_verified=False, motion_commands_sent=False))
    for name in sorted(changed):
        backup = output/'backup_sources'/name
        backup.parent.mkdir(parents=True, exist_ok=True)
        with backup.open('xb') as stream:
            stream.write((W/name).read_bytes()); stream.flush(); os.fsync(stream.fileno())
        require(sha(backup) == baseline[name], 'baseline backup differs')
    installed = []
    try:
        with (CONTROL.parent/'attempt.lock').open('a') as attempt_lock, \
             (CONTROL.parent/'control.lock').open('a') as control_lock:
            fcntl.flock(attempt_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(control_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            require(control_check() == control and all(sha(p) == h for p, h in pinned.items()),
                'installation input/control changed')
            before = cold_pair()
            require(before['after']['boot_id'] == initial_cold['after']['boot_id'],
                'boot changed since preflight')
            require(all(sha(W/p) == h for p, h in baseline.items()), 'baseline changed before write')
            for name in sorted(changed | added):
                cold_scan()
                target = W/name
                temporary = target.with_name(target.name+'.gen57.cold.install.tmp')
                raw = (CANDIDATE/name).read_bytes()
                require(hashlib.sha256(raw).hexdigest() == source[name] and not temporary.exists(),
                    'candidate changed or temporary destination exists')
                require((name in changed and sha(target) == baseline[name])
                    or (name in added and not target.exists()), 'source destination changed')
                target.parent.mkdir(parents=True, exist_ok=True)
                mode = target.stat().st_mode & 0o777 if target.exists() else 0o644
                with temporary.open('xb') as stream:
                    stream.write(raw); stream.flush(); os.fsync(stream.fileno())
                os.chmod(temporary, mode)
                os.replace(temporary, target)
                descriptor = os.open(str(target.parent), os.O_DIRECTORY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
                require(sha(target) == source[name], 'installed read-back differs')
                installed.append(name)
                write(output/('progress_%d.json' % len(installed)), dict(installed=installed,
                    complete=False, runtime_reloaded=False, recorded_ns=time.time_ns()))
            after = cold_pair()
            require(after['after']['boot_id'] == before['after']['boot_id']
                and control_check() == control, 'cold/control state changed during install')
            require(all(sha(W/p) == h and sha(CANDIDATE/p) == h for p, h in source.items()),
                'installed/tested source bytes differ')
            # Mutable live control is checked above, never made immutable evidence.
            require(all(sha(p) == h for p, h in evidence.items()), 'release evidence changed')
            installation = dict(passed=True, installed_paths=installed, source_sha256=source,
                candidate_manifest=str(refs['manifest']), candidate_manifest_sha256=args.manifest_sha256,
                full_regression_passed=True, full_regression_location=str(CANDIDATE),
                installed_bytes_exact_tested_candidate=True, cold_before=before, cold_after=after,
                recorded_ns=time.time_ns(), runtime_reloaded=False, manual_mode_verified=False,
                live_feedback_verified=False, motion_commands_sent=False, temperature_queried=False,
                new_full_startup_acceptance_required=True)
            write(output/'result.json', installation)
            evidence[str(output/'result.json')] = sha(output/'result.json')
            release = dict(generation='GEN57', passed=True, reviewed=True, installed=True,
                runtime_deployed=False, physical_grasp_verified=False, baseline_release_sha256=BASE_SHA,
                previous_deployment_sha256=PREVIOUS_SHA, source_sha256=source, source_count=len(source),
                test_scope_files=full['test_scope_files'], test_summary=full['summary'],
                historical_test_files=339, historical_tests=9658, actual_regression=str(refs['full']),
                actual_regression_sha256=args.full_sha256, candidate_manifest=str(refs['manifest']),
                candidate_manifest_sha256=args.manifest_sha256, runner=str(refs['runner']),
                runner_sha256=args.runner_sha256, worker_count=full['worker_count'],
                independent_review=str(refs['review']), independent_review_sha256=args.review_sha256,
                evidence_sha256=evidence, full_regression_location=str(CANDIDATE),
                installed_bytes_exact_tested_candidate=True, installation=str(output/'result.json'),
                installation_sha256=sha(output/'result.json'), installation_kind='stopped_stack_file_only',
                new_fresh_scene_robot_and_path_acceptance_required=True,
                recorded_ns=time.time_ns(), motion_commands_sent=False)
            write(RELEASE, release)
            require(read(RELEASE) == release, 'release read-back differs')
    except BaseException as exc:
        write(output/'installation_failure.json', dict(error=repr(exc), installed_paths=installed,
            release_exists=RELEASE.exists(), runtime_reloaded=False, recorded_ns=time.time_ns()))
        raise
    print(json.dumps(dict(installed=True, sealed=True, release=str(RELEASE),
        sha256=sha(RELEASE), summary=full['summary']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
