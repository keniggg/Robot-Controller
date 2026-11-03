"""Explicit GEN55 disk installation / release sealing. No ROS or motion calls."""
import argparse
import fcntl
import os
from pathlib import Path
import time

from gen55_idle_reload_common_20261019 import *

V = N/'verification'
BASE_RELEASE = V/'gen54_release_manifest.json'
PRODUCTION = frozenset('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'+name
    for name in ('observation_path_guard.py','contact_sequence_precheck.py','held_pregrasp_selection.py'))
OLD_TESTS = frozenset('src/alicia_flexible_grasp_supervisor/tests/'+name for name in
    ('test_gen54_collision_contract.py', 'test_gen52_contact_terminal_collision_screen.py',
     'test_contact_sequence_precheck.py', 'test_gen38_contact_replay_staging.py'))
READINESS = W/'.runtime/read_gen55_preinstall_readiness_20261019.py'
PRIOR_INSTALLED_MANIFEST = V/'gen55_source_installation_r1/installed_candidate.json'
PRIOR_INSTALLED_MANIFEST_SHA = '38af05487563d1b19c539fa298bfba09b17b166c9c74dce7035419b7ad96deed'
PRIOR_INSTALLATION_RESULT = V/'gen55_source_installation_r1/result.json'
PRIOR_INSTALLATION_RESULT_SHA = '5805ad6a13fd5bffd95fcfac6f4197c583fd50e2d79a7a0c6dd99ba163905dbb'
NEW_MODULE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/finger_cad_separation.py'
NEW_TESTS = frozenset('src/alicia_flexible_grasp_supervisor/tests/'+name for name in
    ('test_gen55_finger_cad_separation.py','test_gen55_contact_cad_replay.py','test_gen55_held_pregrasp_reuse.py'))
NEW_FIXTURES = frozenset('src/alicia_flexible_grasp_supervisor/tests/fixtures/'+name for name in
    ('gen55_g54_held_pregrasp_inputs.json.gz','gen55_finger_cad_actual_receipts.json','gen55_gen54_contact_curves.json'))


def reference(args, name):
    path = getattr(args,name).resolve(strict=True)
    require(sha(path) == getattr(args,name+'_sha256'), name+' input changed')
    return path


def check_delta(manifest):
    require(sha(BASE_RELEASE) == BASE_SHA, 'GEN54 release changed')
    baseline = read(BASE_RELEASE)['source_sha256']; source = manifest['source_sha256']
    require(len(baseline) == 1075 and set(baseline) <= set(source), 'GEN54 source scope lost')
    changed = {p for p in baseline if source[p] != baseline[p]}
    added = set(source)-set(baseline)
    require(changed == PRODUCTION|OLD_TESTS and added == {NEW_MODULE}|NEW_TESTS|NEW_FIXTURES,
        'unreviewed GEN55 source delta')
    require(all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in source), 'unsafe source path')
    return baseline, changed, added


def verify_retained_additions(manifest, added):
    require(sha(PRIOR_INSTALLED_MANIFEST) == PRIOR_INSTALLED_MANIFEST_SHA and
        sha(PRIOR_INSTALLATION_RESULT) == PRIOR_INSTALLATION_RESULT_SHA,
        'prior installed GEN55 evidence changed')
    prior, installed = read(PRIOR_INSTALLED_MANIFEST), read(PRIOR_INSTALLATION_RESULT)
    require(installed['passed'] is True and len(prior['source_sha256']) == 1082 and
        installed['source_sha256'] == prior['source_sha256'] and
        Path(installed['installed_manifest']).resolve(strict=True) == PRIOR_INSTALLED_MANIFEST and
        installed['installed_manifest_sha256'] == PRIOR_INSTALLED_MANIFEST_SHA,
        'prior installed source binding differs')
    require(added == {NEW_MODULE}|NEW_TESTS|NEW_FIXTURES and all(
        prior['source_sha256'].get(p) == manifest['source_sha256'][p] and
        (W/p).is_file() and sha(W/p) == manifest['source_sha256'][p] for p in added),
        'retained seven additions differ from prior installed and candidate bytes')
    return sorted(added)


def verify_review(path, manifest):
    review = read(path)
    require(review['passed'] is True and review['source_sha256'] == manifest['source_sha256'] and
        review['physical_grasp_verified'] is False and review['full_checks_retained'] is True,
        'independent candidate review is incomplete')
    for name,digest in review['dependency_sha256'].items():
        require(sha(name) == digest, 'review dependency changed')
    return review


def install(args, refs):
    manifest, context, focus, evidence = validate_result(refs['focus'], refs['manifest'],
        args.manifest_sha256, refs['runner'], args.runner_sha256, 'focus')
    candidate = Path(manifest['workspace_root']).resolve(strict=True)
    require(candidate == W/'.runtime/gen55_candidate_r1', 'unexpected GEN55 candidate root')
    baseline, changed, added = check_delta(manifest)
    review = verify_review(refs['review'],manifest)
    ready = read(refs['readiness'])
    previous, processes = validate_previous(PREVIOUS)
    require(ready['passed'] is True and ready['manual_mode'] is True and ready['task_active'] is False and
        ready['actions_active'] is False and ready['node_identities'] == processes and
        ready['deployment_processes_unchanged'] is True and ready['source_matches_operational_release'] is True and
        ready['source_count'] == len(baseline) and ready['sample_count'] >= 20 and
        ready['epoch_ns'] == previous['epoch_ns'] and ready['selection'] == previous['selection'] and
        Path(ready['release']).resolve(strict=True) == BASE_RELEASE and
        Path(ready['deployment']).resolve(strict=True) == PREVIOUS and
        0 <= time.time_ns()-ready['recorded_ns'] < 120_000_000_000,
        'fresh accepted GEN54 manual idle readiness required')
    require(ready['helper_sha256'] == sha(READINESS) and
        ready['release_sha256'] == BASE_SHA and ready['deployment_sha256'] == PREVIOUS_SHA and
        ready['control_path'] == str(CONTROL) and ready['control_sha256'] == CONTROL_EXPECTED_SHA and
        ready['control_snapshot'] == validate_control(CONTROL), 'readiness evidence binding differs')
    require(all(process(row['pid']) == row for row in processes.values()), 'runtime process changed')
    require(all(sha(W/p) == h for p,h in baseline.items()), 'production baseline changed')
    retained = verify_retained_additions(manifest, added)
    require(all(sha(candidate/p) == h for p,h in context['sources'].items()), 'candidate source changed')
    output = args.output.resolve(); storage(output); require(not output.exists(), 'output exists')
    pinned = {str(p):sha(p) for p in refs.values()}
    pinned.update({str(BASE_RELEASE):BASE_SHA,str(PREVIOUS):PREVIOUS_SHA,str(CONTROL):CONTROL_EXPECTED_SHA,
        str(READINESS):sha(READINESS), str(PRIOR_INSTALLED_MANIFEST):PRIOR_INSTALLED_MANIFEST_SHA,
        str(PRIOR_INSTALLATION_RESULT):PRIOR_INSTALLATION_RESULT_SHA})
    pinned.update({str(W/p):manifest['source_sha256'][p] for p in retained})
    output.mkdir()
    control = validate_control(CONTROL)
    write(output/'plan.json',dict(changed_paths=sorted(changed),retained_existing_added_paths=retained,baseline_sha256=baseline,
        source_sha256=context['sources'],input_sha256=pinned,created_ns=time.time_ns(),
        runtime_reloaded=False,motion_commands_sent=False))
    for name in sorted(changed):
        backup = output/'backup_sources'/name; backup.parent.mkdir(parents=True,exist_ok=True)
        raw = (W/name).read_bytes()
        with backup.open('xb') as stream: stream.write(raw);stream.flush();os.fsync(stream.fileno())
        require(backup.read_bytes() == raw and sha(backup) == baseline[name], 'source backup mismatch')
    # No ROS waits under this lock. Pause either wins before the write set or
    # prevents later deployment; installation itself never releases a task.
    with (CONTROL.parent/'control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(validate_control(CONTROL) == control and
            all(sha(p) == h for p,h in pinned.items()), 'installation inputs changed')
        require(all(process(row['pid']) == row for row in processes.values()), 'runtime changed before install')
        installed=[]
        for name in sorted(changed):
            target=W/name; temporary=target.with_name(target.name+'.gen55.install.tmp')
            raw=(candidate/name).read_bytes()
            require(hashlib.sha256(raw).hexdigest() == context['sources'][name], 'candidate changed during install')
            require(not temporary.exists() and name in changed and
                sha(target) == baseline[name], 'source destination changed')
            mode=(target.stat().st_mode & 0o777) if target.exists() else 0o644
            with temporary.open('xb') as stream: stream.write(raw);stream.flush();os.fsync(stream.fileno())
            os.chmod(temporary,mode);os.replace(temporary,target)
            require(sha(target) == context['sources'][name], 'installed source read-back differs')
            installed.append(name)
            write(output/'progress.json',dict(installed=installed,complete=False,runtime_reloaded=False))
        require(all(sha(W/p) == h for p,h in context['sources'].items()), 'installed sources differ')
        bound=dict(manifest,workspace_root=str(W),installed_from_manifest=str(refs['manifest']),
            installed_from_manifest_sha256=args.manifest_sha256)
        write(output/'installed_candidate.json',bound)
        write(output/'result.json',dict(passed=True,source_sha256=context['sources'],installed_paths=installed,
            retained_existing_added_paths=retained,retained_additions_rewritten=False,
            input_sha256=pinned,installed_manifest=str(output/'installed_candidate.json'),
            installed_manifest_sha256=sha(output/'installed_candidate.json'),recorded_ns=time.time_ns(),
            full_regression_passed=False,runtime_reloaded=False,motion_commands_sent=False))
    print(json.dumps(dict(installed=True,output=str(output),source_count=len(context['sources']),
        manifest_sha256=sha(output/'installed_candidate.json'))),flush=True)


def seal(args, refs):
    manifest, context, full, evidence = validate_result(refs['full'], refs['manifest'],
        args.manifest_sha256, refs['runner'], args.runner_sha256, 'full')
    require(Path(manifest['workspace_root']).resolve(strict=True) == W, 'full regression must run installed source')
    check_delta(manifest); review=verify_review(refs['review'],manifest)
    helpers=read(refs['helpers_review'])
    require(helpers['passed'] is True and all(sha(p) == h for p,h in helpers['dependency_sha256'].items()),
        'runtime helper independent review is incomplete')
    require(all(str(p) in helpers['dependency_sha256'] for p in HELPERS), 'review omitted runtime helpers')
    evidence.update(review['dependency_sha256']);evidence.update(helpers['dependency_sha256'])
    evidence.update({str(p):sha(p) for p in refs.values()})
    for p in (BASE_RELEASE,PREVIOUS,Path(__file__),W/'.runtime/finalize_gen55_idle_deployment_20261019.py',*HELPERS):
        evidence[str(p)]=sha(p)
    validate_previous(PREVIOUS);validate_control(CONTROL)
    result=dict(generation='GEN55',passed=True,reviewed=True,installed=True,runtime_deployed=False,
        physical_grasp_verified=False,baseline_release_sha256=BASE_SHA,previous_deployment_sha256=PREVIOUS_SHA,
        source_sha256=context['sources'],source_count=len(context['sources']),test_scope_files=full['test_scope_files'],
        test_summary=full['summary'],historical_test_files=329,historical_tests=9255,
        actual_regression=str(refs['full']),actual_regression_sha256=args.full_sha256,
        candidate_manifest=str(refs['manifest']),candidate_manifest_sha256=args.manifest_sha256,
        runner=str(refs['runner']),runner_sha256=args.runner_sha256,worker_count=full['worker_count'],
        independent_review=str(refs['review']),independent_review_sha256=args.review_sha256,
        evidence_sha256=evidence,new_fresh_scene_robot_and_path_acceptance_required=True,
        recorded_ns=time.time_ns(),motion_commands_sent=False)
    output=args.output.resolve();require(output == V/'gen55_release_manifest.json' and not output.exists(), 'release path exists/differs')
    validate_sources(result);write(output,result);require(read(output)==result,'release read-back differs')
    print(json.dumps(dict(sealed=True,path=str(output),sha256=sha(output),summary=full['summary'])),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('install','seal'))
    for name in ('manifest','runner','review','focus','readiness','full','helpers_review'):
        parser.add_argument('--'+name.replace('_','-'),type=Path)
        parser.add_argument('--'+name.replace('_','-')+'-sha256')
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    names=['manifest','runner','review']+(['focus','readiness'] if args.action=='install' else ['full','helpers_review'])
    refs={n:reference(args,n) for n in names}
    (install if args.action=='install' else seal)(args,refs)


if __name__ == '__main__':
    main()
