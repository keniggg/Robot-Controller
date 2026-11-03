"""Install exact fully tested GEN59 bytes while the unchanged GEN58 runtime is idle.

Default creates a reviewable plan only. --install requires that exact plan,
independent helper review and fresh read-only readiness. No ROS, device, network,
motion or runtime reload calls occur here. Existing receipts are never replaced.
"""
import argparse
from collections import Counter
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
O = N/'gen58_resume_r1/observation_pose_fix'
BASE = N/'verification/gen58_release_manifest.json'
BASE_SHA = '03bf9e669329f804921982564f730572e7758ef81c9e9d624a24cfb3700b333e'
PREVIOUS = N/'gen58_resume_r1/full_restart_r1/acceptance/result.json'
PREVIOUS_SHA = '3cfda4096fb1590cafaba2f8df525b1579120c950f1a9fbdb9ecada67888eaf2'
DESTINATION = N/'verification/gen59_release_manifest.json'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
C = W/'.runtime/gen59_observation_pose_candidate_r3'
RUNNER = W/'.runtime/verify_gen59_observation_pose_r3_20261022.py'
PRODUCTION = {
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py',
}
CONTRACTS = {
    'src/alicia_grasp_modes/tests/test_observation_fk_first_strict.py',
    'src/alicia_grasp_modes/tests/test_gen8_missing_side_distance_binding.py',
    'src/alicia_grasp_modes/tests/test_gen8_precompute_gen20_equivalence.py',
    'src/alicia_grasp_modes/tests/test_reachable_observation_early_qualified.py',
}
ADDED = {
    'src/alicia_grasp_modes/tests/test_observation_first_fk_comparison.py',
    'src/alicia_grasp_modes/tests/test_observation_shared_band_gain.py',
    'src/alicia_grasp_modes/tests/fixtures/gen58_observation_shared_band_gain.json',
    'src/alicia_grasp_modes/tests/fixtures/gen59_fk_source.json',
    'src/alicia_grasp_modes/tests/fixtures/gen59_fk_source.npz',
}


def require(value, message):
    if not value:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def storage(path):
    path = Path(path).resolve()
    require(M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY, 'writable SanStand required')
    require(N in path.parents, 'output outside experiment storage')
    parent = path.parent
    while not parent.exists():
        parent = parent.parent
    require(parent.stat().st_dev == M.stat().st_dev, 'output must remain on SanStand')
    config = read(E/'grasp_records/archive_config.json')
    require(config['upload_enabled'] is False and config['delete_enabled'] is False
            and read(E/'maintenance/storage_control.json')['archive_policy'] == 'local_only', 'storage policy differs')
    stat = os.statvfs(E)
    require(stat.f_bavail*stat.f_frsize >= config['min_free_bytes'], 'insufficient configured free space')


def save(path, data):
    storage(path)
    raw = (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with Path(path).open('xb') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    require(Path(path).read_bytes() == raw, 'receipt read-back differs')


def inventory(path):
    root = ET.parse(path).getroot()
    summary = {k: sum(int(s.get(k, 0)) for s in root.iter('testsuite'))
               for k in ('tests', 'failures', 'errors', 'skipped')}
    cases = [(c.get('file', ''), c.get('classname', ''), c.get('name', '')) for c in root.iter('testcase')]
    require(summary['tests'] == len(cases) == len(set(cases)), 'XML cases missing or repeated')
    for key, tag in (('failures', 'failure'), ('errors', 'error'), ('skipped', 'skipped')):
        require(summary[key] == sum(len(c.findall(tag)) for c in root.iter('testcase')), 'XML result counts differ')
    return summary, cases


def validate(manifest_path, full_path, review_path):
    refs = {}
    def bind(path, expected=None):
        path = Path(path).resolve(strict=True); digest = sha(path)
        require(expected is None or digest == expected, 'dependency changed: '+str(path))
        require(str(path) not in refs or refs[str(path)] == digest, 'conflicting dependency identity')
        refs[str(path)] = digest
        return read(path) if path.suffix == '.json' else path
    base = bind(BASE, BASE_SHA); bind(PREVIOUS, PREVIOUS_SHA)
    manifest = bind(manifest_path); full = bind(full_path); review = bind(review_path)
    require(manifest['kind'] == 'GEN59_observation_pose_frozen_offline_candidate'
            and manifest['full_scope_ready'] and Path(manifest['workspace_root']) == C, 'wrong candidate manifest')
    sources = manifest['source_sha256']; old = base['source_sha256']; context = manifest['context']
    require(len(old) == 1089 and len(sources) == 1094 and set(sources)-set(old) == ADDED, 'source scope changed')
    require({p for p in old if old[p] != sources[p]} == PRODUCTION | CONTRACTS, 'unreviewed source delta')
    require(all(not Path(p).is_absolute() and '..' not in Path(p).parts for p in sources), 'unsafe source path')
    require(all(sha(W/p) == h for p,h in old.items()) and all(not (W/p).exists() for p in ADDED), 'active baseline changed')
    require(sources == context['sources'] == full['source_sha256'] == review['source_sha256'], 'source bindings differ')
    require(all(sha(C/p) == h for p,h in manifest['copied_input_sha256'].items()), 'candidate input changed')
    bind(RUNNER, manifest['runner_sha256'])
    for p,h in context['compatibility_dependencies'].items():
        bind(p,h)
    for p,h in context['generated_imports'].items():
        bind(p,h)
    require(review['passed'] is True and review['full_checks_retained'] is True
            and review['physical_grasp_verified'] is False, 'patch review incomplete')
    require(set(review['reviewed_changed_paths']) == PRODUCTION | CONTRACTS | ADDED, 'patch review scope differs')
    for p,h in review['dependency_sha256'].items():
        bind(p,h)
    require(full['mode'] == 'full' and full['candidate_manifest_path'] == str(manifest_path)
            and full['candidate_manifest_sha256'] == sha(manifest_path), 'full run manifest binding differs')
    flags = ('passed','full_regression_passed','full_regression_scope','scope_complete','source_unchanged',
             'test_bodies_executed','combined_xml_matches_worker_results')
    require(all(full[k] is True for k in flags), 'full regression incomplete')
    empty = ('external_boundary_attempts','foreign_or_inconsistent_imports','duplicate_case_identities',
             'missing_case_identities','unexpected_case_identities','missing_worker_results','residual_offline_process_groups')
    require(all(not full[k] for k in empty) and not full['interrupted'] and full['launcher_error'] is None
            and not full['offline_child_cleanup_required'], 'full offline audit failed')
    require(full['worker_count'] <= 6 and full['worker_returncodes'] == [0]*full['worker_count'], 'workers failed')
    require(full['runner_sha256'] == sha(RUNNER) and all(full['loaded_changed_sources_global'][p] for p in PRODUCTION), 'changed source was not tested')
    plan = bind(full_path.parent/'plan.json', full['plan_sha256'])
    require(plan['context'] == context, 'full plan source/case contract differs')
    summary, cases = inventory(bind(full_path.parent/'pytest.xml', full['xml_sha256']))
    expected = list(map(tuple, context['case_identities']))
    historical = list(map(tuple, context['old_case_identities']))
    require(len(historical) == len(set(historical)) == 9893 and set(historical) <= set(expected), 'historical cases lost')
    require(sorted(cases) == sorted(expected) and summary == full['summary'] and summary['tests'] > 9893
            and all(summary[k] == 0 for k in ('failures','errors','skipped')), 'full case union/results differ')
    require(full['test_scope_files'] == 350 and full['source_count'] == 1094
            and dict(Counter(c[0] for c in cases)) == full['test_case_counts_by_file'], 'full scope counts differ')
    union = []
    worker_indices = set()
    for name,digest in full['worker_result_sha256'].items():
        worker = bind(name,digest); folder = Path(name).parent
        require(worker['passed'] and worker['source_unchanged'] and worker['scope_complete']
                and worker['test_bodies_executed'] and worker['returncode'] == 0
                and worker['plan_sha256'] == full['plan_sha256'], 'worker audit failed')
        require(worker['worker_index'] not in worker_indices, 'duplicate worker index')
        worker_indices.add(worker['worker_index'])
        worker_summary, worker_cases = inventory(bind(folder/'pytest.xml',worker['xml_sha256']))
        require(worker_summary == worker['summary'] and sorted(worker_cases) == sorted(map(tuple,worker['case_identities'])), 'worker summary differs')
        union.extend(worker_cases)
        bind(folder/'pytest.log',worker['log_sha256'])
        for name,digest in worker['generated_bootstrap_sha256'].items():
            bind(folder/name,digest)
        provenance = bind(folder/'module_provenance.json',worker['module_provenance_sha256'])
        for item in provenance.values():
            bind(item['path'],item['sha256'])
    require(worker_indices == set(range(full['worker_count'])) and sorted(union) == sorted(expected)
            and len(union) == len(set(union)), 'worker partition differs')
    for path,digest in plan['focus_evidence_sha256'].items():
        bind(path,digest)
    require(all(sha(p) == h for p,h in refs.items()), 'evidence changed during validation')
    return dict(source_sha256=sources, baseline_source_sha256=old, evidence_sha256=refs,
        test_summary=summary, source_count=1094, test_scope_files=350,
        new_case_count=len(cases)-9893, full_elapsed_sec=full['elapsed_sec'], worker_count=full['worker_count'])


def identities_unchanged(rows):
    for row in rows.values():
        fields = (Path('/proc')/str(row['pid'])/'stat').read_text().rsplit(')',1)[1].split()
        require(fields[0] != 'Z' and int(fields[19]) == int(row['start_ticks'])
                and int(fields[1]) == int(row['parent_pid']), 'accepted runtime process changed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', action='store_true')
    for name in ('manifest','full','review','output','plan','helper-review','readiness'):
        parser.add_argument('--'+name,type=Path)
    for name in ('plan','helper-review','readiness'):
        parser.add_argument('--'+name+'-sha256')
    args = parser.parse_args()
    if not args.install:
        require(all((args.manifest,args.full,args.review,args.output)), 'plan inputs required')
        output = args.output.resolve(); storage(output)
        require(O in output.parents and not output.exists() and not DESTINATION.exists(), 'plan destination exists or differs')
        refs = [p.resolve(strict=True) for p in (args.manifest,args.full,args.review)]
        validation = validate(*refs); output.mkdir()
        save(output/'plan.json',dict(kind='GEN59_file_only_install_plan',created_ns=time.time_ns(),
            helper_sha256=sha(__file__),manifest=str(refs[0]),full=str(refs[1]),review=str(refs[2]),
            validation=validation,destination=str(DESTINATION),changed_paths=sorted(PRODUCTION | CONTRACTS | ADDED),
            runtime_reloaded=False,motion_commands_sent=False))
        print(json.dumps(dict(plan=str(output/'plan.json'),sha256=sha(output/'plan.json')))); return
    require(all((args.plan,args.plan_sha256,args.helper_review,args.helper_review_sha256,args.readiness,args.readiness_sha256)), 'exact plan/review/readiness required')
    for path,digest in ((args.plan,args.plan_sha256),(args.helper_review,args.helper_review_sha256),(args.readiness,args.readiness_sha256)):
        require(O in path.resolve().parents and sha(path) == digest, 'installation reference differs')
    planned = read(args.plan); helper_review = read(args.helper_review); ready = read(args.readiness)
    require(planned['kind'] == 'GEN59_file_only_install_plan' and planned['helper_sha256'] == sha(__file__)
            and planned['destination'] == str(DESTINATION), 'plan helper/destination changed')
    require(helper_review['passed'] is True and helper_review['helper_sha256'] == sha(__file__)
            and helper_review['plan_sha256'] == args.plan_sha256, 'independent helper review missing')
    validation = validate(*map(Path,(planned['manifest'],planned['full'],planned['review'])))
    require(validation == planned['validation'], 'validation changed since plan')
    previous = read(PREVIOUS); rows = previous['all_registered_nodes']; control = read(CONTROL)
    remote_name = '/remote_grasp6d_node'
    preserved = {k:v for k,v in rows.items() if k != remote_name}
    require(len(preserved) == 16 and ready['kind'] == 'GEN59_remote_stopped_install_readiness'
            and all(ready[k] is True for k in ('remote_stopped','old_remote_exited',
                'remote_registration_absent','remote_processes_absent'))
            and ready['preserved_node_identities'] == preserved
            and ready['old_remote_identity'] == rows[remote_name], 'remote must be stopped before replacing lazy imports')
    require(len(rows) == 17 and ready['passed'] is True and ready['manual_mode'] is True
            and ready['task_active'] is False and ready['actions_active'] is False, 'manual idle readiness required')
    require(ready['node_identities'] == rows and ready['epoch_ns'] == previous['epoch_ns']
            and ready['selection']['generation'] == 2 and ready['actuation_status'].startswith('CONFIRMED:')
            and ready['sample_count'] >= 20 and 0 <= time.time_ns()-ready['recorded_ns'] < 120_000_000_000, 'fresh current-instance readiness required')
    require(ready['control_snapshot'] == control and ready['control_sha256'] == sha(CONTROL)
            and control['state'] == 'active' and control['revision'] == 67, 'continuation authorization changed')
    require(ready['release_sha256'] == BASE_SHA and ready['deployment_sha256'] == PREVIOUS_SHA
            and ready['helper_sha256'] == sha(ready['helper_path']), 'readiness source binding changed')
    require(ready['candidate_source_sha256'] == validation['source_sha256']
            and ready['full_regression_sha256'] == sha(planned['full'])
            and ready['independent_review_sha256'] == sha(planned['review']), 'stopped readiness candidate differs')
    for name,digest in ready['input_sha256'].items():
        require(sha(name) == digest, 'readiness dependency changed')
    identities_unchanged(preserved)
    remote_proc = Path('/proc')/str(rows[remote_name]['pid'])/'stat'
    if remote_proc.exists():
        fields = remote_proc.read_text().rsplit(')',1)[1].split()
        require(fields[0] == 'Z' or int(fields[19]) != int(rows[remote_name]['start_ticks']), 'old remote still running')
    output = args.plan.parent/'installation'; storage(output)
    require(not output.exists() and not DESTINATION.exists(), 'installation destination exists')
    output.mkdir(); sources = validation['source_sha256']; old = validation['baseline_source_sha256']
    for name in sorted(PRODUCTION | CONTRACTS):
        target = output/'backup_sources'/name; target.parent.mkdir(parents=True,exist_ok=True)
        with target.open('xb') as stream:
            stream.write((W/name).read_bytes()); stream.flush(); os.fsync(stream.fileno())
        require(sha(target) == old[name], 'baseline backup differs')
    with (CONTROL.parent/'control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(read(CONTROL) == control and all(sha(p) == h for p,h in validation['evidence_sha256'].items()), 'installation inputs changed')
        require(all(sha(W/p) == h for p,h in old.items()), 'baseline changed before install')
        identities_unchanged(preserved)
        for index,name in enumerate(planned['changed_paths']):
            target = W/name; temporary = target.with_name(target.name+'.gen59.install.tmp')
            require(not target.is_symlink() and not temporary.exists() and sha(C/name) == sources[name], 'source destination changed')
            require((name in old and sha(target) == old[name]) or (name in ADDED and not target.exists()), 'unexpected destination bytes')
            mode = target.stat().st_mode & 0o777 if target.exists() else 0o644
            with temporary.open('xb') as stream:
                stream.write((C/name).read_bytes()); stream.flush(); os.fsync(stream.fileno())
            require(sha(temporary) == sources[name], 'temporary source differs')
            os.chmod(temporary,mode); os.replace(temporary,target)
            require(sha(target) == sources[name], 'installed read-back differs')
            save(output/('installed_%02d.json'%index),dict(path=name,sha256=sources[name],recorded_ns=time.time_ns()))
        require(all(sha(W/p) == h == sha(C/p) for p,h in sources.items()), 'installed candidate differs')
        require(read(CONTROL) == control, 'control changed during installation')
        identities_unchanged(preserved)
        evidence = dict(validation['evidence_sha256'])
        evidence.update({str(p.resolve()):sha(p) for p in (args.plan,args.helper_review,args.readiness,Path(__file__))})
        release = dict(generation='GEN59',kind='GEN59_observation_pose_reviewed_source_release',
            passed=True,reviewed=True,installed=True,runtime_deployed=False,physical_grasp_verified=False,
            source_sha256=sources,source_count=1094,test_scope_files=350,test_summary=validation['test_summary'],
            full_regression_passed=True,actual_regression=planned['full'],actual_regression_sha256=sha(planned['full']),
            candidate_manifest=planned['manifest'],candidate_manifest_sha256=sha(planned['manifest']),
            runner=str(RUNNER),runner_sha256=sha(RUNNER),worker_count=validation['worker_count'],
            independent_review=planned['review'],independent_review_sha256=sha(planned['review']),
            baseline_release=str(BASE),baseline_release_sha256=BASE_SHA,previous_deployment=str(PREVIOUS),
            previous_deployment_sha256=PREVIOUS_SHA,evidence_sha256=evidence,
            historical_tests_retained=9893,new_tests=validation['new_case_count'],
            installed_bytes_exact_tested_candidate=True,installed_paths=planned['changed_paths'],
            installation=str(output),recorded_ns=time.time_ns(),motion_commands_sent=False,temperature_queried=False,
            new_fresh_scene_robot_and_path_acceptance_required=True)
        save(DESTINATION,release)
        save(output/'result.json',dict(passed=True,release=str(DESTINATION),release_sha256=sha(DESTINATION),
            source_sha256=sources,runtime_reloaded=False,motion_commands_sent=False,recorded_ns=time.time_ns()))
    print(json.dumps(dict(installed=True,release=str(DESTINATION),sha256=sha(DESTINATION),runtime_reloaded=False)))


if __name__ == '__main__':
    main()
