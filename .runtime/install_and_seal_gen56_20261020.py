"""Install an exactly reviewed, fully tested GEN56 candidate while manual/idle.

This file-only writer never starts or stops a node. Full regression runs in the
isolated candidate first. Every installed byte is then matched to that result;
the tested candidate and all GEN55 evidence remain immutable and available.
"""
import argparse
import fcntl
import os
from pathlib import Path
import time

from gen56_idle_reload_common_20261020 import *

BASE_RELEASE = N/'verification/gen55_release_manifest.json'
CANDIDATE = W/'.runtime/gen56_candidate_r1'
RELEASE = N/'verification/gen56_release_manifest.json'
READINESS = W/'.runtime/read_gen56_preinstall_readiness_20261020.py'
ALLOWED = frozenset((
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'))


def reference(args, key):
    path = getattr(args,key).resolve(strict=True)
    require(sha(path) == getattr(args,key+'_sha256'), key+' input changed')
    return path


def delta(manifest):
    require(sha(BASE_RELEASE) == BASE_SHA, 'GEN55 baseline release changed')
    base = read(BASE_RELEASE)['source_sha256']
    source = manifest['source_sha256']
    require(len(base) == 1082 and set(base) <= set(source), 'baseline source omitted')
    require(all(not Path(p).is_absolute() and '..' not in Path(p).parts
        for p in source), 'unsafe source path')
    changed = {p for p in base if base[p] != source[p]}
    added = set(source)-set(base)
    require(changed and changed <= ALLOWED and
        changed == set(manifest['allowed_production_paths']), 'unreviewed production delta')
    require(added and all('/tests/' in p and p.startswith('src/') and
        (Path(p).name.startswith('test_gen56_') and p.endswith('.py') or
         '/tests/fixtures/gen56_' in p) for p in added), 'unreviewed new source')
    return base, changed, added


def review(path, source):
    value = read(path)
    require(value['passed'] is True and value['source_sha256'] == source and
        value['full_checks_retained'] is True and value['physical_grasp_verified'] is False,
        'candidate independent review incomplete')
    require(all(sha(p) == h for p,h in value['dependency_sha256'].items()),
        'independent review dependency changed')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('manifest','runner','focus','full','review','helpers_review','readiness'):
        parser.add_argument('--'+key.replace('_','-'),type=Path,required=True)
        parser.add_argument('--'+key.replace('_','-')+'-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    refs = {k:reference(args,k) for k in
        ('manifest','runner','focus','full','review','helpers_review','readiness')}
    manifest,context,full,evidence = validate_result(refs['full'],refs['manifest'],
        args.manifest_sha256,refs['runner'],args.runner_sha256,'full')
    focus_receipt = read(refs['focus'])
    focus_manifest,focus_context,focus,focus_evidence = validate_result(refs['focus'],
        focus_receipt['candidate_manifest_path'],focus_receipt['candidate_manifest_sha256'],
        refs['runner'],args.runner_sha256,'focus')
    require(focus_context['sources'] == context['sources'] and
        focus_manifest['focus_scope'] == manifest['focus_scope'],
        'focus and full did not verify the same candidate/scope')
    require(Path(manifest['workspace_root']).resolve(strict=True) == CANDIDATE,
        'unexpected candidate workspace')
    baseline,changed,added = delta(manifest)
    source = context['sources']
    require(source == manifest['source_sha256'], 'candidate source map differs')
    reviewed = review(refs['review'], source)
    helpers = read(refs['helpers_review'])
    operational = (*HELPERS,Path(__file__).resolve(),READINESS,
        W/'.runtime/finalize_gen56_idle_deployment_20261020.py',
        W/'.runtime/start_gen56_recorded_attempt_20261020.sh',
        W/'.runtime/start_gen56_recorded_attempt_ready_20261020.py',
        W/'.runtime/bind_gen56_remote_dependency_20261020.py',
        W/'.runtime/verify_active_remote_readiness_20261020.py')
    require(helpers['passed'] is True and all(sha(p)==h
        for p,h in helpers['dependency_sha256'].items()) and
        all(str(p) in helpers['dependency_sha256'] for p in operational),
        'runtime helper independent review incomplete')
    previous,processes = validate_previous(PREVIOUS)
    control = validate_control(CONTROL)
    resumed = validate_resume_reference()
    ready = read(refs['readiness'])
    require(ready['passed'] is True and ready['manual_mode'] is True and
        ready['task_active'] is False and ready['actions_active'] is False and
        ready['source_matches_operational_release'] is True and
        ready['node_identities'] == processes and ready['source_count'] == len(baseline) and
        ready['sample_count'] >= 20 and ready['epoch_ns'] == resumed['epoch_ns'] and
        ready['resume_reference_path'] == str(RESUME_REFERENCE) and
        ready['resume_reference_sha256'] == RESUME_REFERENCE_SHA and
        ready['release_sha256'] == BASE_SHA and ready['deployment_sha256'] == PREVIOUS_SHA and
        Path(ready['release']).resolve(strict=True) == BASE_RELEASE and
        Path(ready['deployment']).resolve(strict=True) == PREVIOUS and
        ready['helper_sha256'] == sha(READINESS) and ready['control_snapshot'] == control and
        ready['control_sha256'] == CONTROL_EXPECTED_SHA and
        0 <= time.time_ns()-ready['recorded_ns'] < 120_000_000_000,
        'fresh current manual/idle baseline readiness required')
    require(all(process(r['pid']) == r for r in processes.values()), 'runtime process changed')
    require(all(sha(W/p)==h for p,h in baseline.items()), 'live baseline differs')
    require(all(not (W/p).exists() for p in added), 'new source destination already exists')
    require(all(sha(CANDIDATE/p)==h for p,h in source.items()), 'tested candidate changed')
    output = args.output.resolve(); storage(output)
    require(not output.exists() and not RELEASE.exists(), 'installation/release already exists')
    pinned = {str(p):sha(p) for p in (*refs.values(),BASE_RELEASE,PREVIOUS,CONTROL,
        RESUME_REFERENCE,RESUME_COLLECTOR,CONTROL_SNAPSHOT,*operational)}
    output.mkdir()
    write(output/'plan.json',dict(changed_paths=sorted(changed),added_paths=sorted(added),
        baseline_sha256=baseline,source_sha256=source,input_sha256=pinned,
        full_test_result=str(refs['full']),full_test_result_sha256=args.full_sha256,
        recorded_ns=time.time_ns(),runtime_reloaded=False,motion_commands_sent=False))
    for name in sorted(changed):
        backup=output/'backup_sources'/name;backup.parent.mkdir(parents=True,exist_ok=True)
        raw=(W/name).read_bytes()
        with backup.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
        require(sha(backup)==baseline[name], 'baseline backup differs')
    with (CONTROL.parent/'control.lock').open('rb') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        require(validate_control(CONTROL)==control and all(sha(p)==h for p,h in pinned.items()),
            'installation inputs/control changed')
        require(all(process(r['pid'])==r for r in processes.values()), 'runtime changed')
        installed=[]
        for name in sorted(changed|added):
            target=W/name; temporary=target.with_name(target.name+'.gen56.install.tmp')
            raw=(CANDIDATE/name).read_bytes()
            require(hashlib.sha256(raw).hexdigest()==source[name] and not temporary.exists(),
                'candidate changed or temporary destination exists')
            require((name in changed and sha(target)==baseline[name]) or
                (name in added and not target.exists()), 'destination changed')
            target.parent.mkdir(parents=True,exist_ok=True)
            mode=target.stat().st_mode&0o777 if target.exists() else 0o644
            with temporary.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
            os.chmod(temporary,mode);os.replace(temporary,target)
            require(sha(target)==source[name], 'installed read-back differs')
            installed.append(name)
            write(output/'progress.json',dict(installed=installed,complete=False,runtime_reloaded=False))
        require(all(sha(W/p)==h and sha(CANDIDATE/p)==h for p,h in source.items()),
            'installed/tested source bytes differ')
        evidence.update(focus_evidence);evidence.update(reviewed['dependency_sha256'])
        evidence.update(helpers['dependency_sha256'])
        evidence.update({str(p):sha(p) for p in refs.values()})
        evidence.update({str(BASE_RELEASE):BASE_SHA,str(PREVIOUS):PREVIOUS_SHA})
        evidence.update({str(RESUME_REFERENCE):RESUME_REFERENCE_SHA,
            str(RESUME_COLLECTOR):RESUME_COLLECTOR_SHA,str(CONTROL_SNAPSHOT):CONTROL_EXPECTED_SHA})
        installation=dict(passed=True,installed_paths=installed,source_sha256=source,
            candidate_manifest=str(refs['manifest']),candidate_manifest_sha256=args.manifest_sha256,
            full_regression_passed=True,full_regression_location=str(CANDIDATE),
            installed_bytes_exact_tested_candidate=True,readiness=str(refs['readiness']),
            readiness_sha256=args.readiness_sha256,recorded_ns=time.time_ns(),
            resume_reference_path=str(RESUME_REFERENCE),resume_reference_sha256=RESUME_REFERENCE_SHA,
            runtime_reloaded=False,motion_commands_sent=False)
        write(output/'result.json',installation)
        evidence[str(output/'result.json')]=sha(output/'result.json')
        release=dict(generation='GEN56',passed=True,reviewed=True,installed=True,runtime_deployed=False,
            physical_grasp_verified=False,baseline_release_sha256=BASE_SHA,
            previous_deployment_sha256=PREVIOUS_SHA,source_sha256=source,source_count=len(source),
            test_scope_files=full['test_scope_files'],test_summary=full['summary'],
            historical_test_files=336,historical_tests=9568,actual_regression=str(refs['full']),
            actual_regression_sha256=args.full_sha256,candidate_manifest=str(refs['manifest']),
            candidate_manifest_sha256=args.manifest_sha256,runner=str(refs['runner']),
            runner_sha256=args.runner_sha256,worker_count=full['worker_count'],
            independent_review=str(refs['review']),independent_review_sha256=args.review_sha256,
            evidence_sha256=evidence,full_regression_location=str(CANDIDATE),
            installed_bytes_exact_tested_candidate=True,installation=str(output/'result.json'),
            installation_sha256=sha(output/'result.json'),
            new_fresh_scene_robot_and_path_acceptance_required=True,
            recorded_ns=time.time_ns(),motion_commands_sent=False)
        validate_sources(release)
        write(RELEASE,release)
        require(read(RELEASE)==release,'release read-back differs')
    print(json.dumps(dict(installed=True,sealed=True,release=str(RELEASE),
        sha256=sha(RELEASE),summary=full['summary']),ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
