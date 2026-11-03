"""Draft: install reviewed GEN53 bytes while the accepted GEN52 runtime is idle.

No ROS/motion API is called. Installed-W full regression is required before
any later reload. This file is prepared for root execution, not run on import.
"""
import argparse
import hashlib
import json
import os
import time
from pathlib import Path

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
BASE = W/'.runtime/gen53_baseline_source/runner_manifest.json'
BASE_SHA = '6609863f28d265d8d9caf1bbba3eaffe3b282538c843c42153ad7095fad195d8'
RELEASE = V/'gen52_release_manifest.json'
RELEASE_SHA = 'dfa045bdf7051cf177e59722ed5ad5b81e856a59e5492d49bc78c2d7a9a367c1'
DEPLOYMENT = E/'ros_log/full_validated_gen52_restart_20261019T013414Z/gen52_current_runtime_identity.json'
DEPLOYMENT_SHA = '75eafaf263889ed8ebb8a018852b2506de7d354d5afe4c282a7fce1f59b6d78c'
READINESS_HELPER = W/'.runtime/read_gen53_preinstall_readiness_20261018.py'
PRODUCTION = {'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/box_surface_registration.py', 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py', 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'}
NEW_PRODUCTION = {'src/alicia_grasp_modes/src/alicia_grasp_modes/station_snapshot_retry.py'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())


def process_matches(row):
    fields = (Path('/proc')/str(row['pid'])/'stat').read_text().rsplit(')', 1)[1].split()
    return (fields[0] != 'Z' and fields[19] == row['start_ticks']
            and int(fields[1]) == row['parent_pid'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'focus', 'review', 'readiness'):
        parser.add_argument('--'+name, type=Path, required=True)
        parser.add_argument('--'+name+'-sha256', required=True)
    parser.add_argument('--expected-control-revision', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert M.is_mount() and args.expected_control_revision > 0
    output = args.output.resolve()
    assert V in output.parents and not output.exists()
    refs = {}
    for name in ('manifest', 'focus', 'review', 'readiness'):
        path = getattr(args, name).resolve(strict=True)
        digest = getattr(args, name+'_sha256')
        assert E in path.parents and sha(path) == digest
        refs[str(path)] = digest
    assert sha(BASE) == BASE_SHA and sha(RELEASE) == RELEASE_SHA
    assert sha(DEPLOYMENT) == DEPLOYMENT_SHA
    baseline = read(BASE)['source_sha256']
    assert len(baseline) == 1065 and read(RELEASE)['source_sha256'] == baseline
    manifest, focus, review, ready = [read(getattr(args, name))
        for name in ('manifest', 'focus', 'review', 'readiness')]
    candidate = Path(manifest['workspace_root']).resolve(strict=True)
    assert candidate == W/'.runtime/gen53_integrated_candidate'
    assert manifest['kind'] == 'GEN53_frozen_offline_candidate'
    assert manifest.get('variant', 'candidate') == 'candidate'
    source = manifest['source_sha256']
    ADDED = {row['path'] for row in manifest['new_tests'] + manifest.get('new_fixture_sources', []) + manifest.get('new_production_sources', [])}
    assert len(manifest['new_tests']) >= 2
    assert {row['path'] for row in manifest.get('new_production_sources', [])} == NEW_PRODUCTION
    for row in manifest['new_production_sources']:
        evidence = Path(row['review_evidence']['path']).resolve(strict=True)
        assert E in evidence.parents and sha(evidence) == row['review_evidence']['sha256']
        assert read(evidence)['passed'] is True
        refs[str(evidence)] = sha(evidence)
    assert all(('/tests/test_gen53_' in name and name.endswith('.py'))
        or ('/tests/fixtures/gen53_' in name and name.endswith('.json')) for name in ADDED - NEW_PRODUCTION)
    assert len(source) == 1065 + len(ADDED) and set(source)-set(baseline) == ADDED
    assert {path for path in baseline if source[path] != baseline[path]} == PRODUCTION
    assert all(not Path(path).is_absolute() and '..' not in Path(path).parts for path in source)
    assert all(sha(candidate/path) == digest for path, digest in source.items())
    assert all(sha(W/path) == digest and sha(BASE.parent/path) == digest
        for path, digest in baseline.items())
    assert all(not (W/path).exists() for path in ADDED)
    assert all(focus.get(key) is True for key in
        ('passed', 'scope_complete', 'test_bodies_executed', 'source_unchanged'))
    assert focus['mode'] == 'focus' and focus['source_sha256'] == source
    assert focus['summary']['tests'] > 0
    assert all(focus['summary'][key] == 0 for key in ('failures', 'errors', 'skipped'))
    assert not focus['external_boundary_attempts'] and not focus['foreign_or_inconsistent_imports']
    assert review['passed'] is True and review['source_sha256'] == source
    assert ready['passed'] is True and ready['manual_mode'] is True and ready['task_active'] is False
    assert ready['deployment_processes_unchanged'] is True
    assert ready['selection']['generation'] == 1 and ready['selection']['mode'] == 'unknown'
    assert ready['selection']['strategy'] == 'observe_return'
    assert ready['epoch_ns'] == 1792373681250409915 and ready['source_count'] == 1065
    assert 0 <= time.time_ns()-ready['recorded_ns'] < 120_000_000_000
    assert Path(ready['release']).resolve(strict=True) == RELEASE.resolve(strict=True)
    assert Path(ready['deployment']).resolve(strict=True) == DEPLOYMENT.resolve(strict=True)
    assert ready['release_sha256'] == RELEASE_SHA and ready['deployment_sha256'] == DEPLOYMENT_SHA
    assert ready['helper_sha256'] == sha(READINESS_HELPER)
    control = read(CONTROL)
    assert control['state'] == 'active' and type(control['revision']) is int
    assert control['revision'] == args.expected_control_revision
    assert control == ready['control_snapshot'] and sha(CONTROL) == ready['control_sha256']
    refs.update({str(CONTROL): sha(CONTROL), str(BASE): BASE_SHA,
        str(RELEASE): RELEASE_SHA, str(DEPLOYMENT): DEPLOYMENT_SHA,
        str(READINESS_HELPER): sha(READINESS_HELPER)})
    assert all(sha(path) == digest for path, digest in refs.items())
    deployment = read(DEPLOYMENT)
    processes = dict(deployment['after'], **deployment['preserved_after'])
    assert len(deployment['after']) == 4 and len(deployment['preserved_after']) == 7
    assert ready['node_identities'] == processes and all(process_matches(row) for row in processes.values())
    output.mkdir()
    write(output/'plan.json', dict(source_sha256=source, baseline_sha256=baseline,
        evidence_sha256=refs, changed_paths=sorted(PRODUCTION | ADDED),
        control_snapshot=control, full_regression_passed=False, runtime_reloaded=False,
        motion_commands_sent=False, created_ns=time.time_ns()))
    for index, relative in enumerate(sorted(PRODUCTION | ADDED)):
        target = W/relative
        temporary = target.with_name(target.name+'.gen53.install.tmp')
        assert not temporary.exists() and all(sha(path) == digest for path, digest in refs.items())
        mode = target.stat().st_mode & 0o777 if target.exists() else 0o644
        with temporary.open('xb') as stream:
            stream.write((candidate/relative).read_bytes())
            stream.flush()
            os.fsync(stream.fileno())
        assert sha(temporary) == source[relative]
        os.chmod(temporary, mode)
        os.replace(temporary, target)
        write(output/('installed_%02d.json' % index), dict(path=relative,
            sha256=sha(target), recorded_ns=time.time_ns()))
    assert all(sha(W/path) == digest for path, digest in source.items())
    assert all(sha(path) == digest for path, digest in refs.items())
    result = dict(passed=True, source_count=len(source), source_sha256=source,
        changed_files={path: source[path] for path in sorted(PRODUCTION | ADDED)},
        original_baseline=str(BASE.parent), evidence_sha256=refs, control_snapshot=control,
        full_regression_passed=False, runtime_reloaded=False, motion_commands_sent=False,
        temperature_queried=False, recorded_ns=time.time_ns(), helper_sha256=sha(__file__))
    write(output/'result.json', result)
    print(json.dumps(dict(passed=True, result=str(output/'result.json'), sha256=sha(output/'result.json'))))


if __name__ == '__main__':
    main()
