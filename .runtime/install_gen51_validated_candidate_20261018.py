"""Draft: install reviewed GEN51 bytes while the accepted GEN50 runtime is idle.

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
BASE = W/'.runtime/gen51_baseline_source/manifest.json'
BASE_SHA = '0e489f2fb7871c1a2e47124f54bb253af154fdde9f78d51b1491c69ad52924c5'
RELEASE = V/'gen50_release_manifest.json'
RELEASE_SHA = '4151ad0e3d0eceb382a2cb006dafd9614a793618f74a75c2d1b5bc234c8630ca'
DEPLOYMENT = N/'gen50_runtime/gen50_runtime_deployment.json'
DEPLOYMENT_SHA = 'e2f7b22c0ccecc9d0ec90cdf88be254ac955e315babe2ee187719272027e194e'
READINESS_HELPER = W/'.runtime/read_gen51_preinstall_readiness_20261018.py'
PRODUCTION = {
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/pregrasp_compensation.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/outer_rgb_rectangle.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py',
}
ADDED = {
    'src/alicia_flexible_grasp_supervisor/tests/test_gen51_shared_held_guide_fallback.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen51_rgb_contour_tolerance.py',
    'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen51_g50_held_guide_inputs.json',
}


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
    assert len(baseline) == 1058 and read(RELEASE)['source_sha256'] == baseline
    manifest, focus, review, ready = [read(getattr(args, name))
        for name in ('manifest', 'focus', 'review', 'readiness')]
    candidate = Path(manifest['workspace_root']).resolve(strict=True)
    assert candidate == W/'.runtime/gen51_integrated_candidate'
    assert manifest['kind'] == 'GEN51_frozen_offline_candidate'
    assert manifest.get('variant', 'candidate') == 'candidate'
    source = manifest['source_sha256']
    assert len(source) == 1061 and set(source)-set(baseline) == ADDED
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
    assert ready['selection']['generation'] == 11 and ready['selection']['mode'] == 'unknown'
    assert ready['selection']['strategy'] == 'observe_return'
    assert ready['epoch_ns'] == 1792286012067798399 and ready['source_count'] == 1058
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
        temporary = target.with_name(target.name+'.gen51.install.tmp')
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
