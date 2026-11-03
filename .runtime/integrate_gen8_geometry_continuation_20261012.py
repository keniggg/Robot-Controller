"""Install, verify and seal the reviewed geometry and continuing-authority patch."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
V = S / 'verification'
sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
read = lambda path: json.loads(Path(path).read_text())


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def sources(expected):
    for name, digest in expected.items():
        assert sha(W / name) == digest, name


def install(args):
    old = read(V / 'gen7_yield_release_manifest.json')
    sources(old['source_sha256'])
    deliveries, changes, evidence, scope = [], {}, {}, []
    for delivery_path, review_path in ((args.geometry_delivery, args.geometry_review),
                                       (args.authority_delivery, args.authority_review)):
        delivery, review = read(delivery_path), read(review_path)
        assert delivery['passed'] is True and review['passed'] is True
        assert review['delivery_sha256'] == sha(delivery_path)
        candidate_root = Path(delivery.get('candidate_root', delivery['stage']))
        for row in delivery['files']:
            relative = row.get('destination', row['path'])
            path = Path(relative)
            assert not path.is_absolute() and '..' not in path.parts and relative not in changes
            candidate = candidate_root / row['path']
            digest = row.get('candidate_sha256', row.get('sha256'))
            assert sha(candidate) == digest
            assert (sha(W / path) if (W / path).exists() else None) == row['base_sha256'], relative
            changes[relative] = dict(candidate=str(candidate), sha256=digest, base_sha256=row['base_sha256'])
        scope.extend(delivery.get('test_scope', []))
        scope.extend(review.get('test_scope', []))
        evidence[str(delivery_path)] = sha(delivery_path)
        evidence[str(review_path)] = sha(review_path)
        deliveries.append(str(delivery_path))
    for result_path in (S / 'gen8_analysis/full_generator_baseline/result.json',
                        S / 'gen8_analysis/full_generator_candidate/result.json'):
        result = read(result_path)
        assert result['passed'] and result['all_original_passing_poses_preserved']
        evidence[str(result_path)] = sha(result_path)
    assert read(S / 'gen8_analysis/full_generator_candidate/result.json')['passing_count'] == 218
    out = V / 'gen8_combined_install'
    out.mkdir(exist_ok=False)
    for relative, row in changes.items():
        target = W / relative
        if target.exists():
            backup = out / 'before' / relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, backup)
            assert sha(backup) == row['base_sha256']
    for relative, row in changes.items():
        target = W / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(row['candidate'], target)
        assert sha(target) == row['sha256']
    source = dict(old['source_sha256'], **{p:r['sha256'] for p,r in changes.items()})
    for relative in ('.runtime/integrate_gen8_geometry_continuation_20261012.py',
                     '.runtime/capture_continuation_scene_20261012.py',
                     '.runtime/restore_manual_after_continuation_20261012.py',
                     '.runtime/replay_gen8_complete_view_generation_20261012.py',
                     'src/alicia_grasp_modes/tests/test_runner_prewarm_subprocess.py'):
        source[relative] = sha(W / relative)
    scope.extend(read(V / 'gen7_yield_integrated/result.json')['test_scope'])
    scope = [p for p in scope if Path(p).name != 'test_runner_prewarm.py']
    scope.append('src/alicia_grasp_modes/tests/test_runner_prewarm_subprocess.py')
    scope = list(dict.fromkeys(scope))
    assert scope and all(Path(path).is_file() if Path(path).is_absolute() else (W / path).is_file() for path in scope)
    sources(source)
    save(out / 'result.json', dict(passed=True, installed_ns=time.time_ns(),
        source_sha256=source, changes=changes, delivery_paths=deliveries,
        evidence_sha256=evidence, test_scope=scope, robot_nodes_restarted=False,
        motion_commands_sent=False, temperature_queried=False))
    print(json.dumps(dict(passed=True, changed_files=len(changes), source_count=len(source), test_files=len(scope))), flush=True)


def test():
    installed = read(V / 'gen8_combined_install/result.json')
    expected = installed['source_sha256']
    sources(expected)
    out = V / 'gen8_combined_integrated'
    out.mkdir(exist_ok=False)
    env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1',
        GEN6_TEST_WORKTREE=str(W), GEN10_TEST_LAUNCHER=str(W / '.runtime/run_reboot_realigned_grasp_20261010.py'),
        TMPDIR='/tmp')
    env['PYTHONPATH'] = ':'.join(str(W / p) for p in (
        'src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    cmd = ['/usr/bin/python3', '-m', 'pytest', '-q', *installed['test_scope'],
        '-p', 'no:cacheprovider', '--junitxml=' + str(out / 'pytest.xml')]
    started = time.monotonic()
    with (out / 'pytest.log').open('x') as stream:
        process = subprocess.run(cmd, cwd=W, env=env, stdout=stream, stderr=subprocess.STDOUT)
    after = {p:sha(W / p) for p in expected}
    summary = {}
    for suite in ET.parse(out / 'pytest.xml').getroot().iter('testsuite'):
        for key in ('tests', 'failures', 'errors', 'skipped'):
            summary[key] = summary.get(key, 0) + int(suite.attrib.get(key, 0))
    passed = process.returncode == 0 and after == expected and summary['tests'] > 0 and not any(
        summary[k] for k in ('failures', 'errors', 'skipped'))
    save(out / 'result.json', dict(passed=passed, source_sha256=expected, source_unchanged=after == expected,
        summary=summary, command=cmd, test_scope=installed['test_scope'],
        returncode=process.returncode, elapsed_sec=time.monotonic() - started,
        production_worktree_tested=True, motion_commands_sent=False, temperature_queried=False))
    print(json.dumps(dict(passed=passed, summary=summary, elapsed_sec=time.monotonic() - started)), flush=True)
    print('\n'.join((out / 'pytest.log').read_text().splitlines()[-15:]), flush=True)
    assert passed


def release():
    installed_path = V / 'gen8_combined_install/result.json'
    tested_path = V / 'gen8_combined_integrated/result.json'
    installed, tested = read(installed_path), read(tested_path)
    assert installed['passed'] and tested['passed'] and tested['source_unchanged']
    assert installed['source_sha256'] == tested['source_sha256']
    for path, digest in installed['evidence_sha256'].items():
        assert sha(path) == digest
    sources(installed['source_sha256'])
    evidence = dict(installed['evidence_sha256'])
    evidence.update({str(path):sha(path) for path in (installed_path, tested_path)})
    target = V / 'gen8_combined_release_manifest.json'
    save(target, dict(passed=True, reviewed=True, installed=True, recorded_ns=time.time_ns(),
        source_sha256=installed['source_sha256'], test_summary=tested['summary'],
        evidence_sha256=evidence, changed_files_sha256={p:r['sha256'] for p,r in installed['changes'].items()},
        kind='actual_center_distance_bound_views_and_continuing_user_authorization',
        limits_unchanged=['original source/stage/solver deadlines', 'collision sampling and iteration limits',
            'maximum two observations per attempt', 'automatic lift disabled', 'exact selection and dual ACK'],
        physical_grasp_verified=False, motion_commands_sent=False, temperature_queried=False))
    print(json.dumps(dict(passed=True, manifest=str(target), sha256=sha(target))), flush=True)


def combine(args):
    previous, reloaded, manifest = read(args.previous), read(args.reload), read(args.manifest)
    assert previous['deployment_passed'] and previous['whole_patch_deployed']
    assert reloaded['deployment_passed'] and manifest['passed'] and manifest['reviewed']
    assert reloaded['previous_deployment'] == str(args.previous)
    assert reloaded['before'] == previous['after']
    assert reloaded['preserved_before'] == reloaded['preserved_after'] == previous['preserved_after']
    assert reloaded['source_sha256'] == manifest['source_sha256']
    sources(manifest['source_sha256'])
    result = dict(reloaded, whole_patch_deployed=True, compute_reloaded=True,
        source_manifest=str(args.manifest), source_manifest_sha256=sha(args.manifest),
        reload_evidence=str(args.reload), reload_sha256=sha(args.reload),
        previous_deployment_sha256=sha(args.previous), recorded_ns=time.time_ns(),
        physical_grasp_verified=False, deployment_kind='dedicated_compute_geometry_and_operational_authorization')
    save(args.output, result)
    print(json.dumps(dict(deployment_passed=True, whole_patch_deployed=True, result=str(args.output))), flush=True)


parser = argparse.ArgumentParser()
parser.add_argument('operation', choices=('install', 'test', 'release', 'combine'))
for name in ('geometry-delivery', 'geometry-review', 'authority-delivery', 'authority-review',
             'previous', 'reload', 'manifest', 'output'):
    parser.add_argument('--' + name, type=Path)
args = parser.parse_args()
assert Path('/media/zhuyupei/SanStand').is_mount()
if args.operation == 'install': install(args)
elif args.operation == 'test': test()
elif args.operation == 'release': release()
else: combine(args)
