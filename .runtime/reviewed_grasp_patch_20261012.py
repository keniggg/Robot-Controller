"""Hash-bound local install and offline regression for subsequent grasp repairs."""
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
MOUNT = Path('/media/zhuyupei/SanStand')
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())


def check(expected):
    for p, h in expected.items():
        assert sha(W / p) == h, p


def save(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def install(a):
    prior = read(a.prior)
    assert prior['passed']
    check(prior['source_sha256'])
    changes, evidence = {}, {str(a.prior):sha(a.prior)}
    scope = list(prior['test_scope'])
    for delivery_path, review_path in a.pair:
        delivery, review = read(delivery_path), read(review_path)
        assert delivery['passed'] and review['passed']
        assert review['delivery_sha256'] == sha(delivery_path)
        stage = Path(delivery.get('candidate_root') or delivery['stage'])
        for row in delivery['files']:
            relative = row.get('destination', row['path'])
            p = Path(relative)
            assert not p.is_absolute() and '..' not in p.parts and relative not in changes
            h = row.get('candidate_sha256') or row['sha256']
            assert sha(stage / row['path']) == h
            assert (sha(W / p) if (W / p).exists() else None) == row['base_sha256'], relative
            changes[relative] = dict(candidate=str(stage / row['path']), sha256=h,
                                     base_sha256=row['base_sha256'])
        scope.extend(delivery.get('test_scope', []))
        scope.extend(review.get('test_scope', []))
        evidence.update({str(p):sha(p) for p in (delivery_path, review_path)})
    assert changes
    scope = list(dict.fromkeys(scope))
    for p in scope:
        assert not Path(p).is_absolute() and '..' not in Path(p).parts
        assert p in changes or (W / p).is_file(), p
    a.output.mkdir(exist_ok=False)
    # Validate every input before any target mutation, then back up all old files.
    for relative, row in changes.items():
        if (W / relative).exists():
            backup = a.output / 'before' / relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(W / relative, backup)
            assert sha(backup) == row['base_sha256']
    for relative, row in changes.items():
        target = W / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(row['candidate'], target)
        assert sha(target) == row['sha256']
    expected = dict(prior['source_sha256'], **{p:r['sha256'] for p,r in changes.items()})
    expected[str(Path(__file__).resolve().relative_to(W))] = sha(__file__)
    expected['.runtime/read_pending_compute_release_20261012.py'] = sha(W / '.runtime/read_pending_compute_release_20261012.py')
    assert all((W / p).is_file() and not Path(p).is_absolute() for p in scope)
    for p in scope:
        expected[p] = sha(W / p)
    check(expected)
    result = dict(passed=True, recorded_ns=time.time_ns(), prior=str(a.prior),
        evidence_sha256=evidence, source_sha256=expected, changes=changes, test_scope=scope,
        motion_commands_sent=False, temperature_queried=False, robot_nodes_restarted=False)
    save(a.output / 'result.json', result)
    print(json.dumps(dict(passed=True, changed_files=len(changes), sources=len(expected), test_files=len(scope))), flush=True)


def test(a):
    receipt = read(a.prior)
    assert receipt['passed']
    check(receipt['source_sha256'])
    a.output.mkdir(exist_ok=False)
    env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1',
        GEN6_TEST_WORKTREE=str(W), GEN10_TEST_LAUNCHER=str(W / '.runtime/run_reboot_realigned_grasp_20261010.py'),
        TMPDIR='/tmp')
    env['PYTHONPATH'] = ':'.join(str(W / p) for p in (
        'src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    cmd = ['/usr/bin/python3', '-m', 'pytest', '-q', *receipt['test_scope'],
        '-p', 'no:cacheprovider', '--junitxml=' + str(a.output / 'pytest.xml')]
    started = time.monotonic()
    with (a.output / 'pytest.log').open('x') as stream:
        process = subprocess.run(cmd, cwd=W, env=env, stdout=stream, stderr=subprocess.STDOUT)
    after = {p:sha(W / p) for p in receipt['source_sha256']}
    counts = {}
    for suite in ET.parse(a.output / 'pytest.xml').getroot().iter('testsuite'):
        for key in ('tests', 'failures', 'errors', 'skipped'):
            counts[key] = counts.get(key, 0) + int(suite.attrib.get(key, 0))
    passed = process.returncode == 0 and after == receipt['source_sha256'] and counts['tests'] > 0 and not any(
        counts[k] for k in ('failures', 'errors', 'skipped'))
    result = dict(passed=passed, source_sha256=after, source_unchanged=after == receipt['source_sha256'],
        prior=str(a.prior), prior_sha256=sha(a.prior), summary=counts, command=cmd,
        elapsed_sec=time.monotonic()-started, returncode=process.returncode,
        production_worktree_tested=True, motion_commands_sent=False, temperature_queried=False)
    save(a.output / 'result.json', result)
    print(json.dumps(dict(passed=passed, summary=counts, elapsed_sec=result['elapsed_sec'])), flush=True)
    print('\n'.join((a.output / 'pytest.log').read_text().splitlines()[-16:]), flush=True)
    assert passed


def seal(a):
    installed, tested = read(a.prior), read(a.tests)
    assert installed['passed'] and tested['passed'] and tested['source_unchanged']
    assert tested['prior_sha256'] == sha(a.prior)
    assert installed['source_sha256'] == tested['source_sha256']
    # Recursively retain every installation/review evidence binding.
    evidence = {}
    item = installed
    while True:
        for p, h in item['evidence_sha256'].items():
            assert sha(p) == h, p
            evidence[p] = h
        if 'prior' not in item:
            break
        item = read(item['prior'])
    check(installed['source_sha256'])
    evidence.update({str(p):sha(p) for p in (a.prior, a.tests)})
    result = dict(passed=True, reviewed=True, installed=True, recorded_ns=time.time_ns(),
        kind='gen8_geometry_fk_constraint_precompute_and_continuing_authorization',
        source_sha256=installed['source_sha256'], test_summary=tested['summary'],
        evidence_sha256=evidence, physical_grasp_verified=False,
        motion_commands_sent=False, temperature_queried=False,
        limits_unchanged=['original source/stage/solver deadlines', 'collision sampling and iteration limits',
            'maximum two observations per attempt', 'automatic lift disabled', 'exact selection and dual ACK'])
    save(a.output, result)
    print(json.dumps(dict(passed=True, release=str(a.output), sha256=sha(a.output))), flush=True)


ap = argparse.ArgumentParser()
ap.add_argument('operation', choices=('install', 'test', 'seal'))
ap.add_argument('--prior', type=Path, required=True)
ap.add_argument('--output', type=Path, required=True)
ap.add_argument('--pair', type=Path, nargs=2, action='append')
ap.add_argument('--tests', type=Path)
a = ap.parse_args()
assert MOUNT.is_mount() and MOUNT in a.output.resolve().parents
if a.operation == 'install': install(a)
elif a.operation == 'test': test(a)
else: seal(a)
