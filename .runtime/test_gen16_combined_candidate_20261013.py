"""Run bounded offline integration tests on the exact merged candidate."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--delivery', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--test', action='append', default=[])
    args = ap.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    d = json.loads(args.delivery.read_text())
    candidate = Path(d['candidate_root'])
    workspace = Path(__file__).resolve().parents[1]
    baseline_path = next(p for p in d['parent_sha256'] if p.endswith('/result.json'))
    baseline = json.loads(Path(baseline_path).read_text())['source_sha256']
    expected = dict(baseline)
    for row in d['files']:
        expected[row['path']] = row['candidate_sha256']
    scope = list(dict.fromkeys(args.test or [r['path'] for r in d['files']
        if '/tests/' in r['path'] and Path(r['path']).name.startswith('test_')]))
    assert scope and all(p in d['test_scope'] for p in scope)
    expected.update({p: sha(candidate / p) for p in scope})

    def verify():
        for name, digest in baseline.items():
            assert sha(workspace / name) == digest, 'actual changed: ' + name
        for name, digest in expected.items():
            assert sha(candidate / name) == digest, 'candidate changed: ' + name
        for name, digest in d['parent_sha256'].items():
            assert sha(name) == digest

    verify()
    args.output.mkdir(exist_ok=False)
    env = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1',
        GEN6_TEST_WORKTREE=str(candidate),
        GEN10_TEST_LAUNCHER=str(candidate / '.runtime/run_reboot_realigned_grasp_20261010.py'),
        TMPDIR='/tmp')
    env['PYTHONPATH'] = ':'.join(str(candidate / p) for p in (
        'src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(candidate / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    cmd = ['/usr/bin/python3', '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
           '--junitxml=' + str(args.output / 'pytest.xml')]
    start = time.monotonic()
    with (args.output / 'pytest.log').open('x') as stream:
        p = subprocess.run(cmd, cwd=candidate, env=env, stdout=stream, stderr=subprocess.STDOUT)
    counts = dict(tests=0, failures=0, errors=0, skipped=0)
    for suite in ET.parse(args.output / 'pytest.xml').getroot().iter('testsuite'):
        for key in counts:
            counts[key] += int(suite.attrib.get(key, 0))
    verify()
    passed = p.returncode == 0 and counts['tests'] > 0 and not any(counts[k] for k in ('failures', 'errors', 'skipped'))
    result = dict(passed=passed, kind='GEN16_EXACT_MERGED_CANDIDATE_INTEGRATION',
        delivery_path=str(args.delivery), delivery_sha256=sha(args.delivery),
        summary=counts, elapsed_sec=time.monotonic()-start, command=cmd, test_scope=scope,
        source_sha256=expected, sources_unchanged=True, actual_sources_unchanged=True,
        motion_commands_sent=False, physical_grasp_verified=False)
    (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(passed=passed, summary=counts, elapsed_sec=result['elapsed_sec'])), flush=True)
    print('\n'.join((args.output / 'pytest.log').read_text().splitlines()[-16:]), flush=True)
    assert passed


if __name__ == '__main__':
    main()
