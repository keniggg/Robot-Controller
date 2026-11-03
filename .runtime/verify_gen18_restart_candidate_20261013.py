"""Verify GEN18 sources and offline tests, retaining all output on SanStand."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
OLD = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
BASE = OLD / 'verification/gen17_station_admission_release_manifest.json'
PATCH = W / '.runtime/gen18_stable_recheck_offline_20261013/source_manifest.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--phase', required=True, choices=('focused', 'combined'))
    args = parser.parse_args()
    session = args.session.resolve()
    assert MOUNT.is_mount() and session.parent == EXT / 'ros_log'
    assert session.parent.stat().st_dev == MOUNT.stat().st_dev
    config = json.loads((EXT / 'grasp_records/archive_config.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert json.loads((EXT / 'maintenance/storage_control.json').read_text())['archive_policy'] == 'local_only'
    assert shutil.disk_usage(EXT).free >= config['min_free_bytes']
    patch = json.loads(PATCH.read_text())
    assert sha(BASE) == patch['baseline_source_manifest_sha256']
    baseline = json.loads(BASE.read_text())
    assert all(baseline[k] is True for k in ('passed', 'reviewed', 'installed'))
    changed = {row['path']: row for row in patch['files']}
    sources = dict(baseline['source_sha256'])
    for relative, expected in sources.items():
        if relative in changed:
            assert changed[relative]['baseline_sha256'] == expected, relative
        else:
            assert sha(W / relative) == expected, relative
    for relative, row in changed.items():
        assert sha(W / relative) == row['sha256'], relative
        if relative.startswith('src/'):
            sources[relative] = row['sha256']
    scope = list(baseline['test_scope'])
    for relative in changed:
        if '/tests/test_' in relative and relative not in scope:
            scope.append(relative)
    if args.phase == 'focused':
        scope = [relative for relative in scope if relative in changed and '/tests/test_' in relative]
        scope.append('src/alicia_flexible_grasp_supervisor/tests/test_contact_height_window_proposals.py')
    assert len(scope) == len(set(scope)) and scope
    session.mkdir(exist_ok=True)
    out = session / 'verification' / ('gen18_' + args.phase)
    out.mkdir(parents=True, exist_ok=False)
    for name in ('tmp', 'ros'):
        (out / name).mkdir()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
               PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='1', ROS_MASTER_URI='http://127.0.0.1:1',
               TMPDIR=str(out / 'tmp'), ROS_LOG_DIR=str(out / 'ros'))
    paths = [W / 'src/alicia_flexible_grasp_supervisor/tests', W / 'src/alicia_grasp_modes/tests',
             W / 'src/alicia_grasp_modes/src', W / 'src/alicia_flexible_grasp_supervisor/src',
             W / 'devel/lib/python3/dist-packages']
    env['PYTHONPATH'] = ':'.join(map(str, paths)) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W / 'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    command = [sys.executable, '-m', 'pytest', '-q', *scope, '-p', 'no:cacheprovider',
               '--tb=short', '--basetemp=' + str(out / 'pytest_tmp'),
               '--junitxml=' + str(out / 'pytest.xml')]
    print(json.dumps(dict(started=True, phase=args.phase, output=str(out),
                          source_count=len(sources), test_files=len(scope))), flush=True)
    started = time.monotonic()
    with (out / 'pytest.log').open('x') as stream:
        completed = subprocess.run(command, cwd=W, env=env, stdout=stream, stderr=subprocess.STDOUT)
    summary = {}
    xml = out / 'pytest.xml'
    if xml.exists():
        suites = list(ET.parse(xml).getroot().iter('testsuite'))
        summary = {key: sum(int(s.get(key, 0)) for s in suites)
                   for key in ('tests', 'failures', 'errors', 'skipped')}
    unchanged = all(sha(W / r) == h for r, h in sources.items())
    passed = (completed.returncode == 0 and unchanged and summary.get('tests', 0) > 0
              and all(summary.get(k, -1) == 0 for k in ('failures', 'errors', 'skipped')))
    result = dict(passed=passed, summary=summary, returncode=completed.returncode,
                  elapsed_sec=time.monotonic() - started, command=command,
                  source_sha256=sources, source_unchanged=unchanged, test_scope=scope,
                  baseline=str(BASE), baseline_sha256=sha(BASE), patch=str(PATCH), patch_sha256=sha(PATCH),
                  log_sha256=sha(out / 'pytest.log'), xml_sha256=sha(xml) if xml.exists() else None,
                  runner_sha256=sha(__file__), production_worktree_tested=True,
                  live_ros_contacted=False, motion_commands_sent=False, temperature_queried=False)
    assert MOUNT.is_mount() and out.stat().st_dev == MOUNT.stat().st_dev
    save(out / 'result.json', result)
    print(json.dumps(dict(passed=passed, summary=summary, elapsed_sec=result['elapsed_sec'],
                          result=str(out / 'result.json'))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
