"""Verify the one-file XML-RPC selection fix against the sealed GEN22 baseline."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
BASE = E/'ros_log/full_latest_gen18_restart_20261014T012216Z/verification/gen22_release_manifest.json'
PRODUCTION = 'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_retirement.py'
SCOPE = (
    'src/alicia_grasp_modes/tests/test_contact_station_retirement.py',
    'src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py',
    'src/alicia_grasp_modes/tests/test_gen11_contact_remote_receipt.py',
    'src/alicia_grasp_modes/tests/test_gen11_contact_compute_admission_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_contact_pregrasp_task.py',
)

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    assert E in out.parents
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
    assert sha(BASE) == 'a99e400819346d3552f37eb2923fae870e6a3826d57451657a52ea25cc3c6da2'
    baseline = json.loads(BASE.read_text())
    sources = dict(baseline['source_sha256'])
    for name, digest in sources.items():
        if name != PRODUCTION:
            assert sha(W/name) == digest, name
    sources[PRODUCTION] = sha(W/PRODUCTION)
    for name in SCOPE:
        sources[name] = sha(W/name)
    out.mkdir(exist_ok=False)
    for name in ('tmp', 'ros'):
        (out/name).mkdir()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out/'tmp'), ROS_LOG_DIR=str(out/'ros'))
    env['PYTHONPATH'] = ':'.join(str(W/p) for p in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W/'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    cmd = [sys.executable, '-m', 'pytest', '-q', *SCOPE, '-p', 'no:cacheprovider', '--tb=short',
           '--basetemp='+str(out/'pytest_tmp'), '--junitxml='+str(out/'pytest.xml')]
    start = time.monotonic()
    with (out/'pytest.log').open('x') as log:
        completed = subprocess.run(cmd, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
    suites = list(ET.parse(out/'pytest.xml').getroot().iter('testsuite'))
    summary = {k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','failures','errors','skipped')}
    unchanged = all(sha(W/name) == digest for name,digest in sources.items())
    result = dict(passed=completed.returncode == 0 and unchanged,
        summary=summary, source_unchanged=unchanged, source_sha256=sources,
        source_count=len(sources), baseline=str(BASE), baseline_sha256=sha(BASE),
        changed_production=[PRODUCTION], test_scope=list(SCOPE),
        log_sha256=sha(out/'pytest.log'), xml_sha256=sha(out/'pytest.xml'),
        runner_sha256=sha(__file__), returncode=completed.returncode,
        elapsed_sec=time.monotonic()-start, live_ros_contacted=False,
        motion_commands_sent=False, temperature_queried=False)
    (out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'], summary=summary, source_unchanged=unchanged,
        output=str(out/'result.json')), sort_keys=True), flush=True)
    return 0 if result['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
