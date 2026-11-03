"""Focused offline station-support verification with external-only artifacts."""
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
PRODUCTION = ('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py', 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py', 'src/alicia_flexible_grasp_supervisor/tests/test_gen26_regular_box_inference_independent.py')
def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('scope', nargs='+')
    args = p.parse_args()
    out = args.output.resolve()
    assert E in out.parents and Path('/media/zhuyupei/SanStand').is_mount()
    sources = {name:sha(W/name) for name in (*PRODUCTION, *args.scope)}
    out.mkdir(exist_ok=False)
    for name in ('tmp', 'ros'):
        (out/name).mkdir()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out/'tmp'), ROS_LOG_DIR=str(out/'ros'))
    env['PYTHONPATH'] = ':'.join(str(W/name) for name in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH', '')
    env['ROS_PACKAGE_PATH'] = str(W/'src') + ':' + env.get('ROS_PACKAGE_PATH', '')
    cmd = [sys.executable, '-m', 'pytest', '-q', *args.scope, '-p', 'no:cacheprovider', '--tb=short',
        '--basetemp='+str(out/'pytest_tmp'), '--junitxml='+str(out/'pytest.xml')]
    start = time.monotonic()
    with (out/'pytest.log').open('x') as log:
        done = subprocess.run(cmd, cwd=W, env=env, stdout=log, stderr=subprocess.STDOUT)
    suites = list(ET.parse(out/'pytest.xml').getroot().iter('testsuite'))
    summary = {k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','failures','errors','skipped')}
    unchanged = all(sha(W/name) == digest for name,digest in sources.items())
    r = dict(summary=summary, source_unchanged=unchanged, source_sha256=sources,
        returncode=done.returncode, elapsed_sec=time.monotonic()-start,
        log_sha256=sha(out/'pytest.log'), xml_sha256=sha(out/'pytest.xml'),
        runner_sha256=sha(__file__), test_scope=args.scope, ros_graph_access=False,
        motion_commands_sent=False, temperature_queried=False)
    (out/'receipt.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(summary=summary, source_unchanged=unchanged, output=str(out/'receipt.json'))),flush=True)
    return done.returncode if unchanged else 99

if __name__ == '__main__':
    raise SystemExit(main())
