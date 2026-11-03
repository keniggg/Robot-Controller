#!/usr/bin/python3
"""Offline independent GEN20 test receipt; writes only under mounted SanStand."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_gen18_restart_20261014T012216Z/verification'
TEST = 'src/alicia_grasp_modes/tests/test_gen20_observation_search_independent.py'
FILES = [TEST,
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py',
    'src/alicia_grasp_modes/tests/test_remote_modes.py',
    'src/alicia_grasp_modes/tests/test_silhouette_observation_coverage.py',
    'src/alicia_grasp_modes/tests/test_silhouette_observation_planning.py',
    'src/alicia_grasp_modes/tests/test_silhouette_observation_view_source.py',
    'src/alicia_grasp_modes/tests/test_mode_runner.py']

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p = argparse.ArgumentParser()
    p.add_argument('tag')
    p.add_argument('--select', default='')
    p.add_argument('--baseline', default='')
    p.add_argument('--legacy-quota-ablation', action='store_true')
    p.add_argument('--test-file', default=TEST)
    a = p.parse_args()
    assert a.tag.startswith('gen20_independent_') and '/' not in a.tag
    assert os.path.ismount('/media/zhuyupei/SanStand') and E.is_dir()
    out = N / a.tag
    out.mkdir()
    for name in ('tmp', 'ros'):
        (out / name).mkdir()
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out/'tmp'), ROS_LOG_DIR=str(out/'ros'),
        ROS_PACKAGE_PATH=str(W/'src'))
    env['PYTHONPATH'] = ':'.join(str(W/s) for s in (
        'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':/opt/ros/noetic/lib/python3/dist-packages'
    args = ['-q', a.test_file, '-p', 'no:cacheprovider', '--tb=short',
        '--basetemp=' + str(out/'pytest_tmp'), '--junitxml=' + str(out/'pytest.xml')]
    if a.select:
        args += ['-k', a.select]
    files = list(dict.fromkeys(FILES + [a.test_file]))
    before = {s: sha(W/s) for s in files}
    baseline = None
    command = ['/usr/bin/python3', '-m', 'pytest'] + args
    if a.baseline:
        old = Path(a.baseline)
        expected = json.loads((N/'gen19_release_manifest.json').read_text())['source_sha256'][FILES[1]]
        assert sha(old) == expected
        # Restore only the old observation discovery methods in this isolated
        # child process. Operational files and loaded ROS nodes are untouched.
        names = ('_claim_observation_tilt_check', '_check_moveit_stable_candidate')
        boot = out/'baseline_bootstrap.py'
        boot.write_text('''import ast, sys, textwrap\nimport pytest\nfrom test_remote_modes import base\nsrc=open(%r).read()\ntree=ast.parse(src)\ncls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=="RemoteGrasp6DNode")\nfor name in %r:\n fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name==name)\n local={}\n exec(compile(textwrap.dedent(ast.get_source_segment(src,fn)),%r,"exec"),base.__dict__,local)\n setattr(base.RemoteGrasp6DNode,name,local[name])\nraise SystemExit(pytest.main(%r))\n''' % (str(old), names, str(old), args))
        command = ['/usr/bin/python3', str(boot)]
        baseline = dict(path=str(old), sha256=expected, rebound_methods=names,
            operational_source_files_modified=False)
    elif a.legacy_quota_ablation:
        boot = out/'legacy_quota_ablation.py'
        boot.write_text('''import pytest\nfrom test_remote_modes import base\nbase.RemoteGrasp6DNode._observation_success_first_search=lambda self:False\nraise SystemExit(pytest.main(%r))\n''' % args)
        command = ['/usr/bin/python3', str(boot)]
        baseline = dict(kind='discovery_off_ablation_not_exact_gen19_replay',
            rebound_methods=['_observation_success_first_search'],
            policy='existing legacy 3 second / 6 strict check quota branch',
            operational_source_files_modified=False)
    start = time.time()
    with (out/'pytest.log').open('w') as log:
        code = subprocess.call(command, cwd=str(W), env=env, stdout=log, stderr=subprocess.STDOUT)
    after = {s: sha(W/s) for s in files}
    suites = ET.parse(str(out/'pytest.xml')).getroot().findall('testsuite')
    counts = {k: sum(int(s.attrib.get(k, 0)) for s in suites)
              for k in ('tests', 'failures', 'errors', 'skipped')}
    counts['passed'] = counts['tests'] - counts['failures'] - counts['errors'] - counts['skipped']
    result = dict(kind='gen20_independent_offline_search', command=command,
        exit_code=code, elapsed_sec=time.time()-start, counts=counts,
        source_sha256_before=before, source_sha256_after=after,
        source_unchanged=before==after, baseline=baseline,
        artifact_sha256={n: sha(out/n) for n in ('pytest.xml', 'pytest.log')},
        ros_started=False, hardware_accessed=False, physical_grasp_verified=False)
    (out/'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(path=str(out), exit_code=code, counts=counts,
        source_unchanged=before==after, result_sha256=sha(out/'result.json'))))
    return code

if __name__ == '__main__':
    raise SystemExit(main())
