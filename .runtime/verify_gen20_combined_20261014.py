"""Run the GEN19 regression plus GEN20 observation search coverage; all logs stay on SanStand."""
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
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
N = E / 'ros_log/full_latest_gen18_restart_20261014T012216Z'
BASE = N / 'verification/gen19_release_manifest.json'
PRODUCTION = (
    'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py',
    'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
    'src/alicia_flexible_grasp_supervisor/config/joint_limits.yaml',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py',
)
CHANGED_TESTS = (
    'src/alicia_grasp_modes/tests/test_silhouette_observation_coverage.py',
    'src/alicia_grasp_modes/tests/test_observation_schedule_wrapper_handoff.py',
    'src/alicia_grasp_modes/tests/test_observation_fk_first_strict.py',
    'src/alicia_grasp_modes/tests/test_information_first_path_coverage.py',
)
NEW_TESTS = (
    'src/alicia_grasp_modes/tests/test_gen20_cached_observation_promotion.py',
    'src/alicia_grasp_modes/tests/test_gen20_observation_search_independent.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen20_observation_planning_error.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_gen20_observation_planning_budget.py',
    'src/alicia_flexible_grasp_supervisor/tests/test_observation_success_first_search.py',
)
RELATED = ('src/alicia_grasp_modes/tests/test_information_first_path_coverage.py',)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert N / 'verification' in out.parents and out.parent.stat().st_dev == M.stat().st_dev
    config = json.loads((E/'grasp_records/archive_config.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert json.loads((E/'maintenance/storage_control.json').read_text())['archive_policy'] == 'local_only'
    assert shutil.disk_usage(E).free >= config['min_free_bytes']
    assert sha(BASE) == '804f2d468ba180185aab93af1c4135393384511b25a15a574430aabda09e4570'
    baseline = json.loads(BASE.read_text())
    sources = dict(baseline['source_sha256'])
    for name, digest in sources.items():
        if name not in PRODUCTION + CHANGED_TESTS:
            assert sha(W/name) == digest, name
    for name in PRODUCTION + CHANGED_TESTS + NEW_TESTS:
        sources[name] = sha(W/name)
    scope = list(dict.fromkeys(baseline['test_scope'] + list(RELATED) + list(NEW_TESTS)))
    for name in scope:
        assert (W/name).is_file(), name
        if name not in sources:
            sources[name] = sha(W/name)
    out.mkdir(exist_ok=False)
    for name in ('tmp','ros'):
        (out/name).mkdir()
    patch = dict(baseline=str(BASE), baseline_sha256=sha(BASE),
        files={name:dict(baseline_sha256=baseline['source_sha256'].get(name),sha256=sources[name])
               for name in PRODUCTION+CHANGED_TESTS+NEW_TESTS})
    (out/'patch_manifest.json').write_text(json.dumps(patch,ensure_ascii=False,indent=2)+'\n')
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(out/'tmp'), ROS_LOG_DIR=str(out/'ros'))
    env['PYTHONPATH'] = ':'.join(str(W/p) for p in (
        'src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests',
        'src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src',
        'devel/lib/python3/dist-packages')) + ':' + env.get('PYTHONPATH','')
    env['ROS_PACKAGE_PATH'] = str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
    command = [sys.executable,'-m','pytest','-q',*scope,'-p','no:cacheprovider','--tb=short',
        '--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml')]
    print(json.dumps(dict(started=True,source_count=len(sources),test_files=len(scope),output=str(out))),flush=True)
    start = time.monotonic()
    with (out/'pytest.log').open('x') as log:
        completed = subprocess.run(command,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
    suites = list(ET.parse(out/'pytest.xml').getroot().iter('testsuite')) if (out/'pytest.xml').exists() else []
    summary = {key:sum(int(s.get(key,0)) for s in suites) for key in ('tests','failures','errors','skipped')}
    unchanged = all(sha(W/name)==digest for name,digest in sources.items())
    passed = completed.returncode==0 and unchanged and summary['tests']>=baseline['test_summary']['tests'] and all(summary[k]==0 for k in ('failures','errors','skipped'))
    result = dict(passed=passed,summary=summary,returncode=completed.returncode,
        elapsed_sec=time.monotonic()-start,source_sha256=sources,source_count=len(sources),
        source_unchanged=unchanged,test_scope=scope,test_scope_files=len(scope),command=command,
        baseline=str(BASE),baseline_sha256=sha(BASE),patch=str(out/'patch_manifest.json'),
        patch_sha256=sha(out/'patch_manifest.json'),log_sha256=sha(out/'pytest.log'),
        xml_sha256=sha(out/'pytest.xml') if suites else None,runner_sha256=sha(__file__),
        production_worktree_tested=True,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
    assert M.is_mount() and out.stat().st_dev==M.stat().st_dev
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,summary=summary,result=str(out/'result.json'))),flush=True)
    return 0 if passed else 1

if __name__ == '__main__':
    raise SystemExit(main())
