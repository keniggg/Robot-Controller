"""Isolated, bounded pytest runner for the manual/contact handoff repair."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1'
PLUGIN = '''import json, os
from pathlib import Path
items=[]
reports=[]
def pytest_collection_finish(session):
    items.extend(item.nodeid for item in session.items)
def pytest_runtest_logreport(report):
    reports.append(dict(nodeid=report.nodeid, when=report.when, outcome=report.outcome, duration=report.duration))
def pytest_sessionfinish(session, exitstatus):
    Path(os.environ['HANDOFF_CASES']).write_text(json.dumps(dict(items=items,reports=reports,exitstatus=exitstatus),indent=2))
'''


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--name', required=True)
    p.add_argument('--jobs', type=int, default=2)
    p.add_argument('tests', nargs='+')
    a = p.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert 1 <= a.jobs <= 2 and Path(a.name).name == a.name
    assert len(a.tests) == len(set(a.tests))
    assert all((W/t.split('::')[0]).is_file() for t in a.tests)
    out = R/a.name; out.mkdir()
    source = {str(f.relative_to(W)): sha(f) for folder in (
        W/'src/alicia_flexible_grasp_supervisor', W/'src/alicia_grasp_modes') for f in folder.rglob('*.py')}
    rows = [[] for _ in range(min(a.jobs, len(a.tests)))]
    # When available, balance by actual preceding focused-run durations.
    history = {}
    for f in sorted(R.glob('*/worker_*/cases.json'), key=lambda f:f.stat().st_mtime_ns):
        for report in json.loads(f.read_text())['reports']:
            key=report['nodeid'].split('::')[0]
            history[key]=history.get(key, 0.)+report['duration']
    totals = [0.]*len(rows)
    for test in sorted(a.tests, key=lambda t:history.get(t.split('::')[0], 1.), reverse=True):
        i = min(range(len(rows)), key=lambda i: totals[i])
        rows[i].append(test); totals[i] += history.get(test.split('::')[0], 1.)
    save(out/'manifest.json', dict(source_sha256=source, runner_sha256=sha(__file__), tests=a.tests,
        shards=rows, estimated_shard_seconds=totals, jobs=len(rows), affinity=sorted(os.sched_getaffinity(0)),
        meminfo=Path('/proc/meminfo').read_text(), host_load=os.getloadavg()))
    children=[]; started=time.monotonic()
    for i, tests in enumerate(rows):
        d=out/('worker_'+str(i)); d.mkdir()
        for name in ('bootstrap','tmp','ros','ros_home','cache'): (d/name).mkdir()
        (d/'bootstrap/sitecustomize.py').write_bytes((R/'guard.py').read_bytes())
        (d/'bootstrap/handoff_import_audit.py').write_bytes((R/'import_audit.py').read_bytes())
        (d/'bootstrap/handoff_cases.py').write_text(PLUGIN)
        env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
            PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
            MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1',
            LD_LIBRARY_PATH='/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu',
            ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(d/'ros'), ROS_HOME=str(d/'ros_home'),
            TMPDIR=str(d/'tmp'), TMP=str(d/'tmp'), TEMP=str(d/'tmp'), XDG_CACHE_HOME=str(d/'cache'),
            OFFLINE_BOUNDARY_LOG=str(d/'boundary.jsonl'), HANDOFF_CASES=str(d/'cases.json'),
            GEN42_MODULE_PROVENANCE=str(d/'imports.json'),
            ROS_PACKAGE_PATH=str(W/'src')+':/opt/ros/noetic/share')
        for k in ('PYTEST_ADDOPTS','PYTEST_PLUGINS'):env.pop(k,None)
        env['PYTHONPATH']=':'.join(map(str, [d/'bootstrap',
            W/'src/alicia_flexible_grasp_supervisor/tests', W/'src/alicia_grasp_modes/tests',
            W/'src/alicia_grasp_modes/src', W/'src/alicia_flexible_grasp_supervisor/src',
            W/'devel/lib/python3/dist-packages', Path('/opt/ros/noetic/lib/python3/dist-packages')]))
        command=['/usr/bin/time','-v','-o',str(d/'resources.txt'),sys.executable,'-B','-m','pytest',
            '-q','-p','no:cacheprovider','-p','handoff_import_audit','-p','handoff_cases',
            '--basetemp='+str(d/'pytest_tmp'),'--junitxml='+str(d/'pytest.xml'),'--tb=short',*tests]
        log=(d/'pytest.log').open('w')
        child=subprocess.Popen(command,cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((child,log,d))
    print(json.dumps(dict(started=True, output=str(out), workers=len(children))),flush=True)
    while any(child.poll() is None for child,_,_ in children):
        time.sleep(.3)
    workers=[]; collected=[]; executed=[]; passed=[]; skipped=[]; boundaries=[]
    for child,log,d in children:
        log.close()
        cases=json.loads((d/'cases.json').read_text()) if (d/'cases.json').exists() else dict(items=[],reports=[])
        collected.extend(cases['items'])
        executed.extend(r['nodeid'] for r in cases['reports'] if r['when']=='call')
        setup_skips=[r['nodeid'] for r in cases['reports'] if r['when']=='setup' and r['outcome']=='skipped']
        executed.extend(setup_skips)
        skipped.extend(r['nodeid'] for r in cases['reports'] if r['when'] in ('setup','call') and r['outcome']=='skipped')
        passed.extend(r['nodeid'] for r in cases['reports'] if r['when']=='call' and r['outcome']=='passed')
        if (d/'boundary.jsonl').exists():boundaries.extend((d/'boundary.jsonl').read_text().splitlines())
        workers.append(dict(path=str(d),exitcode=child.returncode,resources=(d/'resources.txt').read_text()))
    unchanged=all(sha(W/f)==h for f,h in source.items())
    result=dict(passed=all(c.returncode==0 for c,_,_ in children) and unchanged and not boundaries
                    and len(collected)==len(set(collected)) and Counter(collected)==Counter(executed),
        wall_sec=time.monotonic()-started, collected_count=len(collected), passed_count=len(passed), skipped_count=len(skipped), skipped_cases=skipped,
        exact_execution=Counter(collected)==Counter(executed), no_duplicate_cases=len(collected)==len(set(collected)),
        full_repository_regression=False, source_unchanged=unchanged, external_boundary_attempts=boundaries,
        source_sha256=source, workers=workers)
    save(out/'result.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','workers')}),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':sys.exit(main())
