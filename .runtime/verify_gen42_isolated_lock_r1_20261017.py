"""Run source-bound GEN42 experiments in an isolated workspace, never live ROS."""
import argparse
import ast
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
E = M/'实验记录/Robot-Controller'
MANIFEST = W/'.runtime/gen41_baseline_source/manifest.json'
MANIFEST_SHA = '61c2533a61070079fc58e18fade39eb4fa5ff673828ca1b3663d587b4c6994d2'
EXTRA_DEPENDENCIES = {'tools/run_fresh_grasp6d_after_alignment.py':
    '8d750485b6166823486c2933579ef3faef0640a4a323c6b6c2554a42a7a0c45b'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    with path.open('x') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())


def storage(out):
    assert M.is_mount() and (E/'ros_log').resolve() in out.parents
    assert out.parent.stat().st_dev == M.stat().st_dev
    c=json.loads((E/'grasp_records/archive_config.json').read_text())
    s=json.loads((E/'maintenance/storage_control.json').read_text())
    assert c['upload_enabled'] is False and c['delete_enabled'] is False
    assert s['archive_policy']=='local_only' and shutil.disk_usage(E).free>=c['min_free_bytes']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--test',required=True,action='append')
    p.add_argument('--select',help='Explicit pytest -k selection; recorded with the scope')
    a=p.parse_args(); work=a.workspace.resolve(strict=True); out=a.output.resolve()
    assert work in (W/'.runtime/gen42_lock_before',W/'.runtime/gen42_lock_candidate')
    storage(out); assert not out.exists() and sha(MANIFEST)==MANIFEST_SHA
    baseline=json.loads(MANIFEST.read_text())['source_sha256']; assert len(baseline)==1032
    # Real sources cannot change while the existing manual stack is running.
    assert all(sha(W/k)==v for k,v in baseline.items())
    scope=list(dict.fromkeys(a.test)); assert len(scope)==len(a.test)
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts
        and '/tests/test_' in n and (work/n).is_file() for n in scope)
    sources={k:sha(work/k) for k in baseline}
    assert all(sha(W/k)==v and sha(work/k)==v for k,v in EXTRA_DEPENDENCIES.items())
    sources.update(EXTRA_DEPENDENCIES)
    sources.update({k:sha(work/k) for k in scope})
    changed=[k for k,v in baseline.items() if sources[k]!=v]
    assert set(changed)<= {'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'}
    if work.name=='gen42_lock_before':assert not changed
    out.mkdir()
    for n in ('tmp','ros','bootstrap'):(out/n).mkdir()
    save(out/'scope.json',dict(workspace=str(work),tests=scope,selection=a.select,source_sha256=sources,
        changed_from_gen41=changed,extra_dependencies_sha256=EXTRA_DEPENDENCIES,
        baseline_manifest_sha256=MANIFEST_SHA,runner_sha256=sha(__file__)))
    # Reuse the byte-reviewed GEN41 Python socket/device guard, without running
    # the old test runner or importing its ROS dependencies.
    guard_source=W/'.runtime/verify_gen41_offline_tests_20261017.py'
    assert sha(guard_source)=='0c7c13321f4eed998d54f3ff49456179c7c4c28deb193e9ab646113554de21c9'
    tree=ast.parse(guard_source.read_text())
    writes=[n for n in ast.walk(tree) if isinstance(n,ast.Call)
        and isinstance(n.func,ast.Attribute) and n.func.attr=='write_text'
        and isinstance(n.func.value,ast.Name) and n.func.value.id=='bootstrap']
    assert len(writes)==1
    boot=ast.literal_eval(writes[0].args[0]); assert 'sys.addaudithook(device_open)' in boot
    (out/'bootstrap/sitecustomize.py').write_text(boot)
    # Provenance also checks the loaded original base and mode wrappers. A
    # correct result from the live workspace cannot stand in for this candidate.
    plugin="""import json,os,sys
from pathlib import Path
def pytest_sessionfinish(session,exitstatus):
    rows={n:str(Path(m.__file__).resolve()) for n,m in list(sys.modules.items())
          if getattr(m,'__file__',None) and
          ('remote_grasp6d' in n or n.startswith('alicia_grasp_modes') or n.startswith('alicia_flexible_grasp'))}
    Path(os.environ['GEN42_MODULE_PROVENANCE']).write_text(json.dumps(rows,indent=2))
"""
    (out/'bootstrap/gen42_import_audit.py').write_text(plugin)
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
        PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(out/'ros'),TMPDIR=str(out/'tmp'),
        OFFLINE_BOUNDARY_LOG=str(out/'boundary_attempts.jsonl'),
        GEN42_MODULE_PROVENANCE=str(out/'module_provenance.json'))
    paths=[out/'bootstrap',work/'src/alicia_flexible_grasp_supervisor/tests',
        work/'src/alicia_grasp_modes/tests',work/'src/alicia_grasp_modes/src',
        work/'src/alicia_flexible_grasp_supervisor/src',W/'devel/lib/python3/dist-packages']
    env['PYTHONPATH']=':'.join(map(str,paths))+':'+env.get('PYTHONPATH','')
    env['ROS_PACKAGE_PATH']=str(work/'src')+':/opt/ros/noetic/share'
    command=[sys.executable,'-m','pytest','-q','--rootdir='+str(work),*scope,
        '-p','no:cacheprovider','-p','gen42_import_audit','--tb=short',
        '--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml'),'-o','junit_family=legacy']
    if a.select:command.extend(['-k',a.select])
    started=time.monotonic(); runner_sha=sha(__file__)
    print(json.dumps(dict(started=True,workspace=str(work),output=str(out),scope=scope)),flush=True)
    with (out/'pytest.log').open('x') as f:
        run=subprocess.run(command,cwd=work,env=env,stdout=f,stderr=subprocess.STDOUT)
    summary={k:0 for k in ('tests','failures','errors','skipped')}; counts=dict.fromkeys(scope,0)
    unassigned=[]
    if (out/'pytest.xml').exists():
        tree=ET.parse(out/'pytest.xml').getroot()
        for suite in tree.iter('testsuite'):
            for k in summary:summary[k]+=int(suite.get(k,0))
        for case in tree.iter('testcase'):
            n=case.get('file','')
            if n in counts:counts[n]+=1
            else:unassigned.append(dict(file=n,name=case.get('name')))
    provenance=json.loads((out/'module_provenance.json').read_text()) if (out/'module_provenance.json').exists() else {}
    base=str(work/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py')
    original_loaded=base in provenance.values()
    cross_workspace=[(n,f) for n,f in provenance.items() if str(W/'src')+'/' in f]
    unchanged=all(sha(work/k)==v for k,v in sources.items()) and all(sha(W/k)==v for k,v in baseline.items())
    unchanged=unchanged and all(sha(W/k)==v for k,v in EXTRA_DEPENDENCIES.items())
    unchanged=unchanged and sha(__file__)==runner_sha and sha(MANIFEST)==MANIFEST_SHA
    attempts=(out/'boundary_attempts.jsonl').read_text().splitlines() if (out/'boundary_attempts.jsonl').exists() else []
    complete=not unassigned and all(counts.values()) and sum(counts.values())==summary['tests']
    passed=(run.returncode==0 and unchanged and complete and original_loaded and not cross_workspace
        and not attempts and summary['tests']>0 and all(summary[k]==0 for k in ('failures','errors','skipped')))
    result=dict(passed=passed,summary=summary,returncode=run.returncode,elapsed_sec=time.monotonic()-started,
        source_count=len(sources),source_sha256=sources,changed_from_gen41=changed,source_unchanged=unchanged,
        tests=scope,selection=a.select,test_case_counts_by_file=counts,scope_complete=complete,unassigned=unassigned,
        original_base_loaded_from_isolated_workspace=original_loaded,cross_workspace_imports=cross_workspace,
        external_boundary_attempts=attempts,command=command,runner_sha256=runner_sha,
        guard_source_sha256=sha(guard_source),log_sha256=sha(out/'pytest.log'),
        bootstrap_sha256=sha(out/'bootstrap/sitecustomize.py'),
        import_audit_sha256=sha(out/'bootstrap/gen42_import_audit.py'),
        external_guard_scope='Python socket and CPython device-open audit; native SDK factories require inert fixtures',
        extra_dependencies_sha256=EXTRA_DEPENDENCIES,
        xml_sha256=sha(out/'pytest.xml') if (out/'pytest.xml').exists() else None,
        live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,
        deployed=False,physical_grasp_verified=False)
    storage(out);save(out/'result.json',result)
    print(json.dumps(dict(passed=passed,summary=summary,output=str(out))),flush=True)
    raise SystemExit(0 if passed else 1)


if __name__=='__main__':main()
