#!/usr/bin/env python3
import ast,hashlib,json,os,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1];P=W/'.runtime/gen17_station_join_clock_pair_20261013';C=W/'.runtime/gen17_station_status_witness_atomic_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');O=N/'verification/gen17_station_status_witness_atomic_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
rel=N/'verification/gen16_success_first_release_manifest.json';sealed=json.loads(rel.read_text())['source_sha256'];assert all(sha(W/r)==v for r,v in sealed.items())
inherit=P/'delivery_manifest.json';assert sha(inherit)=='f24dde4f1f2384a258b30a15163f0fae9bf31b50d8b3c3a2859d76702d898b38'
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir();files={str(p.relative_to(C)):sha(p) for p in C.rglob('*.py')}
test=C/'src/alicia_grasp_modes/tests/test_station_status_witness_atomic.py'
SCOPE=['src/alicia_grasp_modes/tests/'+s for s in ('test_station_status_witness_atomic.py','test_station_join_clock_pair.py','test_station_contact_current_wait.py','test_station_compute_wait_task.py','test_station_requalification_task_finalize.py','test_station_requalification_three_owners.py','test_station_requalification_contract.py')]
def run(label,root,scope):
 out=O/label;out.mkdir();env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 env['ROS_PACKAGE_PATH']=str(root/'src')+':'+str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
 env['PYTHONPATH']=':'.join(str(root/r) for r in ('src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/tests','src/alicia_flexible_grasp_supervisor/tests'))+':'+str(W/'devel/lib/python3/dist-packages')+':'+env.get('PYTHONPATH','')
 args=['-q','-p','no:cacheprovider',*scope,'--disable-warnings','--tb=short','--basetemp='+str(out/'tmp'),'--junitxml='+str(out/'pytest.xml')]
 code='''import hashlib,json,sys,pytest
from pathlib import Path
import test_task_modes
from alicia_grasp_modes import station_compute_wait_task as wait
root=Path(sys.argv[1]);out=Path(sys.argv[2]);loaded={}
for name,m in [('station_task',wait),('mode_task',test_task_modes.module),('base_task',test_task_modes.base)]:
 p=Path(m.__file__).resolve();assert root==p or root in p.parents;(loaded.update({name:{'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}}))
(out/'loaded_sources.json').write_text(json.dumps(loaded,indent=2)+'\\n')
sys.exit(pytest.main(json.loads(sys.argv[3])))
'''
 with (out/'pytest.log').open('w') as log:p=subprocess.run(['python3','-c',code,str(root),str(out),json.dumps(args)],cwd=W,env=env,stdout=log,stderr=subprocess.STDOUT)
 x=ET.parse(out/'pytest.xml').getroot();counts={k:sum(int(s.attrib.get(k,0)) for s in x.iter('testsuite')) for k in ('tests','errors','failures','skipped')}
 r=dict(returncode=p.returncode,summary=counts,scope=scope,loaded_sources=str(out/'loaded_sources.json'),loaded_sha256=sha(out/'loaded_sources.json'),log_sha256=sha(out/'pytest.log'),xml_sha256=sha(out/'pytest.xml'));print(label,counts,flush=True);return r
baseline=run('unlocked_notice_baseline',P,[str(test)+'::test_notice_and_fresh_join_are_one_record_transaction[validated_parent]',str(test)+'::test_notice_and_fresh_join_are_one_record_transaction[checked_record]'])
focused=run('candidate_focused',C,[str(C/r) for r in SCOPE])
prod='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'
def unwrap_status_guard(text):
 tree=ast.parse(text);unwrapped=0
 for node in ast.walk(tree):
  if isinstance(node,ast.FunctionDef) and node.name=='_station_compute_task_status':
   for branch in node.body:
    if (isinstance(branch,ast.If) and branch.body and isinstance(branch.body[0],ast.With)):
     first=branch.body[0]
     assert len(first.items)==1 and ast.dump(first.items[0].context_expr)==ast.dump(ast.parse('self._grasp6d_plan_guard()',mode='eval').body)
     assert len(branch.body)==1 and len(first.body)==1 and isinstance(first.body[0],ast.Try)
     branch.body=first.body;unwrapped+=1
 return ast.dump(tree),unwrapped
old_ast,old_count=unwrap_status_guard((P/prod).read_text());new_ast,new_count=unwrap_status_guard((C/prod).read_text())
identical=old_ast==new_ast and old_count==0 and new_count==1

r=dict(passed=baseline['summary']==dict(tests=2,errors=0,failures=2,skipped=0) and focused['returncode']==0 and identical,baseline=baseline,focused=focused,candidate_root=str(C),candidate_sha256=files,focused_scope=SCOPE,
 inherited_manifest=str(inherit),inherited_manifest_sha256=sha(inherit),only_existing_witness_branch_guard_AST=identical,
 candidate_unchanged=all(sha(C/r)==v for r,v in files.items()),inherited_candidate_unchanged=all(sha(P/f['path'])==f['candidate_sha256'] for f in json.loads(inherit.read_text())['files']),actual_unchanged=all(sha(W/r)==v for r,v in sealed.items()),actual_source_count=len(sealed),release=str(rel),release_sha256=sha(rel),runner_sha256=sha(__file__),recorded_ns=time.time_ns(),live_ros_contacted=False,motion_commands_sent=False,physical_grasp_verified=False)
r['passed'] &=r['candidate_unchanged'] and r['inherited_candidate_unchanged'] and r['actual_unchanged'];(O/'result.json').write_text(json.dumps(r,indent=2)+'\n');print('RESULT',r['passed'],sha(O/'result.json'),flush=True);assert r['passed']
