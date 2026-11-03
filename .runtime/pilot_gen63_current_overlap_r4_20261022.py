"""Finite GEN63 contract run; frozen GEN62 hardlinks plus isolated copied overlay.
Unchanged GEN57 worker, offline guard and import audit. Never a full run.
"""
import argparse,hashlib,importlib.util,json,os,subprocess,sys,time
from pathlib import Path
from copy import deepcopy
from collections import Counter
from types import SimpleNamespace
W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
G=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1';F=G/'contact_feedback_fix_r1';V=F/'handoff_recovery_verification_r1';C=W/'.runtime/handoff_recovery_candidate'
BM=G/'v62/freeze_r1/manifest_closed.json';RELEASE=N/'verification/gen62_release_manifest.json';SOURCE_MAP=F/'handoff_recovery_fix_r1/candidate_r3_source_map.json'
OLD=W/'.runtime/verify_gen57_parallel_offline_r2_20261020.py';OLD_SHA='0e217bc784b24c603afa1f4bca7c977f35f9036c03b6b263c9801f749d1c2727'
P='src/alicia_grasp_modes/tests/';NEW=P+'test_gen63_station_source_overlap.py';EDIT=P+'test_gen61_seed_insufficiency.py'
ENTRY=P+'test_gen63_station_submit_gate.py'
TESTS=[ENTRY]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def save(p,v):
 with Path(p).open('x') as h:json.dump(v,h,ensure_ascii=False,indent=2,allow_nan=False);h.write('\n');h.flush();os.fsync(h.fileno())
assert sha(OLD)==OLD_SHA
sp=importlib.util.spec_from_file_location('gen63_unchanged_worker',OLD);O=importlib.util.module_from_spec(sp);sp.loader.exec_module(O);original_load=O.load_serial

def bind(path):
 m=read(path);root=Path(m['snapshot']);out=Path(m['output']);assert m['runner_sha256']==sha(__file__) and m['infrastructure_sha256']==OLD_SHA
 assert root.parent==W/'.runtime' and root.name.startswith('gen63_source_overlap_pilot_');assert out.parent==V
 def serial_load():
  s=original_load();s.V=out;return s
 def stable(serial,plan):
  return (sha(__file__)==plan['runner_sha256']==m['runner_sha256'] and sha(OLD)==OLD_SHA and sha(path)==plan['candidate_manifest_sha256'] and all(sha(p)==h for p,h in m['dependencies'].items()) and all(sha(root/p)==h for p,h in m['copied_input_sha256'].items()) and all(sha(W/p)==h for p,h in m['live_source_sha256'].items()) and all(sha(p)==h for p,h in m['context']['generated_imports'].items()) and plan['context']==m['context'])
 O.W=root;O.PROJECT=W;O.load_serial=serial_load;O.stable=stable;return m

def budget(workers):
 return O.resource_budget(SimpleNamespace(worker_cap=2,reserve_cpus=5,memory_per_worker_mib=1024,reserve_memory_mib=2048,workers=str(workers)))
def freeze(revision):
 assert revision == 'r4'
 assert sha(BM)=='940c5256cb302d6127bf8b6d693aa2e5b03a283e6a58e43a272a6ace23511487'
 assert sha(RELEASE)=='e0d5e33bd9add84b4734beb53441dababddceb88c525cbbd6d07477a914a41cf'
 bm=read(BM);release=read(RELEASE);baseline=Path(bm['workspace_root']);old=bm['copied_input_sha256'];assert len(old)==1115 and bm['total_case_count']==10213
 assert release['passed'] and release['installed'];assert Path(release['candidate_manifest'])==BM and release['candidate_manifest_sha256']==sha(BM);live=release['source_sha256'];assert len(live)==1103
 assert all(sha(baseline/p)==h for p,h in old.items());assert all(sha(W/p)==h for p,h in live.items())
 prod=read(SOURCE_MAP)['files'];overlay={str(p.relative_to(C)):sha(p) for p in C.rglob('*') if p.is_file()};assert set(overlay)==set(prod)|{NEW,EDIT,ENTRY};assert all(overlay[p]==h for p,h in prod.items())
 before=(baseline/EDIT).read_text();after=(C/EDIT).read_text();needle="    assert type(caught.value) is ValueError\n";pos=before.index('def test_actual_foreign_component_overlap_remains_a_hard_original_rejection');end=before.index('\n\ndef ',pos);expected=before[:pos]+before[pos:end].replace(needle,"    assert type(caught.value) is owner.CurrentSourceOwnershipConflict\n")+before[end:];assert after==expected
 actual=dict(old);actual.update(overlay);context=deepcopy(bm['context']);sources=dict(bm['source_sha256']);sources.update(overlay);context.update(sources=sources,required_changed_sources=sorted(prod),scope=TESTS)
 root=W/('.runtime/gen63_source_overlap_pilot_'+revision);out=V/('pilot_'+revision);assert not root.exists() and not out.exists();b=budget(2);assert b['execution_resources_sufficient']
 serial=original_load();serial.V=F;serial.storage(V);V.mkdir(exist_ok=True);serial.storage(out);out.mkdir();root.mkdir()
 for p,h in sorted(actual.items()):
  dst=root/p;dst.parent.mkdir(parents=True,exist_ok=True)
  if p in overlay:
   data=(C/p).read_bytes();assert hashlib.sha256(data).hexdigest()==h
   with dst.open('xb') as f:f.write(data)
  else:os.link(baseline/p,dst)
 assert all(sha(root/p)==h for p,h in actual.items());assert overlay=={str(p.relative_to(C)):sha(p) for p in C.rglob('*') if p.is_file()}
 m=dict(kind='GEN63_finite_frozen_overlay_contract_pilot',snapshot=str(root),output=str(out),runner_sha256=sha(__file__),infrastructure_sha256=OLD_SHA,dependencies={str(BM):sha(BM),str(RELEASE):sha(RELEASE),str(SOURCE_MAP):sha(SOURCE_MAP)},copied_input_sha256=actual,live_source_sha256=live,context=context,production_paths=sorted(prod),overlay_sha256=overlay,baseline=str(baseline),baseline_case_count=10213,resource_budget=b,hardlinked_baseline_files=len(old)-len(set(old)&set(overlay)),source_writes_denied_by_original_guard=True,overlay_copied_not_hardlinked=True,intentional_prior_test_change=EDIT,new_tests=[NEW,ENTRY],full_regression_passed=False)
 save(out/'manifest.json',m);print(json.dumps(dict(frozen=str(out/'manifest.json'),sha256=sha(out/'manifest.json'))),flush=True);return out/'manifest.json'

def phase(mp,mode,cases):
 m=bind(mp);out=Path(m['output'])/mode;out.mkdir();groups=[TESTS] if mode=='collect' else [g for g in [TESTS[:3],TESTS[3:]] if g];b=budget(len(groups));assert b['execution_resources_sufficient'];shards=[]
 for group in groups:
  selected=[c for c in cases if c[0] in group];shards.append(dict(scope=group,case_identities=selected,expected_count=len(selected),required_changed_sources=m['production_paths'] if mode!='collect' else []))
 plan=dict(output=str(out),mode=mode,worker_count=len(shards),context=m['context'],runner_sha256=sha(__file__),candidate_manifest_sha256=sha(mp),candidate_manifest_path=str(mp),test_scope=TESTS,scope_kind='finite_contract_pilot',full_regression_scope=False,expected_counts_by_file=dict(Counter(c[0] for c in cases)),shards=shards,resource_budget=b);pp=out/'plan.json';save(pp,plan)
 monitor=O.WorkloadMonitor(out);processes=[];logs=[];ids=[];started=time.monotonic()
 try:
  for i in range(len(shards)):
   log=(out/('coordinator_child_%d.log'%i)).open('x');logs.append(log)
   proc=subprocess.Popen([sys.executable,'-B',str(Path(__file__).resolve()),'--worker-plan',str(pp),'--index',str(i)],cwd=W,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);ids.append(O.process_stat(proc.pid))
  while any(p.poll() is None for p in processes):monitor.sample(processes);time.sleep(.2)
  exits=[p.wait() for p in processes]
 finally:
  for p in processes:
   if p.poll() is None:p.terminate()
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  monitor.sample(processes,force=True);mon=monitor.close()
  for h in logs:h.close()
 residual=[]
 for entry in Path('/proc').iterdir():
  if entry.name.isdecimal():
   r=O.process_stat(entry.name)
   if r and r['pgid'] in [p.pid for p in processes]:residual.append(r)
 results=[read(out/('worker_%d/result.json'%i)) for i in range(len(shards))]
 actual=[];summary=Counter()
 for i,r in enumerate(results):
  if mode=='collect':actual.extend(r['collected_case_identities'])
  else:
   su,ci,_=O.inventory(out/('worker_%d/pytest.xml'%i));actual.extend(ci);summary.update(su)
 unique=all(v==1 for v in Counter(map(tuple,actual)).values());scope=unique and (mode=='collect' or Counter(map(tuple,actual))==Counter(map(tuple,cases)))
 rec=dict(passed=all(x==0 for x in exits) and all(r['passed'] for r in results) and not residual and scope,returncodes=exits,worker_identities=ids,residual_owned_processes=residual,worker_result_sha256={str(out/('worker_%d/result.json'%i)):sha(out/('worker_%d/result.json'%i)) for i in range(len(results))},case_identities=actual,summary=dict(summary),exact_inventory=scope,source_unchanged=all(r.get('source_unchanged') for r in results),monitoring=mon,elapsed_sec=time.monotonic()-started,full_regression_passed=False)
 save(out/'result.json',rec);print(json.dumps(dict(phase=mode,passed=rec['passed'],summary=rec['summary'],case_count=len(actual),output=str(out))),flush=True);return rec

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--revision');ap.add_argument('--worker-plan',type=Path);ap.add_argument('--index',type=int);args=ap.parse_args()
 if args.worker_plan:plan=read(args.worker_plan);bind(Path(plan['candidate_manifest_path']));return O.worker(args.worker_plan,args.index)
 mp=freeze(args.revision);col=phase(mp,'collect',[])
 if not col['passed']:return 1
 result=phase(mp,'targeted',col['case_identities']);m=read(mp);new=[c for c in col['case_identities'] if c[0] in (NEW,ENTRY)]
 save(mp.parent/'result.json',dict(passed=result['passed'],new_case_count=len(new),total_case_count=len(col['case_identities']),manifest_sha256=sha(mp),collection_result_sha256=sha(mp.parent/'collect/result.json'),targeted_result_sha256=sha(mp.parent/'targeted/result.json'),summary=result['summary'],exact_inventory=result['exact_inventory'],source_unchanged=result['source_unchanged'],full_regression_passed=False,case_identities=col['case_identities']))
 return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
