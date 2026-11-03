"""Install only reviewed GEN49 source bytes while the accepted GEN48 runtime is idle.

No ROS/motion API is called. A new full regression on installed bytes is still
required before compute-node reload. Original baseline bytes remain immutable.
"""
import argparse, hashlib, json, os, time
from pathlib import Path

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
CONTROL=E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
BASE=W/'.runtime/gen49_baseline_source/manifest.json'
BASE_SHA='21f6c6a1b0b8072d78332a081af1a425c570c9e5dd1983b2f982960e3911e935'
PRODUCTION={
 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_search_failure.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'}
ADDED={
 'src/alicia_grasp_modes/tests/test_gen49_contact_terminal_handoff.py',
 'src/alicia_grasp_modes/tests/test_gen49_contact_terminal_contract.py',
 'src/alicia_grasp_modes/tests/fixtures/gen49_g48_terminal_inputs.json'}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def write(p,value):
 with p.open('x') as f:
  json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())

def main():
 ap=argparse.ArgumentParser(description=__doc__)
 for name in ('manifest','focus','review','readiness'):
  ap.add_argument('--'+name,type=Path,required=True)
  ap.add_argument('--'+name+'-sha256',required=True)
 ap.add_argument('--output',type=Path,required=True)
 a=ap.parse_args()
 assert E.parent.parent.is_mount()
 out=a.output.resolve();assert V in out.parents and not out.exists()
 refs={}
 for name in ('manifest','focus','review','readiness'):
  path=getattr(a,name).resolve(strict=True);digest=getattr(a,name+'_sha256')
  assert E in path.parents and sha(path)==digest
  refs[str(path)]=digest
 assert sha(BASE)==BASE_SHA
 baseline=read(BASE)['source_sha256'];assert len(baseline)==1053
 manifest=read(a.manifest);focus=read(a.focus);review=read(a.review);ready=read(a.readiness)
 candidate=Path(manifest['workspace_root']).resolve(strict=True)
 assert candidate==W/'.runtime/gen49_integrated_candidate'
 source=manifest['source_sha256'];assert len(source)==1056 and set(source)-set(baseline)==ADDED
 assert {p for p in baseline if source[p]!=baseline[p]}==PRODUCTION
 assert all(sha(candidate/p)==h for p,h in source.items())
 assert all(sha(W/p)==h and sha(BASE.parent/p)==h for p,h in baseline.items())
 assert all(not (W/p).exists() for p in ADDED)
 assert focus['passed'] and focus['test_bodies_executed'] and focus['source_unchanged']
 assert focus['source_sha256']==source and focus['summary']['tests']>0
 assert all(focus['summary'][k]==0 for k in ('failures','errors','skipped'))
 assert not focus['external_boundary_attempts'] and not focus['foreign_or_inconsistent_imports']
 assert review['passed'] and review['source_sha256']==source
 assert ready['passed'] and ready['manual_mode'] and ready['task_active'] is False
 assert ready['deployment_processes_unchanged'] and ready['selection']['generation']==8
 assert ready['epoch_ns']==1792286012067798399 and ready['source_count']==1053
 assert 0<=time.time_ns()-ready['recorded_ns']<120_000_000_000
 assert ready['release']==str(V/'gen48_release_manifest.json')
 assert ready['deployment']==str(N/'gen48_runtime/gen48_runtime_deployment.json')
 control=read(CONTROL);assert control['state']=='active' and control['revision']==54
 refs[str(CONTROL)]=sha(CONTROL);refs[str(BASE)]=sha(BASE)
 refs[ready['release']]='72e26040a4052e3e49ee129a8b3a9de04af43947603fede05e0fa40992904de7'
 refs[ready['deployment']]='ac8e62dff7871105e436621b196ac4322290bc9641a33027945fec8385e73601'
 assert all(sha(p)==h for p,h in refs.items())
 for proc in {**read(ready['deployment'])['after'],**read(ready['deployment'])['preserved_after']}.values():
  stat=Path('/proc/%s/stat'%proc['pid']).read_text().rsplit(')',1)[1].split()
  assert stat[0]!='Z' and stat[19]==proc['start_ticks'] and int(stat[1])==proc['parent_pid']
 out.mkdir()
 write(out/'plan.json',dict(source_sha256=source,baseline_sha256=baseline,evidence_sha256=refs,
  changed_paths=sorted(PRODUCTION|ADDED),full_regression_passed=False,runtime_reloaded=False,
  motion_commands_sent=False,created_ns=time.time_ns()))
 for idx,rel in enumerate(sorted(PRODUCTION|ADDED)):
  target=W/rel;temp=target.with_name(target.name+'.gen49.install.tmp')
  assert not temp.exists() and all(sha(p)==h for p,h in refs.items())
  mode=target.stat().st_mode&0o777 if target.exists() else 0o644
  with temp.open('xb') as f:
   f.write((candidate/rel).read_bytes());f.flush();os.fsync(f.fileno())
  assert sha(temp)==source[rel]
  os.chmod(temp,mode);os.replace(temp,target)
  write(out/('installed_%02d.json'%idx),dict(path=rel,sha256=sha(target),recorded_ns=time.time_ns()))
 assert all(sha(W/p)==h for p,h in source.items())
 assert all(sha(p)==h for p,h in refs.items())
 result=dict(passed=True,source_count=len(source),source_sha256=source,
  changed_files={p:source[p] for p in sorted(PRODUCTION|ADDED)},
  original_baseline=str(BASE.parent),evidence_sha256=refs,
  full_regression_passed=False,runtime_reloaded=False,motion_commands_sent=False,
  temperature_queried=False,recorded_ns=time.time_ns(),helper_sha256=sha(__file__))
 write(out/'result.json',result)
 print(json.dumps(dict(passed=True,result=str(out/'result.json'),sha256=sha(out/'result.json'))))

if __name__=='__main__':main()
