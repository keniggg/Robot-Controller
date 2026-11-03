import ast, hashlib, json, shutil
from pathlib import Path
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen17_frozen_station_retirement_20261013'
R=W/'.runtime/gen17_remote_receipt_current_wait_r2_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen17_frozen_station_retirement_delivery';O.mkdir()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
result=N/'verification/gen17_frozen_station_retirement_r4/result.json';r=json.loads(result.read_text())
assert r['passed'] and r['source_unchanged'] and r['actual_unchanged'] and r['summary']['tests']>22
assert all(sha(C/p)==h for p,h in r['source_sha256'].items())
actual=json.loads((N/'verification/gen16_success_first_release_manifest.json').read_text())['source_sha256']
assert all(sha(W/p)==h for p,h in actual.items())
rel='src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_remote.py'
a,b=ast.parse((W/rel).read_text()),ast.parse((C/rel).read_text())
f=next(x for x in ast.walk(a) if isinstance(x,ast.FunctionDef) and x.name=='_contact_station_context')
g=next(x for x in ast.walk(b) if isinstance(x,ast.FunctionDef) and x.name=='_contact_station_live_context')
old=next(x for x in f.body if isinstance(x,ast.If));new=next(x for x in g.body if isinstance(x,ast.If))
removed=[x for x in old.test.values if isinstance(x,ast.Attribute) and x.attr=='_station_compute_pending'];assert len(removed)==1
old.test.values=[x for x in old.test.values if x not in removed]
assert ast.dump(old,include_attributes=False)==ast.dump(new,include_attributes=False)
loaded=[]
for p in sorted(result.parent.joinpath('tmp').rglob('loaded_sources.json')):
 vals=json.loads(p.read_text());assert all(sha(p)==h for p,h in vals.items());loaded.append(dict(path=str(p),sha256=sha(p),source_sha256=vals))
assert len(loaded)==22,len(loaded)
files=[]
for path in [rel,'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py','src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py']:
 files.append(dict(path=path,base_sha256=sha(W/path) if (W/path).exists() else None,candidate_sha256=sha(C/path)))
prior=R/'delivery_manifest.json';pm=json.loads(prior.read_text())
assert sha(prior)=='f241068ad731d81c7d0b00c9306cec706f72b0c7b3934f2f7813b43c69408482'
for p in files:
 dst=O/'candidate_files'/p['path'];dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(C/p['path'],dst);assert sha(dst)==p['candidate_sha256']
review=dict(base_live_context_ast_identical_except_pending=True,removed_pending_attributes=len(removed),
    loaded_sources=loaded,source_freeze_result=str(result),source_freeze_sha256=sha(result),
    actual_sources=len(actual),actual_unchanged=True,no_ros_master=True,no_live_actions=True)
(O/'review_evidence.json').write_text(json.dumps(review,indent=2)+'\n')
manifest=dict(schema=1,kind='gen17_frozen_station_retirement_history',passed=True,candidate_root=str(C),files=files,
 test_scope=pm['test_scope']+['src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py'],
 inherits=[dict(manifest=str(prior),sha256=sha(prior),remote_base_sha256=sha(R/files[1]['path']),
    rationale='Final remote file includes sealed receipt/current-wait fixes plus this retirement-only increment; merge baseline remains actual GEN16.')],
 validation=dict(result=str(result),sha256=sha(result),summary=r['summary'],new_boundary_tests=22,review_evidence=str(O/'review_evidence.json'),review_evidence_sha256=sha(O/'review_evidence.json')),
 baseline_defect=dict(result=str(N/'verification/gen17_frozen_station_handoff_probe_r2/result.json'),sha256=sha(N/'verification/gen17_frozen_station_handoff_probe_r2/result.json'),two_schedules_reproduced=True),
 preserved_failures=[dict(path=str(N/('verification/'+d+'/result.json')),sha256=sha(N/('verification/'+d+'/result.json'))) for d in ['gen17_frozen_station_handoff_probe_r1','gen17_frozen_station_retirement_r1','gen17_frozen_station_retirement_r3']],
 semantics=['Only exact previously published and atomically audited contact history survives a pure clock gap when MOVE exits stationary scope.',
 'The local query has no request_id, state or ACK. It is not proof of task freeze, new registration, geometry or motion authority.',
 'Reference is disarmed; pending stays true; work certificate stays false. Actual parsed prepare and retire repeat original owner/phase/session/manual/deadline checks and exact audited plan matching.',
 'Real status loss, first_loss, false object, empty mask and historical source contradictions still revoke held history; existing canceled-source and tombstone filtering order is preserved.',
 'No new deadline, motion retry, live ROS call or actual source mutation.'],
 limitations=['Offline real protocol/freeze/atomic audit/retirement orchestration with synthetic RGBD, translated recorded geometry, supplied geometry audit report, inert transport and one action sentinel. No new collision solution, hardware execution or grasp success is claimed.',
 'Source-age and identity cache deadlines are not reused as execution deadlines after stationary scope ends; original phase/session/owner and exact transaction deadlines remain mandatory.'],
 deployed=False,physical_grasp_success=False)
(C/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
shutil.copyfile(C/'delivery_manifest.json',O/'delivery_manifest.json')
for p in [Path(__file__),W/'.runtime/run_gen17_frozen_station_retirement_20261013.py']:
 shutil.copyfile(p,O/p.name)
receipt=dict(passed=True,candidate_root=str(C),manifest_sha256=sha(C/'delivery_manifest.json'),files=files,validation=manifest['validation'],actual_unchanged=True,actual_sources=len(actual))
(O/'result.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(dict(manifest=str(C/'delivery_manifest.json'),sha256=sha(C/'delivery_manifest.json'),receipt=str(O/'result.json'),receipt_sha256=sha(O/'result.json'),summary=r['summary'])))
