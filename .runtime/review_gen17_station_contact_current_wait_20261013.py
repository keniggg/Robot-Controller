#!/usr/bin/env python3
"""Independent read-only review of the sealed task-side current-wait fix."""
from pathlib import Path
import ast,hashlib,json,time,xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification')
C=W/'.runtime/gen17_station_contact_current_wait_20261013';M=C/'delivery_manifest.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert sha(M)=='7bd0c5fcb055ff9b9c67be7632b46b77707651cd98066c8f908920bc4141a742'
m=read(M);assert m['passed'] and len(m['files'])==2
evidence={str(M):sha(M)}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p)
 assert str(p) not in evidence or evidence[str(p)]==h
 assert sha(p)==h,(str(p),'changed')
 evidence[str(p)]=h
for key in ('proof_sha256','runner_sha256'):
 for p,h in m[key].items():bind(p,h)
rp=Path(m['baseline_release']);bind(rp,m['baseline_release_sha256']);release=read(rp)
assert release['passed'] and len(release['source_sha256'])==915
assert all(sha(W/p)==h for p,h in release['source_sha256'].items())
for row in m['files']:
 p=row['path'];bind(C/p,row['candidate_sha256'])
 if row['base_sha256'] is None:assert not (W/p).exists()
 else:assert sha(W/p)==row['base_sha256']==release['source_sha256'][p]
assert len(m['test_scope'])==len(set(m['test_scope']))==177
assert set(m['test_scope'])==set(release['test_scope'])|{'src/alicia_grasp_modes/tests/test_station_contact_current_wait.py'}
p='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'
a,b=ast.parse((W/p).read_text()),ast.parse((C/p).read_text())
changed={'_station_contact_provenance','_freeze_execution_plan','_invoke_plan_bound_action'}
def strip(t):
 t.body=[n for n in t.body if not(isinstance(n,ast.ClassDef) and n.name=='_StationContactPending')]
 for n in t.body:
  if isinstance(n,ast.ClassDef) and n.name=='StationComputeWaitTaskMixin':
   n.body=[x for x in n.body if not(isinstance(x,ast.FunctionDef) and x.name in changed)]
 return ast.dump(t)
assert strip(a)==strip(b)
focusp=V/'gen17_station_contact_current_wait_r1/result.json';focus=read(focusp)
assert focus['passed'] and focus['source_unchanged'] and focus['actual_unchanged']
assert focus['focused']['summary']==dict(tests=165,errors=0,failures=0,skipped=0)
assert focus['test_scope']==m['focused_test_scope']
for rel,h in focus['candidate_sha256'].items():assert sha(C/rel)==h,(rel,'candidate changed')
for typ in ('log','xml'):bind(focusp.parent/'focused'/('pytest.'+typ),focus['focused'][typ+'_sha256'])
proofp=V/'gen17_station_contact_current_wait_boundary_r2/result.json';proof=read(proofp)
assert proof['passed'] and proof['source_unchanged'] and proof['actual_unchanged']
assert proof['baseline']['summary']==dict(tests=3,errors=0,failures=3,skipped=0)
assert proof['candidate']['summary']==dict(tests=3,errors=0,failures=0,skipped=0)
assert proof['baseline']['scope']==proof['candidate']['scope']
for name,root in [('baseline',W),('candidate',C)]:
 info=proof[name];bind(info['loaded_sources'],info['loaded_sha256']);loaded=read(info['loaded_sources'])
 assert loaded['station_task']['path']==str(root/p)
 assert loaded['station_task']['sha256']==sha(root/p)
 for row in loaded.values():assert sha(row['path'])==row['sha256']
 for typ in ('log','xml'):bind(proofp.parent/name/('pytest.'+typ),info[typ+'_sha256'])
 cases=list(ET.parse(proofp.parent/name/'pytest.xml').getroot().iter('testcase'))
 assert len(cases)==3 and {x.attrib['name'] for x in cases}==set(info['scope'])
 if name=='baseline':
  assert all(x.find('failure') is not None for x in cases)
  direct=next(x for x in cases if '[ros]' in x.attrib['name'])
  frozen=next(x for x in cases if '[fresh-freeze]' in x.attrib['name'])
  assert 'station_compute_wait_registration_too_late' in direct.find('failure').text
  assert 'station_compute_wait_registration_too_late' in frozen.find('failure').text
 else:assert all(x.find('failure') is None and x.find('error') is None for x in cases)
wirep=V/'attempt5_registration_timeline_r1/wire.json';extractionp=wirep.parent/'result.json';ex=read(extractionp)
bind(extractionp);bind(wirep,ex['wire_sha256']);rows=read(wirep);log=rows[1392];assert log['index']==1392 and 'registration_too_late' in log['message']['msg'];bind(log['file'],log['sha256'])
new=rows[1390];assert new['message']['state']=='ready';bind(new['file'],new['sha256'])
bind(Path(__file__))
result=dict(schema=1,kind='GEN17_TASK_REGISTERED_CURRENT_WAIT_INDEPENDENT_REVIEW',passed=True,delivery_manifest=str(M),delivery_sha256=sha(M),test_scope=m['test_scope'],test_scope_count=177,evidence_sha256=dict(sorted(evidence.items())),actual_source_count=915,actual_sources_unchanged=True,review_method='Independent exact diff, AST unchanged-scope check, source/evidence hashes, original wire and focused/baseline XML inspection; no test rerun',blocking_findings=[],changed_methods=sorted(changed),unchanged_other_module_AST=True,focused_test_summary=focus['focused']['summary'],baseline_expected_failure_summary=proof['baseline']['summary'],candidate_matched_case_summary=proof['candidate']['summary'],findings=[
'validate_registration remains unchanged for first/new source joins and historical receipt integrity. Only a previously registered current contract is revalidated with validate_contract state semantics.',
'Current geometry expiry is quarantined and returns nonterminal PENDING; identity ROS/monotonic expiry, owner/manual/epoch changes and current source witness changes remain rejecting. No deadline or evidence stamp is renewed.',
'The private typed pending signal is raised only before the inherited freeze mutates authority and only for the exact contact rebind candidate within entered metadata commit. The only catch is the contact metadata branch and re-enters the existing finite wait after releasing its locks.',
'Real three-actor tests cover copy and final-freeze window crossing, actual current registration callbacks, original rebind caller, exactly one base freeze/frozen notice and one inert physical-boundary sentinel after fresh input; manual/epoch/phase/identity loss yields no commit.',
'Whole module AST outside the three named methods and private exception remains unchanged; actual 915 released sources remain untouched.',
'R5 log identifies current source 1791890537728708505 and geometry deadline 1791890538528708505. Diagnostic log time is 8.388 ms later. Recorder had already received a newer READY, but that does not prove task callback completion.'
],limitations=[
'No physical trial, ROS calls, timing benchmark, collision recomputation or full 177-file actual regression was run by this reviewer.',
'Synthetic RGBD and translated recorded geometry exercise protocol and original admission methods; physical grasp success and actual CPU throughput remain unproven.',
'Baseline direct and final-freeze cases expose registration_too_late explicitly; baseline copy case fails waiting for the expected pending signal. These three failures are retained as failures.',
'The analogous remote receipt-generation crossing is outside this two-file delivery and requires its separately reviewed package.',
'Initial preflight diagnostic wrappers may record retry attempts; authority, original clocks, actual base freeze and motion-boundary counts are the tested invariants.',
'Original R5 failure and boundary_r1 runner setup error are historical, not converted into success.'
],integrated_regression_performed=False,installed=False,deployed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
out=W/'.runtime/gen17_station_contact_current_wait_review_draft.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2,sort_keys=True)+'\n');print(str(out),sha(out),'evidence',len(evidence))
