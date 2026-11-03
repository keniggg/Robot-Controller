import ast,hashlib,json,time
from pathlib import Path
import xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
C=W/'.runtime/gen16_session_ack_wait_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=C/'delivery_manifest.json'; expected='68f3e1da6348ce7beda277934bdbcb646b71ad823d4744c46f40a80f194f300c'
assert sha(manifest)==expected
m=json.loads(manifest.read_text());assert m['passed'] is True
sources={};evidence={str(manifest):expected}
for row in m['files']:
 assert sha(C/row['path'])==row['candidate_sha256']
 if row['base_sha256'] is None:assert not (W/row['path']).exists()
 else:assert sha(W/row['path'])==row['base_sha256']
 sources[row['path']]={'base_sha256':row['base_sha256'],'candidate_sha256':row['candidate_sha256']}
for key in ('prior_release','verification','runner','wire_probe'):
 p=Path(m[key]['path']);assert sha(p)==m[key]['sha256'];evidence[str(p)]=sha(p)
for name,key in [('changes.patch','patch_sha256'),('review_notes.md','review_notes_sha256')]:
 assert sha(C/name)==m[key];evidence[str(C/name)]=m[key]
rp=Path(m['verification']['path']);r=json.loads(rp.read_text())
assert r['passed'] and r['source_unchanged'] and r['actual_unchanged'] and r['installed_source_count']==908
assert all(sha(C/p)==h for p,h in r['source_sha256'].items())
release=json.loads(Path(m['prior_release']['path']).read_text());assert len(release['source_sha256'])==908
assert all(sha(W/p)==h for p,h in release['source_sha256'].items())
xmlcases={}
for key,count in [('baseline',3),('candidate',3),('focused',531)]:
 item=r[key];assert item['returncode']==0 and item['summary']==dict(tests=count,errors=0,failures=0,skipped=0)
 root=rp.parent/item['name'];assert json.loads((root/'result.json').read_text())==item
 evidence[str(root/'result.json')]=sha(root/'result.json')
 for filename,digestkey in [('pytest.log','log_sha256'),('pytest.xml','xml_sha256')]:
  assert sha(root/filename)==item[digestkey];evidence[str(root/filename)]=item[digestkey]
 suites=list(ET.parse(root/'pytest.xml').getroot().iter('testsuite'))
 assert {k:sum(int(s.get(k,0)) for s in suites) for k in ('tests','errors','failures','skipped')}==item['summary']
 ids=[x.get('classname')+'::'+x.get('name') for s in suites for x in s.findall('testcase')]
 assert len(ids)==count;xmlcases[key]=ids
assert sum(x.split('::')[0].split('.')[-1]=='test_session_ack_contract_wait' for x in xmlcases['focused'])==49
prior=N/'verification/gen15_phase_timestamp_integrated_install_r2/result.json'
prior_scope=json.loads(prior.read_text())['test_scope'];scope=set(prior_scope)|{'src/alicia_grasp_modes/tests/test_session_ack_contract_wait.py'}
assert len(scope)==166 and set(m['test_scope'])==scope and len(m['test_scope'])==166
evidence[str(prior)]=sha(prior)
wire_results={}
wire=None
for part in ('original_r4_boundary','candidate_r4_boundary'):
 for name in ('recorded_rejected_ack_boundary','counterfactual_accepted_same_latency','recorded_task_failure_boundary'):
  p=rp.parent/part/(name+'.json');d=json.loads(p.read_text());evidence[str(p)]=sha(p)
  assert d['original_ack_accepted'] is False and d['physical_motion'] is False
  assert d['original_ack_latency_ns']==5968901634
  wp=Path(d['wire_path']);assert sha(wp)==d['wire_sha256'];evidence[str(wp)]=sha(wp)
  rows={str(x['index']):x for x in json.loads(wp.read_text())}
  for index,h in d['original_raw_sha256'].items():
   row=rows[index];assert row['sha256']==h and sha(row['file'])==h;evidence[row['file']]=h
  wire_results[part+'/'+name]=dict(error=d['error'],elapsed_ns=d['elapsed_ns'],counterfactual=d['counterfactual'])
assert wire_results['candidate_r4_boundary/recorded_rejected_ack_boundary']['error']=='OBSERVE_RETURN_REMOTE_REJECTED: OBSERVE_RETURN_TARGET_CHANGED'
assert wire_results['candidate_r4_boundary/counterfactual_accepted_same_latency']['error'] is None
assert wire_results['candidate_r4_boundary/recorded_task_failure_boundary']['error']=='OBSERVE_RETURN_OWNER_CHANGED'
path='src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py'
a=ast.parse((W/path).read_text());b=ast.parse((C/path).read_text())
def cls(t):return next(x for x in t.body if isinstance(x,ast.ClassDef) and x.name=='ObserveReturnTask')
ma={x.name:x for x in cls(a).body if isinstance(x,ast.FunctionDef)}
mb={x.name:x for x in cls(b).body if isinstance(x,ast.FunctionDef)}
changed=[k for k in ma if ast.dump(ma[k],include_attributes=False)!=ast.dump(mb[k],include_attributes=False)]
assert changed==['_clear_observe_return_attempt','_observe_return_publish']
for i,x in enumerate(cls(b).body):
 if isinstance(x,ast.FunctionDef) and x.name in changed:cls(b).body[i]=ma[x.name]
assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
report=dict(schema=1,kind='gen16_session_ack_wait_independent_review',passed=True,recorded_ns=time.time_ns(),delivery_manifest=str(manifest),delivery_sha256=expected,
 test_scope=sorted(scope),focused_test_scope=[str(Path(p).relative_to(C)) for p in r['focused']['command'] if p.startswith(str(C)) and p.endswith('.py')],focused_test_summary=r['focused']['summary'],recorded_boundary_test_summaries={key:r[key]['summary'] for key in ('baseline','candidate')},
 candidate_python_sources_count=len(r['source_sha256']),candidate_sources_unchanged=True,actual_source_count=908,actual_sources_unchanged=True,base_and_candidate_sha256=sources,evidence_sha256=evidence,production_ast_changed_methods=changed,blocking_findings=[],
 review_method='Independent read-only source/caller/MRO/test/runner review, AST localization, exact SHA and existing XML checks; no tests rerun.',
 findings=[
 'Five seconds is now one warning for session metadata ACK wait, not automatic failure. Original immutable session/phase/contact ROS cutoffs are retained, with one session wall-clock cap anchored at first publication and reused across transitions.',
 'The four real caller orders agree with the new owner snapshot: initial observing installs local owner before publish; supplemental and returning retain old owner until accepted ACK; returned establishes measured/qualified phase before publishing then updates local session.',
 'Initial observing does not require an unrelated historical near-field phase or handoff. Later phase snapshot equality preserves the actual wire owner; no additional planning-stage, ns-cache or qualified-dict object-identity hard gate is introduced.',
 'Real station wrapper -> super publish -> existing _station_handoff_live and SDK callback coverage passes a 5.97-second ACK and rejects camera change, stale SDK feedback and measured joint movement. Existing station checks remain responsible for physical owner validity.',
 'Explicit rejection, owner/request/selection/epoch mutation, manual takeover, stop, revocation, phase mutation and clock rewind remain terminal. Wrong-key ACK cannot renew or satisfy the wait; same-key ACK retains every original binding check.',
 'The original R4 rejected ACK remains rejected at 5.968901634 seconds and original task FAILED remains terminal. Only a separately marked counterfactual accepted reply succeeds; no recorded failure is relabelled success.',
 'Session-clock state is cleared by the existing attempt clear path. No source freshness, bound-plan admission, motion command or action execution check was weakened.'
 ],recorded_r4_boundary=wire_results,
 limitations=[
 '531 focused tests plus three baseline and three candidate wire-boundary cases are offline; no physical grasp or full concurrent R4 replay is claimed.',
 'R4 qualified monotonic/handoff in-memory records are not recoverable from wire; raw-message proof is scoped to the metadata boundary, with separate real station-branch fixture coverage.',
 'Other waits remain explicit separate concerns: non-qualified measurement ACK min(5,remaining), exact-source capture deadline, and three-second exact-plan audit availability. This package does not claim all asynchronous waits are repaired.',
 'The 166-file test_scope is the required integrated regression union, not the count of files run in focused evidence.',
 'Review approval is for this frozen isolated candidate. Combined patches still require root integration regression and deployment checks.'
 ],installed=False,physical_grasp_verified=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
out=W/'.runtime/gen16_session_ack_wait_independent_review.pending.json'
out.write_text(json.dumps(report,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
print(json.dumps(dict(pending=str(out),sha256=sha(out),passed=True,focused_tests=531,boundary_tests=6,test_scope_count=len(scope),candidate_sources=len(r['source_sha256']))))
