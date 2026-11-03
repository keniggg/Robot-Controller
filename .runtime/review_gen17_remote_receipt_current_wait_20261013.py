from pathlib import Path
import ast,hashlib,json,time,xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification');C=W/'.runtime/gen17_remote_receipt_current_wait_20261013';M=C/'delivery_manifest.json'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert sha(M)=='b813c030e000212eb87c1489acc29193171a9caea172e68b3565dbc4b0543031'
m=read(M);evidence={}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p)
 assert sha(p)==h and (str(p) not in evidence or evidence[str(p)]==h);evidence[str(p)]=h
bind(M)
for p,h in m['proof_sha256'].items():bind(p,h)
bind(m['baseline_release'],m['baseline_release_sha256']);r=read(m['baseline_release'])
assert m['passed'] and r['passed'] and len(m['files'])==2
assert m['baseline_source_sha256']==r['source_sha256'] and len(r['source_sha256'])==915
assert all(sha(W/p)==h for p,h in r['source_sha256'].items())
for row in m['files']:
 p=row['path'];bind(C/p,row['candidate_sha256'])
 if row['base_sha256'] is None:assert not (W/p).exists()
 else:assert sha(W/p)==row['base_sha256']
assert len(m['test_scope'])==len(set(m['test_scope']))==177
assert set(m['test_scope'])==set(r['test_scope'])|{'src/alicia_grasp_modes/tests/test_station_contact_remote_current_wait.py'}
p='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
changed={'_station_contact_computation_receipt','_station_compute_wait_for_current'}
def strip(t):
 for c in t.body:
  if isinstance(c,ast.ClassDef):c.body=[n for n in c.body if not(isinstance(n,ast.FunctionDef) and n.name in changed)]
 return ast.dump(t)
assert strip(ast.parse((W/p).read_text()))==strip(ast.parse((C/p).read_text()))
proofp=V/'gen17_remote_receipt_current_wait_r3/result.json';proof=read(proofp)
assert proof['passed'] and proof['source_unchanged'] and proof['actual_unchanged']
assert proof['baseline']['summary']==dict(tests=1,failures=1,errors=0,skipped=0)
assert proof['candidate']['summary']==dict(tests=141,failures=0,errors=0,skipped=0)
for name,root in [('baseline',W),('candidate',C)]:
 row=proof[name];assert row['loaded']['remote']==str(root/p)
 for kind in ('log','xml'):bind(proofp.parent/name/('pytest.'+kind),row[kind+'_sha256'])
 cases=list(ET.parse(proofp.parent/name/'pytest.xml').getroot().iter('testcase'))
 assert len(cases)==row['summary']['tests']
 if name=='baseline':assert 'station_compute_wait_registration_too_late' in cases[0].find('failure').text
 else:assert all(x.find('failure') is None and x.find('error') is None for x in cases)
for rel,h in proof['candidate_sha256'].items():assert sha(C/rel)==h
bind(W/'.runtime/run_gen17_remote_receipt_current_wait_20261013.py',m['runner_sha256'])
bind(W/'.runtime/seal_gen17_remote_receipt_current_wait_20261013.py',m['sealer_sha256'])
bind(Path(__file__))
result=dict(schema=1,kind='GEN17_REMOTE_RECEIPT_CURRENT_WAIT_INDEPENDENT_REVIEW',passed=True,delivery_manifest=str(M),delivery_sha256=sha(M),test_scope=m['test_scope'],test_scope_count=177,evidence_sha256=dict(sorted(evidence.items())),actual_source_count=915,actual_sources_unchanged=True,blocking_findings=[],review_method='Independent exact diff/AST, immutable source/evidence hashes, baseline and focused XML review; no repeated tests',changed_methods=sorted(changed),unchanged_other_production_AST=True,focused_test_summary=proof['candidate']['summary'],baseline_expected_failure_summary=proof['baseline']['summary'],findings=[
'Only already registered current geometry crossing its ROS or monotonic window enters existing clock-only quarantine and bounded wait; final strict validate_registration and build_receipt remain and use the same valid issuance time.',
'Clock reversal, identity monotonic deadline and support-gap rechecks repeat existing tick semantics; source/identity/phase/session deadlines are not changed.',
'The continue leaves both stream and geometry locks before the existing Condition wait. Actual successor callbacks rejoin object/mask/status and the receipt names the fresh source, retaining original ticket/plan/three historical witnesses.',
'Including existing soft_wait in the wait signature ensures historical source contradiction that changes soft waiting to hard cancellation re-runs the existing ticket check instead of remaining asleep.',
'Eleven new cases plus 130 related cases pass. Independent mono offset advances time without mutating stored deadlines. Manual/epoch/phase/identity/history conflicts reject; original baseline geometry case raises registration_too_late.',
'Outside two methods the production module AST is unchanged; no actual source was edited.'
],limitations=[
'R5 remote issuance had succeeded. This is an independently reproduced analogous code boundary, not a claim it caused the observed R5 failure.',
'Synthetic images/plan geometry and offline clocks exercise admission/control semantics; no real-time performance, physical geometry or grasp success claim.',
'Full installed integration with the task fix and join-clock pairing repair is still required.',
'Potential preserve-identity stream-generation cancellation without a signature change is being separately checked through a real production method; this two-file package does not claim to resolve that distinct issue.',
'Earlier r1 fixture setup errors and r2 historical-conflict negative failure remain immutable evidence, not passing results.'
],integrated_regression_performed=False,installed=False,deployed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
out=W/'.runtime/gen17_remote_receipt_current_wait_review_draft.json';out.write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(str(out),sha(out),'evidence',len(evidence))
