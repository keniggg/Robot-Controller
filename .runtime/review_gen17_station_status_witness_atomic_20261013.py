from pathlib import Path
import ast, hashlib, json, time, xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification')
C=W/'.runtime/gen17_station_status_witness_atomic_20261013'
P=W/'.runtime/gen17_station_join_clock_pair_20261013'
M=C/'delivery_manifest.json'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
assert sha(M)=='4d40befd65bed7f923358797552b011806d70137a44a06c3f31020c6160362a6'
m=read(M)
rp=V/'gen17_station_join_clock_pair_independent_review/result.json'
assert sha(rp)=='67760ade4618b3b37a97dc481f106d4b9b3039a10f478dcb0605b7fb8f40cfe4'
prior=read(rp);evidence={}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p)
 assert sha(p)==h,(p,'changed')
 assert str(p) not in evidence or evidence[str(p)]==h
 evidence[str(p)]=h
for p,h in prior['evidence_sha256'].items(): bind(p,h)
for p,h in m['proof_sha256'].items(): bind(p,h)
bind(M);bind(rp);bind(m['inherits']['manifest'],m['inherits']['sha256'])
bind(m['baseline_release'],m['baseline_release_sha256'])
r=read(m['baseline_release'])
assert m['passed'] and r['passed'] and m['baseline_source_sha256']==r['source_sha256']
assert len(r['source_sha256'])==915
assert all(sha(W/p)==h for p,h in r['source_sha256'].items())
for row in m['files']:
 p=row['path'];bind(C/p,row['candidate_sha256'])
 assert (sha(W/p) if (W/p).exists() else None)==row['base_sha256']
assert len(m['test_scope'])==len(set(m['test_scope']))==179
assert set(m['test_scope'])==set(prior['test_scope'])|{'src/alicia_grasp_modes/tests/test_station_status_witness_atomic.py'}
p='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'
old=ast.parse((P/p).read_text());new=ast.parse((C/p).read_text())
method=next(n for n in ast.walk(new) if isinstance(n,ast.FunctionDef) and n.name=='_station_compute_task_status')
branch=next(n for n in method.body if isinstance(n,ast.If) and 'witness' in ast.dump(n.test))
assert len(branch.body)==1 and isinstance(branch.body[0],ast.With)
block=branch.body[0]
assert ast.dump(block.items[0].context_expr)==ast.dump(ast.parse('self._grasp6d_plan_guard()',mode='eval').body)
assert len(block.items)==1 and len(block.body)==1 and isinstance(block.body[0],ast.Try)
branch.body=block.body
assert ast.dump(new)==ast.dump(old),'production differs beyond one existing RLock block'
proofp=V/'gen17_station_status_witness_atomic_r1/result.json';proof=read(proofp)
assert proof['passed'] and proof['actual_unchanged'] and proof['candidate_unchanged'] and proof['inherited_candidate_unchanged']
assert proof['baseline']['summary']==dict(tests=2,errors=0,failures=2,skipped=0)
assert proof['focused']['summary']==dict(tests=131,errors=0,failures=0,skipped=0)
for name,subdir,root in [('baseline','unlocked_notice_baseline',P),('focused','candidate_focused',C)]:
 q=proof[name];bind(q['loaded_sources'],q['loaded_sha256']);loaded=read(q['loaded_sources'])
 assert loaded['station_task']['path']==str(root/p)
 for row in loaded.values():assert sha(row['path'])==row['sha256']
 for kind in ('log','xml'):bind(proofp.parent/subdir/('pytest.'+kind),q[kind+'_sha256'])
 cases=list(ET.parse(proofp.parent/subdir/'pytest.xml').getroot().iter('testcase'))
 assert len(cases)==q['summary']['tests']
 if name=='focused':assert all(x.find('failure') is None and x.find('error') is None for x in cases)
 else:
  assert all(x.find('failure') is not None and 'test_notice_and_fresh_join_are_one_record_transaction' in x.attrib['name'] for x in cases)
for rel,h in proof['candidate_sha256'].items():assert sha(C/rel)==h
bind(W/'.runtime/run_gen17_station_status_witness_atomic_20261013.py',proof['runner_sha256'])
bind(Path(__file__))
result=dict(schema=1,kind='GEN17_STATION_STATUS_WITNESS_ATOMIC_INDEPENDENT_REVIEW',passed=True,
 delivery_manifest=str(M),delivery_sha256=sha(M),test_scope=m['test_scope'],test_scope_count=179,
 evidence_sha256=dict(sorted(evidence.items())),inherits_review=str(rp),inherits_review_sha256=sha(rp),
 actual_source_count=915,actual_sources_unchanged=True,blocking_findings=[],
 review_method='Read-only source and lock-order inspection, exact inherited AST comparison and saved result/XML/hash checks; no repeated tests',
 only_existing_witness_branch_guard_AST=True,focused_test_summary=proof['focused']['summary'],baseline_expected_failure_summary=proof['baseline']['summary'],
 findings=[
 'The only increment over the sealed clock/copy candidate wraps known-parent lookup, witness validation, current-record check and active quarantine in the existing plan RLock. Removing that with-node leaves the entire parent module AST unchanged.',
 'Status ingress does not hold a conflicting outer feedback/unknown-support lock. The new block uses the same plan-to-feedback/unknown-support order as ordinary status registration. The existing RLock supports nested status handling and contains no long requalification wait.',
 'Two Event counterexamples pause after real witness validation or real current-record checking, feed actual successor callbacks, and verify that the notice quarantines its transaction record before the successor replaces it. Original unlocked code deterministically fails these two cases.',
 'Valid producer drift notices still actively quarantine, preserve contract and monotonic deadlines, clear public geometry and cannot enroll local measured recovery. Invalid parent/witness remains ignored; actual manual/epoch ownership changes still retire.',
 'All inherited task current-wait, pre-freeze typed retry, clock-pair and preview-copy lock changes remain byte/AST verified; actual 915 sources remain unchanged.'
 ],limitations=[
 'Offline synthetic geometry, clocks and scheduled callbacks establish boundary/control behavior, not physical grasp success or runtime throughput.',
 'The active drift notice intentionally quarantines geometry; serialization does not weaken this policy or turn a producer claim into local measured recovery.',
 'This receipt covers the isolated task package. Parent must merge reviewed task/remote packages and run the full installed scope before deployment.',
 'Earlier setup and focused timing failures remain preserved in inherited evidence; no success is inferred from their later diagnosis.'
 ],integrated_regression_performed=False,installed=False,deployed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
out=W/'.runtime/gen17_station_status_witness_atomic_review_draft.json'
out.write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
print(str(out),sha(out),'evidence',len(evidence))
