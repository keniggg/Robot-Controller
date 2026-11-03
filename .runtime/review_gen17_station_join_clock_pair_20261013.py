from pathlib import Path
import ast,hashlib,json,time,xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification');C=W/'.runtime/gen17_station_join_clock_pair_20261013';M=C/'delivery_manifest.json';P=W/'.runtime/gen17_station_contact_current_wait_20261013'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
assert sha(M)=='f24dde4f1f2384a258b30a15163f0fae9bf31b50d8b3c3a2859d76702d898b38'
m=read(M);rp=V/'gen17_station_contact_current_wait_independent_review/result.json';prior=read(rp);assert sha(rp)=='0f230f94598cd5c9d4ad9bbb0d76756d80a7d9ba3a3fd1fd685a59acf3eab1aa';evidence={}
def bind(p,h=None):
 p=Path(p);p=p if p.is_absolute() else W/p;h=h or sha(p);assert sha(p)==h,(p,'changed');assert str(p) not in evidence or evidence[str(p)]==h;evidence[str(p)]=h
for p,h in prior['evidence_sha256'].items():bind(p,h)
for p,h in m['proof_sha256'].items():bind(p,h)
bind(M);bind(rp);bind(m['inherits']['manifest'],m['inherits']['sha256'])
r=read(m['baseline_release']);assert m['passed'] and r['passed'] and m['baseline_source_sha256']==r['source_sha256'];assert len(r['source_sha256'])==915
assert all(sha(W/p)==h for p,h in r['source_sha256'].items())
for row in m['files']:
 p=row['path'];bind(C/p,row['candidate_sha256']);assert (sha(W/p) if (W/p).exists() else None)==row['base_sha256']
assert len(m['test_scope'])==len(set(m['test_scope']))==178
assert set(m['test_scope'])==set(prior['test_scope'])|{'src/alicia_grasp_modes/tests/test_station_join_clock_pair.py'}
p='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py';text=(P/p).read_text()
a="                if getattr(self, 'latest_obj', None) is None:\n                    raise ValueError('station_task_original_object_gate_rejected')\n                now = rospy.Time.now().to_nsec()"
b=a.replace('                now =','                mono = time.monotonic()\n                now =');assert text.count(a)==1;text=text.replace(a,b)
a='                mono = time.monotonic()\n                if gap_sources:';assert text.count(a)==1;text=text.replace(a,'                if gap_sources:')
a='    def _copy_near_field_preview_candidate(self, current_plan, *args, **kwargs):\n        self._station_task_check()';assert text.count(a)==1;text=text.replace(a,a.replace('        self._station_task_check()','        with self._grasp6d_plan_guard():\n            self._station_task_check()'))
assert ast.dump(ast.parse(text))==ast.dump(ast.parse((C/p).read_text()))
proofp=V/'gen17_station_join_clock_pair_r2/result.json';proof=read(proofp);assert proof['passed'] and proof['actual_unchanged'] and proof['candidate_unchanged'] and proof['inherited_candidate_unchanged']
assert proof['baseline']['summary']==dict(tests=2,errors=0,failures=2,skipped=0)
assert proof['focused']['summary']==dict(tests=84,errors=0,failures=0,skipped=0)
for name,subdir,root in [('baseline','frozen_clock_baseline',P),('focused','candidate_focused',C)]:
 q=proof[name];bind(q['loaded_sources'],q['loaded_sha256']);loaded=read(q['loaded_sources']);assert loaded['station_task']['path']==str(root/p)
 for row in loaded.values():assert sha(row['path'])==row['sha256']
 for kind in ('log','xml'):bind(proofp.parent/subdir/('pytest.'+kind),q[kind+'_sha256'])
 cases=list(ET.parse(proofp.parent/subdir/'pytest.xml').getroot().iter('testcase'));assert len(cases)==q['summary']['tests']
 if name=='focused':assert all(x.find('failure') is None and x.find('error') is None for x in cases)
 else:assert all(x.find('failure') is not None for x in cases)
for rel,h in proof['candidate_sha256'].items():assert sha(C/rel)==h
probe=read(V/'gen17_join_clock_pair_probe_r1/result.json')
for path,h in probe['outputs'].items():bind(path,h)
assert read(V/'gen17_join_clock_pair_probe_r1/frozen.json')['action_sentinel_count']==1
for filename in ('test_gen17_join_clock_pair_probe_20261013.py','run_gen17_join_clock_pair_probe_20261013.py','test_gen17_current_record_interleave_20261013.py'):bind(W/'.runtime'/filename)
first=V/'gen17_station_join_clock_pair_r1/result.json'
if first.exists():bind(first)
bind(W/'.runtime/run_gen17_station_join_clock_pair_20261013.py',proof['runner_sha256']);bind(Path(__file__))
result=dict(schema=1,kind='GEN17_TASK_JOIN_CLOCK_PAIR_AND_COPY_GUARD_INDEPENDENT_REVIEW',passed=True,delivery_manifest=str(M),delivery_sha256=sha(M),test_scope=m['test_scope'],test_scope_count=178,evidence_sha256=dict(sorted(evidence.items())),inherits_review=str(rp),inherits_review_sha256=sha(rp),actual_source_count=915,actual_sources_unchanged=True,blocking_findings=[],review_method='Exact inherited AST reconstruction, static lock/clock review, source hashes and saved baseline/focused XML; no repeated tests',only_two_incremental_changes_AST=True,focused_test_summary=proof['focused']['summary'],baseline_expected_failure_summary=proof['baseline']['summary'],findings=[
'All sealed task current-wait changes remain intact. The only additional production edits move the existing monotonic read immediately before its paired final ROS read and place the initial preview-copy station check under the existing plan RLock.',
'Pairing clocks before subsequent validation prevents a paused ROS clock plus scheduler delay from moving the original monotonic expiry forward. Existing final monotonic checks reject or wait without a new gate or deadline change.',
'Real producer/remote/task probe demonstrates the previous condition reaching an inert action sentinel with paused ROS time; it is a conditional synthetic clock counterexample, not the observed cause of R5.',
'Five new cases include long delay with advancing/frozen ROS yielding PENDING and zero action; no/short delay still freezes and retires once; Event-interleaved actual READY callbacks expose old-record quarantine of a new record and prove serialization under the same lock.',
'Requalification work remains outside the short initial-check lock. Old record/contract clocks remain immutable; actual 915 sources and sealed task parent are unchanged.'
],limitations=[
'No live ROS, physical grasp, actual CPU performance or installed full regression is claimed.',
'The separate requalification-status witness entrance is not modified here; its existing explicit drift quarantine semantics and any additional atomicity repair require separate review.',
'Only task registration pairing is changed; producer/remote pairing similarities are outside this package.',
'The original focused r1 82/83 result remains a failure, followed by a deterministic interleaving counterexample and the fixed r2 84/84 result.'],integrated_regression_performed=False,installed=False,deployed=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
out=W/'.runtime/gen17_station_join_clock_pair_review_draft.json';out.write_text(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2)+'\n');print(str(out),sha(out),'evidence',len(evidence))
