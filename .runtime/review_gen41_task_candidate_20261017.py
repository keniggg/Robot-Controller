#!/usr/bin/env python3
"""Read-only candidate review; no ROS, imports from source, or device access."""
from pathlib import Path
import ast, copy, difflib, hashlib, json, time
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
OUT=D/'verification/gen41_task_candidate_independent_review_r1'
S=W/'.runtime/gen41_candidate_source'
TASK='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py'
SHARED='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_reacquisition.py'
SCHEMA='src/alicia_grasp_modes/src/alicia_grasp_modes/station_contact_admission.py'
TEST='src/alicia_grasp_modes/tests/test_gen41_task_availability_wait.py'
EXPECTED={TASK:'4ceb578b1ed20166ead19682e6875ced196d96255cf3c6f1c9a0905b7f23d904',SHARED:'078ca934c3db77adbc608ccf785e617e6118b2c2e749d8f139d370488cf5107c',SCHEMA:'eccc235deb14ab7a2c6e49b5a24435d1ab17af05c419c03e36a5dfa9afd6cdbc',TEST:'a73d4873b4713a2338b9ea063877208aa2b3714d8a106c6aef52a196d999da07'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,a):
 b=(json.dumps(a,ensure_ascii=False,indent=2,sort_keys=True)+'\n').encode();p.write_bytes(b)
 assert p.read_bytes()==b
 return sha(p)
def funcs(text):return {n.name:n for n in ast.walk(ast.parse(text)) if isinstance(n,ast.FunctionDef)}
def canon(n):return ast.dump(n,include_attributes=False)
assert D.is_dir() and (D.parents[1]/'maintenance').is_dir(), 'external record root missing'
assert not OUT.exists(), 'immutable output exists'
release=D/'verification/gen40_release_manifest.json'; baseline=json.loads(release.read_text())['source_sha256']
before={k:sha(W/k) for k in baseline}; assert before==baseline
paths={r:(W/r if r==TEST else S/r) for r in EXPECTED}
actual={r:sha(p) for r,p in paths.items()};assert actual==EXPECTED
texts={r:p.read_text() for r,p in paths.items()};[ast.parse(t) for t in texts.values()]
old=funcs((W/TASK).read_text());new=funcs(texts[TASK])
changed=[k for k in old if k in new and canon(old[k])!=canon(new[k])]
added=[k for k in new if k not in old]
# These original authority / freshness / support gates remain byte-structurally equal.
protected=['_station_contact_provenance','_station_task_context','_station_task_check','_station_task_quarantine','_station_task_support_gap_live','_station_task_support_gap_receipt','_station_task_retire']
for k in protected:assert canon(old[k])==canon(new[k]),k
# Original hard invalidation only gains cleanup of new unavailable-input ledgers.
invalidate=copy.deepcopy(new['_station_contact_invalidate'])
invalidate.body=[n for n in invalidate.body if not (isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and isinstance(n.value.func,ast.Attribute) and isinstance(n.value.func.value,ast.Attribute) and n.value.func.value.attr in {'_station_task_availability_pending','_station_task_availability_verified'})]
assert canon(invalidate)==canon(old['_station_contact_invalidate'])
# Existing shared contract / registration / recovery validators remain unchanged.
a,b=funcs((W/SHARED).read_text()),funcs(texts[SHARED]); unchanged_shared=[]
for k in a:
 if k in b and k not in {'validate_station_pending_notice','validate_station_feedback_recovery_notice','validate_station_pending_input'}:
  assert canon(a[k])==canon(b[k]),k
  unchanged_shared.append(k)
# Execute only the exact pure schema function, without loading production modules.
fn=funcs(texts[SCHEMA])['pending_payload_schema'];ns={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<schema-review>','exec'),ns)
f=ns['pending_payload_schema'];base={'station_compute_wait':{},'station_compute_recovery_chain':None}
cases=[(base,True),({**base,'clock_only':False},True),({**base,'clock_only':False,'station_pending_notice':{'state':'pending'}},True),({**base,'clock_only':False,'station_pending_input':{'kind':'fragment'}},True),({**base,'clock_only':False,'station_pending_notice':{'state':'pending'},'station_pending_input':{'kind':'fragment'}},False),({**base,'clock_only':True,'station_pending_input':{'kind':'fragment'}},False),({**base,'clock_only':False,'station_pending_notice':{}},False),({**base,'clock_only':0},False),({**base,'other':True},False)]
assert all(f(v) is expected for v,expected in cases)
after={k:sha(W/k) for k in baseline};assert before==after
assert {r:sha(p) for r,p in paths.items()}==EXPECTED
OUT.mkdir()
diff=''.join(difflib.unified_diff((W/TASK).read_text().splitlines(True),texts[TASK].splitlines(True),fromfile='GEN40/'+TASK,tofile='GEN41-candidate/'+TASK))
(OUT/'task.diff').write_text(diff)
review=dict(schema_version=1,kind='gen41_task_candidate_independent_static_review',recorded_ns=time.time_ns(),review_complete_for_bound_candidate_hashes=True,release_review_complete=False,full_regression_approved=False,applied=False,blocking_findings=[],baseline_release=str(release),baseline_release_sha256=sha(release),baseline_source_count=len(baseline),baseline_sources_unchanged=True,reviewed_source_sha256={str(paths[r]):v for r,v in actual.items()},new_task_test_declared_case_count=37,tests_for_final_candidate_executed=False,pure_schema_checks={'cases':len(cases),'passed':len(cases)},protected_task_gate_ast_unchanged=protected,hard_cutoff_method_original_ast_retained_except_new_ledger_cleanup=True,unchanged_original_shared_validator_ast=unchanged_shared,changed_task_methods=changed,new_task_methods=added,findings=[dict(id='complete-causal-proof-sufficient',severity='nonblocking',conclusion='Full locally delivered pending rejected-bracket proof or complete recovery proof explains availability without requiring obsolete false-object/empty-mask fragments. Fresh current READY still requires original three-topic, SDK, owner, target, support, current deadlines, and geometry gates.'),dict(id='truthful-provisional-marker',severity='nonblocking',conclusion='Exact partial marker and complete notice are mutually exclusive. Marker binds original parent, newer source and actual fragment digest/descriptor; it only quarantines and cannot register READY, explain a bracket, grant metric authority, or undo a preexisting hard cutoff.'),dict(id='pending-and-ready-proof-dispatch',severity='nonblocking',conclusion='Verified ledgers retain causal_notice and causal_parent. Both storage and late-status comparison select station_feedback_gap versus station_feedback_gap_recovery by causal notice state. Complete pending proof does not misuse READY-only fields.'),dict(id='hard-invalidation-preserved',severity='nonblocking',conclusion='Legacy clockfalse without explicit causal fields remains the original hard path. Contradiction, input mutation, target loss, owner/control change and expiration retain retirement. The original cutoff, provenance, support and context gates remain unchanged; new ledgers are cleared by hard invalidation.'),dict(id='bounded-private-history-only',severity='nonblocking',conclusion='Unexplained fragments block join under original identity/monotonic deadlines, with finite 16-entry ledgers and fail-closed capacity; repeated proof does not refresh original clocks.'),dict(id='new-tests-not-final-proof',severity='limitation',conclusion='Final 37 Task cases and shared Remote/E2E candidates are not executed in this review. Historical older overlay results are not claimed for these bound SHAs. Uniform application and strict focus/full regression are still required.')],limitations=['Static candidate-source review plus 9 pure schema checks; no runtime ROS or physical validation.','No simulated CAD/IK/FK or metadata component success is physical grasp evidence.','Shared validators establish causal/nonmetric provenance only; execution requires the unchanged current source/state/path checks.'],evidence_sha256={str(OUT/'task.diff'):sha(OUT/'task.diff'),str(Path(__file__).resolve()):sha(Path(__file__).resolve()),str(D/'verification/gen41_before_edit_r1/result.json'):sha(D/'verification/gen41_before_edit_r1/result.json'),str(D/'verification/gen41_remote_candidate_independent_review_r1/result.json'):sha(D/'verification/gen41_remote_candidate_independent_review_r1/result.json')},live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,production_modified=False,old_tests_modified=False)
r=dump(OUT/'result.json',review)
s=dump(OUT/'seal.json',dict(kind='gen41_task_candidate_static_review_seal',complete=True,release_review_complete=False,result_sha256=r,reviewed_source_sha256=review['reviewed_source_sha256'],evidence_sha256=review['evidence_sha256'],baseline_sources_unchanged=True,production_modified=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False))
print(json.dumps({'result':str(OUT/'result.json'),'result_sha256':r,'seal_sha256':s,'blocking':[]},ensure_ascii=False))
