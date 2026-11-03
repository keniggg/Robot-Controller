#!/usr/bin/env python3
import ast,difflib,hashlib,json,time
from pathlib import Path
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen17_remote_receipt_current_wait_r2_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen17_remote_receipt_current_wait_delivery_r2'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
release=N/'verification/gen16_success_first_release_manifest.json'
baseline=json.loads(release.read_text())
assert all(sha(W/r)==h for r,h in baseline['source_sha256'].items())
proof=N/'verification/gen17_remote_receipt_current_wait_r5/result.json'
verified=json.loads(proof.read_text())
assert verified['passed'] and verified['candidate']['summary']==dict(tests=142,failures=0,errors=0,skipped=0)
assert all(sha(C/r)==h for r,h in verified['candidate_sha256'].items())
rel='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
test='src/alicia_grasp_modes/tests/test_station_contact_remote_current_wait.py'
def outside(text):
 tree=ast.parse(text)
 for node in tree.body:
  if isinstance(node,ast.ClassDef) and node.name=='StationComputeWaitRemoteMixin':
   node.body=[n for n in node.body if not(isinstance(n,ast.FunctionDef) and n.name in ('_station_contact_computation_receipt','_station_compute_wait_for_current'))]
 return ast.dump(tree)
assert outside((W/rel).read_text())==outside((C/rel).read_text())
assert [r for r,h in baseline['source_sha256'].items() if sha(C/r)!=h]==[rel]
files=[dict(path=rel,base_sha256=sha(W/rel),candidate_sha256=sha(C/rel)),
       dict(path=test,base_sha256=None,candidate_sha256=sha(C/test))]
O.mkdir()
patch=''.join(difflib.unified_diff((W/rel).read_text().splitlines(True),(C/rel).read_text().splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
patch+=''.join(difflib.unified_diff([],(C/test).read_text().splitlines(True),fromfile='/dev/null',tofile='b/'+test))
(O/'candidate.patch').write_text(patch)
prior=[N/'verification/gen17_remote_receipt_current_wait_r1/result.json',N/'verification/gen17_remote_receipt_current_wait_r2/result.json']
evidence=[proof,O/'candidate.patch',N/'verification/gen17_station_admission_gate_audit/result.json',N/'verification/gen17_remote_receipt_current_wait_r3/result.json',N/'verification/gen17_remote_receipt_current_wait_r4/result.json',W/'.runtime/gen17_remote_receipt_current_wait_20261013/delivery_manifest.json']+prior
manifest=dict(schema=1,kind='gen17_remote_receipt_current_wait',passed=True,candidate_root=str(C),files=files,
 test_scope=baseline['test_scope']+[test],focused_test_scope=verified['candidate']['command'][4:-4],
 proof_sha256={str(p):sha(p) for p in evidence},baseline_release=str(release),baseline_release_sha256=sha(release),
 baseline_source_sha256=baseline['source_sha256'],baseline_expected_failures=1,candidate_matched_cases=2,previous_candidate_expected_failures=1,focused_passed=142,
 changes=['Only an already registered current source crossing its geometry ROS/monotonic window during context validation becomes typed clock-only quarantine and retries the original wait outside both locks.',
          'Same original ticket/private work/three source witnesses/plan identity and absolute phase/session/identity limits. Receipt issuance keeps strict validation at its recorded issued_ns.',
          'Repeat existing identity monotonic, clock reversal and support-gap checks at receipt boundary; true faults remain errors.',
          'Add existing soft_wait state to wait signature so a real historical contradiction that hard-quarantines existing pending work immediately triggers existing ticket cancellation.',
          'Also observe existing stream_generation in wait signature; actual MODEL_RELOADED preserve_identity=True cancels old private work even when scope/session/reference stay unchanged. No new cancellation policy.',
          'New 12 cases run real context/wait/ticket/quarantine and three-topic registration with ROS/exact boundary/independent monotonic advancement; original reference deadlines remain unchanged.'],
 prior_attempts_preserved=[dict(path=str(N/'verification/gen17_remote_receipt_current_wait_r4/result.json'),reason='Prior sealed candidate reproduced missed generation cancellation. New candidate canceled correctly; only new test expected lower-priority GENERATION_STALE instead of existing ModeAware processing-pending cancellation code. Fixed test precedence, production unchanged.'),dict(path=str(prior[0]),reason='Missing import of fixture context; 11 new setup errors, original 130 passed; no production behavior evaluated by missing fixtures.'),
                          dict(path=str(prior[1]),reason='140 passed; one actual wait-signature omission exposed by historical-conflict negative control; fixed by adding soft_wait signature field, negative retained.')],
 unchanged_other_production_AST=True,
 limitations=['Synthetic camera pixels and plan geometry; no claim of physical grasp quality or live hardware execution.',
              'Existing fixture configuration/time and publishers are offline; real context/wait/three-topic callbacks run, not success stubs.',
              'Task metadata commit race is a separate reviewed package; join-clock pairing is separately diagnosed and not changed here.',
              'R5 remote receipt itself succeeded; this package fixes a source-proven analogous boundary reproduced independently.'],
 runner_sha256=sha(W/'.runtime/run_gen17_remote_receipt_current_wait_r2_20261013.py'),sealer_sha256=sha(__file__),
 actual_unchanged=True,actual_source_count=len(baseline['source_sha256']),source_changes_installed=False,
 live_ros_contacted=False,motion_commands_sent=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
assert len(manifest['test_scope'])==177
# Record exact file scope, not pytest switches.
manifest['focused_test_scope']=[test,'src/alicia_grasp_modes/tests/test_gen11_contact_remote_receipt.py',
 'src/alicia_grasp_modes/tests/test_station_compute_wait_remote.py','src/alicia_grasp_modes/tests/test_station_compute_work_handoff.py']
(C/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
receipt=dict(manifest=str(C/'delivery_manifest.json'),manifest_sha256=sha(C/'delivery_manifest.json'),files=files,
 verification=str(proof),verification_sha256=sha(proof),patch_sha256=sha(O/'candidate.patch'),actual_unchanged=True)
(O/'result.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(dict(manifest=receipt['manifest'],sha256=receipt['manifest_sha256'],delivery_result=str(O/'result.json'),delivery_sha256=sha(O/'result.json'))))
