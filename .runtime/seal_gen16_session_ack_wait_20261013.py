#!/usr/bin/env python3
import ast
import difflib
import hashlib
import json
from pathlib import Path
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen16_session_ack_wait_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen16_session_ack_wait_delivery'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
release=N/'verification/gen15_phase_timestamp_release_manifest.json'
proof=N/'verification/gen16_session_ack_wait_r1/result.json'
assert json.loads(proof.read_text())['passed']
sealed=json.loads(release.read_text())['source_sha256']
assert all(sha(W/r)==v for r,v in sealed.items())
changed=['src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
 'src/alicia_grasp_modes/tests/test_observe_return_task.py',
 'src/alicia_grasp_modes/tests/test_session_ack_contract_wait.py']
files=[dict(path=r,base_sha256=sha(W/r) if (W/r).exists() else None,candidate_sha256=sha(C/r)) for r in changed]
a,b=[ast.parse(p.read_text()) for p in (W/changed[0],C/changed[0])]
methods=['_clear_observe_return_attempt','_observe_return_publish']
for tree in (a,b):
 cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObserveReturnTask')
 cls.body=[n for n in cls.body if not isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) or n.name not in methods]
assert ast.dump(a)==ast.dump(b)
patch=''.join(''.join(difflib.unified_diff((W/r).read_text().splitlines(True) if (W/r).exists() else [],
 (C/r).read_text().splitlines(True),fromfile='a/'+r if (W/r).exists() else '/dev/null',tofile='b/'+r)) for r in changed)
(C/'changes.patch').write_text(patch)
prior=json.loads((W/'.runtime/gen15_gen11_fixture_completion_20261013/delivery_manifest.json').read_text())
focused=json.loads(proof.read_text())['focused']['command']
focused=[str(Path(x).relative_to(C)) for x in focused if x.startswith(str(C)) and x.endswith('.py')]
scope=sorted(set(prior['test_scope'])|set(focused))
notes='''GEN16 bounded session ACK handoff

Observed R4 cause and reproduced boundary
- Original supplemental session raw wire index 1001 to rejected ACK index 1086 takes 5.968901634 seconds. Original task ends after 5 seconds, before the later producer retirement and rejected ACK.
- Original runtime staging lost-update is separately repaired and sealed. This package prevents an otherwise current metadata handoff from failing solely at the heuristic 5-second threshold.
- Original R4 initial accepted ACK, request, active phase and rejected ACK are decoded directly from SHA-bound raw serialized messages. The original rejected ACK is still rejected by candidate. Replaying the recorded task FAILED outcome still terminates candidate. A separately labelled counterfactual normalized accepted reply at that latency succeeds; it is not recorded or physical success.
- In-memory R4 qualified monotonic/station handoff records cannot be reconstructed from wire. Wire proof is explicitly the metadata boundary, not a full hardware replay. Separate real task MRO station wrapper -> super publish -> existing _station_handoff_live -> actual ACK and accepted SDK callbacks passes 5.97s and rejects actual camera/feedback failures. Real qualified measurement -> phase setter -> ensure -> delayed ACK/reuse passes without renewing its phase.

Final behavior and four callers
- initial observing: no reached station required; unrelated stale prior phase/handoff ignored. First publication creates fixed session monotonic cap.
- supplemental observing: old live session remains owner until ACK; existing station handoff validator retains phase, epoch, camera and measured SDK checks. No metadata ACK grants motion or replaces current geometry.
- returning: observing remains owner while ACK waits; active published phase and its original deadline remain fixed, then original bound return action validation runs.
- returned: measured return and qualified phase are established before publish; same published phase, original contact deadline and session cap remain fixed.
- 5 seconds logs once. Same-key real accepted ACK still needs every original binding field; rejection, changed owner/manual/epoch/revocation/phase or rewind exits. Foreign keys remain unaccepted until original finite cap. No source freshness waiver; source and bound action checks after ACK are unchanged.
- `_execution_authority_revoked` is existing authoritative plan cancellation, not an added heuristic. No added fixed planning_stage, cache-nanosecond or qualified-dict identity requirement. Qualified cache only narrows the wall cap when it describes this published phase. Supplemental live owner checks reuse the existing validator.

Related waits audited, deliberately separate from this minimal package
1. _observation_measurement_publish, qualified=False: min(5, remaining) waits for BOTH producer executing preparation ACKs, or both terminal qualified-receipt ACKs. Failure raises OBSERVATION_MEASUREMENT_ACK_TIMEOUT and caller fails/revokes; this is not proof of collision. Current remote request callback uses mode/stream/geometry locks and independent SDK admission, not the session callback work-drain lock. A future warning-plus-bounded-wait could be appropriate for executing preparation while same request/owner/phase and fixed session wall cap remain current, followed by original pre-submit plan validation. Terminal qualified receipt additionally requires still-current captured sources; it must not be prolonged past capture/source validity. It is unchanged here because it has two distinct protocol states and no R4 failure evidence in this wait, and neither a metadata session ACK nor extending 5 seconds can certify those independent source receipts. This is an identified remaining heuristic, not claimed universally necessary or solved.
2. _observation_measurement_publish, qualified=True: existing capture_deadline_ns bounds acquisition of three actual post-arrival frames and both exact-source ACKs. This is source/measurement validity, not ordinary transmission timeout, and is unchanged.
3. _observe_return_report: 3 seconds waits for a SHA-verified on-disk gate audit by exact plan ID. Used before session creation, supplemental proof reuse, and final observed/returned contact admission. Failure raises OBSERVE_RETURN_BOUND_AUDIT_UNAVAILABLE and may fail the attempt. Initial call has no session contract from which to inherit an upper bound. For later contact calls a recoverable fresh-plan/audit retry within original phase can improve success, but cannot use a stale report or source merely because an audit eventually arrived. No R4 missing-audit failure was recorded. This package leaves that separate orchestration change explicit rather than assuming an ACK validator can validate audit content/source.

Validation
- 49 new task tests plus existing related suites: 531 passed.
- SHA-bound R4 baseline and candidate metadata probes: 3 each passed (baseline expected 5s timeout; candidate exact outcome asserted).
- Actual 908 sealed source hashes unchanged. No ROS connection, robot command or temperature query.
- Existing test_observe_return_task.py changes only one fake timeout clock to reach the original finite contract instead of assuming 5 seconds ends the task; original stale/wrong-key assertions remain.
'''
(C/'review_notes.md').write_text(notes)
manifest=dict(schema=1,kind='GEN16_BOUNDED_SESSION_ACK_WAIT_DELIVERY',passed=True,
 status='ready_for_independent_review_and_actual_combination',candidate_root=str(C),
 current_baseline_sources=len(sealed),prior_release=dict(path=str(release),sha256=sha(release)),
 files=files,test_scope=scope,test_scope_count=len(scope),focused_tests=531,recorded_boundary_cases=6,
 production_ast_unchanged_except=methods,source_authority_unchanged=True,deadline_renewal=False,
 five_second_behavior='one warning, wait only within existing ROS deadlines and fixed monotonic caps',
 patch_sha256=sha(C/'changes.patch'),review_notes_sha256=sha(C/'review_notes.md'),
 verification=dict(path=str(proof),sha256=sha(proof)),
 runner=dict(path=str(W/'.runtime/run_gen16_session_ack_wait_20261013.py'),sha256=sha(W/'.runtime/run_gen16_session_ack_wait_20261013.py')),
 wire_probe=dict(path=str(W/'.runtime/test_gen16_session_ack_r4_wire_20261013.py'),sha256=sha(W/'.runtime/test_gen16_session_ack_r4_wire_20261013.py')),
 actual_unchanged=True,physical_grasp_verified=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
(C/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
for name in ('delivery_manifest.json','changes.patch','review_notes.md'):(O/name).write_bytes((C/name).read_bytes())
print('delivery',str(C/'delivery_manifest.json'),sha(C/'delivery_manifest.json'))
print('scope',len(scope));print(json.dumps(files,indent=2))
