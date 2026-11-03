#!/usr/bin/env python3
"""Independent immutable-source/receipt and static contract review, no ROS."""
import ast
import hashlib
import json
from pathlib import Path
import time
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen16_contact_budget_checkpoint_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen16_contact_budget_checkpoint_independent_review'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=C/'delivery_manifest.json'
assert sha(manifest)=='584f0464b01f375a77f700f26c7b40056a3f1d1ced5608e565d6795427b9bf8c'
m=json.loads(manifest.read_text()); verified=[]
for f in m['files']:
 r=f['relative_path'];assert sha(C/r)==f['sha256']
 assert (not (W/r).exists()) if f['new_file'] else sha(W/r)==f['baseline_sha256']
 verified.append(f)
assert sha(m['verification']['path'])==m['verification']['sha256']=='673e6b480fbbf7d74402c70e8b6f49369b37087727e555e9bba30b97a8ec06c4'
v=json.loads(Path(m['verification']['path']).read_text())
assert v['passed'] and v['summary']==dict(tests=220,failures=0,errors=0,skipped=0)
assert v['candidate_before']==v['candidate_after']
assert all(sha(p)==h for p,h in v['candidate_before'].items())
assert all(C in Path(p).parents for p in v['loaded'].values())
assert sha(m['patch']['path'])==m['patch']['sha256']
for p,h in m['legacy_recorded_fixture_sha256'].items():assert sha(p)==h
release=N/'verification/gen15_phase_timestamp_release_manifest.json'
sealed=json.loads(release.read_text())['source_sha256']
assert len(sealed)==908 and all(sha(W/r)==h for r,h in sealed.items())
rel='src/alicia_grasp_modes/src/alicia_grasp_modes/contact_search_yield.py'
def fn(p,name):return next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.FunctionDef) and n.name==name)
a=fn(W/rel,'contact_search_yield_proof_valid');b=fn(C/rel,'_legacy_contact_search_yield_proof_valid')
assert ast.dump(ast.Module(body=a.body,type_ignores=[]))==ast.dump(ast.Module(body=b.body,type_ignores=[]))
remote=ast.parse((C/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py').read_text())
original=ast.parse((W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py').read_text())
allowed={'_contact_yield_begin','_direct_near_field_comparison_search','_contact_height_window_retry','_contact_search_yield_selection','_contact_yield_proof'}
for tree in (remote,original):
 cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ObserveReturnRemoteMixin')
 cls.body=[n for n in cls.body if not isinstance(n,ast.FunctionDef) or n.name not in allowed]
assert ast.dump(remote)==ast.dump(original)
consumers=['src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
 'src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py']
for r in consumers:
 s=(W/r).read_text()
 assert 'contact_search_yield_proof_valid(' in s
 assert 'maximum_original_families' not in s and 'minimum_completed_families' not in s
notes=[
 'Independent source review, author evidence validation and hashes only; no repeat pytest and no ROS/actions.',
 'All 9 final delivery files, 10 author test-time Python sources/runner, 3 historical fixtures, patch and all 908 actual release hashes independently verified unchanged.',
 'V1 validator body AST identical to actual baseline; explicit integer version dispatcher rejects unknown/bool/string versions. V2 does not inherit exactly-two-family checks.',
 'Producer retains all unique geometric families and both attempted/completed counts. Same-geometry alternate representative track/variant association is by family_sha256; arrays still require unique hashes and pairs, completed<=attempted<=checked, checked<shortlist, and exact unchecked count.',
 'Each counted completed family requires a final still-bound current-selection failed precheck from runtime prepared identity. At least one same-request, non-authorizing true grasp continuous target-OBB collision receipt with exact SDK/accepted source timestamps remains required. Dispatch, timeout and local IK failure are not collision evidence.',
 'Ordinary height retry now forwards to existing strict base implementation including preferred recipe and skipped indices. Parent precheck receipt is cleared before each actual precheck; copied trial runtime cannot create extra original-family counts. Accepted height result becomes incumbent; failed or cleared/misbound final receipt does not claim exhaustive height failure.',
 'Continuing no longer stops at two families. A wall-only checkpoint cannot fabricate a ROS-time yield. V2 requires exact min(start+30s, phase_deadline-2s), yield>=checkpoint and yield<original phase deadline. Invalid/future/reversed receipt or transition ordering cannot form a valid accepted v2 proof.',
 'No valid yield certificate means existing single original-phase fallback, with no child-budget renewal. Finite fully searched shortlist is not relabelled incomplete; a valid incumbent preserves original execution selection and revalidation.',
 'All consumers inspected: remote builder calls public version validator; task supplement and both runner contract branches call the same validator. Independent measured gap, registered multi-view gain, request digest, target/session binding and full subsequent observation audit/planning/submission checks remain. No extra hard-coded two-family consumer found.',
 'Updated old tests reflect intentionally changed scheduling, preserve wrong receipt and zero-proof rejection, and retain immutable recorded v1 fixtures. Three renamed legacy scheduling cases are replaced with checkpoint/finite-shortlist/one-real-family semantics, not silently removed coverage.',
 'Author 220 passing cases include at least one family, >two families, family representative switch, early/forged checkpoint, unknown versions, count/family mismatch, no collision/current feedback, unchanged original phase fallback, real selector and audit->runner->task consumption. Transport/geometry fixtures remain synthetic; no full original-trajectory or physical success is established.'
]
limitations=[
 'Checkpoint is min(start+30s, original phase deadline-2s), not a universal minimum of 30s. With <32s remaining, the existing phase reserve can make it earlier. Parent explicitly confirmed this definition.',
 'The existing 2s publication reserve does not guarantee complete supplemental-view planning/ACK/movement fits. Dynamic reserve design is a separately identified optimization; no deadline extended and no success guarantee.',
 'Root reports R4 had about69s left when contact search began, leaving about39s after a30s checkpoint; this contextual estimate is not treated as a tested global upper bound.',
 'Wire proof is non-authorizing metadata. Negative/collision receipts and partial family checks do not establish all height alternatives failed or that the next view will succeed.',
 'Original single-phase fallback may continue beyond child checkpoint when evidence is insufficient, under the original phase contract. This package does not add a new global bound to the legacy fallback.'
]
result=dict(kind='GEN16_CONTACT_BUDGET_CHECKPOINT_INDEPENDENT_REVIEW',passed=True,
 reviewer='gen14_postqualified_diagnosis',reviewed_ns=time.time_ns(),manifest_path=str(manifest),manifest_sha256=sha(manifest),
 files=verified,verification=m['verification'],patch=m['patch'],author_summary=v['summary'],
 author_test_sources_verified=len(v['candidate_before']),actual_source_count=len(sealed),actual_unchanged=True,
 original_release=str(release),original_release_sha256=sha(release),legacy_v1_validator_body_AST_identical=True,
 shared_consumer_source_sha256={r:sha(W/r) for r in consumers},findings=[],notes=notes,limitations=limitations,
 additional_tests_run=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False,
 physical_grasp_verified=False,review_script_sha256=sha(__file__))
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
(O/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
print(str(O/'result.json'));print(sha(O/'result.json'))
