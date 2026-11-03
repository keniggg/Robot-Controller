from pathlib import Path
import ast,difflib,hashlib,json
W=Path(__file__).resolve().parent.parent;C=W/'.runtime/gen16_contact_budget_checkpoint_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen16_contact_budget_checkpoint_delivery';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
baseline=json.loads((C/'baseline_sha256.json').read_text());assert all(sha(W/r)==h for r,h in baseline.items())
R=N/'verification/gen16_contact_budget_checkpoint_r5/result.json';receipt=json.loads(R.read_text());assert receipt['passed'] and receipt['summary']['tests']==220
assert all(sha(p)==h for p,h in receipt['candidate_after'].items())
files=[];patch=''
for p in sorted(C.rglob('*.py')):
 rel=str(p.relative_to(C));old=W/rel
 files.append(dict(relative_path=rel,sha256=sha(p),baseline_sha256=baseline.get(rel),new_file=not old.exists()))
 patch+=''.join(difflib.unified_diff(old.read_text().splitlines(True) if old.exists() else [],p.read_text().splitlines(True),fromfile='a/'+rel if old.exists() else '/dev/null',tofile='b/'+rel))
(O/'candidate.patch').write_text(patch)
fixtures={}
for name in ('gen7_contact_yield_completed_budget.json','gen17_contact_yield_wire_receipts.json','gen16_recorded_contact_search_yield.json'):
 p=W/'src/alicia_grasp_modes/tests/fixtures'/name
 fixtures[str(p)]=sha(p)
 mirror=C/'src/alicia_grasp_modes/tests/fixtures'/name
 if mirror.exists():assert sha(mirror)==sha(p)
manifest=dict(package='gen16_contact_budget_checkpoint_v2',status='candidate_verified_not_installed',files=files,
 verification=dict(path=str(R),sha256=sha(R),**receipt['summary']),patch=dict(path=str(O/'candidate.patch'),sha256=sha(O/'candidate.patch')),
 legacy_v1_validator_body_AST_identical=True,legacy_recorded_fixture_sha256=fixtures,
 production_methods=['ObserveReturnRemoteMixin._contact_yield_begin','ObserveReturnRemoteMixin._direct_near_field_comparison_search (continuing closure only)','ObserveReturnRemoteMixin._contact_height_window_retry','ObserveReturnRemoteMixin._contact_yield_proof','contact_search_yield_proof_valid version dispatcher plus preserved legacy/new v2 validators'],
 success_first_policy=['Two completed geometry families are no longer an early stop or minimum.','Normal original height retry executes within its existing candidate deadline.','At least one current bound failed family and a same-request true grasp collision receipt are necessary for v2 yield, together with existing measured view gap, registered gain, live owner and strictly incomplete selection.','Retain all unique attempted/completed geometry families; later variants may have different representative track pairs but membership uses the same family SHA256.'],
 timing=dict(checkpoint='exact min(start + 30s, original phase deadline - 2s publication reserve)',less_than_30_sec_possible='Only when original phase reserve bounds the child checkpoint earlier. Existing parent continuation can refuse near its own reserve.',new_phase_budget=False,fallback='Without sufficient valid yield evidence, original phase fallback stays available; not every request is guaranteed to stop at 30s.'),
 evidence_scope=dict(completed_family='At least one final still-bound failed precheck inside listed geometry family, not every height or trajectory rejected.',unchecked_count='Original shortlist candidate entries only; never all height alternatives.',height_exhaustiveness_claimed=False,wire_version='Absent/explicit 1 retains exact legacy validation; explicit 2 new fields; unknown versions rejected.'),
 tests_scope='Real bounded selector, gain callback, mode height dispatch/receipt invalidation, actual audit commit/terminal handoff, original wire selection parsing and old v1 history, v2 builder→runner→task consumers. Strict transport/consumer geometry inputs remain synthetic; no physical or full recorded-trajectory success claimed.',
 merge_note='Separate modes-only package, no base remote methods from the first-qualified/runtime/recenter packages replaced. Apply full minimal diff including updated scheduling-intent tests.',
 retained='No collision, joint limits, exact ownership, current SDK, target identity, required view path validation or hardware guard converted to warning.',
 no_ros_master=True,no_physical_action=True,temperature_queried=False,actual_unchanged=True,
 prior_receipts=[str(N/'verification'/('gen16_contact_budget_checkpoint_r'+str(i))/'result.json') for i in range(1,5)])
(C/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(O/'delivery_manifest.json').write_bytes((C/'delivery_manifest.json').read_bytes())
result=dict(manifest=str(C/'delivery_manifest.json'),sha256=sha(C/'delivery_manifest.json'),mirrored_manifest_sha256=sha(O/'delivery_manifest.json'),files=len(files),actual_unchanged=True)
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
