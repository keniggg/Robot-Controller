from pathlib import Path
import hashlib, json, difflib
W=Path(__file__).resolve().parent.parent
C=W/'.runtime/gen16_success_first_contact_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen16_success_first_contact_delivery';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
baseline=json.loads((C/'baseline_sha256.json').read_text())
assert all(sha(W/r)==h for r,h in baseline.items())
r=N/'verification/gen16_success_first_contact_r3/result.json';receipt=json.loads(r.read_text());assert receipt['passed']
files=[];patch=''
for p in sorted(C.rglob('*.py')):
 rel=str(p.relative_to(C));source=W/rel
 files.append(dict(relative_path=rel,sha256=sha(p),baseline_sha256=baseline.get(rel),new_file=not source.exists()))
 patch+=''.join(difflib.unified_diff(source.read_text().splitlines(True) if source.exists() else [],p.read_text().splitlines(True),fromfile='a/'+rel if source.exists() else '/dev/null',tofile='b/'+rel))
(O/'candidate.patch').write_text(patch)
manifest=dict(status='candidate_verified_not_installed',package='gen16_success_first_contact',files=files,
 verification=dict(path=str(r),sha256=sha(r),**receipt['summary']),patch=dict(path=str(O/'candidate.patch'),sha256=sha(O/'candidate.patch')),
 production_changes=['DIRECT_NEAR_FIELD_COMPARISON_SEC 8.0 -> 0.0; retain 2s publication reserve and original phase',
 'comparison-search docstring only; executable method unchanged',
 '_build_streaming_gate_audit_report one selection_policy field next to comparison budget'],
 audit_termination='Existing live-selected postselection label NEAR_FIELD_COMPARISON_BUDGET_REACHED retained; expired/inactive phase clears selected.',
 retained='All strict feasibility checks, candidate cost validation, mode/ticket/source/publication checks and original deadline remain.',
 merge_note='Recenter sibling also edits _build_streaming_gate_audit_report; apply minimal hunks, never replace method or file from one package.',
 scope='Offline real bounded selector, malformed strict replies, actual post-selection AST live/expiry/inactive branches and publication regressions; no actual candidate or grasp success claimed.',
 no_ros_master=True,no_physical_action=True,temperature_queried=False,actual_unchanged=True)
(C/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(O/'delivery_manifest.json').write_bytes((C/'delivery_manifest.json').read_bytes())
result=dict(manifest=str(C/'delivery_manifest.json'),sha256=sha(C/'delivery_manifest.json'),mirrored_manifest_sha256=sha(O/'delivery_manifest.json'),actual_unchanged=True)
(O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
old=N/'verification/gen16_success_first_contact_r1'
if not (old/'result.json').exists():
 (old/'result.json').write_text(json.dumps(dict(passed=False,scope='Test bootstrap failed before collection: built ROS message PYTHONPATH missing. Original pytest log/XML preserved.',superseded_by=str(r),log_sha256=sha(old/'pytest.log'),xml_sha256=sha(old/'pytest.xml')),indent=2)+'\n')
print(json.dumps(result))
