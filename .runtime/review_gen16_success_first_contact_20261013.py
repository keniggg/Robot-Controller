import ast, hashlib, json, time
from pathlib import Path
import xml.etree.ElementTree as ET
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
ROOT=W.parent.parent
C=W/'.runtime/gen16_success_first_contact_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=C/'delivery_manifest.json'
expected='f084fc558c03e30c69e2bbd5b855ff854fd3047136b01ab96d7342130b32cb67'
assert sha(manifest)==expected
m=json.loads(manifest.read_text())
evidence={str(manifest):expected}
base_candidates={}
for f in m['files']:
 p=C/f['relative_path']; assert sha(p)==f['sha256']
 actual=W/f['relative_path']
 if f['new_file']: assert not actual.exists()
 else: assert sha(actual)==f['baseline_sha256']
 base_candidates[f['relative_path']]={'base_sha256':f['baseline_sha256'],'candidate_sha256':f['sha256']}
for k in ('verification','patch'):
 p=Path(m[k]['path']);assert sha(p)==m[k]['sha256'];evidence[str(p)]=sha(p)
rp=Path(m['verification']['path']);r=json.loads(rp.read_text())
assert r['passed'] and r['returncode']==0 and r['summary']==dict(tests=30,failures=0,errors=0,skipped=0)
assert r['candidate_source_sha256']==r['after_sha256']
for p,h in r['candidate_source_sha256'].items():
 f=Path(p);f=f if f.is_absolute() else ROOT/f
 assert sha(f)==h;(evidence.__setitem__(str(f),h))
for filename,key in [('pytest.log','log_sha256'),('pytest.xml','xml_sha256')]:
 p=rp.parent/filename;assert sha(p)==r[key];evidence[str(p)]=sha(p)
root=ET.parse(rp.parent/'pytest.xml').getroot()
suites=root.findall('testsuite');assert len(suites)==1
assert {k:int(suites[0].get(k)) for k in ('tests','failures','errors','skipped')}==r['summary']
ids=[x.get('classname')+'::'+x.get('name') for x in suites[0].findall('testcase')]
assert len(ids)==len(set(ids))==30
assert Path(r['loaded']['remote'])==C/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
release=N/'verification/gen15_phase_timestamp_release_manifest.json'
assert sha(release)=='16fe70b0bc0699df1ea0faac978dafa2bc4b45d7b0be483e1a679ab9eaefb11b'
sources=json.loads(release.read_text())['source_sha256'];assert len(sources)==908
assert all(sha(W/p)==h for p,h in sources.items())
evidence[str(release)]=sha(release)
prior=N/'verification/gen15_phase_timestamp_integrated_install_r2/result.json'
scope=set(json.loads(prior.read_text())['test_scope']);assert len(scope)==165
related=['src/alicia_flexible_grasp_supervisor/tests/'+x for x in ['test_grasp6d_pipeline.py','test_remote_grasp6d_streaming.py','test_contact_success_first_selection.py']]
scope.update(related);assert len(scope)==168
evidence[str(prior)]=sha(prior)
p='src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
a=ast.parse((W/p).read_text());b=ast.parse((C/p).read_text())
class Normalize(ast.NodeTransformer):
 def visit_Expr(self,node):
  if isinstance(node.value,ast.Constant) and isinstance(node.value.value,str):return None
  return self.generic_visit(node)
 def visit_Assign(self,node):
  if any(isinstance(x,ast.Name) and x.id=='DIRECT_NEAR_FIELD_COMPARISON_SEC' for x in node.targets):node.value=ast.Constant(value=8.0)
  return self.generic_visit(node)
 def visit_Dict(self,node):
  keep=[i for i,k in enumerate(node.keys) if not (isinstance(k,ast.Constant) and k.value=='selection_policy' and isinstance(node.values[i],ast.Constant) and node.values[i].value=='first_complete_hard_safe_candidate')]
  node.keys=[node.keys[i] for i in keep];node.values=[node.values[i] for i in keep]
  return self.generic_visit(node)
assert ast.dump(Normalize().visit(a),include_attributes=False)==ast.dump(Normalize().visit(b),include_attributes=False)
review=dict(schema=1,kind='gen16_success_first_contact_independent_review',passed=True,recorded_ns=time.time_ns(),delivery_manifest=str(manifest),delivery_sha256=expected,
 test_scope=sorted(scope),focused_test_scope=related,focused_test_summary=r['summary'],focused_selected=r['selected'],focused_case_ids=ids,
 base_and_candidate_sha256=base_candidates,evidence_sha256=evidence,actual_source_count=len(sources),actual_sources_unchanged=True,
 executable_changes_limited_to_comparison_constant_and_audit_policy=True,blocking_findings=[],review_method='Independent read-only source/control-flow/AST, SHA and existing XML review; no tests rerun.',
 findings=[
 'The executable comparison checker is unchanged. Setting its additional-comparison constant to zero closes continuation after the first complete strict result, retaining the original task deadline and two-second publication reserve.',
 'Real bounded_moveit_select still checks candidate hard-safety binding and validates strict flags, finite nonnegative path metrics and joint delta. Failed/incomplete results continue through remaining candidates; no incumbent is synthesized.',
 'Actual unchanged post-selection branches retain a live incumbent but clear it for inactive or expired phase. Preview rebuild invokes final registered geometry and strict sequence again; selected runtime, ticket, target epoch, phase precheck and promotion generation/token checks remain unchanged.',
 'Existing owner/ticket checks are retained rather than newly proven for all races by these 30 tests. New actual AST tests prove live/expired/inactive post-selection behavior; full owner/publication checks were also inspected statically.',
 'The current qualified result is accepted by the existing strict contract, not certified as the globally best path. This intentionally trades additional path-cost comparison for prompt publication.',
 'Same audit method also receives sibling recenter changes: merge minimal hunks and preserve both audit fields. Combined-source regression is required after merging.'
 ],limitations=[
 'The 30 existing focused cases use synthetic strict service replies and do not prove availability of a physical candidate or successful grasp.',
 'No planner RPC, ROS master, physical motion, combined installation or deployment was performed for this review.',
 'C1 does not change the two-family early-yield policy before a qualified incumbent; that distinct search-policy issue remains separate.',
 'The 168-file test_scope is a required combined regression scope, not a claim that 168 files ran in this focused evidence.'
 ],installed=False,physical_grasp_verified=False,motion_commands_sent=False,live_ros_contacted=False,temperature_queried=False)
out=W/'.runtime/gen16_success_first_contact_independent_review.pending.json'
out.write_text(json.dumps(review,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
print(json.dumps(dict(passed=True,pending=str(out),sha256=sha(out),test_scope_count=len(scope),actual_source_count=len(sources),focused_tests=len(ids))))
