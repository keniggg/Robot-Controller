import ast,copy,hashlib,json,time
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
C=W/'.runtime/gen16_combined_success_first_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V=N/'verification';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=C/'delivery_manifest_r2.json';expected='7ab6440e7027e0825686a14f21a6d72bbf0b1f23bda38e1d9cdd9215972d006b'
assert sha(p)==expected;m=json.loads(p.read_text());assert m['passed']
original=C/'delivery_manifest.json';assert sha(original)=='286e764ca1765381a0fc40545e3c556897812d7a213abf7f6023cd1363632a32'
old=json.loads(original.read_text());assert m['files']==old['files'] and m['patch_sha256']==old['patch_sha256']
assert set(m['test_scope'])-set(old['test_scope'])=={'src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py'}
evidence={str(p):expected,str(original):sha(original)}
for x,h in m['parent_sha256'].items():assert sha(x)==h;evidence[x]=h
assert sha(C/'delivery.patch')==m['patch_sha256'];evidence[str(C/'delivery.patch')]=m['patch_sha256']
packages=[
 ('gen16_runtime_parameter_transaction_20261013','gen16_runtime_transaction_independent_review','839a531470e189326a2c6ed0e3f4cfde8c0881577946408afd80f8a52e6bc0b1','gen16_runtime_parameter_transaction_r3'),
 ('gen16_contact_margin_recenter_20261013','gen16_contact_recenter_independent_review','75f3f8ec8f7dfdeb39b8b29d90cc0b79214cbda8fedc8daf0f50a66f87f615b7','gen16_contact_recenter_combined_r2'),
 ('gen16_success_first_contact_20261013','gen16_success_first_contact_independent_review','eb43da414cfce344ceb3e1e25e7fb452260a5c50365232de3a1aae4663059d6c','gen16_success_first_contact_r3'),
 ('gen16_session_ack_wait_20261013','gen16_session_ack_wait_independent_review','e13aad068fc3d0d69a7b600b23c2a821fe414667f1677d5736fdc68b257aa97d','gen16_session_ack_wait_r1'),
 ('gen16_contact_budget_checkpoint_20261013','gen16_contact_budget_checkpoint_independent_review','640459fd430eb1caa4a1fa4267e188b3ed3bafa8101d68447c5c382dad6eeab7','gen16_contact_budget_checkpoint_r5')]
parentfiles={};scope=set();parentinfo=[]
for stage,review_name,rh,result_name in packages:
 parent=W/'.runtime'/stage/'delivery_manifest.json';d=json.loads(parent.read_text())
 review=V/review_name/'result.json';assert sha(review)==rh;rv=json.loads(review.read_text());assert rv['passed'] is True
 assert sha(parent)==rv.get('delivery_sha256',rv.get('manifest_sha256'))
 result=V/result_name/'result.json';res=json.loads(result.read_text());assert res['passed'] is True
 if 'test_scope' in rv:review_scope=set(rv['test_scope'])
 else:
  assert stage=='gen16_contact_budget_checkpoint_20261013'
  assert res['returncode']==0
  assert sha(result)==rv['verification']['sha256']
  review_scope=set()
  for selected in res['selected']:
   f=Path(selected.split('::')[0]);root=parent.parent if parent.parent in f.parents else W
   review_scope.add(str(f.relative_to(root)))
 scope.update(review_scope)
 for item in d['files']:
  path=item.get('path',item.get('relative_path'));base=item.get('base_sha256',item.get('baseline_sha256'));candidate=item.get('candidate_sha256',item.get('sha256'))
  assert sha(parent.parent/path)==candidate
  assert (sha(W/path) if (W/path).exists() else None)==base
  parentfiles.setdefault(path,[]).append(dict(manifest=str(parent),base_sha256=base,candidate_sha256=candidate))
 for f in (parent,review,result):evidence[str(f)]=sha(f)
 if stage.startswith('gen16_runtime_'):assert sha(result)==d['focused_regression']['sha256']
 elif stage.startswith('gen16_contact_margin_'):assert sha(result)==d['validation'][0]['sha256']
 else:assert sha(result)==d['verification']['sha256']
 summary=res.get('summary',res.get('test_summary',res.get('counts')))
 if summary is None and 'focused' in res:summary=res['focused']['summary']
 if summary is not None:assert not any(summary[k] for k in ('failures','errors','skipped'))
 parentinfo.append(dict(delivery=str(parent),delivery_sha256=sha(parent),review=str(review),review_sha256=rh,passed_test_result=str(result),passed_test_result_sha256=sha(result),test_scope=sorted(review_scope),test_summary=summary))
assert len(scope)==176 and set(m['test_scope'])==scope and len(m['test_scope'])==176
assert len(m['files'])==19 and {x['path'] for x in m['files']}==set(parentfiles)
remote='src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
for row in m['files']:
 path=row['path'];assert sha(C/path)==row['candidate_sha256']
 parents=parentfiles[path];assert all(x['base_sha256']==row['base_sha256'] for x in parents)
 assert set(row['parent_deliveries'])=={x['manifest'] for x in parents}
 if path!=remote:
  assert len(parents)==1 and row['candidate_sha256']==parents[0]['candidate_sha256']
base=ast.parse((W/remote).read_text());combined=ast.parse((C/remote).read_text())
def classnode(tree):return next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='RemoteGrasp6DNode')
def methods(tree):return {x.name:x for x in classnode(tree).body if isinstance(x,ast.FunctionDef)}
def dump(node):return ast.dump(node,include_attributes=False)
base_methods=methods(base);combined_methods=methods(combined)
parents={name:ast.parse((W/'.runtime'/name/remote).read_text()) for name in ('gen16_runtime_parameter_transaction_20261013','gen16_contact_margin_recenter_20261013','gen16_success_first_contact_20261013')}
expected_methods={
 '_refresh_runtime_params':'gen16_runtime_parameter_transaction_20261013',
 '_prepare_final_registered_contact_plan':'gen16_contact_margin_recenter_20261013',
 '_recenter_final_registered_contact_height':'gen16_contact_margin_recenter_20261013',
 '_build_selected_preview_bundle':'gen16_contact_margin_recenter_20261013',
 '_direct_near_field_comparison_search':'gen16_success_first_contact_20261013'}
for method,parent in expected_methods.items():assert dump(combined_methods[method])==dump(methods(parents[parent])[method]),method
audit=copy.deepcopy(combined_methods['_build_streaming_gate_audit_report']);removed=[]
for node in ast.walk(audit):
 if isinstance(node,ast.Dict):
  for i,k in list(enumerate(node.keys))[::-1]:
   if isinstance(k,ast.Constant) and k.value=='selection_policy':
    assert isinstance(node.values[i],ast.Constant) and node.values[i].value=='first_complete_hard_safe_candidate'
    removed.append(node.values[i].value);del node.keys[i];del node.values[i]
assert len(removed)==1
assert dump(audit)==dump(methods(parents['gen16_contact_margin_recenter_20261013'])['_build_streaming_gate_audit_report'])
expected_changed=set(expected_methods)|{'_build_streaming_gate_audit_report'}
actual_changed={k for k,v in combined_methods.items() if k not in base_methods or dump(v)!=dump(base_methods[k])}
assert actual_changed==expected_changed
assert not (set(base_methods)-set(combined_methods))
normalized=copy.deepcopy(combined);cl=classnode(normalized);rebuilt=[]
for node in cl.body:
 if isinstance(node,ast.FunctionDef) and node.name in expected_changed:
  if node.name in base_methods:rebuilt.append(base_methods[node.name])
 else:rebuilt.append(node)
cl.body=rebuilt
for node in normalized.body:
 if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='DIRECT_NEAR_FIELD_COMPARISON_SEC' for t in node.targets):
  assert isinstance(node.value,ast.Constant) and node.value.value==0.;node.value.value=8.
assert dump(normalized)==dump(base)
release=V/'gen15_phase_timestamp_release_manifest.json';assert sha(release)=='16fe70b0bc0699df1ea0faac978dafa2bc4b45d7b0be483e1a679ab9eaefb11b'
sealed=json.loads(release.read_text())['source_sha256'];assert len(sealed)==908
assert all(sha(W/f)==h for f,h in sealed.items())
changed={x['path'] for x in m['files']}
assert all(sha(C/f)==h for f,h in sealed.items() if f not in changed)
assert all((C/f).is_file() for f in scope)
evidence[str(release)]=sha(release)
helper=W/'.runtime/merge_gen16_reviewed_candidates_20261013.py';evidence[str(helper)]=sha(helper)
report=dict(schema=1,kind='GEN16_COMBINED_INDEPENDENT_SOURCE_REVIEW',passed=True,recorded_ns=time.time_ns(),delivery_manifest=str(p),delivery_sha256=expected,test_scope=sorted(scope),test_scope_count=176,evidence_sha256=evidence,parents=parentinfo,files=m['files'],blocking_findings=[],actual_source_count=908,actual_sources_unchanged=True,unchanged_candidate_baseline_sources_verified=True,
 combined_remote_sha256=sha(C/remote),combined_remote_exact_parent_methods=expected_methods,combined_remote_audit_method='Exact recenter audit method plus the single C1 selection_policy field; no other AST differences.',
 findings=[
 'All 19 delivery paths are exactly the parent file union. The 18 single-parent files are byte-identical to their immutable parent deliveries.',
 'The shared remote module retains exact runtime refresh method, recenter entry/helper and selected audit, C1 first-qualified docstring/checker and zero-second comparison constant. Its streaming audit contains both recenter evidence and C1 selection policy.',
 'Whole remote module AST is baseline-identical after restoring the six explicitly reviewed methods and the single comparison constant. Both recorded textual merges completed without conflict.',
 'Combined C2 wrapper checks base continuation first, so C1 stops after a complete valid incumbent. Its incumbent state prevents yielding that success as missing evidence. Recenter geometry/strict checks still precede incumbent acceptance.',
 'C2 modes-only search/validator changes and ACK task changes occupy separate files and preserve their exact sealed sources. Ordinary height retries remain strict; the session ACK is metadata transport and does not authorize motion.',
 'Manifest r2 corrects the only discovered scope omission by adding unchanged test_grasp6d_pipeline.py. Five review scopes, with C2 normalized from its exact reviewed verification.selected list, union to 176 unique files.',
 'Merge helper normalizes C1/C2 schemas only in memory after validating passed test evidence and zero failures/errors/skips. Original five manifests and reviews retain their exact hashes.'
 ],scope_correction=dict(original_manifest=str(original),original_sha256=sha(original),original_count=175,added=['src/alicia_flexible_grasp_supervisor/tests/test_grasp6d_pipeline.py'],final_count=176),
 limitations=[
 'This is final combined-source review, not an integrated regression result. No tests were rerun and the exact combined candidate has not been claimed tested or deployed by this review.',
 'Parent focused results are independent overlapping scopes and must not be summed as a unique combined-test total. Root must install and run the full 176-file selected scope before deployment.',
 'Empirical contact margin evidence is not a certified physical bound; original R4 boundary reconstruction is not complete raw-cloud replay. ACK counterfactual acceptance is not recorded success.',
 'No physical grasp success is established. Existing geometry/collision, source, ownership and action-admission checks remain in force.'
 ],review_method='Read-only parent and combined source/diff/AST/SHA review; no pytest, ROS, physical action or actual-source mutation.',integrated_regression_performed=False,installed=False,deployed=False,physical_grasp_verified=False,live_ros_contacted=False,motion_commands_sent=False,temperature_queried=False)
out=W/'.runtime/gen16_combined_independent_review.pending.json';out.write_text(json.dumps(report,ensure_ascii=False,sort_keys=True,indent=2)+'\n')
print(json.dumps(dict(passed=True,pending=str(out),sha256=sha(out),file_count=19,test_scope_count=176,actual_source_count=908,remote_sha256=sha(C/remote))))
