#!/usr/bin/env python3
"""Read-only paired GEN47 result/source/XML review; no tests, ROS or hardware."""
from pathlib import Path
from collections import Counter
import hashlib,json,xml.etree.ElementTree as ET,time
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT=V/'gen47_paired_identity_independent_review_r1'
PROD='src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
TEST='src/alicia_grasp_modes/tests/test_gen47_prepared_identity_handoff.py'
FIXTURE='src/alicia_grasp_modes/tests/fixtures/gen47_g46_request2_metadata.json'
OLD='504d45fcb21bae603113080b4ab7df9e3c33689ba149e805c92ec4687d798028'
NEW='4ecadf299611153f32893c7f0ae009848106d5ab7236975a627790c5dcd47d28'
evidence={}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pin(p,expected=None):
 p=Path(p);h=sha(p)
 if expected is not None:assert h==expected,(str(p),h,expected)
 evidence[str(p)]=h;return h
def cases(p):
 rows=[]
 for c in ET.parse(p).getroot().iter('testcase'):
  error=c.find('failure');kind='failure'
  if error is None:error=c.find('error');kind='error'
  if error is None:error=c.find('skipped');kind='skipped'
  rows.append(dict(classname=c.get('classname'),name=c.get('name'),state=kind if error is not None else 'passed',message=error.get('message') if error is not None else None,text=error.text if error is not None else None))
 return rows
runs={}
for variant,mode_sha in [('baseline',OLD),('candidate',NEW)]:
 d=V/f'gen47_{variant}_focus_r1';p=d/'result.json';pin(p)
 q=json.loads(p.read_text());manifest=Path(q['candidate_manifest_path']);pin(manifest,q['candidate_manifest_sha256']);m=json.loads(manifest.read_text());root=Path(m['workspace_root'])
 assert q['source_sha256']==m['source_sha256'];assert len(q['source_sha256'])==1052
 for rel,h in q['source_sha256'].items():assert sha(root/rel)==h,(variant,rel)
 assert q['source_sha256'][PROD]==mode_sha
 assert q['source_sha256'][TEST]=='d09f079eb30880d5982d65abf7ec24e5f5e900015700cfc1560e0ffea862f2a6'
 assert q['source_sha256'][FIXTURE]=='2c6b3a884baa9449bc00cf4e0bc3e9391437f386cac6d600f193d57d20f1cb6c'
 pin(d/'pytest.xml',q['xml_sha256']);r=cases(d/'pytest.xml')
 assert len(r)==101 and q['scope_complete'] and q['source_unchanged']
 assert q['external_boundary_attempts']==[] and q['foreign_or_inconsistent_imports']=={}
 assert q['residual_offline_process_groups']==[] and q['combined_xml_matches_worker_results']
 assert q['duplicate_case_identities']==q['missing_case_identities']==q['unexpected_case_identities']==[]
 assert not q['interrupted'] and q['launcher_error'] is None
 loaded=[]
 for worker_path,h in q['worker_result_sha256'].items():
  wp=Path(worker_path);pin(wp,h);wr=json.loads(wp.read_text());pr=wp.parent/'module_provenance.json';pin(pr,wr['module_provenance_sha256']);pv=json.loads(pr.read_text())
  pin(wp.parent/'pytest.xml',wr['xml_sha256']);pin(wp.parent/'pytest.log',wr['log_sha256'])
  assert wr['scope_complete'] and wr['source_unchanged'] and wr['changed_source_loaded']
  assert wr['external_boundary_attempts']==[] and wr['foreign_or_inconsistent_imports']=={}
  assert not any(wr[k] for k in ['live_ROS_contacted','hardware_contacted','motion_commands_sent','temperature_queried'])
  actual=[dict(label=label,**row) for label,row in pv.items() if row.get('path')==str(root/PROD)]
  assert actual and all(row['sha256']==mode_sha for row in actual)
  loaded.extend(actual)
 counts={}
 for row in r:
  b=counts.setdefault(row['classname'],Counter());b[row['state']]+=1
 assert sum(row['state']=='failure' for row in r)==q['summary']['failures']
 assert sum(row['state']=='error' for row in r)==q['summary']['errors']==0
 assert sum(row['state']=='skipped' for row in r)==q['summary']['skipped']==0
 runs[variant]=dict(result_sha256=sha(p),summary=q['summary'],passed=q['passed'],source_map=q['source_sha256'],case_rows=r,per_file={k:dict(c) for k,c in counts.items()},loaded_mode_provenance=loaded,result_path=str(p),manifest_path=str(manifest),case_identities=q['collected_case_identities'])
b,c=runs['baseline'],runs['candidate']
assert set(b['source_map'])==set(c['source_map'])
source_diff={p:[b['source_map'][p],c['source_map'][p]] for p in b['source_map'] if b['source_map'][p]!=c['source_map'][p]}
assert source_diff=={PROD:[OLD,NEW]}
assert b['case_identities']==c['case_identities']
assert b['summary']==dict(tests=101,failures=9,errors=0,skipped=0) and not b['passed']
assert c['summary']==dict(tests=101,failures=0,errors=0,skipped=0) and c['passed']
failures=[]
for row in b['case_rows']:
 if row['state']!='failure':continue
 name=row['name']
 if name in ['test_real_measured_independent_prepare_reaches_original_accept_source_gate','test_silhouette_independent_helper_keeps_the_exact_original_bound_instance']:
  category='direct_original_source_gate_identity_failure';reason='Original helper bound an instance; baseline post-bind profile dataclass replacement returns a different instance. Original require raises the exact G46 ValueError.'
  assert row['message']=='ValueError: observation view request, camera, handeye or source points changed'
 elif name.startswith('test_duplicate_or_delayed_accept_cannot_transfer_the_bound_instance') or name.startswith('test_original_live_calibration_gate_remains_strict_after_identity_handoff'):
  category='new_negative_case_healthy_prerequisite_identity_failure';reason='Healthy bound-is-returned prerequisite fails before duplicate/delay or camera/handeye fault injection. This is derivative of the same baseline replacement, not evidence that injected faults were admitted.'
  assert '_diagnostic_observation_view_prepared is prepared' in row['text']
 elif name in ['test_normal_observation_profiles_the_final_instance_before_original_bind','test_unbound_normal_prepare_preserves_profile_without_transfer_of_old_owner']:
  category='new_profile_assertion_fixture_derivative';reason='Baseline fixture initially prepared request77; changing the synthetic response ticket to2 retains its historical profile77. profiled_prepared merge rejects mismatching tickets then intentionally keeps the old object/profile. Candidate independent initial fixture has no profile and records current ticket2. This assertion is diagnostic fixture behavior, not an additional physical/source-gate failure.'
  assert row['message']=='assert 77 == 2'
 elif name=='test_regular_prepare_captures_original_before_inference_and_binds_final_prepared':
  category='unchanged_supplement_original_identity_failure';reason='Preexisting unchanged ordinary-route test directly asserts bound is returned. Baseline profiles after original bind, so this existing assertion fails; candidate profiles before bind and passes.'
  assert '_diagnostic_observation_view_prepared is result' in row['text']
 else:raise AssertionError(name)
 failures.append(dict(case=name,file=row['classname'],category=category,message=row['message'],reason=reason))
assert Counter(x['category'] for x in failures)==Counter(direct_original_source_gate_identity_failure=2,new_negative_case_healthy_prerequisite_identity_failure=4,new_profile_assertion_fixture_derivative=2,unchanged_supplement_original_identity_failure=1)
for r in runs.values():r.pop('source_map');r.pop('case_rows');r.pop('case_identities')
report=dict(kind='gen47_paired_identity_handoff_independent_review',reviewed=True,passed=True,timestamp_ns=time.time_ns(),review_scope='Completed paired focus only; no full-regression/release/deployment/physical success claim',runs=runs,only_source_difference=source_diff,paired_case_identities_exact=True,source_maps_read_back_all_1052_each=True,old_tests_changed=False,new_test_cases=13,supplemental_existing_cases=88,baseline_supplement_summary=dict(passed=87,failures=1,errors=0,skipped=0),candidate_supplement_summary=dict(passed=88,failures=0,errors=0,skipped=0),baseline_new_summary=dict(passed=5,failures=8,errors=0,skipped=0),candidate_new_summary=dict(passed=13,failures=0,errors=0,skipped=0),baseline_failure_attribution=failures,guard_boundary_attempts=0,foreign_imports=0,source_unchanged=True,residual_offline_process_groups=[],limitations=['The real Mode prepare -> independent helper -> original bind/require -> original accept source gate is exercised with explicitly synthetic RGBD/TF/geometry providers and an empty endpoint proposal provider. It does not replay original G46 RGBD or validate a MoveIt trajectory.','Metadata fixture pins original G46 request2 error/profile; original physical record remains a failure.','Duplicate/delayed ownership, typed cancellation and calibration fault cases all pass on candidate, while baseline negative tests may stop at the healthy-instance prerequisite.','Independent successful preparation intentionally omits the optional mode preparation profile to preserve the already-bound frozen instance; legacy ros_prepare_ms and preparation exception profile checks remain exercised.'],evidence_sha256=evidence,live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False,temperature_queried=False,tests_run_by_this_review=False,full_regression_passed=False,deployed=False,physical_grasp_verified=False)
OUT.mkdir(exist_ok=False);raw=(json.dumps(report,ensure_ascii=False,indent=2)+'\n').encode();(OUT/'review.json').write_bytes(raw);assert (OUT/'review.json').read_bytes()==raw
seal=dict(review_sha256=hashlib.sha256(raw).hexdigest(),helper_path=str(Path(__file__)),helper_sha256=sha(Path(__file__)),evidence_sha256=evidence);(OUT/'seal.json').write_text(json.dumps(seal,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(review_path=str(OUT/'review.json'),review_sha256=sha(OUT/'review.json'),seal_sha256=sha(OUT/'seal.json'),baseline=report['runs']['baseline']['summary'],candidate=report['runs']['candidate']['summary']),ensure_ascii=False))
