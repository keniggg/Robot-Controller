"""Finalize independent GEN53 static/source/focus review; not release/deployment."""
import argparse,ast,hashlib,json,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
REVIEWS=(
 'gen53_parallel_runner_independent_review_r1/result.json',
 'gen53_idle_helpers_independent_static_review_r1/result.json',
 'gen53_installer_sealer_independent_review_r1/result.json',
 'gen53_station_snapshot_retry_independent_review_r1/result.json',
 'gen53_support_registration_independent_review_r1/result.json',
 'gen53_reacquisition_independent_review_r3/result.json')
HELPERS=(
 'verify_gen53_parallel_offline_20261018.py','install_gen53_validated_candidate_20261018.py',
 'seal_gen53_release_20261018.py','gen53_idle_reload_common_20261018.py',
 'read_gen53_preinstall_readiness_20261018.py','read_gen53_deployment_preflight_20261018.py',
 'load_gen53_idle_compute_gateway_20261018.py','finalize_gen53_idle_deployment_20261018.py',
 'gen53_observation_measurement_consumers_20261018.launch',
 'start_gen53_observation_measurement_consumers_20261018.sh')
EVIDENCE=(
 'gen53_g52_original_box_qualification_r1/result.json',
 'gen53_actual_snapshot_policy_r1/result.json',
 'gen53_r18_support_plane_final_qualification_r1/result.json',
 'gen53_r18_support_plane_final_qualification_r1/final_qualification.json',
 'gen53_r18_materialization_r1/result.json',
 'gen53_r18_materialization_r1/materialized_candidates.json',
 'gen53_combined_focus_r1/result.json',
 'gen53_reacquisition_preparation_r3/preparation.json',
 'gen53_reacquisition_preparation_r3/focus_r1_defect.json',
 'gen52_actual_handoff_monitor_r1/final_review_r2.json',
 'gen52_actual_handoff_monitor_r1/point1056_origin_r1/result.json',
 'gen52_actual_handoff_monitor_r1/exact_box_inputs_r1/result.json')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def main():
 ap=argparse.ArgumentParser(description=__doc__)
 for name in('manifest','focus'):
  ap.add_argument('--'+name,type=Path,required=True);ap.add_argument('--'+name+'-sha256',required=True)
 ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
 assert Path('/media/zhuyupei/SanStand').is_mount()and V in a.output.resolve().parents and not a.output.exists()
 assert sha(a.manifest)==a.manifest_sha256 and sha(a.focus)==a.focus_sha256
 m,f=read(a.manifest),read(a.focus);sources=m['source_sha256'];C=Path(m['workspace_root']);B=W/'.runtime/gen53_baseline_source/runner_manifest.json';baseline=read(B)['source_sha256']
 # This receipt is a collected-candidate installation review.  A future
 # installed-workspace full run is a separate release obligation.
 assert m['kind']=='GEN53_frozen_offline_candidate'and m['full_scope_ready']is False and len(baseline)==1065
 assert len(sources)==1069
 assert f['passed']and f['mode']=='focus'and f['scope_complete']and f['source_unchanged']and f['test_bodies_executed']
 assert f['source_sha256']==sources and not f['external_boundary_attempts']and not f['foreign_or_inconsistent_imports']
 assert f['summary']['tests']>0 and all(f['summary'][k]==0 for k in('failures','errors','skipped'))
 expected_new=[tuple(identity)for row in m['new_tests']for identity in row['case_identities']]
 xml=a.focus.parent/'pytest.xml';assert sha(xml)==f['xml_sha256']
 xmlcases=list(ET.parse(xml).getroot().iter('testcase'))
 actual_new=[(c.attrib['file'],c.attrib['classname'],c.attrib['name'])for c in xmlcases]
 assert len(expected_new)==len(set(expected_new))==43 and len(actual_new)==43 and set(actual_new)==set(expected_new)
 assert all(not any(c.find(k)is not None for k in('failure','error','skipped'))for c in xmlcases)
 assert not any(f[k]for k in('duplicate_case_identities','missing_case_identities','unexpected_case_identities','residual_offline_process_groups'))
 assert not f['interrupted']and not f['full_regression_passed']
 for p,h in sources.items():assert sha(C/p)==h
 for p,h in baseline.items():assert sha(B.parent/p)==h
 changed={r['path']:r['candidate_sha256']for r in m['production_source_changes']}
 changed.update({r['path']:r['sha256']for r in m.get('new_production_sources',[])})
 assert len(changed)==5 and all(sources[p]==h for p,h in changed.items())
 assert set(p for p in baseline if sources[p]!=baseline[p])==set(r['path']for r in m['production_source_changes'])
 assert all(f['loaded_changed_sources_global'].get(p)is True for p in changed)
 deps={str(a.manifest.resolve()):a.manifest_sha256,str(a.focus.resolve()):a.focus_sha256,str(xml):sha(xml),str(B):sha(B)}
 reviewed={str(W/'.runtime'/name):sha(W/'.runtime'/name)for name in HELPERS}
 reviewed_production={}
 for rel in REVIEWS:
  p=V/rel;r=read(p);assert r['passed'];deps[str(p)]=sha(p)
  # Earlier scopes may have inspected an older NEW test revision. Bind those
  # immutable receipts as history, recheck their production/helper pins, and
  # use the final source map + final r3 review for the revised NEW test bytes.
  # Never flatten historical live-path->oldbyte dependencies into the seal.
  for path,h in r.get('reviewed_files_sha256',{}).items():
   if '/tests/' not in path:assert sha(path)==h
  for path,h in r.get('source_sha256',{}).items():
   if path in changed:assert h==changed[path];reviewed_production[path]=h
 assert reviewed_production==changed
 for rel in EVIDENCE:deps[str(V/rel)]=sha(V/rel)
 oldfocus=read(V/'gen53_combined_focus_r1/result.json')
 assert oldfocus['passed']is False and oldfocus['interrupted']is True and oldfocus['summary']==dict(tests=521,failures=0,errors=0,skipped=0)
 for path,h in deps.items():assert sha(path)==h
 for name in HELPERS:
  if name.endswith('.py'):ast.parse((W/'.runtime'/name).read_text())
 result=dict(passed=True,review_scope='static_production_runner_and_idle_loader',
  source_sha256=sources,source_count=len(sources),reviewed_files_sha256=reviewed,
  reviewed_production_sources=changed,dependency_sha256=deps,
  original_failure_replay_reviewed=True,physical_success_claimed=False,
  candidate_manifest=dict(path=str(a.manifest.resolve()),sha256=a.manifest_sha256),
  focused_regression=dict(path=str(a.focus.resolve()),sha256=a.focus_sha256,summary=f['summary']),
  earlier_interrupted_focus=dict(path=str(V/'gen53_combined_focus_r1/result.json'),sha256=sha(V/'gen53_combined_focus_r1/result.json'),passed=False,interrupted=True,completed_historical_case_summary=oldfocus['summary'],overall_pass_claimed=False),
  baseline_source_count=1065,historical_test_files=320,historical_tests=8967,
  old_test_bytes_unchanged=True,blocking_findings=[],
  findings=['FrozenG52 exact original failure binds raw-box-qualified vs fusedconflict; complete support-plane alternative passes unchanged fullpoint consistency/registration/append/boxqualification.','All1563preappend points are checked before future fusion; no rejectedpoint deletion or1mmthreshold change.','Newstationinput policy is only a single new3source window after exclusively missingevidence/zero-path complete negative; originalG52 geometryconflict stays ineligible for it.','The newwindow is consumed inside original successfulsubmissionlock and cannot reopen through expiry/prepcancel; currentowner/CAS/phase/session remain strict.','All5final productionbytes, oldtestbytes, collectedscope and actualguardedfocus bind to one source map; no stale-live-production digest is used as immutable dependency.','Originalmaterialization produces6local contact candidates under frozenYAML+recorded dynamic uncertainty; fullsweptpath/IK/MoveIt/Task/physicalgrasp are not certified by that probe.'],
  materialization_limits='Localadaptivegrid+placement/contactevidence/overlap/opening only; no boundarysearch/fulltrajectory/IK/MoveIt/execution. FrozenYAML is not complete historicalROSparameter reconstruction.',
  full_regression_not_yet_claimed=True,installed_full_expected_cases=9010,installed_full_expected_test_files=322,
  candidate_manifest_full_scope_ready=False,runtime_deployed=False,ROS_hardware_or_temperature_access=False,
  idle_helpers_reviewed_as_static_only=True,
  runtime_identity_revalidation_required='The current user-requested whole-stack restart can change node identity and accepted previous deployment. Existing GEN53 idle helper pins are reviewed against the old GEN52 deployment; refresh these pins and the static receipt before using them with restarted nodes.',
  production_or_old_tests_modified=False,created_ns=time.time_ns(),helper_sha256=sha(__file__))
 a.output.mkdir();p=a.output/'result.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert read(p)==result
 print(json.dumps(dict(result=str(p),sha256=sha(p),passed=True,source_count=len(sources),focus=f['summary']),ensure_ascii=False))
if __name__=='__main__':main()
