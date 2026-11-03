from pathlib import Path
import hashlib,json,difflib
W=Path(__file__).resolve().parent.parent;C=W/'.runtime/gen16_height_recipe_policy_test_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');O=N/'verification/gen16_height_recipe_policy_test_delivery';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
R=N/'verification/gen16_height_recipe_policy_test_r1/result.json';r=json.loads(R.read_text());assert r['passed']
rel='src/alicia_flexible_grasp_supervisor/tests/test_selected_contact_height_recipe.py';assert sha(W/rel)==r['candidate']['baseline_sha256'];assert sha(C/rel)==r['candidate']['after_sha256']
patch=''.join(difflib.unified_diff((W/rel).read_text().splitlines(True),(C/rel).read_text().splitlines(True),fromfile='a/'+rel,tofile='b/'+rel));(O/'candidate.patch').write_text(patch)
manifest=dict(package='gen16_height_recipe_policy_test',status='candidate_verified_not_installed',files=[dict(relative_path=rel,baseline_sha256=sha(W/rel),sha256=sha(C/rel),new_file=False)],verification=dict(path=str(R),sha256=sha(R),**r['summary']),patch=dict(path=str(O/'candidate.patch'),sha256=sha(O/'candidate.patch')),test_only=True,production_modified=False,
 failure=dict(path=r['original_failure']['path'],sha256=r['original_failure']['sha256'],cases=['test_observe_return_override_forwards_options_only_after_original_yield_policy[True-'+v+']' for v in ('preferred','skip','both','ordinary')],cause='Old test required a measured-view opportunity to suppress the original height search. The approved success-first policy explicitly removed that scheduling shortcut.'),
 changes=['Rename defer parameter to measured_opportunity; all 8 option/opportunity combinations forward once with the exact original deadline and option identities.','Assert no deferred audit or family marker is invented.','Add 2 cases using production ObserveReturnMixin + real RemoteGrasp6DNode height method chain: original failure invokes height retry, selected geometry is preserved, publication performs a fresh strict check at unchanged 123s deadline.'],
 retained_tests='All other module AST is identical, including 14 binding changes, rejected geometry/strict result fallback, deadline before/after, real service adapter and cancellation. Existing height-proposal rejection suite also passes.',
 actual_sources_verified_unchanged=r['actual_count'],no_ros_master=True,no_motion_commands=True,temperature_queried=False,
 scope='Synthetic strict transport only; no claim of an actual successful grasp or correction of the separate station-slot failure.')
(C/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(O/'delivery_manifest.json').write_bytes((C/'delivery_manifest.json').read_bytes())
result=dict(manifest=str(C/'delivery_manifest.json'),sha256=sha(C/'delivery_manifest.json'),mirror_sha256=sha(O/'delivery_manifest.json'),file_count=1);(O/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
