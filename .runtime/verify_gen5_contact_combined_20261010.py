"""Single related-suite run against isolated SDK/yaw overlay; no ROS I/O."""
import hashlib,json,os,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET

WORK=Path(__file__).resolve().parents[1]
STAGE=WORK/'.runtime/gen5_contact_combined_staging_20261010'
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT=NEW/'verification/gen5_contact_combined'
assert Path('/media/zhuyupei/SanStand').is_mount()
if OUT.exists():
 previous=OUT/'attempt1_failed'
 assert not previous.exists(),previous
 previous.mkdir()
 for saved in list(OUT.iterdir()):
  if saved.is_file():saved.rename(previous/saved.name)
else:OUT.mkdir()
os.environ.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_LOG_DIR=str(OUT/'ros'))
sys.dont_write_bytecode=True
os.environ['ROS_PACKAGE_PATH']=str(STAGE/'src')+':'+str(WORK/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
paths=[STAGE/'src/alicia_grasp_modes/src',STAGE/'src/alicia_flexible_grasp_supervisor/src',
       STAGE/'src/alicia_grasp_modes/tests',STAGE/'src/alicia_flexible_grasp_supervisor/tests']
os.environ['PYTHONPATH']=':'.join(map(str,paths))+':'+os.environ.get('PYTHONPATH','')
sys.path[:0]=list(map(str,paths))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
base=json.loads((NEW/'initial_preflight_deployment/result.json').read_text())['source_sha256']
def live_hashes():return {p:sha(WORK/p) for p in base}
assert len(base)==135
assert live_hashes()==base,'live baseline135 changed before combined verification'
yaw=json.loads((WORK/'.runtime/gen5_contact_yaw_staging_20261010/delivery.json').read_text())
sdk_paths=['src/alicia_flexible_grasp_supervisor/'+p for p in (
 'src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
 'tests/test_contact_sample_collection.py','tests/test_contact_sequence_precheck.py')]
changed=sdk_paths+[r['path'] for r in yaw['files']]
assert len(set(changed))==8
for item in yaw['files']:assert sha(STAGE/item['path'])==item['sha256']
assert sha(STAGE/sdk_paths[0])=='ce8982f83946f14b7e3c26fc915c2896aee056f1b138a10dae42f59a0b0b1073'
changes=[dict(path=p,original_live_sha256=sha(WORK/p) if (WORK/p).exists() else None,
              staged_sha256=sha(STAGE/p)) for p in changed]
manifest=dict(stage=str(STAGE),baseline135=base,files=changes,
             sdk_test_source_assertion='Checks implementation path against exact colocated test package; portable to combined stage/live. No production change.',
             live_mutated=False)
(OUT/'deployment_changes.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
all_paths=[p for p in STAGE.rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml','.launch','.xml','.npz')]
def source_hashes():return {str(p.relative_to(STAGE)):sha(p) for p in all_paths}
before=source_hashes()
(OUT/'full_staged_source_sha256.json').write_text(json.dumps(before,indent=2,sort_keys=True)+'\n')

source_audit={}
class SourceAudit:
 def pytest_collection_finish(self,session):
  import rospkg
  for pkg in ('alicia_flexible_grasp_supervisor','alicia_grasp_modes'):
   found=Path(rospkg.RosPack().get_path(pkg)).resolve()
   assert found==(STAGE/'src'/pkg).resolve(),(pkg,found)
   source_audit['rospkg_'+pkg]=str(found)
  from alicia_flexible_grasp.robot import contact_sequence_precheck
  from alicia_flexible_grasp.grasp import tabletop_geometry_candidates
  import test_remote_modes,test_remote_grasp6d_streaming
  expected={
   'sdk':(contact_sequence_precheck,'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py'),
   'yaw':(tabletop_geometry_candidates,'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py'),
   'mode':(test_remote_modes.module,'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'),
   'mode_original':(test_remote_modes.module.original,'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'),
   'streaming_original':(test_remote_grasp6d_streaming.remote_node,'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py')}
  for name,(module,rel) in expected.items():
   actual=Path(module.__file__).resolve()
   assert actual==(STAGE/rel).resolve(),(name,actual)
   source_audit[name]=dict(path=str(actual),sha256=sha(actual))
  leaked={name:str(Path(mod.__file__).resolve()) for name,mod in list(sys.modules.items())
   if getattr(mod,'__file__',None) and any(str(Path(mod.__file__).resolve()).startswith(str(WORK/'src'/pkg)+'/')
    for pkg in ('alicia_flexible_grasp_supervisor','alicia_grasp_modes'))}
  assert not leaked,leaked
  source_audit['live_package_module_leaks']=leaked
  (OUT/'module_source_audit.json').write_text(json.dumps(source_audit,indent=2,sort_keys=True)+'\n')

sup='src/alicia_flexible_grasp_supervisor/tests/'
modes='src/alicia_grasp_modes/tests/'
suite=[sup+p for p in ('test_contact_sample_collection.py','test_contact_sequence_precheck.py',
 'test_stationary_following.py','test_gen5_contact_endpoint_yaw.py','test_tabletop_geometry_candidates.py')]
suite += [modes+p for p in ('test_remote_modes.py','test_observe_return_remote.py','test_initial_preflight_remote.py')]
streaming=(
 'test_local_pipeline_keeps_tabletop_candidate_when_graspnet_hits_table',
 'test_tabletop_generation_materializes_geometry_derived_stage_profiles',
 'test_tabletop_generation_and_collision_gate_share_frozen_support_plane',
 'test_final_registered_tabletop_restores_clearance_and_checks_actual_family',
 'test_tabletop_contact_boundary_resolves_live_uncertainty_and_is_stratified',
 'test_tabletop_bounded_batch_represents_each_live_proposal_before_repeats',
 'test_tabletop_stable_recheck_uses_fused_pose_without_depth',
 'test_tabletop_stable_recheck_refreshes_width_from_current_fused_surface',
 'test_tabletop_stable_recheck_reprojects_tracker_drift_to_support_clearance',
 'test_direct_near_field_rank_ignores_sub_wire_width_noise',
 'test_direct_near_field_rank_prefers_narrow_side_before_wrist_delta',
 'test_tabletop_surface_snapshot_is_local_to_one_generation')
suite += [sup+'test_remote_grasp6d_streaming.py::'+n for n in streaming]
args=['-q','-p','no:cacheprovider']+[str(STAGE/p) for p in suite]+['--junitxml',str(OUT/'tests.xml')]
started=time.time_ns()
import pytest
plugin=SourceAudit()
with (OUT/'pytest.log').open('w') as log:
 oldout,olderr=sys.stdout,sys.stderr
 try:
  sys.stdout=sys.stderr=log
  rc=pytest.main(args,plugins=[plugin])
 finally:sys.stdout,sys.stderr=oldout,olderr
plugin.pytest_collection_finish(None)  # Also audit late dependency imports.
cases=list(ET.parse(str(OUT/'tests.xml')).getroot().iter('testcase'))
unique={(c.get('classname'),c.get('name')) for c in cases}
failures=[dict(c.attrib,detail=(c.find('failure').text if c.find('failure') is not None else c.find('error').text)) for c in cases if c.find('failure') is not None or c.find('error') is not None]
unchanged=source_hashes()==before
live_unchanged=live_hashes()==base
result=dict(passed=rc==0 and unchanged and live_unchanged and bool(source_audit),
 source_unchanged=unchanged,source_sha256={p:before[p] for p in changed},unique_tests=len(unique),
 stage=str(STAGE),returncode=int(rc),baseline135=dict(count=len(base),matched_before=True,matched_after=live_unchanged),
 deployment_changes=str(OUT/'deployment_changes.json'),deployment_changes_sha256=sha(OUT/'deployment_changes.json'),
 actual_test_count=len(cases),test_nodeids=[c.get('classname')+'::'+c.get('name') for c in cases],test_selection=suite,
 failures=failures,module_source_audit=source_audit,
 command=['/usr/bin/python3','-m','pytest']+args,driver=str(Path(__file__).resolve()),
 pythonpath=os.environ['PYTHONPATH'],ros_package_path=os.environ['ROS_PACKAGE_PATH'],
 started_ns=started,ended_ns=time.time_ns(),physical_actions=False,live_deployed=False)
(OUT/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(passed=result['passed'],unique_tests=result['unique_tests'],failed=[f['name'] for f in failures],source_unchanged=unchanged,baseline135_unchanged=live_unchanged,report=str(OUT/'result.json'))))
raise SystemExit(0 if result['passed'] else 1)
