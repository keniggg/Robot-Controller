"""Relevant callback regression suite against one frozen isolated delivery."""
import hashlib,json,os,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
WORK=Path(__file__).resolve().parents[1]
DELIVERY=Path(sys.argv[1]) if len(sys.argv)>1 else WORK/'.runtime/gen10_shared_reference_staging_20261010/delivery.json'
delivery=json.loads(DELIVERY.read_text())
assert delivery.get('delivery_ready') and delivery.get('all_delivery_files_frozen'), 'delivery is not frozen and ready'
delivery['files']=delivery.get('files',delivery.get('changes',[]))
if 'test_files' not in delivery:delivery['test_files']=[delivery['tests']['command_scope']]
STAGE=Path(delivery['stage']).resolve()
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT=Path(sys.argv[2]) if len(sys.argv)>2 else NEW/'verification/gen10_shared_reference_combined'
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=False)
(OUT/'tmp').mkdir()
os.environ['TMPDIR']=str(OUT/'tmp')
os.environ.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_LOG_DIR=str(OUT/'ros'))
sys.dont_write_bytecode=True
os.environ['ROS_PACKAGE_PATH']=str(STAGE/'src')+':'+str(WORK/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
paths=[STAGE/'src/alicia_grasp_modes/src',STAGE/'src/alicia_flexible_grasp_supervisor/src',
       STAGE/'src/alicia_grasp_modes/tests',STAGE/'src/alicia_flexible_grasp_supervisor/tests']
os.environ['PYTHONPATH']=':'.join(map(str,paths))+':'+os.environ.get('PYTHONPATH','')
sys.path[:0]=list(map(str,paths))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
base=json.loads((NEW/'silhouette_coverage_deployment/result.json').read_text())['source_sha256']
def live_hashes():return {p:sha(WORK/p) for p in base}
assert len(base)==150
assert live_hashes()==base,'live baseline150 changed before test'
changed=[row['path'] for row in delivery['files']]
assert len(changed)==len(set(changed))
for row in delivery['files']:assert sha(STAGE/row['path'])==row['sha256'],row['path']
base_present={p:d for p,d in base.items() if (STAGE/p).is_file() and p not in changed}
base_errors={p:sha(STAGE/p) for p,d in base_present.items() if sha(STAGE/p)!=d}
assert not base_errors,base_errors
changes=[dict(path=p,original_live_sha256=sha(WORK/p) if (WORK/p).exists() else None,staged_sha256=sha(STAGE/p)) for p in changed]
(OUT/'deployment_changes.json').write_text(json.dumps(dict(stage=str(STAGE),files=changes,baseline150=base,live_mutated=False),indent=2,sort_keys=True)+'\n')
all_paths=[p for p in STAGE.rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml','.launch','.xml','.npz')]
def hashes():return {str(p.relative_to(STAGE)):sha(p) for p in all_paths}
before=hashes()
(OUT/'full_staged_source_sha256.json').write_text(json.dumps(before,indent=2,sort_keys=True)+'\n')
baseline_fixture_report=NEW/'verification/gen10_baseline_variant_fixture/comparison.json'
baseline_fixture=json.loads(baseline_fixture_report.read_text())
assert baseline_fixture['all_27_names_and_exception_details_match'] is True
source_audit={}
class Sources:
 def pytest_collection_finish(self,session):
  import rospkg
  for package in ('alicia_flexible_grasp_supervisor','alicia_grasp_modes'):
   path=Path(rospkg.RosPack().get_path(package)).resolve()
   assert path==(STAGE/'src'/package).resolve(),(package,path)
   source_audit['rospkg_'+package]=str(path)
  from alicia_grasp_modes import initial_preflight_task,initial_preflight_recovery,station_compute_wait_remote,silhouette_observation_order,silhouette_observation_planning
  from alicia_flexible_grasp.vision import latest_only_inference
  from alicia_flexible_grasp.grasp import grasp6d_pipeline
  import test_remote_modes
  modules={
   'silhouette_observation_order':(silhouette_observation_order,'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_order.py'),
   'silhouette_observation_planning':(silhouette_observation_planning,'src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py'),
   'station_compute_wait_remote':(station_compute_wait_remote,'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'),
   'mode_remote':(test_remote_modes.module,'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'),
   'original_remote':(test_remote_modes.base,'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'),
   'grasp6d_pipeline':(grasp6d_pipeline,'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_pipeline.py'),
   'latest_only_inference':(latest_only_inference,'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/latest_only_inference.py'),
   'initial_preflight_task':(initial_preflight_task,'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_task.py'),
   'initial_preflight_recovery':(initial_preflight_recovery,'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_recovery.py')}
  for name,(module,relative) in modules.items():
   actual=Path(module.__file__).resolve()
   assert actual==(STAGE/relative).resolve(),(name,actual)
   source_audit[name]=dict(path=str(actual),sha256=sha(actual))
  leaked={n:str(Path(m.__file__).resolve()) for n,m in list(sys.modules.items()) if getattr(m,'__file__',None)
   and any(str(Path(m.__file__).resolve()).startswith(str(WORK/'src'/pkg)+'/') for pkg in ('alicia_grasp_modes','alicia_flexible_grasp_supervisor'))}
  assert not leaked,leaked
  source_audit['live_package_module_leaks']=leaked
  (OUT/'module_source_audit.json').write_text(json.dumps(source_audit,indent=2,sort_keys=True)+'\n')

suite=list(delivery['test_files'])
prefix='src/alicia_grasp_modes/tests/'
related=[prefix+p for p in (
 'test_remote_modes.py','test_silhouette_observation_planning.py',
 'test_silhouette_observation_coverage.py','test_silhouette_observation_remote.py',
 'test_failed_observation_audit.py','test_observation_fk_first_strict.py',
 'test_observe_return_supplement_remote.py','test_observation_information_selection.py',
 'test_observation_information_scheduler.py')]
related += ['src/alicia_flexible_grasp_supervisor/tests/'+p for p in (
 'test_moveit_planner_pose_feedback.py','test_grasp6d_pipeline.py','test_observation_information_comparison.py',
 'test_observation_preview.py','test_observation_rejection_summary.py',
 'test_observation_continuation_loop.py','test_silhouette_observation_plan_contract.py')]
for p in related:
 if p not in suite:suite.append(p)
unchanged_selected_tests={p:sha(STAGE/p) for p in suite if p not in changed and (WORK/p).is_file()}
assert all(sha(WORK/p)==h for p,h in unchanged_selected_tests.items()), 'pre-existing selected test file changed'
args=['-q','-p','no:cacheprovider','--basetemp',str(OUT/'pytest_tmp')]+[str(STAGE/p) for p in suite]+['--junitxml',str(OUT/'tests.xml')]
started=time.time_ns()
import pytest
plugin=Sources()
with (OUT/'pytest.log').open('w') as log:
 oldout,olderr=sys.stdout,sys.stderr
 try:
  sys.stdout=sys.stderr=log
  rc=pytest.main(args,plugins=[plugin])
 finally:sys.stdout,sys.stderr=oldout,olderr
plugin.pytest_collection_finish(None)
cases=list(ET.parse(str(OUT/'tests.xml')).getroot().iter('testcase'))
unique={(c.get('classname'),c.get('name')) for c in cases}
failures=[dict(c.attrib,detail=(c.find('failure').text if c.find('failure') is not None else c.find('error').text)) for c in cases if c.find('failure') is not None or c.find('error') is not None]
unchanged=hashes()==before;live_unchanged=live_hashes()==base
result=dict(passed=rc==0 and unchanged and live_unchanged and bool(source_audit),
 source_unchanged=unchanged,source_sha256={p:before[p] for p in changed},unique_tests=len(unique),stage=str(STAGE),returncode=int(rc),
 baseline150=dict(count=len(base),matched_before=True,matched_after=live_unchanged,unchanged_stage_baseline_present=len(base_present),stage_baseline_mismatches=base_errors),
 deployment_changes=str(OUT/'deployment_changes.json'),deployment_changes_sha256=sha(OUT/'deployment_changes.json'),
 delivery_path=str(DELIVERY),delivery_sha256=sha(DELIVERY),actual_test_count=len(cases),
 baseline_failure_comparison=dict(report=str(baseline_fixture_report),sha256=sha(baseline_fixture_report),conclusion='same_27_failures_on_original_baseline150_and_patch',existing_failed_tests=27,excluded_file_tests=29),
 test_nodeids=[c.get('classname')+'::'+c.get('name') for c in cases],test_selection=suite,unchanged_selected_test_sha256=unchanged_selected_tests,failures=failures,module_source_audit=source_audit,
 command=['/usr/bin/python3','-m','pytest']+args,driver=str(Path(__file__).resolve()),
 pythonpath=os.environ['PYTHONPATH'],ros_package_path=os.environ['ROS_PACKAGE_PATH'],started_ns=started,ended_ns=time.time_ns(),physical_actions=False,live_deployed=False)
(OUT/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(passed=result['passed'],unique_tests=len(unique),failed=[f['name'] for f in failures],source_unchanged=unchanged,baseline150_unchanged=live_unchanged,report=str(OUT/'result.json'))))
raise SystemExit(0 if result['passed'] else 1)
