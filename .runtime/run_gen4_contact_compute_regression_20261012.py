"""Offline regression of the exact integrated contact changes; logs on SanStand."""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
work=Path(__file__).resolve().parents[1]
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=root/'verification'/('gen4_contact_compute_integration'+(sys.argv[1] if len(sys.argv)>1 else ''))
out.mkdir(parents=True,exist_ok=False)
manifest=json.loads((work/'.runtime/gen4_contact_compute_sources_20261012.json').read_text())
paths=manifest['source_sha256']
def sources(): return {name:hashlib.sha256((work/name).read_bytes()).hexdigest() for name in paths}
before=sources(); assert before==paths
prefix='src/alicia_flexible_grasp_supervisor/tests/'
names=['test_gripper_close_contract.py','test_unknown_close_opening_provenance.py','test_contact_geometry_evidence.py','test_contact_height_window_proposals.py','test_contact_height_retry_diagnostic.py','test_contact_height_placement.py','test_contact_height_margin.py','test_contact_height_audit_binding.py','test_contact_height_task.py','test_contact_execution.py','test_registered_plan_object_geometry.py','test_mujoco_digital_twin_client.py','test_grasp_task_sequence.py','test_final_registered_orientation_resolution.py']
scope=[prefix+n for n in names]
scope.append(prefix+'test_remote_grasp6d_streaming.py::test_graspnet_contact_gate_uses_current_fused_surface_and_fails_closed')
for name in ('test_contact_pregrasp_gateway.py','test_contact_pregrasp_task.py'):
    scope.append(prefix+name)
mode_prefix='src/alicia_grasp_modes/tests/'
for name in ('test_observed_contact_base_execution.py','test_observed_contact_current_pose.py',
    'test_observe_return_task.py','test_observe_return_supplement_task.py',
    'test_station_compute_reacquisition.py','test_station_pending_geometry.py'):
    scope.append(mode_prefix+name)
# Newly delivered regression paths are explicit in the reviewed manifest.
for name in manifest.get('new_test_paths', []):
    if name not in scope:scope.append(name)
env=os.environ.copy();env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1')
# The audit-path unit test creates POSIX hard links, unsupported by SanStand.
# Only disposable unit-test fixtures use /tmp; reports and experiment evidence
# remain directly on SanStand, with no fallback experiment recording path.
env['TMPDIR']='/tmp'
env['PYTHONPATH']=':'.join([str(work/'src/alicia_flexible_grasp_supervisor/src'),str(work/'src/alicia_grasp_modes/src'),str(work/prefix),str(work/'src/alicia_grasp_modes/tests'),env.get('PYTHONPATH','')])
cmd=['/usr/bin/python3','-m','pytest','-q',*scope,'-p','no:cacheprovider','--junitxml='+str(out/'pytest.xml')]
t0=time.monotonic()
with (out/'pytest.log').open('x') as f:
 result=subprocess.run(cmd,cwd=work,env=env,stdout=f,stderr=subprocess.STDOUT)
after=sources()
report=dict(command=cmd,test_scope=scope,source_sha256=before,source_after_sha256=after,source_unchanged=before==after,returncode=result.returncode,elapsed_sec=time.monotonic()-t0,production_worktree_tested=True,motion_commands_sent=False)
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(returncode=result.returncode,source_unchanged=before==after,result=str(out/'result.json'))),flush=True)
print('\n'.join((out/'pytest.log').read_text().splitlines()[-12:]),flush=True)
sys.exit(result.returncode if before==after else 2)
