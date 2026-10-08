import json,pathlib,hashlib,time,subprocess,os,xmlrpc.client
import rospy,rosgraph
from alicia_flexible_grasp_supervisor.msg import GraspState
root=pathlib.Path.cwd();out=pathlib.Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory'])
rospy.init_node('load_validated_contact_planner',anonymous=True)
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
if state.active:raise RuntimeError('Cannot reload active task planner')
master=rosgraph.Master(rospy.get_name());uri=master.lookupNode('/remote_grasp6d_node');proxy=xmlrpc.client.ServerProxy(uri)
old_pid=proxy.getPid(rospy.get_name())[2]
launch=pathlib.Path('/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/resume_modes.launch')
original=launch.read_text();(out/'resume_modes_before_full_lift.launch').write_text(original)
updated=original.replace('name="/mujoco_digital_twin/unknown_lift_gate_enabled" value="false"','name="/mujoco_digital_twin/unknown_lift_gate_enabled" value="true"').replace('name="/grasp/unknown_lift_after_close_enabled" value="false"','name="/grasp/unknown_lift_after_close_enabled" value="true"')
launch.write_text(updated)
previous={k:rospy.get_param(k) for k in ('/mujoco_digital_twin/unknown_lift_gate_enabled','/grasp/unknown_lift_after_close_enabled')}
for k in previous:rospy.set_param(k,True)
files=['src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py','src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/contact_height_placement.py']
record=dict(started_unix=time.time(),previous_planner_pid=old_pid,only_planner_reloaded=True,arm_driver_restarted=False,torque_command_sent=False,target_relocated=False,previous_lift_params=previous,current_lift_params={k:True for k in previous},source_sha256={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files},regression=dict(total_unique_tests_passed=535,combined_run_passed=532,old_wire_fixture_fixed_retest_passed=3,fixture_retest_result='3 passed in 3.15 seconds'),frozen_production_replay=dict(total_candidates=32,height_robust_candidates=22,unchanged_translation_limit_m=.0026767676132463574))
print('RELOADING_PLANNER',old_pid,flush=True);print(proxy.shutdown(rospy.get_name(),'load regression-verified support and height planning'),flush=True)
for _ in range(100):
 try:
  if master.lookupNode('/remote_grasp6d_node') != uri:break
 except Exception:break
 time.sleep(.05)
else:raise RuntimeError('Planner did not unregister')
log=(out/'remote_support_height_runtime.log').open('ab',buffering=0)
proc=subprocess.Popen(['/usr/bin/python3','-u','/home/zhuyupei/alicia_wa_full/.ros_log/unknown_grasp_20260923/remote_with_simulation_audit.py','__name:=remote_grasp6d_node'],stdout=log,stderr=subprocess.STDOUT,cwd=str(root),env=os.environ.copy(),start_new_session=True)
record['new_planner_pid']=proc.pid;record['runtime_log']=str(out/'remote_support_height_runtime.log')
(out/'support_height_deployment.json').write_text(json.dumps(record,indent=2)+'\n');print('NEW_PLANNER_PID',proc.pid,flush=True)
