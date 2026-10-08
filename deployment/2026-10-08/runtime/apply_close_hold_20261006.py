"""Apply the user's existing close/hold switch between attempts; no motion."""
from pathlib import Path
from datetime import datetime, timezone
import json,os,subprocess,sys,hashlib
w=Path(__file__).resolve().parents[1]; mount=Path('/media/zhuyupei/SanStand')
assert mount.is_mount(), 'SanStand unavailable; no fallback'
out=mount/'实验记录/Robot-Controller/ros_log'/('close_hold_policy_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros');os.environ['PYTHONDONTWRITEBYTECODE']='1'
env=dict(os.environ,PYTHONPATH=str(w/'src/alicia_flexible_grasp_supervisor/tests')+os.pathsep+os.environ.get('PYTHONPATH',''))
tests=['src/alicia_flexible_grasp_supervisor/tests/test_grasp_task_sequence.py','src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py','src/alicia_flexible_grasp_supervisor/tests/test_contact_sequence_precheck.py']
commands=[[sys.executable,'-m','pytest','-q','--junitxml='+str(out/'policy_tests.xml'),*tests,'-k','unknown_close_and_hold_never or physical_lift_policy_is or hold_evidence_rejects or hold_requires_sustained or hold_start_response or full_grasp_uses_6d_plan_sequence or measured_contact_failure_prevents or close_hold_strict or close_hold_resolver or readonly_service_never or prediction_reuse_requires_exact'],[sys.executable,'-m','pytest','-q','--junitxml='+str(out/'startup_tests.xml'),'.runtime/full_startup_candidate/test_full_startup_candidate.py']]
report=dict(request='我换一个目标物继续执行抓取任务，就先不要上升阶段',commands_sent=False,tests=[],physical_lift_after_close=False,source_algorithm_changed=False,recorded_at_utc=datetime.now(timezone.utc).isoformat())
try:
 for i,argv in enumerate(commands):
  with (out/('tests_%d.log'%i)).open('w') as log:r=subprocess.run(argv,cwd=w,env=env,stdout=log,stderr=subprocess.STDOUT)
  report['tests'].append(dict(returncode=r.returncode,log=str(log.name)));print(Path(log.name).read_text()[-1400:],flush=True)
  assert r.returncode==0,'policy verification failed'
 import rospy
 from alicia_flexible_grasp_supervisor.msg import GraspState
 rospy.init_node('apply_user_close_hold_policy',anonymous=True,disable_signals=True)
 state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
 assert not state.active,'active grasp must finish before policy change'
 assert rospy.get_param('/mujoco_digital_twin/unknown_lift_gate_enabled') is False
 name='/grasp/unknown_lift_after_close_enabled';report['previous_physical_lift']=rospy.get_param(name)
 rospy.set_param(name,False);report['readback']=rospy.get_param(name);assert report['readback'] is False
 report['applied']=True;report['previous_attempt_operator_confirmation']=dict(plan_id='bf3928359f7e8f57da875780',message_id='call_MvwXl49S1hF9hbVyriTdvqEn:0',text='目标物已夹在两指之间',lift_executed=False)
 report['persistent_files']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [w.parents[1]/'.ros_log/ros_live_20260922_session/resume_modes.launch',w/'.runtime/full_startup_candidate/resume_modes.staged.launch']}
finally:
 (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 (w/'.runtime/current_close_hold_policy_20261006.json').write_text(json.dumps(dict(directory=str(out),result=str(out/'result.json')),indent=2)+'\n')
 print(json.dumps(dict(directory=str(out),applied=report.get('applied',False)),ensure_ascii=False),flush=True)
