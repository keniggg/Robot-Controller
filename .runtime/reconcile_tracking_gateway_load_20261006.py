from pathlib import Path
from datetime import datetime,timezone
import importlib.util,json,hashlib,os,xmlrpc.client
w=Path(__file__).resolve().parents[1];e=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');assert e.parent.parent.is_mount()
oldout=Path(json.loads((w/'.runtime/current_contact_tracking_deployment_20261006.json').read_text())['directory']);old=json.loads((oldout/'result.json').read_text());assert old['error']=="timeout('timed out')" and not old['context_faults'] and 'shutdown_reply' not in old
spec=importlib.util.spec_from_file_location('loader',w/'.runtime/load_contact_margin_20261006.py');loader=importlib.util.module_from_spec(spec);spec.loader.exec_module(loader)
manifest=loader.verified_manifest(Path(old['validation_manifest']));assert loader.sha(Path(old['validation_manifest']))==old['validation_manifest_sha256'];receipt=loader.read_receipt(Path(old['alignment_receipt']));guard=loader.load_guard()
out=e/'ros_log'/('contact_tracking_gateway_reconcile_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('reconcile_loaded_contact_gateway',anonymous=True,disable_signals=True)
master=loader.proxy('http://localhost:11311');caller=rospy.get_name()
def identity(n):
 uri=loader.rpc_value(master.lookupNode(caller,n),'lookup');pid=loader.rpc_value(loader.proxy(uri).getPid(caller),'pid');r=loader.process_identity(pid);r['uri']=uri;return r
report=dict(previous_result=str(oldout/'result.json'),original_timeout_preserved=True,robot_commands_sent=False,task_restarted=False,deployment_passed=False,files=manifest['files'])
try:
 now={n:identity(n) for n in old['before_identities']}
 new=now['/motion_gateway'];prior=old['before_identities']['/motion_gateway'];assert not loader.same_process(new,prior);assert 'motion_gateway_node.py' in new['command'];assert new['ppid']==old['gateway_original_parent']['pid'];assert loader.same_process(loader.process_identity(new['ppid']),old['gateway_original_parent'])
 for n,v in now.items():
  if n!='/motion_gateway':assert loader.same_process(v,old['before_identities'][n]),n
 epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3);msg=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=3);task=rospy.wait_for_message('/grasp/state',GraspState,timeout=3)
 assert not task.active and epoch.stamp.to_nsec()==int(receipt['epoch_ns']) and msg.header.frame_id=='sdk_measured'
 values=dict(zip(msg.name,msg.position));q=[values[n] for n in loader.NAMES];assert max(abs(a-b) for a,b in zip(q,receipt['bound_feedback']['positions']))<=guard.POSE_TOLERANCE_RAD
 ctx=guard.retry_source_context(rospy.get_param('/grasp_mode/selection'),epoch.stamp.to_nsec(),rospy.get_param('/camera'),rospy.get_param('/handeye'),rospy.get_param('/robot_description'));assert guard.digest(ctx)==receipt['source_context_sha256'];rospy.wait_for_service('/supervisor/compensate_grasp_pose',timeout=3)
 report.update(deployment_passed=True,after_identities=now,accepted_positions=q,epoch_ns=epoch.stamp.to_nsec(),selection=ctx['selection'],gateway_new_process_loaded_after_tested_files_saved=True,task_source_unchanged_this_repair=True)
finally:
 report['completed_at_utc']=datetime.now(timezone.utc).isoformat();(out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 (w/'.runtime/current_contact_tracking_gateway_reconcile_20261006.json').write_text(json.dumps(dict(result=str(out/'result.json')),indent=2)+'\n')
print(json.dumps(dict(output=str(out),passed=report['deployment_passed'],gateway_pid=report.get('after_identities',{}).get('/motion_gateway',{}).get('pid')),ensure_ascii=False))
