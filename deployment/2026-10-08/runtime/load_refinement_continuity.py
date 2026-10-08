"""Load two idle Python consumers of the new phase message; no hardware commands."""
import hashlib,json,os,socket,subprocess,time,xmlrpc.client
from pathlib import Path
from datetime import datetime,timezone
mount=Path('/media/zhuyupei/SanStand');assert mount.is_mount(), 'SanStand required'
validation=Path(json.loads(Path('.runtime/latest_refinement_continuity_validation.json').read_text())['directory'])/'validation.json'
proof=json.loads(validation.read_text())
assert proof['regression']['passed']==1094
for name,digest in proof['source_sha256'].items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest, name+' changed after validation'
out=mount/'实验记录/Robot-Controller/ros_log'/('refinement_continuity_load_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
socket.setdefaulttimeout(4.)
import rospy,rosgraph
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState,NearFieldPlanningPhase
from alicia_flexible_grasp_supervisor.srv import TriggerZero
assert NearFieldPlanningPhase._md5sum==proof['phase_message_md5']
rospy.init_node('load_idle_refinement_continuity',anonymous=True,disable_signals=True)
master=rosgraph.Master('/load_idle_refinement_continuity')
def rpc(name):return xmlrpc.client.ServerProxy(master.lookupNode(name))
def identity(pid):
    proc=Path('/proc')/str(pid);stat=(proc/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,start_ticks=int(stat[19]),argv=(proc/'cmdline').read_bytes().decode().split('\0')[:-1])
def node(name):return identity(rpc(name).getPid('/load_idle_refinement_continuity')[2])
protected=['/alicia_d_driver_node','/bessica_d_hw_interface','/motion_gateway','/accepted_robot_state_publisher','/supervisor_camera','/unknown_tabletop_perception','/move_group']
consumers=['/grasp_task_node','/remote_grasp6d_node']
units=['alicia-task-postmove-20260929.service','alicia-remote-ticket-repair-20260929T0620.service']
result=dict(status='preparing',validation=str(validation),source_sha256=proof['source_sha256'],started_ns=time.time_ns(),hardware_commands_sent=0,joint_disable_commands_sent=False,automatic_grasp_started=False,user_instruction='继续处理，我已经对准了目标物；所有设计面向可抓取的未知物体，不能设置物体专属动作。')
try:
    state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4.)
    if state.active:raise RuntimeError('task active, cannot reload software')
    result['before']={name:node(name) for name in protected}
    result['root_before']=identity(8526)
    result['consumers_before']={name:node(name) for name in consumers}
    result['epoch_before_ns']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4.).stamp.to_nsec()
    result['manual_before']=rospy.get_param('/gui/joint_direct_mode',False)
    result['selection_before']=rospy.get_param('/grasp_mode/selection')
    (out/'before.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    retired=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)(False)
    if not retired.success or retired.message not in ('continuous remote 6D inference stopped','continuous remote 6D inference already stopped'):
        raise RuntimeError('inference retirement not acknowledged: '+retired.message)
    result['inference_retirement']=retired.message
    subprocess.run(['systemctl','--user','restart',*units],check=True,timeout=25.)
    deadline=time.monotonic()+20.
    while time.monotonic()<deadline:
        try:
            after={name:node(name) for name in consumers}
            if all(after[n]!=result['consumers_before'][n] for n in consumers):break
        except Exception:pass
        time.sleep(.1)
    else:raise RuntimeError('updated consumers did not return')
    result['consumers_after']=after
    message=rospy.wait_for_message('/grasp/near_field_phase',NearFieldPlanningPhase,timeout=6.)
    if message.active:raise RuntimeError('phase unexpectedly active after reload')
    result['received_phase_message_md5']=message._md5sum
    deadline=time.monotonic()+5.
    while time.monotonic()<deadline:
        bus=rpc('/remote_grasp6d_node').getBusInfo('/load_idle_refinement_continuity')[2]
        connections=[row for row in bus if row[4]=='/grasp/near_field_phase' and row[2]=='i' and row[5]]
        if connections:break
        time.sleep(.1)
    else:raise RuntimeError('remote has no connected new phase subscriber')
    result['phase_connections']=connections
    result['after']={name:node(name) for name in protected}
    result['root_after']=identity(8526)
    result['epoch_after_ns']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4.).stamp.to_nsec()
    result['manual_after']=rospy.get_param('/gui/joint_direct_mode',False)
    result['selection_after']=rospy.get_param('/grasp_mode/selection')
    for a,b in [('before','after'),('root_before','root_after'),('epoch_before_ns','epoch_after_ns'),('manual_before','manual_after'),('selection_before','selection_after')]:
        if result[a]!=result[b]:raise RuntimeError(a+' changed during software load')
    state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4.)
    if state.active:raise RuntimeError('task unexpectedly active after reload')
    result.update(status='loaded_idle_for_fresh_selection',new_task_state=state.state,completed_ns=time.time_ns())
except Exception as exc:
    result.update(status='load_failed_no_automatic_motion',error=str(exc),completed_ns=time.time_ns())
    raise
finally:
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    Path('.runtime/latest_refinement_continuity_load.json').write_text(json.dumps(dict(directory=str(out)),ensure_ascii=False)+'\n')
    print(json.dumps(dict(directory=str(out),status=result['status'],error=result.get('error'),consumers_after={k:v['pid'] for k,v in result.get('consumers_after',{}).items()},epoch_unchanged=result.get('epoch_after_ns')==result.get('epoch_before_ns'),hardware_commands_sent=0),ensure_ascii=False),flush=True)
