"""Load idle planning software only; no hardware/control/action service calls."""
import json, time, hashlib, subprocess, xmlrpc.client
from pathlib import Path
import rospy, rosgraph
from std_msgs.msg import Header, Bool
from control_msgs.msg import FollowJointTrajectoryActionGoal
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import CheckContactSequence

root=Path.cwd()
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20260930T014325Z')
if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand missing')
rospy.init_node('load_contact_candidate_precheck',anonymous=True,disable_signals=True)
master=rosgraph.Master(rospy.get_name())
def pid(name):
    try:
        return xmlrpc.client.ServerProxy(master.lookupNode(name)).getPid(rospy.get_name())[2]
    except (OSError,rosgraph.MasterError):return None
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=5)
if state.active:raise RuntimeError('grasp task is active')
selection=rospy.get_param('/grasp_mode/selection')
if selection['mode']!='unknown' or selection['strategy']!='observe_return':raise RuntimeError('selection changed')
started=rospy.Time.now().to_nsec();action_goals=[]
subs=[rospy.Subscriber(topic,FollowJointTrajectoryActionGoal,
    lambda m,t=topic:action_goals.append(dict(topic=t,stamp_ns=m.header.stamp.to_nsec()))
    if m.header.stamp.to_nsec()>=started else None,queue_size=20)
    for topic in ('/alicia_controller/follow_joint_trajectory/goal','/hand_controller/follow_joint_trajectory/goal')]
report=dict(started_ns=started,scope='idle_software_reload_only',motion_command_sent=False,
            disable_command_sent=False,selection=selection,driver_pid_before=pid('/alicia_d_driver_node'),
            epoch_before_ns=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec(),nodes={})
rospy.set_param('/grasp/unknown_contact_sequence_precheck_enabled',True)
old=pid('/motion_gateway')
try:
    master.lookupService('/supervisor/check_contact_sequence')
    already_loaded=True
except rosgraph.MasterError:
    already_loaded=False
if not already_loaded:
    try:
        xmlrpc.client.ServerProxy(master.lookupNode('/motion_gateway')).shutdown(rospy.get_name(),'load regression-tested read-only candidate precheck')
    except OSError:
        pass  # ROS may close the XML-RPC transport while honoring shutdown.
    for _ in range(150):
        new=pid('/motion_gateway')
        if new is not None and new!=old:break
        time.sleep(.1)
    else:raise RuntimeError('motion gateway did not respawn')
else:
    new=old
rospy.wait_for_service('/supervisor/check_contact_sequence',timeout=8)
report['nodes']['/motion_gateway']=dict(old_pid=old,new_pid=new,owner='existing_root_respawn',already_loaded_from_first_attempt=already_loaded)
if rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active:raise RuntimeError('task ownership changed')
old=pid('/remote_grasp6d_node')
try:
    xmlrpc.client.ServerProxy(master.lookupNode('/remote_grasp6d_node')).shutdown(rospy.get_name(),'load regression-tested candidate policy')
except OSError:
    pass  # Confirm actual process exit below before creating its replacement.
for _ in range(120):
    if pid('/remote_grasp6d_node') is None:break
    time.sleep(.1)
else:raise RuntimeError('old candidate node still registered')
unit='alicia-contact-precheck-remote-20260930.service'
subprocess.run(['systemd-run','--user','--unit='+unit,'--property=Type=simple',
    '--property=PartOf=alicia-ros-live-20260928.service',
    '--property=After=alicia-ros-live-20260928.service','--property=Restart=no',
    '/bin/bash',str(root/'.runtime/start_contact_precheck_remote_20260930.sh')],check=True)
for _ in range(160):
    new=pid('/remote_grasp6d_node')
    if new is not None and new!=old:break
    time.sleep(.1)
else:raise RuntimeError('new candidate node unavailable')
report['nodes']['/remote_grasp6d_node']=dict(old_pid=old,new_pid=new,owner=unit)
report.update(driver_pid_after=pid('/alicia_d_driver_node'),
    epoch_after_ns=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4).stamp.to_nsec(),
    motion_enabled=rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=4).data,
    task_active=rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active,
    selection_after=rospy.get_param('/grasp_mode/selection'), action_goals_observed=action_goals)
assert report['driver_pid_before']==report['driver_pid_after']
assert report['epoch_before_ns']==report['epoch_after_ns']
assert report['selection_after']==selection and not report['task_active'] and not action_goals
files=['src/alicia_grasp_modes/src/alicia_grasp_modes/runner_contract.py',
 'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_linear_compensation.py',
 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py',
 'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml',
 'src/alicia_flexible_grasp_supervisor/srv/CheckContactSequence.srv']
report['source_sha256']={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
report.update(completed_ns=time.time_ns(),new_physical_grasp_verified=False)
(out/'contact_precheck_deployment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False),flush=True)
