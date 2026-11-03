"""Return a finished, stationary task to GUI manual; grant no motion readiness."""
from pathlib import Path
import json, os, threading, time
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1/supplemental_view_r2/actual_validation')
assert Path('/media/zhuyupei/SanStand').is_mount()
os.environ['ROS_LOG_DIR']=str(R/'manual_restore_ros')
import rospy,rosgraph
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool,String,Header
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('finished_idle_manual_restore',anonymous=True,disable_rosout=True)
master=rosgraph.Master(rospy.get_name());lock=threading.Lock();values={};samples=[]
owners=dict(joint='/alicia_d_driver_node',epoch='/alicia_d_driver_node',manual='/alicia_supervisor_gui',
    task='/grasp_task_node',mode='/grasp_mode_router',arm='/bessica_d_hw_interface',hand='/bessica_d_hw_interface',
    actuation='/alicia_d_driver_node',enabled='/alicia_d_driver_node')
topics=[('joint','/alicia_d/accepted_joint_states',JointState),('epoch','/alicia_d/control_reference_epoch',Header),
    ('manual','/gui/joint_direct_mode',Bool),('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),
    ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
    ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
    ('actuation','/alicia_d/actuation_status',String),('enabled','/alicia_d/motion_enabled',Bool)]
def receive(k,m):
    if m._connection_header.get('callerid')!=owners[k]:return
    with lock:
        values[k]=(m,rospy.Time.now().to_nsec())
        if k=='joint':samples.append(m);samples[:]=samples[-30:]
subs=[rospy.Subscriber(t,c,lambda m,k=k:receive(k,m),queue_size=40) for k,t,c in topics]
def verify():
    for k,t,c in topics:
        if k not in ('joint','arm','hand'):receive(k,rospy.wait_for_message(t,c,timeout=3))
    with lock:
        now=rospy.Time.now().to_nsec();assert len(samples)>=20 and len(values)==len(topics)
        assert all(0<=now-stamp<=1000000000 for m,stamp in values.values())
        assert values['task'][0].active is False
        mode=json.loads(values['mode'][0].data)
        assert mode['active'] is False and mode.get('start_reserved') is False and not mode.get('pending')
        for key in ('arm','hand'):
            status=values[key][0]
            assert 0<=now-status.header.stamp.to_nsec()<1000000000
            assert all(s.status not in (0,1,6,7) for s in status.status_list)
        recent=samples[-20:];names=list(recent[-1].name)
        assert len(names)==len(set(names))==7
        assert all(m.header.frame_id=='sdk_measured' and list(m.name)==names and len(m.position)==7 for m in recent)
        assert 0<=now-recent[-1].header.stamp.to_nsec()<200000000
        spans=[max(m.position[i] for m in recent)-min(m.position[i] for m in recent) for i in range(7)]
        assert max(spans)<=.004
        return dict(ns=now,epoch_ns=values['epoch'][0].stamp.to_nsec(),selection=rospy.get_param('/grasp_mode/selection'),
            positions=list(recent[-1].position),spans=spans,actuation_status=values['actuation'][0].data,
            motion_enabled=values['enabled'][0].data,manual=values['manual'][0].data)
deadline=time.monotonic()+5
while len(samples)<20 and time.monotonic()<deadline:time.sleep(.02)
report=dict(motion_commands_sent=False,joint_disable_commands_sent=False,temperature_queried=False,
    purpose='restore GUI manual only; no actuation confirmation or task permission granted')
pub=None
try:
    before=verify();report['before']=before
    publishers=dict(master.getSystemState()[0]);assert all(publishers.get(t)==[owners[k]] for k,t,c in topics)
    if before['manual'] is not True:
        pub=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
        deadline=time.monotonic()+3
        while not pub.get_num_connections() and time.monotonic()<deadline:time.sleep(.02)
        assert pub.get_num_connections()>0
        requested=rospy.Time.now().to_nsec();rospy.set_param('/gui/joint_direct_mode',True);pub.publish(Bool(True))
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            with lock:ack=values['manual']
            if ack[1]>=requested and ack[0].data is True:break
            time.sleep(.02)
        else:raise RuntimeError('GUI-owned manual acknowledgement missing')
        report['ack']=dict(requested_ns=requested,gui_ack_ns=ack[1])
    after=verify();assert after['manual'] and rospy.get_param('/gui/joint_direct_mode') is True
    assert after['epoch_ns']==before['epoch_ns'] and after['selection']==before['selection']
    assert max(abs(a-b) for a,b in zip(after['positions'],before['positions']))<=.004
    report.update(after=after,passed=True)
finally:
    (R/'manual_restoration.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if pub is not None:pub.unregister()
    for s in subs:s.unregister()
print(json.dumps(report,ensure_ascii=False))
