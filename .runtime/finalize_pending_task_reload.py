"""Finish verification of the already launched inactive task; no motion."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import types

p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);a=p.parse_args()
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def ref(path):return dict(path=str(path),sha256=sha(path))
assert sha(a.manifest)=='0c5286a40c0c722dab2dd6047bb822ff43510bf09a7d45ef7d3639b011880058'
plan=read(a.manifest);w=Path(plan['workspace']);o=a.manifest.parent.parent
out=o/'online_reload_final';assert Path('/media/zhuyupei/SanStand').is_mount();out.mkdir()
os.environ['ROS_LOG_DIR']=str(out/'ros')
def module(path,name,expected):
    assert sha(path)==expected
    m=types.ModuleType(name);m.__file__=str(path)
    exec(compile(path.read_bytes(),str(path),'exec'),m.__dict__);return m
h=module(w/'.runtime/recover_initial_pending_task.py','recovery','eb48d883187978cc3df565d87dda5c015168574d33ef3dcfc1d81a3bf7d9628d')
idle=module(w/'.runtime/restore_gen62_after_handoff_20261022.py','idle',h.PRIMITIVES['.runtime/restore_gen62_after_handoff_20261022.py'])
health=module(w/'.runtime/verify_active_remote_readiness_20261020.py','health',h.PRIMITIVES['.runtime/verify_active_remote_readiness_20261020.py'])
baseline=h.read_ref(plan['runtime']);test=h.read_ref(plan['validation'][0])
assert test['passed'] and all(sha(w/f)==v for f,v in test['source_sha256'].items())
previous=read(o/'online_reload_r3/result.json')
assert not previous['passed'] and previous['error']=="RuntimeError('remote reset service owner differs')"
receipt=read(o/'online_reload_r3/grasp_task_node/source_start.json')
assert receipt['source_checked_before_exec'] and receipt['sources']==previous['installed_sources']
assert all(sha(f)==v for f,v in receipt['sources'].items())
import rosgraph
import rospy
from std_msgs.msg import Bool,Header,String
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_grasp_modes.selection import parse_selection
from std_srvs.srv import Trigger
rospy.init_node('finalize_inactive_task_replacement',anonymous=True,disable_rosout=True)
master=rosgraph.Master(rospy.get_name());lock=threading.Lock();values={};samples=[]
topics=[('joint','/alicia_d/accepted_joint_states',JointState),('epoch','/alicia_d/control_reference_epoch',Header),
    ('manual','/gui/joint_direct_mode',Bool),('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),
    ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
    ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
    ('actuation','/alicia_d/actuation_status',String),('ready','/alicia_d/feedback_ready',Bool),('enabled','/alicia_d/motion_enabled',Bool)]
def receive(key,msg):
    if msg._connection_header.get('callerid')!=idle.OWNERS[key]:return
    with lock:
        now=rospy.Time.now().to_nsec();values[key]=(msg,now)
        if key=='joint':
            samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(),received_ns=now,names=list(msg.name),positions=list(msg.position),frame_id=msg.header.frame_id,caller=idle.OWNERS[key]))
            samples[:]=samples[-40:]
subs=[rospy.Subscriber(topic,kind,lambda msg,k=key:receive(k,msg),queue_size=40) for key,topic,kind in topics]
def snapshot():
    end=time.monotonic()+5
    while (len(values)<len(topics) or len(samples)<20) and time.monotonic()<end:time.sleep(.02)
    for key,topic,kind in topics:
        if key not in ('joint','arm','hand'):receive(key,rospy.wait_for_message(topic,kind,timeout=3))
    with lock:
        evidence=idle.validate_idle_snapshot(list(samples[-20:]),dict(values),rospy.Time.now().to_nsec(),baseline['epoch_ns'])
        position=samples[-1].copy()
    assert values['manual'][0].data is True and rospy.get_param('/gui/joint_direct_mode') is True
    assert parse_selection(rospy.get_param('/grasp_mode/selection'))==parse_selection(baseline['selection'])
    publishers=dict(master.getSystemState()[0]);assert all(publishers.get(topic)==[idle.OWNERS[key]] for key,topic,kind in topics)
    return dict(evidence=evidence,joints=position)
def nodes():
    current={name:h.node(master,name) for name in baseline['nodes']}
    assert all(row==baseline['nodes'][name] for name,row in current.items() if name!='/grasp_task_node')
    assert current['/grasp_task_node']['pid']==receipt['pid']!=baseline['nodes']['/grasp_task_node']['pid']
    return current
report=dict(passed=False,started_ns=time.time_ns(),motion_commands_sent=False,mode_commands_sent=False,driver_restarted=False,temperature_queried=False,prior_interrupted_verification=ref(o/'online_reload_r3/result.json'),task_start=ref(o/'online_reload_r3/grasp_task_node/source_start.json'),validation=plan['validation'])
try:
    report['after_nodes']=nodes();report['before']=snapshot()
    config=master.getParam('/grasp');selection=master.getParam('/grasp_mode/selection')
    assert dict(master.getSystemState()[2]).get('/grasp_mode/reset_task')==['/grasp_task_node']
    reply=rospy.ServiceProxy('/grasp_mode/reset_task',Trigger)()
    assert reply.success and parse_selection(json.loads(reply.message))==parse_selection(baseline['selection'])
    report['reset_ack']=json.loads(reply.message)
    binding=health.load_binding(Path(plan['remote_binding']['path']),plan['remote_binding']['sha256'])
    report['remote_health']=health.evaluate(binding,out);assert report['remote_health']['passed']
    report['after']=snapshot();assert nodes()==report['after_nodes']
    assert master.getParam('/grasp')==config and master.getParam('/grasp_mode/selection')==selection
    before=report['before']['joints'];after=report['after']['joints']
    assert before['names']==after['names'] and max(abs(x-y) for x,y in zip(before['positions'],after['positions']))<=.004
    assert all(sha(w/f)==v for f,v in test['source_sha256'].items())
    report.update(passed=True,task_reloaded=True,source_sha256=test['source_sha256'],remote_binding=plan['remote_binding'])
finally:
    report['finished_ns']=time.time_ns();save(out/'result.json',report)
    for sub in subs:sub.unregister()
    rospy.signal_shutdown('inactive task replacement verified')
print(json.dumps(dict(passed=report['passed'],new_task=report['after_nodes']['/grasp_task_node'],output=str(out))))
