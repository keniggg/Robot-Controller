#!/usr/bin/env python3
"""Independent finite raw-feedback recorder. No command publisher or service call."""
import hashlib,json,os,signal,threading,time
from datetime import datetime,timezone
from pathlib import Path


def main():
    mount=Path('/media/zhuyupei/SanStand')
    if not os.path.ismount(mount):raise RuntimeError('SanStand unavailable; no fallback')
    out=mount/'实验记录/Robot-Controller/ros_log'/('j3_small_passive_monitor_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    import rospy,rosbag
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Header,String,Bool
    rospy.init_node('joint_response_passive_monitor',anonymous=True,disable_signals=True)
    lock=threading.RLock();done=threading.Event();counts={};errors=[];subs=[]
    start=time.monotonic();bag=rosbag.Bag(str(out/'evidence.bag'),'w')
    terminal={'commands_published':0,'duration_limit_seconds':90,'status':'recording','prior_fault_preserved':True}
    source=Path(__file__).read_bytes();(out/Path(__file__).name).write_bytes(source)
    terminal['source_sha256']=hashlib.sha256(source).hexdigest()
    def receive(topic,msg):
        with lock:
            if done.is_set():return
            try:
                bag.write(topic,msg,rospy.Time.now());counts[topic]=counts.get(topic,0)+1
            except BaseException as error:
                errors.append(str(error));done.set()
    def shutdown(*_):done.set()
    signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
    topics=[('/alicia_d/accepted_joint_states',JointState),('/alicia_d/sdk_command',JointState),('/alicia_d/control_reference',JointState),('/alicia_d/sdk_diagnostics',DiagnosticArray),('/alicia_d/control_reference_epoch',Header),('/alicia_d/actuation_status',String),('/alicia_d/feedback_ready',Bool),('/alicia_d/protection_latched',Bool),('/joint_commands',JointState)]
    try:
        for topic,kind in topics:
            subs.append(rospy.Subscriber(topic,kind,lambda msg,t=topic:receive(t,msg),queue_size=300))
        print('READY_PASSIVE_MONITOR '+json.dumps({'directory':str(out),'pid':os.getpid(),'duration_seconds':90}),flush=True)
        while time.monotonic()-start<90 and not done.wait(.05) and not rospy.is_shutdown():pass
    except BaseException as error:
        with lock:errors.append('recorder: '+str(error))
        raise
    finally:
        done.set()
        for sub in subs:sub.unregister()
        with lock:
            try:bag.close()
            except BaseException as error:errors.append('bag close: '+str(error))
            terminal.update(status='failed' if errors else 'readonly_complete',counts=dict(counts),errors=list(errors),elapsed_seconds=time.monotonic()-start)
        (out/'terminal.json').write_text(json.dumps(terminal,indent=2)+'\n')
        print('PASSIVE_MONITOR_DONE '+json.dumps({'directory':str(out),**terminal}),flush=True)

if __name__=='__main__':main()
