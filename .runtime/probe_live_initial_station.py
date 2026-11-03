"""One bounded live initial-view perception/planning experiment; no task start."""
import argparse
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--binding',type=Path,required=True)
    p.add_argument('--binding-sha256',required=True)
    a=p.parse_args();out=a.output.resolve()
    mount=Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in out.parents and out.parent.stat().st_dev==mount.stat().st_dev
    out.mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
    import rospy,rosgraph,cv2,numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import JointState,Image
    from std_msgs.msg import String,Bool,Header
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState,Grasp6DPlan
    from alicia_flexible_grasp_supervisor.srv import TriggerZero
    from alicia_grasp_modes.srv import SetGraspMode
    from alicia_grasp_modes.selection import parse_selection
    health_path=Path(__file__).parent/'verify_active_remote_readiness_20261020.py'
    assert hashlib.sha256(health_path.read_bytes()).hexdigest()=='eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'
    spec=importlib.util.spec_from_file_location('health',health_path);health=importlib.util.module_from_spec(spec);spec.loader.exec_module(health)
    binding=health.load_binding(a.binding,a.binding_sha256)
    readiness=health.evaluate(binding,out)
    assert readiness['passed'],readiness
    rospy.init_node('live_initial_station_probe',anonymous=True,disable_rosout=True)
    master=rosgraph.Master(rospy.get_name());bridge=CvBridge()
    lock=threading.RLock();state={};joints=[];statuses=[];previews=[];images={};saved=[];remote_events=[]
    def receive(key,msg):
        with lock:
            state[key]=(msg,rospy.Time.now().to_nsec())
            if key=='joint':
                joints.append(dict(stamp_ns=msg.header.stamp.to_nsec(),names=list(msg.name),positions=list(msg.position),
                    frame_id=msg.header.frame_id,owner=msg._connection_header.get('callerid')))
            if key=='status':
                statuses.append(dict(received_ns=time.time_ns(),owner=msg._connection_header.get('callerid'),data=json.loads(msg.data)))
            if key=='preview':
                previews.append(dict(stamp_ns=msg.header.stamp.to_nsec(),valid=msg.valid,
                    plan_id=msg.plan_id,candidate_source=msg.candidate_source,diagnostic=msg.diagnostic))
            if key.startswith('remote_'):
                remote_events.append(dict(received_ns=time.time_ns(),topic=key,
                    owner=msg._connection_header.get('callerid'),data=msg.data))
    topics=[('joint','/alicia_d/accepted_joint_states',JointState),('manual','/gui/joint_direct_mode',Bool),
        ('task','/grasp/state',GraspState),('router','/grasp_mode/status',String),
        ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('status','/perception/unknown/detector_status',String),('preview','/grasp_6d/preview_plan_enriched',Grasp6DPlan),
        ('remote_status','/grasp_6d/status',String),('remote_audit','/grasp_6d/gate_audit',String)]
    subs=[rospy.Subscriber(t,c,lambda msg,k=k:receive(k,msg),queue_size=64) for k,t,c in topics]
    def picture(key,msg):
        with lock:
            images.setdefault(key,{})[msg.header.stamp.to_nsec()]=msg
            if len(images[key])>45:del images[key][min(images[key])]
    for key,topic in [('rgb','/supervisor/camera/color/image_raw'),('depth','/supervisor/camera/depth/image_raw'),('mask','/perception/unknown/object_mask')]:
        subs.append(rospy.Subscriber(topic,Image,lambda msg,k=key:picture(k,msg),queue_size=8))
    def idle():
        task=rospy.wait_for_message('/grasp/state',GraspState,timeout=3)
        router=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=3).data)
        assert not task.active and not any(router.get(k) for k in ('active','pending','start_reserved'))
        for key in ('arm','hand'):
            assert key in state and not any(x.status in (0,1,6,7) for x in state[key][0].status_list)
        assert len(joints)>=20 and all(x['frame_id']=='sdk_measured' and x['owner']=='/alicia_d_driver_node' for x in joints[-20:])
        assert rospy.Time.now().to_nsec()-joints[-1]['stamp_ns']<200_000_000
        assert all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(joints[-20:],joints[-19:]))
        positions=np.array([x['positions'] for x in joints[-20:]])
        assert float(np.ptp(positions,axis=0).max())<=.004
    def set_manual(value):
        publisher=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
        try:
            end=time.monotonic()+3
            while not publisher.get_num_connections() and time.monotonic()<end:time.sleep(.02)
            assert publisher.get_num_connections()>0
            requested=rospy.Time.now().to_nsec()
            rospy.set_param('/gui/joint_direct_mode',value);publisher.publish(Bool(data=value))
            end=time.monotonic()+5
            while time.monotonic()<end:
                row=state.get('manual')
                if row and row[1]>=requested and row[0].data is value and row[0]._connection_header.get('callerid')=='/alicia_supervisor_gui':
                    assert rospy.get_param('/gui/joint_direct_mode') is value
                    return dict(value=value,requested_ns=requested,gui_ack_ns=row[1])
                time.sleep(.02)
            raise RuntimeError('GUI mode acknowledgement missing')
        finally:publisher.unregister()
    report=dict(kind='live_initial_station_auto_scope_no_motion',passed=False,started_ns=time.time_ns(),
        remote_readiness=readiness,grasp_start_called=False,motion_commands_sent=False,physical_grasp_verified=False)
    streaming=False;mode_requested=False
    control=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)
    try:
        deadline=time.monotonic()+5
        while len(joints)<20 and time.monotonic()<deadline:time.sleep(.02)
        idle()
        services=dict(master.getSystemState()[2])
        assert services.get('/grasp_6d/request_plan')==['/remote_grasp6d_node']
        assert services.get('/grasp_mode/select')==['/grasp_mode_router']
        assert rospy.get_param('/gui/joint_direct_mode') is True
        assert rospy.wait_for_message('/alicia_d/actuation_status',String,timeout=3).data.startswith('CONFIRMED:')
        assert rospy.wait_for_message('/alicia_d/motion_enabled',Bool,timeout=3).data is True
        assert rospy.wait_for_message('/alicia_d/feedback_ready',Bool,timeout=3).data is True
        before=parse_selection(rospy.get_param('/grasp_mode/selection'))
        mode_requested=True;report['auto_ack']=set_manual(False)
        reply=rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','observe_return')
        assert reply.success,reply.message
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            current=parse_selection(rospy.get_param('/grasp_mode/selection'))
            router=json.loads(state['router'][0].data)
            if (current['generation']==before['generation']+1 and current['strategy']=='observe_return'
                    and parse_selection(router)==current and router['state']!='switching'
                    and not any(router.get(k) for k in ('pending','active','start_reserved'))):break
            time.sleep(.02)
        else:raise RuntimeError('selection commit did not arrive')
        report['selection']=current
        streaming=True
        response=control(True)
        assert response.success,response.message
        report['planning_start']=response.message
        started=time.monotonic();last=0
        while time.monotonic()-started<20:
            elapsed=time.monotonic()-started
            assert not state['task'][0].active
            assert not any(g.status in (0,1,6,7) for key in ('arm','hand') for g in state[key][0].status_list)
            if elapsed-last>=2:
                last=elapsed
                with lock:
                    common=set(images.get('rgb',{}))&set(images.get('depth',{}))&set(images.get('mask',{}))
                    trio=None if not common else [images[k][max(common)] for k in ('rgb','depth','mask')]
                if trio:
                    ns=trio[0].header.stamp.to_nsec();prefix=out/str(ns)
                    rgb=bridge.imgmsg_to_cv2(trio[0],'bgr8');depth=bridge.imgmsg_to_cv2(trio[1],'passthrough');mask=bridge.imgmsg_to_cv2(trio[2],'mono8')
                    cv2.imwrite(str(prefix)+'_rgb.png',rgb);cv2.imwrite(str(prefix)+'_mask.png',mask);np.save(str(prefix)+'_depth.npy',depth)
                    saved.append(dict(source_ns=ns,mask_pixels=int(np.count_nonzero(mask))))
            if any(x['stamp_ns']>=current['stamp_ns'] and
                    (x['valid'] or 'observation_prefix' in x['candidate_source']) for x in previews):
                report['early_stop']='initial preview or observation proposal reached';break
            time.sleep(.02)
        report.update(passed=True,observation_sec=time.monotonic()-started)
    except Exception as exc:
        report['error']=repr(exc)
        raise
    finally:
        try:
            if streaming:
                stopped=control(False);report['planning_stop']=dict(success=stopped.success,message=stopped.message)
                assert stopped.success
            if mode_requested:
                idle();report['manual_ack']=set_manual(True)
            idle()
            report['manual_after']=rospy.get_param('/gui/joint_direct_mode')
            report['maximum_joint_span_rad']=float(np.ptp(np.array([x['positions'] for x in joints]),axis=0).max())
            assert report['maximum_joint_span_rad']<=.004
        except Exception as exc:
            report.update(passed=False,cleanup_error=repr(exc))
            raise
        finally:
            report.update(status=statuses,previews=previews,joints=joints,samples=saved,
                remote_events=remote_events,finished_ns=time.time_ns())
            (out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            for sub in subs:sub.unregister()
            rospy.signal_shutdown('live initial station probe complete')
    print(json.dumps({k:report.get(k) for k in ('passed','error','selection','observation_sec','early_stop','maximum_joint_span_rad','manual_after')},ensure_ascii=False))


if __name__=='__main__':main()
