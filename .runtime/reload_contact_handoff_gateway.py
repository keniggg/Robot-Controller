"""Verify an idle runtime and optionally respawn only its existing gateway."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
RUN = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z')
R = RUN/'high_accuracy_grasp_r1/handoff_repair_r2'
GATEWAY = '/motion_gateway'


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v): p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')


def load(name, relative, digest):
    p=W/relative; assert sha(p)==digest
    spec=importlib.util.spec_from_file_location(name,p)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


class Transport(xmlrpc.client.Transport):
    def make_connection(self,host):
        c=super().make_connection(host);c.timeout=3.;return c


def process(pid):
    p=Path('/proc')/str(pid);s=(p/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,parent_pid=int(s[1]),start_ticks=s[19],cmdline_sha256=sha(p/'cmdline'))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--name',required=True)
    parser.add_argument('--reload',action='store_true')
    args=parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount() and Path(args.name).name==args.name
    out=R/args.name;out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    result=dict(passed=False,source_installed=True,gateway_reloaded=False,
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,
        joint_disable_commands_sent=False,physical_grasp_verified=False)
    idle=load('handoff_idle','.runtime/restore_gen62_after_handoff_20261022.py',
        'f1d5ad558dfd40cf5f7e59820b6ab84b55b40933236014debe41229c155ab61d')
    health=load('handoff_health','.runtime/verify_active_remote_readiness_20261020.py',
        'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5')
    subs=[]
    try:
        tests=json.loads((R/'final_regression/result.json').read_text())
        replay=json.loads((R/'frozen_replay.json').read_text())
        assert tests['passed'] and tests['exact_execution'] and not tests['external_boundary_attempts']
        assert all(row['error'] is None for row in replay['results'])
        sources=tests['source_sha256']
        def source_check(): assert all(sha(W/f)==h for f,h in sources.items())
        source_check()
        result['tests_sha256']=sha(R/'final_regression/result.json')
        result['source_sha256']={str(p):sha(p) for p in map(Path,replay['source_sha256'])}
        import rospy,rosgraph,roslaunch
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header,Bool,String
        from actionlib_msgs.msg import GoalStatusArray
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('contact_handoff_reload',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name())
        def identity(name):
            uri=master.lookupNode(name)
            code,_,pid=xmlrpc.client.ServerProxy(uri,transport=Transport()).getPid(rospy.get_name())
            assert code==1
            return process(pid)
        def graph():
            names={n for group in master.getSystemState() for _,owners in group for n in owners}
            names.discard(rospy.get_name())
            return {n:identity(n) for n in sorted(names)}
        before=graph();old=before[GATEWAY];parent=process(old['parent_pid'])
        parent_dir=Path('/proc')/str(parent['pid'])
        argv=(parent_dir/'cmdline').read_bytes().split(b'\0')
        launch=[x.decode() for x in argv if x.endswith(b'.launch')]
        assert len(launch)==1
        launch_path=(parent_dir/'cwd').resolve()/launch[0]
        config=roslaunch.config.load_config_default([str(launch_path)],11311,verbose=False)
        nodes=[n for n in config.nodes if n.name=='motion_gateway']
        assert len(nodes)==1 and nodes[0].respawn and nodes[0].type=='motion_gateway_node.py'
        result.update(before_nodes=before,parent=parent,launch=str(launch_path),launch_sha256=sha(launch_path),
            original_parent_respawn=True)
        topics=[('joint','/alicia_d/accepted_joint_states',JointState),('epoch','/alicia_d/control_reference_epoch',Header),
            ('manual','/gui/joint_direct_mode',Bool),('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),
            ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
            ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
            ('actuation','/alicia_d/actuation_status',String),('ready','/alicia_d/feedback_ready',Bool),
            ('enabled','/alicia_d/motion_enabled',Bool)]
        values={};samples=[];lock=threading.RLock()
        def receive(key,msg):
            with lock:
                ns=rospy.Time.now().to_nsec();values[key]=(msg,ns)
                if key=='joint':
                    samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(),received_ns=ns,
                        names=list(msg.name),positions=list(msg.position),frame_id=msg.header.frame_id,
                        caller=msg._connection_header.get('callerid')))
                    samples[:]=samples[-40:]
        subs=[rospy.Subscriber(topic,kind,lambda msg,key=key:receive(key,msg),queue_size=40)
              for key,topic,kind in topics]
        deadline=time.monotonic()+8.
        while (len(samples)<20 or len(values)<len(topics)) and time.monotonic()<deadline:time.sleep(.02)
        epoch=values['epoch'][0].stamp.to_nsec()
        def snapshot():
            for key,topic,kind in topics:
                if key not in ('joint','arm','hand'):receive(key,rospy.wait_for_message(topic,kind,timeout=3.))
            with lock:
                data=idle.validate_idle_snapshot(list(samples[-20:]),dict(values),rospy.Time.now().to_nsec(),epoch)
                assert values['manual'][0].data is True and rospy.get_param('/gui/joint_direct_mode') is True
                data['positions']=list(samples[-1]['positions']);data['names']=list(samples[-1]['names'])
            pubs=dict(master.getSystemState()[0])
            assert all(pubs.get(topic)==[idle.OWNERS[key]] for key,topic,_ in topics)
            return data
        def parameters():
            return {k:rospy.get_param(k) for k in ('/grasp_mode/selection','/camera/mode_depth_preset','/grasp','/robot')}
        params=parameters()
        assert params['/camera/mode_depth_preset']['unknown_preset']=='high_accuracy'
        result['idle_before']=snapshot();result['epoch_ns']=epoch
        binding=health.load_binding(RUN/'repair_deployment_r1/remote_startup_binding.json',
            '19e6f75f9c4e29abca1a4872f2ee176433f492b42c9736e81da6395414ee16d6')
        result['remote_readiness']=health.evaluate(binding,out)
        assert result['remote_readiness']['passed'],result['remote_readiness']
        services={s for s,owners in master.getSystemState()[2] if owners==[GATEWAY]}
        assert '/supervisor/move_contact_pregrasp' in services
        source_check();snapshot();assert graph()==before and parameters()==params
        save(out/'before_reload.json',result)
        if args.reload:
            # No new launcher or controller. The existing parent replaces one idle Python node.
            result['shutdown_requested_ns']=time.time_ns();save(out/'result.json',result)
            endpoint=xmlrpc.client.ServerProxy(master.lookupNode(GATEWAY),transport=Transport())
            assert identity(GATEWAY)==old
            try:
                code,message,_=endpoint.shutdown(rospy.get_name(),'load tested manual/contact handoff while idle')
                assert code==1,message
            except (OSError,xmlrpc.client.Error) as exc:result['shutdown_transport']=repr(exc)
            deadline=time.monotonic()+35.;new=None
            while time.monotonic()<deadline:
                assert process(parent['pid'])==parent
                try:
                    row=identity(GATEWAY)
                    if row!=old and row['parent_pid']==old['parent_pid']:new=row;break
                except Exception:pass
                time.sleep(.1)
            assert new is not None,'gateway did not respawn; no automatic retry'
            for service in services:rospy.wait_for_service(service,timeout=5.)
            after=graph()
            preserved={n:r for n,r in before.items() if n!=GATEWAY and r!=old}
            assert all(after.get(n)==r for n,r in preserved.items())
            assert set(after)<=set(preserved)|{GATEWAY}
            result.update(after_nodes=after,gateway_reloaded=True,all_other_node_instances_preserved=True)
        else:result['after_nodes']=graph()
        result['idle_after']=snapshot()
        assert max(abs(a-b) for a,b in zip(result['idle_before']['positions'],result['idle_after']['positions']))<=.004
        assert parameters()==params
        source_check()
        result.update(passed=True,high_accuracy_retained=True,saved_baseline_inactive=True,finished_ns=time.time_ns())
    except Exception as exc:
        result.update(error=repr(exc),finished_ns=time.time_ns());raise
    finally:
        save(out/'result.json',result)
        for sub in subs:sub.unregister()
        print(json.dumps({k:v for k,v in result.items() if k in ('passed','error','gateway_reloaded','high_accuracy_retained')},ensure_ascii=False))


if __name__=='__main__':main()
