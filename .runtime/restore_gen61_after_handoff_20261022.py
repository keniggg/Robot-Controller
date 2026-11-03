"""Return this exact completed GEN61 attempt to GUI manual mode; no motion."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import traceback

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
G = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
S = G / 'actual_attempt_r1'
R = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T092802.096230Z_567d85bbe8f54a56af71145548b078f8')
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    assert not sys.flags.optimize
    assert Path('/media/zhuyupei/SanStand').is_mount()
    out = S / 'manual_restore_r2'
    out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    result = dict(kind='GEN61_returned_generation5_manual_restore', passed=False,
                  started_ns=time.time_ns(), helper_sha256=sha(__file__),
                  motion_commands_sent=False, temperature_queried=False,
                  joint_disable_commands_sent=False, manual_request_sent=False)
    subscriptions = []
    publisher = None
    try:
        helper = W / '.runtime/read_gen61_post_attempt_20261022.py'
        assert sha(helper) == '29435535f9b5dffe4b7930556b503462cb67b2be814881d92abae69787a7d9a1'
        spec = importlib.util.spec_from_file_location('checked_gen61_post_idle', helper)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        args = argparse.Namespace(release=mod.RELEASE, release_sha256=mod.RELEASE_SHA,
            deployment=mod.ACCEPTED, deployment_sha256=mod.ACCEPTED_SHA,
            runtime_common_sha256=mod.RUNTIME_SHA, common_sha256=mod.COMMON_SHA,
            helper_sha256=sha(helper), expected_generation=5,
            attempt_result=S/'scene_start_r9/recorded_entry_result.json',
            attempt_result_sha256='261183043edd56a0295816f5ba62c42f2241fb824546460ea11f887a0bc8502f',
            runner_exit=R/'user_request_runner_exit.json',
            runner_exit_sha256='e0446cb32bb22c77ff3618cbf430a16db32fdbb06c75248634ee546612fe2148',
            authorization_sha256='d5fb4756d227d453c57cb5c13f0e2667058afd1ae03b219ce91f5963036ea5ec')
        common, ctx, release, deployment, selected, record = mod.load_bindings(args)
        assert record == R and json.loads((R/'manifest.json').read_text())['recorder_closed'] is True
        import rosgraph
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('gen61_completed_attempt_manual_restore', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        before = mod.accepted_graph(common, ctx, rospy, master)
        policy = mod.readonly_policy(rospy)
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection')) == selected
        lock, values, samples = threading.Lock(), {}, []
        topics = [
            ('joint','/alicia_d/accepted_joint_states',JointState,'/alicia_d_driver_node'),
            ('epoch','/alicia_d/control_reference_epoch',Header,'/alicia_d_driver_node'),
            ('manual','/gui/joint_direct_mode',Bool,'/alicia_supervisor_gui'),
            ('task','/grasp/state',GraspState,'/grasp_task_node'),
            ('mode','/grasp_mode/status',String,'/grasp_mode_router'),
            ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray,'/bessica_d_hw_interface'),
            ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray,'/bessica_d_hw_interface'),
            ('actuation','/alicia_d/actuation_status',String,'/alicia_d_driver_node'),
            ('ready','/alicia_d/feedback_ready',Bool,'/alicia_d_driver_node'),
            ('enabled','/alicia_d/motion_enabled',Bool,'/alicia_d_driver_node')]
        def receive(key, owner, msg):
            if msg._connection_header.get('callerid') != owner:
                return
            with lock:
                values[key] = (msg, rospy.Time.now().to_nsec())
                if key == 'joint':
                    samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=rospy.Time.now().to_nsec(),
                        names=list(msg.name), positions=list(msg.position), frame_id=msg.header.frame_id))
                    samples[:] = samples[-40:]
        for key, topic, kind, owner in topics:
            subscriptions.append(rospy.Subscriber(topic,kind,lambda msg,k=key,o=owner:receive(k,o,msg),queue_size=40))
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            with lock:
                done=len(values)==len(topics) and len(samples)>=20
            if done:break
            time.sleep(.025)
        with lock: initial=list(samples[-20:])
        assert len(initial)==20 and len({x['stamp_ns'] for x in initial})==20
        names=initial[-1]['names']
        assert {'Joint%d'%i for i in range(1,7)}<=set(names) and len(names)==len(set(names))
        assert all(x['frame_id']=='sdk_measured' and x['names']==names and len(x['positions'])==len(names)
                   and all(math.isfinite(q) for q in x['positions'])
                   and 0<=x['received_ns']-x['stamp_ns']<200_000_000 for x in initial)
        assert all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(initial,initial[1:]))
        assert max(max(x['positions'][i] for x in initial)-min(x['positions'][i] for x in initial) for i in range(len(names)))<=.004
        with lock: states=dict(values)
        now=rospy.Time.now().to_nsec()
        assert len(states)==len(topics) and 0<=now-initial[-1]['stamp_ns']<200_000_000
        assert states['epoch'][0].stamp.to_nsec()==deployment['epoch_ns']
        assert not states['task'][0].active
        mode=json.loads(states['mode'][0].data)
        assert not any(mode.get(k) for k in ('active','pending','start_reserved'))
        for key in ('arm','hand'):
            msg=states[key][0]
            assert 0<=now-msg.header.stamp.to_nsec()<=2_000_000_000
            assert not any(g.status in (0,1,6,7) for g in msg.status_list)
        assert states['actuation'][0].data.startswith('CONFIRMED:')
        assert states['ready'][0].data is True and states['enabled'][0].data is True
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection'))==selected
        result.update(selection=selected, epoch_ns=deployment['epoch_ns'], samples_before=initial,
                      prior_manual_mode=rospy.get_param('/gui/joint_direct_mode'),
                      task_active=False, actions_active=False, actuation_status=states['actuation'][0].data)
        mod.verify_sources(release['source_sha256']);common.recheck(ctx)
        publisher=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
        deadline=time.monotonic()+5
        while publisher.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.025)
        assert publisher.get_num_connections()>0
        requested_ns=rospy.Time.now().to_nsec()
        rospy.set_param('/gui/joint_direct_mode',True)
        publisher.publish(Bool(data=True))
        result.update(manual_request_sent=True,manual_request_ns=requested_ns)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            with lock: ack=values.get('manual')
            if ack and ack[0].data is True and ack[1]>=requested_ns:break
            time.sleep(.025)
        assert ack and ack[0].data is True and ack[1]>=requested_ns
        assert ack[0]._connection_header.get('callerid')=='/alicia_supervisor_gui'
        assert rospy.get_param('/gui/joint_direct_mode') is True
        publisher.unregister();publisher=None
        after=mod.accepted_graph(common,ctx,rospy,master)
        assert after['node_identities']==before['node_identities']
        assert mod.readonly_policy(rospy)==policy
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection'))==selected
        result.update(passed=True,gui_own_manual_ack=True,gui_ack_received_ns=ack[1],
                      exact_nodes_before=before['node_identities'],exact_nodes_after=after['node_identities'])
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        if publisher is not None:publisher.unregister()
        for sub in subscriptions:sub.unregister()
        result['finished_ns']=time.time_ns()
        raw=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode()
        with (out/'result.json').open('xb') as f:f.write(raw);f.flush();os.fsync(f.fileno())
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples_before','exact_nodes_before','exact_nodes_after')},ensure_ascii=False))
    return 0 if result['passed'] else 2

if __name__=='__main__':sys.exit(main())
