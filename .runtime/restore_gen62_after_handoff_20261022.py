#!/usr/bin/env python3
"""Return one exactly bound, sealed GEN62 attempt to GUI manual mode; no motion."""
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
sys.path.insert(0, str(W/'tools'))
from experiment_storage import select as select_storage
G = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
F = G / 'contact_feedback_fix_r1'
S = F / 'actual_attempt_r1'
POST = W / '.runtime/read_gen62_post_attempt_20261022.py'
POST_SHA = '43e3066c93d104922664570ddd1a10a451e8ce7577d5add8533e351a8163187c'
BASELINE = W / '.runtime/restore_gen61_after_handoff_20261022.py'
BASELINE_SHA = 'feb3888f6914161f3d91b5f4430879b4f10dfd36d8e7450718e88f193eafc38d'
GUI = '/alicia_supervisor_gui'
DRIVER = '/alicia_d_driver_node'
OWNERS = dict(joint=DRIVER, epoch=DRIVER, manual=GUI, task='/grasp_task_node',
    mode='/grasp_mode_router', arm='/bessica_d_hw_interface', hand='/bessica_d_hw_interface',
    actuation=DRIVER, ready=DRIVER, enabled=DRIVER)
sys.dont_write_bytecode = True
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate_idle_snapshot(initial, states, now, epoch):
    """Original SDK/idle gates, also applied after all pre-request waiting."""
    assert set(states) == set(OWNERS) and type(epoch) is int and epoch > 0
    assert all(msg._connection_header.get('callerid') == OWNERS[key]
               and 0 <= now-received < 1_000_000_000
               for key, (msg, received) in states.items())
    assert states['epoch'][0].stamp.to_nsec() == epoch
    assert len(initial) == 20 and len({x['stamp_ns'] for x in initial}) == 20
    names = initial[-1]['names']
    assert {'Joint%d'%i for i in range(1,7)} <= set(names) and len(names) == len(set(names))
    assert all(x['caller'] == DRIVER and x['frame_id'] == 'sdk_measured'
               and x['names'] == names and len(x['positions']) == len(names)
               and all(math.isfinite(q) for q in x['positions'])
               and epoch <= x['stamp_ns'] <= x['received_ns'] < x['stamp_ns']+200_000_000
               for x in initial)
    assert all(b['stamp_ns'] > a['stamp_ns'] for a,b in zip(initial,initial[1:]))
    spans = {name:max(x['positions'][i] for x in initial)-min(x['positions'][i] for x in initial)
             for i,name in enumerate(names)}
    assert max(spans.values()) <= .004 and 0 <= now-initial[-1]['stamp_ns'] < 200_000_000
    assert states['manual'][0]._connection_header.get('latching') == '1'
    assert not states['task'][0].active
    mode = json.loads(states['mode'][0].data)
    assert not any(mode.get(k) for k in ('active','pending','start_reserved'))
    actions = {}
    for key in ('arm','hand'):
        msg = states[key][0]
        assert 0 <= now-msg.header.stamp.to_nsec() <= 2_000_000_000
        assert not any(g.status in (0,1,6,7) for g in msg.status_list)
        actions[key] = dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=states[key][1],
            caller=msg._connection_header.get('callerid'), statuses=[g.status for g in msg.status_list])
    assert states['actuation'][0].data.startswith('CONFIRMED:')
    assert states['ready'][0].data is True and states['enabled'][0].data is True
    return dict(sample_spans_rad=spans, actions=actions, router=mode,
        actuation_status=states['actuation'][0].data,
        state_received_ns={key:row[1] for key,row in states.items()}, sampled_ns=now)


def own_ack(ack, requested_ns):
    return bool(ack and ack[0].data is True and ack[1] >= requested_ns
        and ack[0]._connection_header.get('callerid') == GUI
        and ack[0]._connection_header.get('latching') == '1')


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--record', type=Path, required=True)
    ap.add_argument('--expected-generation', type=int, required=True)
    for key in ('release','deployment','attempt-result','runner-exit','authorization','record-manifest'):
        ap.add_argument('--'+key, type=Path, required=True)
        ap.add_argument('--'+key+'-sha256', required=True)
    for key in ('helper','post-helper','runtime-common','common'):
        ap.add_argument('--'+key+'-sha256', required=True)
    return ap.parse_args()


def main():
    if sys.flags.optimize:
        raise RuntimeError('original assertion gates require unoptimized Python')
    args = parse_args()
    assert args.expected_generation == 6
    out = args.output.absolute()
    selected = select_storage(path=out)
    selected.validate(out)
    selected.validate(args.record.absolute(), subtree=selected.archive_root)
    assert out.parent.exists()
    out.mkdir(exist_ok=False)
    os.environ.update(selected.environment(out/'ros', session=out))
    result = dict(kind='GEN62_returned_generation6_manual_restore', passed=False,
        started_ns=time.time_ns(), helper_sha256=args.helper_sha256,
        motion_commands_sent=False, temperature_queried=False,
        joint_disable_commands_sent=False, manual_request_sent=False,
        automatic_attempt_authorized_by_this_receipt=False, deadline_refresh_performed=False)
    subscriptions = []
    publisher = None
    mod = None
    try:
        assert sha(__file__) == args.helper_sha256
        assert args.post_helper_sha256 == POST_SHA and sha(POST) == POST_SHA
        assert sha(BASELINE) == BASELINE_SHA
        spec = importlib.util.spec_from_file_location('checked_gen62_post_idle', POST)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        # common.load_context must receive the post helper's independently pinned
        # hash, while this helper has its own explicit caller-provided hash.
        post_args = argparse.Namespace(**vars(args))
        post_args.helper_sha256 = args.post_helper_sha256
        common, ctx, release, deployment, selected, record = mod.load_bindings(post_args)
        mod.add_pin(Path(__file__).resolve(), args.helper_sha256)
        mod.add_pin(BASELINE, BASELINE_SHA)
        assert record == args.record.resolve(strict=True)
        import rosgraph
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('gen62_completed_attempt_manual_restore', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        before = mod.accepted_graph(common, ctx, rospy, master)
        policy = mod.readonly_policy(rospy)
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection')) == selected
        lock, values, samples = threading.Lock(), {}, []
        topics = [
            ('joint','/alicia_d/accepted_joint_states',JointState,DRIVER),
            ('epoch','/alicia_d/control_reference_epoch',Header,DRIVER),
            ('manual','/gui/joint_direct_mode',Bool,GUI),
            ('task','/grasp/state',GraspState,'/grasp_task_node'),
            ('mode','/grasp_mode/status',String,'/grasp_mode_router'),
            ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray,'/bessica_d_hw_interface'),
            ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray,'/bessica_d_hw_interface'),
            ('actuation','/alicia_d/actuation_status',String,DRIVER),
            ('ready','/alicia_d/feedback_ready',Bool,DRIVER),
            ('enabled','/alicia_d/motion_enabled',Bool,DRIVER)]

        def receive(key, owner, msg):
            if msg._connection_header.get('callerid') != owner:
                return
            received = rospy.Time.now().to_nsec()
            with lock:
                values[key] = (msg, received)
                if key == 'joint':
                    samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=received,
                        names=list(msg.name), positions=list(msg.position), frame_id=msg.header.frame_id,
                        caller=msg._connection_header.get('callerid')))
                    samples[:] = samples[-40:]

        def refresh_latched():
            for key, topic, kind, owner in topics:
                if key not in ('joint','arm','hand'):
                    receive(key, owner, rospy.wait_for_message(topic, kind, timeout=3.0))

        def checked_snapshot():
            with lock:
                initial, states = list(samples[-20:]), dict(values)
            assert not rospy.is_shutdown()
            evidence = validate_idle_snapshot(initial, states, rospy.Time.now().to_nsec(), deployment['epoch_ns'])
            return initial, states, evidence

        for key, topic, kind, owner in topics:
            subscriptions.append(rospy.Subscriber(topic,kind,lambda msg,k=key,o=owner:receive(k,o,msg),queue_size=40))
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            with lock:
                done = len(values)==len(topics) and len(samples)>=20
            if done:
                break
            time.sleep(.025)
        refresh_latched()
        initial, states, sampled = checked_snapshot()
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection')) == selected
        result.update(selection=selected, epoch_ns=deployment['epoch_ns'], samples_before=initial,
            prior_manual_mode=rospy.get_param('/gui/joint_direct_mode'), record_dir=str(record),
            closed_record_manifest=dict(path=str(args.record_manifest.resolve()),sha256=args.record_manifest_sha256),
            original_runner_exit=dict(path=str(args.runner_exit.resolve()),sha256=args.runner_exit_sha256),
            returned_entry=dict(path=str(args.attempt_result.resolve()),sha256=args.attempt_result_sha256),
            selection_authorization=dict(path=str(args.authorization.resolve()),sha256=args.authorization_sha256),
            task_active=False, actions_active=False, initial_snapshot=sampled)
        mod.verify_sources(release['source_sha256'])
        common.recheck(ctx)
        publisher = rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
        deadline = time.monotonic()+5
        while publisher.get_num_connections()==0 and time.monotonic()<deadline:
            time.sleep(.025)
        assert publisher.get_num_connections()>0
        # File/control checks and publisher waiting precede the final live sample.
        # A slow check cannot turn the original snapshot into current evidence.
        mod.verify_sources(release['source_sha256'])
        common.recheck(ctx)
        mod.closed_record(post_args, mod.read(args.attempt_result), mod.read(args.runner_exit))
        before_request = mod.accepted_graph(common,ctx,rospy,master)
        assert before_request['node_identities'] == before['node_identities']
        assert mod.readonly_policy(rospy) == policy
        refresh_latched()
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection')) == selected
        final_samples, final_states, final_sampled = checked_snapshot()
        requested_ns = rospy.Time.now().to_nsec()
        assert 0 <= requested_ns-final_samples[-1]['stamp_ns'] < 200_000_000
        result['manual_request_started'] = True
        rospy.set_param('/gui/joint_direct_mode',True)
        result['manual_parameter_write_returned'] = True
        publisher.publish(Bool(data=True))
        result.update(manual_request_sent=True,manual_request_ns=requested_ns,
            samples_before_request=final_samples, final_snapshot=final_sampled,
            exact_nodes_before_request=before_request['node_identities'])
        deadline = time.monotonic()+5
        ack = None
        while time.monotonic()<deadline:
            with lock:
                ack = values.get('manual')
            if own_ack(ack, requested_ns):
                break
            time.sleep(.025)
        assert own_ack(ack, requested_ns)
        assert rospy.get_param('/gui/joint_direct_mode') is True
        publisher.unregister()
        publisher = None
        after = mod.accepted_graph(common,ctx,rospy,master)
        assert after['node_identities'] == before['node_identities']
        assert mod.readonly_policy(rospy) == policy
        assert mod.selection_identity(rospy.get_param('/grasp_mode/selection')) == selected
        result.update(passed=True,gui_own_manual_ack=True,gui_ack_received_ns=ack[1],
            exact_nodes_before=before['node_identities'],exact_nodes_after=after['node_identities'])
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        if publisher is not None:
            publisher.unregister()
        for sub in subscriptions:
            sub.unregister()
        result['finished_ns'] = time.time_ns()
        if mod is not None:
            result['input_sha256'] = {str(p):h for p,h in mod.PINS.items()}
        raw = (json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
        with (out/'result.json').open('xb') as f:
            f.write(raw)
            f.flush()
            os.fsync(f.fileno())
        assert (out/'result.json').read_bytes() == raw
    print(json.dumps({k:v for k,v in result.items() if k not in
        ('samples_before','samples_before_request','exact_nodes_before','exact_nodes_after')},ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
