"""Verify a sealed recorded attempt is idle, then request GUI manual mode only.

The post-attempt selection comes from the recorded runner acknowledgement. This
does not renew a scene or authorize another attempt, trajectory, or gripper move.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback

import recorded_attempt_common as common

OLD = common.W / '.runtime/restore_gen62_after_handoff_20261022.py'
OLD_SHA = 'f1d5ad558dfd40cf5f7e59820b6ab84b55b40933236014debe41229c155ab61d'


def bind_completed_auxiliary(context, owner, evidence, current, system):
    """Bind a reviewed lazy ROS facade to the unchanged gateway process.

    Called only after closed_binding, for returning ownership to manual. This
    does not classify the facade as passive: its MoveIt publishers are retained.
    The original entry contract and production inventory are not modified.
    """
    require = common.require
    require(owner == '/motion_gateway' and owner in context['runtime_rows'],
            'completed auxiliary owner is not the accepted motion gateway')
    original = context['runtime_rows'][owner]
    name = evidence['node']
    require(type(name) is str and name.startswith('/move_group_commander_wrappers_')
            and name not in context['nodes'] and name not in context['deployment']['auxiliary_nodes'],
            'completed auxiliary name conflicts with an accepted node')
    expected = {k: evidence[k] for k in ('identity', 'uri', 'cmdline_sha256')}
    require(expected['identity'] == original['identity']
            and expected['cmdline_sha256'] == original['cmdline_sha256'],
            'completed auxiliary is not the original gateway process')
    require(current == expected, 'completed auxiliary live instance differs')
    require(sorted(evidence['services']) == [name + '/get_loggers', name + '/set_logger_level'],
            'completed auxiliary services are not the reviewed ROS logger services')
    require(not set(evidence['publishers']).intersection(common.PUBLISHERS),
            'completed auxiliary cannot own protected feedback or mode topics')
    for index, key in enumerate(('publishers', 'subscribers', 'services')):
        declared = evidence[key]
        require(type(declared) is list and all(type(v) is str for v in declared)
                and len(set(declared)) == len(declared), 'invalid completed auxiliary graph declaration')
        actual = sorted(topic for topic, owners in system[index] if name in owners)
        require(actual == sorted(declared), 'completed auxiliary graph differs: ' + key)
    auxiliary = dict(context['deployment']['auxiliary_nodes'])
    auxiliary[name] = dict(expected['identity'], uri=expected['uri'])
    services = dict(context['contract']['expected_services'])
    for service in evidence['services']:
        require(service not in services, 'completed auxiliary service owner conflicts')
        services[service] = [name]
    return dict(context, deployment=dict(context['deployment'], auxiliary_nodes=auxiliary),
                contract=dict(context['contract'], expected_services=services))


def live_post_identity(context, rospy, master):
    post = context
    for item in context['contract'].get('post_attempt_auxiliaries', []):
        evidence = context['artifacts'][item['evidence_ref']]
        current = common._node_row(master, evidence['node'], rospy.get_name())
        post = bind_completed_auxiliary(post, item['owner'], evidence, current, master.getSystemState())
    return common.live_identity(post, rospy, master)


def closed_binding(scene, record, prior, *, current=None, authorization_root=None):
    """No old generation number or successful-result assumption is inherited."""
    require = common.require
    require(record.parent == common.E / 'grasp_records', 'unexpected record root')
    manifest = common.read(record / 'manifest.json')
    outcome = common.read(record / 'recorder_outcome.json')
    require(manifest['state'] == 'complete' and manifest['recorder_closed'] is True
            and outcome['bag_closed'] is True, 'record is not sealed')
    entry = common.read(scene / 'recorded_entry_result.json')
    launch = common.read(scene / 'recording_launch_claim.json')
    require(manifest['metadata']['command'] == launch['inner_command'], 'record command differs')
    require(Path(launch['observation']['path']).parent == scene, 'record scene differs')
    if entry.get('record_dir'):
        require(Path(entry['record_dir']).resolve() == record, 'entry record differs')
    authorization_path = record / 'authorization.json'
    selected = common.selection_identity(prior)
    if authorization_path.exists():
        authorization = common.read(authorization_path)
        require(Path(authorization['session_dir']).resolve() == scene
                and common.same_selection(authorization['previous_selection'], prior),
                'recorded prior selection or session differs')
        if authorization.get('selection') is not None:
            selected = common.selection_identity(authorization['selection'])
            require(selected['generation'] == common.selection_identity(prior)['generation'] + 1
                    and selected['stamp_ns'] > common.selection_identity(prior)['stamp_ns']
                    and selected['mode'] == prior['mode'] and selected['strategy'] == prior['strategy'],
                    'recorded successor selection differs')
            commit = authorization['selection_commit']
            require(commit['latched_selection_matched'] is True
                    and common.same_selection({k: commit['router_status'][k] for k in selected}, selected),
                    'recorded selection was not acknowledged')
        elif current is not None and not common.same_selection(current, prior):
            # A failed reset/ACK can advance the router parameter before the
            # original launcher records its successor. Recover manual ownership
            # only if that recorded request never committed a runner release.
            selected = common.selection_identity(current)
            prior_identity = common.selection_identity(prior)
            requested_ns = authorization['runner_startup_timing']['selection_request_ros_ns']
            require(type(requested_ns) is int and requested_ns > prior_identity['stamp_ns']
                    and selected['generation'] == prior_identity['generation'] + 1
                    and selected['stamp_ns'] >= requested_ns
                    and selected['mode'] == prior_identity['mode']
                    and selected['strategy'] == prior_identity['strategy'],
                    'unacknowledged selection is not this recorded request successor')
            require(authorization_root is not None and 'continuation_release_commit' not in authorization,
                    'unacknowledged selection may already have released a runner')
            claim = common.read(record / 'continuation_attempt_claim.json')
            attempt_id = claim['attempt_id']
            require(type(attempt_id) is str and len(attempt_id) == 32
                    and all(c in '0123456789abcdef' for c in attempt_id), 'invalid attempt ledger key')
            ledger = common.read(authorization_root / 'ledger' / (attempt_id + '.json'))
            require(ledger['attempt_id'] == claim['attempt_id']
                    and ledger['authorization_id'] == claim['authorization_id']
                    and Path(ledger['record_dir']).resolve() == Path(claim['record_dir']).resolve() == record
                    and Path(ledger['session_dir']).resolve() == Path(claim['session_dir']).resolve() == scene
                    and 'release_commit' not in ledger and ledger['status'] == 'launcher_error',
                    'failed selection lacks an unreleased, closed launcher ledger')
    return selected


def main():
    p = common.parser(__doc__)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--record', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    out = common.storage(a.output)
    out.mkdir()
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    result = dict(passed=False, started_ns=time.time_ns(), kind='sealed_attempt_idle_to_GUI_manual',
                  motion_commands_sent=False, temperature_queried=False, joint_disable_commands_sent=False,
                  manual_request_sent=False, new_attempt_authorized=False, physical_success='unverified')
    subscriptions, publisher = [], None
    try:
        ctx = common.load_context(a, __file__)
        scene, record = a.scene.resolve(strict=True), a.record.resolve(strict=True)
        common.require(scene.parent == ctx['session_root'], 'scene outside bound session')
        old = common.checked_module(OLD, OLD_SHA, 'reviewed_idle_manual_primitives')
        ctx['pins'][OLD] = OLD_SHA
        import rosgraph
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        rospy.init_node('completed_bound_attempt_manual_restore', anonymous=True, disable_rosout=True)
        master = rosgraph.Master(rospy.get_name())
        current = rospy.get_param('/grasp_mode/selection')
        selected = closed_binding(scene, record, ctx['deployment']['selection'], current=current,
                                  authorization_root=ctx['authorization_path'].parent)
        # This reader binds the completed successor or an unreleased failed
        # selection only for returning to manual. No auto authority is renewed.
        # manual=true is an entry requirement, not a precondition for restoring
        # manual after an automatic attempt. Its live GUI owner/ACK is checked
        # explicitly below; the original entry contract remains unchanged.
        post_policy = {k: v for k, v in ctx['contract']['policy_parameters'].items()
                       if k != '/gui/joint_direct_mode'}
        post = dict(ctx, deployment=dict(ctx['deployment'], selection=selected),
                    contract=dict(ctx['contract'], policy_parameters=post_policy))
        lock, values, samples = threading.Lock(), {}, []
        topics = [
            ('joint', '/alicia_d/accepted_joint_states', JointState),
            ('epoch', '/alicia_d/control_reference_epoch', Header),
            ('manual', '/gui/joint_direct_mode', Bool), ('task', '/grasp/state', GraspState),
            ('mode', '/grasp_mode/status', String),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray),
            ('actuation', '/alicia_d/actuation_status', String),
            ('ready', '/alicia_d/feedback_ready', Bool), ('enabled', '/alicia_d/motion_enabled', Bool)]

        def receive(key, msg):
            if msg._connection_header.get('callerid') != old.OWNERS[key]:
                return
            now = rospy.Time.now().to_nsec()
            with lock:
                values[key] = (msg, now)
                if key == 'joint':
                    samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=now,
                        names=list(msg.name), positions=list(msg.position), frame_id=msg.header.frame_id,
                        caller=msg._connection_header.get('callerid')))
                    samples[:] = samples[-40:]

        def snapshot():
            for key, topic, kind in topics:
                if key not in ('joint', 'arm', 'hand'):
                    receive(key, rospy.wait_for_message(topic, kind, timeout=3))
            with lock:
                history, state = list(samples[-20:]), dict(values)
            evidence = old.validate_idle_snapshot(history, state, rospy.Time.now().to_nsec(),
                                                 ctx['deployment']['epoch_ns'])
            common.require(common.same_selection(rospy.get_param('/grasp_mode/selection'), selected),
                           'post-attempt selection changed')
            return history, state, evidence

        subscriptions = [rospy.Subscriber(topic, kind, lambda msg, k=key: receive(k, msg), queue_size=40)
                         for key, topic, kind in topics]
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            with lock:
                ready = len(values) == len(topics) and len(samples) >= 20
            if ready:
                break
            time.sleep(.025)
        before = live_post_identity(post, rospy, master)
        history, state, evidence = snapshot()
        result.update(selection=selected, record_dir=str(record), prior_manual=state['manual'][0].data,
                      before=evidence, manifest=common.ref(record / 'manifest.json'))
        auth = common.read(record / 'authorization.json') if (record / 'authorization.json').exists() else {}
        result['selection_basis'] = ('recorded_acknowledged_successor' if auth.get('selection') is not None
            else ('recorded_unreleased_failed_selection' if not common.same_selection(selected, ctx['deployment']['selection'])
                  else 'unchanged_pre_attempt_selection'))
        publisher = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
        deadline = time.monotonic() + 5
        while not publisher.get_num_connections() and time.monotonic() < deadline:
            time.sleep(.025)
        common.require(publisher.get_num_connections() > 0, 'GUI manual subscriber unavailable')
        common.recheck(ctx)
        common.require(closed_binding(scene, record, ctx['deployment']['selection'],
                       current=rospy.get_param('/grasp_mode/selection'),
                       authorization_root=ctx['authorization_path'].parent) == selected,
                       'closed record binding changed')
        live_post_identity(post, rospy, master)
        history, state, evidence = snapshot()
        requested = rospy.Time.now().to_nsec()
        common.require(0 <= requested - history[-1]['stamp_ns'] < 200_000_000, 'SDK expired before manual request')
        rospy.set_param('/gui/joint_direct_mode', True)
        publisher.publish(Bool(data=True))
        result.update(manual_request_sent=True, requested_ns=requested, final_before_request=evidence)
        deadline = time.monotonic() + 5
        ack = None
        while time.monotonic() < deadline:
            with lock:
                ack = values.get('manual')
            if old.own_ack(ack, requested):
                break
            time.sleep(.025)
        common.require(old.own_ack(ack, requested) and rospy.get_param('/gui/joint_direct_mode') is True,
                       'GUI own manual acknowledgement missing')
        publisher.unregister(); publisher = None
        after = live_post_identity(post, rospy, master)
        common.require(after['nodes'] == before['nodes'], 'production instances changed during manual return')
        _, final, final_evidence = snapshot()
        common.require(final['manual'][0].data is True, 'GUI no longer manual')
        result.update(passed=True, gui_own_manual_ack=True, gui_ack_received_ns=ack[1], after=final_evidence)
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        if publisher is not None:
            publisher.unregister()
        for subscriber in subscriptions:
            subscriber.unregister()
        result['finished_ns'] = time.time_ns()
        common.new_json(out / 'result.json', result)
    print(json.dumps({k: result.get(k) for k in ('passed', 'manual_request_sent', 'gui_own_manual_ack', 'error')}))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
