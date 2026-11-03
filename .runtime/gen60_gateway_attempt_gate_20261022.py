"""Additional read-only gate at the unchanged runner's manual-to-auto line.

No ROS work on import. Only the reviewed entry calls final_gate. The recorder
exception is one proven process, never a general ROS observer allow-list.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import xmlrpc.client

from gen60_gateway_attempt_common_20261022 import (
    A, ACTION_OWNER, CHECKER, CHECKER_SHA, CONTROL, CONTROL_SHA, DRIVER, E, GUI, Transport, W,
    live_identity, new_json, read, recheck, ref, require, same_selection, sha)

RECORDER = W / 'tools/record_grasp_keyframes_readonly.py'
RECORDER_SHA = 'b8b3eae0138aaa7f4e42e68da0f7b8a8d76f1f8c4ac266c2e888e5a191b81748'


def argv(pid):
    return (Path('/proc') / str(pid) / 'cmdline').read_bytes().rstrip(b'\0').decode().split('\0')


class RecordedMaster:
    """Expose the original 17-node graph after proving the sole recorder extra."""
    def __init__(self, context, rospy, master, record, launch):
        self.context, self.rospy, self.master = context, rospy, master
        self.record, self.launch = record, launch
        self.initial = None
        self.evidence = None

    def getPid(self):
        return self.master.getPid()

    def lookupNode(self, name):
        return self.master.lookupNode(name)

    def getSystemState(self):
        c, r, launch = self.context, self.record, self.launch
        ready_path = r / 'recorder_ready.json'
        ready, manifest = read(ready_path), read(r / 'manifest.json')
        require(r.parent == E / 'grasp_records' and ready['record_dir'] == str(r) and
                ready['bag_path'] == str(r / 'keyframes.bag') and
                manifest['state'] == 'recording' and manifest['recorder_closed'] is False and
                manifest['metadata']['command'] == launch['inner_command'] and
                manifest['metadata']['owner_pid'] == os.getppid() == launch['owner']['pid'] and
                manifest['owner']['pid'] == os.getppid() and
                manifest['owner']['start_ticks'] == launch['owner']['start_ticks'] and
                manifest['owner']['boot_id'] == c['startup']['boot_id'], 'record ownership/ready differs')
        owner, process = c['verifier'].process(os.getppid()), c['verifier'].process(ready['pid'])
        require(owner == launch['owner'] and process['parent_pid'] == owner['pid'] and
                int(process['start_ticks']) >= int(owner['start_ticks']) and
                argv(owner['pid']) == launch['record_command'] and
                sha(RECORDER) == RECORDER_SHA, 'recorder/wrapper process identity differs')
        expected_argv = [sys.executable, '-u', str(RECORDER), '--record-dir', str(r),
                         '--output', str(r / 'keyframes.bag'), '--duration', '600.0',
                         '--defer-finalize', '--config', str(E / 'grasp_records/archive_config.json'),
                         '--min-free-bytes', str(read(E / 'grasp_records/archive_config.json')['min_free_bytes'])]
        require(argv(process['pid']) == expected_argv, 'actual recorder argv differs')
        system = self.master.getSystemState()
        names = {name for group in system for _, rows in group for name in rows}
        extra = names - set(c['nodes']) - set(c['deployment']['auxiliary_nodes']) - {self.rospy.get_name()}
        require(len(extra) == 1, 'expected exactly this recording observer: ' + repr(sorted(extra)))
        observer = next(iter(extra))
        require(observer.startswith('/grasp_keyframe_recorder_readonly_'), 'extra ROS node is not recorder')
        response = xmlrpc.client.ServerProxy(self.master.lookupNode(observer), transport=Transport()).getPid(
            self.rospy.get_name())
        require(response[0] == 1 and response[2] == process['pid'], 'recorder ROS PID differs')
        current = dict(ready=ref(ready_path), process=process, owner=owner,
                       recorder_node=observer, recorder_argv=expected_argv)
        if self.initial is not None:
            require(current == self.initial, 'recorder instance changed during final gate')
        self.initial = current
        self.evidence = dict(current, raw_registered_nodes=sorted(names),
                             excluded_observer=observer, runner_node=self.rospy.get_name())
        return [[[topic, [n for n in rows if n != observer]] for topic, rows in group]
                for group in system]


class CurrentFeedback:
    def __init__(self, rospy, context, pre):
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy, self.context, self.pre = rospy, context, pre
        self.lock, self.latest, self.history = threading.Lock(), {}, []
        self.topics = [
            ('joint', '/alicia_d/accepted_joint_states', JointState, DRIVER),
            ('epoch', '/alicia_d/control_reference_epoch', Header, DRIVER),
            ('task', '/grasp/state', GraspState, '/grasp_task_node'),
            ('mode', '/grasp_mode/status', String, '/grasp_mode_router'),
            ('manual', '/gui/joint_direct_mode', Bool, GUI),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray, ACTION_OWNER),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray, ACTION_OWNER),
            ('feedback_ready', '/alicia_d/feedback_ready', Bool, DRIVER),
            ('actuation', '/alicia_d/actuation_status', String, DRIVER),
            ('motion_enabled', '/alicia_d/motion_enabled', Bool, DRIVER)]
        self.subscriptions = [rospy.Subscriber(topic, kind, lambda m, k=key: self.receive(k, m), queue_size=40)
                              for key, topic, kind, _ in self.topics]

    def receive(self, key, message):
        now = self.rospy.Time.now().to_nsec()
        with self.lock:
            self.latest[key] = (message, now)
            if key == 'joint':
                self.history.append(dict(stamp_ns=message.header.stamp.to_nsec(), received_ns=now,
                    frame_id=message.header.frame_id, names=list(message.name), positions=list(message.position),
                    callerid=message._connection_header.get('callerid')))
                self.history = self.history[-80:]

    def wait(self):
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            with self.lock:
                if len(self.latest) == len(self.topics) and len(self.history) >= 20:
                    return
            time.sleep(.025)
        raise RuntimeError('final 20 SDK/state observations incomplete')

    def refresh_latches(self):
        for key, topic, kind, _ in self.topics:
            if key not in ('joint', 'arm', 'hand'):
                self.receive(key, self.rospy.wait_for_message(topic, kind, timeout=3.))

    def check(self):
        rospy, pre = self.rospy, self.pre
        with self.lock:
            state, samples = dict(self.latest), list(self.history[-20:])
        now = rospy.Time.now().to_nsec()
        require(not rospy.is_shutdown() and len(state) == len(self.topics) and len(samples) == 20,
                'incomplete final current feedback')
        for key, _, _, owner in self.topics:
            require(0 <= now - state[key][1] < 1_000_000_000, 'stale final receipt: ' + key)
            if owner:
                require(state[key][0]._connection_header.get('callerid') == owner,
                        'final callback publisher differs: ' + key)
        mode, task = json.loads(state['mode'][0].data), state['task'][0]
        require(rospy.get_param('/gui/joint_direct_mode') is True and state['manual'][0].data is True and
                not task.active and not any(mode.get(k) for k in ('active', 'pending', 'start_reserved')),
                'GUI own manual/idle no longer true')
        for key in ('arm', 'hand'):
            message = state[key][0]
            require(0 <= now - message.header.stamp.to_nsec() <= 2_000_000_000 and
                    not any(g.status in (0, 1, 6, 7) for g in message.status_list), 'action busy or stale')
        require(state['epoch'][0].stamp.to_nsec() == pre['epoch_ns'] and
                same_selection(rospy.get_param('/grasp_mode/selection'), pre['selection']), 'runtime epoch/selection changed')
        require(state['feedback_ready'][0].data is True and state['motion_enabled'][0].data is True and
                state['actuation'][0].data.startswith('CONFIRMED:') and
                rospy.get_param('/grasp/require_actuation_confirmation') is True,
                'ACTUATION_OR_FEEDBACK_NOT_READY: ' + str(state['actuation'][0].data))
        original = pre['topics']['/alicia_d/accepted_joint_states']
        names, stamps = original['names'], [s['stamp_ns'] for s in samples]
        require(len(set(stamps)) == 20 and all(b > a for a, b in zip(stamps, stamps[1:])) and
                0 <= now - stamps[-1] < 200_000_000 and
                all(s['frame_id'] == 'sdk_measured' and s['callerid'] == DRIVER and s['names'] == names and
                    len(s['positions']) == len(names) and all(math.isfinite(q) for q in s['positions']) and
                    pre['epoch_ns'] <= s['stamp_ns'] <= s['received_ns'] and
                    s['received_ns'] - s['stamp_ns'] < 200_000_000 for s in samples), 'SDK provenance/age differs')
        spans = [max(s['positions'][i] for s in samples)-min(s['positions'][i] for s in samples)
                 for i in range(len(names))]
        maximum = max(abs(s['positions'][i]-original['positions'][i])
                      for s in samples for i in range(len(names)))
        require(max(spans) <= .004 and maximum <= .004, 'SDK no longer stationary at reviewed scene pose')
        require(rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False and
                rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.0 and
                rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False and
                rospy.get_param('/grasp/unknown_observe_return_max_observations') == 1,
                'temperature/lift/one-observation policy changed')
        return dict(checked_ns=now, samples=samples, maximum_joint_delta_rad=maximum,
                    sample_spans=spans, manual_mode=True, gui_owning_manual_ack=True,
                    task_active=False, actions_active=False, feedback_ready=True, motion_enabled=True,
                    actuation_status=state['actuation'][0].data, epoch_ns=pre['epoch_ns'],
                    selection=pre['selection'], state_received_ns={k: t for k, (_, t) in state.items()},
                    latch_receipt_time_is_not_source_generation_time=True)

    def close(self):
        subscriptions, self.subscriptions = self.subscriptions, []
        for subscription in subscriptions:
            subscription.unregister()


def final_gate(context, scene, record, launch, original_frame, result):
    import rosgraph
    import rospy
    pre = read(scene / 'preflight.json')
    continuation, runtime = original_frame.f_locals['continuation'], original_frame.f_locals['runtime']
    require(continuation is not None and runtime is not None and
            continuation.record == record and continuation.session == scene and
            original_frame.f_locals['control'].get_num_connections() > 0,
            'original continuation/runtime/GUI connection is not established')
    view = RecordedMaster(context, rospy, rosgraph.Master(rospy.get_name()), record, launch)
    feedback = CurrentFeedback(rospy, context, pre)
    try:
        feedback.wait()
        identity_before = live_identity(context, rospy, view)
        runtime.refresh_idle()
        runtime.check(original_frame.f_locals['expected_prior'])
        recheck(context)
        output = scene / 'remote_dependency_at_mode_switch'
        require(not output.exists(), 'no second remote/automatic attempt for this scene')
        completed = subprocess.run([sys.executable, '-B', str(CHECKER), '--output', str(output),
            '--startup-binding', str(context['binding_path']), '--startup-binding-sha256',
            context['pins'][context['binding_path']]], cwd=W, timeout=20.,
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
        remote = read(output / 'result.json')
        result['remote_dependency'] = ref(output / 'result.json')
        require(completed.returncode == 0 and remote['passed'] is True and remote['dependency_ready'] is True and
                remote['helper_sha256'] == CHECKER_SHA and
                remote['startup_binding_sha256'] == context['pins'][context['binding_path']] and
                same_selection(remote['after']['selection'], pre['selection']), 'current remote dependency not ready')
        identity_after = live_identity(context, rospy, view, graph_wait=0.)
        require(identity_before == identity_after, 'runtime17/GUI/publishers changed during final remote check')
        recheck(context)
        feedback.refresh_latches()
        current = feedback.check()
        continuation.check_active(rospy.Time.now().to_nsec())
        now = time.time_ns()
        require(sha(CONTROL) == CONTROL_SHA and 0 <= now - remote['finished_ns'] <= 5_000_000_000 and
                0 <= now - pre['receipt_ns'] < 90_000_000_000,
                'control or final remote/scene freshness expired at automatic-mode boundary')
        result.update(passed=True, checked_ns=now, final_feedback=current,
                      identity_before=identity_before, identity_after=identity_after,
                      exact_recorder=view.evidence, remote_finished_ns=remote['finished_ns'],
                      new_motion_authority_created=False, original_guards_replaced=False)
        new_json(scene / 'mode_switch_gate.json', result)
        # The trace returns directly to the unchanged set_param line. Recheck
        # short-lived evidence after fsync, too; a slow disk must fail closed.
        require(all(sha(p) == h for p, h in context['pins'].items()),
                'pinned input changed before original mode line')
        feedback.close()
        continuation.check_active(rospy.Time.now().to_nsec())
        feedback.check()
        require(0 <= time.time_ns() - remote['finished_ns'] <= 5_000_000_000 and
                time.time_ns() - pre['receipt_ns'] < 90_000_000_000,
                'pinned input or freshness changed before returning to original mode line')
    finally:
        feedback.close()
