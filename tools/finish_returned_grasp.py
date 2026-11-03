#!/usr/bin/env python3
"""Finish a sealed grasp attempt in manual idle, without starting another task.

The caller supplies a SHA-bound release/record validator and the existing SDK
idle validator. Those validators retain their source, authorization, instance,
epoch and selection checks. This entry owns only the ending sequence: verify
closure, observe, request manual only when needed, then verify the final state.
It never decides the outcome of the grasp or retries a motion.
"""

import argparse
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parent))
from experiment_storage import select as select_storage


class FinishError(RuntimeError):
    pass


def require(condition, reason):
    if not condition:
        raise FinishError(reason)


def manual_state(snapshot):
    parameter, acknowledged = snapshot['manual_parameter'], snapshot['gui_manual_ack']
    require(type(parameter) is bool and type(acknowledged) is bool,
            'manual state must contain two booleans')
    require(parameter == acknowledged, 'manual parameter and GUI own acknowledgement disagree')
    return parameter


def same_binding(first, current):
    for key in ('node_identities', 'selection', 'epoch_ns', 'policy'):
        require(first[key] == current[key], 'finish binding changed: ' + key)


def finish_returned_attempt(adapter):
    """Use one checked observation window; a repeated manual request is a no-op.

    observe_idle() must validate real SDK samples and their owners/freshness,
    action/task idle and the GUI's own latched acknowledgement. Returning a
    finished task to manual does not grant or require actuation confirmation.
    recheck() must freshly verify current authorization, source and closure.
    No method may start a grasp. A request is sent at most once, after the
    publisher is connected and a new boundary observation has been validated.
    """
    adapter.verify_closed()
    adapter.recheck()
    first = adapter.observe_idle()
    if manual_state(first):
        return dict(passed=True, outcome='already_manual', manual_request_sent=False,
                    final_snapshot=first)

    adapter.prepare_manual_request()
    adapter.verify_closed()
    adapter.recheck()
    current = adapter.observe_idle()
    same_binding(first, current)
    # Manual may have been restored by the original runner while connecting.
    if manual_state(current):
        return dict(passed=True, outcome='already_manual', manual_request_sent=False,
                    final_snapshot=current)
    request = adapter.request_manual(current)
    adapter.wait_gui_own_ack(request)
    adapter.verify_closed()
    adapter.recheck()
    final = adapter.observe_idle()
    same_binding(first, final)
    require(manual_state(final), 'manual restoration did not complete')
    return dict(passed=True, outcome='restored_manual', manual_request_sent=True,
                manual_request_ns=request, final_snapshot=final)


def checked_module(path, expected, name):
    path = Path(path).resolve(strict=True)
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, 'validator bytes changed: ' + str(path))
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    # Execute the bytes just verified; do not reopen the path through a loader.
    exec(compile(raw, str(path), 'exec'), module.__dict__)
    return module


@contextmanager
def validator_imports(*paths):
    """Preserve each validator's sibling imports through its later callbacks."""
    previous = list(sys.path)
    roots = list(dict.fromkeys(str(Path(path).resolve(strict=True).parent) for path in paths))
    sys.path[:] = roots + [entry for entry in previous if entry not in roots]
    try:
        yield
    finally:
        sys.path[:] = previous


def close_adapter(adapter, result):
    """Preserve the primary outcome even if transport cleanup also fails."""
    if adapter is not None:
        try:
            adapter.close()
        except BaseException as exc:
            result.update(passed=False, cleanup_error=repr(exc), cleanup_traceback=traceback.format_exc())


class RosFinishAdapter:
    def __init__(self, args, post, validator, result):
        self.args, self.post, self.validator, self.result = args, post, validator, result
        self.boundary = checked_module(Path(__file__).with_name('grasp_attempt_boundary.py'),
            args.boundary_sha256, 'checked_finish_manual_boundary')
        bound_args = argparse.Namespace(**vars(args))
        bound_args.helper_sha256 = args.binding_helper_sha256
        self.bound_args = bound_args
        self.common, self.ctx, self.release, self.deployment, self.selected, self.record = post.load_bindings(bound_args)
        self.subscriptions, self.publisher = [], None
        self.lock, self.values, self.samples = threading.Lock(), {}, []

    def verify_closed(self):
        return self.post.closed_record(self.bound_args, self.post.read(self.args.attempt_result),
                                       self.post.read(self.args.runner_exit))

    def recheck(self):
        self.post.verify_sources(self.release['source_sha256'])
        self.common.recheck(self.ctx)

    def connect(self):
        import rosgraph
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from alicia_flexible_grasp_supervisor.msg import GraspState
        self.rospy, self.Bool = rospy, Bool
        rospy.init_node('finish_returned_grasp', anonymous=True, disable_rosout=True)
        self.master = rosgraph.Master(rospy.get_name())
        self.topics = [
            ('joint', '/alicia_d/accepted_joint_states', JointState),
            ('epoch', '/alicia_d/control_reference_epoch', Header),
            ('manual', '/gui/joint_direct_mode', Bool),
            ('task', '/grasp/state', GraspState),
            ('mode', '/grasp_mode/status', String),
            ('arm', '/alicia_controller/follow_joint_trajectory/status', GoalStatusArray),
            ('hand', '/hand_controller/follow_joint_trajectory/status', GoalStatusArray),
            ('actuation', '/alicia_d/actuation_status', String),
            ('ready', '/alicia_d/feedback_ready', Bool),
            ('enabled', '/alicia_d/motion_enabled', Bool)]
        require(set(self.validator.OWNERS) == {key for key, _, _ in self.topics},
                'idle validator topic ownership differs')
        for key, topic, kind in self.topics:
            self.subscriptions.append(rospy.Subscriber(
                topic, kind, lambda msg, k=key: self.receive(k, msg), queue_size=40))
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if len(self.values) == len(self.topics) and len(self.samples) >= 20:
                    return
            time.sleep(.025)
        raise FinishError('initial idle sample window unavailable')

    def receive(self, key, msg):
        if msg._connection_header.get('callerid') != self.validator.OWNERS[key]:
            return
        received = self.rospy.Time.now().to_nsec()
        with self.lock:
            self.values[key] = (msg, received)
            if key == 'joint':
                self.samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), received_ns=received,
                    names=list(msg.name), positions=list(msg.position), frame_id=msg.header.frame_id,
                    caller=msg._connection_header.get('callerid')))
                self.samples[:] = self.samples[-40:]

    def observe_idle(self):
        if not hasattr(self, 'rospy'):
            self.connect()
        post, rospy = self.post, self.rospy
        graph = post.accepted_graph(self.common, self.ctx, rospy, self.master)
        policy = post.readonly_policy(rospy)
        require(post.selection_identity(rospy.get_param('/grasp_mode/selection')) == self.selected,
                'returned selection changed')
        # Refresh the existing latched state while keeping the bounded SDK
        # subscription/window; do not wait for another 20 samples per boundary.
        for key, topic, kind in self.topics:
            if key not in ('joint', 'arm', 'hand'):
                self.receive(key, rospy.wait_for_message(topic, kind, timeout=3.))
        require(post.selection_identity(rospy.get_param('/grasp_mode/selection')) == self.selected,
                'returned selection changed while sampling')
        parameter = rospy.get_param('/gui/joint_direct_mode')
        with self.lock:
            samples, states = list(self.samples[-20:]), dict(self.values)
        require(not rospy.is_shutdown(), 'ROS stopped during finish')
        evidence = self.boundary.validate_manual_return_snapshot(samples, states,
            rospy.Time.now().to_nsec(), self.deployment['epoch_ns'], self.validator.OWNERS)
        return dict(manual_parameter=parameter, gui_manual_ack=states['manual'][0].data,
            gui_ack_received_ns=states['manual'][1], node_identities=deepcopy(graph['node_identities']),
            selection=deepcopy(self.selected), epoch_ns=self.deployment['epoch_ns'], policy=policy,
            latest_sdk_stamp_ns=samples[-1]['stamp_ns'], idle=evidence)

    def prepare_manual_request(self):
        self.publisher = self.rospy.Publisher('/gui/joint_direct_mode', self.Bool, queue_size=1, latch=True)
        deadline = time.monotonic() + 5.
        while self.publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            require(not self.rospy.is_shutdown(), 'ROS stopped before manual request')
            time.sleep(.025)
        require(self.publisher.get_num_connections() > 0, 'manual publisher has no connection')

    def request_manual(self, snapshot):
        requested = self.rospy.Time.now().to_nsec()
        require(0 <= requested - snapshot['latest_sdk_stamp_ns'] < 200_000_000,
                'SDK expired before manual request')
        self.result['manual_request_started'] = True
        self.rospy.set_param('/gui/joint_direct_mode', True)
        self.result['manual_parameter_write_returned'] = True
        self.publisher.publish(self.Bool(data=True))
        self.result.update(manual_request_sent=True, manual_request_ns=requested)
        return requested

    def wait_gui_own_ack(self, requested):
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            with self.lock:
                ack = self.values.get('manual')
            if self.validator.own_ack(ack, requested):
                return
            time.sleep(.025)
        raise FinishError('fresh GUI own manual acknowledgement unavailable')

    def close(self):
        errors = []
        handles = ([self.publisher] if self.publisher is not None else []) + self.subscriptions
        for handle in handles:
            try:
                handle.unregister()
            except BaseException as exc:
                errors.append(repr(exc))
        require(not errors, 'finish transport cleanup failed: ' + '; '.join(errors))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--expected-generation', type=int, required=True)
    for key in ('binding-helper', 'idle-validator', 'release', 'deployment', 'attempt-result',
                'runner-exit', 'authorization', 'record-manifest'):
        parser.add_argument('--' + key, type=Path, required=True)
        parser.add_argument('--' + key + '-sha256', required=True)
    for key in ('helper', 'runtime-common', 'common', 'boundary'):
        parser.add_argument('--' + key + '-sha256', required=True)
    return parser.parse_args(argv)


def main(argv=None):
    require(not sys.flags.optimize, 'original assertion gates require unoptimized Python')
    args = parse_args(argv)
    require(hashlib.sha256(Path(__file__).read_bytes()).hexdigest() == args.helper_sha256,
            'finish helper bytes changed')
    out = args.output.absolute()
    selected = select_storage(path=out)
    selected.validate(out)
    require(out.parent.exists(), 'output parent unavailable')
    out.mkdir(exist_ok=False)
    os.environ.update(selected.environment(out/'ros', session=out))
    result = dict(kind='returned_grasp_finish', passed=False, started_ns=time.time_ns(),
        helper_sha256=args.helper_sha256, manual_request_sent=False, motion_commands_sent=False,
        temperature_queried=False, joint_disable_commands_sent=False,
        automatic_attempt_authorized_by_this_receipt=False, physical_success='unverified',
        original_attempt_result=dict(path=str(args.attempt_result.resolve()), sha256=args.attempt_result_sha256))
    result['input_bindings'] = {key: dict(path=str(getattr(args, key).resolve()),
        sha256=getattr(args, key + '_sha256')) for key in ('binding_helper', 'idle_validator',
        'release', 'deployment', 'attempt_result', 'runner_exit', 'authorization', 'record_manifest')}
    adapter = None
    try:
        with validator_imports(args.binding_helper, args.idle_validator):
            post = checked_module(args.binding_helper, args.binding_helper_sha256, 'checked_finish_binding')
            idle = checked_module(args.idle_validator, args.idle_validator_sha256, 'checked_finish_idle')
            adapter = RosFinishAdapter(args, post, idle, result)
            result.update(finish_returned_attempt(adapter))
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc())
    finally:
        close_adapter(adapter, result)
        result['finished_ns'] = time.time_ns()
        raw = (json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
        with (out / 'result.json').open('xb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        require((out / 'result.json').read_bytes() == raw, 'finish receipt readback differs')
    print(json.dumps({k: result.get(k) for k in
        ('passed', 'outcome', 'manual_request_sent', 'error', 'original_attempt_result')}, ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.dont_write_bytecode = True
    sys.exit(main())
