#!/usr/bin/env python3
"""One reversible perception selection and bounded evidence; never starts a grasp.

Historical user receipts are provenance only, not a newly confirmed alignment or
an inherited target/plan. Import is pure. Live execution belongs to the operator.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
FROZEN = {
 'record_support_gap_15s.py': '2505f6353de06298101adc2d4c7799abc5c58a961e95b07b0a4152ad72f0534c',
 'observe_support_metadata_20261006.py': '3d643bdccf16e36d5685f2a5eb8751356f70bf21e08c4929214b5d8374e6a107'}
RUNNERS = {'mode_aware_grasp_runner.py', 'run_rgb_acquisition.py', 'run_recorded_grasp.py',
 'run_fresh_grasp6d_after_alignment.py', 'run_confirmed_unknown_grasp_20261004.py'}
NODE_PREFIXES = ('/mode_aware_grasp_runner', '/rgb_acquisition_runner',
 '/fresh_grasp6d_execution_runner', '/confirmed_unknown_grasp_handoff')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda:handle.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write(path, value):
    with Path(path).open('x', encoding='utf8') as handle:
        json.dump(value, handle, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        handle.write('\n')


def read(path):
    return json.loads(Path(path).read_text())


def selection(value):
    require(isinstance(value, dict), 'selection_not_object')
    require(type(value.get('generation')) is int and value['generation'] >= 0, 'selection_generation_invalid')
    result = {key:value[key] for key in ('mode', 'strategy', 'generation')}
    result['stamp_ns'] = int(value['stamp_ns'])
    require(result['stamp_ns'] > 0, 'selection_source_invalid')
    return result


def validate_ack(previous, actual):
    a, b = selection(previous), selection(actual)
    require(b['generation'] == a['generation']+1 and b['stamp_ns'] > a['stamp_ns']
        and b['mode'] == 'unknown' and b['strategy'] == 'observe_return', 'wrong_new_selection')
    return b


def validate_idle(task, mode):
    require(task.get('active') is False, 'grasp_task_not_inactive')
    require(mode.get('active') is False and mode.get('start_reserved') is False
        and not mode.get('pending') and mode.get('state') not in
        ('initializing', 'switching', 'switch_queued', 'switch_failed', 'error'), 'mode_not_idle')


def validate_latched_task(task, publisher_connections):
    # GraspState is latched and change-only; its old receipt is not a heartbeat.
    require(publisher_connections > 0, 'grasp_task_publisher_disconnected')
    require(task.get('active') is False, 'grasp_task_not_inactive')


def historical(path, kind):
    path = Path(path).resolve()
    require(EXT.resolve() in path.parents, 'historical_receipt_not_external')
    value = read(path)
    require(value.get('kind') == kind and value.get('confirmation_source', {}).get('kind') == 'user_message'
        and value.get('confirmation_id'), 'invalid_historical_user_receipt')
    event = Path(value['event_file']).resolve()
    require(EXT.resolve() in event.parents and sha(event) == value['event_file_sha256'], 'historical_event_changed')
    origin = read(event)
    require(origin.get('confirmation_source') == value['confirmation_source']
        and origin.get('confirmed_ns') == value['confirmed_ns'], 'historical_confirmation_mismatch')
    return dict(path=str(path), sha256=sha(path), event_path=str(event), event_sha256=sha(event),
        confirmation_id=value['confirmation_id'], original_confirmed_ns=value['confirmed_ns'],
        original_confirmation_source=value['confirmation_source'], use='historical authorization context only',
        current_alignment_inferred=False, inherited_target_or_plan=False)


def runner_audit(master, caller):
    hits = []
    unreadable = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit() or int(directory.name) == os.getpid():
            continue
        try:
            argv = (directory/'cmdline').read_bytes().split(b'\0')
        except FileNotFoundError:
            continue
        except PermissionError:
            unreadable.append(int(directory.name)); continue
        names = {Path(part.decode(errors='replace')).name for part in argv if part}
        if RUNNERS & names:
            hits.append(dict(pid=int(directory.name), matched=sorted(RUNNERS & names)))
    code, message, graph = master.getSystemState(caller)
    require(code == 1, 'node_graph_unavailable:'+str(message))
    nodes = sorted({name for section in graph for _, owners in section for name in owners})
    node_hits = [name for name in nodes if name.startswith(NODE_PREFIXES)]
    return dict(local_matching_processes=hits, registered_matching_nodes=node_hits,
        inaccessible_pids=unreadable, no_known_runner=not hits and not node_hits and not unreadable,
        limitation='Known local runner entrypoints and current ROS graph; not a global execution lock.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--alignment-receipt', required=True, type=Path)
    parser.add_argument('--stationary-receipt', required=True, type=Path)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    require(Path('/media/zhuyupei/SanStand').is_mount() and EXT.resolve() in output.parents
        and not output.exists() and output.parent.is_dir(), 'new_external_output_required_no_fallback')
    provenance = [historical(args.alignment_receipt, 'explicit_user_target_alignment'),
        historical(args.stationary_receipt, 'explicit_user_stationary_capture')]
    sources = {str(W/'.runtime'/name):digest for name,digest in FROZEN.items()}
    sources[str(Path(__file__).resolve())] = sha(__file__)
    require(all(sha(path) == digest for path,digest in sources.items()), 'frozen_source_changed')
    spec = importlib.util.spec_from_file_location('_fresh_probe_metadata', W/'.runtime/observe_support_metadata_20261006.py')
    meta = importlib.util.module_from_spec(spec); spec.loader.exec_module(meta)
    output.mkdir()
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    import rospy
    from std_msgs.msg import Header, String
    from sensor_msgs.msg import JointState
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_grasp_modes.srv import SetGraspMode
    rospy.init_node('fresh_perception_diagnostic_only', anonymous=True, disable_signals=True)
    caller = rospy.get_name()
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'], transport=meta.TimeoutTransport(), allow_none=True)
    latest, children, logs, subscriptions = {}, [], [], []
    lock = threading.Lock()
    stop = threading.Event()
    inbox = meta.BoundedInbox(maximum_items=1024, maximum_bytes=8*1024*1024)
    sink = meta.JsonlSink(output/'current_feedback_and_state.jsonl', 8*1024*1024)
    summary = dict(kind='fresh_perception_diagnostic_session', historical_authorization=provenance,
        new_user_confirmation_received=False, inherited_alignment=False, inherited_target_or_plan=False,
        motion_authorized=False, planning_requested=False, runner_started=False,
        robot_commands_sent=False, torque_commands_sent=False, global_indices_modified=False,
        service_calls=[], sources=sources, started_wall_ns=time.time_ns())
    def receipt():
        return dict(monotonic=time.monotonic(), wall_ns=time.time_ns(), ros_ns=rospy.Time.now().to_nsec())
    def callback(message, kind):
        now = receipt()
        if kind == 'mode':
            row = dict(original_payload=message.data)
            try: parsed = json.loads(message.data)
            except (ValueError, TypeError): parsed = None
        elif kind == 'epoch':
            row = dict(stamp_ns=message.stamp.to_nsec(), seq=message.seq, frame_id=message.frame_id); parsed = row
        else:
            row = dict(stamp_ns=message.header.stamp.to_nsec(), seq=message.header.seq, frame_id=message.header.frame_id)
            if kind == 'feedback':
                row.update(names=list(message.name), positions=list(message.position))
            else:
                row.update(active=bool(message.active), state=message.state, stage=message.stage, message=message.message)
            parsed = row
        record = dict(kind=kind, receipt=now, **row)
        with lock: latest[kind] = dict(value=parsed, receipt=now)
        inbox.put(record, len(meta.encode(record)))
    def drain():
        while True:
            item = inbox.pop()
            if item is None: break
            require(sink.append(item), 'feedback_metadata_budget_reached')
    def pause(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            require(not stop.is_set() and not rospy.is_shutdown(), 'probe_interrupted')
            drain(); time.sleep(min(.03, max(0,end-time.monotonic())))
    def get(parameter, optional=False):
        started = receipt(); code,message,value = master.getParam(caller, parameter)
        row = dict(parameter=parameter, started=started, finished=receipt(), original_rpc_code=code,
            original_rpc_message=message, original_parameter_value=value)
        summary.setdefault('parameter_reads', []).append(row)
        require(code == 1 or optional, 'parameter_unavailable:'+parameter)
        return value if code == 1 else None
    def state_snapshot():
        with lock: snap = json.loads(json.dumps(latest))
        require(all(k in snap for k in ('task','mode','feedback','epoch')), 'current_state_missing')
        validate_latched_task(snap['task']['value'], subscriptions[0].get_num_connections())
        snap['task']['publisher_connections_at_snapshot'] = subscriptions[0].get_num_connections()
        snap['task']['receipt_semantics'] = 'latched change-only state; receipt age is not heartbeat age'
        f = snap['feedback']['value']; now = rospy.Time.now().to_nsec()
        require(f['frame_id'] == 'sdk_measured' and 0 <= now-f['stamp_ns'] <= 1_000_000_000
            and len(f['positions']) == len(f['names']) and len(f['positions']) >= 6
            and all(math.isfinite(x) for x in f['positions']), 'current_sdk_feedback_invalid')
        require(0 < snap['epoch']['value']['stamp_ns'] <= f['stamp_ns'], 'feedback_epoch_invalid')
        validate_idle(snap['task']['value'], snap['mode']['value'])
        return snap
    def context():
        return {name:get(name, name == '/unknown_perception') for name in
            ('/camera','/handeye','/unknown_perception','/robot_description','/grasp_mode/selection')}
    old_handlers = {sig:signal.signal(sig, lambda *_:stop.set()) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        for topic, cls, kind in (('/grasp/state',GraspState,'task'),('/grasp_mode/status',String,'mode'),
            ('/alicia_d/accepted_joint_states',JointState,'feedback'),('/alicia_d/control_reference_epoch',Header,'epoch')):
            subscriptions.append(rospy.Subscriber(topic, cls, lambda m,k=kind:callback(m,k), queue_size=64))
        deadline = time.monotonic()+4.
        while len(latest) < 4 and time.monotonic() < deadline: pause(.05)
        summary['before'] = dict(context=context(), state=state_snapshot(), runner_audit=runner_audit(master,caller))
        require(summary['before']['runner_audit']['no_known_runner'], 'current_runner_present_or_unknown')
        previous = selection(summary['before']['context']['/grasp_mode/selection'])
        expected_generation = previous['generation']+1
        for name, subdirectory, extra in (
            ('record_support_gap_15s.py','raw_support',['--seconds','15']),
            ('observe_support_metadata_20261006.py','metadata',['--seconds','20','--expected-generation',str(expected_generation)])):
            log = (output/(subdirectory+'.log')).open('xb'); logs.append(log)
            child = subprocess.Popen([sys.executable,str(W/'.runtime'/name),'--output',str(output/subdirectory)]+extra,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=os.environ.copy())
            children.append((subdirectory,child))
        deadline = time.monotonic()+6.
        while not all((output/name/'ready.json').exists() for name,_ in children):
            require(time.monotonic() < deadline and all(p.poll() is None for _,p in children), 'recorder_not_ready')
            pause(.05)
        summary['recorder_ready'] = {name:read(output/name/'ready.json') for name,_ in children}
        summary['pre_select_state'] = state_snapshot()
        require(selection(get('/grasp_mode/selection')) == previous, 'selection_changed_before_single_call')
        require(runner_audit(master,caller)['no_known_runner'], 'runner_appeared_before_selection')
        raw_ready = summary['recorder_ready']['raw_support']
        require(raw_ready['initial_generation'] == previous['generation']
            and int(raw_ready['initial_selection_stamp_ns']) == previous['stamp_ns'], 'recorder_initial_selection_mismatch')
        rospy.wait_for_service('/grasp_mode/select', timeout=2.)
        call = dict(service='/grasp_mode/select', request=dict(mode='unknown',strategy='observe_return'), started=receipt())
        summary['service_calls'].append(call)
        outcome = {}
        def select_once():
            try:
                result = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown','observe_return')
                outcome.update(success=bool(result.success), message=result.message)
            except Exception as exc: outcome['error'] = repr(exc)
            finally: outcome['finished'] = receipt()
        worker = threading.Thread(target=select_once, daemon=True); worker.start()
        deadline = time.monotonic()+3.
        while worker.is_alive() and time.monotonic() < deadline: pause(.03)
        call.update(outcome)
        require(not worker.is_alive(), 'select_response_timeout_effect_unknown_no_retry')
        require(outcome.get('success') is True and outcome.get('message') == 'SWITCHING', 'select_not_switching_no_retry')
        deadline = time.monotonic()+5.
        while True:
            selected = get('/grasp_mode/selection')
            try:
                selected = validate_ack(previous,selected)
                snap = state_snapshot()
                require(selection(snap['mode']['value']) == selected, 'mode_ack_not_current')
                break
            except ValueError as exc:
                reason = str(exc)
                failures = summary.setdefault('ack_wait_rejections', {})
                failures[reason] = failures.get(reason, 0)+1
                summary['ack_wait_last_rejection'] = reason
                require(time.monotonic() < deadline, 'selection_ack_timeout:'+reason)
                pause(.1)
        summary['selected'] = selected
        summary['selection_ack_state'] = snap
        while any(p.poll() is None for _,p in children):
            require(time.monotonic() < deadline+22., 'recorders_completion_timeout')
            pause(.05)
        summary['after'] = dict(context=context(), state=state_snapshot(), runner_audit=runner_audit(master,caller))
        require(selection(summary['after']['context']['/grasp_mode/selection']) == selected, 'selection_changed_during_capture')
        require(summary['after']['state']['epoch']['value']['stamp_ns'] == summary['before']['state']['epoch']['value']['stamp_ns'], 'epoch_changed_during_capture')
        require(summary['after']['runner_audit']['no_known_runner'], 'runner_present_after_capture')
    except BaseException as exc:
        summary['error'] = type(exc).__name__+': '+str(exc)
    finally:
        # Only our own finite read-only recorder processes are signalled. Never ROS nodes/robot drivers.
        for name,child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try: child.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    try: child.wait(timeout=2.)
                    except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=2.)
        for sub in subscriptions: sub.unregister()
        inbox.close()
        try: drain()
        except Exception as exc: summary['drain_error'] = repr(exc)
        sink.close()
        for log in logs: log.close()
        for sig,handler in old_handlers.items(): signal.signal(sig,handler)
        summary.update(finished_wall_ns=time.time_ns(), queue=inbox.snapshot(), feedback_log_sha256=sink.hash.hexdigest(),
            source_unchanged=all(sha(path)==digest for path,digest in sources.items()),
            recorder_returncodes={name:p.returncode for name,p in children},
            quality_or_target_acceptance=False, grasp_or_fusion_validation=False,
            limitations=['Selection service clears previous perception/planning caches and publishes invalid plan tombstones; no planning/start request is sent.',
                'Known-runner/inactive checks are observations, not an exclusive control lock.',
                'Raw recording lasts 15 seconds after its ready; metadata lasts at most 20 seconds and may end first-loss+2 seconds. Actual intervals are preserved.',
                'Pending or retired new perception is a valid diagnostic outcome, not a successful grasp.'])
        summary['child_summaries'] = {name:read(output/name/'summary.json') if (output/name/'summary.json').exists() else None for name,_ in children}
        summary['capture_closed'] = bool(len(children)==2 and all(p.returncode==0 for _,p in children)
            and all(summary['child_summaries'].values()) and not summary.get('error') and not summary.get('drain_error') and summary['source_unchanged'])
        write(output/'summary.json',summary)
        manifest = {str(path.relative_to(output)):dict(bytes=path.stat().st_size,sha256=sha(path))
            for path in output.rglob('*') if path.is_file() and not path.is_symlink() and 'ros' not in path.relative_to(output).parts}
        write(output/'manifest.json', dict(schema=1,files=manifest,scope='closed probe files; ROS logs excluded as independently rotating'))
        print(json.dumps(dict(output=str(output),capture_closed=summary['capture_closed'],error=summary.get('error')),ensure_ascii=False),flush=True)
    return 0 if summary['capture_closed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
