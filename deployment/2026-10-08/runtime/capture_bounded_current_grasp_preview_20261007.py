#!/usr/bin/env python3
"""Bounded current-selection planning diagnostic; never selects or executes.

Import is pure. The only mutating RPC is request_plan: True starts inference,
False stops it only when this invocation received the exact 'started' response.
The capture deadline is cooperative; a blocked local RPC/I/O can delay cleanup.
Saved previews are diagnostic records, never transferable motion authority.
"""
import argparse
import base64
from collections import Counter, deque
import hashlib
import io
import json
import os
from pathlib import Path
import threading
import time

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
W = Path(__file__).resolve().parents[1]
LIMIT = 10 * 1024 * 1024
RESERVE = 512 * 1024
STARTED = 'continuous remote 6D inference started'
RUNNING = 'continuous remote 6D inference already running'
TOPICS = {
    'preview': '/grasp_6d/preview_plan_enriched',
    'gate_audit': '/grasp_6d/gate_audit',
    'metrics': '/grasp_6d/pipeline_metrics',
    'planner_status': '/grasp_6d/status',
    'detector_status': '/perception/unknown/detector_status',
}


def require(value, reason):
    if not value:
        raise ValueError(reason)


def selection(value):
    require(isinstance(value, dict), 'selection_not_object')
    require(type(value.get('generation')) is int and value['generation'] >= 0,
            'selection_generation_invalid')
    stamp = value.get('stamp_ns')
    require(type(stamp) is int or (type(stamp) is str and stamp.isdigit()),
            'selection_stamp_invalid')
    require(int(stamp) > 0, 'selection_stamp_invalid')
    require(value.get('mode') == 'unknown' and value.get('strategy') == 'observe_return',
            'expected_unknown_observe_return')
    return dict(mode='unknown', strategy='observe_return',
                generation=value['generation'], stamp_ns=int(stamp))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, value):
    with Path(path).open('x', encoding='utf8') as handle:
        json.dump(value, handle, ensure_ascii=False, sort_keys=True, indent=2,
                  allow_nan=False)
        handle.write('\n')


def message_value(value):
    if hasattr(value, 'to_nsec'):
        return dict(stamp_ns=int(value.to_nsec()))
    if hasattr(value, '__slots__'):
        return {key: message_value(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [message_value(item) for item in value]
    return value


def ownership(response):
    require(response.success is True, 'request_plan_rejected:' + str(response.message))
    require(response.message in (STARTED, RUNNING), 'unexpected_request_plan_response')
    return response.message == STARTED


def verify_idle(task, mode, expected, actual, connected, mode_connected=True):
    require(connected, 'grasp_task_publisher_disconnected')
    require(mode_connected, 'grasp_mode_publisher_disconnected')
    # Both publishers are latched/change-only; receipt age is not a heartbeat.
    require(task.get('active') is False, 'grasp_task_active_or_unknown')
    require(selection(actual) == expected, 'selection_changed')
    require(mode.get('generation') == expected['generation']
            and mode.get('mode') == expected['mode']
            and mode.get('strategy') == expected['strategy'], 'mode_selection_mismatch')
    require(mode.get('active') is False and mode.get('start_reserved') is False
            and not mode.get('pending') and mode.get('state') not in
            ('switching', 'switch_queued', 'switch_failed', 'initializing', 'error'),
            'mode_busy_or_unknown')


class Capture:
    """Small callback queue; file reads and writes run only in the main loop."""
    def __init__(self, output, expected, *, limit=LIMIT, per_kind=100):
        self.output = Path(output)
        self.expected = selection(expected)
        self.limit = limit
        self.per_kind = per_kind
        self.lock = threading.Lock()
        self.queue = deque()
        self.counts = Counter()
        self.dropped = Counter()
        self.bytes_reserved = 0
        self.start_ns = None
        self.closed = False
        self.budget_reached = False
        self.previews = []
        self.audits = {}
        self.audit_errors = Counter()
        self.matched = None
        self.terminal = None
        self.records = 0
        self.files = {}
        self.handle = (self.output / 'messages.jsonl').open('xb')

    def enqueue(self, kind, message, receipt_ns, mono):
        with self.lock:
            if self.closed or self.start_ns is None or receipt_ns <= self.start_ns:
                return
            if self.counts[kind] >= self.per_kind:
                self.dropped[kind + ':count_limit'] += 1
                return
            try:
                stream = io.BytesIO()
                message.serialize(stream)
                serialized = stream.getvalue()
                require(len(serialized) <= 2 * 1024 * 1024, 'message_too_large')
                row = dict(kind=kind, topic=TOPICS[kind], receipt_ros_ns=receipt_ns,
                           receipt_monotonic=mono, ros_type=message._type,
                           serialized_base64=base64.b64encode(serialized).decode('ascii'),
                           serialized_sha256=sha(serialized), message=message_value(message))
                if kind != 'preview':
                    row['raw_text'] = message.data
                data = (json.dumps(row, ensure_ascii=True, sort_keys=True,
                                   allow_nan=False) + '\n').encode()
            except Exception as exc:
                self.dropped[kind + ':' + type(exc).__name__] += 1
                return
            if self.bytes_reserved + len(data) > self.limit - RESERVE:
                self.dropped[kind + ':byte_limit'] += 1
                self.budget_reached = True
                return
            self.bytes_reserved += len(data)
            self.counts[kind] += 1
            self.queue.append((data, row))

    def _read_audit(self, reference):
        digest = reference.get('report_sha256', '')
        require(isinstance(digest, str) and len(digest) == 64
                and all(c in '0123456789abcdef' for c in digest), 'audit_hash_invalid')
        if digest in self.audits:
            return
        path = Path(reference.get('report_path', ''))
        require(path.is_absolute() and not path.is_symlink(), 'audit_path_invalid')
        real = path.resolve()
        require(EXT.resolve() in real.parents or (W / '.runtime').resolve() in real.parents,
                'audit_path_outside_allowed_roots')
        with path.open('rb') as source:
            raw = source.read(2 * 1024 * 1024 + 1)
        require(len(raw) <= 2 * 1024 * 1024, 'audit_too_large')
        require(sha(raw) == digest, 'audit_atomic_reference_superseded')
        report = json.loads(raw)
        require(selection(report.get('grasp_mode_selection')) == self.expected,
                'audit_selection_mismatch')
        require(type(report.get('snapshot_stamp_ns')) is int
                and report['snapshot_stamp_ns'] > self.start_ns, 'audit_source_not_new')
        with self.lock:
            if self.bytes_reserved + len(raw) > self.limit - RESERVE:
                self.budget_reached = True
                raise ValueError('audit_byte_limit')
            self.bytes_reserved += len(raw)
        name = 'audit_' + digest + '.json'
        with (self.output / name).open('xb') as target:
            target.write(raw)
        self.files[name] = dict(sha256=digest, bytes=len(raw), original_path=str(path))
        self.audits[digest] = dict(plan_id=report.get('plan_id'),
            source_stamp_ns=report['snapshot_stamp_ns'], outcome=report.get('outcome'),
            request_id=report.get('request_id'), generation=report.get('generation'),
            target_epoch=report.get('target_epoch'), name=name)

    def _pair(self):
        for preview in self.previews:
            for digest, audit in self.audits.items():
                if (audit['plan_id'] == preview['plan_id']
                        and audit['source_stamp_ns'] == preview['source_stamp_ns']
                        and (audit['outcome'] or {}).get('preview_valid') is True):
                    self.matched = dict(preview=preview, audit=audit, audit_sha256=digest)
                    return

    def drain(self):
        while True:
            with self.lock:
                if not self.queue:
                    break
                data, row = self.queue.popleft()
            self.handle.write(data)
            self.records += 1
            if row['kind'] == 'preview':
                value = row['message']
                stamp = value.get('header', {}).get('stamp', {}).get('stamp_ns', 0)
                if stamp <= self.start_ns:
                    continue
                item = dict(source_stamp_ns=stamp, plan_id=value.get('plan_id', ''),
                            candidate_source=value.get('candidate_source'),
                            diagnostic=value.get('diagnostic'), valid=value.get('valid'))
                if value.get('valid') is True and item['plan_id']:
                    self.previews.append(item)
                elif value.get('candidate_source') == 'near_field_terminal':
                    # Terminal header is publication time, not a measured image stamp.
                    self.terminal = dict(item, stamp_semantics='terminal_publication_time',
                                         source_pairing_claimed=False)
            elif row['kind'] == 'gate_audit':
                try:
                    self._read_audit(json.loads(row['raw_text']))
                except Exception as exc:
                    self.audit_errors[str(exc)[:180]] += 1
            self._pair()

    def finish(self):
        with self.lock:
            self.closed = True
        try:
            self.drain()
        finally:
            self.handle.close()
        path = self.output / 'messages.jsonl'
        self.files[path.name] = dict(sha256=sha(path.read_bytes()), bytes=path.stat().st_size)
        return dict(records=self.records, counts=dict(self.counts), dropped=dict(self.dropped),
                    budget_reached=self.budget_reached, audit_errors=dict(self.audit_errors),
                    matched_preview=self.matched, terminal=self.terminal, files=self.files)


def capture_loop(capture, *, request, validate, clock=time.monotonic,
                 sleep=time.sleep, ros_now_ns, shutdown, seconds=90):
    """Injectable transaction: no selection/start/replan callback exists here."""
    require(60 <= seconds <= 100, 'duration_outside_60_100_seconds')
    owned = False
    result = dict(stream_owned=False, request_calls=[], reason=None, cleanup_error=None)
    try:
        validate()
        capture.start_ns = int(ros_now_ns())
        require(capture.start_ns > capture.expected['stamp_ns'], 'source_clock_invalid')
        started = clock()
        response = request(True)
        result['request_calls'].append(dict(trigger=True, success=response.success, message=response.message))
        owned = ownership(response)
        result['stream_owned'] = owned
        while not shutdown():
            validate()
            capture.drain()
            if capture.matched:
                result['reason'] = 'valid_preview_with_exact_audit'; break
            if capture.terminal:
                result['reason'] = 'planner_terminal'; break
            if capture.budget_reached:
                result['reason'] = 'evidence_byte_budget'; break
            if clock() - started >= seconds:
                result['reason'] = 'capture_timeout'; break
            sleep(.05)
        result['reason'] = result['reason'] or 'ros_shutdown'
        result['elapsed_sec'] = clock() - started
    except BaseException as exc:
        result['reason'] = 'capture_error'
        result['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        if owned:
            try:
                response = request(False)
                result['request_calls'].append(dict(trigger=False, success=response.success, message=response.message))
                require(response.success is True, 'stop_inference_rejected')
            except BaseException as exc:
                result['cleanup_error'] = type(exc).__name__ + ': ' + str(exc)
        try:
            result['evidence'] = capture.finish()
        except BaseException as exc:
            result['write_error'] = type(exc).__name__ + ': ' + str(exc)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--expected-selection', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=90.)
    args = parser.parse_args(argv)
    require(60 <= args.seconds <= 100, 'duration_outside_60_100_seconds')
    require(Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand_required_no_fallback')
    out = args.output.resolve()
    require(EXT.resolve() in out.parents, 'output_must_be_on_SanStand')
    original = args.expected_selection.read_bytes()
    envelope = json.loads(original)
    expected = selection(envelope.get('new_selection', envelope.get('selection', envelope)))
    out.mkdir(parents=False, exist_ok=False)
    (out / 'ros').mkdir()
    # Bound our own ROS logger independently from the evidence budget.
    logging = out / 'logging.conf'
    logging.write_text('[loggers]\nkeys=root\n[handlers]\nkeys=rotating\n[formatters]\nkeys=plain\n'
        '[logger_root]\nlevel=WARNING\nhandlers=rotating\n[handler_rotating]\n'
        'class=handlers.RotatingFileHandler\nlevel=WARNING\nformatter=plain\n'
        'args=("' + str(out / 'ros' / 'node.log') + '","a",65536,1)\n'
        '[formatter_plain]\nformat=%(asctime)s %(levelname)s %(message)s\n')
    os.environ['ROS_LOG_DIR'] = str(out / 'ros')
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = str(logging)
    import rospy
    from std_msgs.msg import String
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, GraspState
    from alicia_flexible_grasp_supervisor.srv import TriggerZero
    rospy.init_node('bounded_current_grasp_preview', anonymous=True, disable_signals=True)
    live, lock = {}, threading.Lock()
    def update(kind, msg):
        with lock:
            if kind == 'task':
                live[kind] = message_value(msg)
            else:
                try:
                    live[kind] = json.loads(msg.data)
                except Exception:
                    live[kind] = {}
            live[kind + '_mono'] = time.monotonic()
    task_sub = rospy.Subscriber('/grasp/state', GraspState, lambda msg:update('task', msg), queue_size=1)
    mode_sub = rospy.Subscriber('/grasp_mode/status', String, lambda msg:update('mode', msg), queue_size=1)
    until = time.monotonic() + 5
    while time.monotonic() < until and not ('task' in live and 'mode' in live):
        time.sleep(.05)
    capture = Capture(out, expected)
    subscriptions = []
    for kind, topic in TOPICS.items():
        typ = Grasp6DPlan if kind == 'preview' else String
        subscriptions.append(rospy.Subscriber(topic, typ,
            lambda msg, key=kind:capture.enqueue(key, msg, rospy.Time.now().to_nsec(), time.monotonic()),
            queue_size=8))
    def validate():
        actual = rospy.get_param('/grasp_mode/selection')
        with lock:
            verify_idle(live.get('task', {}), live.get('mode', {}), expected, actual,
                        task_sub.get_num_connections() > 0, mode_sub.get_num_connections() > 0)
    write_json(out / 'context.json', dict(expected_selection=expected,
        selection_file=str(args.expected_selection.resolve()), selection_file_sha256=sha(original),
        script_sha256=sha(Path(__file__).read_bytes()), duration_sec=args.seconds,
        motion_requested=False, target_alignment_inferred=False, selection_changed=False,
        caller_must_exclude_independent_execution_runners=True, planning_only=True))
    result = None
    try:
        rospy.wait_for_service('/grasp_6d/request_plan', timeout=5)
        request = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)
        result = capture_loop(capture, request=request, validate=validate,
            ros_now_ns=lambda:rospy.Time.now().to_nsec(), shutdown=rospy.is_shutdown, seconds=args.seconds)
    except BaseException as exc:
        result = dict(reason='startup_error', error=type(exc).__name__ + ': ' + str(exc),
                      evidence=capture.finish())
    finally:
        for sub in subscriptions + [task_sub, mode_sub]:
            sub.unregister()
        if result is None:
            result = dict(reason='startup_error', evidence=capture.finish())
        result.update(expected_selection=expected, motion_requested=False,
            selection_changed=False, interpretation='preview diagnosis only; no grasp/fusion validation',
            byte_limit=LIMIT, cooperative_deadline=True)
        result['directory_bytes_before_summary'] = sum(p.stat().st_size for p in out.rglob('*') if p.is_file())
        result['budget_passed_before_summary'] = result['directory_bytes_before_summary'] < LIMIT - 65536
        write_json(out / 'summary.json', result)
    print(json.dumps(dict(output=str(out), reason=result['reason'], stream_owned=result.get('stream_owned'),
                          cleanup_error=result.get('cleanup_error')), ensure_ascii=False), flush=True)
    return 1 if (result.get('error') or result.get('cleanup_error') or result.get('write_error')
                 or not result['budget_passed_before_summary']) else 0


if __name__ == '__main__':
    raise SystemExit(main())
