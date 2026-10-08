#!/usr/bin/env python3
"""Bounded read-only metadata capture; no images, publishers or ROS services.

The status String payload and camera parameter envelope/payload_json are saved
unaltered as JSON values. Parameter receipt intervals are not exposure times.
Nothing here establishes a target identity or authorizes robot motion.
"""
import argparse
from collections import Counter, deque
import hashlib
import http.client
import json
import math
import os
from pathlib import Path
import threading
import time
import xmlrpc.client

EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
STATUS_TOPIC = '/perception/unknown/detector_status'
PARAMETER = '/camera_read_diagnostics'
DIRECTORY_LIMIT = 64*1024*1024
FINAL_RESERVE = 64*1024
QUEUE_LIMIT = 256
QUEUE_BYTES = 16*1024*1024
ITEM_LIMIT = 4*1024*1024
RPC_TIMEOUT_SEC = 1.0
POLL_INTERVAL_SEC = .5


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def encode(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True,
                       separators=(',', ':'), allow_nan=False)+'\n').encode('utf8')


def atomic_json(path, value):
    raw = encode(value)
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('xb') as handle:
        handle.write(raw)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)
    return len(raw)


class BoundedInbox:
    """Callbacks enqueue only; no file writes, JSON parsing or RPC under lock."""
    def __init__(self, maximum_items=QUEUE_LIMIT, maximum_bytes=QUEUE_BYTES):
        self.maximum_items, self.maximum_bytes = maximum_items, maximum_bytes
        self.lock = threading.Lock()
        self.items = deque()
        self.bytes = 0
        self.closed = False
        self.counts = Counter()

    def put(self, item, size):
        with self.lock:
            if self.closed:
                self.counts['rejected_after_close'] += 1
                return False
            if size > ITEM_LIMIT or size < 0:
                self.counts['item_size_rejected'] += 1
                return False
            if len(self.items) >= self.maximum_items or self.bytes+size > self.maximum_bytes:
                self.counts['queue_bound_dropped'] += 1
                return False
            self.items.append((item, size))
            self.bytes += size
            self.counts['enqueued'] += 1
            self.counts['maximum_queued_items'] = max(self.counts['maximum_queued_items'], len(self.items))
            self.counts['maximum_queued_bytes'] = max(self.counts['maximum_queued_bytes'], self.bytes)
            return True

    def pop(self):
        with self.lock:
            if not self.items:
                return None
            item, size = self.items.popleft()
            self.bytes -= size
            self.counts['dequeued'] += 1
            return item

    def close(self):
        with self.lock:
            self.closed = True

    def snapshot(self):
        with self.lock:
            return dict(self.counts, queued_items=len(self.items), queued_bytes=self.bytes,
                        admission_closed=self.closed)


class JsonlSink:
    def __init__(self, path, budget):
        self.path, self.budget = path, budget
        self.handle = path.open('xb')
        self.hash = hashlib.sha256()
        self.bytes = 0
        self.records = 0
        self.limit_reached = False
        self.closed = False

    def append(self, item):
        raw = encode(item)
        if self.bytes+len(raw) > self.budget:
            self.limit_reached = True
            return False
        written = self.handle.write(raw)
        if written != len(raw):
            raise OSError('short metadata write')
        self.hash.update(raw)
        self.bytes += written
        self.records += 1
        return True

    def close(self):
        self.handle.flush()
        os.fsync(self.handle.fileno())
        self.handle.close()
        self.closed = True


def status_annotation(payload, expected_generation):
    """Only explicit integer-generation first_loss can shorten observation."""
    result = dict(expected_generation=expected_generation, generation_matches=False,
                  first_loss_for_expected_generation=False)
    try:
        value = json.loads(payload)
    except (ValueError, TypeError):
        result['parse_error'] = 'invalid_status_json'
        return result
    if not isinstance(value, dict):
        result['parse_error'] = 'status_not_object'
        return result
    generation = value.get('generation')
    if type(generation) is not int:
        result['parse_error'] = 'generation_not_integer'
        return result
    result['observed_generation'] = generation
    result['generation_matches'] = generation == expected_generation
    loss = value.get('first_loss')
    result['first_loss_for_expected_generation'] = bool(
        generation == expected_generation and isinstance(loss, dict) and loss)
    if result['first_loss_for_expected_generation']:
        loss_generation = loss.get('generation', expected_generation)
        if type(loss_generation) is not int or loss_generation != expected_generation:
            result['first_loss_for_expected_generation'] = False
            result['parse_error'] = 'first_loss_generation_conflict'
    return result


def parameter_annotation(value):
    """Diagnostic inspection never changes the original envelope/string."""
    result = dict(expected_parameter=PARAMETER, parse_valid=False, schema_valid=False)
    if not isinstance(value, dict):
        result['error'] = 'parameter_not_object'
        return result
    result['observed_schema'] = value.get('schema')
    payload = value.get('payload_json')
    if value.get('encoding') != 'json' or not isinstance(payload, str):
        result['error'] = 'parameter_encoding_or_payload_invalid'
        return result
    result['payload_utf8_sha256'] = digest(payload.encode('utf8'))
    try:
        parsed = json.loads(payload)
    except (ValueError, TypeError):
        result['error'] = 'payload_json_invalid'
        return result
    result['parse_valid'] = True
    result['decoded_payload_type'] = type(parsed).__name__
    if type(value.get('schema')) is not int or value['schema'] != 1:
        result['error'] = 'unsupported_envelope_schema'
        return result
    if not isinstance(parsed, dict):
        result['error'] = 'diagnostic_payload_not_object'
        return result
    generation = parsed.get('geometry_generation')
    entries = parsed.get('entries')
    if (type(parsed.get('schema')) is not int or parsed['schema'] != 1
            or type(parsed.get('capacity')) is not int or parsed['capacity'] != 32
            or type(generation) is not int or generation < 0
            or parsed.get('diagnostic_only') is not True
            or parsed.get('motion_authorized') is not False
            or not isinstance(entries, list) or len(entries) > 32):
        result['error'] = 'diagnostic_payload_schema_invalid'
        return result
    for row in entries:
        if (not isinstance(row, dict) or type(row.get('source_stamp_ns')) is not int
                or row['source_stamp_ns'] <= 0
                or type(row.get('geometry_generation')) is not int
                or row['geometry_generation'] != generation
                or not isinstance(row.get('evidence'), dict)):
            result['error'] = 'diagnostic_source_entry_invalid'
            return result
    result.update(schema_valid=True, source_entry_count=len(entries), geometry_generation=generation)
    return result


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        chost, self._extra_headers, _ = self.get_host_info(host)
        return http.client.HTTPConnection(chost, timeout=RPC_TIMEOUT_SEC)


def read_parameter_once(proxy, caller, ros_now, monotonic=time.monotonic, wall=time.time_ns):
    started = dict(monotonic=monotonic(), wall_ns=wall(), ros_ns=ros_now())
    record = dict(kind='camera_read_diagnostics_parameter', parameter=PARAMETER,
                  rpc_started=started, receipt_semantics='RPC read interval; not source exposure')
    try:
        code, message, value = proxy.getParam(caller, PARAMETER)
        record['original_rpc_code'] = code
        record['original_rpc_message'] = message
        if code == 1:
            record['original_parameter_value'] = value
            record['annotation'] = parameter_annotation(value)
        else:
            record['error'] = 'parameter_unavailable_or_rpc_rejected'
    except Exception as exc:
        record['error'] = type(exc).__name__+': '+str(exc)
    record['rpc_finished'] = dict(monotonic=monotonic(), wall_ns=wall(), ros_ns=ros_now())
    return record


class Capture:
    def __init__(self, output, expected_generation, seconds, rospy):
        self.output, self.expected_generation, self.seconds = output, expected_generation, seconds
        self.rospy = rospy
        self.inbox = BoundedInbox()
        self.stop = threading.Event()
        self.active = threading.Event()
        self.rpc_initialized = threading.Event()
        self.start = None
        self.first_loss = None
        self.counts = Counter()
        self.metadata_errors = Counter()
        self.generations = Counter()
        self.callback_counts = Counter()
        self.rpc_counts = Counter()

    def ros_now(self):
        return self.rospy.Time.now().to_nsec()

    def status_callback(self, msg):
        if not self.active.is_set() or self.stop.is_set():
            self.callback_counts['outside_capture'] += 1
            return
        receipt = dict(monotonic=time.monotonic(), wall_ns=time.time_ns(), ros_ns=self.ros_now())
        payload = msg.data
        if not isinstance(payload, str):
            self.callback_counts['non_string_rejected'] += 1
            return
        raw = payload.encode('utf8')
        if len(raw) > ITEM_LIMIT-1024:
            self.callback_counts['oversize_rejected'] += 1
            return
        row = dict(kind='detector_status', topic=STATUS_TOPIC, receipt=receipt,
            original_status_data=payload, original_payload_utf8_sha256=digest(raw))
        self.callback_counts['received'] += 1
        self.inbox.put(row, len(raw)+1024)

    def poll_parameters(self):
        master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        proxy = xmlrpc.client.ServerProxy(master, transport=TimeoutTransport(), allow_none=True)
        try:
            self.rpc_initialized.set()
            while not self.stop.is_set():
                started = time.monotonic()
                row = read_parameter_once(proxy, self.rospy.get_name(), self.ros_now)
                self.rpc_counts['completed'] += 1
                # Explicit per-item bound; the native XMLRPC reply is read only
                # from the local ROS master, not arbitrary untrusted endpoints.
                raw = encode(row)
                if not self.stop.is_set():
                    self.inbox.put(row, len(raw))
                else:
                    self.rpc_counts['finished_after_capture_closed'] += 1
                self.stop.wait(max(0., POLL_INTERVAL_SEC-(time.monotonic()-started)))
        except Exception as exc:
            self.rpc_counts['thread_error'] += 1
            self.inbox.put(dict(kind='metadata_poll_thread_error', error=type(exc).__name__+': '+str(exc),
                                receipt_monotonic=time.monotonic()), 1024)
        finally:
            proxy('close')()

    def prepare(self, row):
        if row['kind'] == 'detector_status':
            annotation = status_annotation(row['original_status_data'], self.expected_generation)
            annotation['receipt_phase'] = ('capture' if self.start is not None
                and row['receipt']['monotonic'] >= self.start else 'startup_admitted_before_ready')
            row['annotation'] = annotation
            if 'observed_generation' in annotation:
                self.generations[str(annotation['observed_generation'])] += 1
        return row

    def saved(self, row):
        self.counts[row['kind']] += 1
        if row.get('error'):
            self.metadata_errors[row['error']] += 1
        if row['kind'] == 'detector_status' and row.get('annotation', {}).get('parse_error'):
            self.metadata_errors['status_schema_invalid:'+row['annotation']['parse_error']] += 1
        if row['kind'] == 'camera_read_diagnostics_parameter':
            if 'original_parameter_value' in row:
                self.counts['camera_parameter_read_success'] += 1
            annotation = row.get('annotation', {})
            if annotation.get('schema_valid') is True:
                self.counts['camera_parameter_schema_valid'] += 1
                if annotation['source_entry_count'] > 0:
                    self.counts['camera_parameter_with_source_entries'] += 1
            elif 'original_parameter_value' in row:
                self.metadata_errors['camera_parameter_schema_invalid:'+annotation.get('error', 'missing_annotation')] += 1
        if (row['kind'] == 'detector_status'
                and row['annotation']['first_loss_for_expected_generation']
                and row['annotation']['receipt_phase'] == 'capture'
                and self.first_loss is None):
            self.first_loss = dict(receipt=row['receipt'],
                original_payload_utf8_sha256=row['original_payload_utf8_sha256'])

    def due(self, now):
        if now >= self.start+self.seconds:
            return 'duration_limit'
        if self.first_loss is not None and now >= self.first_loss['receipt']['monotonic']+2.:
            return 'expected_generation_first_loss_plus_2_seconds'
        return None

    def run(self):
        from std_msgs.msg import String
        self.output.mkdir(exist_ok=False)
        setup_started = time.monotonic()
        ready = dict(schema=1, diagnostic_only=True, expected_generation=self.expected_generation,
            requested_seconds=self.seconds, hardware_commands_sent=False, images_subscribed=False,
            parameters_written=False, selection_changed=False, status_topic=STATUS_TOPIC,
            parameter=PARAMETER, poll_interval_sec=POLL_INTERVAL_SEC, rpc_timeout_sec=RPC_TIMEOUT_SEC,
            capture_timer_begins_after_ready_file_write=True, prepared_monotonic=time.monotonic(),
            storage_policy='local_only', max_directory_bytes=DIRECTORY_LIMIT,
            max_queue_items=QUEUE_LIMIT, max_queue_bytes=QUEUE_BYTES, max_item_bytes=ITEM_LIMIT,
            script_sha256=digest(Path(__file__).read_bytes()))
        sink = thread = subscriber = None
        ready_published = False
        reason, error = None, None
        try:
            # Publish ready only after file admission, subscription and the
            # independent RPC worker are live. A ready-triggered selection
            # therefore cannot outrun status capture. Any preceding receipts
            # remain explicitly marked as startup evidence, not capture time.
            sink = JsonlSink(self.output/'metadata.jsonl', DIRECTORY_LIMIT-FINAL_RESERVE-len(encode(ready)))
            self.active.set()
            subscriber = self.rospy.Subscriber(STATUS_TOPIC, String, self.status_callback,
                queue_size=64, buff_size=4*1024*1024, tcp_nodelay=True)
            thread = threading.Thread(target=self.poll_parameters, name='readonly_camera_metadata_rpc', daemon=True)
            thread.start()
            if not self.rpc_initialized.wait(RPC_TIMEOUT_SEC+.5):
                raise RuntimeError('metadata_rpc_thread_not_initialized')
            atomic_json(self.output/'ready.json', ready)
            self.start = time.monotonic()
            ready_published = True
            print('READY '+json.dumps(dict(output=str(self.output), capture_start_monotonic=self.start)), flush=True)
            while reason is None:
                now = time.monotonic()
                reason = self.due(now)
                if reason is not None:
                    break
                if self.rospy.is_shutdown():
                    reason = 'ros_shutdown'
                    break
                row = self.inbox.pop()
                if row is None:
                    self.stop.wait(.01)
                    continue
                self.prepare(row)
                if not sink.append(row):
                    reason = 'metadata_byte_budget'
                    break
                self.saved(row)
        except BaseException as exc:
            reason = 'capture_exception'
            error = type(exc).__name__+': '+str(exc)
        finally:
            capture_finished = time.monotonic()
            self.stop.set()
            self.active.clear()
            self.inbox.close()
            if subscriber is not None:
                subscriber.unregister()
            if thread is not None:
                thread.join(RPC_TIMEOUT_SEC+.5)
            try:
                while sink is not None and not sink.limit_reached:
                    row = self.inbox.pop()
                    if row is None:
                        break
                    self.prepare(row)
                    if not sink.append(row):
                        break
                    self.saved(row)
                if sink is not None:
                    sink.close()
            except Exception as exc:
                error = error or type(exc).__name__+': '+str(exc)
            actual_bytes = sum(p.stat().st_size for p in self.output.iterdir() if p.is_file())
            queue_status = self.inbox.snapshot()
            sealed = bool(sink is not None and sink.closed)
            thread_alive = bool(thread is not None and thread.is_alive())
            evidence_complete = bool(error is None and ready_published and sealed and not thread_alive
                and str(self.expected_generation) in self.generations
                and self.counts['camera_parameter_schema_valid'] > 0
                and self.counts['camera_parameter_with_source_entries'] > 0
                and not self.metadata_errors and not sink.limit_reached
                and not queue_status.get('queue_bound_dropped', 0)
                and not queue_status.get('item_size_rejected', 0)
                and not self.callback_counts.get('oversize_rejected', 0)
                and not self.rpc_counts.get('thread_error', 0))
            summary = dict(schema=1, sealed=sealed, expected_generation=self.expected_generation,
                observation_complete_without_reported_evidence_gaps=evidence_complete,
                ready_published=ready_published, setup_started_monotonic=setup_started,
                expected_generation_observed=str(self.expected_generation) in self.generations,
                capture_start_monotonic=self.start, capture_finished_monotonic=capture_finished,
                capture_duration_sec=(capture_finished-self.start if self.start is not None else None), requested_seconds=self.seconds,
                first_loss=self.first_loss, reason=reason, error=error, records_by_kind=dict(self.counts),
                generations=dict(self.generations), callback_counts=dict(self.callback_counts),
                rpc_counts=dict(self.rpc_counts), rpc_thread_alive=thread_alive,
                metadata_errors=dict(self.metadata_errors), queue=queue_status,
                jsonl_records=(sink.records if sink is not None else 0),
                jsonl_bytes=(sink.bytes if sink is not None else 0),
                jsonl_sha256=(sink.hash.hexdigest() if sink is not None else None),
                byte_budget_reached=(sink.limit_reached if sink is not None else False), directory_bytes_before_summary=actual_bytes,
                maximum_directory_bytes=DIRECTORY_LIMIT, hardware_commands_sent=False,
                limitations=['Metadata only; no target or motion authority.',
                    'Original status payload and parameter envelope/payload_json values are retained; XMLRPC wire bytes are not captured.',
                    'Parameter reads have actual RPC intervals, not exposure timestamps. Missing/schema-invalid data and queue/budget omissions remain explicit.',
                    'Other generations are recorded under their original identity and do not trigger the expected-generation early stop.',
                    'Socket timeouts and capture duration are cooperative; storage or scheduling stalls can delay final sealing.'])
            summary_raw = encode(summary)
            if actual_bytes+len(summary_raw) > DIRECTORY_LIMIT:
                raise RuntimeError('reserved_summary_budget_exceeded')
            atomic_json(self.output/'summary.json', summary)
            manifest = dict(schema=1, sealed=sealed,
                files={p.name:dict(bytes=p.stat().st_size, sha256=digest(p.read_bytes()))
                    for p in (self.output/'ready.json',self.output/'metadata.jsonl',self.output/'summary.json') if p.exists()})
            if actual_bytes+len(summary_raw)+len(encode(manifest)) > DIRECTORY_LIMIT:
                raise RuntimeError('reserved_manifest_budget_exceeded')
            atomic_json(self.output/'manifest.json', manifest)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
        return 0 if error is None and sealed and not thread_alive else 2


def validated_output(value):
    output = Path(value).expanduser().resolve()
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise ValueError('SanStand must be mounted; no fallback')
    if EXTERNAL.resolve() not in output.parents or output.exists():
        raise ValueError('output must be a new directory under SanStand Robot-Controller')
    if not output.parent.is_dir():
        raise ValueError('output parent must already exist')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--expected-generation', type=int, default=20)
    parser.add_argument('--seconds', type=float, default=60.)
    args = parser.parse_args()
    if (args.expected_generation < 0 or not math.isfinite(args.seconds)
            or not 1 <= args.seconds <= 60):
        parser.error('expected generation must be nonnegative and seconds must be 1..60')
    output = validated_output(args.output)
    import rospy
    rospy.init_node('support_metadata_readonly_observer', anonymous=True, disable_signals=False)
    return Capture(output, args.expected_generation, args.seconds, rospy).run()


if __name__ == '__main__':
    raise SystemExit(main())
