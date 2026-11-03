#!/usr/bin/env python3
"""Finite current-scene RGB-D evidence only. Never selects a target or moves.

Published rectified images are not SDK raw images. Exact image stamp matching,
TF and measured joint bracketing are evidence, not physical-static validation.
Dependencies are frozen, pure-import recording helpers; ROS imports occur only
in the live entrypoint. No old selection, status or support reference is read.
"""
import argparse
from collections import Counter, deque
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import threading
import time
import xmlrpc.client

HERE = Path(__file__).resolve().parent
FROZEN = {
 'record_support_gap_15s.py':'2505f6353de06298101adc2d4c7799abc5c58a961e95b07b0a4152ad72f0534c',
 'observe_support_metadata_20261006.py':'3d643bdccf16e36d5685f2a5eb8751356f70bf21e08c4929214b5d8374e6a107'}
def load(name):
    path = HERE/name
    if hashlib.sha256(path.read_bytes()).hexdigest() != FROZEN[name]:
        raise ValueError('recording dependency source changed: '+name)
    spec = importlib.util.spec_from_file_location('_current_support_'+path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
base = load('record_support_gap_15s.py')
meta = load('observe_support_metadata_20261006.py')
EXTERNAL = base.EXTERNAL
MAX_DIRECTORY = 40*1024*1024
MAX_ARCHIVE = 38*1024*1024
MAX_IMAGE = 4*1024*1024
MAX_IMAGE_CACHE = 16*1024*1024
PARAMETERS = ('/camera_read_diagnostics', '/camera', '/handeye',
              '/unknown_perception', '/robot_description')
ARM = tuple('Joint%d'%i for i in range(1, 7))+('right_finger',)


def image_contract(message, stream):
    if stream not in ('color', 'depth'):
        raise ValueError('invalid stream')
    stamp = message.header.stamp.to_nsec()
    if type(stamp) is not int or stamp <= 0 or not message.header.frame_id:
        raise ValueError('invalid image source')
    channels, encoding = (3, 'bgr8') if stream == 'color' else (2, '16UC1')
    if (message.encoding != encoding or not 0 < message.width <= 1920
            or not 0 < message.height <= 1080 or message.step < channels*message.width
            or len(message.data) != message.step*message.height or len(message.data) > MAX_IMAGE):
        raise ValueError('invalid or oversized published image')


def choose_pair(cache, now, last_stamp, last_arrival, interval):
    """Quality-blind chronological sampling; never nearest-stamp substitution."""
    with cache.lock:
        cache._prune(now)
        for stamp in sorted(set(cache.rows['color']) & set(cache.rows['depth'])):
            rows = tuple(dict(cache.rows[s][stamp]) for s in ('color', 'depth'))
            arrival = max(row['arrival_monotonic'] for row in rows)
            if stamp <= last_stamp or arrival-last_arrival < interval or now-arrival < .25:
                continue
            a, b = (row['message'] for row in rows)
            if (a.header.frame_id != b.header.frame_id or a.width != b.width or a.height != b.height):
                continue
            return rows
    return None


def joint_row(message, receipt):
    names, positions = tuple(message.name), tuple(message.position)
    if (message.header.frame_id != 'sdk_measured' or len(names) != len(ARM)
            or len(positions) != len(ARM) or set(names) != set(ARM)
            or message.header.stamp.to_nsec() <= 0 or not all(math.isfinite(v) for v in positions)):
        raise ValueError('invalid accepted measured joint message')
    return dict(source_stamp_ns=message.header.stamp.to_nsec(), seq=message.header.seq,
                frame_id=message.header.frame_id, names=list(names), positions=list(positions),
                receipt=receipt, scope='original header/name/position; no temperature query or use')


def bracket(rows, stamp, epoch):
    ordered = sorted(rows, key=lambda row:row['source_stamp_ns'])
    before = next((r for r in reversed(ordered) if r['source_stamp_ns'] <= stamp), None)
    after = next((r for r in ordered if r['source_stamp_ns'] >= stamp), None)
    span = after['source_stamp_ns']-before['source_stamp_ns'] if before and after else None
    available = bool(before and after and epoch is not None
                     and before['source_stamp_ns'] >= epoch and span <= 200_000_000)
    delta = None
    if before and after:
        lo, hi = (dict(zip(r['names'], r['positions'])) for r in (before, after))
        delta = {name: hi[name]-lo[name] for name in ARM}
    return dict(requested_stamp_ns=stamp, epoch_stamp_ns=epoch, before=before, after=after,
        span_ns=span, maximum_span_ns=200_000_000, source_bracket_available=available,
        endpoint_position_delta=delta, interpolation_applied=False,
        physical_stationarity_established=False)


def tf_audit(message, stamp, parent, child):
    tr, rot = message.transform.translation, message.transform.rotation
    vals = [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
    returned = message.header.stamp.to_nsec()
    exact = bool(returned == stamp and message.header.frame_id == parent
        and message.child_frame_id == child and all(math.isfinite(v) for v in vals)
        and abs(sum(v*v for v in vals[3:])-1.) <= 1e-6)
    return dict(requested_stamp_ns=stamp, returned_stamp_ns=returned, delta_ns=returned-stamp,
        parent=message.header.frame_id, child=message.child_frame_id, exact_valid=exact,
        translation=vals[:3], quaternion_xyzw=vals[3:], latest_time_substitution=False)


def read_parameter(proxy, caller, parameter, receipt):
    result = dict(parameter=parameter, started=receipt(), receipt_semantics='RPC interval; not exposure')
    try:
        code, message, value = proxy.getParam(caller, parameter)
        result.update(original_rpc_code=code, original_rpc_message=message)
        if code == 1:
            result['original_parameter_value'] = value
            if parameter == '/camera_read_diagnostics':
                result['annotation'] = meta.parameter_annotation(value)
        else:
            result['error'] = 'parameter_unavailable_or_rejected'
    except Exception as exc:
        result['error'] = type(exc).__name__+': '+str(exc)
    result['finished'] = receipt()
    return result


def diagnostic_rows(record):
    if record.get('parameter') != '/camera_read_diagnostics':
        return []
    if record.get('annotation', {}).get('schema_valid') is not True:
        return []
    return json.loads(record['original_parameter_value']['payload_json'])['entries']


class Recorder:
    def __init__(self, output, seconds, pairs, rospy, buffer, args):
        self.output, self.seconds, self.maximum_pairs = output, seconds, pairs
        self.rospy, self.buffer, self.args = rospy, buffer, args
        self.cache = base.ExactPairCache(seconds=1.5, maximum_bytes=MAX_IMAGE_CACHE)
        self.inbox = meta.BoundedInbox(maximum_items=64, maximum_bytes=8*1024*1024)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.feedback = deque(maxlen=256)
        self.epochs = deque(maxlen=32)
        self.counts = Counter()
        self.saved = []
        self.sources = {}
        self.parameter_records = []
        self.subs = []
        self.started = self.deadline = None
        self.archive = None

    def receipt(self):
        return dict(monotonic=time.monotonic(), ros_ns=self.rospy.Time.now().to_nsec(), wall_ns=time.time_ns())

    def active(self):
        return self.deadline is not None and time.monotonic() < self.deadline and not self.stop.is_set()

    def image(self, message, stream):
        if not self.active():
            return
        try:
            image_contract(message, stream)
            now = self.receipt()
            self.cache.add(stream, message, now['ros_ns'], now['monotonic'])
        except Exception:
            self.counts['image_contract_rejected'] += 1

    def joints(self, message):
        if not self.active():
            return
        try:
            row = joint_row(message, self.receipt())
            with self.lock:
                self.feedback.append(row)
        except Exception:
            self.counts['accepted_feedback_contract_rejected'] += 1

    def epoch(self, message):
        if self.active() and message.stamp.to_nsec() > 0:
            with self.lock:
                self.epochs.append(dict(source_stamp_ns=message.stamp.to_nsec(), receipt=self.receipt()))

    def poll(self):
        proxy = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
            transport=meta.TimeoutTransport(), allow_none=True)
        sequence = 0
        try:
            while self.active():
                started = time.monotonic()
                # Exact read-history first; rotate current geometry/config independently.
                for parameter in ('/camera_read_diagnostics', PARAMETERS[1+sequence%4]):
                    if not self.active():
                        break
                    row = read_parameter(proxy, self.rospy.get_name(), parameter, self.receipt)
                    raw = base.json_bytes(row)
                    if self.active():
                        self.inbox.put(row, len(raw))
                sequence += 1
                self.stop.wait(max(0., .5-(time.monotonic()-started)))
        except Exception as exc:
            self.inbox.put(dict(error='metadata_worker: '+repr(exc)), 2048)
        finally:
            proxy('close')()

    def save_metadata(self):
        while True:
            row = self.inbox.pop()
            if row is None:
                break
            name = 'metadata/%04d.json'%len(self.parameter_records)
            raw = base.json_bytes(row)
            if not self.archive.put_many([(name, raw)]):
                self.counts['metadata_budget_rejected'] += 1
                continue
            self.parameter_records.append(dict(entry=name, parameter=row.get('parameter'), sha256=base.sha(raw)))
            if row.get('error') or (row.get('parameter') == '/camera_read_diagnostics'
                    and row.get('annotation', {}).get('schema_valid') is not True):
                self.counts['metadata_missing_or_invalid'] += 1
            for source in diagnostic_rows(row):
                stamp = source['source_stamp_ns']
                ref = dict(entry=name, source_sha256=base.sha(base.json_bytes(source)),
                    geometry_generation=source['geometry_generation'], frame_id=source.get('frame_id'),
                    geometry_activation_stamp_ns=source.get('geometry_activation_stamp_ns'))
                if stamp not in self.sources and len(self.sources) >= 2048:
                    self.counts['source_index_budget_rejected'] += 1
                    continue
                known = self.sources.setdefault(stamp, [])
                if not any(item['source_sha256'] == ref['source_sha256'] for item in known):
                    known.append(ref)

    def save_pair(self, rows):
        stamp = rows[0]['source_stamp_ns']
        index = len(self.saved)
        entries, images = [], []
        for stream, row in zip(('color', 'depth'), rows):
            data, serialization = base.serialize(row['message'])
            name = 'pairs/%02d_%s.rosmsg.zlib'%(index, stream)
            entries.append((name, data))
            images.append(dict(stream=stream, entry=name, **serialization,
                **{key:value for key,value in row.items() if key != 'message'}))
        with self.lock:
            epoch = self.epochs[-1]['source_stamp_ns'] if self.epochs else None
            feedback = bracket(list(self.feedback), stamp, epoch)
        at = dict(feedback=feedback, tf_error=None, tf=None)
        try:
            request = self.rospy.Time(stamp//10**9, stamp%10**9)
            # Non-blocking exact lookup. Missing evidence is recorded, not substituted.
            transform = self.buffer.lookup_transform(self.args.parent_frame, self.args.camera_frame,
                                                     request, self.rospy.Duration(0))
            at['tf'] = tf_audit(transform, stamp, self.args.parent_frame, self.args.camera_frame)
            data, serialization = base.serialize(transform)
            name = 'pairs/%02d_lookup_transform.rosmsg.zlib'%index
            entries.append((name, data))
            at['tf_serialization'] = dict(entry=name, **serialization)
        except Exception as exc:
            at['tf_error'] = type(exc).__name__+': '+str(exc)
        row = dict(index=index, source_stamp_ns=stamp, images=images, at_source=at,
            diagnostic_only=True, target_identity=None, motion_authorized=False,
            image_domain='unaltered actually-published rectified ROS messages; not SDK raw',
            source_clock='image header host-read-completion; not calibrated SDK exposure clock')
        name = 'pairs/%02d.json'%index
        entries.append((name, base.json_bytes(row)))
        if not self.archive.put_many(entries, category='images'):
            self.counts['pair_budget_rejected'] += 1
            return False
        self.saved.append(dict(index=index, entry=name, source_stamp_ns=stamp,
            frame_id=rows[0]['frame_id'], exact_tf_valid=bool(at['tf'] and at['tf']['exact_valid']),
            feedback_bracket_available=feedback['source_bracket_available']))
        return True

    def run(self):
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Header
        import tf2_ros
        reason, failure, thread, listener = 'duration_limit', None, None, None
        ready = False
        self.started = time.monotonic()
        self.deadline = self.started+self.seconds
        try:
            self.archive = base.BoundedArchive(self.output/'diagnostic.zip',
                maximum_bytes=MAX_ARCHIVE, status_reserve=4*1024*1024)
            listener = tf2_ros.TransformListener(self.buffer)
            for topic, datatype, callback, arg, limit in (
                (self.args.color_topic, Image, self.image, 'color', 2),
                (self.args.depth_topic, Image, self.image, 'depth', 2),
                ('/alicia_d/accepted_joint_states', JointState, self.joints, None, 64),
                ('/alicia_d/control_reference_epoch', Header, self.epoch, None, 8)):
                self.subs.append(self.rospy.Subscriber(topic, datatype, callback, callback_args=arg,
                    queue_size=limit, buff_size=8*1024*1024 if arg else 65536, tcp_nodelay=True))
            thread = threading.Thread(target=self.poll, name='current_support_readonly_rpc', daemon=True)
            thread.start()
            manifest = dict(schema=1, diagnostic_only=True, target_identity=None, motion_authorized=False,
                hardware_commands_sent=False, selection_read=False, selection_changed=False,
                maximum_seconds=self.seconds, maximum_pairs=self.maximum_pairs,
                maximum_directory_bytes=MAX_DIRECTORY, maximum_image_cache_bytes=MAX_IMAGE_CACHE,
                sampling_interval_sec=1., feedback_fields='header/name/position only',
                capture_start_monotonic=self.started, capture_deadline_monotonic=self.deadline,
                ready_monotonic=time.monotonic(), user_context_id=self.args.confirmation_id,
                user_context_scope='stationary scene acknowledgement only; no target authority',
                source_sha256=base.sha(Path(__file__).read_bytes()), dependency_sha256=FROZEN,
                parameters_read=list(PARAMETERS), color_topic=self.args.color_topic,
                depth_topic=self.args.depth_topic, parent_frame=self.args.parent_frame,
                camera_frame=self.args.camera_frame, capture_timer_includes_subscriber_setup=True)
            base.atomic_json(self.output/'ready.json', manifest)
            ready = True
            print('READY '+str(self.output), flush=True)
            last_stamp, last_arrival = 0, -math.inf
            while self.active():
                if self.rospy.is_shutdown():
                    reason = 'ros_shutdown'
                    break
                self.save_metadata()
                if len(self.saved) < self.maximum_pairs:
                    rows = choose_pair(self.cache, time.monotonic(), last_stamp, last_arrival, 1.)
                    if rows is not None:
                        last_stamp = rows[0]['source_stamp_ns']
                        last_arrival = max(r['arrival_monotonic'] for r in rows)
                        if not self.save_pair(rows):
                            reason = 'pair_archive_budget'
                            break
                self.stop.wait(.01)
        except BaseException as exc:
            failure, reason = type(exc).__name__+': '+str(exc), 'recorder_exception'
        finally:
            finished = time.monotonic()
            self.stop.set()
            self.inbox.close()
            for sub in self.subs:
                sub.unregister()
            if listener is not None:
                listener.unregister()
            if thread is not None:
                thread.join(meta.RPC_TIMEOUT_SEC+.25)
            if self.archive is not None:
                try:
                    self.save_metadata()
                    with self.lock:
                        context = dict(epochs=list(self.epochs), last_accepted_feedback=list(self.feedback))
                    if not self.archive.put_many([('final_feedback_context.json', base.json_bytes(context))]):
                        self.counts['final_feedback_budget_rejected'] += 1
                except Exception as exc:
                    failure = failure or repr(exc)
                self.archive.close()
            for pair in self.saved:
                matches = self.sources.get(pair['source_stamp_ns'], [])
                pair['exact_read_history_references'] = matches
                pair['exact_read_history_unique'] = len(matches) == 1 and matches[0]['frame_id'] == pair['frame_id']
            archive_path = self.output/'diagnostic.zip'
            archive_hash = None
            if archive_path.exists():
                with archive_path.open('rb') as handle:
                    checksum = hashlib.sha256()
                    for data in iter(lambda:handle.read(1024*1024), b''):
                        checksum.update(data)
                archive_hash = checksum.hexdigest()
            actual_bytes = sum(p.stat().st_size for p in self.output.rglob('*') if p.is_file())
            summary = dict(schema=1, sealed=self.archive is not None and archive_hash is not None,
                ready=ready, reason=reason, error=failure, diagnostic_only=True, motion_authorized=False,
                target_identity=None, target_or_support_accepted=False, hardware_commands_sent=False,
                source_sha256=base.sha(Path(__file__).read_bytes()), dependencies=FROZEN,
                capture_start_monotonic=self.started, capture_finished_monotonic=finished,
                capture_duration_sec=finished-self.started, duration_limit_sec=self.seconds,
                pair_count=len(self.saved), requested_pair_count=self.maximum_pairs, pairs=self.saved,
                counts=dict(self.counts), image_cache_counts=dict(self.cache.counts), inbox=self.inbox.snapshot(),
                parameter_records=self.parameter_records, rpc_thread_alive=bool(thread and thread.is_alive()),
                archive_sha256=archive_hash, archive_entries=(self.archive.entries if self.archive else []),
                bytes_before_summary=actual_bytes, maximum_directory_bytes=MAX_DIRECTORY,
                measurement_quality_not_evaluated=True,
                limitations=['No old target reference, generation or alignment is reused.',
                    'Source is host read completion, not hardware exposure; TF exactness does not calibrate clocks.',
                    'Pair sampling is fixed cadence, independent of visual/depth quality; unsampled frames are not retained.',
                    'Metadata raw JSON values/strings retained, not XMLRPC wire bytes; actual RPC read intervals are retained.',
                    'Missing TF, source metadata or measured feedback remains an explicit gap.',
                    'No support plane acceptance or physical stationarity follows from successful file capture.'])
            encoded = base.json_bytes(summary)
            if len(encoded) > 512*1024 or actual_bytes+len(encoded)+2048 > MAX_DIRECTORY:
                raise RuntimeError('summary reserve invariant failed')
            base.atomic_json(self.output/'summary.json', summary)
            (self.output/'SHA256SUMS').write_text('%s  diagnostic.zip\n%s  summary.json\n'%(
                archive_hash or 'MISSING', base.sha(encoded)), encoding='utf8')
            print(json.dumps(dict(output=str(self.output), pairs=len(self.saved), error=failure)), flush=True)
        return 0 if ready and failure is None and len(self.saved) >= 5 else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confirmation-id', required=True)
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--pairs', type=int, default=10)
    parser.add_argument('--color-topic', default='/supervisor/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/supervisor/camera/depth/image_raw')
    parser.add_argument('--parent-frame', default='base_link')
    parser.add_argument('--camera-frame', default='camera_link')
    args = parser.parse_args()
    if not 0 < args.seconds <= 15 or not 5 <= args.pairs <= 10:
        raise ValueError('requires 5-10 pairs, <=15 seconds')
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand not mounted; no fallback')
    output = args.output.resolve()
    if EXTERNAL.resolve() not in output.parents or args.output.is_symlink():
        raise ValueError('new output must be inside SanStand experiment root')
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    logging_config = output/'logging.conf'
    logging_config.write_text('''[loggers]
keys=root
[handlers]
keys=fileHandler
[formatters]
keys=defaultFormatter
[logger_root]
level=WARNING
handlers=fileHandler
[handler_fileHandler]
class=handlers.RotatingFileHandler
level=WARNING
formatter=defaultFormatter
args=(os.environ['ROS_LOG_FILENAME'],'a', 131072, 1)
[formatter_defaultFormatter]
format=[%(name)s][%(levelname)s] %(asctime)s: %(message)s
''', encoding='utf8')
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = str(logging_config)
    import rospy
    import tf2_ros
    rospy.init_node('current_support_readonly_recorder', anonymous=True, disable_signals=True)
    recorder = Recorder(output, args.seconds, args.pairs, rospy,
        tf2_ros.Buffer(cache_time=rospy.Duration(3.)), args)
    return recorder.run()


if __name__ == '__main__':
    raise SystemExit(main())
