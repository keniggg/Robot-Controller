#!/usr/bin/env python3
"""Bounded diagnostic recorder. Original messages only; no motion authority.

This independent ROS subscriber does not decode, modify, publish or replay
images. A 2-second receipt-time cache matches exact integer source stamps.
Only status-referenced image pairs are persisted. Parameter snapshots retain
their actual read intervals; they are not claimed to be exposure-synchronous.
"""
import argparse
from collections import Counter, OrderedDict, deque
import hashlib
import io
import json
import os
from pathlib import Path
import threading
import time
import zipfile
import zlib

EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
PARAMETERS = ('/camera', '/handeye', '/unknown_perception',
              '/robot_description', '/grasp_mode/selection', '/camera_read_diagnostics')
MAX_ARCHIVE_BYTES = 85_000_000
READY_IMAGE_BYTES = 32_000_000
RESERVED_STATUS_BYTES = 5_000_000
MAX_STATUS_BYTES = 2_000_000
MAX_PENDING_STATUSES = 128
MAX_DIRECTORY_BYTES = 90*1024*1024


def sha(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + '\n').encode('utf8')


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_bytes(json_bytes(value))
    os.replace(str(temporary), str(path))


def serialize(message):
    buffer = io.BytesIO()
    message.serialize(buffer)
    raw = buffer.getvalue()
    return zlib.compress(raw, 3), dict(message_type=message._type,
        md5sum=message._md5sum, serialized_sha256=sha(raw), serialized_bytes=len(raw),
        encoding='unaltered_ROS_serialization_then_zlib_level3')


class ExactPairCache:
    """The two arrival streams share only a short bookkeeping lock."""
    def __init__(self, seconds=2., maximum_bytes=96_000_000):
        self.seconds = seconds
        self.maximum_bytes = maximum_bytes
        self.lock = threading.Lock()
        self.rows = {'color': OrderedDict(), 'depth': OrderedDict()}
        self.bytes = 0
        self.counts = Counter()
        self.arrivals = deque(maxlen=2048)

    def _prune(self, now):
        for stream in self.rows.values():
            while stream and now-next(iter(stream.values()))['arrival_monotonic'] > self.seconds:
                _, old = stream.popitem(last=False)
                self.bytes -= old['pixel_bytes']
                self.counts['expired_messages'] += 1
        while self.bytes > self.maximum_bytes:
            available = [(next(iter(rows.values()))['arrival_monotonic'], name)
                         for name, rows in self.rows.items() if rows]
            _, name = min(available)
            _, old = self.rows[name].popitem(last=False)
            self.bytes -= old['pixel_bytes']
            self.counts['memory_bound_evictions'] += 1

    def add(self, stream, message, receipt_ns, now):
        stamp = message.header.stamp.to_nsec()
        row = dict(message=message, source_stamp_ns=stamp, source_seq=int(message.header.seq),
            frame_id=message.header.frame_id, receipt_ros_ns=receipt_ns,
            arrival_monotonic=now, pixel_bytes=len(message.data))
        with self.lock:
            old = self.rows[stream].pop(stamp, None)
            if old is not None:
                self.bytes -= old['pixel_bytes']
                self.counts['duplicate_source_messages'] += 1
            self.rows[stream][stamp] = row
            self.bytes += row['pixel_bytes']
            self.counts[stream+'_arrivals'] += 1
            if len(self.arrivals) == self.arrivals.maxlen:
                self.counts['arrival_header_evictions'] += 1
            self.arrivals.append(dict(stream=stream, **{k:v for k,v in row.items() if k != 'message'}))
            self._prune(now)

    def exact(self, stamp, now):
        with self.lock:
            self._prune(now)
            rows = [self.rows[stream].get(stamp) for stream in ('color', 'depth')]
            if any(row is None for row in rows):
                return None
            return tuple(dict(row) for row in rows)


class BoundedArchive:
    """One archive avoids per-file allocation amplification on the external disk."""
    def __init__(self, path, maximum_bytes=MAX_ARCHIVE_BYTES,
                 ready_bytes=READY_IMAGE_BYTES, status_reserve=RESERVED_STATUS_BYTES):
        self.archive = zipfile.ZipFile(str(path), 'x', compression=zipfile.ZIP_STORED)
        self.maximum_bytes = maximum_bytes
        self.ready_limit = ready_bytes
        self.status_reserve = status_reserve
        self.conservative_bytes = 22
        self.ready_bytes = 0
        self.entries = []

    def put_many(self, entries, category='status'):
        # Account for both ZIP headers, UTF-8 names, and more than the maximum
        # normal per-entry overhead. All entries of a pair are admitted together.
        cost = sum(len(data)+4*len(name.encode('utf8'))+512 for name, data in entries)
        limit = self.maximum_bytes - (self.status_reserve if category != 'status' else 0)
        if self.conservative_bytes+cost > limit:
            return False
        if category == 'ready' and self.ready_bytes+cost > self.ready_limit:
            return False
        for name, data in entries:
            self.archive.writestr(name, data)
            self.entries.append(dict(name=name, bytes=len(data), sha256=sha(data)))
        self.conservative_bytes += cost
        if category == 'ready':
            self.ready_bytes += cost
        return True

    def close(self):
        self.archive.close()


def referenced_stamps(status):
    """No closest/newest-frame substitution when a status has no source."""
    result = []
    value = status.get('source_stamp_ns')
    if type(value) is int and value > 0:
        result.append(('source_stamp_ns', value))
    loss = status.get('first_loss')
    if isinstance(loss, dict):
        for key in ('source_stamp_ns', 'last_good_source_stamp_ns', 'last_consumed_source_stamp_ns'):
            value = loss.get(key)
            if type(value) is int and value > 0:
                result.append(('first_loss.'+key, value))
    return result


def read_metadata_parameters(rospy):
    """Record optional namespaces explicitly without hiding RPC errors."""
    values, errors, absent_defaults = {}, {}, {}
    for key in PARAMETERS:
        try:
            values[key] = rospy.get_param(key)
        except KeyError as exc:
            if key in ('/unknown_perception', '/camera_read_diagnostics'):
                values[key] = {}
                absent_defaults[key] = dict(parameter_present=False, effective_default={},
                    semantics=("producer get_param('/unknown_perception', {}) followed by Config defaults"
                        if key == '/unknown_perception' else
                        'optional diagnostic unavailable; no substitution or execution authority'))
            else:
                errors[key] = repr(exc)
        except Exception as exc:
            errors[key] = repr(exc)
    return values, errors, absent_defaults


class Recorder:
    def __init__(self, output, duration, rospy):
        self.output, self.duration, self.rospy = output, duration, rospy
        self.started = time.monotonic()
        self.total_deadline = self.started+20.
        self.startup_deadline = self.started+5.
        self.deadline = self.total_deadline
        self.ready = None
        self.initial_generation = None
        self.initial_selection_ns = None
        self.acquired = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.pairs = ExactPairCache()
        self.statuses = deque()
        self.transforms = deque()
        self.static = deque(maxlen=128)
        self.metadata = deque(maxlen=20)
        self.written_metadata = set()
        self.written_transforms = set()
        self.transform_keys = {}
        self.pending = []
        self.saved_pairs = {}
        self.counts = Counter()
        self.archive = BoundedArchive(output/'diagnostic.zip')
        self.subscribers = []

    def active(self):
        return not self.stop.is_set() and time.monotonic() < self.deadline

    def image(self, message, stream):
        if self.active():
            self.pairs.add(stream, message, self.rospy.Time.now().to_nsec(), time.monotonic())

    def status(self, message):
        if not self.active():
            return
        received, now = self.rospy.Time.now().to_nsec(), time.monotonic()
        # Keep the exact unparsed payload. Parsing and disk writes are never
        # performed by the image callback or its bookkeeping lock.
        with self.lock:
            if len(message.data) > MAX_STATUS_BYTES:
                self.counts['oversize_status_not_saved'] += 1
            elif len(self.statuses) >= MAX_PENDING_STATUSES:
                self.counts['status_queue_full_not_saved'] += 1
            else:
                self.statuses.append((message.data, received, now))

    def tf(self, message, topic):
        if not self.active():
            return
        row = (self.rospy.Time.now().to_nsec(), time.monotonic(), message)
        with self.lock:
            if topic == '/tf_static':
                # Keep every original message: one authority may publish
                # partial updates rather than repeat its complete static set.
                if len(self.static) == self.static.maxlen:
                    self.counts['tf_static_memory_bound_evictions'] += 1
                self.static.append(row)
            else:
                self.transforms.append(row)
                while self.transforms and row[1]-self.transforms[0][1] > 2.:
                    self.transforms.popleft()
                while len(self.transforms) > 512:
                    self.transforms.popleft()
                    self.counts['tf_memory_bound_evictions'] += 1

    def metadata_worker(self):
        sequence = 0
        while self.active():
            start_ns = self.rospy.Time.now().to_nsec()
            values, errors, absent_defaults = read_metadata_parameters(self.rospy)
            end_ns = self.rospy.Time.now().to_nsec()
            row = dict(sequence=sequence, read_start_ros_ns=start_ns, read_end_ros_ns=end_ns,
                parameters=values, errors=errors, absent_optional_parameter_defaults=absent_defaults,
                image_time_association='parameter_read_interval_only_not_exposure_synchronous')
            with self.lock:
                self.metadata.append(row)
            sequence += 1
            self.stop.wait(min(.5, max(0., self.deadline-time.monotonic())))

    def context(self):
        with self.lock:
            return (list(self.transforms), list(self.static), list(self.metadata))

    def persist_context(self, context):
        transform_ids, metadata_ids, missing_transform_ids, missing_metadata_ids = [], [], [], []
        for topic, rows in (('/tf', context[0]), ('/tf_static', context[1])):
            for receipt, arrived, message in rows:
                receipt_key = (topic, receipt, id(message))
                if receipt_key in self.transform_keys:
                    transform_ids.append(self.transform_keys[receipt_key])
                    continue
                if time.monotonic() >= self.total_deadline-.2:
                    missing_transform_ids.append(dict(topic=topic, receipt_ros_ns=receipt,
                        reason='finalization_time_budget'))
                    continue
                data, info = serialize(message)
                key = '%s_%d_%s' % (topic.strip('/'), receipt, info['serialized_sha256'])
                if key in self.written_transforms:
                    transform_ids.append(key)
                    self.transform_keys[receipt_key] = key
                    continue
                info.update(topic=topic, receipt_ros_ns=receipt, receipt_monotonic=arrived,
                    original_connection_header=getattr(message, '_connection_header', {}))
                if self.archive.put_many([('tf/'+key+'.rosmsg.zlib', data),
                        ('tf/'+key+'.json', json_bytes(info))]):
                    self.written_transforms.add(key)
                    self.transform_keys[receipt_key] = key
                    transform_ids.append(key)
                else:
                    missing_transform_ids.append(dict(key=key, reason='archive_byte_budget'))
                    self.counts['tf_budget_not_saved'] += 1
        for row in context[2]:
            key = str(row['sequence'])
            if key not in self.written_metadata:
                if self.archive.put_many([('metadata/'+key+'.json', json_bytes(row))]):
                    self.written_metadata.add(key)
                else:
                    missing_metadata_ids.append(key)
                    self.counts['metadata_budget_not_saved'] += 1
            if key in self.written_metadata:
                metadata_ids.append(key)
        return dict(tf_record_keys=transform_ids, metadata_record_keys=metadata_ids,
            missing_tf_records=missing_transform_ids, missing_metadata_records=missing_metadata_ids,
            tf_window='original_messages_retained_for_last_2_seconds_at_status_dequeue',
            tf_completeness_or_exact_transform_lookup_proven=False)

    def process_status(self, item):
        raw, receipt, arrived = item
        index = self.counts['status_processed']
        self.counts['status_processed'] += 1
        if len(raw.encode('utf8')) > MAX_STATUS_BYTES:
            self.counts['oversize_status_not_saved'] += 1
            return
        try:
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError('status_not_object')
            references = referenced_stamps(value)
            priority = bool(value.get('first_loss') or value.get('state') != 'ready'
                or 'pending' in str(value.get('detail', '')).lower())
            parsed_error = None
        except (ValueError, TypeError) as exc:
            references, priority, parsed_error = [], True, str(exc)
        context = self.persist_context(self.context())
        event = dict(status_index=index, topic='/perception/unknown/detector_status',
            original_status_data=raw, original_status_utf8_sha256=sha(raw.encode('utf8')),
            receipt_ros_ns=receipt, receipt_monotonic=arrived, parse_error=parsed_error,
            requested_image_references=[dict(field=k, source_stamp_ns=v) for k,v in references],
            priority_evidence=priority, context=context, diagnostic_only=True, motion_authorized=False)
        if not self.archive.put_many([('status/%06d.json'%index, json_bytes(event))]):
            self.counts['status_budget_not_saved'] += 1
            return
        for field, stamp in references:
            self.pending.append(dict(status_index=index, field=field, stamp=stamp,
                priority=priority, status_receipt_monotonic=arrived,
                acquisition=(dict(generation=value['generation'],
                    selection_stamp_ns=value['selection_stamp_ns'], source_stamp_ns=stamp,
                    status_receipt_ros_ns=receipt, status_receipt_monotonic=arrived,
                    original_status_utf8_sha256=event['original_status_utf8_sha256'])
                    if self.ready is not None and parsed_error is None
                    and value.get('state') == 'ready' and field == 'source_stamp_ns'
                    and type(value.get('generation')) is int
                    and value['generation'] > self.initial_generation
                    and type(value.get('selection_stamp_ns')) is int
                    and value['selection_stamp_ns'] > self.initial_selection_ns
                    and stamp > value['selection_stamp_ns']
                    and arrived >= self.ready['ready_monotonic'] else None),
                context=context, expires=min(self.deadline, arrived+2.)))

    def process_pending(self, final=False):
        remaining = []
        work_deadline = self.total_deadline-.2 if final else self.deadline
        for item in sorted(self.pending, key=lambda row: not row['priority']):
            if time.monotonic() >= work_deadline:
                remaining.append(item)
                continue
            stamp = item['stamp']
            pair = self.pairs.exact(stamp, time.monotonic())
            outcome = self.saved_pairs.get(stamp)
            if outcome is None and pair is not None:
                entries, info = [], dict(source_stamp_ns=stamp,
                    exact_source_stamp_match=True, original_pixels_modified=False,
                    diagnostic_only=True, motion_authorized=False, context=item['context'], streams={})
                for stream, row in zip(('color', 'depth'), pair):
                    data, evidence = serialize(row['message'])
                    evidence.update({key:value for key,value in row.items() if key != 'message'})
                    info['streams'][stream] = evidence
                    entries.append(('pairs/%d_%s.rosmsg.zlib'%(stamp, stream), data))
                entries.append(('pairs/%d.json'%stamp, json_bytes(info)))
                category = 'priority' if item['priority'] else 'ready'
                if self.archive.put_many(entries, category):
                    outcome = 'saved_exact_pair'
                    self.saved_pairs[stamp] = outcome
                    self.counts['pairs_saved'] += 1
                else:
                    outcome = 'pair_budget_not_saved'
                    self.counts[outcome] += 1
            if outcome is None and not final and time.monotonic() < item['expires']:
                remaining.append(item)
                continue
            outcome = outcome or 'exact_pair_unavailable_in_original_2_second_cache'
            if (outcome == 'saved_exact_pair' and self.acquired is None
                    and item.get('acquisition') is not None):
                self.acquired = dict(item['acquisition'], status_index=item['status_index'],
                    recorded_wall_ns=time.time_ns(), recorded_monotonic=time.monotonic(),
                    exact_pair_persisted=True, diagnostic_only=True, motion_authorized=False)
                atomic_json(self.output/'acquired.json', self.acquired)
            record = dict(status_index=item['status_index'], reference_field=item['field'],
                requested_source_stamp_ns=stamp, result=outcome,
                approximate_or_relabelled_frame_used=False)
            self.archive.put_many([('association/%06d_%s.json'%(item['status_index'], item['field']),
                json_bytes(record))])
            self.counts['association_'+outcome] += 1
        self.pending = remaining

    def run(self):
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        worker = threading.Thread(target=self.metadata_worker, daemon=True)
        worker.start()
        # Parameter RPCs are isolated from image callbacks and the bounded
        # main loop. A stalled RPC cannot renew the startup/capture deadlines.
        first_metadata = None
        while time.monotonic() < self.startup_deadline:
            with self.lock:
                first_metadata = self.metadata[0] if self.metadata else None
            if first_metadata is not None:
                break
            self.stop.wait(.01)
        if first_metadata is None or first_metadata['errors'] or set(first_metadata['parameters']) != set(PARAMETERS):
            self.stop.set()
            self.archive.close()
            reason = ('metadata_read_deadline_exceeded' if first_metadata is None else
                      'metadata_parameter_error' if first_metadata['errors'] else 'metadata_incomplete')
            atomic_json(self.output/'startup_failure.json', dict(reason=reason,
                startup_limit_sec=5., observed_monotonic=time.monotonic(), diagnostic_only=True,
                metadata_received=first_metadata is not None,
                metadata_errors=None if first_metadata is None else first_metadata['errors'],
                parameter_keys_received=[] if first_metadata is None else sorted(first_metadata['parameters']),
                parameter_read_start_ros_ns=None if first_metadata is None else first_metadata['read_start_ros_ns'],
                parameter_read_end_ros_ns=None if first_metadata is None else first_metadata['read_end_ros_ns']))
            raise RuntimeError(reason+': '+repr(None if first_metadata is None else first_metadata['errors']))
        camera = first_metadata['parameters']['/camera']
        selection = first_metadata['parameters']['/grasp_mode/selection']
        self.initial_generation = int(selection['generation'])
        self.initial_selection_ns = int(selection['stamp_ns'])
        topics = dict(color=camera.get('color_topic', '/supervisor/camera/color/image_raw'),
            depth=camera.get('depth_topic', '/supervisor/camera/depth/image_raw'))
        for stream, topic in topics.items():
            self.subscribers.append(self.rospy.Subscriber(topic, Image, self.image,
                callback_args=stream, queue_size=2, buff_size=2**24))
        self.subscribers.append(self.rospy.Subscriber('/perception/unknown/detector_status',
            String, self.status, queue_size=64, buff_size=2**22))
        for topic in ('/tf', '/tf_static'):
            self.subscribers.append(self.rospy.Subscriber(topic, TFMessage, self.tf,
                callback_args=topic, queue_size=128))
        two_streams = False
        while time.monotonic() < self.startup_deadline:
            with self.pairs.lock:
                two_streams = bool(self.pairs.rows['color'] and self.pairs.rows['depth'])
            if two_streams:
                break
            self.stop.wait(.01)
        if not two_streams:
            self.stop.set()
            for sub in self.subscribers:
                sub.unregister()
            self.archive.close()
            atomic_json(self.output/'startup_failure.json', dict(reason='both_image_streams_unavailable',
                startup_limit_sec=5., observed_monotonic=time.monotonic(), diagnostic_only=True))
            raise RuntimeError('both image streams unavailable within startup limit')
        ready_monotonic = time.monotonic()
        # Usually readiness takes under a second. Retain a finalization reserve
        # inside the 20-second total window even if startup uses nearly 5 s.
        self.deadline = min(ready_monotonic+self.duration, self.total_deadline-.5)
        ready = dict(diagnostic_only=True, motion_authorized=False, output=str(self.output),
            capture_duration_limit_sec=self.duration, started_monotonic=self.started,
            ready_monotonic=ready_monotonic, ready_wall_ns=time.time_ns(),
            ready_ros_ns=self.rospy.Time.now().to_nsec(),
            initial_generation=self.initial_generation, initial_selection_stamp_ns=self.initial_selection_ns,
            both_image_streams_received=True, complete_metadata_received=True,
            startup_limit_sec=5., total_limit_sec=20., total_deadline_monotonic=self.total_deadline,
            deadline_monotonic=self.deadline, topics=topics, archive_byte_limit=MAX_ARCHIVE_BYTES,
            exact_pair_cache_seconds=2., parameters_whitelist=list(PARAMETERS))
        self.ready = ready
        atomic_json(self.output/'ready.json', ready)
        print('READY '+json.dumps(ready, ensure_ascii=False), flush=True)
        error = None
        next_context_write = time.monotonic()
        try:
            while self.active() and not self.rospy.is_shutdown():
                with self.lock:
                    items = list(self.statuses)
                    self.statuses.clear()
                for offset, item in enumerate(items):
                    if not self.active():
                        with self.lock:
                            self.statuses.extendleft(reversed(items[offset:]))
                        break
                    self.process_status(item)
                self.process_pending()
                if time.monotonic() >= next_context_write and self.active():
                    self.persist_context(self.context())
                    next_context_write = time.monotonic()+.25
                self.stop.wait(min(.01, max(0., self.deadline-time.monotonic())))
        except BaseException as exc:
            error = repr(exc)
            raise
        finally:
            self.stop.set()
            for sub in self.subscribers:
                sub.unregister()
            # Drain only already admitted status records; no capture renewal.
            with self.lock:
                items = list(self.statuses)
                self.statuses.clear()
            try:
                for item in items:
                    if time.monotonic() >= self.total_deadline-.2:
                        self.counts['finalization_time_budget_status_omissions'] += 1
                    else:
                        self.process_status(item)
                self.process_pending(final=True)
                self.persist_context(self.context())
                if self.pending:
                    self.archive.put_many([('unresolved_at_capture_deadline.json', json_bytes([
                        dict(status_index=row['status_index'], reference_field=row['field'],
                            requested_source_stamp_ns=row['stamp'],
                            reason='finalization_time_budget', approximate_or_relabelled_frame_used=False)
                        for row in self.pending]))])
                    self.counts['pending_finalization_time_budget_omissions'] += len(self.pending)
                with self.pairs.lock:
                    arrivals = list(self.pairs.arrivals)
                self.archive.put_many([('raw_image_header_arrivals.json', json_bytes(arrivals))])
                self.archive.put_many([('entry_manifest.json', json_bytes(self.archive.entries))])
            except BaseException as exc:
                error = (error+'; ' if error else '')+'finalization: '+repr(exc)
            finally:
                self.archive.close()
            size = (self.output/'diagnostic.zip').stat().st_size
            directory_bytes = sum(path.stat().st_size for path in self.output.rglob('*') if path.is_file())
            summary = dict(ready, finished_monotonic=time.monotonic(),
                archive_closed=True, archive_bytes=size, counts=dict(self.counts),
                ingress_counts=dict(self.pairs.counts), saved_pair_source_stamps_ns=sorted(self.saved_pairs),
                byte_budget_passed=size<MAX_ARCHIVE_BYTES,
                total_elapsed_sec=time.monotonic()-self.started,
                cooperative_total_deadline_passed=time.monotonic() <= self.total_deadline,
                error=error, complete_without_writer_error=error is None,
                directory_bytes_before_summary=directory_bytes,
                directory_budget_limit_bytes=MAX_DIRECTORY_BYTES,
                directory_budget_passed=directory_bytes+65536 < MAX_DIRECTORY_BYTES,
                hardware_commands_sent=False,
                live_images_decoded_or_modified=False,
                limitation='Diagnostics only; metadata has read intervals, TF completeness unproven, '
                    'missing exact pairs and budget omissions are explicit. No frame authorizes motion.')
            atomic_json(self.output/'summary.json', summary)
            print(json.dumps(dict(archive_closed=True, output=str(self.output),
                archive_bytes=size, counts=dict(self.counts)), ensure_ascii=False), flush=True)
            if error:
                raise RuntimeError(error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=15.)
    args = parser.parse_args()
    if not 0. < args.seconds <= 15.:
        raise ValueError('diagnostic capture is bounded to 15 seconds')
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand unavailable; no system-disk fallback')
    output = args.output.resolve()
    if EXTERNAL.resolve() not in output.parents:
        raise ValueError('diagnostic output must be on the configured external experiment root')
    output.mkdir(parents=True, exist_ok=False)
    # All recorder ROS logs remain on SanStand and rotate within a small,
    # explicit budget; this affects only this diagnostic process.
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    logging_path = output/'python_logging.conf'
    logging_path.write_text('''[loggers]
keys=root
[handlers]
keys=fileHandler
[formatters]
keys=defaultFormatter
[logger_root]
level=INFO
handlers=fileHandler
[handler_fileHandler]
class=handlers.RotatingFileHandler
level=INFO
formatter=defaultFormatter
args=(os.environ['ROS_LOG_FILENAME'],'a', 131072, 1)
[formatter_defaultFormatter]
format=[%(name)s][%(levelname)s] %(asctime)s: %(message)s
''', encoding='utf8')
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = str(logging_path)
    import rospy
    rospy.init_node('bounded_support_gap_recorder', anonymous=True, disable_signals=True)
    Recorder(output, args.seconds, rospy).run()


if __name__ == '__main__':
    main()
