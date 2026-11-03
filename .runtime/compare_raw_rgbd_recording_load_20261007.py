#!/usr/bin/env python3
"""Passive A/B/A: raw, exact RGB-D LZ4 recording only in B. No device queries.

This is independent of the production detected-keyframe recorder. It can impose
more load: the intervention includes subscriptions, serialization, compression,
memory and disk, not disk alone. Missing sources are never manufactured.
"""
import argparse
from collections import Counter, OrderedDict, deque
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client

MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
TOTAL_BUDGET = 4 * 1024**3
SIDE_RESERVE = 64 * 1024**2
BAG_BUDGET = TOTAL_BUDGET - SIDE_RESERVE
MIN_FREE = 4831838208
DURATION = 45.0
CHILD_LIMIT = 70.0
TOPICS = {'color': '/supervisor/camera/color/image_raw',
          'depth': '/supervisor/camera/depth/image_raw'}
MAX_PAIRS = 4096
MAX_JSON_BYTES = 16 * 1024**2


class BudgetError(RuntimeError):
    pass


def atomic_json(path, value):
    data = (json.dumps(value, ensure_ascii=False, separators=(',', ':'),
                       allow_nan=False) + '\n').encode()
    if len(data) > MAX_JSON_BYTES:
        raise BudgetError('JSON sidecar exceeds reserved bound')
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('xb') as handle:
        handle.write(data)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def directory_bytes(path):
    return sum(p.stat().st_size for p in path.rglob('*') if p.is_file())


def external_directory(path, *, create):
    path = Path(path).absolute()
    if path != path.resolve() or EXT not in path.parents or not MOUNT.is_mount():
        raise RuntimeError('Canonical mounted SanStand output required; no fallback')
    if not path.parent.is_dir():
        raise RuntimeError('Output parent must already exist')
    if shutil.disk_usage(MOUNT).free < MIN_FREE:
        raise RuntimeError('At least 4.5 GiB available space is required')
    if create:
        path.mkdir()
    elif not path.is_dir():
        raise RuntimeError('Owned output directory missing')
    return path


class LimitedBagFile(io.IOBase):
    """Seekable rosbag stream with an exact file-length ceiling, including close."""
    mode = 'w+b'

    def __init__(self, path, limit=BAG_BUDGET):
        self.raw = Path(path).open('x+b')
        self.limit = int(limit)

    def writable(self): return True
    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self.raw.tell()
    def seek(self, *args): return self.raw.seek(*args)
    def read(self, *args): return self.raw.read(*args)
    def fileno(self): return self.raw.fileno()

    def write(self, data):
        if self.tell() + len(data) > self.limit:
            raise BudgetError('Raw diagnostic bag byte limit reached')
        return self.raw.write(data)

    def truncate(self, size=None):
        size = self.tell() if size is None else size
        if size > self.limit:
            raise BudgetError('Diagnostic truncate exceeds byte limit')
        return self.raw.truncate(size)

    def flush(self):
        if not self.raw.closed:
            self.raw.flush()

    def close(self):
        if self.closed:
            return
        try:
            self.flush()
            os.fsync(self.raw.fileno())
        finally:
            self.raw.close()
            super().close()


def image_header(message, received, receipt_ns):
    return dict(source_ns=int(message.header.stamp.to_nsec()),
                sequence=int(message.header.seq), frame_id=str(message.header.frame_id),
                height=int(message.height), width=int(message.width),
                encoding=str(message.encoding), step=int(message.step),
                data_bytes=len(message.data), receipt_monotonic=received,
                receipt_ros_ns=int(receipt_ns))


class ExactPairs:
    """Bounded bookkeeping only in callbacks; original message references retained."""
    def __init__(self, start, end, max_bytes=64*1024**2, max_entries=64):
        self.start, self.end = start, end
        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pending, self.ready, self.seen = OrderedDict(), deque(), set()
        self.bytes = 0
        self.counts = Counter()
        self.arrivals = {'color': [], 'depth': []}
        self.closed = False
        self.lock = threading.Lock()

    def _make_room(self, ns, size):
        """Under lock, evict only oldest incomplete sources on capacity pressure.

        Complete ready pairs remain owned by the writer. Preserve the pending
        half of the incoming exact source, so admitting its mate can make
        progress. Evictions are permanent diagnostic loss, not valid capture.
        """
        remaining_bytes, remaining_entries = self.bytes, len(self.pending)
        needs_entry = ns not in self.pending
        if (remaining_bytes + size <= self.max_bytes and
                (not needs_entry or remaining_entries < self.max_entries)):
            return True
        victims = []
        for old_ns, row in self.pending.items():
            if old_ns == ns or len(row) != 1:
                continue
            victims.append((old_ns, row))
            remaining_bytes -= sum(h['data_bytes'] for _, h in row.values())
            remaining_entries -= 1
            if (remaining_bytes + size <= self.max_bytes and
                    (not needs_entry or remaining_entries < self.max_entries)):
                break
        else:
            # The incoming message plus protected ready/exact-source data
            # cannot fit. Do not discard pending data for an impossible admit.
            return False
        for old_ns, row in victims:
            del self.pending[old_ns]
            released = sum(h['data_bytes'] for _, h in row.values())
            self.bytes -= released
            self.counts['pending_capacity_evicted_sources'] += 1
            self.counts['pending_capacity_evicted_messages'] += len(row)
            self.counts['pending_capacity_evicted_bytes'] += released
        return True

    def add(self, stream, message, now, receipt_ns):
        h = image_header(message, now, receipt_ns)
        ns, size = h['source_ns'], h['data_bytes']
        with self.lock:
            if self.closed or not self.start <= now < self.end:
                self.counts['outside_capture_window'] += 1
                return
            if ns <= 0 or size <= 0:
                self.counts['invalid_source_or_empty_image'] += 1
                return
            if len(self.arrivals[stream]) >= MAX_PAIRS * 2:
                self.counts['arrival_limit'] += 1
                return
            self.arrivals[stream].append(h)
            self.counts[stream + '_received'] += 1
            if ns in self.seen or stream in self.pending.get(ns, {}):
                self.counts['duplicate'] += 1
                return
            if not self._make_room(ns, size):
                self.counts['memory_limit'] += 1
                return
            self.pending.setdefault(ns, {})[stream] = (message, h)
            self.bytes += size
            row = self.pending[ns]
            if set(row) == set(TOPICS):
                if len(self.seen) >= MAX_PAIRS:
                    self.counts['pair_limit'] += 1
                    return
                self.ready.append((ns, self.pending.pop(ns)))
                self.seen.add(ns)
                self.counts['exact_pairs'] += 1

    def pop(self):
        with self.lock:
            if not self.ready:
                return None
            ns, row = self.ready.popleft()
            self.bytes -= sum(h['data_bytes'] for _, h in row.values())
            return ns, row

    def stop(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            # Preserve the first terminal counts when normal close is followed
            # by the unconditional finally cleanup. Ready pairs remain drainable.
            self.counts['unmatched_sources_at_end'] = len(self.pending)
            self.counts['unmatched_messages_at_end'] = sum(map(len, self.pending.values()))
            self.bytes -= sum(h['data_bytes'] for row in self.pending.values()
                              for _, h in row.values())
            self.pending.clear()

    def snapshot(self):
        with self.lock:
            return dict(counts=dict(self.counts), arrivals=self.arrivals,
                        queue_pairs_remaining=len(self.ready), retained_pixel_bytes=self.bytes)


def setup_logging(out):
    logdir = out / 'ros'
    logdir.mkdir(exist_ok=True)
    config = out / 'python_logging.conf'
    config.write_text('''[loggers]
keys=root
[handlers]
keys=file
[formatters]
keys=plain
[logger_root]
level=WARNING
handlers=file
[handler_file]
class=handlers.RotatingFileHandler
level=WARNING
formatter=plain
args=(os.path.join(os.environ['ROS_LOG_DIR'], 'passive.log'), 'a', 262144, 2)
[formatter_plain]
format=%(asctime)s %(levelname)s %(message)s
''')
    os.environ['ROS_LOG_DIR'] = str(logdir)
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = str(config)


def record_child(out):
    """Only this owned subprocess opens a bag. It never opens the camera SDK."""
    external_directory(out, create=False)
    setup_logging(out)
    import rospy
    import rosbag
    from sensor_msgs.msg import Image
    rospy.init_node('passive_raw_rgbd_load_recorder', anonymous=True,
                    disable_signals=True, disable_rosout=True)
    stop = threading.Event()
    old = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    active, closed = out/'raw_rgbd.bag.active', out/'raw_rgbd.bag'
    stream = bag = cache = None
    subs, pairs, errors = [], [], []
    started = time.monotonic()
    hard_end = started + CHILD_LIMIT - 5
    report = dict(schema=1, kind='passive_exact_raw_rgbd_load', bag_closed=False,
                  compression='lz4', stop_reason='startup_failed', motion_authorized=False,
                  maximum_bag_bytes=BAG_BUDGET, duration_requested_sec=DURATION)
    try:
        stream = LimitedBagFile(active)
        bag = rosbag.Bag(stream, 'w', compression=rosbag.Compression.LZ4)
        cache = ExactPairs(float('inf'), float('inf'))
        def callback(message, name):
            try:
                cache.add(name, message, time.monotonic(), rospy.Time.now().to_nsec())
            except Exception as exc:
                if len(errors) < 16:
                    errors.append('callback:' + repr(exc))
                stop.set()
        for name, topic in TOPICS.items():
            subs.append(rospy.Subscriber(topic, Image, callback, callback_args=name,
                                         queue_size=4, buff_size=2**24))
        start = time.monotonic()
        with cache.lock:
            cache.start, cache.end = start, start + DURATION
        report.update(capture_start_monotonic=start, capture_deadline_monotonic=cache.end)
        atomic_json(out/'ready.json', dict(pid=os.getpid(), capture_start_monotonic=start,
                    capture_deadline_monotonic=cache.end, topics=TOPICS, diagnostic_only=True))
        report['stop_reason'] = 'duration_elapsed'
        tick = start
        while time.monotonic() < cache.end and not stop.is_set() and not rospy.is_shutdown():
            item = cache.pop()
            if item is None:
                time.sleep(.002)
            else:
                write_pair(bag, item, pairs, rospy.Time)
            if time.monotonic() - tick >= 1.0:
                tick = time.monotonic()
                if not MOUNT.is_mount() or shutil.disk_usage(MOUNT).free < MIN_FREE:
                    raise BudgetError('SanStand mount lost or available space below 4.5 GiB')
                if stream.tell() > BAG_BUDGET - 16*1024**2:
                    raise BudgetError('Bag close reserve reached')
                print('RAW_HEARTBEAT pairs=%d elapsed=%.1f' % (len(pairs), tick-start), flush=True)
        report['capture_end_monotonic'] = time.monotonic()
        if stop.is_set() or rospy.is_shutdown():
            report['stop_reason'] = 'interrupted'
        cache.stop()
        for sub in subs:
            sub.unregister()
        subs.clear()
        while time.monotonic() < hard_end:
            item = cache.pop()
            if item is None:
                break
            write_pair(bag, item, pairs, rospy.Time)
        if cache.snapshot()['queue_pairs_remaining']:
            raise RuntimeError('Owned writer did not drain within close reserve')
    except BaseException as exc:
        errors.append(repr(exc))
        report['stop_reason'] = 'error'
    finally:
        if cache is not None:
            cache.stop()
        for sub in subs:
            sub.unregister()
        if bag is not None:
            try:
                bag.close()
                os.replace(active, closed)
                report['bag_closed'] = True
            except BaseException as exc:
                errors.append('close:' + repr(exc))
        if stream is not None and not stream.closed:
            try:
                stream.close()
            except BaseException as exc:
                errors.append('stream_close:' + repr(exc))
        for sig, handler in old.items():
            signal.signal(sig, handler)
    report.update(errors=errors, pairs=pairs, pair_count=len(pairs),
                  source_stamps_unmodified=True, images_unmodified=True,
                  topic_message_counts={t:len(pairs) for t in TOPICS.values()},
                  raw_pixel_bytes=sum(sum(v['data_bytes'] for v in row['images'].values()) for row in pairs),
                  completed_monotonic=time.monotonic(), bag_path=str(closed if report['bag_closed'] else active),
                  bag_bytes=(closed if report['bag_closed'] else active).stat().st_size if active.exists() or closed.exists() else 0,
                  receive=None if cache is None else cache.snapshot())
    report['capture_valid'] = validate_capture(report)[0]
    atomic_json(out/'outcome.json', report)
    print('RAW_FINISHED valid=%s pairs=%d' % (report['capture_valid'], len(pairs)), flush=True)
    return 0 if report['capture_valid'] else 2


def write_pair(bag, item, pairs, stamp_type):
    ns, row = item
    if len(pairs) >= MAX_PAIRS:
        raise BudgetError('Pair report bound exceeded')
    # Original objects and source stamps are handed to rosbag, never decoded.
    for name, topic in TOPICS.items():
        message, header = row[name]
        if int(message.header.stamp.to_nsec()) != ns:
            raise RuntimeError('Message source mutated before recording')
        bag.write(topic, message, stamp_type(ns//10**9, ns % 10**9))
    pairs.append(dict(source_ns=ns, write_completed_monotonic=time.monotonic(),
                      images={name:h for name, (_, h) in row.items()}))


def validate_capture(report):
    reasons = []
    if report.get('bag_closed') is not True: reasons.append('bag_not_closed')
    if report.get('errors'): reasons.append('recorder_error')
    if report.get('stop_reason') != 'duration_elapsed': reasons.append('recording_did_not_run_full_window')
    if report.get('capture_end_monotonic', 0) - report.get('capture_start_monotonic', 0) < DURATION:
        reasons.append('short_capture')
    if report.get('pair_count', 0) <= 0: reasons.append('no_exact_raw_rgbd_pairs')
    receive = report.get('receive') or {}
    if receive.get('queue_pairs_remaining', 0): reasons.append('writer_not_drained')
    counts = receive.get('counts', {})
    if any(counts.get(k, 0) for k in ('memory_limit', 'pair_limit', 'arrival_limit',
                                    'invalid_source_or_empty_image', 'pending_capacity_evicted_sources')):
        reasons.append('diagnostic_input_loss_or_limit')
    return not reasons, reasons


def validate_child_result(ready, outcome, child_pid, returncode):
    good, reasons = validate_capture(outcome)
    if not isinstance(ready, dict) or ready.get('pid') != child_pid:
        reasons.append('ready_owner_missing_or_mismatched')
    elif (ready.get('capture_start_monotonic') != outcome.get('capture_start_monotonic')
          or ready.get('capture_deadline_monotonic') != outcome.get('capture_deadline_monotonic')):
        reasons.append('capture_window_not_bound_to_ready')
    if returncode != 0: reasons.append('owned_recorder_nonzero_or_still_running')
    return not reasons, reasons


class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        c = super().make_connection(host)
        c.timeout = 1.0
        return c


class Monitor:
    def __init__(self, name, master_uri):
        self.master = xmlrpc.client.ServerProxy(master_uri, transport=BoundedTransport())
        self.name, self.rows, self.sdk = name, [], {}

    def observe(self):
        start = time.monotonic()
        row = dict(monotonic=start, wall_ns=time.time_ns())
        try:
            code, message, value = self.master.getParam(self.name, '/camera_read_diagnostics')
            if code != 1: raise RuntimeError(message)
            payload = json.loads(value['payload_json'])
            row['camera_diagnostic_owner'] = {k:payload.get(k) for k in
                ('schema','geometry_generation','geometry_activation_stamp_ns','frame_id','copy_errors')}
            for entry in payload.get('entries', []):
                ns = int(entry['source_stamp_ns'])
                if ns not in self.sdk:
                    if len(self.sdk) >= 12000: raise BudgetError('SDK metadata bound')
                    e = entry.get('evidence', {})
                    self.sdk[ns] = dict(source_ns=ns, monotonic=entry['read_completed_monotonic'],
                        geometry_generation=entry.get('geometry_generation'),
                        geometry_activation_stamp_ns=entry.get('geometry_activation_stamp_ns'),
                        host_wait=e.get('host_wait'), stages_sec=e.get('stages_sec'),
                        result=e.get('result'), raw_frames=e.get('raw_frames'),
                        regional_reset_reason=e.get('regional_stationary_evidence', {}).get('global_history_reset_reason'))
        except Exception as exc:
            row['camera_metadata_error'] = repr(exc)
        row['metadata_read_seconds'] = time.monotonic() - start
        for name in ('cpu', 'memory', 'io'):
            row['psi_'+name] = Path('/proc/pressure/'+name).read_text()
        vm = dict(line.split() for line in Path('/proc/vmstat').read_text().splitlines())
        row['vmstat'] = {k:int(v) for k,v in vm.items() if k.startswith('allocstall') or k in
            ('pswpin','pswpout','pgmajfault','nr_dirty','nr_writeback','pgscan_direct','pgsteal_direct')}
        row['meminfo'] = {line.split(':')[0]:line.split(':',1)[1].strip()
            for line in Path('/proc/meminfo').read_text().splitlines()
            if line.split(':')[0] in ('MemAvailable','MemFree','Cached','Dirty','Writeback','SwapFree')}
        row['diskstats'] = [line.split() for line in Path('/proc/diskstats').read_text().splitlines()
            if line.split()[2].startswith(('sd', 'nvme'))]
        if len(self.rows) >= 1024: raise BudgetError('System sample bound')
        self.rows.append(row)


def stop_owned(child):
    """Only the Popen child, never a ROS node name, process group or arm process."""
    if child is None or child.poll() is not None: return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=3)
    except subprocess.TimeoutExpired:
        child.terminate()
        try:
            child.wait(timeout=1)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=1)


def run(out):
    out = external_directory(out, create=True)
    setup_logging(out)
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('passive_raw_rgbd_recording_load_comparison', anonymous=True,
                    disable_signals=True, disable_rosout=True)
    monitor = Monitor(rospy.get_name(), os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
    feedback = {'accepted': [], 'heartbeat': []}
    lock, stop = threading.Lock(), threading.Event()
    old = {s:signal.signal(s, lambda *_:stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    def callback(msg, key):
        with lock:
            if len(feedback[key]) < 30000:
                feedback[key].append(dict(source_ns=msg.header.stamp.to_nsec(),
                    receipt_ns=rospy.Time.now().to_nsec(), monotonic=time.monotonic(),
                    caller=msg._connection_header.get('callerid'), positions=list(msg.position)))
    subs = [rospy.Subscriber(topic, JointState, callback, callback_args=key, queue_size=200)
        for key,topic in [('accepted','/alicia_d/accepted_joint_states'),('heartbeat','/joint_states')]]
    phases, errors = [], []
    child, bstart, bfinished, ready = None, None, None, None
    bdir = out/'B_raw_recording'
    outcome = {}
    started = time.monotonic()
    mount_device = MOUNT.stat().st_dev
    def sample():
        if stop.is_set() or rospy.is_shutdown(): raise RuntimeError('comparison_interrupted')
        if not MOUNT.is_mount() or MOUNT.stat().st_dev != mount_device:
            raise RuntimeError('SanStand mount identity changed')
        monitor.observe()
    def phase(name, seconds):
        begin = time.monotonic()
        print('PHASE '+name, flush=True)
        try:
            last = begin
            while time.monotonic() - begin < seconds:
                sample()
                if time.monotonic() - last >= 5:
                    last = time.monotonic()
                    print('HEARTBEAT %s elapsed=%.1f' % (name,last-begin), flush=True)
                time.sleep(min(.5, max(0, begin+seconds-time.monotonic())))
        finally:
            phases.append(dict(name=name,start=begin,end=time.monotonic()))
    try:
        phase('A1_without_extra_raw_recording', DURATION)
        external_directory(bdir, create=True)
        bstart = time.monotonic()
        # stdout is deliberately inherited: no unbounded pipe or extra log file.
        child = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()),
                                  '--record-child', '--output', str(bdir)])
        while child.poll() is None:
            elapsed = time.monotonic()-bstart
            if elapsed >= CHILD_LIMIT-7:
                raise RuntimeError('Owned B recorder close deadline exceeded')
            if ready is None and (bdir/'ready.json').exists():
                ready = json.loads((bdir/'ready.json').read_text())
                if ready.get('pid') != child.pid: raise RuntimeError('Recorder ready owner mismatch')
                phases.append(dict(name='B_startup',start=bstart,end=ready['capture_start_monotonic']))
            if ready is None and elapsed > 15:
                raise RuntimeError('Owned B recorder readiness timeout')
            sample()
            print('HEARTBEAT B elapsed=%.1f ready=%s' % (elapsed,ready is not None), flush=True)
            time.sleep(.5)
        bfinished=time.monotonic()
        if (bdir/'outcome.json').exists():
            outcome = json.loads((bdir/'outcome.json').read_text())
        good, why = validate_child_result(ready, outcome, child.pid, child.returncode)
        if not good:
            raise RuntimeError('B invalid: returncode=%s reasons=%s' % (child.returncode,why))
        phases.extend([dict(name='B_raw_rgbd_recording', start=outcome['capture_start_monotonic'],
                            end=outcome['capture_end_monotonic']),
                       dict(name='B_close',start=outcome['capture_end_monotonic'],end=time.monotonic())])
        phase('writeback_cooldown_not_a_baseline', 10.0)
        phase('A2_without_extra_raw_recording', DURATION)
    except BaseException as exc:
        errors.append(repr(exc))
    finally:
        try:
            stop_owned(child)
        except BaseException as exc:
            errors.append('owned_child_cleanup:'+repr(exc))
        if bstart is not None:
            phases.append(dict(name='B_owned_process_lifetime',start=bstart,
                               end=time.monotonic() if bfinished is None else bfinished))
        if not outcome and (bdir/'outcome.json').exists():
            try:
                outcome=json.loads((bdir/'outcome.json').read_text())
            except Exception as exc:
                errors.append('partial_child_report:'+repr(exc))
        for sub in subs: sub.unregister()
        for sig, handler in old.items(): signal.signal(sig,handler)
    report = dict(schema=1, diagnostic_only=True, motion_authorized=False,
        hardware_commands_sent=False, configuration_changed=False,
        experiment='raw_exact_rgbd_recording_load_ABA', elapsed_sec=time.monotonic()-started,
        phases=phases, samples=monitor.rows, sdk_reads=list(monitor.sdk.values()), feedback=feedback,
        recorder_returncode=None if child is None else child.poll(), recorder_outcome=outcome,
        recorder_ready=ready,
        owned_recorder_still_running=child is not None and child.poll() is None,
        errors=errors, minimum_free_bytes=MIN_FREE, maximum_directory_bytes=TOTAL_BUDGET,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        valid_contrast=not errors and bool(outcome.get('capture_valid')),
        limits=['No target selection or reuse; no active inference or robot workload was created.',
                'Raw RGB-D recording can exceed sparse production keyframe load.',
                'B changes subscriptions, memory, serialization, LZ4 and external writes together; not disk alone.',
                'Exact matching preserves original images; unmatched source messages are explicitly counted, never paired nearest.',
                'SDK history denotes completed reads, not guaranteed publications; pre-phase entries require exclusion during analysis.',
                'A2 may contain outstanding writeback; 10s cooldown cannot guarantee cache reset.',
                'Deadlines are cooperative under OS scheduling/I/O; timeout or nonzero exit invalidates the contrast.',
                'No actual RGB-D pairs means an invalid experiment, never a clean recording-on result.'])
    atomic_json(out/'observations.json', report)
    hashes = {}
    # Hash only after every observation phase and the owned child have finished.
    files = [out/'observations.json'] if report['owned_recorder_still_running'] else sorted(out.rglob('*'))
    for p in files:
        if p.is_file() and p.name != 'SHA256SUMS':
            h=hashlib.sha256()
            with p.open('rb') as f:
                for block in iter(lambda:f.read(4*1024**2),b''):h.update(block)
            hashes[str(p.relative_to(out))]=h.hexdigest()
    (out/'SHA256SUMS').write_text(''.join(h+'  '+p+'\n' for p,h in hashes.items()))
    size=directory_bytes(out)
    atomic_json(out/'summary.json',dict(valid_contrast=report['valid_contrast'],
        errors=errors, observations_sha256=hashes['observations.json'],
        bytes_before_summary=size, maximum_directory_bytes=TOTAL_BUDGET,
        owned_recorder_still_running=report['owned_recorder_still_running'],
        artifact_hashes=hashes,
        hash_scope='observations_only_live_owned_recorder_unresolved' if report['owned_recorder_still_running'] else 'closed_capture_and_sidecars',
        note='ROS log file may receive late shutdown text; bag digest is computed only after owned process exit.'))
    size=directory_bytes(out)
    if size>TOTAL_BUDGET: raise BudgetError('Final directory budget exceeded')
    print('FINISHED valid=%s bytes=%d path=%s' % (report['valid_contrast'],size,out),flush=True)
    return 0 if report['valid_contrast'] else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--record-child',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    return record_child(args.output) if args.record_child else run(args.output)


if __name__=='__main__':
    sys.exit(main())
