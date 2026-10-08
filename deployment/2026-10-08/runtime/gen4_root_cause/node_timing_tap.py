"""Finite, opt-in timing of existing camera work; no ROS or device operations.

Wall minus thread CPU is an unclassified wait/scheduling interval, not a GIL
measurement. Source stamps are existing host-read stamps, never exposure time.
Only scalar events are queued; the original image objects are never retained.
"""
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time
import weakref


class NodeTimingTap:
    EVENT_BYTES = 512
    SUMMARY_BYTES = 64 * 1024
    MAX_BYTES = 4 * 1024 * 1024

    def __init__(self, camera_class, publication_class, output, *, seconds=30.,
                 max_bytes=MAX_BYTES, queue_size=1024, clock_ns=time.monotonic_ns,
                 cpu_ns=time.thread_time_ns, provenance=None, start_thread=True):
        if not 0 < seconds <= 30 or not self.SUMMARY_BYTES + 2*self.EVENT_BYTES <= max_bytes <= self.MAX_BYTES:
            raise ValueError('30 second / 4 MiB diagnostic bounds required')
        if not 1 <= queue_size <= 4096:
            raise ValueError('bounded timing queue required')
        self.camera_class, self.publication_class = camera_class, publication_class
        self.output = Path(output)
        self.seconds, self.max_bytes = float(seconds), int(max_bytes)
        self.clock_ns, self.cpu_ns = clock_ns, cpu_ns
        self.provenance = dict(provenance or {})
        self.lock = threading.RLock()
        self.local = threading.local()
        self.queue = queue.Queue(maxsize=queue_size)
        self.start_ns = self.last_ns = None
        self.closed = self.sealed = False
        self.closed_ns = None
        self.finish_wall = None
        self.reason = None
        self.sequence = self.reserved = self.written_bytes = 0
        self.digest = hashlib.sha256()
        self.inflight, self.nodes, self.workers = {}, {}, {}
        self.errors = []
        self.counts = dict(started=0, completed=0, enqueued=0, written=0,
                           queue_dropped=0, diagnostic_errors=0, budget_rejected=0)
        self.hooks = []
        specs = [(camera_class, '_timed_stage', 'read'),
                 (camera_class, '_remember_successful_read', 'remember'),
                 (camera_class, '_publish_temporal_status', 'temporal_status'),
                 (camera_class, '_publish_snapshot', 'publish'),
                 (publication_class, 'submit', 'submit')]
        # Validate before installing anything, so a missing method cannot leave
        # half an instrumentation set attached to the normal camera.
        originals = [(cls, name, stage, getattr(cls, name)) for cls, name, stage in specs]
        if not all(callable(original) for _, _, _, original in originals):
            raise ValueError('original camera timing methods unavailable')
        try:
            for cls, name, stage, original in originals:
                wrapper = self._wrapper(original, stage)
                self.hooks.append((cls, name, original, wrapper))
                setattr(cls, name, wrapper)
            self.thread = threading.Thread(target=self._writer, name='camera-node-timings', daemon=True)
            if start_thread:
                self.thread.start()
        except BaseException:
            self.finish('installation_failed')
            raise

    @staticmethod
    def _stamp(value):
        return value if type(value) is int and 0 < value < 2**63 else None

    def _error(self, exc):
        with self.lock:
            self.counts['diagnostic_errors'] += 1
            if len(self.errors) < 8:
                self.errors.append((type(exc).__name__+': '+str(exc))[:256])

    def _fail_diagnostic(self, exc, reason):
        # Even a failing diagnostic clock/receiver must not mask the original
        # camera exception or prevent the original call.
        try:
            self._error(exc)
            self.finish(reason)
        except Exception:
            pass

    def _wrapper(self, original, stage):
        tap = self
        def timed(receiver, *args, **kwargs):
            if stage == 'read' and (args[1] if len(args) > 1 else kwargs.get('stage')) != 'read':
                return original(receiver, *args, **kwargs)
            token = None
            try:
                token = tap._begin(stage, receiver, args, kwargs)
            except Exception as exc:
                tap._fail_diagnostic(exc, 'diagnostic_begin_error')
            failure = None
            try:
                return original(receiver, *args, **kwargs)
            except BaseException as exc:
                failure = type(exc).__name__[:64]
                raise
            finally:
                if token is not None:
                    try:
                        tap._end(token, failure)
                    except Exception as exc:
                        tap._fail_diagnostic(exc, 'diagnostic_end_error')
        return timed

    def _track(self, receiver, stage):
        registry = self.workers if stage == 'submit' else self.nodes
        registry[id(receiver)] = weakref.ref(receiver)
        if stage != 'submit':
            worker = getattr(receiver, '_publication_worker', None)
            if worker is not None:
                self.workers[id(worker)] = weakref.ref(worker)

    def _begin(self, stage, receiver, args, kwargs):
        now, cpu, thread = self.clock_ns(), self.cpu_ns(), threading.get_native_id()
        with self.lock:
            if self.closed:
                # Timer/worker can have captured a bound method just before
                # finish restored the class, then stored it just afterwards.
                self._detach_receiver(receiver, stage)
                return None
            self._track(receiver, stage)
            if self.start_ns is None:
                if stage != 'read':
                    return None
                self.start_ns = now
            if now - self.start_ns >= int(self.seconds * 1e9):
                self.finish('window_elapsed')
                return None
            if self.reserved + 2*self.EVENT_BYTES > self.max_bytes-self.SUMMARY_BYTES:
                self.counts['budget_rejected'] += 1
                self.finish('event_budget_exhausted')
                return None
            if len(self.inflight) >= 128:
                self.finish('inflight_limit')
                return None
            self.sequence += 1
            token = dict(call_id=self.sequence, stage=stage, thread_id=thread,
                         owner_id=id(receiver), source_stamp_ns=None, read_call_id=None)
            if stage == 'read':
                callback = args[2] if len(args) > 2 else kwargs.get('callback')
                token['read_owner_id'] = id(getattr(callback, '__self__', None))
                self.local.read = None
            elif stage == 'remember':
                token['source_stamp_ns'] = self._stamp(args[0] if args else kwargs.get('source_stamp_ns'))
                camera = kwargs.get('camera')
                if camera is None:
                    camera = getattr(receiver, 'cam', None)
                previous = getattr(self.local, 'read', None)
                if previous and previous['read_owner_id'] == id(camera) and previous['owner_id'] == id(receiver):
                    token['read_call_id'] = previous['call_id']
            elif stage == 'submit':
                token['source_stamp_ns'] = self._stamp(args[3] if len(args) > 3 else kwargs.get('host_read_completed_stamp_ns'))
            elif stage == 'publish':
                snapshot = args[0] if args else kwargs.get('snapshot')
                token['source_stamp_ns'] = self._stamp(getattr(snapshot, 'host_read_completed_stamp_ns', None))
            row = dict(token, event='start', monotonic_ns=now, thread_cpu_ns=cpu)
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                self.counts['queue_dropped'] += 1
                self.finish('queue_full')
                return None
            self.inflight[token['call_id']] = token
            self.reserved += 2*self.EVENT_BYTES
            self.counts['started'] += 1
            self.counts['enqueued'] += 1
            self.last_ns = now
            token['entry_hook_ns'] = self.clock_ns()-now
            token['entry_hook_cpu_ns'] = self.cpu_ns()-cpu
            return token

    def _end(self, token, failure):
        now, cpu = self.clock_ns(), self.cpu_ns()
        with self.lock:
            self.inflight.pop(token['call_id'], None)
            if token['stage'] == 'read':
                self.local.read = dict(token) if failure is None else None
            if self.sealed:
                return
            row = dict(token, event='end', monotonic_ns=now, thread_cpu_ns=cpu,
                       exception_type=failure, crosses_window_end=now-self.start_ns >= int(self.seconds*1e9))
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                self.counts['queue_dropped'] += 1
                self.finish('queue_full')
            else:
                self.counts['completed'] += 1
                self.counts['enqueued'] += 1
            self.last_ns = now
            if now-self.start_ns >= int(self.seconds*1e9):
                self.finish('window_elapsed')

    def _restore_bound(self, target, attribute, receiver, name):
        if target is None:
            return
        for cls, method, original, wrapper in self.hooks:
            if method != name:
                continue
            current = getattr(target, attribute, None)
            if getattr(current, '__func__', None) is wrapper and getattr(current, '__self__', None) is receiver:
                setattr(target, attribute, original.__get__(receiver, type(receiver)))

    def _detach_receiver(self, receiver, stage):
        if stage == 'submit':
            callback = getattr(receiver, '_publish', None)
            owner = getattr(callback, '__self__', None)
            if owner is not None:
                self._restore_bound(receiver, '_publish', owner, '_publish_snapshot')
            return
        self._restore_bound(getattr(receiver, '_runtime_status_timer', None), '_callback',
                            receiver, '_publish_temporal_status')
        self._restore_bound(getattr(receiver, '_publication_worker', None), '_publish',
                            receiver, '_publish_snapshot')

    def finish(self, reason='explicit_finish'):
        with self.lock:
            if not self.closed:
                self.closed, self.reason, self.finish_wall = True, reason, time.monotonic()
                try:self.closed_ns = self.clock_ns()
                except Exception:pass
            for cls, name, original, wrapper in self.hooks:
                if getattr(cls, name, None) is wrapper:
                    setattr(cls, name, original)
            for registry, stage in ((self.nodes, 'node'), (self.workers, 'submit')):
                for reference in tuple(registry.values()):
                    receiver = reference()
                    if receiver is not None:
                        self._detach_receiver(receiver, stage)

    def _summary(self):
        with self.lock:
            return dict(schema=1, diagnostic_only=True, no_pixel_copies=True,
                window_clock='first CameraNode read invocation, monotonic_ns',
                source_clock='original host read-completion stamp; not exposure time',
                requested_window_sec=self.seconds, first_read_start_ns=self.start_ns,
                last_event_ns=self.last_ns, close_reason=self.reason,
                closed_monotonic_ns=self.closed_ns,
                window_complete=(self.reason == 'window_elapsed'),
                sealed=self.sealed, events_sha256=self.digest.hexdigest(),
                event_bytes=self.written_bytes, reserved_event_bytes=self.reserved,
                max_directory_bytes=self.max_bytes, counts=dict(self.counts), errors=list(self.errors),
                queued_unwritten=self.queue.qsize(), unfinished_calls=list(self.inflight.values()),
                class_hooks_restored=all(getattr(c, n, None) is o for c, n, o, _ in self.hooks),
                observed_bound_callbacks_restored=self._bound_callbacks_restored(),
                observed_node_count=len(self.nodes), observed_publication_worker_count=len(self.workers),
                source_implementation=self.provenance,
                limitations=['Thread CPU does not measure GIL ownership; wall-minus-CPU includes SDK waits and scheduling.',
                    'Publication timing does not prove subscriber delivery.',
                    'Disk I/O and Python scheduling are cooperative; no hard wall-clock completion bound.',
                    'A read span includes the existing read-observer metadata work.',
                    'Span starts include the separately reported entry hook overhead.'])

    def _bound_callbacks_restored(self):
        wrappers = {wrapper for _, _, _, wrapper in self.hooks}
        for reference in self.nodes.values():
            node = reference()
            if node is not None:
                timer = getattr(node, '_runtime_status_timer', None)
                if getattr(getattr(timer, '_callback', None), '__func__', None) in wrappers:
                    return False
        for reference in self.workers.values():
            worker = reference()
            if worker is not None and getattr(getattr(worker, '_publish', None), '__func__', None) in wrappers:
                return False
        return True

    def _writer(self):
        stream = None
        try:
            self.output.mkdir(parents=True, exist_ok=False)
            stream = (self.output/'events.jsonl').open('xb', buffering=64*1024)
            while True:
                with self.lock:
                    if self.start_ns is not None and self.clock_ns()-self.start_ns >= int(self.seconds*1e9):
                        self.finish('window_elapsed')
                    if self.closed and ((not self.inflight and self.queue.empty())
                            or time.monotonic()-self.finish_wall >= 1.):
                        self.sealed = True
                        break
                try:
                    row = self.queue.get(timeout=.02)
                except queue.Empty:
                    continue
                data = (json.dumps(row, allow_nan=False, separators=(',', ':'))+'\n').encode()
                if len(data) > self.EVENT_BYTES:
                    raise ValueError('timing_event_exceeds_reserved_budget')
                if stream.write(data) != len(data):
                    raise OSError('short timing-event write')
                self.written_bytes += len(data)
                self.digest.update(data)
                self.counts['written'] += 1
            stream.flush()
            os.fsync(stream.fileno())
        except Exception as exc:
            self._error(exc)
            self.finish('writer_error')
        finally:
            if stream is not None:
                try:
                    stream.close()
                except Exception as exc:
                    self._error(exc)
            self.finish('writer_closed')
            with self.lock:
                self.sealed = True
            try:
                summary = self._summary()
                summary['complete'] = bool(not summary['errors'] and not summary['queued_unwritten']
                    and not summary['unfinished_calls'] and summary['counts']['queue_dropped'] == 0
                    and summary['counts']['budget_rejected'] == 0 and summary['class_hooks_restored']
                    and summary['observed_bound_callbacks_restored']
                    and summary['counts']['started'] > 0
                    and summary['counts']['written'] == 2*summary['counts']['started'])
                raw = (json.dumps(summary, indent=2, allow_nan=False)+'\n').encode()
                if len(raw) > self.SUMMARY_BYTES or self.written_bytes+len(raw) > self.max_bytes:
                    raise ValueError('timing_summary_budget_exceeded')
                with (self.output/'summary.json').open('xb') as target:
                    target.write(raw)
            except Exception as exc:
                self._error(exc)

    def shutdown(self):
        self.finish('shutdown')
        if self.thread.ident is not None:
            self.thread.join(timeout=2.)
