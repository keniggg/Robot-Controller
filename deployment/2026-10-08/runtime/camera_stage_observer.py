#!/usr/bin/env python3
"""Finite, opt-in taps around existing camera operations; no device reconfiguration.

Run as the camera entry wrapper after an explicitly authorized camera-only reload.
The unchanged camera entry continues after capture; this is not a second camera
owner. Arrays are copied while SDK frames are alive, then written asynchronously.
"""
import argparse
import atexit
import copy
import hashlib
import importlib
import io
import json
import math
import os
from pathlib import Path
import queue
import runpy
import sys
import threading
import time

import numpy as np


MIB = 1024 * 1024
REQUIRED = ('raw_depth', 'raw_rgb', 'aligned_depth', 'post_spatial_depth',
            'pre_stationary_depth', 'post_stationary_depth', 'final_depth', 'final_rgb')


class TapProxy:
    def __init__(self, target, method, tap):
        self.target, self.method, self.tap = target, method, tap

    def __getattr__(self, name):
        original = getattr(self.target, name)
        if name != self.method:
            return original
        def call(*args, **kwargs):
            result = original(*args, **kwargs)
            self.tap(result)
            return result
        return call


class CameraStageObserver:
    def __init__(self, camera_class, output, *, seconds=30., samples=12,
                 interval=2., warmup=3., stable_reads=10, queue_size=2,
                 maximum_bytes=64*MIB, clock=time.monotonic, start_thread=True):
        numeric = (seconds, interval, warmup)
        if (not all(math.isfinite(v) for v in numeric) or not 0 < seconds <= 30
                or not 1 <= samples <= 12 or not 1 <= queue_size <= 2
                or not .5 <= interval <= 5 or not 0 <= warmup < seconds
                or not 1 <= stable_reads <= 100 or not MIB <= maximum_bytes <= 64*MIB):
            raise ValueError('invalid bounded observer limits')
        self.camera_class, self.original = camera_class, camera_class.read
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.seconds, self.samples, self.interval = seconds, samples, interval
        self.warmup, self.stable_reads = warmup, stable_reads
        self.maximum_bytes, self.clock = maximum_bytes, clock
        self.first_read = self.last_read = self.last_sample = None
        self.consecutive_complete = self.attempts = self.inflight = 0
        self.closed, self.close_reason = False, None
        self.errors, self.files = [], []
        self.total_written = 0
        self.counts = dict(reads=0, selected=0, complete=0, written=0, queue_dropped=0,
                           capture_errors=0, original_read_errors=0, write_errors=0)
        self.queue = queue.Queue(maxsize=queue_size)
        self.state_lock = threading.RLock()
        self.read_gate = threading.Lock()
        self.timer = None
        self.thread = threading.Thread(target=self._writer, name='camera-stage-writer', daemon=True)
        observer = self
        def observed_read(camera, *args, **kwargs):
            return observer.read(camera, *args, **kwargs)
        self.wrapper = observed_read
        if start_thread:
            self.thread.start()
        # No patched class remains if creating the diagnostic writer fails.
        camera_class.read = self.wrapper

    def _error(self, kind, exc):
        with self.state_lock:
            if len(self.errors) < 12:
                self.errors.append(dict(kind=kind, error=str(exc)[:600]))

    def finish(self, reason='requested'):
        with self.state_lock:
            if not self.closed:
                self.close_reason = reason
            self.closed = True
            # A caller must not bypass read_gate while temporary instance
            # proxies still exist in an already running production read.
            if not self.inflight and self.camera_class.read is self.wrapper:
                self.camera_class.read = self.original
            if self.timer is not None:
                self.timer.cancel()

    def _safe(self, row, fn, *args):
        if self.closed or row.get('capture_error'):
            return
        started = self.clock()
        try:
            fn(*args)
        except Exception as exc:
            row['capture_error'] = str(exc)[:600]
            self.counts['capture_errors'] += 1
            self._error('capture', exc)
            self.finish('capture_error')
        finally:
            row['capture_overhead_sec'] += max(0., self.clock()-started)

    def _array(self, row, name, value):
        array = np.asanyarray(value)
        color = name.endswith('rgb')
        if (array.dtype != (np.uint8 if color else np.uint16)
                or array.ndim != (3 if color else 2)
                or color and array.shape[2] != 3
                or array.size <= 0 or array.nbytes > 2*MIB):
            raise ValueError('unexpected array layout: ' + name)
        if name in row['arrays']:
            raise ValueError('duplicate stage: ' + name)
        if sum(v.nbytes for v in row['arrays'].values()) + array.nbytes > 8*MIB-128*1024:
            raise ValueError('per-pair memory budget exceeded')
        row['arrays'][name] = np.array(array, copy=True, order='C')

    @staticmethod
    def _intrinsics(profile):
        i = profile.as_video_stream_profile().get_intrinsics()
        return dict(width=int(i.width), height=int(i.height), fx=float(i.fx), fy=float(i.fy),
                    ppx=float(i.ppx), ppy=float(i.ppy), model=str(i.model),
                    coeffs=[float(v) for v in i.coeffs])

    def _raw(self, row, camera, frames):
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            raise ValueError('missing raw SDK frame')
        self._array(row, 'raw_depth', depth.get_data())
        self._array(row, 'raw_rgb', color.get_data())
        metadata = {}
        fields = ('frame_counter', 'frame_timestamp', 'sensor_timestamp', 'backend_timestamp',
                  'time_of_arrival', 'actual_fps', 'actual_exposure', 'gain_level', 'auto_exposure')
        for name, frame in (('depth', depth), ('color', color)):
            entry = dict(frame_number=int(frame.get_frame_number()),
                         timestamp_ms=float(frame.get_timestamp()),
                         timestamp_domain=str(frame.get_frame_timestamp_domain()),
                         intrinsics=self._intrinsics(frame.get_profile()), metadata_raw={}, unavailable={})
            for key in fields:
                try:
                    field = getattr(camera.rs.frame_metadata_value, key)
                    if frame.supports_frame_metadata(field):
                        entry['metadata_raw'][key] = int(frame.get_frame_metadata(field))
                    else:
                        entry['unavailable'][key] = 'unsupported'
                except Exception as exc:
                    entry['unavailable'][key] = type(exc).__name__
            metadata[name] = entry
        extrinsic = depth.get_profile().get_extrinsics_to(color.get_profile())
        row['sdk_frames'] = metadata
        row['depth_to_color'] = dict(rotation_column_major=list(extrinsic.rotation),
                                    translation_m=list(extrinsic.translation))

    def _selected_read(self, camera, row, args, kwargs):
        replacements = []
        def replace(name, value):
            had = name in camera.__dict__
            original = camera.__dict__.get(name)
            setattr(camera, name, value)
            replacements.append((name, had, original, value))
        def tap(fn):
            return lambda value:self._safe(row, fn, value)
        try:
            replace('pipeline', TapProxy(camera.pipeline, 'wait_for_frames',
                                         tap(lambda f:self._raw(row, camera, f))))
            if camera.align is not None:
                def aligned(frames):
                    frame = frames.get_depth_frame()
                    self._array(row, 'aligned_depth', frame.get_data())
                    row['aligned_depth_intrinsics'] = self._intrinsics(frame.get_profile())
                replace('align', TapProxy(camera.align, 'process', tap(aligned)))
            else:
                row['alignment_disabled'] = True
            filters = list(camera.depth_filters)
            names = [v.get('name') for v in camera.applied_depth_filters]
            if len(names) != len(filters) or names.count('spatial') > 1:
                raise ValueError('unknown configured filter chain')
            proxies = []
            for original, name in zip(filters, names):
                def filtered(frame, stage=name):
                    if stage == 'spatial':
                        self._array(row, 'post_spatial_depth', frame.get_data())
                proxies.append(TapProxy(original, 'process', tap(filtered)))
            replace('depth_filters', proxies)
            original_stationary = camera._filter_stationary_depth
            def stationary(frame, *a, **kw):
                self._safe(row, lambda:self._array(row, 'pre_stationary_depth', frame.get_data()))
                result = original_stationary(frame, *a, **kw)
                self._safe(row, lambda:self._array(row, 'post_stationary_depth', result.get_data()))
                return result
            replace('_filter_stationary_depth', stationary)
        except Exception as exc:
            row['capture_error'] = str(exc)[:600]
            self.counts['capture_errors'] += 1
            self._error('install taps', exc)
            self.finish('tap_install_error')
        try:
            # Original production exceptions must propagate, unmodified.
            result = self.original(camera, *args, **kwargs)
            if result[0] is not None and result[1] is not None:
                self._safe(row, self._array, row, 'final_rgb', result[0])
                self._safe(row, self._array, row, 'final_depth', result[1])
            return result
        finally:
            for name, had, original, proxy in reversed(replacements):
                if camera.__dict__.get(name) is proxy:
                    if had:
                        setattr(camera, name, original)
                    else:
                        delattr(camera, name)

    def read(self, camera, *args, **kwargs):
        # Normal camera has one read loop. A concurrent read cannot be allowed
        # to encounter another call's temporary instance proxies.
        with self.read_gate:
            now = self.clock()
            with self.state_lock:
                if self.first_read is None:
                    self.first_read = now
                    try:
                        self.timer = threading.Timer(self.seconds, self.finish, args=('deadline',))
                        self.timer.daemon = True
                        self.timer.start()
                    except Exception as exc:
                        self._error('deadline timer', exc)
                        self.finish('timer_error')
                if now-self.first_read >= self.seconds:
                    self.finish('deadline')
                self.inflight += 1
            selected = (not self.closed and now-self.first_read >= self.warmup
                        and self.consecutive_complete >= self.stable_reads
                        and (self.last_sample is None or now-self.last_sample >= self.interval)
                        and self.attempts < self.samples)
            row = dict(arrays={}, capture_overhead_sec=0., read_started_monotonic=now,
                       raw_sdk_source_clock_is_not_ros_exposure_clock=True)
            if selected:
                self.attempts += 1
                self.last_sample = now
                self.counts['selected'] += 1
            try:
                result = (self._selected_read(camera, row, args, kwargs) if selected
                          else self.original(camera, *args, **kwargs))
                diagnostic = getattr(camera, 'last_read_diagnostics', None)
                complete = (result[0] is not None and result[1] is not None
                            and isinstance(diagnostic, dict) and diagnostic.get('result') == 'complete')
                self.consecutive_complete = self.consecutive_complete+1 if complete else 0
                if selected and not self.closed:
                    def finalize():
                        row['read_completed_monotonic'] = self.clock()
                        row['read_completed_wall_ns'] = time.time_ns()
                        row['production_read_diagnostics'] = copy.deepcopy(diagnostic)
                        row['runtime_profile'] = copy.deepcopy(camera.runtime_profile)
                        row['depth_scale_m'] = float(camera.depth_scale)
                        row['array_coordinate_spaces'] = dict(raw_depth='native SDK depth intrinsics',
                            raw_rgb='native SDK color intrinsics', aligned_depth='SDK color intrinsics',
                            post_spatial_depth='SDK color intrinsics', pre_stationary_depth='SDK color intrinsics',
                            post_stationary_depth='SDK color intrinsics', final_depth='SDK color intrinsics',
                            final_rgb='published color; inspect color_projection_correction')
                        row['complete'] = bool(complete and all(k in row['arrays'] for k in REQUIRED))
                        row['sample_index'] = self.attempts
                    self._safe(row, finalize)
                    # No further row mutation after ownership passes to writer.
                    if not self.closed:
                        try:
                            self.queue.put_nowait(row)
                        except queue.Full:
                            self.counts['queue_dropped'] += 1
                        except Exception as exc:
                            self._error('queue', exc)
                            self.finish('queue_error')
                        else:
                            self.counts['complete'] += int(row['complete'])
                return result
            except BaseException:
                self.counts['original_read_errors'] += 1
                self.consecutive_complete = 0
                raise
            finally:
                self.counts['reads'] += 1
                self.last_read = self.clock()
                if self.attempts >= self.samples:
                    self.finish('sample_limit')
                with self.state_lock:
                    self.inflight -= 1
                    if self.closed:
                        self.finish(self.close_reason)

    def _write_row(self, row):
        arrays = row.pop('arrays')
        row['arrays'] = {k:dict(dtype=str(v.dtype), shape=list(v.shape), bytes=v.nbytes,
                            sha256=hashlib.sha256(v.tobytes()).hexdigest(),
                            zero_depth_pixels=int(np.count_nonzero(v == 0)) if v.ndim == 2 else None)
                         for k,v in arrays.items()}
        encoded = (json.dumps(row, ensure_ascii=False, allow_nan=False, indent=2)+'\n').encode()
        if len(encoded) > 128*1024:
            raise ValueError('metadata exceeds bounded size')
        buffer = io.BytesIO()
        # Preserve both raw and corrected RGB even when byte-identical.
        np.savez(buffer, **arrays)
        payload = buffer.getvalue()
        size = len(payload)+len(encoded)
        if size > 8*MIB or self.total_written+size+256*1024 > self.maximum_bytes:
            raise ValueError('artifact byte budget exhausted')
        stem = 'sample_%02d' % row['sample_index']
        for name, contents in ((stem+'.npz',payload),(stem+'.json',encoded)):
            path = self.output/name
            with path.open('xb') as stream:
                stream.write(contents)
                stream.flush()
                os.fsync(stream.fileno())
            self.total_written += len(contents)
            self.files.append(dict(name=name, bytes=len(contents), sha256=hashlib.sha256(contents).hexdigest()))
        self.counts['written'] += 1

    def _writer(self):
        try:
            while not self.closed or self.inflight or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                try:
                    self._write_row(row)
                except Exception as exc:
                    self.counts['write_errors'] += 1
                    self._error('writer', exc)
                    self.finish('writer_error')
                    while not self.queue.empty():
                        try:self.queue.get_nowait()
                        except queue.Empty:break
                    break
        finally:
            self.finish('writer_exit')
            summary = self.summary()
            try:
                with (self.output/'summary.json').open('x') as stream:
                    json.dump(summary, stream, ensure_ascii=False, allow_nan=False, indent=2)
                    stream.write('\n')
            except Exception as exc:
                self._error('summary', exc)
                summary['summary_write_error'] = str(exc)
            print('CAMERA_STAGE_OBSERVER '+json.dumps(summary,ensure_ascii=False),flush=True)

    def summary(self):
        return dict(schema=1, counts=dict(self.counts), errors=list(self.errors), files=list(self.files),
                    capture_complete=self.counts['written'] == self.samples and not self.errors
                        and self.counts['complete'] == self.samples,
                    requested_samples=self.samples, maximum_capture_seconds=self.seconds,
                    warmup_sec=self.warmup, stable_complete_reads=self.stable_reads,
                    sample_interval_sec=self.interval, first_read_monotonic=self.first_read,
                    last_read_monotonic=self.last_read, close_reason=self.close_reason,
                    maximum_artifact_bytes=self.maximum_bytes, written_bytes=self.total_written,
                    queued_unwritten=self.queue.qsize(), inflight_reads=self.inflight,
                    original_read_restored=self.camera_class.read is self.original,
                    camera_process_continues=True, device_configuration_written=False,
                    robot_commands_sent=False, temperature_access=False,
                    timestamp_semantics='SDK source timestamps plus host read interval; no fabricated ROS image stamp',
                    timing_limitation='Capture taps add copy time; an in-flight SDK read is never cancelled at the deadline')

    def shutdown(self):
        self.finish('shutdown')
        if self.thread.ident is not None:
            self.thread.join(timeout=3.)


def loaded_modules():
    manifest = {}
    for name in ('alicia_flexible_grasp.vision.realsense_manager',
                 'alicia_flexible_grasp.vision.visual_stationary',
                 'alicia_flexible_grasp.vision.stationary_depth',
                 'alicia_flexible_grasp.vision.depth_preset', 'pyrealsense2'):
        item = importlib.import_module(name)
        path = Path(item.__file__).resolve()
        manifest[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                              version=getattr(item, '__version__', None))
    return manifest


def run_camera_entry(module, args):
    """Diagnostic setup cannot prevent the original camera entry from running."""
    output = args.output.resolve()
    observer = None
    original = module.RealSenseManager.read
    try:
        observer = CameraStageObserver(module.RealSenseManager, output, seconds=args.seconds,
                                       samples=args.samples, interval=args.interval, warmup=args.warmup)
        atexit.register(observer.shutdown)
    except Exception as exc:
        failure = dict(schema=1, capture_complete=False, startup_error=str(exc),
                       original_read_restored=module.RealSenseManager.read is original,
                       camera_process_continues=True, robot_commands_sent=False)
        try:
            with (output/'summary.json').open('x') as stream:
                json.dump(failure, stream, indent=2)
        except Exception as secondary:
            failure['summary_write_error'] = str(secondary)
        print('CAMERA_STAGE_OBSERVER_STARTUP_FAILED '+json.dumps(failure),flush=True)
    if observer is not None:
        try:
            with (output/'loaded_modules.json').open('x') as stream:
                json.dump(loaded_modules(), stream, indent=2)
                stream.write('\n')
        except Exception as exc:
            observer._error('loaded module manifest', exc)
            observer.finish('manifest_error')
    ros_args = args.ros_args[1:] if args.ros_args[:1] == ['--'] else args.ros_args
    sys.argv = [str(args.camera_script)]+ros_args
    try:
        runpy.run_path(str(args.camera_script), run_name='__main__')
    finally:
        if observer is not None:
            observer.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-script', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--interval', type=float, default=2.)
    parser.add_argument('--warmup', type=float, default=3.)
    parser.add_argument('ros_args', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    mount = Path('/media/zhuyupei/SanStand')
    output = args.output.resolve()
    if not mount.is_mount() or mount not in output.parents:
        raise RuntimeError('mounted SanStand output is required')
    manager = importlib.import_module('alicia_flexible_grasp.vision.realsense_manager')
    run_camera_entry(manager, args)


if __name__ == '__main__':
    main()
