#!/usr/bin/env python3
"""Run the unchanged camera entry point with a finite existing-diagnostic tap.

By default no pixels are copied or reprocessed. An explicit visual-snapshot
option additionally retains bounded raw RGB and actual LK anchors for offline
diagnosis; it never changes production image outputs or visual permissions.
The production read method is restored
when the first-read-relative window expires. The camera process then continues
normally under its independent systemd service.
"""
import argparse
import atexit
import copy
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import queue
import runpy
import sys
import threading
import time

CAMERA_TIMING_SOURCE = (Path(__file__).resolve().parents[2]
    /'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py')


class ExistingReadObserver:
    def __init__(self, camera_class, output, seconds=45., queue_size=256, clock=time.monotonic):
        if not 0 < seconds <= 120 or not 1 <= queue_size <= 1024:
            raise ValueError('bounded diagnostic duration/queue required')
        self.camera_class = camera_class
        self.original = camera_class.read
        self.output = Path(output)
        self.seconds = float(seconds)
        self.clock = clock
        self.first_read = None
        self.last_read = None
        self.closed = False
        self.queue = queue.Queue(maxsize=queue_size)
        self.counts = dict(observed=0,enqueued=0,written=0,queue_dropped=0,
                           copy_errors=0,serialization_errors=0,write_errors=0)
        self.errors = []
        self.thread = threading.Thread(target=self._writer, name='existing-camera-diagnostics', daemon=True)
        self.summary_path = self.output.with_suffix('.summary.json')
        observer = self
        def observed_read(camera, *args, **kwargs):
            started = observer.clock()
            if observer.first_read is None:
                observer.first_read = started
            if observer.closed or started-observer.first_read >= observer.seconds:
                observer.finish()
                return observer.original(camera,*args,**kwargs)
            try:
                return observer.original(camera,*args,**kwargs)
            finally:
                observer.last_read = observer.clock()
                observer.counts['observed'] += 1
                try:
                    row = dict(read_started_monotonic=started,
                               read_completed_monotonic=observer.last_read,
                               captured_wall_time_ns=time.time_ns(),
                               existing_read_diagnostics=copy.deepcopy(camera.last_read_diagnostics),
                               stationary_temporal=copy.deepcopy(getattr(camera,'runtime_profile',{}).get('stationary_temporal')))
                    provider=getattr(camera,'publication_diagnostics_provider',None)
                    row['publication_worker']=copy.deepcopy(provider()) if callable(provider) else None
                    try:
                        observer.queue.put_nowait(row)
                        observer.counts['enqueued'] += 1
                    except queue.Full:
                        observer.counts['queue_dropped'] += 1
                except Exception as exc:
                    observer.counts['copy_errors'] += 1
                    observer._error('diagnostic metadata copy',exc)
                if observer.last_read-observer.first_read >= observer.seconds:
                    observer.finish()
        self.wrapper = observed_read
        camera_class.read = observed_read
        self.thread.start()

    def _error(self, stage, error):
        if len(self.errors) < 8:
            self.errors.append(dict(stage=stage,error=str(error)))

    def finish(self):
        if self.camera_class.read is self.wrapper:
            self.camera_class.read = self.original
        self.closed = True

    def _writer(self):
        stream = None
        try:
            stream = self.output.open('w', buffering=256*1024)
            while not self.closed or not self.queue.empty():
                try: row = self.queue.get(timeout=.2)
                except queue.Empty: continue
                try:
                    encoded = json.dumps(row,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n'
                except Exception as exc:
                    self.counts['serialization_errors'] += 1; self._error('serialization',exc); continue
                try:
                    stream.write(encoded); self.counts['written'] += 1
                except Exception as exc:
                    self.counts['write_errors'] += 1; self._error('write',exc)
                    self.finish(); break
            stream.flush()
            os.fsync(stream.fileno())
        except Exception as exc:
            self.counts['write_errors'] += 1; self._error('open/flush',exc); self.finish()
        finally:
            if stream is not None:
                try: stream.close()
                except Exception as exc: self._error('close',exc)
            summary = dict(schema=1,counts=self.counts,errors=self.errors,
                           first_read_monotonic=self.first_read,last_read_monotonic=self.last_read,
                           requested_window_sec=self.seconds,clock='first read start / monotonic',
                           queued_unwritten=self.queue.qsize(),
                           original_read_restored=self.camera_class.read is self.original,
                           observer_changes_source_stamp_semantics=False,
                           camera_process_continues_after_observer=True)
            try:
                self.summary_path.write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
            except Exception as exc:
                summary['summary_write_error'] = str(exc)
            print('CAMERA_EXISTING_READ_OBSERVER '+json.dumps(summary,ensure_ascii=False,allow_nan=False),flush=True)

    def shutdown(self):
        self.finish()
        self.thread.join(timeout=3.)


def loaded_module_manifest():
    # Importing these pure modules does not construct a device. Include the
    # regional module before its first lazy read-time import so the observer
    # records the exact implementation the camera will use.
    loaded = {}
    for name in ('alicia_flexible_grasp.vision.realsense_manager','alicia_flexible_grasp.vision.visual_stationary',
                 'alicia_flexible_grasp.vision.latest_rgbd_publication',
                 'alicia_flexible_grasp.vision.regional_stationary_depth',
                 'alicia_flexible_grasp.vision.pair_history_continuity'):
        item = importlib.import_module(name)
        path = Path(item.__file__).resolve()
        loaded[name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return loaded


def prepare_visual_snapshot_helper():
    """Opt-in only: import a pure diagnostic module, never open a device."""
    path=Path(__file__).resolve().with_name('visual_snapshot_tap.py')
    spec=importlib.util.spec_from_file_location('camera_visual_snapshot_tap',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module,dict(path=str(Path(module.__file__).resolve()),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def prepare_node_timing_helper():
    """Import the opt-in scalar timer without starting a node or device."""
    path=Path(__file__).resolve().with_name('node_timing_tap.py')
    spec=importlib.util.spec_from_file_location('camera_node_timing_tap',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module,dict(path=str(Path(module.__file__).resolve()),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def resolve_camera_timing_namespace(namespace, camera_script):
    """Resolve only the actual source or its known catkin devel relay.

    The relay executes the source into its explicit ``context`` dictionary.
    A random similarly named context is not an acceptable camera entry.
    """
    source=CAMERA_TIMING_SOURCE.resolve()
    relay=source.parents[3]/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py'
    entry=Path(camera_script).resolve()
    if entry == source:
        actual=namespace
    elif (entry == relay.resolve() and namespace.get('python_script') == str(source)
            and isinstance(namespace.get('context'),dict)):
        actual=namespace['context']
        if actual.get('__name__') != namespace.get('__name__'):
            raise ValueError('catkin camera relay namespace mismatch')
    else:
        raise ValueError('unsupported camera timing entry')
    if Path(actual.get('__file__','')).resolve() != source:
        raise ValueError('camera timing source path mismatch')
    camera, main=actual.get('CameraNode'),actual.get('main')
    functions=[main]+[getattr(camera,name,None) for name in (
        '_timed_stage','_remember_successful_read','_publish_temporal_status','_publish_snapshot')]
    for function in functions:
        code=getattr(function,'__code__',None)
        globals_=getattr(function,'__globals__',{})
        if (code is None or Path(code.co_filename).resolve()!=source
                or globals_.get('CameraNode') is not camera
                or Path(globals_.get('__file__','')).resolve()!=source):
            raise ValueError('camera timing functions do not originate from actual source')
    return actual


def record_node_timing_startup_error(output, error):
    try:
        Path(output).parent.joinpath('node_timing_startup_error.json').write_text(
            json.dumps(dict(error=repr(error),diagnostic_only=True))+'\n')
    except Exception:pass


def run_camera_entry(camera_script, timing_module=None, *, output=None, seconds=30., provenance=None):
    """Default entry is unchanged; opt-in wraps this file's actual class/main.

    Loading a namespace avoids executing main before its class can be wrapped.
    We still call the original main exactly once, including its shutdown finally.
    Diagnostic installation failure falls back to that same original main.
    """
    if timing_module is None:
        return runpy.run_path(str(camera_script),run_name='__main__')
    try:
        namespace=runpy.run_path(str(camera_script),run_name='__camera_node_timing_diagnostic__')
        namespace=resolve_camera_timing_namespace(namespace,camera_script)
    except Exception as exc:
        record_node_timing_startup_error(output,exc)
        return runpy.run_path(str(camera_script),run_name='__main__')
    tap=None
    try:
        try:
            publication=importlib.import_module('alicia_flexible_grasp.vision.latest_rgbd_publication')
            tap=timing_module.NodeTimingTap(namespace['CameraNode'],publication.LatestRGBDPublication,
                output,seconds=seconds,provenance=provenance)
        except Exception as exc:
            # No instrumentation failure is allowed to prevent the original
            # camera from running. The absent/failed timing summary is explicit.
            record_node_timing_startup_error(output,exc)
        return namespace['main']()
    finally:
        if tap is not None:
            try:tap.shutdown()
            except Exception:
                # A timing cleanup problem must not replace main's original
                # return/exception. Its absent/incomplete summary is a failure.
                pass


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-script',type=Path,required=True)
    parser.add_argument('--diagnostic-path',type=Path,required=True)
    parser.add_argument('--diagnostic-seconds',type=float,default=45.)
    optional=parser.add_mutually_exclusive_group()
    optional.add_argument('--capture-visual-snapshots',action='store_true',
        help='Explicit bounded raw RGB/actual LK anchor capture; default off')
    optional.add_argument('--capture-node-timings',action='store_true',
        help='Explicit scalar timing of existing node stages, <=30 s / 4 MiB; default off')
    parser.add_argument('ros_args',nargs=argparse.REMAINDER)
    args=parser.parse_args(argv)
    module=importlib.import_module('alicia_flexible_grasp.vision.realsense_manager')
    diagnostic_path=args.diagnostic_path.resolve()
    if not Path('/media/zhuyupei/SanStand').is_mount() or Path('/media/zhuyupei/SanStand') not in diagnostic_path.parents:
        raise RuntimeError('camera diagnostics require mounted SanStand output')
    loaded=loaded_module_manifest()
    tap_module=None;tap_error=None;tap=None
    timing_module=None;timing_error=None
    if args.capture_visual_snapshots:
        try:
            if not 0 < args.diagnostic_seconds <= 30:
                raise ValueError('visual capture requires a <=30 second window')
            tap_module,tap_manifest=prepare_visual_snapshot_helper()
            loaded['diagnostic.visual_snapshot_tap']=tap_manifest
        except Exception as exc:tap_error=repr(exc)
    if args.capture_node_timings:
        try:
            if not 0 < args.diagnostic_seconds <= 30:
                raise ValueError('node timing requires a <=30 second window')
            timing_module,timing_manifest=prepare_node_timing_helper()
            loaded['diagnostic.node_timing_tap']=timing_manifest
            loaded['diagnostic.camera_node_timing_entry']=dict(path=str(args.camera_script.resolve()),
                sha256=hashlib.sha256(args.camera_script.read_bytes()).hexdigest())
            loaded['diagnostic.camera_node_timing_source']=dict(path=str(CAMERA_TIMING_SOURCE.resolve()),
                sha256=hashlib.sha256(CAMERA_TIMING_SOURCE.read_bytes()).hexdigest())
        except Exception as exc:timing_error=repr(exc)
    (diagnostic_path.parent/'loaded_observer_modules.json').write_text(json.dumps(loaded,indent=2)+'\n')
    observer=ExistingReadObserver(module.RealSenseManager,diagnostic_path,args.diagnostic_seconds)
    atexit.register(observer.shutdown)
    if tap_module is not None:
        try:
            regional=importlib.import_module('alicia_flexible_grasp.vision.regional_stationary_depth')
            visual=importlib.import_module('alicia_flexible_grasp.vision.visual_stationary')
            tap=tap_module.VisualSnapshotTap(regional.RegionalStationaryDepth,visual.VisualStationaryWindow,
                diagnostic_path.parent/'visual_snapshots',seconds=args.diagnostic_seconds,
                start_time=lambda:observer.first_read,provenance=loaded)
            atexit.register(tap.shutdown)
        except Exception as exc:tap_error=repr(exc)
    if tap_error is not None:
        # A diagnostic failure must never prevent the original camera entry.
        # The loader rejects this capture as incomplete; the camera continues.
        try:(diagnostic_path.parent/'visual_snapshot_startup_error.json').write_text(
            json.dumps(dict(error=tap_error,diagnostic_only=True))+'\n')
        except Exception as exc:print('VISUAL_SNAPSHOT_ERROR_RECORD_FAILED '+repr(exc),flush=True)
        print('VISUAL_SNAPSHOT_STARTUP_ERROR '+tap_error,flush=True)
    if timing_error is not None:
        try:(diagnostic_path.parent/'node_timing_startup_error.json').write_text(
            json.dumps(dict(error=timing_error,diagnostic_only=True))+'\n')
        except Exception:pass
        print('NODE_TIMING_STARTUP_ERROR '+timing_error,flush=True)
    ros_args=args.ros_args[1:] if args.ros_args[:1]==['--'] else args.ros_args
    sys.argv=[str(args.camera_script)]+ros_args
    try:run_camera_entry(args.camera_script,timing_module,output=diagnostic_path.parent/'node_timings',
                        seconds=args.diagnostic_seconds,provenance=loaded)
    finally:
        if tap is not None:tap.shutdown()
        observer.shutdown()


if __name__=='__main__':
    main()
