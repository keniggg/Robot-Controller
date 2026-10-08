#!/usr/bin/env python3
"""Opt-in, single camera-owner A1 / saved baseline / A2 diagnostic.

Disabled unless --enable-preset-ab is explicit. Does not start or stop other
nodes, change selection, publish arm commands, or access temperature. Root must
quiesce unknown/remote consumers before enabling this sensor experiment.
"""
import argparse
import collections
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import queue
import runpy
import sys
import threading
import time
import weakref

import numpy as np

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location('ab_stage_taps', HERE/'camera_stage_observer.py')
_stage = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_stage)
MIB = 1024*1024
MAX_BYTES = 159_000_000  # Includes all NPZ, JSON, manifest, and summary bytes.
ARM = tuple('Joint%d' % i for i in range(1, 7))
PHASES = (('A1', 'unknown', 'high_accuracy'),
          ('B', 'carton', 'saved_baseline'), ('A2', 'unknown', 'high_accuracy'))


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2)+'\n').encode()


def digest(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def shadow_capture_budget(profile, requested, maximum):
    """Bound all complete pairs before capture, including the extra Z16 array."""
    def pixels(key):
        model = profile[key]
        w, h = model['width'], model['height']
        if type(w) is not int or type(h) is not int or not 1 <= w <= 4096 or not 1 <= h <= 4096:
            raise ValueError('invalid shadow budget geometry')
        return w*h
    native = pixels('native_depth_intrinsics')
    color = pixels('color_intrinsics')
    final = pixels('geometry_intrinsics')
    if max(2*native, 3*color, 3*final) > 2*MIB:
        raise ValueError('shadow array exceeds original 2 MiB per-array limit')
    # Raw native Z16; raw RGB and five aligned Z16 arrays including shadow;
    # final RGB/Z16; full metadata and conservative NPZ headers.
    pair = 2*native + 13*color + 5*final + 128*1024 + 8192
    if pair > 8*MIB:
        raise ValueError('shadow pair cannot fit original 8 MiB limit')
    reserved = 256*1024 + 64*1024
    per_phase = min(requested, (maximum-reserved)//(3*pair))
    if per_phase < 1:
        raise ValueError('shadow capture cannot fit three bounded phases')
    return dict(requested_samples_per_phase=requested, samples_per_phase=per_phase,
                maximum_pair_bytes=pair, reserved_summary_manifest_bytes=reserved,
                maximum_total_bytes=3*per_phase*pair+reserved)


class PrivateDepthCopy:
    """SDK-owned copy of a real input frame; no context, pipeline or device."""
    def __init__(self, rs):
        self.rs = rs
        self.queue = rs.frame_queue(1)
        self.error = None
        owner = weakref.ref(self)
        self.block = rs.processing_block(lambda frame, source:owner()._copy(frame, source))
        self.block.start(self.queue.enqueue)

    def _copy(self, frame, source):
        try:
            data = np.asanyarray(frame.get_data())
            if data.dtype != np.uint16 or data.ndim != 2 or not data.size or data.nbytes > 2*MIB:
                raise ValueError('invalid shadow input Z16')
            height, width = data.shape
            copied = source.allocate_video_frame(frame.get_profile(), frame, 2, width, height,
                                                 width*2, self.rs.extension.depth_frame)
            destination = np.asanyarray(copied.get_data())
            if np.shares_memory(data, destination):
                raise ValueError('shadow SDK allocation aliases production input')
            np.copyto(destination, data, casting='no')
            source.frame_ready(copied)
        except Exception as exc:
            self.error = exc

    def copy(self, frame):
        self.error = None
        self.block.invoke(frame)
        if self.error is not None:
            raise self.error
        result = self.queue.poll_for_frame()
        if not result:
            raise ValueError('shadow private frame unavailable after synchronous SDK processing')
        return result


class MatchedStationaryShadow:
    """Independent, bounded history; same real eligibility rules in every phase."""
    def __init__(self, node, evidence, clock):
        from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
        from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow
        self.window_class = StationaryDepthWindow
        self.visual = VisualStationaryWindow()
        self.node, self.camera, self.evidence, self.clock = node, node.cam, evidence, clock
        self.window = getattr(node, '_stationary_window', None)
        self.provider = getattr(self.camera, 'stationary_depth_provider', None)
        if (type(self.window) is not StationaryDepthWindow or not callable(self.provider)
                or not self.camera.mode_depth_preset_cfg.get('stationary_temporal_enabled', False)
                or evidence.initial_context['selection'].get('mode') != 'unknown'):
            raise ValueError('matched shadow requires real unknown selection and original stationary evidence window')
        self.private_copy = None
        self.temporal = None
        self.previous_depth = None
        self.history_id = 0
        self.phase = None
        self.phase_audits = []
        self.last = None

    def owned(self):
        return (self.node.cam is self.camera and self.node._stationary_window is self.window
                and self.camera.stationary_depth_provider is self.provider)

    def reset(self, reason):
        self.temporal = None
        self.visual.reset()
        self.previous_depth = None
        if self.phase is not None:
            self.phase['resets'][reason] = self.phase['resets'].get(reason, 0)+1
            self.phase['chain_sha256'] = digest(dict(previous=self.phase['chain_sha256'],
                                                    reset=reason, host_monotonic_sec=self.clock()))

    def begin(self, label):
        self.reset('phase_boundary')
        self.phase = dict(label=label, reads=0, eligible_reads=0, reasons={}, resets={},
                          chain_sha256='0'*64, total_processing_sec=0., max_processing_sec=0.,
                          total_tap_processing_sec=0., max_tap_processing_sec=0.)
        self.phase_audits.append(self.phase)
        self.last = None

    def _encoder(self):
        # Copy the exact real window under its lock, then call the unchanged
        # original algorithm on that copy. Do not call stationary() under its
        # non-reentrant lock, and do not relabel a later snapshot as its input.
        with self.window.lock:
            snapshot = {kind:tuple(rows) for kind, rows in self.window.history.items()}
        if any(len(rows)>24 for rows in snapshot.values()):
            raise ValueError('stationary source window exceeded bound')
        now = self.evidence.now_ns()
        frozen = self.window_class()
        frozen.history = snapshot
        qualified = frozen.stationary(now*1e-9)
        return dict(qualified=qualified, evaluation_ros_stamp_ns=now, source_window=snapshot,
                    source_window_sha256=digest(snapshot),
                    method='unchanged StationaryDepthWindow.stationary on lock-copied real input')

    def process(self, frame, diagnostic, color, source):
        started = self.clock()
        if not self.owned() or self.phase is None:
            raise ValueError('shadow evidence owner or phase changed')
        if (not isinstance(diagnostic, dict)
                or diagnostic.get('pair_timing', {}).get('accepted') is not True):
            self.reset('pair_not_accepted')
            raise ValueError('shadow tap lacks accepted original RGB-D pair')
        raw = diagnostic.get('raw_frames', {})
        if raw.get('color') != source:
            raise ValueError('shadow RGB source differs from original pair')
        depth_source = raw.get('depth', {})
        number, stamp, fps = (depth_source.get(key) for key in
                              ('frame_number', 'device_timestamp_ms', 'stream_fps'))
        domain = str(depth_source.get('timestamp_domain', '')).rsplit('.', 1)[-1]
        if (depth_source.get('present') is not True or type(number) is not int or number < 0
                or domain not in ('hardware_clock', 'global_time')
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                       for v in (stamp, fps)) or stamp < 0 or not 1 <= fps <= 240):
            raise ValueError('shadow depth source unavailable')
        current = (number, stamp, domain, fps)
        if self.previous_depth is not None:
            p = self.previous_depth
            if (number != p[0]+1 or domain != p[2] or fps != p[3]
                    or not .25/fps <= (stamp-p[1])/1000. <= 1.75/fps):
                self.reset('depth_source_discontinuity')
        self.previous_depth = current
        encoder = self._encoder()
        if encoder['qualified']:
            visual = self.visual.observe(color, source, started)
        else:
            self.reset('encoder_not_stationary')
            visual = dict(qualified=False, reason='encoder_not_stationary')
        active = bool(encoder['qualified'] and visual['qualified'])
        first = False
        output = frame
        if active:
            if self.private_copy is None:
                self.private_copy = PrivateDepthCopy(self.camera.rs)
            private = self.private_copy.copy(frame)
            if self.temporal is None:
                self.temporal = self.camera.rs.temporal_filter()
                for option, value in ((self.camera.rs.option.filter_smooth_alpha, .4),
                                      (self.camera.rs.option.filter_smooth_delta, 100.),
                                      (self.camera.rs.option.holes_fill, 0.)):
                    self.temporal.set_option(option, value)
                self.history_id += 1
                first = True
            output = self.temporal.process(private)
            incoming, filtered = np.asanyarray(frame.get_data()), np.asanyarray(output.get_data())
            if (incoming.dtype != np.uint16 or filtered.dtype != np.uint16 or filtered.shape != incoming.shape
                    or np.any(filtered[incoming == 0]) or first and not np.array_equal(incoming, filtered)):
                raise ValueError('shadow changed holes, geometry, or first-frame values')
        else:
            # Visual warmup must retain its own anchor; only depth history is
            # revoked. Resetting its visual window here would prevent warmup.
            self.temporal = None
        elapsed = max(0., self.clock()-started)
        reason = visual['reason']
        self.phase['reads'] += 1
        self.phase['eligible_reads'] += int(active)
        self.phase['reasons'][reason] = self.phase['reasons'].get(reason, 0)+1
        self.phase['total_processing_sec'] += elapsed
        self.phase['max_processing_sec'] = max(self.phase['max_processing_sec'], elapsed)
        link = dict(previous=self.phase['chain_sha256'], phase=self.phase['label'],
                    depth_source=depth_source, color_source=source,
                    encoder_stamp_ns=encoder['evaluation_ros_stamp_ns'],
                    encoder_source_window_sha256=encoder['source_window_sha256'],
                    visual=visual, active=active, history_id=self.history_id,
                    first_history_frame=first, processing_sec=elapsed)
        self.phase['chain_sha256'] = digest(link)
        self.last = dict(active=active, encoder=encoder, visual=visual, depth_source=copy.deepcopy(depth_source),
                         color_source=copy.deepcopy(source), first_history_frame=first, history_id=self.history_id,
                         phase=copy.deepcopy(self.phase), processing_sec=elapsed,
                         processing_sec_scope='Core eligibility/private copy/filter/hole checks; excludes audit hashing and selected array copy.',
                         output_kind='filtered' if active else 'current_frame_passthrough',
                         eligibility_scope='Same rules across A/B; independent of production history and sensor policy mode.',
                         alpha=.4, delta_depth_units=100, persistence=0,
                         original_output_replaced=False, geometry_authority=False)
        return output, self.last


def validate_sample_geometry(row):
    arrays = row['arrays']
    def shape(name, profile):
        expected = (int(profile['height']), int(profile['width']))
        if arrays[name].shape[:2] != expected:
            raise ValueError(name+' shape does not match its captured geometry')
    shape('raw_depth', row['sdk_frames']['depth']['intrinsics'])
    shape('raw_rgb', row['sdk_frames']['color']['intrinsics'])
    for name in ('aligned_depth', 'post_spatial_depth', 'pre_stationary_depth', 'post_stationary_depth'):
        shape(name, row['aligned_depth_intrinsics'])
    if 'matched_shadow_depth' in arrays:
        shape('matched_shadow_depth', row['aligned_depth_intrinsics'])
    published = row['published_camera_parameters']['runtime_profile']['geometry_intrinsics']
    runtime = row['runtime_profile']['geometry_intrinsics']
    if runtime != published:
        raise ValueError('camera runtime and published final geometry differ')
    shape('final_depth', published)
    shape('final_rgb', published)


class EvidenceGuard:
    """Callbacks latch excursions, including changes which subsequently return."""
    def __init__(self):
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.current = dict(epoch=None, sdk=None, accepted=None, task=None)
        self.frozen = None
        self.faults = []
        self.accepted = collections.deque(maxlen=128)

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.current)

    def fail(self, reason):
        with self.condition:
            if reason not in self.faults and len(self.faults) < 16:
                self.faults.append(str(reason)[:500])
            self.condition.notify_all()

    def observe(self, kind, stamp_ns, value):
        with self.condition:
            try:
                if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
                    raise ValueError('invalid source timestamp')
                if kind in ('accepted', 'sdk'):
                    q = tuple(value)
                    count = 7 if kind == 'accepted' else 6
                    if len(q) != count or not all(math.isfinite(v) for v in q):
                        raise ValueError('invalid joint evidence')
                    if self.current[kind] and stamp_ns <= self.current[kind]['source_stamp_ns']:
                        raise ValueError('nonmonotonic joint evidence')
                    item = dict(source_stamp_ns=stamp_ns, q=list(q))
                    if self.frozen and item['q'] != self.frozen[kind]['q']:
                        self.fail(kind+'_changed')
                    self.current[kind] = item
                    if kind == 'accepted':
                        self.accepted.append(copy.deepcopy(item))
                elif kind == 'epoch':
                    if self.frozen and stamp_ns != self.frozen['epoch']:
                        self.fail('epoch_changed')
                    self.current[kind] = stamp_ns
                elif kind == 'task':
                    if not isinstance(value, dict) or value.get('active') is not False:
                        raise ValueError('active_or_missing_task')
                    if self.frozen and value != self.frozen['task']:
                        self.fail('task_changed')
                    self.current[kind] = copy.deepcopy(value)
                else:
                    raise ValueError('unknown guard input')
            except Exception as exc:
                self.fail(kind+': '+str(exc))
            self.condition.notify_all()

    def check(self, now_ns, *, freeze=False):
        with self.lock:
            if self.faults:
                raise ValueError('; '.join(self.faults))
            if any(value is None for value in self.current.values()):
                return False
            row = self.current['accepted']
            if not 0 <= now_ns-row['source_stamp_ns'] <= 500_000_000:
                if self.frozen:
                    self.fail('accepted_feedback_stale')
                    raise ValueError('accepted_feedback_stale')
                return False
            if row['source_stamp_ns'] < self.current['epoch']:
                self.fail('accepted_feedback_before_epoch')
                raise ValueError('accepted_feedback_before_epoch')
            sdk = self.current['sdk']
            if not 0 <= now_ns-sdk['source_stamp_ns'] <= 1_000_000_000:
                if self.frozen:
                    self.fail('sdk_command_stale')
                    raise ValueError('sdk_command_stale')
                return False
            if sdk['source_stamp_ns'] < self.current['epoch']:
                self.fail('sdk_command_before_epoch')
                raise ValueError('sdk_command_before_epoch')
            if freeze and self.frozen is None:
                self.frozen = copy.deepcopy(self.current)
            return True

    def bracket(self, ns, timeout=.35):
        deadline = time.monotonic()+timeout
        with self.condition:
            while True:
                rows = list(self.accepted)
                lo = next((r for r in reversed(rows) if r['source_stamp_ns'] <= ns), None)
                hi = next((r for r in rows if r['source_stamp_ns'] >= ns), None)
                if lo and hi or time.monotonic() >= deadline:
                    break
                self.condition.wait(min(.025, max(0., deadline-time.monotonic())))
            span = hi['source_stamp_ns']-lo['source_stamp_ns'] if lo and hi else None
            valid = bool(lo and hi and self.frozen and not self.faults
                         and lo['source_stamp_ns'] >= self.frozen['epoch']
                         and span <= 200_000_000
                         and lo['q'] == hi['q'] == self.frozen['accepted']['q'])
            return dict(anchor_ros_stamp_ns=ns, accepted_before=lo, accepted_after=hi,
                        span_ns=span, maximum_span_ns=200_000_000, qualified=valid,
                        q_at_anchor=lo['q'] if valid else None,
                        interpolation='constant measured endpoints' if valid else None,
                        faults=list(self.faults))


class RosEvidence:
    """Only subscriptions, parameter reads, and exact-time TF lookup."""
    def __init__(self, rospy):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        from alicia_flexible_grasp_supervisor.msg import GraspState
        import tf2_ros
        self.rospy = rospy
        self.guard = EvidenceGuard()
        perception = rospy.get_param('/unknown_perception', {})
        camera = rospy.get_param('/camera', {})
        self.parent = str(perception.get('base_frame', 'base_link'))
        self.child = str(perception.get('camera_frame', camera.get('frame_id', 'camera_link')))
        self.initial_context = self.context()
        if len(encoded(self.initial_context)) > 32*1024:
            raise ValueError('initial context exceeds summary budget')
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.subs = []
        try:
            for topic, datatype, callback, limit in (
                ('/alicia_d/accepted_joint_states', JointState, lambda m:self.joints('accepted', m), 128),
                ('/alicia_d/sdk_command', JointState, lambda m:self.joints('sdk', m), 64),
                ('/alicia_d/control_reference_epoch', Header,
                 lambda m:self.guard.observe('epoch', m.stamp.to_nsec(), None), 10),
                ('/grasp/state', GraspState, self.task, 20)):
                self.subs.append(rospy.Subscriber(topic, datatype, callback, queue_size=limit))
        except Exception:
            self.close()
            raise

    def joints(self, kind, message):
        try:
            expected = ARM+(('right_finger',) if kind == 'accepted' else ())
            names = tuple(message.name)
            if (message.header.frame_id != ('sdk_measured' if kind == 'accepted' else 'sdk_transmitted')
                    or len(names) != len(expected) or len(message.position) != len(expected)
                    or set(names) != set(expected)):
                raise ValueError('joint names/frame mismatch')
            mapping = dict(zip(names, message.position))
            self.guard.observe(kind, message.header.stamp.to_nsec(), tuple(float(mapping[n]) for n in expected))
        except Exception as exc:
            self.guard.fail(kind+': '+str(exc))

    def task(self, message):
        self.guard.observe('task', message.header.stamp.to_nsec(),
                           {k:getattr(message, k) for k in ('state', 'stage', 'active', 'success', 'message')})

    def now_ns(self):
        return self.rospy.Time.now().to_nsec()

    def context(self):
        return dict(selection=self.rospy.get_param('/grasp_mode/selection'),
                    handeye=self.rospy.get_param('/handeye', None),
                    robot_description_sha256=hashlib.sha256(
                        self.rospy.get_param('/robot_description', '').encode()).hexdigest())

    def check_context(self):
        if self.context() != self.initial_context:
            self.guard.fail('selection_handeye_or_robot_description_changed')
            raise ValueError('selection_handeye_or_robot_description_changed')

    def at_anchor(self, ns):
        feedback = self.guard.bracket(ns)
        tf = None
        error = None
        # Keep the actual TF binding response even when exact admission fails.
        # Some ROS 1 Python/C++ bindings round through double seconds. This is
        # diagnostic provenance only: it never widens the exact-stamp gate or
        # rewrites the returned timestamp as the requested anchor.
        stamp_audit = dict(requested_stamp_ns=ns, returned_stamp_ns=None,
                           delta_ns=None, exact_match=False)
        try:
            stamp = self.rospy.Time(ns//10**9, ns%10**9)
            msg = self.buffer.lookup_transform(self.parent, self.child, stamp, self.rospy.Duration(.35))
            returned_ns = msg.header.stamp.to_nsec()
            stamp_audit.update(returned_stamp_ns=returned_ns,
                               delta_ns=returned_ns-ns, exact_match=returned_ns == ns)
            if returned_ns != ns:
                raise ValueError('TF timestamp differs from exact requested anchor')
            tr, rot = msg.transform.translation, msg.transform.rotation
            values = [tr.x,tr.y,tr.z,rot.x,rot.y,rot.z,rot.w]
            if (msg.header.frame_id != self.parent or msg.child_frame_id != self.child
                    or not all(math.isfinite(v) for v in values)
                    or abs(sum(v*v for v in values[3:])-1.) > 1e-6):
                raise ValueError('invalid exact TF frame or transform')
            tf = dict(parent=msg.header.frame_id, child=msg.child_frame_id, source_stamp_ns=ns,
                      translation=[tr.x,tr.y,tr.z], quaternion_xyzw=[rot.x,rot.y,rot.z,rot.w])
        except Exception as exc:
            error = str(exc)[:600]
        return dict(feedback=feedback, exact_tf=tf, tf_error=error, tf_stamp_audit=stamp_audit,
                    evidence_complete=bool(feedback['qualified'] and tf is not None),
                    scope='Host read-completion ROS anchor; not exposure time or published image header')

    def close(self):
        for sub in self.subs:
            sub.unregister()
        self.listener.unregister()


class PresetABObserver(_stage.CameraStageObserver):
    """Reuses only isolated owned-copy taps and bounded NPZ serialization."""
    def __init__(self, node, output, evidence, *, seconds=75., interval=.5,
                 warmup=3., stable_reads=10, samples_per_phase=8, maximum_bytes=MAX_BYTES,
                 clock=time.monotonic, start_thread=True, matched_stationary_shadow=False):
        if (not all(math.isfinite(v) for v in (seconds, interval, warmup))
                or not 15 <= seconds <= 75 or not .5 <= interval <= 1.
                or not 3 <= warmup <= 10 or not 10 <= stable_reads <= 100
                or not 1 <= samples_per_phase <= 8 or not MIB <= maximum_bytes <= MAX_BYTES):
            raise ValueError('invalid bounded A/B limits')
        self.node, self.camera, self.evidence = node, node.cam, evidence
        self.camera_class = type(self.camera)
        self.policy = getattr(self.camera, 'mode_depth_preset', None)
        if self.policy is None or self.policy.mode != 'unknown':
            raise ValueError('requires an already active unknown/high_accuracy camera policy')
        self.shadow_budget = None
        self.shadow = None
        if matched_stationary_shadow:
            self.shadow_budget = shadow_capture_budget(self.camera.runtime_profile, samples_per_phase, maximum_bytes)
            samples_per_phase = self.shadow_budget['samples_per_phase']
            self.shadow = MatchedStationaryShadow(node, evidence, clock)
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.seconds, self.interval, self.warmup = seconds, interval, warmup
        self.stable_reads, self.samples_per_phase = stable_reads, samples_per_phase
        self.maximum_bytes, self.clock = maximum_bytes, clock
        self.samples = 3*samples_per_phase
        self.state_lock, self.read_gate = threading.RLock(), threading.RLock()
        self.queue = queue.Queue(maxsize=2)
        self.errors, self.files, self.phases = [], [], []
        self.last_raw_sources = {}
        self.total_written = 0
        self.counts = dict(reads=0, selected=0, complete=0, written=0, queue_dropped=0,
                           capture_errors=0, original_read_errors=0, write_errors=0)
        self.closed = False
        self.stop_reason = self.close_reason = None
        self.inflight = 0
        self.restore = dict(attempted=False, succeeded=False)
        self.phase = -1
        self.phase_samples = self.consecutive_complete = self.attempts = 0
        self.phase_started = self.last_sample = self.last_read = None
        self.first_read = self.clock()
        self.original_mode = self.policy.mode
        self.sensor_write_attempted = False
        self.initial_sensor = self.sensor_snapshot()
        self.original_read = self.camera.read
        self.original = lambda camera, *a, **kw:self.original_read(*a, **kw)
        self.original_sync = node._sync_depth_preset
        self.saved_read = ('read' in self.camera.__dict__, self.camera.__dict__.get('read'))
        self.saved_sync = ('_sync_depth_preset' in node.__dict__, node.__dict__.get('_sync_depth_preset'))
        self.wrapper, self.sync_wrapper = self.read, self.sync
        self.thread = threading.Thread(target=self._writer, name='camera-preset-ab-writer', daemon=True)
        self.timer = threading.Timer(seconds, self.finish, args=('deadline',))
        self.timer.daemon = True
        if start_thread:
            self.thread.start()
            try:
                self.timer.start()
            except Exception:
                self.finish('timer_start_failed')
                self.closed = True
                self.thread.join(1.)
                raise
        # Patch instances only after all preflight/setup operations succeed.
        self.camera.read = self.wrapper
        self.node._sync_depth_preset = self.sync_wrapper

    def _raw(self, row, camera, frames):
        super()._raw(row, camera, frames)
        for stream, entry in row['sdk_frames'].items():
            source = (entry['frame_number'], entry['timestamp_ms'], entry['timestamp_domain'])
            previous = self.last_raw_sources.get(stream)
            if previous and (source[2] != previous[2] or source[0] <= previous[0]
                             or source[1] <= previous[1]):
                raise ValueError(stream+' SDK source repeated, reversed, or clock domain changed')
            self.last_raw_sources[stream] = source

    def sensor_snapshot(self):
        raw_parameters = self.policy.advanced.serialize_json()
        if len(raw_parameters.encode()) > 16*1024:
            raise ValueError('sensor parameters exceed diagnostic bound')
        parameters = json.loads(raw_parameters)
        options = {}
        for name in ('visual_preset', 'enable_auto_exposure', 'exposure', 'gain',
                     'enable_auto_white_balance', 'white_balance'):
            try:
                option = getattr(self.camera.rs.option, name)
                if not self.policy.sensor.supports(option):
                    options[name] = dict(unavailable='unsupported')
                else:
                    options[name] = dict(value=float(self.policy.sensor.get_option(option)),
                                         semantics='option readback, not per-frame measured exposure')
            except Exception as exc:
                options[name] = dict(unavailable=str(exc)[:200])
        return dict(advanced_settings=parameters, advanced_parameters_sha256=digest(parameters['parameters']),
                    options=options, depth_scale_m=float(self.policy.sensor.get_depth_scale()),
                    policy=copy.deepcopy(self.policy.evidence()))

    def finish(self, reason='requested'):
        # Called by timer/writer/callback paths: never touch SDK off the owner.
        with self.state_lock:
            if self.stop_reason is None:
                self.stop_reason = reason

    def _owns_camera(self):
        return (self.node.cam is self.camera
                and self.camera.mode_depth_preset is self.policy
                and self.camera.__dict__.get('read') is self.wrapper
                and self.node.__dict__.get('_sync_depth_preset') is self.sync_wrapper)

    def sync(self, publish=True, timings=None):
        if self.closed:
            return self.original_sync(publish=publish, timings=timings)
        if not self._owns_camera():
            self._error('ownership', ValueError('camera owner replaced during experiment'))
            self.finish('camera_owner_changed')
            with self.read_gate:
                self._complete_finish()
            return self.original_sync(publish=publish, timings=timings)
        # No selection substitution: actual per-phase mode is set by this owner.

    def _phase_start(self, index):
        self.evidence.check_context()
        self.evidence.guard.check(self.evidence.now_ns(), freeze=True)
        if self.clock()-self.first_read >= self.seconds:
            self.finish('deadline')
        if self.stop_reason:
            return
        label, mode, name = PHASES[index]
        if not self._owns_camera():
            raise ValueError('camera owner changed before preset transition')
        self.sensor_write_attempted = True
        self.camera.set_depth_preset_mode(mode)
        snapshot = self.sensor_snapshot()
        if self.policy.mode != mode or snapshot['policy']['preset'] != name:
            raise ValueError('preset readback mismatch')
        if mode == 'carton':
            expected = json.loads(self.policy.baseline)['parameters']
            if snapshot['advanced_settings']['parameters'] != expected:
                raise ValueError('saved baseline readback mismatch')
        elif float(self.policy.sensor.get_option(self.camera.rs.option.visual_preset)) != 3.:
            raise ValueError('high_accuracy readback mismatch')
        if snapshot['depth_scale_m'] != self.initial_sensor['depth_scale_m']:
            raise ValueError('depth scale changed')
        self.phase, self.phase_samples = index, 0
        if self.shadow is not None:
            self.shadow.begin(label)
        self.phase_started = self.clock()
        self.consecutive_complete = 0
        self.last_sample = None
        self.phases.append(dict(label=label, sdk_policy_mode=mode, sensor_condition=name,
                               meaning='diagnostic sensor branch; global task selection is unchanged',
                               started_monotonic=self.phase_started, sensor_before=snapshot))
        self.camera.runtime_profile['preset_ab_diagnostic'] = dict(
            active=True, phase=label, sensor_condition=name, sdk_policy_mode=mode,
            global_selection_changed=False, comparison='raw SDK and aligned before filters')
        self.node._publish_runtime_camera_params()

    def _restore_owner(self):
        self.restore['attempted'] = True
        try:
            if not self._owns_camera():
                raise ValueError('original camera owner replaced; refusing to configure a replacement')
            # Force both existing API edges, including after an A2 failure.
            self.camera.set_depth_preset_mode('carton')
            self.camera.set_depth_preset_mode(self.original_mode)
            snapshot = self.sensor_snapshot()
            self.restore['sensor_after'] = snapshot
            if (snapshot['advanced_settings']['parameters'] != self.initial_sensor['advanced_settings']['parameters']
                    or snapshot['depth_scale_m'] != self.initial_sensor['depth_scale_m']
                    or self.policy.mode != self.original_mode):
                raise ValueError('original sensor parameters restoration mismatch')
            self.restore['succeeded'] = True
        except Exception as exc:
            self.restore['error'] = str(exc)[:600]
            self._error('restoration', exc)
        finally:
            if self._owns_camera():
                self.camera.runtime_profile['preset_ab_diagnostic'] = dict(
                    active=False, restoration_succeeded=self.restore['succeeded'],
                    experiment_valid=False, consult_summary=True)
                try:
                    self.node._publish_runtime_camera_params()
                except Exception as exc:
                    self._error('restored metadata publication', exc)
                    self.restore['succeeded'] = False
            else:
                self.restore['metadata_publication_skipped'] = 'camera_owner_changed'

    def _complete_finish(self):
        if self.closed:
            return
        self.close_reason = self.stop_reason or 'finished'
        try:
            if self.sensor_write_attempted:
                self._restore_owner()
            else:
                self.restore = dict(attempted=False, succeeded=True, reason='sensor experiment never started')
            try:
                self.evidence.check_context()
                self.evidence.guard.check(self.evidence.now_ns())
            except Exception as exc:
                self._error('final context', exc)
        finally:
            if self.shadow is not None:
                self.shadow.reset('experiment_finished')
                self.shadow.private_copy = None
            # Never replace a newer owner's wrapper.
            for obj, attr, wrapper, saved in ((self.camera, 'read', self.wrapper, self.saved_read),
                    (self.node, '_sync_depth_preset', self.sync_wrapper, self.saved_sync)):
                if obj.__dict__.get(attr) is wrapper:
                    if saved[0]:
                        setattr(obj, attr, saved[1])
                    else:
                        delattr(obj, attr)
            self.timer.cancel()
            self.closed = True

    def _read_with_shadow(self, row, args, kwargs):
        """Install an owner-local tap for every real read, including unrecorded ones."""
        if self.shadow is None:
            return (self._selected_read(self.camera, row, args, kwargs) if row is not None
                    else self.original_read(*args, **kwargs))
        original = self.camera._filter_stationary_depth
        had = '_filter_stationary_depth' in self.camera.__dict__
        saved = self.camera.__dict__.get('_filter_stationary_depth')
        taps = 0
        self.shadow.last = None
        def tapped(frame, diagnostic=None, color=None, source=None):
            nonlocal taps
            # Production runs first and always returns its own untouched object.
            result = original(frame, diagnostic, color=color, source=source)
            if self.stop_reason or self.phase < 0:
                return result
            shadow_started = self.clock()
            try:
                taps += 1
                if taps != 1:
                    raise ValueError('multiple stationary taps in one production read')
                shadow_frame, audit = self.shadow.process(frame, diagnostic, color, source)
                if row is not None:
                    self._array(row, 'matched_shadow_depth', shadow_frame.get_data())
                    row['matched_stationary_shadow'] = copy.deepcopy(audit)
            except Exception as exc:
                self.shadow.reset('shadow_error')
                self._error('matched stationary shadow', exc)
                self.finish('shadow_error')
                if row is not None:
                    row['matched_stationary_shadow'] = dict(active=False, error=str(exc)[:600],
                        geometry_authority=False, original_output_replaced=False)
            finally:
                elapsed = max(0., self.clock()-shadow_started)
                phase = self.shadow.phase
                if phase is not None:
                    phase['total_tap_processing_sec'] += elapsed
                    phase['max_tap_processing_sec'] = max(phase['max_tap_processing_sec'], elapsed)
                if row is not None:
                    row['capture_overhead_sec'] += elapsed
                    audit = row.get('matched_stationary_shadow')
                    if audit is not None:
                        audit.update(tap_processing_sec=elapsed,
                            tap_processing_sec_scope='Eligibility, SDK copy/filter, audit hash and selected array copy; excludes this final timing bookkeeping.',
                            phase=copy.deepcopy(phase))
            return result
        self.camera._filter_stationary_depth = tapped
        try:
            result = (self._selected_read(self.camera, row, args, kwargs) if row is not None
                      else self.original_read(*args, **kwargs))
            diagnostic = self.camera.last_read_diagnostics
            complete = (result[0] is not None and result[1] is not None
                        and isinstance(diagnostic, dict) and diagnostic.get('result') == 'complete')
            if not complete:
                self.shadow.reset('incomplete_production_pair')
                if row is not None and 'matched_stationary_shadow' in row:
                    row['matched_stationary_shadow'].update(active=False, pair_complete=False)
            elif self.phase >= 0 and taps != 1 and not self.stop_reason:
                self.shadow.reset('missing_stationary_tap')
                self._error('matched stationary shadow', ValueError('complete read missed continuous shadow tap'))
                self.finish('shadow_error')
            return result
        except Exception:
            self.shadow.reset('production_read_exception')
            raise
        finally:
            if self.camera.__dict__.get('_filter_stationary_depth') is tapped:
                if had:
                    self.camera._filter_stationary_depth = saved
                else:
                    del self.camera._filter_stationary_depth

    def read(self, *args, **kwargs):
        with self.read_gate:
            self.inflight += 1
            row = None
            result = None
            original_invoked = False
            queued = False
            try:
                if self.closed:
                    original_invoked = True
                    return self.original_read(*args, **kwargs)
                if self.clock()-self.first_read >= self.seconds:
                    self.finish('deadline')
                if not self._owns_camera():
                    self.finish('camera_owner_changed')
                if self.shadow is not None and not self.shadow.owned():
                    self.finish('shadow_evidence_owner_changed')
                if not self.stop_reason:
                    ready = self.evidence.guard.check(self.evidence.now_ns(), freeze=True)
                    if ready and self.phase < 0:
                        self._phase_start(0)
                    elif ready and self.phase_samples == self.samples_per_phase:
                        self.phases[-1]['sensor_after'] = self.sensor_snapshot()
                        if self.phase == 2:
                            self.finish('sample_limit')
                        else:
                            self._phase_start(self.phase+1)
                if self.stop_reason:
                    self._complete_finish()
                    original_invoked = True
                    return self.original_read(*args, **kwargs)
                now = self.clock()
                selected = (self.phase >= 0 and now-self.phase_started >= self.warmup
                            and self.consecutive_complete >= self.stable_reads
                            and (self.last_sample is None or now-self.last_sample >= self.interval))
                if selected:
                    self.evidence.check_context()
                    self.evidence.guard.check(self.evidence.now_ns())
                    if self.clock()-self.first_read >= self.seconds:
                        self.finish('deadline')
                    if self.stop_reason:
                        self._complete_finish()
                        original_invoked = True
                        return self.original_read(*args, **kwargs)
                    self.attempts += 1
                    self.phase_samples += 1
                    self.last_sample = now
                    self.counts['selected'] += 1
                    row = dict(arrays={}, capture_overhead_sec=0., read_started_monotonic=now,
                               sample_index=self.attempts, phase=PHASES[self.phase][0],
                               phase_sample_index=self.phase_samples,
                               sensor_condition=PHASES[self.phase][2])
                    original_invoked = True
                    result = self._read_with_shadow(row, args, kwargs)
                else:
                    original_invoked = True
                    result = self._read_with_shadow(None, args, kwargs)
                anchor = self.evidence.now_ns()
                diagnostic = self.camera.last_read_diagnostics
                complete = (result[0] is not None and result[1] is not None
                            and isinstance(diagnostic, dict) and diagnostic.get('result') == 'complete')
                self.consecutive_complete = self.consecutive_complete+1 if complete else 0
                if row is not None:
                    # Preserve obtained raw evidence even if a guard/tap failed.
                    try:
                        row.update(anchor_ros_stamp_ns=anchor, read_completed_monotonic=self.clock(),
                                   read_completed_wall_ns=time.time_ns(),
                                   production_read_diagnostics=copy.deepcopy(diagnostic),
                                   runtime_profile=copy.deepcopy(self.camera.runtime_profile),
                                   published_camera_parameters=copy.deepcopy(self.node.cfg),
                                   depth_scale_m=float(self.camera.depth_scale),
                                   guard_at_capture=self.evidence.guard.snapshot(),
                                   array_coordinate_spaces=dict(
                                       raw_depth='native SDK depth profile', raw_rgb='native SDK color profile',
                                       aligned_depth='aligned_depth_intrinsics (SDK color)',
                                       post_spatial_depth='aligned_depth_intrinsics (SDK color)',
                                       pre_stationary_depth='aligned_depth_intrinsics (SDK color)',
                                       post_stationary_depth='aligned_depth_intrinsics (SDK color)',
                                       matched_shadow_depth='aligned_depth_intrinsics (SDK color); private diagnostic branch, not returned',
                                       final_depth='runtime_profile.geometry_intrinsics / rectification canvas',
                                       final_rgb='runtime_profile.geometry_intrinsics / rectification canvas'),
                                   timestamp_semantics='SDK timestamps are preserved; ROS anchor is host read completion, not exposure or a published header',
                                   complete=bool(complete and not row.get('capture_error')
                                                 and all(k in row['arrays'] for k in _stage.REQUIRED)
                                                 and (self.shadow is None or 'matched_shadow_depth' in row['arrays']
                                                      and not row.get('matched_stationary_shadow', {}).get('error'))))
                        if row['complete']:
                            validate_sample_geometry(row)
                    except Exception as exc:
                        row.update(anchor_ros_stamp_ns=anchor, complete=False, context_incomplete=True,
                                   metadata_error=str(exc)[:600])
                        self._error('sample finalization', exc)
                        self.finish('sample_finalization_error')
                    self.counts['complete'] += int(row['complete'])
                    try:
                        self.queue.put_nowait(row)
                        queued = True
                    except queue.Full:
                        self.counts['queue_dropped'] += 1
                        self._error('queue', ValueError('bounded diagnostic writer queue full'))
                        self.finish('queue_full')
                self.evidence.guard.check(anchor)
                return result
            except Exception as exc:
                self._error('read or preflight', exc)
                self.finish('read_or_preflight_error')
                # Diagnostic failure must not trigger production camera recovery.
                if result is not None:
                    return result
                if original_invoked:
                    self.counts['original_read_errors'] += 1
                    if row is not None and row.get('arrays') and not queued:
                        row.update(complete=False, context_incomplete=True,
                                   original_read_error=str(exc)[:600], anchor_ros_stamp_ns=None)
                        try:
                            self.queue.put_nowait(row)
                        except queue.Full:
                            self.counts['queue_dropped'] += 1
                    raise  # The actual camera read failed, without losing copied stages.
                self._complete_finish()
                return self.original_read(*args, **kwargs)
            finally:
                self.counts['reads'] += 1
                self.last_read = self.clock()
                self.inflight -= 1
                if self.stop_reason:
                    self._complete_finish()

    def _write_preserving_raw(self, row):
        # Context serialization errors must not discard owned raw arrays.
        try:
            metadata = {k:v for k,v in row.items() if k != 'arrays'}
            if len(encoded(metadata)) > 120*1024:
                raise ValueError('sample context exceeds metadata budget')
        except Exception as exc:
            self._error('sample metadata', exc)
            self.finish('sample_metadata_error')
            row = dict(arrays=row['arrays'], sample_index=row['sample_index'],
                       phase=str(row.get('phase')), complete=False,
                       anchor_ros_stamp_ns=row.get('anchor_ros_stamp_ns'),
                       context_incomplete=True, metadata_error=str(exc)[:600],
                       source_evidence=dict(evidence_complete=False),
                       timestamp_semantics='Host ROS read-completion anchor, not exposure')
        self._write_row(dict(row))

    def _writer(self):
        pair_write_failed = False
        while not self.closed or self.inflight or not self.queue.empty():
            try:
                row = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            if pair_write_failed:
                self.counts['queue_dropped'] += 1
                continue
            try:
                try:
                    row['source_evidence'] = self.evidence.at_anchor(row['anchor_ros_stamp_ns'])
                except Exception as exc:
                    row['source_evidence'] = dict(evidence_complete=False, error=str(exc)[:600])
                if not row['source_evidence']['evidence_complete']:
                    self._error('source evidence', ValueError('accepted bracket or exact anchor TF unavailable'))
                self._write_preserving_raw(row)
            except Exception as exc:
                self.counts['write_errors'] += 1
                self._error('writer', exc)
                self.finish('writer_error')
                # A failed write/fsync may already have consumed disk bytes.
                # Never attempt another pair after that ambiguous I/O result.
                pair_write_failed = True
        try:
            self.evidence.close()
        except Exception as exc:
            self._error('evidence cleanup', exc)
        summary = self.summary()
        try:
            actual_files = {p.name:p.stat().st_size for p in self.output.iterdir() if p.is_file()}
            actual_bytes = sum(actual_files.values())
            summary['actual_artifact_bytes_before_summary'] = actual_bytes
            summary['actual_files_before_summary'] = actual_files
            payload = encoded(summary)
            if len(payload) > 256*1024 or actual_bytes+len(payload) > self.maximum_bytes:
                raise ValueError('summary exceeds reserved budget')
            with (self.output/'summary.json').open('xb') as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
        except Exception as exc:
            summary['summary_write_error'] = str(exc)
        print('CAMERA_PRESET_AB '+json.dumps(summary, ensure_ascii=False), flush=True)

    def summary(self):
        faults = list(self.evidence.guard.faults)
        return dict(schema=1, diagnostic='camera_preset_A1_baseline_A2',
                    capture_complete=self.counts['written'] == self.samples and self.counts['complete'] == self.samples,
                    experiment_valid=bool(self.counts['written'] == self.samples
                        and self.counts['complete'] == self.samples and self.restore['succeeded']
                        and not self.errors and not faults and self.close_reason == 'sample_limit'),
                    counts=dict(self.counts), errors=list(self.errors), guard_faults=faults,
                    frozen_guard=copy.deepcopy(self.evidence.guard.frozen),
                    initial_context=copy.deepcopy(self.evidence.initial_context),
                    initial_sensor=self.initial_sensor, phases=self.phases, restoration=self.restore,
                    maximum_seconds=self.seconds, samples_per_phase=self.samples_per_phase,
                    warmup_seconds=self.warmup, stable_complete_pairs=self.stable_reads,
                    sample_interval_seconds=self.interval, first_read_monotonic=self.first_read,
                    last_read_monotonic=self.last_read, close_reason=self.close_reason,
                    files=list(self.files), written_bytes_before_summary=self.total_written,
                    maximum_artifact_bytes=self.maximum_bytes, queue_size=2,
                    original_read_restored=self.camera.read == self.original_read,
                    original_sync_restored=self.node._sync_depth_preset == self.original_sync,
                    camera_process_continues=True, other_nodes_controlled=False,
                    sensor_parameters_write_attempted=self.sensor_write_attempted, robot_commands_sent=False, temperature_access=False,
                    primary_comparison='raw SDK depth and aligned pre-filter depth',
                    temporal_confound='Existing stationary temporal applies only to unknown; baseline API carton branch disables it. Do not attribute final-chain differences only to matcher.',
                    matched_stationary_shadow=dict(enabled=self.shadow is not None, budget=self.shadow_budget,
                        phases=copy.deepcopy(self.shadow.phase_audits) if self.shadow is not None else [],
                        production_modified=False, policy_mode_modified=False, output_replaced=False,
                        qualifies_old_experiment=False, geometry_authority=False,
                        scope='Same eligibility algorithms and parameters across new A/B continuous inputs. Independent histories; not proof of physical accuracy or matched exposure.',
                        evidence_scope='Saved samples include actual feedback windows and RGB qualification; rolling per-read chain records continuity processing, not a replayable full RGB video.',
                        new_condition_scope='A different object or viewpoint verifies only that newly captured condition, never retroactively repairs the old A/B experiment.'),
                    clock_scope='SDK acquisition clocks are not calibrated into ROS exposure; q/TF evidence is exact to host read-completion anchor only.',
                    deadline_scope='Stops new samples by deadline; does not cancel an SDK call already in flight. Restoration runs on the camera owner after that call returns.')

    def shutdown(self):
        self.finish('shutdown')
        with self.read_gate:
            self._complete_finish()
        if self.thread.ident is not None:
            self.thread.join(timeout=3.)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enable-preset-ab', action='store_true')
    parser.add_argument('--external-consumers-quiesced', action='store_true')
    parser.add_argument('--matched-stationary-shadow', action='store_true')
    parser.add_argument('--camera-script', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--seconds', type=float, default=75.)
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('ros_args', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.enable_preset_ab and (args.output is None or not args.external_consumers_quiesced):
        parser.error('enabled experiment requires --output and --external-consumers-quiesced')
    if args.matched_stationary_shadow and not args.enable_preset_ab:
        parser.error('matched shadow requires the explicitly enabled A/B diagnostic')
    if not math.isfinite(args.seconds) or not 15 <= args.seconds <= 75 or not .5 <= args.interval <= 1.:
        parser.error('seconds must be 15..75 and interval .5..1')
    return args


def load_camera_definition(entry):
    """Load a source entry or catkin devel relay without running its main block.

    Catkin executes the real source in a separate ``context`` dictionary. Its
    returned outer namespace is the relay, not the camera module globals.
    """
    entry = Path(entry).resolve()
    outer = runpy.run_path(str(entry), run_name='camera_preset_ab_entry')
    namespace = outer
    actual_source = entry
    if 'CameraNode' not in outer or 'rospy' not in outer:
        namespace = outer.get('context')
        target = outer.get('python_script')
        if not isinstance(namespace, dict) or not isinstance(target, str):
            raise ValueError('camera entry is neither a source module nor a supported catkin relay')
        actual_source = Path(target).resolve()
        if (not isinstance(namespace.get('__file__'), str)
                or Path(namespace['__file__']).resolve() != actual_source):
            raise ValueError('catkin camera relay source/context mismatch')
    camera_class = namespace.get('CameraNode')
    init = getattr(camera_class, '__init__', None)
    globals_ = getattr(init, '__globals__', {})
    code = getattr(init, '__code__', None)
    if (not isinstance(camera_class, type) or namespace.get('rospy') is None
            or globals_.get('CameraNode') is not camera_class
            or globals_.get('rospy') is not namespace['rospy'] or code is None
            or Path(code.co_filename).resolve() != actual_source
            or Path(globals_.get('__file__', '')).resolve() != actual_source):
        raise ValueError('camera definition does not belong to the captured entry source')
    return namespace, actual_source


def main(argv=None):
    args = arguments(argv)
    script = args.camera_script.resolve()
    tail = args.ros_args[1:] if args.ros_args[:1] == ['--'] else args.ros_args
    sys.argv = [str(script)]+tail
    if not args.enable_preset_ab:
        return runpy.run_path(str(script), run_name='__main__')
    mount = Path('/media/zhuyupei/SanStand')
    output = args.output.resolve()
    root = mount/'实验记录/Robot-Controller/ros_log'
    if not mount.is_mount() or root not in output.parents or output.exists():
        raise RuntimeError('requires a new SanStand ros_log directory; no fallback')
    # Imports execute no camera entry. This remains its only camera owner.
    namespace, actual_source = load_camera_definition(script)
    rospy = namespace['rospy']
    rospy.init_node('camera_node')
    node = namespace['CameraNode']()
    observer = evidence = None
    try:
        evidence = RosEvidence(rospy)
        observer = PresetABObserver(node, output, evidence, seconds=args.seconds, interval=args.interval,
                                    matched_stationary_shadow=args.matched_stationary_shadow)
        manifest = dict(wrapper=str(Path(__file__).resolve()),
                        wrapper_sha256=hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest(),
                        reused_stage_observer_sha256=hashlib.sha256((HERE/'camera_stage_observer.py').read_bytes()).hexdigest(),
                        camera_script=str(script), camera_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
                        camera_source_script=str(actual_source),
                        camera_source_sha256=hashlib.sha256(actual_source.read_bytes()).hexdigest(),
                        external_consumers_quiesced_asserted=args.external_consumers_quiesced,
                        modules=_stage.loaded_modules())
        payload = encoded(manifest)
        if len(payload) > 64*1024:
            raise ValueError('manifest exceeds reserved budget')
        (output/'loaded_modules.json').write_bytes(payload)
        observer.total_written += len(payload)
        rospy.on_shutdown(lambda:observer.finish('ros_shutdown'))
        print('CAMERA_PRESET_AB_READY '+json.dumps(dict(output=str(output), seconds=args.seconds,
              samples=observer.samples, maximum_artifact_bytes=MAX_BYTES,
              matched_stationary_shadow=args.matched_stationary_shadow, robot_commands_sent=False)), flush=True)
    except Exception as exc:
        if observer is not None:
            observer._error('setup', exc)
            observer.shutdown()
        elif evidence is not None:
            evidence.close()
        failure = dict(error=str(exc)[:1000], experiment_valid=False, capture_complete=False,
                       robot_commands_sent=False, camera_process_continues=True)
        if observer is None:
            try:
                output.mkdir(exist_ok=True)
                with (output/'summary.json').open('xb') as stream:
                    stream.write(encoded(failure))
            except Exception as secondary:
                failure['summary_write_error'] = str(secondary)[:500]
        print('CAMERA_PRESET_AB_SETUP_FAILED '+json.dumps(failure), flush=True)
    try:
        node.spin()
    finally:
        if observer is not None:
            observer.shutdown()
        elif evidence is not None:
            evidence.close()
        node.shutdown()


if __name__ == '__main__':
    main()
