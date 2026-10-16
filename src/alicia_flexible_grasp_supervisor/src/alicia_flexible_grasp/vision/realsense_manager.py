import copy
import json
import math
from contextlib import contextmanager
import time
import warnings
from importlib import metadata

import numpy as np

from .visual_stationary import VisualStationaryWindow
from .pair_history_continuity import (frame_profile_signature,
    interleaved_pair_reason, successor_pair_reason)


def _parameter_pair_history(record):
    """Detached ROS XML-RPC diagnostic, preserving typed raw read evidence.

    ROS parameters accept signed 32-bit integers and no null values. SDK
    timestamps routinely exceed that integer range. Exact decimal strings
    preserve these values; explicit paths distinguish absent/null fields.
    No filter or continuity decision reads this representation.
    """
    omitted, list_nulls = [], []

    def convert(value, path):
        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                child = path+[str(key)]
                if item is None:
                    omitted.append(child)
                else:
                    result[key] = convert(item, child)
            return result
        if isinstance(value, (list, tuple)):
            result = []
            for index, item in enumerate(value):
                child = path+[str(index)]
                if item is None:
                    list_nulls.append(child)
                    result.append('')
                else:
                    result.append(convert(item, child))
            return result
        if isinstance(value, int) and not isinstance(value, bool):
            return str(value) if not -(2**31) <= value < 2**31 else value
        return copy.deepcopy(value)

    result = convert(record, [])
    result['parameter_encoding'] = dict(
        large_integers='exact_decimal_string_outside_signed_int32',
        omitted_null_fields=omitted, empty_string_list_nulls=list_nulls,
        typed_raw_source='last_read_diagnostics.stationary_pair_history')
    return result


class _DepthArrayFrame:
    """Array-only result for the existing clip/rectify tail, never an SDK input."""
    def __init__(self, array):
        self._array = array

    def get_data(self):
        return self._array


class RealSenseManager:
    def __init__(
        self,
        width=640,
        height=480,
        fps=30,
        align_depth_to_color=True,
        simulate=False,
        depth_filter_cfg=None,
        color_projection_cfg=None,
        mode_depth_preset_cfg=None,
        pair_rectification_cfg=None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.align_depth_to_color = bool(align_depth_to_color)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
        self.color_projection_cfg = dict(color_projection_cfg or {})
        self.color_projection_correction = None
        self.pair_rectification_cfg = dict(pair_rectification_cfg or {})
        self.pair_rectifier = None
        self.mode_depth_preset_cfg = dict(mode_depth_preset_cfg or {})
        if type(self.mode_depth_preset_cfg.get('regional_stationary_enabled', False)) is not bool:
            raise ValueError('regional_stationary_enabled must be a boolean')
        self.mode_depth_preset = None
        self.stationary_depth_provider = None
        self._stationary_temporal = None
        self._stationary_visual_anchor = None
        self._regional_stationary = None
        self._sdk_color_alignment_owner = None
        self._visual_stationary = VisualStationaryWindow()
        self.pipeline = None
        self.align = None
        self.rs = None
        self.depth_filters = []
        self.applied_depth_filters = []
        self.runtime_profile = {}
        self.depth_scale = 0.0001
        self.depth_scale_source = 'simulation_constant' if self.simulate else 'default'
        self.depth_min_m = float(self.depth_filter_cfg.get('depth_min_m', 0.03))
        self.depth_max_m = float(self.depth_filter_cfg.get('depth_max_m', 2.0))
        self.t = 0
        self._diagnostic_clock = time.monotonic
        self._diagnostic_wall_clock = time.time
        self._diagnostic_thread_clock = getattr(time, 'thread_time', None)
        self.last_read_diagnostics = None
        self._previous_completed_raw_frames = None
        self._stationary_history_epoch = 0
        self._last_stationary_pair = None
        self._stationary_pair_quarantine = None
        self._stationary_pair_last_decision = None

    def start(self):
        if self.simulate:
            if self.pair_rectification_cfg.get('enabled', False):
                raise RuntimeError('paired rectification requires the recorded real SDK geometry')
            if self.color_projection_cfg.get('enabled', False):
                raise RuntimeError('calibrated color projection requires the recorded real camera')
            return True
        try:
            import pyrealsense2 as rs
            self.rs = rs
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            profile = self.pipeline.start(config)
            self.depth_scale = self._read_depth_scale(profile)
            if self.align_depth_to_color:
                self.align = rs.align(rs.stream.color)
                self._sdk_color_alignment_owner = self.align
            self._configure_color_projection(profile)
            self._configure_pair_rectification(profile)
            self._configure_depth_filters()
            self.runtime_profile = self._read_runtime_profile(profile)
            if self.mode_depth_preset_cfg.get("enabled", False):
                from .depth_preset import ModeDepthPreset
                self.mode_depth_preset = ModeDepthPreset(
                    rs, profile.get_device(), self.mode_depth_preset_cfg, self.depth_scale)
            return True
        except Exception as exc:
            raise RuntimeError('Failed to start RealSense: %s' % exc)

    def set_depth_preset_mode(self, mode):
        if self.mode_depth_preset is None:
            return False
        changed = self.mode_depth_preset.apply(mode)
        if changed:
            self.runtime_profile['mode_depth_preset'] = self.mode_depth_preset.evidence()
            self._reset_stationary_history()
        return changed

    def stop(self):
        try:
            if self.mode_depth_preset is not None:
                self.mode_depth_preset.close()
        finally:
            self._stop_pipeline()

    def _reset_stationary_history(self, reason='history_reset'):
        if self._stationary_pair_quarantine is not None:
            self._pair_history_decision(None, 'discarded', reason,
                                        self._stationary_pair_quarantine)
        self._stationary_history_epoch += 1
        self._last_stationary_pair = None
        self._stationary_pair_quarantine = None
        self._stationary_temporal = None
        self._stationary_visual_anchor = None
        self._visual_stationary.reset()
        if self._regional_stationary is not None:
            self._regional_stationary.reset()
        status = self.runtime_profile.get('stationary_temporal')
        if isinstance(status, dict):
            status['active'] = False
            status['history_warming_up'] = False
            if status.get('regional_enabled'):
                status.update(regional_output_active=False, fully_active=False,
                    filtered_fraction=0., filtered_pixel_count=0, regions=[], history_reset=True)

    def _regional_history_scope(self):
        return (not self.simulate
                and self.mode_depth_preset_cfg.get('regional_stationary_enabled') is True
                and self.mode_depth_preset_cfg.get('stationary_temporal_enabled') is True
                and self.mode_depth_preset is not None
                and self.mode_depth_preset.mode == 'unknown'
                and callable(self.stationary_depth_provider)
                and self.align_depth_to_color and self.align is not None
                and self.align is self._sdk_color_alignment_owner)

    def _stationary_history_owner(self):
        """Private identity/configuration token; never hardware evidence."""
        if not self._regional_history_scope() or self._regional_stationary is None:
            return None
        regions = getattr(self._regional_stationary, '_regions', ())
        if not regions:
            return None
        try:
            configuration = json.dumps(dict(
                depth_filters=self.depth_filter_cfg, applied=self.applied_depth_filters,
                preset=self.mode_depth_preset_cfg, projection=self.color_projection_cfg,
                rectification=self.pair_rectification_cfg,
                size=[self.width, self.height, self.fps],
                depth=[self.depth_scale, self.depth_min_m, self.depth_max_m],
                geometry={key: self.runtime_profile.get(key) for key in
                    ('color_intrinsics', 'native_depth_intrinsics', 'depth_to_color_extrinsics',
                     'rectification', 'color_projection_correction')}),
                sort_keys=True, allow_nan=False)
            return (self._stationary_history_epoch, id(self.pipeline), id(self.rs),
                id(self.align), id(self._sdk_color_alignment_owner),
                tuple(id(value) for value in self.depth_filters),
                id(self.mode_depth_preset), self.mode_depth_preset.mode,
                id(self.pair_rectifier), id(self.color_projection_correction),
                id(self.stationary_depth_provider), id(self._regional_stationary),
                getattr(self._regional_stationary, '_profile', None),
                getattr(self._regional_stationary, '_shape', None),
                getattr(self._regional_stationary, '_previous_source', None),
                tuple((id(row['visual']), id(row['visual']._anchor),
                       id(row['filter']), id(row['anchor'])) for row in regions),
                configuration)
        except (AttributeError, KeyError, TypeError, ValueError):
            return None

    def _pair_history_decision(self, diagnostic, state, reason, context, current=None):
        record = dict(state=state, reason=reason, diagnostic_only=True,
            authorizes_pair_or_motion=False, history_epoch=self._stationary_history_epoch,
            last_complete_raw_frames=copy.deepcopy(context.get('raw_frames')),
            last_complete_filter_host_monotonic=context.get('filter_host'),
            rejected_raw_frames=copy.deepcopy(context.get('rejected_raw_frames')),
            latest_rejected_raw_frames=copy.deepcopy(context.get('latest_rejected_raw_frames')),
            first_rejection_host_monotonic=context.get('first_rejection_host'),
            rejected_packet_count=context.get('rejected_packet_count', 0),
            next_accepted_raw_frames=copy.deepcopy(current))
        self._stationary_pair_last_decision = record
        if diagnostic is not None:
            diagnostic['stationary_pair_history'] = copy.deepcopy(record)
        status = self.runtime_profile.get('stationary_temporal')
        if isinstance(status, dict):
            status['pair_history'] = _parameter_pair_history(record)
            if state == 'quarantined':
                # No current output is available on a rejected read. Private
                # filter objects stay parked; old activity is not advertised.
                status.update(regional_output_active=False, fully_active=False,
                              filtered_pixel_count=0, filtered_fraction=0., active=False)

    def _quarantine_interleaved_pair(self, diagnostic):
        if not self._regional_history_scope():
            return False
        previous = self._stationary_pair_quarantine or self._last_stationary_pair
        if previous is None:
            return False
        now = self._diagnostic_clock()
        rejection = dict(previous,
            rejected_raw_frames=copy.deepcopy(previous.get('rejected_raw_frames', diagnostic.get('raw_frames'))),
            latest_rejected_raw_frames=copy.deepcopy(diagnostic.get('raw_frames')),
            first_rejection_host=previous.get('first_rejection_host', now),
            rejected_packet_count=previous.get('rejected_packet_count', 0)+1)
        reason = ('history_owner_changed' if previous['owner'] != self._stationary_history_owner()
                  else interleaved_pair_reason(previous['raw_frames'], diagnostic.get('raw_frames'),
                                               previous['filter_host'], now))
        if reason is not None:
            self._pair_history_decision(diagnostic, 'discarded', reason, rejection)
            return False
        self._stationary_pair_quarantine = parked = rejection
        # Neither the last complete source nor its host deadline is renewed.
        self._pair_history_decision(diagnostic, 'quarantined',
                                    'pre_filter_interleaved_sdk_packet', parked)
        return True

    def _resolve_pair_quarantine(self, diagnostic):
        parked = self._stationary_pair_quarantine
        if parked is None:
            return
        reason = ('history_owner_changed' if parked['owner'] != self._stationary_history_owner()
                  else successor_pair_reason(parked['raw_frames'], diagnostic.get('raw_frames'),
                                             parked['filter_host'], self._diagnostic_clock()))
        self._stationary_pair_quarantine = None
        if reason is not None:
            self._reset_stationary_history(reason)
            self._pair_history_decision(diagnostic, 'discarded', reason, parked,
                                        diagnostic.get('raw_frames'))
        else:
            # Existing encoder and per-region visual gates still run below,
            # with their original anchors, counters and host/device clocks.
            self._pair_history_decision(diagnostic, 'resumed_for_existing_gates',
                                        'both_raw_streams_are_immediate_successors', parked,
                                        diagnostic.get('raw_frames'))

    def _remember_stationary_pair(self, diagnostic):
        owner = self._stationary_history_owner()
        raw = diagnostic.get('raw_frames')
        host = diagnostic.get('stationary_pair_filter_host_monotonic')
        if owner is None or host is None or not isinstance(raw, dict):
            self._last_stationary_pair = None
            return
        # Missing profile evidence can never be borrowed from another frame.
        if any(not isinstance(raw.get(stream), dict)
               or 'profile_signature' not in raw[stream] for stream in ('color', 'depth')):
            self._last_stationary_pair = None
            return
        self._last_stationary_pair = dict(owner=owner, raw_frames=copy.deepcopy(raw), filter_host=host)

    def _stop_pipeline(self):
        self._reset_stationary_history()
        self._sdk_color_alignment_owner = None
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass

    @contextmanager
    def _read_stage(self, diagnostic, name):
        # No runtime-profile mutations or I/O. Wall time includes scheduling
        # inside the call, so it is not a device-only/CPU-time measurement.
        if diagnostic is None:
            yield
            return
        started = self._diagnostic_clock()
        try:
            yield
        except Exception:
            if diagnostic['failed_stage'] is None:
                diagnostic['failed_stage'] = name
            raise
        finally:
            elapsed = self._diagnostic_clock() - started
            stages = diagnostic['stages_sec']
            stages[name] = stages.get(name, 0.) + elapsed

    @staticmethod
    def _diagnostic_number(callback):
        # Optional clock/SDK evidence cannot turn a good frame into a failure.
        try:
            value = float(callback())
            return value if math.isfinite(value) else None
        except Exception:
            return None

    def _host_wait_snapshot(self):
        return dict(
            monotonic_sec=self._diagnostic_number(self._diagnostic_clock),
            wall_time_sec=self._diagnostic_number(self._diagnostic_wall_clock),
            thread_cpu_sec=self._diagnostic_number(self._diagnostic_thread_clock))

    @staticmethod
    def _diagnostic_elapsed(start, end):
        return None if start is None or end is None else end - start

    def _raw_frame_diagnostics(self, frames):
        # Inspect only the just-returned raw frames, before align/filtering.
        # These SDK timestamps retain their original domain; they are NOT ROS
        # exposure timestamps or a device-to-host clock calibration. Metadata
        # values are SDK integers with field-specific units, kept verbatim.
        result = {}
        for stream in ('color', 'depth'):
            entry = dict(present=None, metadata_raw={}, unavailable={})
            result[stream] = entry
            try:
                frame = getattr(frames, 'get_%s_frame' % stream)()
                entry['present'] = bool(frame)
            except Exception as exc:
                entry['unavailable']['frame'] = type(exc).__name__
                continue
            if not entry['present']:
                continue
            for key, method, convert in (
                    ('frame_number', 'get_frame_number', int),
                    ('device_timestamp_ms', 'get_timestamp', float),
                    ('timestamp_domain', 'get_frame_timestamp_domain', str)):
                try:
                    value = convert(getattr(frame, method)())
                    if isinstance(value, float) and not math.isfinite(value):
                        raise ValueError('nonfinite diagnostic')
                    entry[key] = value
                except Exception as exc:
                    entry['unavailable'][key] = type(exc).__name__
            try:
                fps = float(frame.get_profile().fps())
                if not math.isfinite(fps):
                    raise ValueError('nonfinite stream FPS')
                entry['stream_fps'] = fps
            except Exception as exc:
                entry['unavailable']['stream_fps'] = type(exc).__name__
            if self._regional_history_scope():
                try:
                    entry['profile_signature'] = frame_profile_signature(frame)
                except Exception as exc:
                    # Optional for unchanged normal read behavior, mandatory
                    # before retaining any history across a rejected packet.
                    entry['unavailable']['profile_signature'] = type(exc).__name__
            for name in ('frame_counter', 'frame_timestamp',
                         'sensor_timestamp', 'backend_timestamp',
                         'time_of_arrival', 'actual_fps'):
                try:
                    field = getattr(self.rs.frame_metadata_value, name)
                    if frame.supports_frame_metadata(field):
                        entry['metadata_raw'][name] = int(frame.get_frame_metadata(field))
                    else:
                        entry['unavailable'][name] = 'unsupported'
                except Exception as exc:
                    entry['unavailable'][name] = type(exc).__name__
        return result

    @staticmethod
    def _pair_timing(raw_frames):
        # This is a pair-validity policy, not a hardware-sync assertion or a
        # device-to-ROS clock calibration. Half the slower stream's nominal
        # period is the nearest-frame tolerance (30/30 FPS => 16.67 ms).
        # It intentionally rejects older frames reused by the SDK aggregator.
        result = dict(accepted=False, reason=None,
                      policy='half_slowest_stream_period', stream_fps={})
        if not isinstance(raw_frames, dict):
            result['reason'] = 'raw_frames_unavailable'
            return result
        timestamps, domains = {}, {}
        for stream in ('color', 'depth'):
            frame = raw_frames.get(stream, {})
            if frame.get('present') is not True:
                result['reason'] = stream + '_frame_unavailable'
                return result
            stamp = frame.get('device_timestamp_ms')
            if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp < 0:
                result['reason'] = stream + '_timestamp_unavailable'
                return result
            timestamps[stream] = stamp
            domain = frame.get('timestamp_domain', '')
            # Enum string spelling has differed across SDK/Python versions.
            domains[stream] = str(domain).rsplit('.', 1)[-1]
            if domains[stream] not in ('hardware_clock', 'global_time'):
                # system_time is the SDK host-clock fallback. Close host
                # arrivals do not establish when the two images were acquired.
                result['reason'] = stream + '_timestamp_domain_unusable'
                return result
            fps = frame.get('stream_fps')
            if not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0:
                result['reason'] = stream + '_stream_fps_unavailable'
                return result
            result['stream_fps'][stream] = fps
        result['timestamp_domains'] = domains
        if domains['color'] != domains['depth']:
            result['reason'] = 'timestamp_domain_mismatch'
            return result
        result['skew_ms'] = abs(timestamps['color'] - timestamps['depth'])
        result['max_skew_ms'] = 500. / min(result['stream_fps'].values())
        if result['skew_ms'] > result['max_skew_ms']:
            result['reason'] = 'timestamp_skew_exceeded'
            return result
        result['accepted'] = True
        result['reason'] = 'within_nominal_pair_tolerance'
        return result

    def read(self):
        started = self._diagnostic_clock()
        quarantine_this_read = False
        diagnostic = dict(schema=2, result='error', failed_stage=None, stages_sec={},
                          previous_completed_raw_frames=self._previous_completed_raw_frames)
        self.last_read_diagnostics = None
        try:
            if self.simulate:
                with self._read_stage(diagnostic, 'simulate'):
                    color, depth = self._simulate()
                with self._read_stage(diagnostic, 'depth_array_and_clip'):
                    clipped = self._clip_depth_range(depth)
                diagnostic['result'] = 'complete'
                return color, clipped
            host_wait = diagnostic['host_wait'] = {}
            with self._read_stage(diagnostic, 'wait_for_frames'):
                host_wait['enter'] = self._host_wait_snapshot()
                try:
                    frames = self.pipeline.wait_for_frames()
                finally:
                    host_wait['exit'] = self._host_wait_snapshot()
                    for name, clock in (('elapsed_sec', 'monotonic_sec'),
                                        ('thread_cpu_elapsed_sec', 'thread_cpu_sec')):
                        host_wait[name] = self._diagnostic_elapsed(
                            host_wait['enter'][clock], host_wait['exit'][clock])
            with self._read_stage(diagnostic, 'raw_frame_diagnostics'):
                # Optional metadata is best effort; acquisition timestamp,
                # domain and actual stream FPS are required for pair validity.
                try:
                    diagnostic['raw_frames'] = self._raw_frame_diagnostics(frames)
                except Exception as exc:
                    diagnostic['raw_frames_unavailable'] = type(exc).__name__
            with self._read_stage(diagnostic, 'pair_timing'):
                pair_timing = self._pair_timing(diagnostic.get('raw_frames'))
                diagnostic['pair_timing'] = pair_timing
            if not pair_timing['accepted']:
                raw = diagnostic.get('raw_frames', {})
                missing = any(raw.get(stream, {}).get('present') is False
                              for stream in ('color', 'depth'))
                diagnostic['result'] = 'missing_frame' if missing else 'rejected_pair'
                if pair_timing.get('reason') == 'timestamp_skew_exceeded':
                    quarantine_this_read = self._quarantine_interleaved_pair(diagnostic)
                # No exception: an unusable pair is dropped, not a camera read
                # failure that could trigger the node's automatic restart path.
                return None, None
            self._resolve_pair_quarantine(diagnostic)
            diagnostic['sdk_color_alignment_applied'] = False
            if self.align is not None:
                with self._read_stage(diagnostic, 'align'):
                    alignment_owner = self.align
                    frames = alignment_owner.process(frames)
                    diagnostic['sdk_color_alignment_applied'] = bool(
                        self.align_depth_to_color and alignment_owner is self._sdk_color_alignment_owner
                        and self.align is alignment_owner)
            with self._read_stage(diagnostic, 'extract_frames'):
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                diagnostic['result'] = 'missing_frame'
                return None, None
            # Motion evidence uses this pair's raw color pixels, before optional
            # projection correction, and the SDK source identity read above.
            with self._read_stage(diagnostic, 'color_array'):
                color = np.asanyarray(color_frame.get_data())
            for index, depth_filter in enumerate(self.depth_filters):
                # Indexes are the configured filter order; names are diagnostic.
                name = (self.applied_depth_filters[index].get('name', 'unknown')
                        if index < len(self.applied_depth_filters) else 'unknown')
                with self._read_stage(diagnostic, 'depth_filter_%d_%s' % (index, name)):
                    depth_frame = depth_filter.process(depth_frame)
            depth_frame = self._filter_stationary_depth(
                depth_frame, diagnostic, color=color,
                source=diagnostic.get('raw_frames', {}).get('color'))
            if self.color_projection_correction is not None:
                with self._read_stage(diagnostic, 'color_projection'):
                    color = self.color_projection_correction.apply(color)
            with self._read_stage(diagnostic, 'depth_array_and_clip'):
                clipped = self._clip_depth_range(depth_frame.get_data())
            if self.pair_rectifier is not None:
                with self._read_stage(diagnostic, 'pair_rectification'):
                    color, clipped = self.pair_rectifier.apply(color, clipped)
            diagnostic['result'] = 'complete'
            self._remember_stationary_pair(diagnostic)
            # Retain one completed raw pair only, not a recursive history or
            # SDK frame handles. Failed/missing reads do not replace it.
            if 'raw_frames' in diagnostic:
                self._previous_completed_raw_frames = dict(
                    host_wait=host_wait, raw_frames=diagnostic['raw_frames'])
            return color, clipped
        finally:
            if diagnostic['result'] != 'complete' and not quarantine_this_read:
                self._reset_stationary_history('read_'+diagnostic['result'])
                decision = diagnostic.get('stationary_pair_history')
                if decision and decision.get('state') == 'resumed_for_existing_gates':
                    # A later align/filter/rectify failure must not leave a
                    # successful continuity decision looking like good output.
                    decision.update(state='discarded', reason='read_'+diagnostic['result'],
                                    failed_stage=diagnostic.get('failed_stage'))
                if decision:
                    # The unconditional reset must not replace an exact
                    # discontinuity reason with the generic read result.
                    self._stationary_pair_last_decision = copy.deepcopy(decision)
                    status = self.runtime_profile.get('stationary_temporal')
                    if isinstance(status, dict):
                        status['pair_history'] = _parameter_pair_history(decision)
            diagnostic['total_sec'] = self._diagnostic_clock() - started
            # Kept outside runtime_profile: no extra /camera parameter writes.
            self.last_read_diagnostics = diagnostic

    def _filter_stationary_depth(self, frame, diagnostic=None, color=None, source=None):
        with self._read_stage(diagnostic, 'stationary_evidence'):
            enabled = bool(self.mode_depth_preset_cfg.get('stationary_temporal_enabled', False))
            active = bool(enabled and self.mode_depth_preset is not None
                          and self.mode_depth_preset.mode == 'unknown'
                          and self.stationary_depth_provider is not None
                          and self.stationary_depth_provider())
        encoder_active = active
        if self.mode_depth_preset_cfg.get('regional_stationary_enabled', False) is True:
            # Opt-in has its own independent regional histories. A prior global
            # history cannot be transferred into them or resumed afterwards.
            self._stationary_temporal = None
            self._stationary_visual_anchor = None
            self._visual_stationary.reset()
            return self._filter_regional_stationary_depth(frame, diagnostic, color, source,
                                                           enabled, encoder_active)
        if self._regional_stationary is not None:
            self._reset_stationary_history()
            self._regional_stationary = None
        with self._read_stage(diagnostic, 'visual_stationary_evidence'):
            if encoder_active:
                visual = self._visual_stationary.observe(color, source, self._diagnostic_clock())
            else:
                self._visual_stationary.reset()
                visual = dict(qualified=False, reason='encoder_or_mode_inactive')
            active = bool(encoder_active and visual['qualified'])
            warming = bool(encoder_active and visual.get('reason') == 'warming_up')
            if diagnostic is not None:
                diagnostic['stationary_rgb_evidence'] = visual
                diagnostic['stationary_temporal_history_warming_up'] = warming and not active
        if enabled:
            self.runtime_profile['stationary_temporal'] = dict(
                enabled=True, active=active, history_warming_up=warming and not active,
                alpha=.4, delta_depth_units=100,
                persistence=0, stationary_window_sec=.35, evidence_max_age_sec=.25,
                visual_stationarity_required=True,
                visual_window_sec=VisualStationaryWindow.WINDOW_SEC,
                visual_max_motion_full_resolution_px=VisualStationaryWindow.MAX_MOTION_PX,
                scope='unknown_only; encoder AND continuous SDK RGB evidence; reset on motion or missing evidence')
        anchor = self._visual_stationary._anchor
        if not (active or warming) or anchor is None:
            self._stationary_temporal = None
            self._stationary_visual_anchor = None
            return frame
        # VisualStationaryWindow seeds a fresh immutable anchor on every reset
        # or failed motion/track/continuity check. Private depth history belongs
        # to exactly that anchor, including its unqualified warmup interval.
        if self._stationary_visual_anchor is not anchor:
            self._stationary_temporal = None
            self._stationary_visual_anchor = anchor
        if self._stationary_temporal is None:
            with self._read_stage(diagnostic, 'stationary_temporal_init'):
                temporal = self.rs.temporal_filter()
                temporal.set_option(self.rs.option.filter_smooth_alpha, .4)
                temporal.set_option(self.rs.option.filter_smooth_delta, 100.)
                temporal.set_option(self.rs.option.holes_fill, 0.)
                self._stationary_temporal = temporal
        with self._read_stage(diagnostic, 'stationary_temporal_process'):
            result = self._stationary_temporal.process(frame)
        # Persistence=0 must never reconstruct a depth hole from history.
        with self._read_stage(diagnostic, 'stationary_hole_check'):
            invalid = np.asanyarray(frame.get_data()) == 0
            if np.any(np.asanyarray(result.get_data())[invalid] != 0):
                self._reset_stationary_history()
                raise ValueError('stationary temporal filter filled current depth holes')
        # The SDK creates a distinct output frame. Warming only accumulates
        # private history; the original current frame remains the sole output
        # until BOTH unchanged stationary gates have fully qualified.
        return result if active else frame

    def _new_regional_temporal_filter(self, *, alpha, delta, persistence):
        # Regional ownership changes eligibility, never the SDK filter policy.
        if (alpha, delta, persistence) != (.4, 100, 0):
            raise ValueError('regional stationary temporal policy changed')
        temporal = self.rs.temporal_filter()
        temporal.set_option(self.rs.option.filter_smooth_alpha, alpha)
        temporal.set_option(self.rs.option.filter_smooth_delta, delta)
        temporal.set_option(self.rs.option.holes_fill, persistence)
        return temporal

    def _filter_regional_stationary_depth(self, frame, diagnostic, color, source, enabled, encoder_active):
        from .regional_stationary_depth import RegionalStationaryDepth
        if self._regional_stationary is None:
            self._regional_stationary = RegionalStationaryDepth(self._new_regional_temporal_filter)
        # Equal dimensions alone cannot show which RGB rays own aligned depth.
        # The exact SDK align(color) instance created at start must have run
        # successfully for this read and must still be the current owner.
        aligned = bool(self.align_depth_to_color and self.align is not None
            and self.align is self._sdk_color_alignment_owner
            and isinstance(diagnostic, dict)
            and diagnostic.get('sdk_color_alignment_applied') is True)
        with self._read_stage(diagnostic, 'regional_stationary_depth'):
            filter_host = self._diagnostic_clock()
            if diagnostic is not None:
                diagnostic['stationary_pair_filter_host_monotonic'] = filter_host
            array, evidence = self._regional_stationary.process(frame, color, source,
                filter_host, encoder_active, aligned_to_color=aligned)
        if diagnostic is not None:
            diagnostic['regional_stationary_evidence'] = copy.deepcopy(evidence)
        # The legacy active bit means full-frame visual qualification. Regional
        # activity is explicit; one qualified tile must never masquerade as it.
        self.runtime_profile['stationary_temporal'] = dict(
            enabled=enabled, active=False, regional_enabled=True,
            regional_output_active=bool(evidence['active']),
            whole_frame_stationarity_qualified=False, fully_active=bool(evidence['fully_active']),
            filtered_pixel_count=evidence['filtered_pixel_count'], filtered_fraction=evidence['filtered_fraction'],
            owner_schema=evidence['owner_schema'], regions=copy.deepcopy(evidence['regions']),
            history_warming_up=any(row.get('warming') is True for row in evidence['regions']),
            alignment_proven=aligned, alpha=.4, delta_depth_units=100, persistence=0,
            stationary_window_sec=.35, evidence_max_age_sec=.25, visual_stationarity_required=True,
            visual_window_sec=VisualStationaryWindow.WINDOW_SEC,
            visual_max_motion_full_resolution_px=VisualStationaryWindow.MAX_MOTION_PX,
            scope='unknown_only; independently qualified regional ownership; legacy full-frame gate not claimed')
        if self._stationary_pair_last_decision is not None:
            decision = copy.deepcopy(self._stationary_pair_last_decision)
            current = diagnostic.get('stationary_pair_history') if diagnostic is not None else None
            if current is not None and current.get('state') == 'resumed_for_existing_gates':
                decision['existing_gate_evaluation'] = dict(
                    encoder_active=bool(encoder_active), alignment_proven=aligned,
                    filtered_pixel_count=evidence['filtered_pixel_count'],
                    regional_global_reset_reason=evidence.get('global_history_reset_reason'),
                    regions=[{key: row.get(key) for key in
                        ('name', 'filter_applied', 'qualified', 'warming', 'history_reset')}
                        for row in evidence['regions']])
                diagnostic['stationary_pair_history'] = copy.deepcopy(decision)
                self._stationary_pair_last_decision = copy.deepcopy(decision)
            self.runtime_profile['stationary_temporal']['pair_history'] = _parameter_pair_history(decision)
        return _DepthArrayFrame(array) if evidence['filtered_pixel_count'] else frame

    def _configure_color_projection(self, profile):
        self.color_projection_correction = None
        cfg = self.color_projection_cfg
        if not cfg.get('enabled', False):
            return
        if not self.align_depth_to_color:
            raise ValueError('calibrated color projection requires SDK depth-to-color alignment')
        from .color_projection import ColorProjectionCorrection

        serial = profile.get_device().get_info(self.rs.camera_info.serial_number)
        if not cfg.get('device_serial') or str(cfg['device_serial']) != str(serial):
            raise ValueError('color projection calibration does not match camera serial')
        intrinsic = profile.get_stream(self.rs.stream.color).as_video_stream_profile().get_intrinsics()
        actual = dict(width=intrinsic.width, height=intrinsic.height,
                      fx=intrinsic.fx, fy=intrinsic.fy, cx=intrinsic.ppx, cy=intrinsic.ppy,
                      model=str(intrinsic.model).split('.')[-1], coeffs=list(intrinsic.coeffs))
        expected = cfg['sdk_intrinsics']
        for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy'):
            if not np.isclose(float(expected[key]), float(actual[key]), rtol=0, atol=1e-6):
                raise ValueError('color projection SDK profile changed: ' + key)
        if (str(expected['model']).split('.')[-1] != actual['model'] or
                not np.allclose(expected['coeffs'], actual['coeffs'], rtol=0, atol=1e-9)):
            raise ValueError('color projection SDK distortion profile changed')
        self.color_projection_correction = ColorProjectionCorrection(cfg['measured_intrinsics'], actual)

    @staticmethod
    def _intrinsics_dict(stream_profile):
        intrinsic = stream_profile.as_video_stream_profile().get_intrinsics()
        result = dict(width=int(intrinsic.width), height=int(intrinsic.height),
                      fx=float(intrinsic.fx), fy=float(intrinsic.fy),
                      cx=float(intrinsic.ppx), cy=float(intrinsic.ppy),
                      model=str(intrinsic.model).split('.')[-1],
                      coeffs=[float(value) for value in intrinsic.coeffs])
        numeric = [result[key] for key in ('width', 'height', 'fx', 'fy', 'cx', 'cy')]
        if (not np.all(np.isfinite(numeric + result['coeffs']))
                or min(numeric[:4]) <= 0 or len(result['coeffs']) != 5):
            raise ValueError('invalid active RealSense stream intrinsics')
        return result

    def _configure_pair_rectification(self, profile):
        self.pair_rectifier = None
        enabled = self.pair_rectification_cfg.get('enabled', False)
        if type(enabled) is not bool:
            raise ValueError('paired rectification enabled must be a boolean')
        if not enabled:
            return
        if not self.align_depth_to_color:
            raise ValueError('paired rectification requires SDK depth-to-color alignment')
        # The legacy independent RGB calibration is a different source mapping.
        # No unproved composition or double RGB resampling is admitted here.
        if self.color_projection_correction is not None:
            raise ValueError('paired rectification with legacy color projection is not supported')
        from .rgbd_rectification import PairedPinholeRectification
        color = self._intrinsics_dict(profile.get_stream(self.rs.stream.color))
        self.pair_rectifier = PairedPinholeRectification(color, self.rs)

    def _read_runtime_profile(self, profile):
        """Describe the active SDK geometry; never write calibration to the device."""
        color_stream = profile.get_stream(self.rs.stream.color)
        depth_stream = profile.get_stream(self.rs.stream.depth)
        color = self._intrinsics_dict(color_stream)
        depth = self._intrinsics_dict(depth_stream)
        extrinsic = depth_stream.get_extrinsics_to(color_stream)
        rotation = [float(value) for value in extrinsic.rotation]
        translation = [float(value) for value in extrinsic.translation]
        if (len(rotation) != 9 or len(translation) != 3
                or not np.all(np.isfinite(rotation + translation))):
            raise ValueError('invalid active RealSense depth-to-color extrinsics')
        device = profile.get_device()
        info = {}
        for key in ('serial_number', 'firmware_version'):
            try:
                info[key] = str(device.get_info(getattr(self.rs.camera_info, key)))
            except Exception:
                info[key] = 'unavailable'
        sdk_version = getattr(self.rs, '__version__', '')
        if not sdk_version:
            try:
                sdk_version = metadata.version('pyrealsense2')
            except metadata.PackageNotFoundError:
                sdk_version = 'unavailable'
        source = ('sdk_active_color_profile' if self.align_depth_to_color
                  else 'sdk_active_native_depth_profile')
        correction = {'enabled': self.color_projection_correction is not None}
        if correction['enabled']:
            correction.update(measured_intrinsics=copy.deepcopy(self.color_projection_cfg['measured_intrinsics']),
                              sdk_intrinsics=copy.deepcopy(self.color_projection_cfg['sdk_intrinsics']))
        result = dict(
            schema_version=1, source='realsense_active_profile', sdk_version=str(sdk_version),
            device=info, color_intrinsics=color, native_depth_intrinsics=depth,
            depth_to_color_extrinsics=dict(rotation_column_major=rotation, translation_m=translation),
            alignment=dict(enabled=self.align_depth_to_color,
                           target_stream='color' if self.align_depth_to_color else 'depth'),
            geometry_intrinsics=copy.deepcopy(color if self.align_depth_to_color else depth),
            geometry_intrinsics_source=source, depth_scale_m_per_unit=float(self.depth_scale),
            depth_scale_source=self.depth_scale_source,
            depth_filters=dict(requested=copy.deepcopy(self.depth_filter_cfg),
                               applied=copy.deepcopy(self.applied_depth_filters),
                               range_m=[self.depth_min_m, self.depth_max_m]),
            color_projection_correction=correction,
        )
        if self.pair_rectifier is not None:
            result['geometry_intrinsics'] = copy.deepcopy(self.pair_rectifier.output_intrinsics)
            result['geometry_intrinsics_source'] = 'sdk_color_rectified_pinhole_canvas'
            result['rectification'] = copy.deepcopy(self.pair_rectifier.contract)
        return result

    def _clip_depth_range(self, depth):
        clipped = np.asanyarray(depth).copy()
        depth_scale = float(self.depth_scale)
        if np.issubdtype(clipped.dtype, np.integer) and depth_scale > 0.0:
            # RealSense depth is uint16. Compare in raw units so the 30 FPS
            # acquisition path avoids a full-frame float64 allocation.
            min_raw = int(np.ceil(self.depth_min_m / depth_scale - 1e-9))
            max_raw = int(np.floor(self.depth_max_m / depth_scale + 1e-9))
            clipped[(clipped < min_raw) | (clipped > max_raw)] = 0
        else:
            depth_m = clipped.astype(np.float64) * depth_scale
            clipped[(depth_m < self.depth_min_m) | (depth_m > self.depth_max_m)] = 0
        return clipped

    def _configure_depth_filters(self):
        self.depth_filters = []
        self.applied_depth_filters = []
        cfg = dict(self.depth_filter_cfg or {})
        if bool(cfg.get('spatial_enabled', True)):
            spatial = None
            option_name = 'constructor'
            try:
                spatial = self.rs.spatial_filter()
                values = (
                    ('filter_magnitude', int(cfg.get('spatial_magnitude', 2))),
                    ('filter_smooth_alpha', float(cfg.get('spatial_smooth_alpha', 0.5))),
                    ('filter_smooth_delta', float(cfg.get('spatial_smooth_delta', 20))),
                )
                for option_name, value in values:
                    spatial.set_option(getattr(self.rs.option, option_name), value)
            except Exception as exc:
                self._warn_filter_disabled('spatial', option_name, exc)
            else:
                self.depth_filters.append(spatial)
                self.applied_depth_filters.append(dict(name='spatial', options=dict(values)))
        if bool(cfg.get('temporal_enabled', False)):
            self._append_simple_filter('temporal', 'temporal_filter')
        if bool(cfg.get('hole_filling_enabled', False)):
            self._append_simple_filter('hole_filling', 'hole_filling_filter')

    def _append_simple_filter(self, filter_name, constructor_name):
        try:
            constructor = getattr(self.rs, constructor_name)
            depth_filter = constructor()
        except Exception as exc:
            self._warn_filter_disabled(filter_name, 'constructor', exc)
        else:
            self.depth_filters.append(depth_filter)
            self.applied_depth_filters.append(dict(name=filter_name, options={}))

    @staticmethod
    def _warn_filter_disabled(filter_name, option_name, exc):
        warnings.warn(
            'RealSense depth filter disabled: %s/%s: %s'
            % (filter_name, option_name, exc)
        )

    def _read_depth_scale(self, profile):
        try:
            sensor = profile.get_device().first_depth_sensor()
            scale = float(sensor.get_depth_scale())
            self.depth_scale_source = 'sdk_depth_sensor'
            return scale
        except Exception:
            # Preserve the existing recovery behavior, but do not label this
            # fallback as a fresh hardware measurement in the runtime record.
            self.depth_scale_source = 'previous_value_fallback'
            return float(self.depth_scale)

    def _simulate(self):
        self.t += 1
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        x = int((self.t*5) % self.width)
        y = self.height//2
        img[:, :, 1] = 30
        img[max(0,y-40):min(self.height,y+40), max(0,x-40):min(self.width,x+40), :] = [0, 180, 0]
        depth = np.ones((self.height, self.width), dtype=np.uint16) * 6000
        return img, depth
