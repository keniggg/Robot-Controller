import copy
import math
from contextlib import contextmanager
import time
import warnings
from importlib import metadata

import numpy as np


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
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.align_depth_to_color = bool(align_depth_to_color)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
        self.color_projection_cfg = dict(color_projection_cfg or {})
        self.color_projection_correction = None
        self.mode_depth_preset_cfg = dict(mode_depth_preset_cfg or {})
        self.mode_depth_preset = None
        self.stationary_depth_provider = None
        self._stationary_temporal = None
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

    def start(self):
        if self.simulate:
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
            self._configure_color_projection(profile)
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
            self._stationary_temporal = None
        return changed

    def stop(self):
        try:
            if self.mode_depth_preset is not None:
                self.mode_depth_preset.close()
        finally:
            self._stop_pipeline()

    def _stop_pipeline(self):
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
            for name in ('sensor_timestamp', 'backend_timestamp',
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

    def read(self):
        started = self._diagnostic_clock()
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
                # Do not let optional metadata failures change the SDK read,
                # alignment, filter, clipping or recovery contracts.
                try:
                    diagnostic['raw_frames'] = self._raw_frame_diagnostics(frames)
                except Exception as exc:
                    diagnostic['raw_frames_unavailable'] = type(exc).__name__
            if self.align is not None:
                with self._read_stage(diagnostic, 'align'):
                    frames = self.align.process(frames)
            with self._read_stage(diagnostic, 'extract_frames'):
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                diagnostic['result'] = 'missing_frame'
                return None, None
            for index, depth_filter in enumerate(self.depth_filters):
                # Indexes are the configured filter order; names are diagnostic.
                name = (self.applied_depth_filters[index].get('name', 'unknown')
                        if index < len(self.applied_depth_filters) else 'unknown')
                with self._read_stage(diagnostic, 'depth_filter_%d_%s' % (index, name)):
                    depth_frame = depth_filter.process(depth_frame)
            depth_frame = self._filter_stationary_depth(depth_frame, diagnostic)
            with self._read_stage(diagnostic, 'color_array'):
                color = np.asanyarray(color_frame.get_data())
            if self.color_projection_correction is not None:
                with self._read_stage(diagnostic, 'color_projection'):
                    color = self.color_projection_correction.apply(color)
            with self._read_stage(diagnostic, 'depth_array_and_clip'):
                clipped = self._clip_depth_range(depth_frame.get_data())
            diagnostic['result'] = 'complete'
            # Retain one completed raw pair only, not a recursive history or
            # SDK frame handles. Failed/missing reads do not replace it.
            if 'raw_frames' in diagnostic:
                self._previous_completed_raw_frames = dict(
                    host_wait=host_wait, raw_frames=diagnostic['raw_frames'])
            return color, clipped
        finally:
            diagnostic['total_sec'] = self._diagnostic_clock() - started
            # Kept outside runtime_profile: no extra /camera parameter writes.
            self.last_read_diagnostics = diagnostic

    def _filter_stationary_depth(self, frame, diagnostic=None):
        with self._read_stage(diagnostic, 'stationary_evidence'):
            enabled = bool(self.mode_depth_preset_cfg.get('stationary_temporal_enabled', False))
            active = bool(enabled and self.mode_depth_preset is not None
                          and self.mode_depth_preset.mode == 'unknown'
                          and self.stationary_depth_provider is not None
                          and self.stationary_depth_provider())
        if enabled:
            self.runtime_profile['stationary_temporal'] = dict(
                enabled=True, active=active, alpha=.4, delta_depth_units=100,
                persistence=0, stationary_window_sec=.35, evidence_max_age_sec=.25,
                scope='unknown_only; reset on motion or missing current evidence')
        if not active:
            self._stationary_temporal = None
            return frame
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
                self._stationary_temporal = None
                raise ValueError('stationary temporal filter filled current depth holes')
        return result

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
        return dict(
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
