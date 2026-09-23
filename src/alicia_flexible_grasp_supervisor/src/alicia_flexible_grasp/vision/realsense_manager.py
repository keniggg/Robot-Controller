import copy
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

    def read(self):
        if self.simulate:
            color, depth = self._simulate()
            return color, self._clip_depth_range(depth)
        frames = self.pipeline.wait_for_frames()
        if self.align is not None:
            frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        for depth_filter in self.depth_filters:
            depth_frame = depth_filter.process(depth_frame)
        depth_frame = self._filter_stationary_depth(depth_frame)
        color = np.asanyarray(color_frame.get_data())
        if self.color_projection_correction is not None:
            color = self.color_projection_correction.apply(color)
        return color, self._clip_depth_range(depth_frame.get_data())

    def _filter_stationary_depth(self, frame):
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
            temporal = self.rs.temporal_filter()
            temporal.set_option(self.rs.option.filter_smooth_alpha, .4)
            temporal.set_option(self.rs.option.filter_smooth_delta, 100.)
            temporal.set_option(self.rs.option.holes_fill, 0.)
            self._stationary_temporal = temporal
        result = self._stationary_temporal.process(frame)
        # Persistence=0 must never reconstruct a depth hole from history.
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
