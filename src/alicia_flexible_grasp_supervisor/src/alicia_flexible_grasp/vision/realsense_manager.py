import time
import warnings

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
    ):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.align_depth_to_color = bool(align_depth_to_color)
        self.simulate = bool(simulate)
        self.depth_filter_cfg = dict(depth_filter_cfg or {})
        self.pipeline = None
        self.align = None
        self.rs = None
        self.depth_filters = []
        self.depth_scale = 0.0001
        self.depth_min_m = float(self.depth_filter_cfg.get('depth_min_m', 0.03))
        self.depth_max_m = float(self.depth_filter_cfg.get('depth_max_m', 2.0))
        self.t = 0

    def start(self):
        if self.simulate:
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
            self._configure_depth_filters()
            return True
        except Exception as exc:
            raise RuntimeError('Failed to start RealSense: %s' % exc)

    def stop(self):
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
        color = np.asanyarray(color_frame.get_data())
        return color, self._clip_depth_range(depth_frame.get_data())

    def _clip_depth_range(self, depth):
        clipped = np.asanyarray(depth).copy()
        depth_m = clipped.astype(np.float64) * float(self.depth_scale)
        clipped[(depth_m < self.depth_min_m) | (depth_m > self.depth_max_m)] = 0
        return clipped

    def _configure_depth_filters(self):
        self.depth_filters = []
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

    @staticmethod
    def _warn_filter_disabled(filter_name, option_name, exc):
        warnings.warn(
            'RealSense depth filter disabled: %s/%s: %s'
            % (filter_name, option_name, exc)
        )

    def _read_depth_scale(self, profile):
        try:
            sensor = profile.get_device().first_depth_sensor()
            return float(sensor.get_depth_scale())
        except Exception:
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
