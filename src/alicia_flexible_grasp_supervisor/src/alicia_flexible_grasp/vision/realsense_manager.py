import time
import numpy as np

class RealSenseManager:
    def __init__(self, width=640, height=480, fps=30, align_depth_to_color=True, simulate=False):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.align_depth_to_color = bool(align_depth_to_color)
        self.simulate = bool(simulate)
        self.pipeline = None
        self.align = None
        self.rs = None
        self.depth_scale = 0.001
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
            return self._simulate()
        frames = self.pipeline.wait_for_frames()
        if self.align is not None:
            frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        return color, depth

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
        depth = np.ones((self.height, self.width), dtype=np.uint16) * 600
        return img, depth
