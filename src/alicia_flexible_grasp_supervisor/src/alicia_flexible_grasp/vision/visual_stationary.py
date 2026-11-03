"""Bounded RGB evidence for depth-history use; independent of ROS and the SDK.

This is a conservative permission gate, not a motion estimator or a calibration.
An unqualified result leaves the current image usable. Coordinates and thresholds
are in full input-image pixels, even though tracking uses a <=320px image.
"""
import math
import numbers

import cv2
import numpy as np


class VisualStationaryWindow:
    WINDOW_SEC = .35
    MAX_MOTION_PX = .25
    MAX_FORWARD_BACKWARD_PX = .5
    MAX_SIDE = 320
    MAX_POINTS = 64
    MIN_POINTS = 24

    def __init__(self):
        self.reset()

    def reset(self):
        self._previous = None
        self._anchor = None
        self._points = None
        self._anchor_stamp = None
        self._anchor_host = None
        self._count = 0

    @staticmethod
    def _coverage(points, shape):
        xy = points.reshape(-1, 2)
        cells = (xy[:, 0] >= shape[1] / 2).astype(int) + 2 * (xy[:, 1] >= shape[0] / 2)
        return int(sum(np.count_nonzero(cells == cell) >= 4 for cell in range(4)))

    def _seed(self, gray, stamp, host):
        # Per-quadrant quotas prevent a small textured patch from monopolizing
        # the bounded feature budget. Feature discovery happens only on reset.
        points = []
        h, w = gray.shape
        for row in range(2):
            for col in range(2):
                y0, y1 = row * h // 2, (row + 1) * h // 2
                x0, x1 = col * w // 2, (col + 1) * w // 2
                found = cv2.goodFeaturesToTrack(gray[y0:y1, x0:x1], 16, .02, 8,
                                                blockSize=5)
                if found is not None:
                    found += np.float32([x0, y0])
                    points.append(found)
        pts = np.concatenate(points) if points else np.empty((0, 1, 2), np.float32)
        if len(pts) < self.MIN_POINTS or self._coverage(pts, gray.shape) < 3:
            self._anchor = self._points = None
            return False
        self._anchor = gray.copy()
        self._points = pts
        self._anchor_stamp = stamp
        self._anchor_host = host
        self._count = 1
        return True

    def observe(self, color, source, host_monotonic_sec):
        """Observe the current raw SDK color pixels and their exact source metadata.

        ``source`` must contain frame_number, device_timestamp_ms,
        timestamp_domain, and stream_fps. The host monotonic clock detects
        acquisition/processing pauses; it is never substituted for source time.
        All missing/malformed evidence and OpenCV failures revoke permission.
        """
        report = dict(qualified=False, reason='invalid_evidence',
                      window_sec=self.WINDOW_SEC, max_motion_px=self.MAX_MOTION_PX,
                      max_points=self.MAX_POINTS, tracked_points=0,
                      coordinate_units='full_resolution_pixels')
        try:
            if not isinstance(source, dict):
                raise ValueError('source')
            number = source.get('frame_number')
            stamp = source.get('device_timestamp_ms')
            fps = source.get('stream_fps')
            domain = str(source.get('timestamp_domain', '')).rsplit('.', 1)[-1]
            if (isinstance(number, bool) or not isinstance(number, numbers.Integral)
                    or number < 0 or domain not in ('hardware_clock', 'global_time')):
                raise ValueError('source_identity')
            for value in (stamp, fps, host_monotonic_sec):
                if (isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real)
                        or not math.isfinite(value)):
                    raise ValueError('source_clock')
            if stamp < 0 or not 1 <= fps <= 240 or host_monotonic_sec < 0:
                raise ValueError('source_clock_range')
            if (not isinstance(color, np.ndarray) or color.dtype != np.uint8
                    or color.ndim not in (2, 3)
                    or color.ndim == 3 and color.shape[2] != 3
                    or min(color.shape[:2]) < 48 or max(color.shape[:2]) > 8192):
                raise ValueError('image')
            height, width = color.shape[:2]
            ratio = min(1., self.MAX_SIDE / float(max(height, width)))
            small = cv2.resize(color, (max(1, round(width * ratio)), max(1, round(height * ratio))),
                               interpolation=cv2.INTER_AREA) if ratio < 1 else color
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY) if small.ndim == 3 else small
            scale = np.float32([width / float(gray.shape[1]), height / float(gray.shape[0])])
            current = (int(number), float(stamp), domain, float(fps), float(host_monotonic_sec),
                       tuple(color.shape))
            report.update(frame_number=int(number), device_timestamp_ms=float(stamp),
                          timestamp_domain=domain, stream_fps=float(fps))
            discontinuous = False
            if self._previous is not None:
                prev = self._previous
                dt = (stamp - prev[1]) / 1000.
                host_dt = host_monotonic_sec - prev[4]
                discontinuous = (number != prev[0] + 1 or domain != prev[2] or fps != prev[3]
                    or current[5] != prev[5] or not .25 / fps <= dt <= 1.75 / fps
                    or not 0 < host_dt <= max(.1, 2.5 / fps))
                if discontinuous:
                    self.reset()
            self._previous = current
            if self._anchor is None:
                seeded = self._seed(gray, stamp, host_monotonic_sec)
                report['reason'] = ('source_discontinuity' if discontinuous else 'warming_up') if seeded else 'insufficient_texture'
                return report
            kwargs = dict(winSize=(15, 15), maxLevel=2,
                          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, .01))
            forward, status, _ = cv2.calcOpticalFlowPyrLK(self._anchor, gray, self._points, None, **kwargs)
            if forward is None or status is None or not np.isfinite(forward).all():
                raise ValueError('tracking')
            backward, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, self._anchor, forward, None, **kwargs)
            if backward is None or reverse is None or not np.isfinite(backward).all():
                raise ValueError('tracking')
            fb = np.linalg.norm((backward - self._points).reshape(-1, 2) * scale, axis=1)
            xy = forward.reshape(-1, 2)
            valid = ((status.ravel() == 1) & (reverse.ravel() == 1)
                     & (fb <= self.MAX_FORWARD_BACKWARD_PX)
                     & (xy[:, 0] >= 0) & (xy[:, 0] < gray.shape[1])
                     & (xy[:, 1] >= 0) & (xy[:, 1] < gray.shape[0]))
            tracked = int(valid.sum())
            coverage = self._coverage(forward[valid], gray.shape)
            report.update(tracked_points=tracked, covered_quadrants=coverage)
            if tracked < max(self.MIN_POINTS, math.ceil(.8 * len(self._points))) or coverage < 3:
                self._seed(gray, stamp, host_monotonic_sec)
                report['reason'] = 'insufficient_tracks'
                return report
            motion = np.linalg.norm((forward - self._points).reshape(-1, 2)[valid] * scale, axis=1)
            motion90 = float(np.percentile(motion, 90))
            report.update(motion_p90_px=motion90, motion_median_px=float(np.median(motion)))
            if motion90 > self.MAX_MOTION_PX:
                self._seed(gray, stamp, host_monotonic_sec)
                report['reason'] = 'visual_motion'
                return report
            self._count += 1
            span = (stamp - self._anchor_stamp) / 1000.
            host_span = host_monotonic_sec - self._anchor_host
            report.update(quiet_span_sec=span, host_quiet_span_sec=host_span, continuous_frames=self._count)
            # Buffered frames replayed quickly after a stall are not a fresh
            # stationary interval, even if their source clock is continuous.
            report['qualified'] = bool(min(span, host_span) >= self.WINDOW_SEC and self._count >= 4)
            report['reason'] = 'stationary' if report['qualified'] else 'warming_up'
            return report
        except (ValueError, TypeError, OverflowError, cv2.error):
            self.reset()
            return report
