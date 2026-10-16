from collections import deque
import math
import time

class LowPassArray:
    def __init__(self, alpha=0.35):
        self.alpha = float(alpha)
        self.last = None
    def update(self, values):
        values = [float(v) for v in values]
        if self.last is None or len(self.last) != len(values):
            self.last = values[:]
        else:
            a = self.alpha
            self.last = [a*v + (1-a)*p for v, p in zip(values, self.last)]
        return self.last[:]


def split_values(values, mode='half'):
    if mode == 'single':
        return values, []
    mid = len(values)//2
    return values[:mid], values[mid:]


def contact_center(values, rows=5, cols=12):
    if not values:
        return 0.0, 0.0, 0, 0.0
    max_val = max(values)
    max_idx = int(values.index(max_val))
    total = float(sum(values))
    if total <= 1e-6:
        return 0.0, 0.0, max_idx, float(max_val)
    # If split has fewer points than rows*cols, adapt cols.
    n = len(values)
    c = cols if rows*cols == n else max(1, int(n / max(1, rows)))
    sx = sy = 0.0
    for i, v in enumerate(values):
        x = i % c
        y = i // c
        sx += x * v
        sy += y * v
    return sx/total, sy/total, max_idx, float(max_val)

class SlipDetector:
    def __init__(self, window_sec=0.25, drop_ratio=0.30, center_shift_threshold=0.0):
        self.window_sec = float(window_sec)
        self.drop_ratio = float(drop_ratio)
        self.center_shift_threshold = float(center_shift_threshold)
        self.buf = deque()

    def update(self, force, centers=None):
        now = time.time()
        center_tuple = tuple(float(v) for v in centers) if centers is not None else None
        self.buf.append((now, float(force), center_tuple))
        while self.buf and now - self.buf[0][0] > self.window_sec:
            self.buf.popleft()
        if len(self.buf) < 3:
            return False
        peak = max(item[1] for item in self.buf)
        force_drop = peak > 1e-6 and force < peak * (1.0 - self.drop_ratio)
        center_motion = False
        if center_tuple is not None and self.center_shift_threshold > 0.0:
            previous = next((item for item in self.buf if item[2] is not None), None)
            if previous is not None and now - previous[0] >= self.window_sec * 0.9:
                center_motion = math.sqrt(sum(
                    (current - old) ** 2
                    for current, old in zip(center_tuple, previous[2])
                )) >= self.center_shift_threshold
        return force_drop or center_motion
