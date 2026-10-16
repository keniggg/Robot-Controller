"""Accepted encoder and transmitted-command evidence for stationary filtering."""
from collections import deque
import math
import threading

import numpy as np


class StationaryDepthWindow:
    ARM = tuple('Joint%d' % i for i in range(1, 7))
    FRAMES = {'sdk': 'sdk_transmitted', 'accepted': 'sdk_measured'}

    def __init__(self):
        self.lock = threading.Lock()
        self.history = {key: deque(maxlen=24) for key in self.FRAMES}

    def observe(self, kind, message):
        with self.lock:
            rows = self.history[kind]
            try:
                stamp = int(message.header.stamp.to_nsec())
                names, positions = tuple(message.name), tuple(message.position)
                expected = self.ARM + (('right_finger',) if kind == 'accepted' else ())
                if (message.header.frame_id != self.FRAMES[kind] or stamp <= 0
                        or len(names) != len(positions) or len(set(names)) != len(names)
                        or set(names) != set(expected)):
                    raise ValueError('invalid motion evidence')
                mapping = dict(zip(names, positions))
                values = tuple(float(mapping[name]) for name in expected)
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('nonfinite motion evidence')
                if rows and stamp <= rows[-1][0]:
                    raise ValueError('nonmonotonic motion evidence')
                rows.append((stamp, values))
            except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
                rows.clear()

    def stationary(self, now_sec):
        if not math.isfinite(now_sec) or now_sec <= 0:
            return False
        with self.lock:
            snapshots = {key: tuple(rows) for key, rows in self.history.items()}
        for kind, rows in snapshots.items():
            if not rows or not 0 <= now_sec - rows[-1][0]*1e-9 <= .25:
                return False
            boundary = int((now_sec - .35)*1e9)
            before = [i for i, row in enumerate(rows) if row[0] <= boundary]
            if not before:
                return False
            window = rows[before[-1]:]
            if (len(window) < 3 or any((b[0]-a[0])*1e-9 > .25
                                      for a, b in zip(window, window[1:]))):
                return False
            values = np.asarray([row[1] for row in window])
            tolerance = (0. if kind == 'sdk' else
                         np.asarray([2*math.pi/4096 + 1e-12]*6 + [.00025]))
            if np.any(np.ptp(values, axis=0) > tolerance):
                return False
        return True
