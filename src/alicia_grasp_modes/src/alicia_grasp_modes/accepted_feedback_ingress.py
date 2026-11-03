"""Bounded SDK feedback ingress independent of perception/planning locks."""
from collections import deque
import threading

import numpy as np


class FeedbackIngressError(ValueError):
    def __init__(self, reason, diagnostic):
        super().__init__(reason)
        self.diagnostic = diagnostic


class AcceptedFeedbackIngress:
    def __init__(self, maxlen=128):
        self._lock = threading.Lock()
        self._entries = deque(maxlen=maxlen)
        self._sequence = 0
        self._checked = {}

    def clear(self):
        with self._lock:
            self._entries.clear()
            self._checked.clear()

    def append(self, row):
        """Import an already decoded accepted row (offline/internal callers)."""
        ns, values = row
        q = np.array(values, dtype=float, copy=True)
        if type(ns) is not int or ns <= 0 or q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('invalid_feedback')
        q.setflags(write=False)
        with self._lock:
            self._sequence += 1
            self._entries.append((self._sequence, ns, q, None, ns, None))

    def ingest(self, msg, received_ns, received_monotonic):
        if msg.header.frame_id != 'sdk_measured':
            return
        ns = 0; q = None; error = None
        try:
            ns = msg.header.stamp.to_nsec()
            names = [str(name).lower() for name in msg.name]
            if ns <= 0 or len(names) != len(set(names)) or len(names) != len(msg.position):
                raise ValueError('invalid_feedback')
            values = dict(zip(names, msg.position))
            q = np.asarray([values['joint%d' % i] for i in range(1, 7)], dtype=float)
            if q.shape != (6,) or not np.isfinite(q).all():
                raise ValueError('invalid_feedback')
            q.setflags(write=False)
        except (ValueError, KeyError, TypeError, AttributeError, OverflowError):
            error = 'invalid_feedback'; q = None
        with self._lock:
            self._sequence += 1
            self._entries.append((self._sequence, ns, q, error,
                                  int(received_ns), float(received_monotonic)))

    def snapshot(self, *, owner=None, epoch_ns=0, since_ns=0):
        """Snapshot and validate journal continuity for an exact owner.

        A consumer cursor advances only after inspecting all new entries.
        Losing uninspected entries is a sticky failure for that owner, even
        when the latest q has returned to the original station.
        """
        with self._lock:
            entries = tuple(self._entries)
            rows = [(ns, q) for _, ns, q, error, _, _ in entries
                    if error is None and ns >= epoch_ns]
            valid = [row for row in entries if row[3] is None and row[1] >= epoch_ns]
            diagnostic = dict(buffer_count=len(entries), sequence=self._sequence,
                latest_source_stamp_ns=max((ns for ns, _ in rows), default=0),
                latest_receive_ros_ns=valid[-1][4] if valid else 0,
                latest_receive_monotonic=valid[-1][5] if valid else None,
                control_epoch_ns=epoch_ns, since_ns=since_ns)
            if owner is not None:
                scope, binding = owner
                previous = self._checked.get(scope)
                if previous is None or previous['binding'] != binding:
                    previous = dict(binding=binding,
                        sequence=entries[0][0]-1 if entries else self._sequence,
                        fault=None)
                    self._checked[scope] = previous
                diagnostic['previous_validated_sequence'] = previous['sequence']
                if entries and previous['sequence'] < entries[0][0]-1:
                    previous['fault'] = previous['fault'] or 'feedback_buffer_overrun'
                if any(error and seq > previous['sequence'] and received >= since_ns
                       for seq, ns, q, error, received, mono in entries):
                    previous['fault'] = previous['fault'] or 'invalid_feedback'
                if previous['fault'] is not None:
                    raise FeedbackIngressError(previous['fault'], diagnostic)
                previous['sequence'] = self._sequence
            return rows, diagnostic

    def mark_fault(self, owner, reason):
        with self._lock:
            scope, binding = owner
            previous = self._checked.get(scope)
            if previous is not None and previous['binding'] == binding:
                previous['fault'] = previous['fault'] or reason

    def __iter__(self):
        return iter(self.snapshot()[0])
