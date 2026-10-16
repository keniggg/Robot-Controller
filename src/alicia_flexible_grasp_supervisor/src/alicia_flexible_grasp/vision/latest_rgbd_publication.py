"""Bounded ROS-independent delivery of the latest complete RGB-D observation.

The reader owns acquisition, filtering and admission. This module only copies
already processed pixels and runs a publication callable on another thread. It
does not provide stationary evidence, change timestamps or operate a device.
"""
from dataclasses import dataclass
import math
import numbers
import threading
import time

import numpy as np


def _finite_number(value, name):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real)
            or not math.isfinite(value)):
        raise ValueError(name + ' must be a finite number')
    return float(value)


@dataclass(frozen=True)
class RGBDSnapshot:
    """One independently owned pair; the stamp is a host read-completion stamp.

    The arrays have immutable bytes backing, rather than merely a mutable SDK
    buffer with its writeable flag cleared. Their backing cannot be made
    writeable again through NumPy. The host stamp is not an exposure timestamp.
    """
    color: np.ndarray
    depth: np.ndarray
    read_completed_monotonic: float
    host_read_completed_stamp_ns: int

    @classmethod
    def copy_pair(cls, color, depth, read_completed_monotonic,
                  host_read_completed_stamp_ns, maximum_bytes):
        completed = _finite_number(read_completed_monotonic, 'read completion')
        stamp = host_read_completed_stamp_ns
        if completed < 0:
            raise ValueError('read completion must be nonnegative')
        if (isinstance(stamp, (bool, np.bool_)) or not isinstance(stamp, numbers.Integral)
                or stamp <= 0):
            raise ValueError('host read-completion stamp must be positive integer nanoseconds')
        if (not isinstance(color, np.ndarray) or color.dtype != np.uint8
                or color.ndim != 3 or color.shape[2] != 3
                or not isinstance(depth, np.ndarray) or depth.dtype != np.uint16
                or depth.ndim != 2 or color.shape[:2] != depth.shape
                or min(depth.shape) <= 0):
            raise ValueError('an aligned uint8 BGR / uint16 depth pair is required')
        if color.nbytes + depth.nbytes > maximum_bytes:
            raise ValueError('RGB-D snapshot exceeds its memory bound')
        # tobytes performs the only pixel copy, including for noncontiguous
        # input views. frombuffer/reshape keep only those independent bytes.
        owned_color = np.frombuffer(color.tobytes(order='C'), dtype=color.dtype).reshape(color.shape)
        owned_depth = np.frombuffer(depth.tobytes(order='C'), dtype=depth.dtype).reshape(depth.shape)
        return cls(owned_color, owned_depth, completed, int(stamp))


class LatestRGBDPublication:
    """One pending whole pair plus at most one pair inside the publisher.

    ``submit`` copies outside the short queue lock and never waits for the
    publisher. False means closed/failed admission; malformed input raises
    ValueError. ``publish(snapshot)`` may block, but never holds that lock.
    ``close`` discards pending work, waits at most one second for an in-flight
    call, and reports its actual worker state; it cannot cancel a callable.
    """
    def __init__(self, publish, monotonic=time.monotonic, max_age_sec=.15,
                 maximum_snapshot_bytes=16 * 1024 * 1024,
                 thread_name='latest-rgbd-publication'):
        age = _finite_number(max_age_sec, 'maximum age')
        if not 0 < age <= .15:
            raise ValueError('maximum age must be positive and at most 0.15 seconds')
        if (isinstance(maximum_snapshot_bytes, (bool, np.bool_))
                or not isinstance(maximum_snapshot_bytes, numbers.Integral)
                or maximum_snapshot_bytes <= 0):
            raise ValueError('snapshot memory bound must be a positive integer')
        if not callable(publish) or not callable(monotonic):
            raise ValueError('publisher and monotonic clock must be callable')
        self._publish = publish
        self._monotonic = monotonic
        self.max_age_sec = age
        self.maximum_snapshot_bytes = int(maximum_snapshot_bytes)
        self._condition = threading.Condition()
        self._copy_lock = threading.Lock()
        self._pending = None
        self._inflight = False
        self._publishing = False
        self._generation = 0
        self._closing = False
        self._failure = None
        self._counts = dict(submitted=0, replaced_pending=0, published=0,
                            rejected_after_failure=0, rejected_after_close=0,
                            rejected_busy=0, rejected_generation_change=0,
                            dropped_expired=0, dropped_future=0,
                            dropped_invalid_clock=0, dropped_on_failure=0,
                            dropped_on_close=0, dropped_discarded=0, publish_failures=0)
        self._peak_pair_bytes = 0
        self._last_publish_seconds = None
        self._maximum_publish_seconds = 0.
        self._total_publish_seconds = 0.
        self._worker = threading.Thread(target=self._run, name=thread_name, daemon=True)
        self._worker.start()

    def _admission_open(self):
        # Caller holds condition. Failed admission takes precedence over close.
        if self._failure is not None:
            self._counts['rejected_after_failure'] += 1
            return False
        if self._closing:
            self._counts['rejected_after_close'] += 1
            return False
        return True

    def submit(self, color, depth, read_completed_monotonic,
               host_read_completed_stamp_ns):
        with self._condition:
            if not self._admission_open():
                return False
            generation = self._generation
        # A single camera reader normally never contends here. Concurrent
        # producers are rejected, rather than allocating unbounded pixel
        # copies or blocking behind another producer's memory copy.
        if not self._copy_lock.acquire(blocking=False):
            with self._condition:
                self._counts['rejected_busy'] += 1
            return False
        try:
            snapshot = RGBDSnapshot.copy_pair(color, depth, read_completed_monotonic,
                                             host_read_completed_stamp_ns,
                                             self.maximum_snapshot_bytes)
            with self._condition:
                # Failure/shutdown/generation changes during the copy cannot
                # admit the old pair into the new stream or mode generation.
                if not self._admission_open():
                    return False
                if generation != self._generation:
                    self._counts['rejected_generation_change'] += 1
                    return False
                if self._pending is not None:
                    self._counts['replaced_pending'] += 1
                self._pending = (snapshot, generation)
                self._counts['submitted'] += 1
                self._peak_pair_bytes = max(self._peak_pair_bytes,
                                            snapshot.color.nbytes + snapshot.depth.nbytes)
                self._condition.notify()
            return True
        finally:
            self._copy_lock.release()

    def discard_pending(self):
        """Revoke queued/selected work for a stream or mode generation change.

        A publisher already admitted cannot be cancelled. This method never
        waits for it; the generation check also revokes a selected pair that
        has not yet passed the final publication check. Return the number of
        pending pairs cleared immediately (zero or one).
        """
        with self._condition:
            self._generation += 1
            cleared = int(self._pending is not None)
            self._pending = None
            self._counts['dropped_discarded'] += cleared
            self._condition.notify_all()
            return cleared

    def _latch_failure(self, error):
        # Caller holds condition; no SDK or external callback is invoked here.
        if self._failure is None:
            self._failure = (type(error).__name__ + ': ' + str(error))[:2048]
        if self._pending is not None:
            self._pending = None
            self._counts['dropped_on_failure'] += 1

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._closing and self._failure is None:
                    self._condition.wait()
                if self._closing or self._failure is not None:
                    return
                snapshot, generation = self._pending
                self._pending = None
                self._inflight = True
            if not self._check_before_publish(snapshot, generation, final=False):
                with self._condition:
                    failed = self._failure is not None
                if failed:
                    return
                continue
            # Recheck immediately before admitting the callable; a successful
            # dequeue-time check alone is not a freshness guarantee.
            if not self._check_before_publish(snapshot, generation, final=True):
                with self._condition:
                    failed = self._failure is not None
                if failed:
                    return
                continue
            # Keep a real duration clock independent of an injected source-age
            # clock. A stopped/fake clock must not erase publication latency.
            started = time.monotonic()
            failure = None
            try:
                self._publish(snapshot)
            except BaseException as error:
                failure = error
            elapsed = time.monotonic() - started
            with self._condition:
                self._last_publish_seconds = elapsed
                self._maximum_publish_seconds = max(self._maximum_publish_seconds, elapsed)
                self._total_publish_seconds += elapsed
                self._inflight = self._publishing = False
                if failure is None:
                    self._counts['published'] += 1
                else:
                    self._counts['publish_failures'] += 1
                    self._latch_failure(failure)
                self._condition.notify_all()
            if failure is not None:
                return

    def _check_before_publish(self, snapshot, generation, final):
        try:
            now = _finite_number(self._monotonic(), 'monotonic clock')
        except BaseException as error:
            with self._condition:
                self._counts['dropped_invalid_clock'] += 1
                self._inflight = False
                self._latch_failure(error)
                self._condition.notify_all()
            return False
        age = now - snapshot.read_completed_monotonic
        with self._condition:
            if self._closing:
                reason = 'dropped_on_close'
            elif generation != self._generation:
                reason = 'dropped_discarded'
            elif age < 0:
                reason = 'dropped_future'
            elif age > self.max_age_sec:
                reason = 'dropped_expired'
            else:
                if final:
                    self._publishing = True
                return True
            self._counts[reason] += 1
            self._inflight = False
            self._condition.notify_all()
            return False

    def summary(self):
        with self._condition:
            return dict(self._counts, pending=int(self._pending is not None),
                        inflight=self._inflight, publishing=self._publishing, closing=self._closing,
                        failure=self._failure, worker_alive=self._worker.is_alive(),
                        shutdown_complete=self._closing and not self._worker.is_alive(),
                        max_age_sec=self.max_age_sec,
                        maximum_snapshot_bytes=self.maximum_snapshot_bytes,
                        max_pending_pairs=1, max_pair_bytes=self.maximum_snapshot_bytes,
                        peak_pair_bytes=self._peak_pair_bytes,
                        last_publish_seconds=self._last_publish_seconds,
                        maximum_publish_seconds=self._maximum_publish_seconds,
                        total_publish_seconds=self._total_publish_seconds)

    def close(self, timeout=1.):
        limit = _finite_number(timeout, 'close timeout')
        if limit < 0:
            raise ValueError('close timeout must be nonnegative')
        with self._condition:
            self._closing = True
            if self._pending is not None:
                self._pending = None
                self._counts['dropped_on_close'] += 1
            self._condition.notify_all()
        if threading.current_thread() is not self._worker:
            self._worker.join(min(limit, 1.))
        return self.summary()
