from dataclasses import dataclass
import math
import threading
import time
from typing import Optional


@dataclass(frozen=True)
class InferenceTicket:
    request_id: int
    generation: int
    snapshot_stamp_sec: float
    target_epoch: int
    payload: object
    submitted_monotonic_sec: float


@dataclass(frozen=True)
class SubmitDecision:
    ticket_to_start: Optional[InferenceTicket]
    pending_request_id: Optional[int]
    replaced_request_id: Optional[int]


@dataclass(frozen=True)
class CompletionDecision:
    accepted: bool
    code: str
    next_ticket: Optional[InferenceTicket]
    result_age_sec: float


class LatestOnlyInferenceCoordinator:
    def __init__(self, result_max_age_sec=1.2, clock=None):
        result_max_age_sec = float(result_max_age_sec)
        if not math.isfinite(result_max_age_sec) or result_max_age_sec <= 0.0:
            raise ValueError('result_max_age_sec must be finite and positive')
        self._result_max_age_sec = result_max_age_sec
        self._clock = clock or time.monotonic
        self._lock = threading.RLock()
        self._generation = 0
        self._next_request_id = 1
        self._running = False
        self._target_epoch = None
        self._active = None
        self._pending = None

    @property
    def pending_count(self):
        with self._lock:
            return int(self._pending is not None)

    def start(self):
        with self._lock:
            self._generation += 1
            self._running = True
            self._target_epoch = None
            self._pending = None
            return self._generation

    def stop(self):
        with self._lock:
            self._generation += 1
            self._running = False
            self._target_epoch = None
            self._pending = None
            return self._generation

    def submit(self, payload, snapshot_stamp_sec, target_epoch):
        snapshot_stamp_sec = float(snapshot_stamp_sec)
        if not math.isfinite(snapshot_stamp_sec) or snapshot_stamp_sec <= 0.0:
            raise ValueError('snapshot_stamp_sec must be finite and positive')
        target_epoch = self._validated_target_epoch(target_epoch)
        with self._lock:
            if not self._running:
                raise RuntimeError('inference coordinator is not running')
            if self._target_epoch is None:
                self._target_epoch = target_epoch
            elif target_epoch != self._target_epoch:
                raise ValueError(
                    'target_epoch does not match the current target epoch'
                )
            ticket = InferenceTicket(
                request_id=self._next_request_id,
                generation=self._generation,
                snapshot_stamp_sec=snapshot_stamp_sec,
                target_epoch=target_epoch,
                payload=payload,
                submitted_monotonic_sec=float(self._clock()),
            )
            self._next_request_id += 1
            if self._active is None:
                self._active = ticket
                return SubmitDecision(
                    ticket_to_start=ticket,
                    pending_request_id=None,
                    replaced_request_id=None,
                )
            replaced_request_id = (
                None if self._pending is None else self._pending.request_id
            )
            self._pending = ticket
            return SubmitDecision(
                ticket_to_start=None,
                pending_request_id=ticket.request_id,
                replaced_request_id=replaced_request_id,
            )

    def complete(self, ticket, now_sec=None, target_epoch=None):
        now_sec = float(self._clock() if now_sec is None else now_sec)
        if not math.isfinite(now_sec):
            raise ValueError('now_sec must be finite')
        validated_target_epoch = (
            None
            if target_epoch is None
            else self._validated_target_epoch(target_epoch)
        )
        result_age_sec = now_sec - float(ticket.snapshot_stamp_sec)
        with self._lock:
            if self._active is None or ticket.request_id != self._active.request_id:
                return CompletionDecision(
                    accepted=False,
                    code='UNKNOWN_REQUEST',
                    next_ticket=None,
                    result_age_sec=result_age_sec,
                )
            if not self._tickets_correlate(ticket, self._active):
                return CompletionDecision(
                    accepted=False,
                    code='REQUEST_MISMATCH',
                    next_ticket=None,
                    result_age_sec=result_age_sec,
                )

            self._active = None
            if not self._running or ticket.generation != self._generation:
                accepted = False
                code = 'GENERATION_STALE'
            else:
                current_target_epoch = self._target_epoch
                completion_target_epoch = (
                    current_target_epoch
                    if validated_target_epoch is None
                    else validated_target_epoch
                )
                if (
                    current_target_epoch is None
                    or ticket.target_epoch != current_target_epoch
                    or completion_target_epoch != current_target_epoch
                ):
                    accepted = False
                    code = 'TARGET_EPOCH_STALE'
                elif result_age_sec > self._result_max_age_sec:
                    accepted = False
                    code = 'RESULT_EXPIRED'
                else:
                    accepted = True
                    code = 'ACCEPTED'

            next_ticket = self._promote_pending_locked()
            return CompletionDecision(
                accepted=accepted,
                code=code,
                next_ticket=next_ticket,
                result_age_sec=result_age_sec,
            )

    def reset_target_epoch(self, target_epoch):
        target_epoch = self._validated_target_epoch(target_epoch)
        with self._lock:
            self._target_epoch = target_epoch
            self._pending = None

    def _promote_pending_locked(self):
        pending = self._pending
        self._pending = None
        if (
            not self._running
            or pending is None
            or pending.generation != self._generation
            or pending.target_epoch != self._target_epoch
        ):
            return None
        self._active = pending
        return pending

    @staticmethod
    def _tickets_correlate(first, second):
        return (
            first.request_id == second.request_id
            and first.generation == second.generation
            and first.snapshot_stamp_sec == second.snapshot_stamp_sec
            and first.target_epoch == second.target_epoch
        )

    @staticmethod
    def _validated_target_epoch(target_epoch):
        if isinstance(target_epoch, bool):
            raise ValueError('target_epoch must be a non-negative integer')
        try:
            value = int(target_epoch)
        except (TypeError, ValueError, OverflowError):
            raise ValueError('target_epoch must be a non-negative integer')
        if value != target_epoch or value < 0:
            raise ValueError('target_epoch must be a non-negative integer')
        return value
