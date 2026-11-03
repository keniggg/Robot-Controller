"""Bounded, non-authorizing station callback/failure diagnostics.

No measurement/contract is validated or changed here. Diagnostics are always
best effort; the original station retirement runs before failure logging.
"""
from collections import deque
from functools import wraps
import json
import threading
import time
import weakref

_TLS = threading.local()
_HISTORY = weakref.WeakKeyDictionary()
_TASK_HISTORY = weakref.WeakKeyDictionary()
_HISTORY_LOCK = threading.Lock()
_MAX_HISTORY = 32
_MAX_FRAMES = 8
_CONTRACT_FIELDS = ('source_ns', 'geometry_deadline_ns', 'identity_deadline_ns',
                    'session_deadline_ns', 'phase_deadline_ns')
_VALIDATOR_FRAMES = frozenset(('validate_contract', 'validate_registration',
    'validate_recovery_parent', 'validate_station_pending_notice',
    'validate_station_feedback_recovery_notice', 'validate_station_pending_input',
    '_station_pending_envelope', '_station_private_pending_input',
    '_station_private_pending_reconcile', '_station_private_pending_feedback_repeat',
    '_station_private_pending_feedback_cause', '_station_compute_context',
    '_station_task_join', '_station_task_context', '_station_task_check', 'bracket_feedback'))


def _integer(value):
    return value if type(value) is int else None


def _contract(value):
    if not isinstance(value, dict):
        return None
    if 'contract' in value and isinstance(value['contract'], dict):
        value = value['contract']
    if (type(value.get('source_ns')) is not int
            or type(value.get('identity_deadline_ns')) is not int):
        return None
    out = {name: _integer(value.get(name)) for name in _CONTRACT_FIELDS}
    binding = value.get('binding', {})
    if isinstance(binding, dict):
        selection = binding.get('selection', {})
        if isinstance(selection, dict):
            out['selection_generation'] = _integer(selection.get('generation'))
        for key in ('phase_id', 'phase_started_ns', 'phase_reference_source_ns',
                    'view_index', 'target_epoch', 'control_epoch_ns'):
            out[key] = _integer(binding.get(key))
        for key in ('target_track_id', 'session_id', 'lease_id', 'view_plan_id', 'source_id'):
            token = binding.get(key)
            if isinstance(token, str):
                out[key] = token[:128]
    return out


def _keys(value):
    if not isinstance(value, dict):
        return None
    # Existing station caches are bounded. Never traverse an arbitrary input
    # dictionary or export its payload; retain only up to six integer keys.
    keys = []
    for index, key in enumerate(value):
        if index == 64:
            break
        if type(key) is int:
            keys.append(key)
    return dict(count=len(value), source_keys=sorted(keys)[-6:],
                summary_truncated=len(value) > 64)


def _context():
    stack = getattr(_TLS, 'stack', ())
    return stack[-1] if stack else None


def callback_snapshot():
    """Return only this thread's current diagnostic context, never authority."""
    try:
        current = _context()
        if current is None:
            return None
        out = dict(current)
        out['locks'] = [dict(row) for row in current['locks']]
        out['elapsed_ns'] = max(0, time.monotonic_ns()-current['entry_monotonic_ns'])
        out['thread_cpu_elapsed_ns'] = max(0, time.thread_time_ns()-current['entry_thread_cpu_ns'])
        out.pop('_waiting', None)
        return out
    except Exception:
        return None


def source_parsed(source_ns):
    try:
        current = _context()
        if current is not None and type(source_ns) is int:
            current['source_ns'] = source_ns
    except Exception:
        pass


def lock_wait_start(name):
    try:
        current = _context()
        if current is not None:
            current['_waiting'] = (str(name)[:32], time.monotonic_ns(), time.thread_time_ns())
    except Exception:
        pass


def lock_acquired(name):
    try:
        current = _context()
        waiting = None if current is None else current.pop('_waiting', None)
        if waiting is not None and waiting[0] == name and len(current['locks']) < 4:
            current['locks'].append(dict(name=name,
                wait_ns=max(0, time.monotonic_ns()-waiting[1]),
                thread_cpu_ns=max(0, time.thread_time_ns()-waiting[2])))
    except Exception:
        pass


def recent_callbacks(node):
    try:
        with _HISTORY_LOCK:
            return [dict(row) for row in _HISTORY.get(node, ())]
    except Exception:
        return []


def observe_callback(kind):
    """Record entry before any original callback lock; preserve return/raise."""
    def decorate(function):
        @wraps(function)
        def wrapped(node, message):
            context = None
            old_stack = getattr(_TLS, 'stack', ())
            try:
                # Stamp before header access, parsing, or any planning lock.
                entry = time.monotonic_ns()
                cpu = time.thread_time_ns()
                stamp = getattr(getattr(message, 'header', None), 'stamp', None)
                source = _integer(stamp.to_nsec()) if stamp is not None else None
                context = dict(kind=kind, source_ns=source,
                    thread_id=threading.get_ident(), entry_monotonic_ns=entry,
                    entry_thread_cpu_ns=cpu, locks=[])
                _TLS.stack = tuple(old_stack[-3:])+(context,)
            except Exception:
                context = None
            try:
                return function(node, message)
            finally:
                try:
                    if context is not None:
                        report = callback_snapshot()
                        if report is not None:
                            with _HISTORY_LOCK:
                                history = _HISTORY.get(node)
                                if history is None:
                                    history = deque(maxlen=_MAX_HISTORY)
                                    _HISTORY[node] = history
                                history.append(report)
                except Exception:
                    pass
                finally:
                    _TLS.stack = old_stack
        return wrapped
    return decorate


def failure_before_retire(node):
    """Best-effort tiny snapshot; no arrays, source payloads or config export."""
    try:
        rows = getattr(node, '_station_private_pending_sources', {})
        pending = []
        if isinstance(rows, dict):
            for index, (source, row) in enumerate(rows.items()):
                if index == 6:
                    break
                kind = row.get('kind') if isinstance(row, dict) else None
                pending.append(dict(source_ns=_integer(source),
                    kind=kind[:48] if isinstance(kind, str) else None))
        return dict(current_reference=_contract(getattr(node, '_station_compute_reference', None)),
            retired_before=getattr(node, '_station_compute_retired', None) is True,
            pending_before=getattr(node, '_station_compute_pending', None) is True,
            private_sources_count=len(rows) if isinstance(rows, dict) else None,
            private_source_sample=pending,
            cache_summaries={name: _keys(getattr(node, name, None)) for name in (
                '_station_compute_objects', '_station_compute_masks', '_station_compute_status',
                '_station_compute_work')})
    except Exception:
        return None


def _validator_frames(error):
    frames = []
    tb = error.__traceback__
    visited = 0
    while tb is not None and visited < 32:
        frame = tb.tb_frame
        name = frame.f_code.co_name
        if name in _VALIDATOR_FRAMES:
            local = frame.f_locals
            contracts = {}
            for key in ('value', 'expected', 'contract', 'parent', 'reference'):
                summary = _contract(local.get(key))
                if summary is not None:
                    contracts[key] = summary
            clocks = {key: _integer(local.get(key)) for key in ('now_ns', 'now', 'source_ns')
                      if type(local.get(key)) is int}
            frames.append(dict(function=name, path=frame.f_code.co_filename,
                line=tb.tb_lineno, clocks=clocks, contracts=contracts))
        tb = tb.tb_next
        visited += 1
    return frames[-_MAX_FRAMES:]


def _failure_report(node, error, input_kind, source_ns, before):
    current = callback_snapshot()
    return dict(kind='station_private_pending_failure_diagnostic',
        diagnostic_only=True, execution_authorized=False,
        original_exception_type=type(error).__name__,
        input_kind=input_kind if input_kind in ('status', 'object', 'mask') else None,
        input_source_ns=_integer(source_ns), before_retire=before,
        retired_after=getattr(node, '_station_compute_retired', None) is True,
        current_reference_after=_contract(getattr(node, '_station_compute_reference', None)),
        validator_frames=_validator_frames(error), callback=current,
        recent_callbacks=recent_callbacks(node)[-8:])


def log_failure_after_retire(node, error, input_kind, source_ns, before):
    """Called only after the original retirement; all diagnostic failures drop."""
    try:
        report = _failure_report(node, error, input_kind, source_ns, before)
        import rospy
        rospy.logwarn('Station handoff diagnostic: %s',
            json.dumps(report, sort_keys=True, separators=(',', ':'), allow_nan=False))
    except Exception:
        pass


def task_station_snapshot(node):
    """Small description of the task's local fan-in, never an authority input."""
    try:
        record = getattr(node, '_station_task_record', None)
        with _HISTORY_LOCK:
            events = [dict(row) for row in _TASK_HISTORY.get(node, ())]
        return dict(current_reference=_contract(record),
            pending_before=isinstance(record, dict) and record.get('pending') is True,
            cache_summaries={name: _keys(getattr(node, name, None)) for name in (
                '_station_task_objects', '_station_task_masks', '_station_task_statuses',
                '_station_task_availability_pending')}, recent_join_events=events)
    except Exception:
        return None


def task_station_event(node, kind, source_ns, error=None):
    """Keep at most 16 distinct events; repeated timer rejection only increments."""
    try:
        import rospy
        reason = None if error is None else str(error)[:256]
        stamp = int(rospy.Time.now().to_nsec())
        row = dict(kind=kind, source_ns=_integer(source_ns), reason=reason,
            first_observed_ns=stamp, last_observed_ns=stamp, occurrences=1,
            current_reference=_contract(getattr(node, '_station_task_record', None)),
            validator_frames=[] if error is None else _validator_frames(error))
        with _HISTORY_LOCK:
            history = _TASK_HISTORY.get(node)
            if history is None:
                history = deque(maxlen=16)
                _TASK_HISTORY[node] = history
            if history and all(history[-1][key] == row[key]
                    for key in ('kind', 'source_ns', 'reason', 'current_reference')):
                history[-1]['last_observed_ns'] = stamp
                history[-1]['occurrences'] += 1
            else:
                history.append(row)
        if error is not None and row['current_reference'] is None:
            # Initial fan-in can fail before any record exists to retire. Keep
            # its original reason observable instead of only logging the later
            # producer timeout; this diagnostic cannot authorize registration.
            rospy.logwarn_throttle(2., 'Station initial registration diagnostic: %s',
                json.dumps(row, sort_keys=True, separators=(',', ':'), allow_nan=False))
    except Exception:
        pass


def log_task_retirement(node, reason, before):
    """Observe completed retirement; logging cannot defer it or restore work."""
    try:
        import rospy
        report = dict(kind='station_task_retirement_diagnostic', diagnostic_only=True,
            execution_authorized=False, reason=str(reason)[:256], before_retire=before,
            retired_after=getattr(node, '_station_task_retired', None) is True,
            current_reference_after=_contract(getattr(node, '_station_task_record', None)))
        rospy.logwarn('Station task diagnostic: %s',
            json.dumps(report, sort_keys=True, separators=(',', ':'), allow_nan=False))
    except Exception:
        pass
