"""Fixed task lifetime for initial contact, independent of observation leases.

Only a real inactive -> active task transition can create this budget.  It is
not a trajectory permit: the gateway still requires the exact published plan,
live control ownership and every existing geometry and execution check.
"""
from copy import deepcopy
import hashlib
import json
import math
import re


def authority_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _integer(value, minimum=1):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value < minimum:
        raise ValueError('DIRECT_CONTACT_AUTHORITY_INTEGER_INVALID')
    return value


def authority_bindings(selection, epoch_ns, revocation, model, config):
    selected = {k: selection[k] for k in ('mode', 'strategy', 'generation', 'stamp_ns')}
    selected['generation'] = _integer(selected['generation'])
    selected['stamp_ns'] = _integer(selected['stamp_ns'])
    if (selected['mode'] != 'unknown' or selected['strategy'] not in ('direct', 'observe_return')
            or not isinstance(model, str) or not model.strip() or not isinstance(config, dict)):
        raise ValueError('DIRECT_CONTACT_AUTHORITY_BINDING_INVALID')
    return dict(selection=selected, epoch_ns=_integer(epoch_ns),
                revocation=_integer(revocation, 0),
                model_sha256=hashlib.sha256(model.encode()).hexdigest(),
                config_sha256=authority_digest(config))


class DirectContactTaskAuthority:
    """Caller serializes access with the gateway's existing context lock."""
    def __init__(self):
        self._inactive_stamp_ns = None
        self._active_seen = False
        self._lease = None
        self._track_id = None
        self._last_ros_ns = 0
        self._last_monotonic = None
        self._last_event_stamp_ns = 0
        self._budget_required = False

    @property
    def has_budget(self):
        return self._lease is not None

    @property
    def budget_required(self):
        """Revocation cannot turn an admitted task back into a fresh local episode."""
        return self._budget_required

    def revoke(self, reason='revoked'):
        self._lease = None
        self._track_id = None
        self._inactive_stamp_ns = None
        self._active_seen = True
        self.reason = str(reason)

    def _clock(self, now_ns, monotonic_now):
        now_ns = _integer(now_ns)
        if (type(monotonic_now) not in (int, float) or not math.isfinite(monotonic_now)
                or now_ns < self._last_ros_ns
                or (self._last_monotonic is not None and monotonic_now < self._last_monotonic)):
            self.revoke('clock rollback')
            raise ValueError('DIRECT_CONTACT_AUTHORITY_CLOCK_ROLLBACK')
        self._last_ros_ns, self._last_monotonic = now_ns, monotonic_now
        return now_ns, monotonic_now

    def observe(self, *, active, terminal, stamp_ns, now_ns, monotonic_now,
                bindings=None, budget_sec=None, manual=False):
        now_ns, mono = self._clock(now_ns, monotonic_now)
        stamp_ns = _integer(stamp_ns)
        if (type(active) is not bool or type(terminal) is not bool or type(manual) is not bool
                or stamp_ns > now_ns):
            self.revoke('invalid task event')
            raise ValueError('DIRECT_CONTACT_AUTHORITY_EVENT_INVALID')
        if stamp_ns <= self._last_event_stamp_ns:
            return deepcopy(self._lease)
        self._last_event_stamp_ns = stamp_ns
        if not active:
            self._lease, self._track_id = None, None
            self._budget_required = False
            self._inactive_stamp_ns, self._active_seen = stamp_ns, False
            return None
        if terminal or manual:
            self.revoke('terminal or manual task')
            return None
        if self._active_seen:
            # A missed/invalid first active event cannot be repaired by a
            # heartbeat, plan publication, or later configuration change.
            if self._lease is not None:
                self.validate(self._lease, bindings=bindings,
                              now_ns=now_ns, monotonic_now=mono)
            return deepcopy(self._lease)
        inactive = self._inactive_stamp_ns
        self._active_seen, self._inactive_stamp_ns = True, None
        if inactive is None or stamp_ns <= inactive:
            return None
        try:
            if (not isinstance(bindings, dict)
                    or not bindings['selection']['stamp_ns'] <= stamp_ns
                    or not bindings['epoch_ns'] <= stamp_ns
                    or type(budget_sec) not in (int, float)
                    or not math.isfinite(budget_sec) or not 0. < budget_sec <= 600.):
                raise ValueError('DIRECT_CONTACT_AUTHORITY_START_INVALID')
            deadline = stamp_ns + round(budget_sec * 1e9)
            if now_ns >= deadline:
                raise ValueError('DIRECT_CONTACT_AUTHORITY_EXPIRED')
            payload = dict(bindings=deepcopy(bindings), start_stamp_ns=stamp_ns,
                           ros_deadline_ns=deadline,
                           monotonic_deadline=mono+(deadline-now_ns)*1e-9)
            self._lease = dict(payload, task_key=authority_digest(payload))
            self._budget_required = True
            return deepcopy(self._lease)
        except (KeyError, TypeError, ValueError):
            self.revoke('invalid first active task')
            raise

    def capture(self, *, bindings, now_ns, monotonic_now):
        self.validate(self._lease, bindings=bindings, now_ns=now_ns,
                      monotonic_now=monotonic_now)
        return deepcopy(self._lease)

    def validate(self, lease, *, bindings, now_ns, monotonic_now):
        now_ns, mono = self._clock(now_ns, monotonic_now)
        if (self._lease is None or lease != self._lease or bindings != self._lease['bindings']
                or now_ns >= self._lease['ros_deadline_ns']
                or mono >= self._lease['monotonic_deadline']):
            self.revoke('task expired or binding changed')
            raise ValueError('DIRECT_CONTACT_AUTHORITY_CHANGED_OR_EXPIRED')
        return min((self._lease['ros_deadline_ns']-now_ns)*1e-9,
                   self._lease['monotonic_deadline']-mono)

    def bind_plan(self, lease, *, plan_sha256, source_ns, track_id, bindings,
                  now_ns, monotonic_now):
        self.validate(lease, bindings=bindings, now_ns=now_ns, monotonic_now=monotonic_now)
        source_ns = _integer(source_ns)
        if (not isinstance(plan_sha256, str) or not re.fullmatch('[0-9a-f]{64}', plan_sha256)
                or not isinstance(track_id, str) or not track_id or track_id != track_id.strip()
                or not bindings['selection']['stamp_ns'] < source_ns <= now_ns
                or (self._track_id is not None and track_id != self._track_id)):
            self.revoke('published contact plan identity changed')
            raise ValueError('DIRECT_CONTACT_AUTHORITY_PLAN_INVALID')
        self._track_id = track_id
        return dict(task_key=lease['task_key'], plan_sha256=plan_sha256,
                    source_ns=source_ns, target_track_id=track_id)


class StartedDirectContactTaskBudget:
    """Task-side fixed lifetime, constructed only at start_cb's locked transition.

    The gateway independently observes that transition and uses the same first
    active source stamp. This object grants no trajectory or geometry authority.
    """
    def __init__(self, *, bindings, stamp_ns, monotonic_now, budget_sec):
        self.start_stamp_ns = _integer(stamp_ns)
        if (not isinstance(bindings, dict)
                or bindings['selection']['stamp_ns'] > self.start_stamp_ns
                or bindings['epoch_ns'] > self.start_stamp_ns
                or type(budget_sec) not in (int, float)
                or not math.isfinite(budget_sec) or not 0. < budget_sec <= 600.
                or type(monotonic_now) not in (int, float) or not math.isfinite(monotonic_now)):
            raise ValueError('DIRECT_CONTACT_TASK_START_INVALID')
        self.bindings = deepcopy(bindings)
        self.ros_deadline_ns = self.start_stamp_ns + round(budget_sec*1e9)
        self.monotonic_deadline = monotonic_now + budget_sec
        self._last_ros_ns = self.start_stamp_ns
        self._last_monotonic = monotonic_now
        self.first_active_publication = True
        self.revoked = False

    def revoke(self):
        self.revoked = True

    def validate(self, *, bindings, now_ns, monotonic_now, manual):
        if (self.revoked or manual is not False or bindings != self.bindings
                or type(now_ns) is not int or now_ns < self._last_ros_ns
                or type(monotonic_now) not in (int, float) or not math.isfinite(monotonic_now)
                or monotonic_now < self._last_monotonic
                or now_ns >= self.ros_deadline_ns or monotonic_now >= self.monotonic_deadline):
            self.revoke()
            raise ValueError('DIRECT_CONTACT_TASK_CHANGED_OR_EXPIRED')
        self._last_ros_ns, self._last_monotonic = now_ns, monotonic_now
