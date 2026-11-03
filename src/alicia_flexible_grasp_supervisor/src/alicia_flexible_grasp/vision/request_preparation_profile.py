"""Request-local preparation observations; they confer no execution authority."""
from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import replace
from functools import wraps
import math
from numbers import Integral, Real
from types import MappingProxyType
import time

KEY = 'request_preparation_profile'


def ticket_identity(ticket):
    values = {key: getattr(ticket, key) for key in
              ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_sec')}
    if (any(isinstance(values[key], bool) or not isinstance(values[key], Integral)
            for key in ('request_id', 'generation', 'target_epoch'))
            or values['request_id'] <= 0 or values['generation'] < 0 or values['target_epoch'] < 0
            or isinstance(values['snapshot_stamp_sec'], bool)
            or not isinstance(values['snapshot_stamp_sec'], Real)
            or not math.isfinite(values['snapshot_stamp_sec']) or values['snapshot_stamp_sec'] <= 0):
        raise ValueError('unobservable request profile identity')
    return freeze(dict(request_id=int(values['request_id']), generation=int(values['generation']),
        target_epoch=int(values['target_epoch']), snapshot_stamp_sec=float(values['snapshot_stamp_sec'])))


class NoOpPreparationProfile:
    @contextmanager
    def stage(self, name):
        yield

    def call(self, name, function, *args, **kwargs):
        return function(*args, **kwargs)

    def inherit(self, profile):
        return False

    def snapshot(self):
        return None


def freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({key: freeze(item) for key, item in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(freeze(item) for item in value)
    return value


def merge(first, second):
    if not first:
        return second
    if not second:
        return first
    if first['ticket'] != second['ticket']:
        raise ValueError('request profile ticket mismatch')
    scopes = {row['scope']: row for row in first['scopes']}
    scopes.update({row['scope']: row for row in second['scopes']})
    return freeze(dict(schema_version=1, ticket=first['ticket'], scopes=tuple(scopes.values())))


class RequestPreparationProfile:
    def __init__(self, ticket, scope, wall_clock=None, cpu_clock=None):
        self.wall = wall_clock or time.perf_counter
        self.cpu = cpu_clock or time.thread_time
        self.ticket = ticket_identity(ticket)
        self.scope = scope
        self.components = []
        self.stack = []
        self.inherited = None
        self.started_wall, self.started_cpu = self.wall(), self.cpu()

    @contextmanager
    def stage(self, name):
        wall, cpu = self.wall(), self.cpu()
        item = dict(name=name, depth=len(self.stack), completed=False,
                    child_wall_sec=0., child_thread_cpu_sec=0.)
        self.stack.append(item)
        try:
            yield
            item['completed'] = True
        except BaseException as error:
            item['exception_type'] = type(error).__name__
            code = getattr(error, 'code', None)
            if isinstance(code, str):
                item['exception_code'] = code
            raise
        finally:
            elapsed_wall, elapsed_cpu = max(0., self.wall()-wall), max(0., self.cpu()-cpu)
            self.stack.pop()
            item.update(wall_sec=elapsed_wall, thread_cpu_sec=elapsed_cpu,
                exclusive_wall_sec=max(0., elapsed_wall-item.pop('child_wall_sec')),
                exclusive_thread_cpu_sec=max(0., elapsed_cpu-item.pop('child_thread_cpu_sec')))
            self.components.append(item)
            if self.stack:
                self.stack[-1]['child_wall_sec'] += elapsed_wall
                self.stack[-1]['child_thread_cpu_sec'] += elapsed_cpu

    def call(self, name, function, *args, **kwargs):
        with self.stage(name):
            return function(*args, **kwargs)

    def inherit(self, profile):
        try:
            if profile:
                if profile['ticket'] != self.ticket:
                    return False
                self.inherited = merge(self.inherited, profile)
            return True
        except Exception:
            return False

    def snapshot(self):
        own = freeze(dict(schema_version=1, ticket=self.ticket, scopes=(dict(
            scope=self.scope, elapsed_wall_sec=max(0., self.wall()-self.started_wall),
            elapsed_thread_cpu_sec=max(0., self.cpu()-self.started_cpu),
            components=tuple(dict(item) for item in self.components)),)))
        return merge(self.inherited, own)


def profiled_preparation(scope):
    """Attach only immutable metadata; rethrow the original exception object."""
    def decorate(function):
        @wraps(function)
        def observed(self, ticket, *args, **kwargs):
            try:
                profile = RequestPreparationProfile(ticket, scope)
            except Exception:
                profile = NoOpPreparationProfile()
            try:
                return function(self, ticket, *args, _request_profile=profile, **kwargs)
            except BaseException as error:
                # Reporting must not replace a typed cancellation or its cause.
                try:
                    snapshot = profile.snapshot()
                    if snapshot is not None:
                        existing = getattr(error, KEY, None)
                        setattr(error, KEY, merge(existing, snapshot))
                except Exception:
                    pass
                raise
        return observed
    return decorate


def profiled_prepared(prepared, profile):
    """Attach request observations without changing legacy diagnostics."""
    try:
        snapshot = profile.snapshot()
        if snapshot is None:
            return prepared
        snapshot = merge(getattr(prepared, KEY, None), snapshot)
        return replace(prepared, **{KEY: snapshot})
    except Exception:
        # A diagnostic attachment cannot turn an original success into failure.
        return prepared


def error_preparation_profile(error, ticket):
    """Reject another request's diagnostic even if an exception was reused."""
    try:
        profile = getattr(error, KEY, None)
        required = ('wall_sec', 'thread_cpu_sec', 'exclusive_wall_sec', 'exclusive_thread_cpu_sec')
        if (isinstance(profile, Mapping) and profile['ticket'] == ticket_identity(ticket)
                and type(profile['schema_version']) is int and profile['schema_version'] == 1
                and isinstance(profile['scopes'], tuple)
                and all(isinstance(scope, Mapping) and isinstance(scope['scope'], str)
                    and isinstance(scope['components'], tuple)
                    and all(isinstance(component, Mapping) and isinstance(component['name'], str)
                        and type(component['completed']) is bool
                        and all(not isinstance(component[key], bool)
                            and isinstance(component[key], Real) and math.isfinite(component[key])
                            and component[key] >= 0. for key in required)
                        for component in scope['components']) for scope in profile['scopes'])):
            return profile
    except Exception:
        pass
    return None


def inherit_prepared_profile(profile, prepared):
    try:
        snapshot = getattr(prepared, KEY, None)
        return profile.inherit(snapshot) if snapshot is not None else False
    except Exception:
        return False
