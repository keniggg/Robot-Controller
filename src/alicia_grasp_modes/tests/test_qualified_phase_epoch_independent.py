"""Independent Unix-time phase ownership regressions; no ROS or hardware I/O.

Reuse the existing inert transport fixtures, but call actual task MRO phase
creation/reuse and the actual observed-contact audit builder and task reader.
Time translation preserves the fixture's exact source/owner relationships.
"""
from copy import deepcopy
import math
from types import SimpleNamespace as NS

import pytest
import numpy as np

from test_observe_return_task import context, at_ns, audit_reference, module
from test_qualified_station_phase import qualified_context, finish
from test_observed_contact_current_pose import observed, error
from alicia_grasp_modes.observe_return_task import committed_plan_digest
from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
from alicia_grasp_modes import observe_return_task as owner


RECORDED_START_NS = 1791880763644008874
RECORDED_DEADLINE_NS = 1791880853639396667
OFFSETS_NS = (0, 1, 2, 31, 63, 127, 128, 129, 191, 237, 238, 239, 255, 256, 257, 511)


def translate(value, delta, key=''):
    if isinstance(value, dict):
        return {name: translate(item, delta, name) for name, item in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(translate(item, delta, key) for item in value)
    if key.endswith('_ns') and type(value) in (int, str) and str(value).isdigit() and int(value) > 0:
        return str(int(value)+delta) if type(value) is str else value+delta
    return deepcopy(value)


def translate_context(c, delta):
    n = c.node
    c.now[0] += delta
    c.selection = translate(c.selection, delta)
    c.params[module.SELECTION_PARAM] = translate(c.params[module.SELECTION_PARAM], delta)
    n._mode_selection = translate(n._mode_selection, delta)
    n._mode_bound_selection = translate(n._mode_bound_selection, delta)
    n._observe_return_session = translate(n._observe_return_session, delta)
    n._observe_return_origin_proof = translate(n._observe_return_origin_proof, delta)
    n._following_epoch_ns += delta


def finish_at(c, phase_ns=RECORDED_START_NS, view='first'):
    # Existing finish advances 200 ms to arrival, then three 100 ms sources,
    # then one ns. All subsequent checks use the ordinary exact ROS clock.
    delta = phase_ns - (c.now[0] + 500_000_001)
    translate_context(c, delta)
    for header in (c.plan.header, c.plan.object_geometry.header):
        header.stamp = at_ns(header.stamp.to_nsec()+delta)
    c.node._bound_execution_plan = deepcopy(c.plan)
    c.node._bound_execution_plan_digest = committed_plan_digest(c.plan)
    c.cfg['near_field_replan_timeout_sec'] = 90.
    done = finish(c, view)
    assert c.phases[0].header.stamp.to_nsec() == phase_ns
    return done


@pytest.mark.parametrize('offset', OFFSETS_NS)
@pytest.mark.parametrize('view', ('first', 'supplement', 'return'))
def test_actual_unix_phase_reuse_keeps_original_wire_and_deadline(qualified_context, offset, view):
    c = qualified_context
    done = finish_at(c, RECORDED_START_NS+offset, view)
    phase = deepcopy(c.phases[0])
    deadline = c.node._qualified_station_phase['monotonic_deadline']
    assert c.node._near_field_phase_started_sec == phase.header.stamp.to_sec()
    assert c.node._near_field_phase_deadline_sec == phase.deadline.to_sec()
    assert c.node._near_field_phase_started_ns == phase.header.stamp.to_nsec()
    assert c.node._near_field_phase_deadline_ns == phase.deadline.to_nsec()
    # Repeat the second call, at the recorded ~0.4 s caller delay and later.
    for advance in (404_785_752, 1, 999_999_999):
        c.now[0] += advance
        assert c.node._ensure_qualified_station_phase(c.plan, c.cfg) == phase
        assert c.node._qualified_station_phase['monotonic_deadline'] == deadline
        assert c.node._observation_measurement_request == done
        assert c.phases == [phase]
    assert c.gateways == [('/supervisor/capture_observe_return_origin', c.plan.plan_id, True)]


def test_recorded_clock_value_is_nonidentical_under_previous_two_conversions():
    stamp = at_ns(RECORDED_START_NS)
    assert float(RECORDED_START_NS)*1e-9 != stamp.to_sec()
    assert abs(float(RECORDED_START_NS)*1e-9-stamp.to_sec()) == np.spacing(stamp.to_sec())
    deadline = at_ns(RECORDED_DEADLINE_NS+1)
    assert module.rospy.Time.from_sec(deadline.to_sec()).to_nsec() != RECORDED_DEADLINE_NS+1


@pytest.mark.parametrize('offset', OFFSETS_NS)
def test_production_setter_preserves_supported_absolute_deadline_ns(qualified_context, offset):
    c = qualified_context; n = c.node
    finish_at(c)
    requested = RECORDED_DEADLINE_NS+offset
    assert c.now[0] < requested < c.now[0]+90_000_000_000
    n._set_near_field_active(True, force=True, budget_sec=90., reference_plan=c.plan,
        reference_object=n.latest_obj, absolute_deadline_ns=requested)
    phase = c.phases[-1]
    assert phase.deadline.to_nsec() == requested
    assert n._near_field_phase_deadline_ns == requested
    assert n._near_field_phase_deadline_sec == phase.deadline.to_sec()
    assert n._near_field_phase_started_ns == phase.header.stamp.to_nsec()
    assert n._near_field_phase_started_sec == phase.header.stamp.to_sec()
    if offset == 1:
        assert module.rospy.Time.from_sec(n._near_field_phase_deadline_sec).to_nsec() != requested


@pytest.mark.parametrize('change', ('start_cache_up', 'start_cache_down', 'deadline_cache_up',
    'deadline_cache_down', 'phase_start_ns', 'phase_deadline_ns', 'reference_source_ns',
    'phase_id', 'inactive', 'manual', 'task_stopped', 'shutdown', 'epoch', 'lease',
    'ack', 'selection', 'ros_deadline', 'monotonic_deadline', 'monotonic_after',
    'start_ns_cache_up', 'start_ns_cache_down', 'deadline_ns_cache_up', 'deadline_ns_cache_down'))
def test_unix_phase_genuine_owner_changes_and_exact_expiry_still_reject(
        qualified_context, monkeypatch, change):
    c = qualified_context; n = c.node
    finish_at(c)
    phase = n._published_near_field_phase
    if change.startswith('start_ns_cache_'):
        n._near_field_phase_started_ns += 1 if change.endswith('_up') else -1
    elif change.startswith('deadline_ns_cache_'):
        n._near_field_phase_deadline_ns += 1 if change.endswith('_up') else -1
    elif change.startswith('start_cache_'):
        n._near_field_phase_started_sec = np.nextafter(n._near_field_phase_started_sec,
            math.inf if change.endswith('_up') else -math.inf)
    elif change.startswith('deadline_cache_'):
        n._near_field_phase_deadline_sec = np.nextafter(n._near_field_phase_deadline_sec,
            math.inf if change.endswith('_up') else -math.inf)
    elif change == 'phase_start_ns': phase.header.stamp = at_ns(phase.header.stamp.to_nsec()+1)
    elif change == 'phase_deadline_ns': phase.deadline = at_ns(phase.deadline.to_nsec()+1)
    elif change == 'reference_source_ns': phase.reference_source_stamp = at_ns(phase.reference_source_stamp.to_nsec()+1)
    elif change == 'phase_id': n._near_field_phase_id += 1
    elif change == 'inactive': n._set_near_field_active(False)
    elif change == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif change == 'task_stopped': n.active = False
    elif change == 'shutdown': monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: True)
    elif change == 'epoch': n._following_epoch_ns += 1
    elif change == 'lease': n._observation_measurement_request['lease_id'] += '-changed'
    elif change == 'ack': n._observation_measurement_acks.clear()
    elif change == 'selection': c.params[module.SELECTION_PARAM]['generation'] += 1
    elif change == 'ros_deadline': c.now[0] = phase.deadline.to_nsec()
    else:
        expiry = n._qualified_station_phase['monotonic_deadline']
        monkeypatch.setattr(owner.time, 'monotonic', lambda: expiry+(1. if change.endswith('_after') else 0.))
    with pytest.raises((ValueError, RuntimeError)):
        n._ensure_qualified_station_phase(c.plan, c.cfg)
    assert n._qualified_station_phase is None and n._near_field_active is False
    assert len([p for p in c.phases if p.active]) == 1


def test_last_integer_nanosecond_before_original_expiry_remains_live(qualified_context, monkeypatch):
    c = qualified_context; n = c.node
    finish_at(c)
    phase = deepcopy(c.phases[0])
    c.now[0] = phase.deadline.to_nsec()-1
    expiry = n._qualified_station_phase['monotonic_deadline']
    monkeypatch.setattr(owner.time, 'monotonic', lambda: np.nextafter(expiry, -math.inf))
    assert n._ensure_qualified_station_phase(c.plan, c.cfg) == phase
    assert c.phases == [phase]


def observed_at(c, deadline_ns=RECORDED_DEADLINE_NS):
    delta = deadline_ns-c.reached_phase.deadline.to_nsec()
    translate_context(c, delta)
    c.contact_report = translate(c.contact_report, delta)
    c.measurement = translate(c.measurement, delta)
    for plan in (c.plan, c.contact):
        plan.header.stamp = at_ns(plan.header.stamp.to_nsec()+delta)
    phase = c.reached_phase
    for obj, name in ((phase.header, 'stamp'), (phase, 'deadline'), (phase, 'reference_source_stamp')):
        setattr(obj, name, at_ns(getattr(obj, name).to_nsec()+delta))
    n = c.node
    n._near_field_phase_deadline_sec = phase.deadline.to_sec()
    n._near_field_phase_started_sec = phase.header.stamp.to_sec()
    n._near_field_phase_deadline_ns = phase.deadline.to_nsec()
    n._near_field_phase_started_ns = phase.header.stamp.to_nsec()
    n._observation_measurement_request = c.measurement
    r = c.remote
    r._observe_return_session = deepcopy(n._observe_return_session)
    r._measurement_remote_request = c.measurement
    r._observe_return_initial_stamp += delta
    r._observe_return_reached_contact_context = translate(r._observe_return_reached_contact_context, delta)
    fused = c.contact_report['multiview_surface']['source_stamps_ns']
    r._active_multiview_surface = lambda: (NS(view_stamps_ns=fused), None)
    ObserveReturnRemoteMixin._observe_return_audit(r, c.contact_report,
        NS(snapshot=NS(stamp_ns=c.contact.header.stamp.to_nsec())))
    n._observe_return_audit_cb(audit_reference(c.tmp_path, c.contact_report))
    assert not error(c)
    return c


@pytest.mark.parametrize('offset', OFFSETS_NS)
def test_actual_contact_audit_accepts_exact_unix_deadline_and_adjacent_ns(observed, offset):
    c = observed_at(observed, RECORDED_DEADLINE_NS+offset)
    phase = deepcopy(c.reached_phase)
    assert c.node._observe_return_report(c.contact, observed_contact=True) == c.contact_report
    assert c.reached_phase == phase
    assert not c.replans


@pytest.mark.parametrize('change', ('cache_up', 'cache_down', 'phase_deadline_ns',
    'phase_start_ns', 'reference_source_ns', 'phase_id', 'inactive', 'manual',
    'task_stopped', 'epoch', 'deadline', 'measurement_source',
    'start_ns_cache_up', 'start_ns_cache_down', 'deadline_ns_cache_up', 'deadline_ns_cache_down'))
def test_contact_audit_does_not_accept_real_unix_owner_or_ns_mutation(observed, change):
    c = observed_at(observed); n = c.node; p = c.reached_phase
    if change.startswith('start_ns_cache_'):
        n._near_field_phase_started_ns += 1 if change.endswith('_up') else -1
    elif change.startswith('deadline_ns_cache_'):
        n._near_field_phase_deadline_ns += 1 if change.endswith('_up') else -1
    elif change.startswith('cache_'):
        n._near_field_phase_deadline_sec = np.nextafter(n._near_field_phase_deadline_sec,
            math.inf if change.endswith('_up') else -math.inf)
    elif change == 'phase_deadline_ns': p.deadline = at_ns(p.deadline.to_nsec()+1)
    elif change == 'phase_start_ns': p.header.stamp = at_ns(p.header.stamp.to_nsec()+1)
    elif change == 'reference_source_ns': p.reference_source_stamp = at_ns(p.reference_source_stamp.to_nsec()+1)
    elif change == 'phase_id': n._near_field_phase_id += 1
    elif change == 'inactive': n._near_field_active = False
    elif change == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif change == 'task_stopped': n.active = False
    elif change == 'epoch': n._following_epoch_ns += 1
    elif change == 'deadline': c.now[0] = p.deadline.to_nsec()
    else: c.measurement['source_stamps_ns'][-1] += 1
    with pytest.raises(ValueError):
        n._observe_return_report(c.contact, observed_contact=True)
    assert not c.replans
