"""Phase timestamp caches retain exact published ownership at Unix epochs.

Only clock/transport and pre-existing audit fixture geometry are synthetic.
No ROS master, hardware command, or physical grasp is used by these tests.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
from test_observe_return_task import at_ns, audit_reference, context, module
from test_observed_contact_current_pose import observed
from test_station_compute_wait_task import configured, station


R3_START_NS = 1791880763644008874
R3_DEADLINE_NS = 1791880853639396667


@pytest.mark.parametrize('offset', [0, 1, -1, 63, 127, 255, 511, 999999999])
def test_published_integer_phase_owns_both_cache_representations(station, offset):
    n = station.node
    station.now[0] = R3_START_NS + offset
    deadline_ns = R3_DEADLINE_NS + offset
    n._set_near_field_active(True, force=True, budget_sec=100.,
                             absolute_deadline_ns=deadline_ns)
    phase = n._published_near_field_phase
    assert phase.header.stamp.to_nsec() == R3_START_NS + offset
    assert phase.deadline.to_nsec() == deadline_ns
    assert n._near_field_phase_started_ns == phase.header.stamp.to_nsec()
    assert n._near_field_phase_deadline_ns == phase.deadline.to_nsec()
    assert n._near_field_phase_started_sec == phase.header.stamp.to_sec()
    assert n._near_field_phase_deadline_sec == phase.deadline.to_sec()
    if offset == 0:
        # The R3 start hits the original writer/reader representation mismatch.
        assert float(R3_START_NS)*1e-9 != phase.header.stamp.to_sec()
    if offset == 1:
        # The adjacent legal deadline cannot round-trip through seconds.
        assert module.rospy.Time.from_sec(phase.deadline.to_sec()).to_nsec() != deadline_ns
    before = deepcopy(phase)
    station.now[0] += 500000000
    assert n._set_near_field_active(True) is False
    assert n._published_near_field_phase == before
    assert n._near_field_phase_deadline_ns == deadline_ns


@pytest.mark.parametrize('budget', [0.123456789, 12., 30., 60., 89.123456789])
def test_float_budget_cache_uses_actual_quantized_wire_without_changing_wire(station, budget):
    n = station.node
    station.now[0] = R3_START_NS
    # Preserve the existing deadline calculation; only cache representation
    # changes after the original ROS Time construction.
    original_deadline = module.rospy.Time.from_sec(float(R3_START_NS)*1e-9 + budget)
    n._set_near_field_active(True, force=True, budget_sec=budget)
    phase = n._published_near_field_phase
    assert phase.deadline == original_deadline
    assert n._near_field_phase_deadline_ns == original_deadline.to_nsec()
    assert n._near_field_phase_deadline_sec == original_deadline.to_sec()
    n._set_near_field_active(False)
    assert n._near_field_phase_started_ns == n._near_field_phase_deadline_ns == 0
    assert n._near_field_phase_started_sec == n._near_field_phase_deadline_sec == 0.
    assert not n._published_near_field_phase.active


def prepare_nonroundtrippable_contact(c):
    deadline_ns = 80000000101
    c.reached_phase.deadline = at_ns(deadline_ns)
    c.node._near_field_phase_deadline_ns = deadline_ns
    c.node._near_field_phase_deadline_sec = c.reached_phase.deadline.to_sec()
    assert module.rospy.Time.from_sec(c.node._near_field_phase_deadline_sec).to_nsec() != deadline_ns
    c.remote._observe_return_reached_contact_context['deadline_ns'] = deadline_ns
    ObserveReturnRemoteMixin._observe_return_audit(
        c.remote, c.contact_report, NS(snapshot=NS(stamp_ns=c.contact.header.stamp.to_nsec())))
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, c.contact_report))


def test_real_contact_audit_callback_accepts_exact_nonroundtrippable_deadline(observed):
    c = observed
    prepare_nonroundtrippable_contact(c)
    assert c.node._observe_return_report(c.contact, observed_contact=True) == c.contact_report
    assert c.node._published_near_field_phase.deadline.to_nsec() == 80000000101
    assert not c.replans


@pytest.mark.parametrize('field', ['_near_field_phase_started_ns', '_near_field_phase_deadline_ns'])
@pytest.mark.parametrize('mutation', ['missing', -1, 1])
def test_contact_report_rejects_missing_or_one_ns_changed_owner_cache(observed, field, mutation):
    c = observed
    prepare_nonroundtrippable_contact(c)
    if mutation == 'missing':
        delattr(c.node, field)
    else:
        setattr(c.node, field, getattr(c.node, field)+mutation)
    with pytest.raises(ValueError, match='OBSERVE_RETURN_CONTACT_PHASE_CHANGED'):
        c.node._observe_return_report(c.contact, observed_contact=True)
    assert not c.replans
