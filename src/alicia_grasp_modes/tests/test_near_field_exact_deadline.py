"""Exact recovery cap keeps integer nanoseconds; ordinary phases unchanged."""
from types import SimpleNamespace as NS
import pytest
from test_station_compute_wait_task import station, configured


def test_exact_deadline_survives_float_roundtrip_boundary(station):
    n=station.node;now=station.now[0];deadline=now+3_209_999_999
    phases=[];n.near_field_phase_pub=NS(publish=phases.append)
    n._set_near_field_active(True,force=True,budget_sec=(deadline-now)/1e9,absolute_deadline_ns=deadline)
    assert phases[-1].deadline.to_nsec()==deadline
    assert n._published_near_field_phase.deadline.to_nsec()==deadline
    assert n._near_field_phase_deadline_sec==phases[-1].deadline.to_sec()


@pytest.mark.parametrize('value',[True,False,1.5,-1,100_050_000_000])
def test_exact_deadline_rejects_ambiguous_or_expired_values(station,value):
    with pytest.raises(ValueError):
        station.node._set_near_field_active(True,force=True,budget_sec=30.,absolute_deadline_ns=value)


@pytest.mark.parametrize('budget,float_cap,expected',[(.2,None,100_250_000_000),(30.,100.75,100_750_000_000)])
def test_exact_deadline_cannot_lift_other_caps(station,budget,float_cap,expected):
    n=station.node
    n._set_near_field_active(True,force=True,budget_sec=budget,absolute_deadline_ns=110_000_000_007,
        absolute_deadline_sec=float_cap)
    assert n._published_near_field_phase.deadline.to_nsec()==expected


def test_ordinary_float_phase_and_inactive_keep_original_behavior(station):
    n=station.node
    n._set_near_field_active(True,force=True,budget_sec=30.,absolute_deadline_sec=101.)
    assert n._published_near_field_phase.deadline.to_nsec()==101_000_000_000
    n._set_near_field_active(False)
    assert n._published_near_field_phase.deadline.to_nsec()==0


def test_exact_relative_budget_floors_sub_nanosecond_rounding(station):
    n=station.node
    n._set_near_field_active(True,force=True,budget_sec=1.5e-9,absolute_deadline_ns=110_000_000_000)
    assert n._published_near_field_phase.deadline.to_nsec()==station.now[0]+1
    with pytest.raises(ValueError):
        n._set_near_field_active(True,force=True,budget_sec=.5e-9,absolute_deadline_ns=110_000_000_000)
