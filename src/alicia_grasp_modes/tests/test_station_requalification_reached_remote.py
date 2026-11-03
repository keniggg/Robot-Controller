"""Actual reached-phase callback and registration after a new measured lease."""
from copy import deepcopy

import pytest

from test_observation_measurement_remote import context, at
from test_station_compute_wait_remote import station, feedback, row_at
from test_station_requalification_remote import qualified_recovery
from test_unknown_pending_identity import deliver
from test_remote_modes import base, direct_observation


def reached_phase(c, done):
    n=c.node
    # Measured initial cloud fixture, fused with the original implementation.
    initial=direct_observation(n,c.binding['phase_reference_source_ns'])
    ref=base.surface_view_from_observation(initial)
    surface=base.fused_surface_from_view(ref,voxel_size_m=.0025)
    n._near_field_reference_view=ref;n._near_field_fused_surface=surface
    n._near_field_surface_phase_id=n._near_field_phase_id
    n._near_field_surface_generation=n._stream_generation
    n._observe_return_saved=(surface,ref)
    n._observe_return_session['stamp_ns']=n._observe_return_session['view_source_stamp_ns']
    n._observe_return_reached_phase_id=c.phase.phase_id
    n._observe_return_view_uncertainty_m=.001
    c.clock[0]+=.05
    phase=deepcopy(c.phase);phase.phase_id+=1
    phase.header.stamp=at(n._ros_source_clock_ns());phase.header.frame_id='base_link'
    phase.reference_source_stamp=at(done['source_stamps_ns'][-1])
    return phase, surface


@pytest.mark.parametrize('order',[('mask','object','status'),('status','object','mask')])
def test_actual_reached_phase_registers_unpauses_then_full_source_join_finishes_recovery(station,monkeypatch,order):
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    phase,before=reached_phase(c,done)
    observation=direct_observation(n,done['source_stamps_ns'][-1])
    # Only image reconstruction is mocked: the public phase callback, measured
    # cross-view registration, generation handoff and unpause are real code.
    monkeypatch.setattr(n,'_resolve_reached_view_observation',lambda msg: observation)
    n.near_field_state_cb(phase)
    assert n._observe_return_reached_phase_id==phase.phase_id
    assert not n._observe_return_paused
    assert n._latest_registration_evidence.result.ok
    assert n._latest_registration_evidence.stamp_ns==observation.stamp_ns
    fused,ref=n._active_multiview_surface()
    assert fused.view_stamps_ns==before.view_stamps_ns+(observation.stamp_ns,)
    assert n._station_requalification_hold is not None
    assert n._station_compute_pending # Registration alone is no source join.
    c.phase=phase;c.binding=deepcopy(c.binding)
    c.binding.update(lease_id=done['lease_id'],phase_id=phase.phase_id,
        phase_started_ns=phase.header.stamp.to_nsec(),phase_reference_source_ns=observation.stamp_ns,
        station_q_rad=q)
    c.ready=row;c.clock[0]+=.1;ns=n._ros_source_clock_ns()-1
    feedback(c,ns-1,q);feedback(c,ns+1,q)
    new=row_at(c,ns)
    for key in order[:-1]:
        deliver(n,new,(key,));assert n._station_requalification_hold is not None
    deliver(n,new,order[-1:])
    assert n._station_requalification_hold is None
    assert n._station_compute_reference['contract']==new['status']['station_compute_wait']
    assert not n._station_compute_pending and not n._station_compute_retired
    assert n._current_stream_target_identity()==c.identity
    assert phase.deadline.to_nsec()==p['phase_deadline_ns']


def test_same_phase_callback_cannot_unpause_new_lease_or_skip_registration(station,monkeypatch):
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    phase,_=reached_phase(c,done);phase.phase_id-=1
    called=[]
    monkeypatch.setattr(n,'_resolve_reached_view_observation',lambda msg:called.append(msg))
    n.near_field_state_cb(phase)
    assert not called
    assert n._observe_return_paused
    assert n._station_compute_pending and n._station_requalification_hold is not None


def test_same_view_requalification_preserves_its_prior_measured_information_gain(station,monkeypatch):
    from test_observe_return_supplement_remote import gained
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    phase,before=reached_phase(c,done)
    old=gained(c.identity,before.view_stamps_ns[-1]);n._observe_return_step_gain=deepcopy(old)
    n._observe_return_visited_camera_origins=[[.1,.2,.3],[.2,.2,.3]]
    visited=deepcopy(n._observe_return_visited_camera_origins)
    observation=direct_observation(n,done['source_stamps_ns'][-1])
    monkeypatch.setattr(n,'_resolve_reached_view_observation',lambda msg:observation)
    n.near_field_state_cb(phase)
    assert n._latest_registration_evidence.result.ok
    assert not n._observe_return_paused
    assert n._observe_return_step_gain==old
    assert n._observe_return_visited_camera_origins==visited
    assert n._observe_return_session['view_index']==done['view_index']


@pytest.mark.parametrize('bad',['identity','source','invalid'])
def test_requalification_never_preserves_foreign_or_invalid_prior_gain(station,monkeypatch,bad):
    from test_observe_return_supplement_remote import gained
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    phase,before=reached_phase(c,done)
    old=gained(c.identity,before.view_stamps_ns[-1])
    if bad=='identity':old['target_epoch']+=1
    elif bad=='source':old=gained(c.identity,before.view_stamps_ns[-1]-10)
    else:old['ok']=False
    n._observe_return_step_gain=old
    observation=direct_observation(n,done['source_stamps_ns'][-1])
    monkeypatch.setattr(n,'_resolve_reached_view_observation',lambda msg:observation)
    n.near_field_state_cb(phase)
    assert n._latest_registration_evidence.result.ok
    assert not n._observe_return_step_gain['ok']
    assert n._observe_return_step_gain['source_stamp_ns']==observation.stamp_ns


def test_reached_callback_cannot_register_expired_requalification(station,monkeypatch):
    c=station;n=c.node;p,done,row,q=qualified_recovery(c)
    phase,before=reached_phase(c,done)
    observation=direct_observation(n,done['source_stamps_ns'][-1])
    monkeypatch.setattr(n,'_resolve_reached_view_observation',lambda msg:observation)
    c.clock[0]=p['identity_deadline_ns']/1e9+.001
    n.near_field_state_cb(phase)
    assert n._station_compute_retired
    assert n._observe_return_reached_phase_id!=phase.phase_id
