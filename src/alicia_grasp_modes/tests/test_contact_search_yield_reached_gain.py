"""Actual reached callback creates private gain before any typed search yield.

Two observation inputs use an existing measured cloud and a strict subset.
Their timestamps/session are synthetic transport inputs, not GEN16 replay.
Only acquisition is replaced; registration, fusion and gain production run.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.active_observation_policy import information_gain_valid, evidence_digest
from test_remote_modes import configured, module, base
from test_contact_search_yield_remote import ready, search
from alicia_grasp_modes.contact_search_yield import CODE, contact_search_yield_proof_valid


def reached_gain_context(configured, monkeypatch):
    node, prepared, clock, candidates=ready(configured,monkeypatch)
    family_sequences={key:deepcopy(value["sequence"]) for key,value in node._stable_variant_runtime.items()}
    surface,reference=node._near_field_fused_surface,node._near_field_reference_view
    stamp=prepared.snapshot.stamp_ns
    prior=replace(reference,stamp_ns=stamp-2_000_000_000,points_base=reference.points_base[::2].copy())
    before=base.fused_surface_from_view(prior)
    node._near_field_reference_view=prior
    node._near_field_fused_surface=before
    node._observe_return_saved=(before,prior)
    node._observe_return_phase='observing'
    node._observe_return_paused=True
    node._observe_return_step_gain=None
    node._observe_return_view_uncertainty_m=.001
    identity=prepared.snapshot.target_identity
    node._observe_return_session=dict(version=2,phase='observing',session_id='a'*64,
        selection=deepcopy(node._mode_selection),stamp_ns=stamp-1_000_000_000,
        deadline_ns=stamp+180_000_000_000,origin_plan_id='b'*24,
        origin_snapshot_stamp_ns=stamp-1_500_000_000,target_epoch=identity.epoch,
        target_track_id=identity.track_id,max_observations=3,view_index=1,
        view_plan_id='b'*24,view_source_stamp_ns=stamp-1_500_000_000,
        view_request_id='initial',previous_gain_sha256='0'*64)
    observation=base.TargetObservation(identity,stamp,'base_link',reference.points_base,
        reference.support_normal_base,reference.support_offset_m,(12,12,10,10),(480,640),12,'instance_mask')
    monkeypatch.setattr(node,'_resolve_reached_view_observation',lambda _:observation)
    def at(ns):return module.rospy.Time(ns//1_000_000_000,ns%1_000_000_000)
    phase=NS(active=True,phase_id=1,
        header=NS(stamp=at(stamp+100_000_000),frame_id='base_link'),
        deadline=at(stamp+60_100_000_000))
    clock[0]=stamp+1_000_000_000
    node.near_field_state_cb(phase)
    assert node._observe_return_phase=='observing_contact'
    assert node._latest_registration_evidence.result.ok
    assert node._latest_registration_evidence.stamp_ns==stamp
    assert node._observe_return_reached_phase_id==phase.phase_id
    assert not node._observe_return_paused
    assert information_gain_valid(node._observe_return_step_gain),node._observe_return_step_gain
    assert node._observe_return_step_gain['previous_source_stamps_ns']==[prior.stamp_ns]
    assert node._observe_return_step_gain['source_stamps_ns']==[prior.stamp_ns,stamp]
    assert node._observe_return_session['previous_gain_sha256']=='0'*64
    prepared.ticket.generation=node._stream_generation
    prepared.ticket.target_epoch=node.target_instance_epoch
    prepared.near_field=True
    prepared.near_field_phase_id=phase.phase_id
    prepared.near_field_phase_deadline_sec=phase.deadline.to_sec()
    node._active_gate_audit_report.update(generation=node._stream_generation,
        target_epoch=node.target_instance_epoch,grasp_mode_selection=deepcopy(node._mode_selection))
    # The actual phase transition correctly clears prior candidate runtimes.
    # Materialize fresh inert strict-transport families for this reached source.
    node._stable_variant_runtime={key:dict(prepared=prepared,sequence=deepcopy(sequence))
        for key,sequence in family_sequences.items()}
    prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles']=[
        dict(depth_uncertainty_m=.001,execution_position_error_m=0.)]
    monkeypatch.setattr(node,'_require_stream_ticket_current_locked',lambda _:None)
    return node,prepared,clock,before,candidates


def test_real_reached_callback_registers_private_gain_without_prior_serialized_gain(configured,monkeypatch):
    node,prepared,clock,before,candidates=reached_gain_context(configured,monkeypatch)
    gain=node._observe_return_step_gain
    assert gain['new_occupied_voxel_count']>=3
    assert node._active_multiview_surface()[0].view_stamps_ns==(
        before.view_stamps_ns[0],prepared.snapshot.stamp_ns)
    assert node._contact_yield_measured_opportunity(prepared)
    assert node._observe_return_pending_supplement is None


def test_actual_gain_callback_then_real_search_and_typed_supplement_prepare(configured,monkeypatch):
    node,p,clock,before,candidates=reached_gain_context(configured,monkeypatch)
    actual_gain=deepcopy(node._observe_return_step_gain)
    original_deadline=node._observe_return_session['deadline_ns']
    selection,calls,retries=search(node,p,clock,candidates,monkeypatch)
    assert calls==[1,2] and retries==[], (selection,calls,retries,node._contact_yield_search, node._direct_near_field_deadline_sec(p), clock[0], node._near_field_phase_id, p.near_field_phase_id)
    assert selection.termination_reason==CODE and selection.terminated_early
    assert node._observe_return_prepare_supplement(p,CODE)
    pending=node._observe_return_pending_supplement
    assert contact_search_yield_proof_valid(pending['failed_search'])
    assert pending['failed_search']['yield_evidence']['transition_ns']==clock[0]
    assert pending['visual_opportunity']['yielded_contact_search']==pending['failed_search']
    assert pending['previous_gain']==actual_gain
    assert pending['previous_gain_sha256']==evidence_digest(actual_gain)
    assert node._observe_return_phase=='observation_prefix'
    assert pending['view_index']==2
    assert pending['deadline_ns']==original_deadline
    assert node._observe_return_session['previous_gain_sha256']=='0'*64
    assert not pending['contact_or_motion_authorized']
