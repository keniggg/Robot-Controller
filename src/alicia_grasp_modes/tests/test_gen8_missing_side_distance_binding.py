"""Recorded GEN8 rejected directions, without planning services or motion."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.observe_return_visual import (
    distance_bound_missing_side_rotation, missing_side_view_gain)
from test_remote_modes import configured, base

FIXTURE = Path(__file__).parent / 'fixtures/gen8_missing_side_distance_binding.json'


def recorded():
    raw = FIXTURE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == '9c098e8bf307c26897e640ea2ede836f6b346321e1891554eb15678b741e84f4'
    return json.loads(raw)


def repair(data, direction, distance, rotation=None, center=None):
    return distance_bound_missing_side_rotation(data['gaps'],
        direction['minimum_camera_rotation'] if rotation is None else rotation,
        direction['metadata'], data['target_center_base_m'] if center is None else center,
        distance, .003, data['support_normal_base'])


@pytest.mark.parametrize('direction_index', [0,1])
@pytest.mark.parametrize('distance_index', range(4))
@pytest.mark.parametrize('roll_deg', [0.,15.,-90.,180.])
def test_recorded_low_slots_fail_at_actual_center_then_pass_unchanged_gate(
        direction_index, distance_index, roll_deg):
    data = recorded(); before = deepcopy(data)
    direction = data['directions'][direction_index]
    distance = direction['distances_m'][distance_index]
    rotation = np.array(direction['minimum_camera_rotation'])
    roll = np.deg2rad(roll_deg)
    rotation = rotation @ np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    center = np.array(data['target_center_base_m'])
    old_center = np.array(data['gaps'][0]['target_center_base_m'])
    assert direction['recorded_rejected_slot_count'] == 56
    # Same old direction is valid around retained centre and invalid around
    # the centre actually used by the original per-distance constructor.
    assert missing_side_view_gain(data['gaps'], old_center-distance*rotation[:,0], .003)['ok']
    assert not missing_side_view_gain(data['gaps'], center-distance*rotation[:,0], .003)['ok']
    fixed, evidence = repair(data, direction, distance, rotation=rotation)
    gain = missing_side_view_gain(data['gaps'], center-distance*fixed[:,0], .003)
    assert gain['ok'] and gain['predicted_new_side_gain_m'] > .003
    assert direction['metadata']['camera_rotation_offset_deg'] < evidence['camera_rotation_offset_deg'] < 45.
    assert fixed[:,0] @ np.array(data['support_normal_base']) < 0.
    np.testing.assert_allclose(fixed.T@fixed, np.eye(3), atol=1e-12, rtol=0.)
    proof = evidence['distance_bound_construction']
    assert proof['retained_gap_center_base_m'] == data['gaps'][0]['target_center_base_m']
    assert proof['retained_gap_recentered'] is False and proof['execution_authority'] is False
    assert proof['exact_original_gate']['ok'] and proof['global_angular_optimum_claimed'] is False
    assert data == before


def test_existing_passing_pose_is_preserved_exactly_even_if_another_gap_satisfies_it():
    data = recorded(); direction = data['directions'][0]
    rotation = np.asarray(direction['minimum_camera_rotation'])
    center = np.asarray(data['gaps'][0]['target_center_base_m'])
    fixed, metadata = repair(data, direction, .285, center=center)
    assert np.array_equal(fixed, rotation)
    assert metadata is direction['metadata']
    # Add an independently valid side; a bad slot binding cannot replace a
    # pose that already passes the original all-gap gate.
    direction['metadata']['target_track_id'] = 'not-the-current-gap'
    fixed, metadata = repair(data, direction, .285, center=center)
    assert np.array_equal(fixed, rotation) and metadata is direction['metadata']


@pytest.mark.parametrize('fault', ['wrong_track','wrong_epoch','wrong_jaw','wrong_sign','duplicate_gap',
                                  'upper_slot','no_upper_proposal','upward','no_angular_room'])
def test_unbound_or_unqualified_bracket_retains_original_slot(fault):
    data = recorded(); direction = data['directions'][0]
    metadata = direction['metadata']
    if fault == 'wrong_track': metadata['target_track_id'] = 'other'
    elif fault == 'wrong_epoch': metadata['target_epoch'] += 1
    elif fault == 'wrong_jaw': metadata['jaw_axis_base'][0] += .01
    elif fault == 'wrong_sign': metadata['missing_side_sign'] *= -1
    elif fault == 'duplicate_gap': data['gaps'] += deepcopy(data['gaps'])
    elif fault == 'upper_slot': metadata['camera_rotation_offset_deg'] = 45.
    elif fault == 'no_upper_proposal': data['target_center_base_m'] = [4.,4.,4.]
    elif fault == 'upward': data['support_normal_base'] = (-np.array(data['support_normal_base'])).tolist()
    elif fault == 'no_angular_room': metadata['maximum_rotation_deg'] = metadata['camera_rotation_offset_deg']+1e-10
    old = np.array(direction['minimum_camera_rotation'])
    fixed, evidence = repair(data, direction, .285)
    assert np.array_equal(fixed, old) and evidence is metadata


@pytest.mark.parametrize('field,value', [('target_center_base_m',[float('nan'),0.,0.]),
                                        ('support_normal_base',[0.,0.,0.])])
def test_invalid_actual_geometry_does_not_construct_proposal(field,value):
    data = recorded(); data[field] = value
    with pytest.raises(ValueError): repair(data,data['directions'][0],.285)


@pytest.mark.parametrize('distance', [0.,-1.,float('inf'),float('nan')])
def test_invalid_actual_distance_is_rejected(distance):
    data = recorded()
    with pytest.raises(ValueError): repair(data,data['directions'][0],distance)


def hook_context(configured, monkeypatch):
    node, _ = configured
    data = recorded(); direction = data['directions'][0]
    rotation = np.array(direction['minimum_camera_rotation'])
    mount = np.array(data['tool_from_camera'])
    transform = np.eye(4); transform[:3,:3] = rotation@mount[:3,:3].T
    reference = base.make_pose_stamped('base_link', transform[:3,3], base.quaternion_from_matrix(transform))
    variant = dict(reference_pose=reference, preference_index=14, camera_roll_offset_deg=0.,
                   camera_tilt_offset_deg=direction['metadata']['camera_rotation_offset_deg'])
    node._mode_selection = dict(mode='unknown', strategy='observe_return', generation=8, stamp_ns=1)
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_diagnostic_observation_only = False
    node._observe_return_directed_views = ((rotation, direction['metadata']),
        (np.eye(3),dict(direction['metadata'],camera_rotation_offset_deg=45.)))
    node._observe_return_view_gaps = data['gaps']
    node._observe_return_view_uncertainty_m = .003
    monkeypatch.setattr(node,'_current_stream_target_identity',lambda: NS(epoch=4,track_id='g1-t4'))
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: mount)
    geometry = NS(center_base=np.array(data['target_center_base_m']),
                  support_normal_base=np.array(data['support_normal_base']))
    return node,data,geometry,variant,mount


def test_hook_binds_each_distance_without_mutating_reference(configured,monkeypatch):
    node,data,geometry,variant,mount = hook_context(configured,monkeypatch)
    old = base.pose_matrix(variant['reference_pose']).copy()
    outputs = [node._observation_distance_reference_variant(variant,geometry,d) for d in (.188,.285,.382)]
    assert len({o['camera_tilt_offset_deg'] for o in outputs}) == 3
    for output,distance in zip(outputs,(.188,.285,.382)):
        camera = base.pose_matrix(output['reference_pose'])@mount
        origin = geometry.center_base-distance*camera[:3,0]
        assert missing_side_view_gain(data['gaps'],origin,.003)['ok']
        assert output['distance_bound_directed_missing_side']['distance_bound_construction']['camera_distance_m'] == distance
    assert np.array_equal(base.pose_matrix(variant['reference_pose']),old)


def test_real_base_generator_still_rejects_repaired_pose_at_original_envelope_gate(configured,monkeypatch):
    node,data,geometry,variant,mount = hook_context(configured,monkeypatch)
    node._stream_condition = None
    node.observation_envelope_gate_enabled = True
    monkeypatch.setattr(node,'_observation_reference_roll_variants',lambda _: ())
    monkeypatch.setattr(node,'_observation_reference_tilt_variants',lambda *_: (variant,))
    monkeypatch.setattr(node,'_observation_distance_preferences',lambda *_: (.188,.285))
    seen = []
    def sequence(*args,**kwargs):
        reference = kwargs['observation_reference_pose']
        camera = base.pose_matrix(reference)@mount
        camera[:3,3] = geometry.center_base-kwargs['observation_camera_distance_m']*camera[:3,0]
        transform = camera@np.linalg.inv(mount)
        pose = base.make_pose_stamped('base_link',transform[:3,3],base.quaternion_from_matrix(transform))
        return NS(pregrasp=pose,observation_view_audit={},adaptive_stage_profile=None)
    def reject(_geometry,pose):
        camera = base.pose_matrix(pose)@mount
        seen.append(camera)
        assert missing_side_view_gain(data['gaps'],camera[:3,3],.003)['ok']
        return NS(ok=False,failure_code='ORIGINAL_ENVELOPE_REJECTION',failure_reason='fixture collision',minimum_support_clearance_m=-1.)
    monkeypatch.setattr(node,'_make_observation_sequence',sequence)
    monkeypatch.setattr(node,'_observation_side_evidence',lambda *_: {'observation_side_evidence_deficit_m':0.})
    monkeypatch.setattr(node,'_observation_envelope_gate',reject)
    passing,rejected = base.RemoteGrasp6DNode._far_field_observation_variants(
        node,None,geometry,None,NS(),variant['reference_pose'],NS(ticket=NS()))
    assert passing == () and len(seen) == len(rejected) == 2
    assert {r['failure_code'] for r in rejected} == {'ORIGINAL_ENVELOPE_REJECTION'}
