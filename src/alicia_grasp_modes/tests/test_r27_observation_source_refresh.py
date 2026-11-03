"""Original measured ICP/enclosure/materialization, without ROS or device I/O."""
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import base, module
from test_tabletop_geometry_candidates import GRIPPER, TabletopGeometryConfig, rotation_about_z
from alicia_flexible_grasp.grasp.adaptive_stage_profiles import AdaptiveStageLimits
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.multiview_surface import (
    RegistrationConfig, SurfaceView, fused_surface_from_view)
from alicia_grasp_modes.observation_source_refresh import (
    MATERIALIZATION_FIELDS, capture_materialization_configuration, freeze_value,
    registered_measurement, materialize_measured_source, audit_value)


def measured_fixture(yaw=0., size=(.045, .030, .020)):
    identity = base.TargetTrackIdentity(4, 'g1-t4')
    rotation = rotation_about_z(yaw)
    center = np.array([0., 0., size[2]/2])
    points = np.array([(x, y, size[2]) for x in np.linspace(-size[0]/2, size[0]/2, 21)
        for y in np.linspace(-size[1]/2, size[1]/2, 17)]) @ rotation.T
    reference = SurfaceView(identity, 10_000_000_000, points, np.array([0.,0.,1.]), 0., 12)
    saved = (fused_surface_from_view(reference), reference)
    observation = base.TargetObservation(identity, 11_000_000_000, 'base_link', points,
        np.array([0.,0.,1.]), 0., (12, 12, 40, 30), (60, 80), 12, 'instance_mask')
    geometry = GeometryEstimate(True, '', '', center, rotation, np.array(size),
        np.array([0.,0.,1.]), 0., .9, points, 'instance_mask')
    return saved, observation, geometry


def registered(saved, observation, geometry):
    return registered_measurement(saved, observation, geometry,
        config=RegistrationConfig(), voxel_size_m=.0025, max_size_m=.6,
        max_height_m=.5, evidence_type=base.RegistrationEvidence,
        snapshot_surface_values=base.RemoteGrasp6DNode._snapshot_surface_values)


def materializer_configuration(node=None):
    node = NS() if node is None else node
    node.tabletop_geometry_enabled = True
    node.tabletop_geometry_config = TabletopGeometryConfig()
    node.gripper_geometry = GRIPPER
    node.opening_fit_clearance_each_side_m = .002
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.adaptive_stage_limits = AdaptiveStageLimits()
    node.candidate_min_downward_approach_cos = .65
    node.candidate_max_final_approach_lateral_m = .010
    return capture_materialization_configuration(node)


@pytest.mark.parametrize('yaw,size', [(0.,(.045,.030,.020)),
    (.73,(.060,.035,.027)), (-1.2,(.032,.025,.016))])
def test_real_registration_preserves_original_reference_and_measured_provenance(yaw, size):
    saved, observation, geometry = measured_fixture(yaw, size)
    outcome = registered(saved, observation, geometry)
    assert outcome.reference is saved[1]
    assert outcome.surface.view_stamps_ns == (saved[1].stamp_ns, observation.stamp_ns)
    assert outcome.evidence.result.ok is True
    assert all(set(sources) <= {0,1} for sources in outcome.surface.view_sources)
    local = (outcome.snapshot_surface.points_base-outcome.geometry.center_base) @ geometry.axes_base
    assert np.all(np.abs(local) <= outcome.geometry.size_xyz_m/2 + 1e-12)
    assert outcome.enclosure['synthetic_points_added'] == 0
    assert not outcome.enclosure['contact_or_motion_authorized']
    assert saved[0].view_stamps_ns == (saved[1].stamp_ns,)


@pytest.mark.parametrize('fault', ['same_stamp', 'other_identity', 'support', 'far_points'])
def test_registration_rejection_cannot_produce_refreshed_surface(fault):
    saved, observation, geometry = measured_fixture()
    if fault == 'same_stamp': observation = replace(observation, stamp_ns=saved[1].stamp_ns)
    elif fault == 'other_identity': observation = replace(observation, identity=base.TargetTrackIdentity(5,'other'))
    elif fault == 'support': observation = replace(observation, support_offset_m=.02)
    else: observation = replace(observation, points_base=observation.points_base + [1.,0.,0.])
    with pytest.raises(ValueError, match='REGISTRATION'):
        registered(saved, observation, geometry)
    assert saved[0].view_stamps_ns == (10_000_000_000,)


def test_isolated_real_mode_aware_materialization_has_no_io_or_live_node_copy(monkeypatch):
    saved, observation, geometry = measured_fixture()
    measured = registered(saved, observation, geometry)
    configuration = materializer_configuration()
    calls = []
    original = module.ModeAwareRemoteGrasp6DNode._generate_tabletop_candidates
    def record(staged, *args, **kwargs):
        calls.append(staged)
        assert set(staged.__dict__) == set(configuration) | {
            '_mode_selection','_measurement_remote_request','_observe_return_phase',
            'near_field_planning_active','_regular_box_snapshot',
            '_snapshot_multiview_surface','_runtime_execution_position_error_m'}
        assert staged.near_field_planning_active and staged._observe_return_phase == 'initial_contact'
        assert staged._tabletop_placement_gripper(True).support_clearance_m == pytest.approx(
            GRIPPER.support_clearance_m+.0005)
        return original(staged, *args, **kwargs)
    monkeypatch.setattr(module.ModeAwareRemoteGrasp6DNode, '_generate_tabletop_candidates', record)
    def forbidden(*args, **kwargs): pytest.fail('pure materialization attempted ROS/device I/O')
    for name in ('get_param', 'get_param_cached', 'Publisher', 'Subscriber', 'ServiceProxy'):
        monkeypatch.setattr(base.rospy, name, forbidden)
    snapshot = NS(quality=NS(depth_mad_m=.001, depth_repeatability_m=.001))
    candidates, audit, profiles, uncertainty, gaps = materialize_measured_source(
        module.ModeAwareRemoteGrasp6DNode, configuration, measured, snapshot,
        selection=dict(mode='unknown',strategy='observe_return'), receipt={},
        contact_phase='initial_contact', execution_position_error_m=.001,
        camera_origin=np.array([0.,-.15,.16]))
    assert len(calls) == 1 and candidates == ()
    assert audit['materialized_count'] == 0 and audit['plan_phase'] == 'CONTACT_EXECUTION_PLAN'
    assert audit['contact_execution_gate_deferred'] is False
    assert profiles and uncertainty > 0. and gaps
    assert all(profile.execution_position_error_m == .001 for profile in profiles)
    assert gaps[0]['source_view_stamps_ns'] == list(measured.surface.view_stamps_ns)
    import json
    json.dumps(audit_value(measured.evidence.result), allow_nan=False)
    json.dumps(audit_value(audit), allow_nan=False)


def test_materialization_inputs_are_frozen_and_unlisted_state_is_absent():
    node = NS(secret_publisher=object(), _mode_work_lock=object())
    configuration = materializer_configuration(node)
    assert set(configuration) == set(MATERIALIZATION_FIELDS)
    with pytest.raises(TypeError): configuration['secret_publisher'] = object()
    with pytest.raises(ValueError): configuration['gripper_geometry'].finger_size_xyz_m[0] = 10.
    with pytest.raises(ValueError): freeze_value(node.secret_publisher)
    assert node.gripper_geometry is GRIPPER


@pytest.mark.parametrize('missing', MATERIALIZATION_FIELDS)
def test_every_materialization_configuration_field_is_required(missing):
    node = NS()
    configuration = dict(materializer_configuration(node))
    delattr(node, missing)
    with pytest.raises(ValueError, match='configuration missing'):
        capture_materialization_configuration(node)
    configuration.pop(missing)
    with pytest.raises(ValueError, match='materialization context'):
        materialize_measured_source(module.ModeAwareRemoteGrasp6DNode,
            configuration, None, None, selection=dict(mode='unknown',strategy='observe_return'),
            receipt={},contact_phase='initial_contact',execution_position_error_m=0.,camera_origin=None)
