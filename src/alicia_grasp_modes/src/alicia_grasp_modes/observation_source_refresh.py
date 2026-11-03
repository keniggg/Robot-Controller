"""Pure measured-source preparation; no ROS, node lifecycle or motion authority.

Same-station measurements retain the original ICP reference and all source
provenance. The isolated contact materializer uses the real mode-aware class,
with a finite whitelist of frozen values and no copied node or transport state.
"""
from collections.abc import Mapping
from dataclasses import dataclass, fields, is_dataclass, replace
from types import MappingProxyType
import math

import numpy as np

from alicia_flexible_grasp.vision.multiview_surface import (
    append_registered_view, surface_view_from_observation)
from alicia_flexible_grasp.vision.fused_scene_geometry import enclose_registered_surface
from .observe_return_visual import measured_bilateral_view_gaps
from .observation_information_uncertainty import observation_information_uncertainty_m


MATERIALIZATION_FIELDS = (
    'tabletop_geometry_enabled', 'tabletop_geometry_config', 'gripper_geometry',
    'opening_fit_clearance_each_side_m', 'gripper_tool_jaw_axis',
    'gripper_tool_finger_length_axis', 'candidate_min_downward_approach_cos',
    'candidate_max_final_approach_lateral_m', 'adaptive_stage_limits',
)


def freeze_value(value):
    """Copy only data; mutable service handles and arbitrary objects fail closed."""
    if value is None or type(value) in (bool, int, float, str, bytes):
        return value
    if isinstance(value, np.generic):
        return freeze_value(value.item())
    if isinstance(value, np.ndarray):
        if value.dtype.hasobject:
            raise ValueError('object arrays are not immutable materialization inputs')
        array = np.ascontiguousarray(value)
        return np.frombuffer(array.tobytes(), dtype=array.dtype).reshape(array.shape)
    if isinstance(value, Mapping):
        return MappingProxyType({k: freeze_value(v) for k, v in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(freeze_value(v) for v in value)
    if is_dataclass(value) and value.__dataclass_params__.frozen:
        return replace(value, **{f.name: freeze_value(getattr(value, f.name))
            for f in fields(value) if f.init})
    raise ValueError('unsupported materialization input: ' + type(value).__name__)


def capture_materialization_configuration(node):
    if any(not hasattr(node, name) for name in MATERIALIZATION_FIELDS):
        raise ValueError('required physical materialization configuration missing')
    return MappingProxyType({name: freeze_value(getattr(node, name))
        for name in MATERIALIZATION_FIELDS})


def audit_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if is_dataclass(value):
        return {f.name: audit_value(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, Mapping):
        return {str(k): audit_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [audit_value(v) for v in value]
    return value


def request_binding(ticket, snapshot):
    identity = snapshot.target_identity
    result = dict(request_id=ticket.request_id, generation=ticket.generation,
        target_epoch=ticket.target_epoch, target_track_id=identity.track_id,
        snapshot_stamp_ns=snapshot.stamp_ns)
    if (any(type(result[key]) is not int or result[key] < 0 for key in
            ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'))
            or result['request_id'] <= 0 or result['snapshot_stamp_ns'] <= 0
            or identity.epoch != ticket.target_epoch):
        raise ValueError('invalid measured refresh request binding')
    return result


@dataclass(frozen=True)
class RegisteredMeasurement:
    surface: object
    reference: object
    evidence: object
    snapshot_surface: object
    snapshot_reference: object
    geometry: object
    enclosure: object


def registered_measurement(saved, observation, geometry, *, config, voxel_size_m,
        max_size_m, max_height_m, evidence_type, snapshot_surface_values):
    """Run original ICP, snapshot-coordinate conversion and outward enclosure."""
    surface, reference = saved
    moving = surface_view_from_observation(observation)
    result, fused = append_registered_view(surface, reference, moving,
        config=config, voxel_size_m=voxel_size_m)
    if result.ok is not True:
        raise ValueError('OBSERVATION_REFRESH_REGISTRATION_' + str(result.code))
    evidence = evidence_type(identity=observation.identity, stamp_ns=observation.stamp_ns,
        result=result, source_clipped=bool(moving.is_clipped))
    snapshot_surface, snapshot_reference = snapshot_surface_values(
        fused, reference, observation, evidence)
    enclosed, audit = enclose_registered_surface(geometry, snapshot_surface,
        expected_identity=observation.identity, max_size_m=max_size_m,
        max_height_m=max_height_m)
    return RegisteredMeasurement(fused, reference, evidence, snapshot_surface,
        snapshot_reference, enclosed, audit)


def materialize_measured_source(node_type, configuration, measurement, snapshot,
        *, selection, receipt, contact_phase, execution_position_error_m, camera_origin):
    """Actual full physical materialization, in a new context without live owners."""
    if (set(configuration) != set(MATERIALIZATION_FIELDS)
            or selection.get('mode') != 'unknown'
            or selection.get('strategy') != 'observe_return'
            or contact_phase not in ('initial_contact', 'observing_contact')
            or isinstance(execution_position_error_m, bool)
            or not math.isfinite(execution_position_error_m)
            or execution_position_error_m < 0.):
        raise ValueError('invalid isolated physical materialization context')
    staged = object.__new__(node_type)
    for name, value in configuration.items():
        setattr(staged, name, freeze_value(value))
    staged._mode_selection = freeze_value(selection)
    staged._measurement_remote_request = freeze_value(receipt)
    staged._observe_return_phase = contact_phase
    staged.near_field_planning_active = True
    # The caller admits only a measured source without a qualified box model.
    staged._regular_box_snapshot = None
    surface = (measurement.snapshot_surface, measurement.snapshot_reference)
    staged._snapshot_multiview_surface = lambda: surface
    staged._runtime_execution_position_error_m = lambda: execution_position_error_m
    candidates, diagnostics = staged._generate_tabletop_candidates(
        measurement.geometry, snapshot, contact_execution_phase=True)
    profiles = tuple(staged._adaptive_stage_profiles(measurement.geometry, snapshot))
    uncertainty = observation_information_uncertainty_m(profiles)
    gaps = measured_bilateral_view_gaps(dict(tabletop_geometry=diagnostics),
        measurement.geometry, staged.tabletop_geometry_config,
        measurement.snapshot_surface, staged.gripper_geometry, camera_origin)
    return candidates, diagnostics, profiles, uncertainty, gaps
