"""Bounded RGB-interest view proposals and visibility/utility previews.

There is no measured object model here. A proposal or high score is neither an
IK solution nor a collision-free trajectory, and cannot authorize movement.
Only an independent motion owner with the actual scene may execute a view.
"""
from dataclasses import dataclass
import math

import numpy as np

from .observation_view_utility import information_motion_rank
from .rgb_observation_volume import (
    RGBObservationVolume, _real_array, _transform, _rows, _positive_real,
    project_rgb_observation_volume, project_rgb_observation_vertices)


POLICY = 'rgb_interest_visibility_and_bounded_motion_preview_v1'


def _fail(code):
    raise ValueError('RGB_OBSERVATION_VIEW_' + code)


def _joint_vector(value):
    try:
        size = len(value)
    except TypeError:
        _fail('JOINT_VECTOR_INVALID')
    if not 1 <= size <= 32:
        _fail('JOINT_VECTOR_INVALID')
    return _real_array(value, (size,), 'JOINT_VECTOR_INVALID')


def _inside_roi(pixels, shape, margin):
    if margin == 0.:
        return True  # Full pixel-area frustum already checked by projection.
    values = np.asarray(pixels)
    return bool(np.all(values >= margin-1e-10)
        and np.all(values[:, 0] <= shape[1]-1.-margin+1e-10)
        and np.all(values[:, 1] <= shape[0]-1.-margin+1e-10))


def _acquisition_distance(pixels, centres):
    values = np.asarray(pixels)
    return float(np.max(np.linalg.norm(values[:, None, :]-centres[None, :, :], axis=2)))


def _rotation_angle(first, second):
    relative = first.T @ second
    return float(math.acos(float(np.clip((np.trace(relative)-1.)*.5, -1., 1.))))


@dataclass(frozen=True)
class ObservationPoseCandidate:
    """A camera pose with optional FK-associated joint/tool endpoint hints.

    Hints are caller-provided; this module does not verify that FK links them.
    Joint order/unit binding, limits, continuity and trajectory validation are
    motion-owner responsibilities. Joint angles are unwrapped radians; no
    wrapping may hide a large physical move.
    """
    candidate_id: str
    T_base_camera: tuple
    joint_positions: tuple = None
    T_base_tool: tuple = None
    proposal_kind: str = 'caller_camera_pose_or_fk_hint'

    def __post_init__(self):
        if type(self.candidate_id) is not str or not self.candidate_id.strip():
            _fail('CANDIDATE_ID_INVALID')
        if type(self.proposal_kind) is not str or not self.proposal_kind.strip():
            _fail('PROPOSAL_KIND_INVALID')
        object.__setattr__(self, 'T_base_camera', _rows(_transform(self.T_base_camera)))
        if self.joint_positions is not None:
            object.__setattr__(self, 'joint_positions', tuple(float(v) for v in
                _joint_vector(self.joint_positions)))
        if self.T_base_tool is not None:
            object.__setattr__(self, 'T_base_tool', _rows(_transform(self.T_base_tool)))


def _preview_contract(volume):
    return dict(policy=POLICY, source_stamp_ns=volume.source_stamp_ns,
        prediction_only=True, measured_information_gain=False,
        object_surface_reconstructed=False, complete_collision_geometry=False,
        joint_and_tool_fk_association_verified=False,
        reachability_verified=False, collision_checked=False,
        execution_authority=False, motion_authorized=False,
        independent_motion_owner_required=True)


def rank_rgb_observation_candidates(volume, candidates, *, current_T_base_camera,
        camera_matrix, image_shape, working_depth_range_m,
        current_joint_positions=None, current_T_base_tool=None, roi_margin_px=0.,
        acquisition_center_xy=(.5, .5), acquisition_radius_fraction=None,
        translation_cost_scale_m=.025, rotation_cost_scale_rad=math.radians(10.),
        joint_cost_scale_rad=math.radians(15.), novelty_scale_rad=math.radians(5.),
        resolution_gain_scale=.15, observed_T_base_cameras=(), frozen_reference_vertices=()):
    """Rank finite camera candidates using full-volume FOV and bounded proxies.

    View novelty is the minimum ray-angle change over all interest vertices;
    pure rotation at the same optical centre adds no viewpoint novelty.
    Resolution gain is a conservative angular-sampling proxy: minimum ratio of
    old/new Euclidean ranges, minus one. Full projected volume area is *not*
    object resolution; slab uncertainty can inflate it without adding detail.
    Neither proxy predicts measured depth recovery or occlusion resolution.

    Optional history contains at most four actually observed camera poses.
    Novelty is incremental to every historical view, and range improvement is
    incremental to the best historical range per vertex. A one-pixel angular
    or projected-span gain dead zone suppresses tiny revisit/roundoff rewards;
    this is a ranking preference, never proof that useful data was acquired.
    Omitting history preserves the original current-view-only comparison.

    Optional frozen reference vertices contain the original envelope followed
    by up to four original members, bound by the caller to its execution owner.
    Every exact wire vertex remains a visibility constraint at the current and
    proposed poses. The original envelope supplies the utility rays; rebuilding
    a smaller current-view frustum cannot move the reference or create gain.
    Current measured volume visibility remains an independent constraint.

    Cost scales are soft preferences, never allowed distance/angle bands.
    Candidate optical depths are checked against the passed sensor range only.
    Positive ROI margins preserve the existing pixel-centre limits [margin,
    dimension-1-margin]; zero retains the full pixel-area pinhole frustum.
    Optional acquisition radius checks ALL eight vertices against both existing
    initial-entry centre conventions (RGB uses dimension-1; 3D uses dimension).
    This preserves the caller's current perception acquisition window, not a
    fixed RGB-D observation distance. None explicitly leaves it unchecked.
    Absent joint/tool hints are reported as absent, not zero physical cost.
    Supplying current joint/tool state requires corresponding candidate hints,
    preventing an uncosted candidate outranking an otherwise identical FK pose.
    """
    if type(volume) is not RGBObservationVolume:
        _fail('VOLUME_TYPE_INVALID')
    if not isinstance(candidates, (tuple, list)) or len(candidates) > 128:
        _fail('CANDIDATE_SET_INVALID')
    current = _transform(current_T_base_camera)
    # Also establish that the supplied current geometry can see this interest.
    current_pixels = project_rgb_observation_volume(volume, T_base_camera=current,
        camera_matrix=camera_matrix, image_shape=image_shape,
        working_depth_range_m=working_depth_range_m)
    margin = float(_real_array([roi_margin_px], (1,), 'ROI_MARGIN_INVALID')[0])
    if not 0. <= margin < .5*(min(image_shape)-1.):
        _fail('ROI_MARGIN_INVALID')
    if not _inside_roi(current_pixels, image_shape, margin):
        _fail('CURRENT_VOLUME_OUTSIDE_ROI')
    if (not isinstance(frozen_reference_vertices, (tuple, list))
            or len(frozen_reference_vertices) > 5
            or len(frozen_reference_vertices) == 1):
        _fail('FROZEN_REFERENCE_SET_INVALID')
    frozen = tuple(_real_array(value, (8, 3), 'FROZEN_REFERENCE_VERTEX_INVALID').copy()
                   for value in frozen_reference_vertices)
    projection_args = dict(camera_matrix=camera_matrix, image_shape=image_shape,
        working_depth_range_m=working_depth_range_m)
    for vertices in frozen:
        projected = project_rgb_observation_vertices(vertices, T_base_camera=current, **projection_args)
        if not _inside_roi(projected, image_shape, margin):
            _fail('CURRENT_FROZEN_REFERENCE_OUTSIDE_ROI')
    attention = _real_array(acquisition_center_xy, (2,), 'ACQUISITION_CENTRE_INVALID')
    if np.any(attention < 0.) or np.any(attention > 1.):
        _fail('ACQUISITION_CENTRE_INVALID')
    dimensions = np.array([image_shape[1], image_shape[0]], dtype=float)
    centres = np.array([attention*dimensions, attention*(dimensions-1.)])
    radius_fraction = radius = current_acquisition_distance = None
    if acquisition_radius_fraction is not None:
        radius_fraction = _positive_real(acquisition_radius_fraction, 'ACQUISITION_RADIUS_INVALID')
        if radius_fraction > 1.:
            _fail('ACQUISITION_RADIUS_INVALID')
        radius = min(image_shape)*radius_fraction
        current_acquisition_distance = _acquisition_distance(current_pixels, centres)
        if current_acquisition_distance > radius+1e-10:
            _fail('CURRENT_VOLUME_OUTSIDE_ACQUISITION_WINDOW')
    scales = [_positive_real(value, 'VIEW_COST_SCALE_INVALID') for value in
        (translation_cost_scale_m, rotation_cost_scale_rad, joint_cost_scale_rad,
         novelty_scale_rad, resolution_gain_scale)]
    translation_scale, rotation_scale, joint_scale, novelty_scale, resolution_scale = scales
    current_joints = None if current_joint_positions is None else _joint_vector(current_joint_positions)
    current_tool = None if current_T_base_tool is None else _transform(current_T_base_tool)
    vertices = frozen[0] if frozen else np.asarray(volume.vertices_base_m)
    if not isinstance(observed_T_base_cameras, (tuple, list)) or len(observed_T_base_cameras) > 4:
        _fail('OBSERVED_HISTORY_INVALID')
    history = [_transform(value) for value in observed_T_base_cameras]
    history_aware = bool(history)
    # Including current also rejects a do-nothing candidate when the caller
    # has not yet appended its latest real observation to the history.
    camera_centres = np.asarray([pose[:3, 3] for pose in history]+[current[:3, 3]])
    previous_rays = vertices[None, :, :]-camera_centres[:, None, :]
    previous_ranges = np.linalg.norm(previous_rays, axis=2)
    if np.any(previous_ranges <= 0.):
        _fail('OBSERVED_HISTORY_AT_INTEREST_VERTEX')
    previous_unit = previous_rays/previous_ranges[:, :, None]
    best_ranges = np.min(previous_ranges, axis=0)
    intrinsic = np.asarray(camera_matrix)
    novelty_deadzone = (max(math.atan2(1., intrinsic[0, 0]), math.atan2(1., intrinsic[1, 1]))
        if history_aware else 1e-10)
    ranked, rejected, identifiers = [], [], set()
    for candidate in candidates:
        if type(candidate) is not ObservationPoseCandidate:
            _fail('CANDIDATE_TYPE_INVALID')
        if candidate.candidate_id in identifiers:
            _fail('DUPLICATE_CANDIDATE_ID')
        identifiers.add(candidate.candidate_id)
        if current_joints is not None and candidate.joint_positions is None:
            rejected.append(dict(candidate_id=candidate.candidate_id,
                                 reason='RGB_OBSERVATION_VIEW_JOINT_HINT_MISSING'))
            continue
        if current_tool is not None and candidate.T_base_tool is None:
            rejected.append(dict(candidate_id=candidate.candidate_id,
                                 reason='RGB_OBSERVATION_VIEW_TOOL_HINT_MISSING'))
            continue
        transform = np.asarray(candidate.T_base_camera)
        try:
            pixels = project_rgb_observation_volume(volume, T_base_camera=transform,
                camera_matrix=camera_matrix, image_shape=image_shape,
                working_depth_range_m=working_depth_range_m)
        except ValueError as exc:
            rejected.append(dict(candidate_id=candidate.candidate_id, reason=str(exc)))
            continue
        if not _inside_roi(pixels, image_shape, margin):
            rejected.append(dict(candidate_id=candidate.candidate_id,
                                 reason='RGB_OBSERVATION_VIEW_CANDIDATE_VOLUME_OUTSIDE_ROI'))
            continue
        frozen_pixels = []
        for index, original_vertices in enumerate(frozen):
            try:
                projected = project_rgb_observation_vertices(original_vertices,
                    T_base_camera=transform, **projection_args)
                if not _inside_roi(projected, image_shape, margin):
                    _fail('CANDIDATE_FROZEN_REFERENCE_OUTSIDE_ROI')
                frozen_pixels.append(projected)
            except ValueError as exc:
                rejected.append(dict(candidate_id=candidate.candidate_id, reason=str(exc),
                    frozen_reference_index=index, frozen_reference_visibility_rejected=True))
                break
        if len(frozen_pixels) != len(frozen):
            continue
        acquisition_distance = None if radius is None else _acquisition_distance(pixels, centres)
        if acquisition_distance is not None and acquisition_distance > radius+1e-10:
            rejected.append(dict(candidate_id=candidate.candidate_id,
                reason='RGB_OBSERVATION_VIEW_CANDIDATE_VOLUME_OUTSIDE_ACQUISITION_WINDOW',
                maximum_acquisition_radius_px=acquisition_distance,
                acquisition_radius_px=radius))
            continue
        candidate_rays = vertices-transform[:3, 3]
        candidate_ranges = np.linalg.norm(candidate_rays, axis=1)
        unit = candidate_rays/candidate_ranges[:, None]
        # atan2 avoids acos roundoff pretending an unchanged camera gained information.
        angles = np.arctan2(np.linalg.norm(np.cross(previous_unit, unit), axis=2),
                            np.sum(previous_unit*unit, axis=2))
        raw_novelty = float(np.min(angles))
        raw_resolution_gain = max(0., float(np.min(best_ranges/candidate_ranges))-1.)
        utility_pixels = frozen_pixels[0] if frozen_pixels else pixels
        projected_short_span = float(np.min(np.ptp(np.asarray(utility_pixels), axis=0)))
        resolution_deadzone = (1./max(1.,projected_short_span) if history_aware else 1e-10)
        novelty = raw_novelty if raw_novelty >= novelty_deadzone else 0.
        resolution_gain = raw_resolution_gain if raw_resolution_gain >= resolution_deadzone else 0.
        novelty_utility = novelty/(novelty+novelty_scale)
        resolution_utility = resolution_gain/(resolution_gain+resolution_scale)
        utility = .65*novelty_utility+.35*resolution_utility
        if utility == 0.:
            rejected.append(dict(candidate_id=candidate.candidate_id,
                                 reason='RGB_OBSERVATION_VIEW_NO_PREDICTED_GAIN',
                                 raw_minimum_view_ray_change_rad=raw_novelty,
                                 raw_minimum_angular_sampling_gain_ratio=raw_resolution_gain,
                                 novelty_deadzone_rad=novelty_deadzone,
                                 resolution_deadzone_ratio=resolution_deadzone))
            continue
        translation = float(np.linalg.norm(transform[:3, 3]-current[:3, 3]))
        rotation = _rotation_angle(current[:3, :3], transform[:3, :3])
        motion_cost = translation/translation_scale+rotation/rotation_scale
        joint_l1 = joint_max = tool_translation = tool_rotation = None
        if candidate.joint_positions is not None:
            if current_joints is None or len(candidate.joint_positions) != len(current_joints):
                rejected.append(dict(candidate_id=candidate.candidate_id,
                                     reason='RGB_OBSERVATION_VIEW_JOINT_BINDING_MISSING'))
                continue
            delta = np.abs(np.asarray(candidate.joint_positions)-current_joints)
            joint_l1, joint_max = float(np.sum(delta)), float(np.max(delta))
            motion_cost += joint_l1/joint_scale
        if candidate.T_base_tool is not None:
            if current_tool is None:
                rejected.append(dict(candidate_id=candidate.candidate_id,
                                     reason='RGB_OBSERVATION_VIEW_TOOL_BINDING_MISSING'))
                continue
            tool = np.asarray(candidate.T_base_tool)
            tool_translation = float(np.linalg.norm(tool[:3, 3]-current_tool[:3, 3]))
            tool_rotation = _rotation_angle(current_tool[:3, :3], tool[:3, :3])
            motion_cost += tool_translation/translation_scale+tool_rotation/rotation_scale
        rank = information_motion_rank(utility, motion_cost,
            0. if joint_max is None else joint_max, 0. if joint_l1 is None else joint_l1)
        ranked.append(dict(candidate_id=candidate.candidate_id,
            proposal_kind=candidate.proposal_kind, T_base_camera=candidate.T_base_camera,
            joint_positions=candidate.joint_positions, T_base_tool=candidate.T_base_tool,
            projected_interest_vertices_px=pixels, all_interest_vertices_visible=True,
            projected_frozen_reference_vertices_px=frozen_pixels,
            frozen_reference_visibility_checked=bool(frozen),
            acquisition_window_checked=radius is not None,
            maximum_acquisition_radius_px=acquisition_distance,
            history_aware_utility=history_aware,
            raw_minimum_view_ray_change_rad=raw_novelty,
            raw_minimum_angular_sampling_gain_ratio=raw_resolution_gain,
            novelty_deadzone_rad=novelty_deadzone, resolution_deadzone_ratio=resolution_deadzone,
            minimum_view_ray_change_rad=novelty,
            minimum_angular_sampling_gain_ratio=resolution_gain,
            novelty_utility=novelty_utility, resolution_utility=resolution_utility,
            bounded_utility=utility, camera_translation_m=translation,
            camera_rotation_rad=rotation, joint_l1_delta_rad=joint_l1,
            joint_max_delta_rad=joint_max, tool_translation_m=tool_translation,
            tool_rotation_rad=tool_rotation,
            joint_motion_cost_available=joint_l1 is not None,
            tool_motion_cost_available=tool_translation is not None,
            motion_cost_kind='endpoint_geometric_and_available_joint_tool_proxy_not_duration_or_path',
            motion_cost_proxy=motion_cost, rank_key=rank,
            execution_authority=False, collision_checked=False))
    ranked.sort(key=lambda row: (row['rank_key'], row['candidate_id']))
    return dict(_preview_contract(volume), ranked=ranked, rejected=rejected,
        frozen_reference_visibility_checked=bool(frozen), frozen_reference_volume_count=len(frozen),
        utility_geometry='original_frozen_envelope' if frozen else 'current_interest_volume',
        observed_view_count=len(history), history_aware_utility=history_aware,
        gain_deadzone_pixels=1. if history_aware else None,
        history_gain_semantics='incremental_geometric_proxy_not_measured_information',
        acquisition_window_checked=radius is not None,
        acquisition_center_xy=tuple(float(v) for v in attention),
        acquisition_centres_px=_rows(centres),
        acquisition_centre_conventions=('initial_3d_dimension', 'initial_rgb_dimension_minus_one'),
        acquisition_radius_fraction=radius_fraction, acquisition_radius_px=radius,
        current_maximum_acquisition_radius_px=current_acquisition_distance,
        acquisition_scope='existing_perception_entry_window_not_rgbd_distance_band',
        roi_margin_px=margin, roi_margin_convention=(
            'pixel_centre_limits_margin_to_dimension_minus_one_minus_margin' if margin > 0.
            else 'full_pixel_area_outer_edges'),
        cost_scales=dict(translation_m=translation_scale, rotation_rad=rotation_scale,
            joint_rad=joint_scale, novelty_rad=novelty_scale,
            resolution_gain_ratio=resolution_scale),
        utility_weights=dict(view_novelty=.65, angular_sampling=.35))


def generate_local_rgb_observation_candidates(volume, *, current_T_base_camera,
        translation_steps_m=(.015, .03), orbit_angles_rad=(math.radians(3.), math.radians(6.))):
    """Generate at most 32 local, unplanned camera poses, without a distance band.

    Translations use each current camera axis, both signs. Small orbits use its
    left/up axes around the arithmetic mean of the eight *interest vertices*.
    This reference is explicitly not an estimated object centre. Orbiting does
    not rotate a guessed target model. No joint pose or trajectory is invented.
    The caller ranks full-volume visibility then independently solves IK/scene.
    """
    if type(volume) is not RGBObservationVolume:
        _fail('VOLUME_TYPE_INVALID')
    current = _transform(current_T_base_camera)
    for values in (translation_steps_m, orbit_angles_rad):
        if not isinstance(values, (tuple, list)) or len(values) > 2:
            _fail('LOCAL_STEPS_INVALID')
    steps = tuple(_positive_real(v, 'LOCAL_STEP_INVALID') for v in translation_steps_m)
    angles = tuple(_positive_real(v, 'LOCAL_ANGLE_INVALID') for v in orbit_angles_rad)
    if any(angle > math.radians(15.) for angle in angles):
        _fail('LOCAL_ORBIT_NOT_SMALL')
    if len(set(steps)) != len(steps) or len(set(angles)) != len(angles):
        _fail('DUPLICATE_LOCAL_STEP')
    reference = np.mean(np.asarray(volume.vertices_base_m), axis=0)
    candidates = []
    for index, step in enumerate(steps):
        for axis_index in range(3):
            for sign in (-1., 1.):
                moved = current.copy()
                moved[:3, 3] += sign*step*current[:3, axis_index]
                candidates.append(ObservationPoseCandidate(
                    'translation_%d_axis%d_%s' % (index, axis_index, 'minus' if sign < 0 else 'plus'),
                    moved, proposal_kind='camera_local_translation_without_ik'))
    for index, angle in enumerate(angles):
        for axis_index in (1, 2):
            axis = current[:3, axis_index]
            skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                             [-axis[1], axis[0], 0.]])
            for sign in (-1., 1.):
                a = sign*angle
                rotation = np.eye(3)+math.sin(a)*skew+(1.-math.cos(a))*(skew@skew)
                moved = current.copy()
                moved[:3, :3] = rotation@current[:3, :3]
                moved[:3, 3] = reference+rotation@(current[:3, 3]-reference)
                candidates.append(ObservationPoseCandidate(
                    'interest_orbit_%d_axis%d_%s' % (index, axis_index, 'minus' if sign < 0 else 'plus'),
                    moved, proposal_kind='interest_reference_orbit_without_ik'))
    return dict(_preview_contract(volume), candidates=tuple(candidates),
        interest_reference_base_m=tuple(float(v) for v in reference),
        interest_reference_semantics='mean_of_observation_volume_vertices_not_object_centre',
        observation_distance_band=None)
