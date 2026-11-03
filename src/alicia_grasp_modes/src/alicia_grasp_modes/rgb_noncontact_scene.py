"""Independent local non-contact checks for RGB-only observation acquisition.

The scene is a measured support plane, conservative source-depth beam boxes,
and an *additional* RGB-interest keepout. It is never a reconstructed target,
never fills unknown space with free space, and never replaces MoveIt's existing
whole-arm/environment checks. Camera housing and unseen environment remain
outside this local Link6/7/8 certificate, exactly as in the original guard.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math
import time

import numpy as np

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _box_corners)
from alicia_flexible_grasp.grasp.rich_plan_integrity import stamp_nanoseconds
from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, SerialUrdfFk, ObservationSupportBox, _axis_rotation,
    _array, _polynomial, _polynomial_bounds, segment_coefficients, wire_digest)
from .rgb_observation_volume import RGBObservationVolume


class RgbNoncontactError(ObservationPathError):
    def __init__(self, reason, audit=None):
        self.audit = dict(audit or {}, ok=False, reason=str(reason), motion_authorized=False)
        super().__init__(str(reason))


def _hash_array(values):
    return hashlib.sha256(np.ascontiguousarray(values).tobytes()).hexdigest()


def _immutable(values):
    array = np.array(values, dtype=float, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class FrozenRgbNoncontactScene:
    """Source-frozen local constraints, with no target/contact geometry fields."""
    scene_id: str
    digest: str
    source_ns: int
    normal: np.ndarray
    offset: float
    interest_bounds: np.ndarray
    depth_bounds: np.ndarray
    source_depth_sha256: str
    source_record_id: str
    depth_uncertainty_m: float
    valid_depth_range_m: tuple
    valid_depth_pixel_count: int
    unavailable_depth_pixel_count: int
    image_shape: tuple
    pixel_block_size: int
    depth_block_ids: tuple
    valid_mask: np.ndarray
    T_base_camera: np.ndarray
    camera_matrix: np.ndarray
    containment_reserve_m: float
    binding_json: str
    additional_interest_bounds: np.ndarray = field(default_factory=lambda:np.empty((0,2,3)))
    rectification_contract_json: str = ''
    rectification_beam_margin_px: tuple = (0., 0.)

    def __post_init__(self):
        record = json.loads(self.binding_json)
        if self.rectification_contract_json:
            from alicia_flexible_grasp.vision.rgbd_rectification import validate_rectification_contract
            contract = json.loads(self.rectification_contract_json)
            margin = validate_rectification_contract(contract)
            if (record.get('rectification') != contract
                    or tuple(self.rectification_beam_margin_px) != margin):
                raise RgbNoncontactError('RGB_NONCONTACT_RECTIFICATION_BINDING_INVALID')
        elif record.get('rectification') is not None or tuple(self.rectification_beam_margin_px) != (0., 0.):
            raise RgbNoncontactError('RGB_NONCONTACT_RECTIFICATION_BINDING_INVALID')
        if (hashlib.sha256(self.binding_json.encode()).hexdigest() != self.digest
                or self.scene_id != 'rgb-local:'+self.digest
                or self.source_ns != record['source_ns']
                or self.source_depth_sha256 != record['depth_sha256']
                or self.source_record_id != record['source_record_id']
                or not np.array_equal(np.r_[self.normal, self.offset], record['support_plane'])
                or _hash_array(self.depth_bounds) != record['depth_boxes_sha256']
                or not np.array_equal(self.interest_bounds, record['interest_bounds'])
                or not np.array_equal(self.additional_interest_bounds,
                    np.asarray(record['additional_interest_bounds']).reshape(-1,2,3))
                or _hash_array(self.valid_mask) != record['valid_mask_sha256']
                or tuple(self.depth_block_ids) != tuple(record['depth_block_ids'])
                or not np.array_equal(self.T_base_camera, record['T_base_camera'])
                or not np.array_equal(self.camera_matrix, record['camera_matrix'])
                or self.containment_reserve_m != record['containment_reserve_m']
                or self.depth_uncertainty_m != record['depth_uncertainty_m']
                or tuple(self.valid_depth_range_m) != tuple(record['depth_range_m'])
                or self.pixel_block_size != record['pixel_block_size']
                or tuple(self.image_shape) != tuple(record['image_shape'])
                or self.valid_depth_pixel_count != int(np.count_nonzero(self.valid_mask))
                or self.unavailable_depth_pixel_count != self.valid_mask.size-self.valid_depth_pixel_count):
            raise RgbNoncontactError('RGB_NONCONTACT_SCENE_BINDING_INVALID')
        for name in ('normal', 'interest_bounds', 'additional_interest_bounds',
                     'depth_bounds', 'T_base_camera', 'camera_matrix'):
            object.__setattr__(self, name, _immutable(getattr(self, name)))
        valid = np.array(self.valid_mask, dtype=bool, copy=True); valid.setflags(write=False)
        object.__setattr__(self, 'valid_mask', valid)

    @property
    def interest_bounds_all(self):
        return np.concatenate((self.interest_bounds[None],self.additional_interest_bounds))

    def scope(self):
        result = dict(scene_kind='rgb_acquisition_local_noncontact_constraints',
            source_stamp_ns=self.source_ns, scene_sha256=self.digest,
            source_depth_sha256=self.source_depth_sha256, source_record_id=self.source_record_id,
            depth_uncertainty_m=self.depth_uncertainty_m, containment_reserve_m=self.containment_reserve_m,
            measured_depth_pixel_count=self.valid_depth_pixel_count,
            unavailable_depth_pixel_count=self.unavailable_depth_pixel_count,
            depth_beam_box_count=len(self.depth_bounds), pixel_block_size=self.pixel_block_size,
            rgb_interest_keepout_count=len(self.interest_bounds_all),
            rgb_interest_role='additional_keepout_not_complete_object_geometry',
            source_depth_role='conservative_observed_surface_beams_not_complete_scene',
            unknown_depth_filled=False, unknown_space_assumed_free=False,
            complete_environment_certified=False, camera_housing_collision_certified=False,
            whole_arm_environment_owner='existing_independent_MoveIt_planning_scene',
            object_identity_created=False, motion_authorized=False)
        if self.rectification_contract_json:
            result.update(rectification_geometry_digest=json.loads(self.rectification_contract_json)['digest'],
                rectification_extra_beam_margin_px=list(self.rectification_beam_margin_px),
                rectification_source_cells_continuously_enclosed=True,
                rectification_all_source_samples_retained=True)
        return result


@dataclass(frozen=True)
class FrozenRgbNoncontactSceneBundle:
    """Finite occupied-space retention within one stationary acquisition episode.

    Keep initial, previous-proof and current measured frames, never accumulate
    an unbounded history. Old missing pixels retain their measured occupancy;
    they do not become free space or invented full-sensor-range obstacles.
    """
    scenes: tuple

    def __post_init__(self):
        if (type(self.scenes) is not tuple or not 2 <= len(self.scenes) <= 3
                or any(type(scene) is not FrozenRgbNoncontactScene for scene in self.scenes)
                or any(b.source_ns <= a.source_ns for a,b in zip(self.scenes,self.scenes[1:]))):
            raise RgbNoncontactError('RGB_NONCONTACT_RETAINED_SCENE_INVALID')

    @property
    def current(self): return self.scenes[-1]
    @property
    def source_ns(self): return self.current.source_ns
    @property
    def normal(self): return self.current.normal
    @property
    def offset(self): return self.current.offset
    @property
    def digest(self):
        return hashlib.sha256(json.dumps([s.digest for s in self.scenes]).encode()).hexdigest()
    @property
    def scene_id(self): return 'rgb-retained:'+self.digest
    @property
    def depth_bounds(self): return np.concatenate([s.depth_bounds for s in self.scenes])
    @property
    def interest_bounds_all(self): return np.concatenate([s.interest_bounds_all for s in self.scenes])

    def scope(self):
        report=self.current.scope()
        report.update(scene_kind='rgb_stationary_episode_retained_measured_constraints',
            scene_sha256=self.digest, retained_scene_sha256=[s.digest for s in self.scenes],
            retained_source_stamp_ns=[s.source_ns for s in self.scenes],
            retained_support_planes=[np.r_[s.normal,s.offset].tolist() for s in self.scenes],
            retained_measured_depth_pixel_count=sum(s.valid_depth_pixel_count for s in self.scenes),
            depth_beam_box_count=sum(len(s.depth_bounds) for s in self.scenes),
            rgb_interest_keepout_count=sum(len(s.interest_bounds_all) for s in self.scenes),
            retained_frame_count=len(self.scenes), maximum_retained_frame_count=3,
            lost_depth_occupancy_policy='retain_prior_measured_beams',
            certifies_new_unobserved_dynamic_obstacles=False,
            whole_depth_range_unknown_beams_created=False)
        return report


def retain_observed_obstacles(previous_scene, fresh_scene):
    """Retain bounded initial/previous/current occupied evidence, with new TTL.

    The caller must independently establish the same stationary episode,
    identity, camera/robot model and support consistency. Only the fresh source
    age can pass submission freshness; retained frames never renew their stamp.
    Every retained support plane is independently certified by the path proof.
    """
    if type(previous_scene) is FrozenRgbNoncontactScene:
        history=(previous_scene,)
    elif type(previous_scene) is FrozenRgbNoncontactSceneBundle:
        history=(previous_scene.scenes[0],previous_scene.current)
    else:
        raise RgbNoncontactError('RGB_NONCONTACT_RETAINED_SCENE_INVALID')
    prior=history[-1]
    if (type(fresh_scene) is not FrozenRgbNoncontactScene
            or fresh_scene.source_ns <= prior.source_ns
            or any(s.image_shape != fresh_scene.image_shape
                   or s.valid_depth_range_m != fresh_scene.valid_depth_range_m
                   or not np.array_equal(s.camera_matrix,fresh_scene.camera_matrix)
                   or s.rectification_contract_json != fresh_scene.rectification_contract_json
                   or s.depth_uncertainty_m > fresh_scene.depth_uncertainty_m for s in history)):
        raise RgbNoncontactError('RGB_NONCONTACT_RETAINED_SOURCE_MISMATCH')
    return FrozenRgbNoncontactSceneBundle(history+(fresh_scene,))


def _depth_beam_bounds(depth, valid, matrix, transform, block_size, uncertainty, reserve,
                       extra_pixel_margin=(0., 0.)):
    """Batch the original per-block beam construction without changing cells.

    Min/max reductions select the same valid pixel extrema as the scalar loop.
    Batched solves retain each independent 3-by-4 system and each 8-by-3 corner
    transform, including their arithmetic order. Empty blocks remain absent;
    partial edge blocks and holes never supply an extremum. The small geometry
    batches bound temporary corner storage independently of the image size.
    """
    height, width = depth.shape
    rows = np.arange(0, height, block_size)
    columns = np.arange(0, width, block_size)

    def reduce(values, operation):
        return operation.reduceat(operation.reduceat(values, rows, axis=0), columns, axis=1).ravel()

    minimum = reduce(np.where(valid, depth, np.inf), np.minimum)
    selected = np.flatnonzero(np.isfinite(minimum))
    maximum = reduce(np.where(valid, depth, -np.inf), np.maximum)[selected]
    lower = np.maximum(0., minimum[selected]-uncertainty)
    upper = maximum+uncertainty
    xx = np.arange(width)[None, :]
    yy = np.arange(height)[:, None]
    x0 = reduce(np.where(valid, xx, width), np.minimum)[selected]-.5
    x1 = reduce(np.where(valid, xx, -1), np.maximum)[selected]+.5
    y0 = reduce(np.where(valid, yy, height), np.minimum)[selected]-.5
    y1 = reduce(np.where(valid, yy, -1), np.maximum)[selected]+.5
    if any(extra_pixel_margin):
        x0 = np.nextafter(x0-extra_pixel_margin[0], -np.inf)
        x1 = np.nextafter(x1+extra_pixel_margin[0], np.inf)
        y0 = np.nextafter(y0-extra_pixel_margin[1], -np.inf)
        y1 = np.nextafter(y1+extra_pixel_margin[1], np.inf)
    bounds = np.empty((len(selected), 2, 3), dtype=float)
    for start in range(0, len(selected), 4096):
        section = slice(start, start+4096)
        count = len(selected[section])
        pixels = np.ones((count, 4, 3), dtype=float)
        pixels[:, :, 0] = np.stack((x0[section], x1[section], x1[section], x0[section]), axis=1)
        pixels[:, :, 1] = np.stack((y0[section], y0[section], y1[section], y1[section]), axis=1)
        optical = np.linalg.solve(matrix, pixels.swapaxes(1, 2)).swapaxes(1, 2)
        rays = np.stack((optical[:, :, 2], -optical[:, :, 0], -optical[:, :, 1]), axis=2)
        local = np.concatenate((rays*lower[section, None, None],
                                rays*upper[section, None, None]), axis=1)
        corners = local @ transform[:3, :3].T+transform[:3, 3]
        bounds[section, 0] = corners.min(axis=1)-reserve
        bounds[section, 1] = corners.max(axis=1)+reserve
    return bounds, selected.tolist()


def build_rgb_noncontact_scene(*, interest_volume, depth_image, depth_scale_m,
        source_stamp_ns, source_depth_sha256, depth_uncertainty_m,
        valid_depth_range_m, source_record_id, pixel_block_size=8, containment_reserve_m=0.,
        additional_interest_volumes=(), camera_geometry=None):
    """Cover every valid depth pixel's full beam cell at depth +/- uncertainty.

    Pixel blocks only *enlarge* occupancy to reduce query cost. Missing pixels
    do not become measured surfaces or free space. RGB-interest columns with
    unreliable depth are covered by the separate full-height interest keepout.
    Unobserved regions outside that interest retain the existing environment
    model responsibility. The uncertainty is a caller-supplied physical bound,
    not a claim inferred from the support fit's RMS statistic.
    """
    if type(interest_volume) is not RGBObservationVolume:
        raise RgbNoncontactError('RGB_NONCONTACT_INTEREST_TYPE_INVALID')
    if not isinstance(additional_interest_volumes,(tuple,list)) or len(additional_interest_volumes)>3:
        raise RgbNoncontactError('RGB_NONCONTACT_INTEREST_SET_BUDGET_INVALID')
    for extra in additional_interest_volumes:
        if (type(extra) is not RGBObservationVolume
                or extra.source_stamp_ns!=interest_volume.source_stamp_ns
                or extra.image_shape!=interest_volume.image_shape
                or extra.support_provenance!=interest_volume.support_provenance
                or extra.support_plane_base!=interest_volume.support_plane_base
                or extra.foreground_max_m!=interest_volume.foreground_max_m
                or extra.working_depth_range_m!=interest_volume.working_depth_range_m
                or not np.array_equal(extra.camera_matrix,interest_volume.camera_matrix)
                or not np.array_equal(extra.T_base_camera,interest_volume.T_base_camera)):
            raise RgbNoncontactError('RGB_NONCONTACT_INTEREST_SET_SOURCE_MISMATCH')
    if type(source_stamp_ns) is not int or source_stamp_ns != interest_volume.source_stamp_ns:
        raise RgbNoncontactError('RGB_NONCONTACT_SOURCE_STAMP_MISMATCH')
    image = np.asarray(depth_image)
    if (image.shape != interest_volume.image_shape or image.dtype not in
            (np.dtype('uint16'), np.dtype('float32'), np.dtype('float64'))
            or image.size > 2097152 or not image.size):
        raise RgbNoncontactError('RGB_NONCONTACT_DEPTH_IMAGE_INVALID')
    if (type(source_depth_sha256) is not str or len(source_depth_sha256) != 64
            or _hash_array(image) != source_depth_sha256):
        raise RgbNoncontactError('RGB_NONCONTACT_DEPTH_DIGEST_MISMATCH')
    if (type(source_record_id) is not str or not source_record_id.strip()
            or isinstance(depth_scale_m, (bool, np.bool_))
            or not isinstance(depth_scale_m, (int, float, np.floating))
            or not math.isfinite(depth_scale_m) or depth_scale_m <= 0.
            or isinstance(depth_uncertainty_m, (bool, np.bool_))
            or not isinstance(depth_uncertainty_m, (int, float, np.floating))
            or not math.isfinite(depth_uncertainty_m) or depth_uncertainty_m <= 0.
            or type(pixel_block_size) is not int or not 1 <= pixel_block_size <= 32
            or isinstance(containment_reserve_m, (bool, np.bool_))
            or not isinstance(containment_reserve_m, (int, float, np.floating))
            or not math.isfinite(containment_reserve_m) or containment_reserve_m < 0.):
        raise RgbNoncontactError('RGB_NONCONTACT_DEPTH_CONTRACT_INVALID')
    limits = _array(valid_depth_range_m, 2, 'depth range')
    if not 0. < limits[0] < limits[1]:
        raise RgbNoncontactError('RGB_NONCONTACT_DEPTH_RANGE_INVALID')
    depth = image.astype(float)*float(depth_scale_m)
    valid = np.isfinite(depth) & (depth >= limits[0]) & (depth <= limits[1])
    if not np.any(valid):
        raise RgbNoncontactError('RGB_NONCONTACT_NO_MEASURED_DEPTH')
    matrix, transform = np.asarray(interest_volume.camera_matrix), np.asarray(interest_volume.T_base_camera)
    contract, margin, contract_json = None, (0., 0.), ''
    if camera_geometry is not None:
        from alicia_flexible_grasp.vision.rgbd_rectification import rectification_contract, validate_rectification_contract
        contract = rectification_contract(camera_geometry)
        if contract is not None:
            margin = validate_rectification_contract(contract)
            output = contract['output_intrinsics']
            expected = np.array([[output['fx'], 0., output['cx']], [0., output['fy'], output['cy']], [0., 0., 1.]])
            if image.shape != (output['height'], output['width']) or not np.array_equal(matrix, expected):
                raise RgbNoncontactError('RGB_NONCONTACT_RECTIFICATION_GEOMETRY_MISMATCH')
            contract_json = json.dumps(contract, sort_keys=True, separators=(',', ':'), allow_nan=False)
    if contract is None:
        bounds, block_ids = _depth_beam_bounds(depth, valid, matrix, transform,
            pixel_block_size, depth_uncertainty_m, containment_reserve_m)
    else:
        bounds, block_ids = _depth_beam_bounds(depth, valid, matrix, transform,
            pixel_block_size, depth_uncertainty_m, containment_reserve_m, margin)
    interest = np.asarray(interest_volume.bounds_base_m)+np.array([[-1.], [1.]])*containment_reserve_m
    additional=np.asarray([np.asarray(v.bounds_base_m)+np.array([[-1.],[1.]])*containment_reserve_m
                           for v in additional_interest_volumes]).reshape(-1,2,3)
    plane = np.asarray(interest_volume.support_plane_base)
    record = dict(source_ns=source_stamp_ns, depth_sha256=source_depth_sha256,
        depth_scale_m=float(depth_scale_m), depth_uncertainty_m=float(depth_uncertainty_m),
        depth_range_m=limits.tolist(), pixel_block_size=pixel_block_size,
        source_record_id=source_record_id, interest_vertices=interest_volume.vertices_base_m,
        support_plane=plane.tolist(), camera_matrix=matrix.tolist(),
        T_base_camera=transform.tolist(), depth_boxes_sha256=_hash_array(bounds),
        interest_bounds=interest.tolist(), containment_reserve_m=float(containment_reserve_m),
        additional_interest_bounds=additional.tolist(),
        additional_interest_vertices=[v.vertices_base_m for v in additional_interest_volumes],
        depth_block_ids=block_ids, valid_mask_sha256=_hash_array(valid), image_shape=list(image.shape))
    if contract is not None:
        record['rectification'] = contract
    binding = json.dumps(record, sort_keys=True, allow_nan=False)
    digest = hashlib.sha256(binding.encode()).hexdigest()
    return FrozenRgbNoncontactScene('rgb-local:'+digest, digest, source_stamp_ns,
        _immutable(plane[:3]), float(plane[3]), _immutable(interest), _immutable(bounds),
        source_depth_sha256, source_record_id, float(depth_uncertainty_m), tuple(limits),
        int(valid.sum()), int(valid.size-valid.sum()), tuple(image.shape), pixel_block_size,
        tuple(block_ids), valid, transform, matrix, float(containment_reserve_m), binding, additional,
        contract_json, margin)


class _JointBoxObstacles:
    """Taylor-enclose every CAD corner for all joints in an entire error box."""
    def __init__(self, fk, gripper, opening, scene):
        self.fk, self.scene = fk, scene
        identity = np.eye(4)
        parts = _stage_boxes(identity, gripper, opening, 'y', 'z')
        self.part_names = [name for name, _, _ in parts]
        self.local = np.concatenate([_box_corners(center, identity[:3, :3], size)
                                     for _, center, size in parts])
        self.local_boxes = parts
        radii = np.array([fk.corner_motion_radii(point) for point in self.local])
        self.hessian = np.minimum(radii[:, :, None], radii[:, None, :])
        interests = scene.interest_bounds_all
        self.interest_count = len(interests)
        self.obstacles = np.concatenate((interests, scene.depth_bounds))

    def enclosure(self, center, half_width):
        pose = np.eye(4)
        origins, axes = np.zeros((len(center), 3)), np.zeros((len(center), 3))
        for origin, index, axis in self.fk.chain:
            pose = pose @ origin
            if index is not None:
                origins[index] = pose[:3, 3]
                axes[index] = pose[:3, :3] @ axis
                pose[:3, :3] = pose[:3, :3] @ _axis_rotation(axis, center[index])
        points = self.local @ pose[:3, :3].T+pose[:3, 3]
        gradient = np.cross(axes[None], points[:, None]-origins[None])
        remainder = .5*np.einsum('i,cij,j->c', half_width, self.hessian, half_width)
        allowance = np.einsum('cij,i->cj', np.abs(gradient), half_width)+remainder[:, None]+1e-12
        low, high = points-allowance, points+allowance
        bounds = np.array([[low[i:i+8].min(axis=0), high[i:i+8].max(axis=0)]
                           for i in range(0, len(points), 8)])
        scores = half_width*np.max(np.linalg.norm(gradient, axis=2), axis=0)
        scores += half_width*np.max(np.einsum('cij,j->ci', self.hessian, half_width), axis=0)
        return bounds, pose, scores, points, gradient, remainder

    def unresolved_pairs(self, bounds, inherited=None):
        # A parent joint-box certificate covers all its descendants. Once an
        # obstacle is proved separated on the parent, never rescan it in either
        # child. This changes work only, not the occupied geometry or joint box.
        previous = ([(part,np.arange(len(self.obstacles))) for part in range(len(bounds))]
                    if inherited is None else inherited)
        result=[]
        for part,indices in previous:
            boxes=self.obstacles[indices]
            overlap=(np.all(bounds[part,1]>=boxes[:,0],axis=1)
                     & np.all(bounds[part,0]<=boxes[:,1],axis=1))
            result.append((part,indices[overlap]))
        return result

    def check(self, lower, upper, *, budget, clock, started, max_seconds):
        stack = [(.5*(lower+upper), .5*(upper-lower), None)]
        while stack:
            if budget[0] <= 0 or clock()-started > max_seconds:
                raise RgbNoncontactError('RGB_NONCONTACT_LOCAL_OBSTACLE_PROOF_BUDGET_EXHAUSTED')
            center, width, inherited = stack.pop()
            budget[0] -= 1
            bounds, pose, scores, points, gradient, remainder = self.enclosure(center, width)
            pairs = []
            # A base AABB around a rotated CAD box need not converge to its
            # exact shape. Use additional fixed SAT axes from this box centre;
            # Taylor bounds still enclose every uncertain posture on each axis.
            rotation_axes=pose[:3,:3].T
            crosses=np.cross(rotation_axes[:,None,:],np.eye(3)[None,:,:]).reshape(-1,3)
            lengths=np.linalg.norm(crosses,axis=1)
            active=lengths>1e-9
            directions=np.vstack((np.eye(3),rotation_axes,crosses[active]/lengths[active,None]))
            for part, indices in self.unresolved_pairs(bounds,inherited):
                if not len(indices):
                    continue
                sl = slice(8*part, 8*(part+1))
                projected = points[sl]@directions.T
                allowance = np.einsum('cid,i->cd',
                    np.abs(np.einsum('cij,dj->cid', gradient[sl], directions)), width)
                allowance += remainder[sl, None]+1e-12
                low = np.min(projected-allowance, axis=0)
                high = np.max(projected+allowance, axis=0)
                candidate_boxes = self.obstacles[indices]
                centres = .5*(candidate_boxes[:, 0]+candidate_boxes[:, 1])@directions.T
                radii = .5*(candidate_boxes[:, 1]-candidate_boxes[:, 0])@np.abs(directions.T)
                separated = np.any((high[None] < centres-radii) | (low[None] > centres+radii), axis=1)
                if np.all(separated):
                    continue
                # Same complete OBB/AABB SAT as the former per-obstacle helper,
                # batched on axes shared by this CAD part. Keep its 1 nm contact
                # tolerance; nominal overlap is a rejection, never acceptance.
                nominal_separated=np.any(
                    (projected.max(axis=0)[None] < centres-radii-1e-9)
                    | (projected.min(axis=0)[None] > centres+radii+1e-9),axis=1)
                nominal_overlap=np.flatnonzero(~separated & ~nominal_separated)
                if len(nominal_overlap):
                    index=int(indices[nominal_overlap[0]])
                    raise RgbNoncontactError('RGB_NONCONTACT_LOCAL_KEEPOUT_OVERLAP',
                        dict(failure_kind='rgb_interest_keepout' if index < self.interest_count else 'source_depth_beam',
                             gripper_part=self.part_names[part],obstacle_index=index,
                             counterexample_joint_positions_rad=center.tolist(),
                             obstacle_bounds_base_m=self.obstacles[index].tolist(),
                             physical_collision_proven=False))
                pairs.append((part,indices[~separated]))
            if not pairs:
                continue
            axis = int(np.argmax(scores))
            if width[axis] < 1e-8:
                raise RgbNoncontactError('RGB_NONCONTACT_LOCAL_OBSTACLE_PRECISION_UNRESOLVED')
            child = width.copy(); child[axis] *= .5
            shift = np.zeros_like(width); shift[axis] = child[axis]
            stack.extend(((center-shift,child,pairs),(center+shift,child,pairs)))
        return True


def validate_rgb_noncontact_trajectory(plan, scene, fk, gripper, opening,
        controller_desired, max_checks=32768, max_depth=20, max_seconds=25.,
        clock=time.monotonic, joint_error_bounds_rad=None):
    """Continuous local gripper proof, including actual bridge and joint band.

    Reuse the original support Taylor proof and Noetic spline coefficients.
    All source-depth boxes and RGB keepout are excluded for every CAD point in
    the commanded joint interval PLUS the supplied following/stopping error box.
    This cannot certify tracking hardware, calibration, or complete environment.
    """
    started = clock()
    audit = dict(scope='continuous_Link6_Link7_Link8_rgb_local_noncontact_only',
        ok=False, motion_authorized=False, physical_execution_verified=False)
    try:
        if type(scene) not in (FrozenRgbNoncontactScene,FrozenRgbNoncontactSceneBundle) or not isinstance(fk, SerialUrdfFk):
            raise RgbNoncontactError('RGB_NONCONTACT_SCENE_OR_FK_INVALID')
        if not isinstance(gripper, GripperGeometry) or gripper.support_clearance_m < .003:
            raise RgbNoncontactError('RGB_NONCONTACT_ORIGINAL_3MM_CLEARANCE_REQUIRED')
        if not math.isfinite(opening) or not 0. <= opening <= gripper.max_inner_gap_m:
            raise RgbNoncontactError('RGB_NONCONTACT_OPENING_INVALID')
        if (type(max_checks) is not int or max_checks < 1 or type(max_depth) is not int
                or not 0 <= max_depth <= 30 or not math.isfinite(max_seconds) or max_seconds <= 0.):
            raise RgbNoncontactError('RGB_NONCONTACT_BUDGET_INVALID')
        trajectory = plan.joint_trajectory
        if (tuple(trajectory.joint_names) != fk.names
                or stamp_nanoseconds(trajectory.header.stamp) != 0
                or getattr(getattr(plan, 'multi_dof_joint_trajectory', None), 'points', ())):
            raise RgbNoncontactError('RGB_NONCONTACT_TRAJECTORY_HEADER_INVALID')
        points = list(trajectory.points)
        times = [float(point.time_from_start.to_sec()) for point in points]
        if (not 2 <= len(points) <= 4096 or not all(math.isfinite(t) and t >= 0. for t in times)
                or any(b <= a for a, b in zip(times, times[1:]))):
            raise RgbNoncontactError('RGB_NONCONTACT_TRAJECTORY_TIME_INVALID')
        hold = _array(controller_desired.positions, len(fk.names), 'stationary hold')
        for name in ('velocities', 'accelerations'):
            if np.any(_array(getattr(controller_desired, name, ()), len(fk.names), name) != 0.):
                raise RgbNoncontactError('RGB_NONCONTACT_HOLD_NOT_STATIONARY')
        if joint_error_bounds_rad is None:
            raise RgbNoncontactError('RGB_NONCONTACT_FOLLOWING_CONTRACT_REQUIRED')
        epsilon = _array(joint_error_bounds_rad, len(fk.names), 'following error')
        if np.any(epsilon <= 0.) or np.any(epsilon > math.pi):
            raise RgbNoncontactError('RGB_NONCONTACT_FOLLOWING_CONTRACT_INVALID')
        first = next((i for i, value in enumerate(times) if value > 0.), None)
        if first is None:
            raise RgbNoncontactError('RGB_NONCONTACT_NO_POSITIVE_TIME_WAYPOINT')
        segments = [(segment_coefficients(controller_desired, points[first], times[first], len(fk.names)), 'bridge')]
        if first:
            segment_coefficients(points[0], points[first], times[first], len(fk.names))
        for i in range(first, len(points)-1):
            segments.append((segment_coefficients(points[i], points[i+1], times[i+1]-times[i], len(fk.names)), str(i)))
        support_scenes = scene.scenes if type(scene) is FrozenRgbNoncontactSceneBundle else (scene,)
        supports = [ObservationSupportBox(fk,gripper,opening,s.normal,s.offset) for s in support_scenes]
        obstacles = _JointBoxObstacles(fk, gripper, opening, scene)
        remaining = [max_checks]
        minimum = float('inf')
        checks = interval_checks = 0
        swept_bounds = []
        certified_boxes=set()
        repeated_box_hits=0
        audit.update(scene.scope(), trajectory_sha256=wire_digest(plan), model_sha256=fk.model_sha256,
            opening_width_m=float(opening), bridge_start_joint_positions_rad=hold.tolist(),
            bridge='stationary_desired_to_first_positive_time_waypoint',
            joint_error_bounds_rad=epsilon.tolist())

        def certify(lower, upper, location):
            nonlocal minimum, checks, repeated_box_hits
            if remaining[0] <= 0 or clock()-started >= max_seconds:
                raise RgbNoncontactError('RGB_NONCONTACT_PATH_PROOF_BUDGET_EXHAUSTED')
            # Scope is this one immutable scene/model/opening/error contract.
            # A repeated hold or dropped t=0 point adds no new joint states.
            key=(np.ascontiguousarray(lower).tobytes(),np.ascontiguousarray(upper).tobytes())
            if key in certified_boxes:
                repeated_box_hits+=1
                return
            for support_scene,support in zip(support_scenes,supports):
                report = support.check(lower-epsilon, upper+epsilon,
                    max_checks=remaining[0], max_seconds=max_seconds-(clock()-started), clock=clock)
                remaining[0] -= report['box_checks']; checks += report['box_checks']
                if not report['ok']:
                    raise RgbNoncontactError('RGB_NONCONTACT_FOLLOWING_SUPPORT_INVALID',
                        dict(report, failure_location=location, support_source_stamp_ns=support_scene.source_ns))
                minimum = min(minimum, report['minimum_support_clearance_lower_bound_m'])
            obstacles.check(lower-epsilon, upper+epsilon, budget=remaining,
                            clock=clock, started=started, max_seconds=max_seconds)
            bounds = obstacles.enclosure(.5*(lower+upper), .5*(upper-lower)+epsilon)[0]
            swept_bounds.extend(bounds.tolist())
            certified_boxes.add(key)

        # Neither the existing controller hold nor the dropped t=0 point may
        # bypass source depth/keepouts. These are actual finite geometry checks.
        for label, q in [('controller_hold', hold), ('terminal', _array(points[-1].positions, len(fk.names), 'terminal'))]:
            fk(q)
            certify(q, q, label)
        if first:
            q = _array(points[0].positions, len(fk.names), 'dropped start')
            fk(q); certify(q, q, 'dropped_start')
        for coefficients, label in segments:
            stack = [(0., 1., 0)]
            while stack:
                lo, hi, level = stack.pop()
                lower, upper = _polynomial_bounds(coefficients, lo, hi)
                interval_checks += 1
                if np.any(lower < fk.limits[:, 0]) or np.any(upper > fk.limits[:, 1]):
                    for t in (lo, .5*(lo+hi), hi):
                        fk(_polynomial(coefficients, t))
                    if level >= max_depth:
                        raise RgbNoncontactError('RGB_NONCONTACT_JOINT_LIMIT_ENCLOSURE_UNRESOLVED')
                    mid = .5*(lo+hi)
                    stack.extend(((lo, mid, level+1), (mid, hi, level+1)))
                    continue
                try:
                    certify(lower, upper, label)
                except RgbNoncontactError as exc:
                    witness = exc.audit.get('counterexample_joint_positions_rad')
                    if witness is not None and np.all(np.abs(np.asarray(witness)-_polynomial(coefficients, .5*(lo+hi))) <= epsilon):
                        raise
                    if level >= max_depth or remaining[0] <= 0 or clock()-started >= max_seconds:
                        raise
                    mid = .5*(lo+hi)
                    stack.extend(((lo, mid, level+1), (mid, hi, level+1)))
        audit.update(ok=True, certified_minimum_clearance_m=minimum,
            certified_minimum_local_support_clearance_m=minimum,
            interval_checks=interval_checks, box_checks=max_checks-remaining[0],
            repeated_joint_box_certificate_hits=repeated_box_hits,
            certified_swept_cad_bounds_base_m=swept_bounds,
            required_support_clearance_m=float(gripper.support_clearance_m),
            following_support=dict(scope='conditional_continuous_local_support_and_keepouts',
                joint_error_bounds_rad=epsilon.tolist(), minimum_support_clearance_lower_bound_m=minimum,
                box_checks=checks, includes_controller_bridge=True,
                local_depth_and_rgb_keepout_tracking_uncertainty_checked=True,
                certifies_hardware_tracking_stopping_or_calibration=False),
            elapsed_sec=clock()-started)
        return audit
    except (ValueError, TypeError, AttributeError, OverflowError) as exc:
        audit.update(getattr(exc, 'audit', {}), elapsed_sec=clock()-started)
        raise RgbNoncontactError(str(exc), audit) from exc


def revalidate_rgb_noncontact_scene(previous_scene, fresh_scene, audit):
    """Geometric fresh-frame receipt for an unchanged, already proved path.

    This never renews the source timestamp or leases movement. The caller must
    separately verify fresh source age, same selected RGB interest/epoch/model,
    unchanged path/hold, and the receipt at submission. Newly measured obstacles
    must be inside an old certified obstacle box OR disjoint from all proved CAD
    swept enclosures. Newly missing depth is checked as an unknown full-range
    beam, not silently deleted. Support-plane rise is bounded over every swept
    enclosure vertex, consuming actual proved clearance rather than a looser
    tabletop-consistency threshold.
    """
    started = time.monotonic()
    if (type(previous_scene) is not FrozenRgbNoncontactScene
            or type(fresh_scene) is not FrozenRgbNoncontactScene
            or not isinstance(audit, dict) or audit.get('ok') is not True
            or audit.get('scene_sha256') != previous_scene.digest
            or not fresh_scene.source_ns > previous_scene.source_ns
            or fresh_scene.image_shape != previous_scene.image_shape
            or fresh_scene.pixel_block_size != previous_scene.pixel_block_size
            or fresh_scene.valid_depth_range_m != previous_scene.valid_depth_range_m
            or not np.array_equal(fresh_scene.camera_matrix, previous_scene.camera_matrix)
            or fresh_scene.rectification_contract_json != previous_scene.rectification_contract_json
            or fresh_scene.depth_uncertainty_m < previous_scene.depth_uncertainty_m):
        raise RgbNoncontactError('RGB_NONCONTACT_FRESH_BINDING_INVALID')
    swept = np.asarray(audit.get('certified_swept_cad_bounds_base_m'), dtype=float)
    if (swept.ndim != 3 or swept.shape[1:] != (2, 3) or not len(swept)
            or not np.all(np.isfinite(swept)) or np.any(swept[:, 0] > swept[:, 1])):
        raise RgbNoncontactError('RGB_NONCONTACT_SWEPT_ENCLOSURES_MISSING')
    clearance = float(audit['certified_minimum_clearance_m'])
    required = float(audit['required_support_clearance_m'])
    if not math.isfinite(clearance) or not math.isfinite(required) or required < .003:
        raise RgbNoncontactError('RGB_NONCONTACT_SUPPORT_CERTIFICATE_INVALID')
    # max (old signed height - new signed height), exact over each AABB.
    delta = previous_scene.normal-fresh_scene.normal
    worst_points = np.where(delta[None] >= 0., swept[:, 1], swept[:, 0])
    rise = float(np.max(worst_points@delta+previous_scene.offset-fresh_scene.offset))
    if clearance-rise < required:
        raise RgbNoncontactError('RGB_NONCONTACT_FRESH_SUPPORT_MARGIN_EXCEEDED',
            dict(maximum_support_rise_m=rise, available_support_margin_m=clearance-required,
                 fresh_clearance_lower_bound_m=clearance-rise))

    def disjoint(box, beam_vertices=None):
        # The continuous support certificate is tighter than its world AABB:
        # a newly measured table beam wholly below that proved half-space cannot
        # intersect a CAD point, even if their loose AABBs overlap.
        upper_support_point = np.where(previous_scene.normal >= 0., box[1], box[0])
        if float(upper_support_point@previous_scene.normal+previous_scene.offset) < clearance:
            return True
        if np.all(np.any((box[1] < swept[:, 0]) | (box[0] > swept[:, 1]), axis=1)):
            return True
        return False

    def contained(new, old):
        return bool(np.all(new[0] >= old[0]) and np.all(new[1] <= old[1]))

    for interest in fresh_scene.interest_bounds_all:
        if not any(contained(interest,old) for old in previous_scene.interest_bounds_all) and not disjoint(interest):
            raise RgbNoncontactError('RGB_NONCONTACT_FRESH_INTEREST_NOT_COVERED')
    old_boxes = dict(zip(previous_scene.depth_block_ids, previous_scene.depth_bounds))
    reused = independently_clear = 0
    for identity, box in zip(fresh_scene.depth_block_ids, fresh_scene.depth_bounds):
        if identity in old_boxes and contained(box, old_boxes[identity]):
            reused += 1
        elif disjoint(box):
            independently_clear += 1
        else:
            raise RgbNoncontactError('RGB_NONCONTACT_FRESH_DEPTH_NOT_COVERED',
                                    dict(depth_block_id=identity, obstacle_bounds_base_m=box.tolist()))
    holes = _validate_new_depth_holes(previous_scene, fresh_scene, disjoint)
    return dict(ok=True, kind='fresh_measured_scene_geometric_revalidation',
        previous_scene_sha256=previous_scene.digest, fresh_scene_sha256=fresh_scene.digest,
        fresh_source_stamp_ns=fresh_scene.source_ns, trajectory_sha256=audit['trajectory_sha256'],
        source_timestamp_renewed=False, motion_authorized=False,
        maximum_support_rise_m=rise, fresh_clearance_lower_bound_m=clearance-rise,
        contained_depth_boxes=reused, independently_clear_new_depth_boxes=independently_clear,
        newly_unavailable_depth_pixels=holes,
        unknown_space_assumed_free=False, elapsed_sec=time.monotonic()-started)


def _validate_new_depth_holes(previous_scene, fresh_scene, disjoint):
    holes = previous_scene.valid_mask & ~fresh_scene.valid_mask
    yy, xx = np.nonzero(holes)
    block = fresh_scene.pixel_block_size
    columns = int(math.ceil(fresh_scene.image_shape[1]/block))
    identities = (yy//block)*columns+xx//block
    inverse = np.linalg.inv(fresh_scene.camera_matrix)
    transform = fresh_scene.T_base_camera
    for identity in np.unique(identities):
        selected = identities == identity
        u, v = xx[selected], yy[selected]
        pixels = np.array([[u.min()-.5, v.min()-.5, 1.], [u.max()+.5, v.min()-.5, 1.],
                           [u.max()+.5, v.max()+.5, 1.], [u.min()-.5, v.max()+.5, 1.]])
        mx, my = fresh_scene.rectification_beam_margin_px
        if mx or my:
            pixels[:, 0] = np.nextafter(pixels[:, 0]+np.array([-mx, mx, mx, -mx]),
                                        np.array([-np.inf, np.inf, np.inf, -np.inf]))
            pixels[:, 1] = np.nextafter(pixels[:, 1]+np.array([-my, -my, my, my]),
                                        np.array([-np.inf, -np.inf, np.inf, np.inf]))
        optical = pixels@inverse.T
        rays = np.c_[optical[:, 2], -optical[:, 0], -optical[:, 1]]
        local = np.vstack((rays*fresh_scene.valid_depth_range_m[0], rays*fresh_scene.valid_depth_range_m[1]))
        points = local@transform[:3, :3].T+transform[:3, 3]
        box = np.array([points.min(axis=0), points.max(axis=0)])
        if not disjoint(box, points):
            raise RgbNoncontactError('RGB_NONCONTACT_NEW_DEPTH_HOLE_INTERSECTS_SWEPT_REGION',
                                    dict(depth_block_id=int(identity), unknown_beam_bounds_base_m=box.tolist()))
    return int(len(xx))
