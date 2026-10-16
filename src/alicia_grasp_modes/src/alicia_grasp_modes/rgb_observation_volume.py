"""RGB interest rays clipped by an independently accepted support-height slab.

This is *observation interest*, not an object reconstruction. Its eight model
vertices are never depth measurements. Occluded parts may lie outside the RGB
box/slab, so this volume is not complete object or collision geometry, and does
not authorize motion. Collision clearance, fresh TF/image association, target
identity and motion planning remain independent caller responsibilities.

Camera coordinates are ROS camera_link (forward, left, up): optical [x,y,z]
maps to [z,-x,-y]. Depth limits refer to optical Z / camera_link X, not range.
"""
from dataclasses import dataclass, field
from numbers import Real

import numpy as np


_PREFIX = 'RGB_OBSERVATION_VOLUME_'
_TOLERANCE = 1e-10


def _fail(code):
    raise ValueError(_PREFIX + code)


def _real_array(value, shape, code):
    try:
        raw = np.asarray(value, dtype=object)
        if raw.shape != shape or any(
                isinstance(item, (bool, np.bool_)) or not isinstance(item, Real)
                for item in raw.flat):
            _fail(code)
        result = np.asarray(raw, dtype=float)
        if not np.all(np.isfinite(result)):
            _fail(code)
        return result
    except (TypeError, ValueError, OverflowError):
        _fail(code)


def _positive_real(value, code):
    result = _real_array([value], (1,), code)[0]
    if result <= 0.:
        _fail(code)
    return float(result)


def _stamp(value):
    if type(value) is not int or value <= 0:
        _fail('SOURCE_STAMP_INVALID')
    return value


def _shape(value):
    if (not isinstance(value, (tuple, list)) or len(value) != 2
            or any(type(item) is not int or item <= 0 for item in value)):
        _fail('IMAGE_SHAPE_INVALID')
    return tuple(value)


def _intrinsics(value):
    matrix = _real_array(value, (3, 3), 'INTRINSICS_INVALID')
    if (matrix[0, 0] <= 0. or matrix[1, 1] <= 0.
            or not np.array_equal(matrix[2], [0., 0., 1.])
            or matrix[1, 0] != 0.):
        _fail('INTRINSICS_INVALID')
    return matrix


def _transform(value):
    transform = _real_array(value, (4, 4), 'TRANSFORM_INVALID')
    rotation = transform[:3, :3]
    if (not np.array_equal(transform[3], [0., 0., 0., 1.])
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0.)
            or abs(float(np.linalg.det(rotation))-1.) > 1e-8):
        _fail('TRANSFORM_INVALID')
    return transform


def _plane(value):
    plane = _real_array(value, (4,), 'SUPPORT_PLANE_INVALID')
    if abs(float(np.linalg.norm(plane[:3]))-1.) > 1e-8:
        _fail('SUPPORT_PLANE_NOT_UNIT')
    return plane


def _depth_range(value):
    limits = _real_array(value, (2,), 'WORKING_DEPTH_RANGE_INVALID')
    if not 0. < limits[0] < limits[1]:
        _fail('WORKING_DEPTH_RANGE_INVALID')
    return limits


def _rows(value):
    return tuple(tuple(float(item) for item in row) for row in value)


@dataclass(frozen=True)
class SupportPlaneProvenance:
    """Caller evidence that a measured plane passed independent support checks.

    This type records an existing verdict; it does not run or replace those
    quality checks. Its recorded fit, inlier and spatial coverage must meet the
    supplied criteria, which cannot relax the original 2 mm / 70% / 120-inlier /
    3-quadrant limits. The plane and exact RGB-D stamp must match the volume input.
    RGB appearance, box/slab vertices and assumed target heights cannot supply
    support evidence. ``source_record_id`` identifies the caller's retained
    measurement/quality record, not necessarily a filesystem path.
    """
    source_stamp_ns: int
    support_plane_base: tuple
    source_record_id: str
    inlier_count: int
    inlier_ratio: float
    fit_rmse_m: float
    spatial_quadrants: int
    max_fit_rmse_m: float = .002
    min_inlier_ratio: float = .70
    min_spatial_quadrants: int = 3
    min_inlier_count: int = 120
    estimator: str = 'independent_measured_depth_support'
    quality_accepted: bool = True

    def __post_init__(self):
        _stamp(self.source_stamp_ns)
        plane = _plane(self.support_plane_base)
        if (self.estimator != 'independent_measured_depth_support'
                or self.quality_accepted is not True
                or type(self.source_record_id) is not str
                or not self.source_record_id.strip()
                or type(self.inlier_count) is not int
                or type(self.min_inlier_count) is not int
                or not 120 <= self.min_inlier_count <= self.inlier_count
                or type(self.spatial_quadrants) is not int
                or type(self.min_spatial_quadrants) is not int
                or not 3 <= self.min_spatial_quadrants <= self.spatial_quadrants <= 4):
            _fail('SUPPORT_PROVENANCE_INVALID')
        metrics = _real_array([self.inlier_ratio, self.fit_rmse_m,
                               self.max_fit_rmse_m, self.min_inlier_ratio], (4,),
                              'SUPPORT_PROVENANCE_INVALID')
        if (not .70 <= metrics[3] <= metrics[0] <= 1.
                or not 0. <= metrics[1] <= metrics[2] <= .002
                or metrics[2] == 0.):
            _fail('SUPPORT_PROVENANCE_INVALID')
        object.__setattr__(self, 'support_plane_base', tuple(float(v) for v in plane))
        object.__setattr__(self, 'inlier_ratio', float(metrics[0]))
        object.__setattr__(self, 'fit_rmse_m', float(metrics[1]))
        object.__setattr__(self, 'max_fit_rmse_m', float(metrics[2]))
        object.__setattr__(self, 'min_inlier_ratio', float(metrics[3]))


@dataclass(frozen=True)
class RGBObservationVolume:
    """Immutable slab-clipped visible RGB interest frustum; no contact evidence.

    Vertices 0..3 intersect the support plane; 4..7 intersect its upper slab
    plane. Each group is ordered top-left, top-right, bottom-right, bottom-left
    in the original image. ``bounds_base_m`` is an axis-aligned interest bound,
    not an occupied object box. The height is a caller-supplied scene assumption
    shared across unknown objects, not a measured object height.
    """
    bbox_xywh: tuple
    image_shape: tuple
    camera_matrix: tuple
    T_base_camera: tuple
    support_plane_base: tuple
    support_provenance: SupportPlaneProvenance
    foreground_max_m: float
    source_stamp_ns: int
    working_depth_range_m: tuple
    pixel_boundary_xy: tuple = field(init=False)
    vertices_base_m: tuple = field(init=False)
    vertex_optical_depths_m: tuple = field(init=False)
    bounds_base_m: tuple = field(init=False)
    camera_support_height_m: float = field(init=False)
    source_semantics: str = field(init=False, default='rgb_visible_interest_rays_with_measured_support_and_assumed_height')
    vertex_semantics: str = field(init=False, default='constructed_observation_volume_vertices_not_measurements')
    observation_only: bool = field(init=False, default=True)
    complete_object_volume: bool = field(init=False, default=False)
    usable_as_collision_geometry: bool = field(init=False, default=False)
    executable_contact_geometry: bool = field(init=False, default=False)
    motion_authorized: bool = field(init=False, default=False)
    synthetic_measured_points_added: int = field(init=False, default=0)

    def __post_init__(self):
        stamp = _stamp(self.source_stamp_ns)
        shape = _shape(self.image_shape)
        box = _real_array(self.bbox_xywh, (4,), 'BBOX_INVALID')
        x, y, width, height = box
        if (x < 0. or y < 0. or width <= 0. or height <= 0.
                or x+width > shape[1] or y+height > shape[0]):
            _fail('BBOX_OUTSIDE_IMAGE')
        matrix = _intrinsics(self.camera_matrix)
        transform = _transform(self.T_base_camera)
        plane = _plane(self.support_plane_base)
        maximum_height = _positive_real(self.foreground_max_m, 'FOREGROUND_HEIGHT_INVALID')
        limits = _depth_range(self.working_depth_range_m)
        provenance = self.support_provenance
        if type(provenance) is not SupportPlaneProvenance:
            _fail('SUPPORT_PROVENANCE_REQUIRED')
        if provenance.source_stamp_ns != stamp:
            _fail('SUPPORT_SOURCE_STAMP_MISMATCH')
        if not np.array_equal(plane, provenance.support_plane_base):
            _fail('SUPPORT_PROVENANCE_PLANE_MISMATCH')
        normal, offset = plane[:3], plane[3]
        origin, rotation = transform[:3, 3], transform[:3, :3]
        camera_height = float(normal @ origin + offset)
        if camera_height <= 0.:
            _fail('SUPPORT_NOT_FACING_CAMERA')
        if camera_height <= maximum_height + _TOLERANCE:
            _fail('CAMERA_INSIDE_OR_TOUCHING_SLAB')
        # The box encloses pixel centres x..x+w-1, including all pixel areas.
        # Do not shrink to centre rays or clip the interest border.
        pixels = np.array([[x-.5, y-.5], [x+width-.5, y-.5],
                           [x+width-.5, y+height-.5], [x-.5, y+height-.5]])
        optical = np.linalg.solve(matrix, np.c_[pixels, np.ones(4)].T).T
        camera_rays = np.c_[optical[:, 2], -optical[:, 0], -optical[:, 1]]
        rays = camera_rays @ rotation.T
        slopes = rays @ normal
        if np.any(np.abs(slopes) <= _TOLERANCE):
            _fail('RAY_PARALLEL_TO_SUPPORT')
        if np.any(slopes >= 0.):
            _fail('RAY_DOES_NOT_INTERSECT_FORWARD_SLAB')
        # Rays have camera_link X == 1, so these parameters are optical depths.
        depths = np.r_[-camera_height/slopes, (maximum_height-camera_height)/slopes]
        if not np.all(np.isfinite(depths)) or np.any(depths <= 0.):
            _fail('RAY_DEPTH_INVALID')
        if np.any(depths < limits[0]-_TOLERANCE) or np.any(depths > limits[1]+_TOLERANCE):
            _fail('VERTEX_OUTSIDE_WORKING_DEPTH')
        vertices = origin + np.vstack((rays, rays))*depths[:, None]
        if not np.all(np.isfinite(vertices)):
            _fail('VERTEX_INVALID')
        for name, value in (
                ('bbox_xywh', tuple(float(v) for v in box)), ('image_shape', shape),
                ('camera_matrix', _rows(matrix)), ('T_base_camera', _rows(transform)),
                ('support_plane_base', tuple(float(v) for v in plane)),
                ('foreground_max_m', maximum_height),
                ('working_depth_range_m', tuple(float(v) for v in limits)),
                ('pixel_boundary_xy', _rows(pixels)), ('vertices_base_m', _rows(vertices)),
                ('vertex_optical_depths_m', tuple(float(v) for v in depths)),
                ('bounds_base_m', _rows(np.array([vertices.min(axis=0), vertices.max(axis=0)]))),
                ('camera_support_height_m', camera_height)):
            object.__setattr__(self, name, value)


def build_rgb_observation_volume(*, bbox_xywh, image_shape, camera_matrix,
                                 T_base_camera, support_plane_base,
                                 support_provenance, foreground_max_m,
                                 source_stamp_ns, working_depth_range_m):
    """Construct observation interest from a full RGB pixel box and trusted slab.

    No height or working-range default is supplied: the caller must explicitly
    pass its general scene foreground bound and validated RGB-D depth range.
    Invalid/unsupported geometry raises a ``RGB_OBSERVATION_VOLUME_*`` ValueError.
    This does not select a view, plan, authorize movement, or establish graspability.
    """
    return RGBObservationVolume(bbox_xywh, image_shape, camera_matrix,
        T_base_camera, support_plane_base, support_provenance, foreground_max_m,
        source_stamp_ns, working_depth_range_m)


def project_rgb_observation_volume(volume, *, T_base_camera, camera_matrix,
                                   image_shape, working_depth_range_m):
    """Return projected eight vertices iff the entire convex interest is visible.

    A positive-depth convex polytope lies inside a pinhole image frustum iff all
    its vertices do. This validates only prospective FOV/depth, not visibility
    through occluders, camera/robot reachability or collision freedom. Projection
    failure raises ValueError; success never authorizes motion.
    """
    if type(volume) is not RGBObservationVolume:
        _fail('VOLUME_TYPE_INVALID')
    return project_rgb_observation_vertices(volume.vertices_base_m,
        T_base_camera=T_base_camera, camera_matrix=camera_matrix,
        image_shape=image_shape, working_depth_range_m=working_depth_range_m)


def project_rgb_observation_vertices(vertices_base_m, *, T_base_camera, camera_matrix,
                                     image_shape, working_depth_range_m):
    """Project frozen constructed vertices without rebuilding their source volume.

    A receiver must bind these vertices to its original frozen reference. They
    remain observation geometry, never measured object points or authority.
    The same arithmetic and complete frustum checks serve both volume objects
    and their authenticated wire snapshots.
    """
    transform = _transform(T_base_camera)
    matrix = _intrinsics(camera_matrix)
    shape = _shape(image_shape)
    limits = _depth_range(working_depth_range_m)
    points = _real_array(vertices_base_m, (8, 3), 'VERTEX_INVALID')
    camera = (points-transform[:3, 3]) @ transform[:3, :3]
    optical = np.c_[-camera[:, 1], -camera[:, 2], camera[:, 0]]
    depths = optical[:, 2]
    if np.any(depths <= 0.):
        _fail('PROJECTED_VERTEX_BEHIND_CAMERA')
    if np.any(depths < limits[0]-_TOLERANCE) or np.any(depths > limits[1]+_TOLERANCE):
        _fail('PROJECTED_VERTEX_OUTSIDE_WORKING_DEPTH')
    projected = optical @ matrix.T
    pixels = projected[:, :2]/projected[:, 2, None]
    if (not np.all(np.isfinite(pixels)) or np.any(pixels < -.5-_TOLERANCE)
            or np.any(pixels[:, 0] > shape[1]-.5+_TOLERANCE)
            or np.any(pixels[:, 1] > shape[0]-.5+_TOLERANCE)):
        _fail('PROJECTED_VOLUME_OUTSIDE_IMAGE')
    return _rows(pixels)
