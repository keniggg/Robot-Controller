#!/usr/bin/env python3
"""Capture and benchmark a real tabletop-geometry planning fixture."""

import argparse
import functools
import hashlib
import json
import math
import pathlib
import re
import tempfile
import sys
import time

import numpy as np
import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE_SRC = PACKAGE_ROOT / 'src'
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import (  # noqa: E402
    TabletopGeometryConfig,
    generate_tabletop_proposals,
)


FIXTURE_KEYS = frozenset((
    'schema_version',
    'source',
    'depth_stamp_ns',
    'depth_frame',
    'intrinsics',
    'T_base_depth',
    'object_points_base',
    'obb_center_base',
    'obb_size_xyz_m',
    'support_point_base',
    'support_normal_base',
    'R_base_obb',
    'audit_sha256',
))
SYNC_SLOP_NS = 80_000_000


def validate_intrinsics(camera):
    if not isinstance(camera, dict):
        raise ValueError('/camera must be a mapping')
    values = {}
    for name in ('fx', 'fy', 'cx', 'cy', 'depth_scale'):
        value = camera.get(name)
        if type(value) not in (int, float) or not math.isfinite(float(value)):
            raise ValueError('/camera/%s must be finite numeric' % name)
        values[name] = float(value)
    for name in ('fx', 'fy', 'depth_scale'):
        if values[name] <= 0.0:
            raise ValueError('/camera/%s must be positive' % name)
    return values


def _finite_matrix(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError('%s must be a finite %s array' % (name, shape))
    return array


def back_project_masked_depth(depth, mask, encoding, intrinsics, T_base_depth):
    values = validate_intrinsics(intrinsics)
    depth_array = np.asarray(depth)
    mask_array = np.asarray(mask)
    if depth_array.ndim != 2 or mask_array.shape != depth_array.shape:
        raise ValueError('depth and mask must be same-sized 2D arrays')
    normalized_encoding = str(encoding).upper()
    if normalized_encoding == '16UC1':
        metres = depth_array.astype(float) * values['depth_scale']
    elif normalized_encoding == '32FC1':
        metres = depth_array.astype(float)
    else:
        raise ValueError('depth encoding must be 16UC1 or 32FC1')
    valid = (mask_array != 0) & np.isfinite(metres) & (metres > 0.0)
    rows, columns = np.nonzero(valid)
    z = metres[rows, columns]
    optical = np.column_stack((
        (columns.astype(float) - values['cx']) * z / values['fx'],
        (rows.astype(float) - values['cy']) * z / values['fy'],
        z,
    ))
    transform = _finite_matrix(T_base_depth, (4, 4), 'T_base_depth')
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError('T_base_depth must be homogeneous')
    return optical.dot(transform[:3, :3].T) + transform[:3, 3]


def clean_target_points(
    points,
    obb_center_base,
    R_base_obb,
    obb_size_xyz_m,
    support_normal_base,
    support_offset_m,
    min_height_m,
):
    cloud = np.asarray(points, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1:] != (3,):
        raise ValueError('object points must be an Nx3 array')
    center = _finite_matrix(obb_center_base, (3,), 'obb_center_base')
    rotation = _finite_matrix(R_base_obb, (3, 3), 'R_base_obb')
    size = _finite_matrix(obb_size_xyz_m, (3,), 'obb_size_xyz_m')
    normal = _finite_matrix(support_normal_base, (3,), 'support_normal_base')
    if np.any(size <= 0.0):
        raise ValueError('obb_size_xyz_m must be positive')
    if not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-6) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1e-6
    ):
        raise ValueError('R_base_obb must be orthonormal')
    normal_norm = float(np.linalg.norm(normal))
    if not math.isclose(normal_norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError('support_normal_base must be unit length')
    offset = float(support_offset_m)
    minimum = float(min_height_m)
    if not math.isfinite(offset) or not math.isfinite(minimum) or minimum < 0.0:
        raise ValueError('support plane values must be finite and non-negative')

    finite = cloud[np.all(np.isfinite(cloud), axis=1)]
    heights = finite.dot(normal) + offset
    supported = finite[heights >= minimum]
    local = (supported - center).dot(rotation)
    inside = np.all(np.abs(local) <= size / 2.0 + 0.005 + 1e-12, axis=1)
    cropped = supported[inside]

    voxels = np.floor(cropped / 0.002).astype(np.int64)
    order = np.lexsort((voxels[:, 2], voxels[:, 1], voxels[:, 0]))
    ordered_points = cropped[order]
    ordered_voxels = voxels[order]
    if len(ordered_voxels):
        first = np.ones(len(ordered_voxels), dtype=bool)
        first[1:] = np.any(ordered_voxels[1:] != ordered_voxels[:-1], axis=1)
        retained = ordered_points[first]
    else:
        retained = np.empty((0, 3), dtype=float)
    if len(retained) > 512:
        indices = np.linspace(0, len(retained) - 1, 512).astype(np.int64)
        retained = retained[indices]
    if len(retained) < 120:
        raise ValueError('capture must retain at least 120 points')
    return retained


def _reject_json_constant(value):
    raise ValueError('non-finite JSON constant %s' % value)


def validate_planning_audit_bytes(audit_bytes, depth_stamp_ns):
    try:
        raw = bytes(audit_bytes)
        report = json.loads(
            raw.decode('utf-8'), parse_constant=_reject_json_constant
        )
    except (TypeError, ValueError, UnicodeDecodeError) as error:
        raise ValueError('planning audit must be strict JSON: %s' % error) from error
    if not isinstance(report, dict) or report.get('report_version') != 3:
        raise ValueError('planning audit must use report_version 3')
    transform = report.get('snapshot_transform')
    if not isinstance(transform, dict) or transform.get('snapshot_stamp_ns') != int(
        depth_stamp_ns
    ):
        raise ValueError('planning audit snapshot stamp does not match depth')
    rows = report.get('rows')
    if not isinstance(rows, list) or not any(
        isinstance(row, dict)
        and row.get('candidate_source') == 'tabletop_geometry'
        and isinstance(row.get('analytical_result'), dict)
        and row['analytical_result'].get('ok') is True
        for row in rows
    ):
        raise ValueError('planning audit has no successful tabletop_geometry row')
    return hashlib.sha256(raw).hexdigest()


def validate_gate_audit_path(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('gate audit output path must be non-empty')
    return pathlib.Path(value.strip()).expanduser()


def support_point_from_plane(support_normal_base, support_offset_m):
    normal = _finite_matrix(
        support_normal_base, (3,), 'support_normal_base'
    )
    if not math.isclose(
        float(np.linalg.norm(normal)), 1.0, rel_tol=0.0, abs_tol=1e-6
    ):
        raise ValueError('support_normal_base must be unit length')
    offset = float(support_offset_m)
    if not math.isfinite(offset):
        raise ValueError('support_offset_m must be finite')
    return -offset * normal


def require_stamp_match(depth_stamp_ns, other_stamp_ns):
    if abs(int(depth_stamp_ns) - int(other_stamp_ns)) > SYNC_SLOP_NS:
        raise ValueError('message stamp exceeds synchronizer tolerance')


def quaternion_xyzw_to_rotation(quaternion):
    values = _finite_matrix(quaternion, (4,), 'quaternion_xyzw')
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError('quaternion_xyzw must be non-zero')
    x, y, z, w = values / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def validate_fixture(fixture):
    if not isinstance(fixture, dict) or set(fixture) != FIXTURE_KEYS:
        raise ValueError('fixture must contain exact top-level keys')
    if type(fixture['schema_version']) is not int or fixture['schema_version'] != 1:
        raise ValueError('schema_version must be integer 1')
    if fixture['source'] != 'realsense':
        raise ValueError('source must be realsense')
    stamp = fixture['depth_stamp_ns']
    if type(stamp) is not int or stamp <= 0:
        raise ValueError('depth_stamp_ns must be a positive integer')
    if not isinstance(fixture['depth_frame'], str) or not fixture['depth_frame'].strip():
        raise ValueError('depth_frame must be non-empty')
    intrinsics = fixture['intrinsics']
    if not isinstance(intrinsics, dict) or set(intrinsics) != {
        'fx', 'fy', 'cx', 'cy', 'depth_scale'
    }:
        raise ValueError('intrinsics must contain exact camera keys')
    validate_intrinsics(intrinsics)
    transform = _finite_matrix(fixture['T_base_depth'], (4, 4), 'T_base_depth')
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError('T_base_depth must be homogeneous')
    points = np.asarray(fixture['object_points_base'], dtype=float)
    if (
        points.ndim != 2
        or points.shape[1:] != (3,)
        or not 120 <= len(points) <= 512
        or not np.all(np.isfinite(points))
    ):
        raise ValueError('object_points_base must have 120 to 512 finite XYZ rows')
    for name in (
        'obb_center_base', 'obb_size_xyz_m', 'support_point_base',
        'support_normal_base',
    ):
        _finite_matrix(fixture[name], (3,), name)
    if np.any(np.asarray(fixture['obb_size_xyz_m'], dtype=float) <= 0.0):
        raise ValueError('obb_size_xyz_m must be positive')
    normal = np.asarray(fixture['support_normal_base'], dtype=float)
    if not math.isclose(
        float(np.linalg.norm(normal)), 1.0, rel_tol=0.0, abs_tol=1e-6
    ):
        raise ValueError('support_normal_base must be unit length')
    rotation = _finite_matrix(fixture['R_base_obb'], (3, 3), 'R_base_obb')
    if (
        not np.allclose(rotation.T.dot(rotation), np.eye(3), atol=1e-6)
        or not math.isclose(
            float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1e-6
        )
    ):
        raise ValueError('R_base_obb must be finite orthonormal')
    digest = fixture['audit_sha256']
    if not isinstance(digest, str) or re.fullmatch(r'[0-9a-f]{64}', digest) is None:
        raise ValueError('audit_sha256 must be 64 lowercase hex characters')
    return fixture


def _planning_audit_stamp(audit_bytes):
    try:
        report = json.loads(
            bytes(audit_bytes).decode('utf-8'),
            parse_constant=_reject_json_constant,
        )
    except (TypeError, ValueError, UnicodeDecodeError):
        return None
    if not isinstance(report, dict) or report.get('report_version') != 3:
        return None
    transform = report.get('snapshot_transform')
    if not isinstance(transform, dict):
        return None
    stamp = transform.get('snapshot_stamp_ns')
    return stamp if type(stamp) is int and stamp > 0 else None


def _vector3(message):
    return np.array([message.x, message.y, message.z], dtype=float)


def _stamp_ns(stamp):
    value = int(stamp.to_nsec())
    if value <= 0:
        raise ValueError('message stamp must be positive')
    return value


def _transform_matrix(transform):
    translation = transform.transform.translation
    quaternion = transform.transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_xyzw_to_rotation([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w,
    ])
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class _RosCaptureRuntime:
    """Lazy ROS adapter; importing or benchmarking this module stays offline."""

    def __init__(self):
        import message_filters
        import rospy
        import tf2_ros
        from alicia_flexible_grasp_supervisor.msg import ObjectGeometry
        from alicia_flexible_grasp_supervisor.srv import TriggerZero
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self.message_filters = message_filters
        self.rospy = rospy
        self.Image = Image
        self.ObjectGeometry = ObjectGeometry
        self.bridge = CvBridge()
        if not rospy.core.is_initialized():
            rospy.init_node(
                'capture_tabletop_realsense_fixture',
                anonymous=True,
                disable_signals=True,
            )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.wait_for_service('/grasp_6d/request_plan', timeout=10.0)
        self.preview_service = rospy.ServiceProxy(
            '/grasp_6d/request_plan', TriggerZero
        )

    def set_preview(self, trigger):
        response = self.preview_service(trigger=bool(trigger))
        if not bool(response.success):
            raise RuntimeError(
                '/grasp_6d/request_plan rejected trigger=%s: %s'
                % (str(bool(trigger)).lower(), response.message)
            )

    def capture(self, timeout_sec):
        timeout = float(timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('capture timeout must be positive and finite')
        camera = validate_intrinsics(self.rospy.get_param('/camera', {}))
        audit_path = validate_gate_audit_path(self.rospy.get_param(
            '/grasp_6d/remote/gate_audit_output_path', ''
        ))
        min_height = self.rospy.get_param(
            '/grasp_6d/remote/target_cloud_support_plane_min_height_m',
            None,
        )
        if type(min_height) not in (int, float) or not math.isfinite(
            float(min_height)
        ) or float(min_height) < 0.0:
            raise ValueError('target cloud support-plane minimum height is invalid')

        captured = {}

        def synchronized(depth_message, mask_message, geometry_message):
            try:
                depth_stamp = _stamp_ns(depth_message.header.stamp)
                require_stamp_match(
                    depth_stamp, _stamp_ns(mask_message.header.stamp)
                )
                require_stamp_match(
                    depth_stamp, _stamp_ns(geometry_message.header.stamp)
                )
                captured[depth_stamp] = (
                    depth_message, mask_message, geometry_message
                )
                while len(captured) > 64:
                    captured.pop(next(iter(captured)))
            except (TypeError, ValueError):
                return

        subscribers = [
            self.message_filters.Subscriber(
                '/supervisor/camera/depth/image_raw', self.Image
            ),
            self.message_filters.Subscriber(
                '/perception/object_mask', self.Image
            ),
            self.message_filters.Subscriber(
                '/grasp_6d/object_geometry', self.ObjectGeometry
            ),
        ]
        synchronizer = self.message_filters.ApproximateTimeSynchronizer(
            subscribers, queue_size=10, slop=0.08
        )
        synchronizer.registerCallback(synchronized)
        deadline = time.monotonic() + timeout
        try:
            while not self.rospy.is_shutdown() and time.monotonic() < deadline:
                try:
                    audit_bytes = audit_path.read_bytes()
                except OSError:
                    audit_bytes = b''
                audit_stamp = _planning_audit_stamp(audit_bytes)
                messages = captured.get(audit_stamp)
                if messages is not None:
                    return self._fixture_from_messages(
                        messages,
                        camera,
                        float(min_height),
                        audit_bytes,
                    )
                time.sleep(0.02)
        finally:
            for subscriber in subscribers:
                subscriber.sub.unregister()
        raise RuntimeError(
            'timed out waiting for synchronized depth/mask/geometry and matching audit'
        )

    def _fixture_from_messages(
        self, messages, intrinsics, min_height_m, audit_bytes
    ):
        depth_message, mask_message, geometry = messages
        if not bool(geometry.valid):
            raise ValueError('ObjectGeometry must be valid')
        depth_stamp_ns = _stamp_ns(depth_message.header.stamp)
        require_stamp_match(
            depth_stamp_ns, _stamp_ns(geometry.header.stamp)
        )
        frame = str(depth_message.header.frame_id).strip()
        if not frame:
            raise ValueError('depth frame must be non-empty')
        transform_message = self.tf_buffer.lookup_transform(
            'base_link',
            frame,
            depth_message.header.stamp,
            self.rospy.Duration(2.0),
        )
        T_base_depth = _transform_matrix(transform_message)
        depth = self.bridge.imgmsg_to_cv2(
            depth_message, desired_encoding='passthrough'
        )
        mask = self.bridge.imgmsg_to_cv2(
            mask_message, desired_encoding='passthrough'
        )
        pose = geometry.pose_base
        center = np.array([
            pose.position.x, pose.position.y, pose.position.z,
        ], dtype=float)
        R_base_obb = quaternion_xyzw_to_rotation([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        size = _vector3(geometry.size_xyz_m)
        normal = _vector3(geometry.support_normal_base)
        support_offset = float(geometry.support_offset_m)
        projected = back_project_masked_depth(
            depth,
            mask,
            depth_message.encoding,
            intrinsics,
            T_base_depth,
        )
        cleaned = clean_target_points(
            projected,
            obb_center_base=center,
            R_base_obb=R_base_obb,
            obb_size_xyz_m=size,
            support_normal_base=normal,
            support_offset_m=support_offset,
            min_height_m=min_height_m,
        )
        digest = validate_planning_audit_bytes(
            audit_bytes, depth_stamp_ns
        )
        return validate_fixture({
            'schema_version': 1,
            'source': 'realsense',
            'depth_stamp_ns': depth_stamp_ns,
            'depth_frame': frame,
            'intrinsics': dict(intrinsics),
            'T_base_depth': T_base_depth.tolist(),
            'object_points_base': cleaned.tolist(),
            'obb_center_base': center.tolist(),
            'obb_size_xyz_m': size.tolist(),
            'support_point_base': support_point_from_plane(
                normal, support_offset
            ).tolist(),
            'support_normal_base': normal.tolist(),
            'R_base_obb': R_base_obb.tolist(),
            'audit_sha256': digest,
        })


def _write_fixture(path, fixture):
    target = pathlib.Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(
        validate_fixture(fixture),
        allow_nan=False,
        indent=2,
        sort_keys=True,
    ) + '\n'
    with tempfile.NamedTemporaryFile(
        mode='w',
        encoding='utf-8',
        dir=str(target.parent),
        prefix=target.name + '.',
        suffix='.tmp',
        delete=False,
    ) as temporary:
        temporary.write(text)
        temporary.flush()
        temporary_path = pathlib.Path(temporary.name)
    temporary_path.replace(target)


def capture_tabletop_fixture(path, timeout_sec=30.0, runtime=None):
    active_runtime = _RosCaptureRuntime() if runtime is None else runtime
    try:
        active_runtime.set_preview(True)
        fixture = active_runtime.capture(float(timeout_sec))
        _write_fixture(path, fixture)
        return fixture
    finally:
        active_runtime.set_preview(False)


@functools.lru_cache(maxsize=1)
def load_production_tabletop_config():
    config_path = PACKAGE_ROOT / 'config' / 'grasp_params.yaml'
    payload = yaml.safe_load(config_path.read_text())
    remote = payload['grasp_6d']['remote']
    configured = remote['tabletop_geometry_candidates']
    gripper = remote['gripper_geometry']
    return TabletopGeometryConfig(
        max_inner_gap_m=float(gripper['max_inner_gap_m']),
        angle_step_deg=float(configured['angle_step_deg']),
        angle_dedup_deg=float(configured['angle_dedup_deg']),
        jaw_clearance_each_side_m=float(
            configured['jaw_clearance_each_side_m']
        ),
        min_contact_band_points=int(configured['min_contact_band_points']),
        contact_band_fraction=float(configured['contact_band_fraction']),
        max_candidates=int(configured['max_candidates']),
        approach_tilt_degrees=tuple(
            float(value)
            for value in configured.get('approach_tilt_degrees', ())
        ),
    )


def load_fixture(path):
    try:
        fixture = json.loads(
            pathlib.Path(path).read_text(), parse_constant=_reject_json_constant
        )
    except (OSError, TypeError, ValueError) as error:
        raise ValueError('could not load strict fixture JSON: %s' % error) from error
    return validate_fixture(fixture)


def generate_tabletop_proposals_from_fixture(fixture):
    validate_fixture(fixture)
    return generate_tabletop_proposals(
        object_points_base=np.asarray(fixture['object_points_base'], dtype=float),
        obb_center_base=np.asarray(fixture['obb_center_base'], dtype=float),
        R_base_obb=np.asarray(fixture['R_base_obb'], dtype=float),
        obb_size_xyz_m=np.asarray(fixture['obb_size_xyz_m'], dtype=float),
        support_point_base=np.asarray(fixture['support_point_base'], dtype=float),
        support_normal_base=np.asarray(fixture['support_normal_base'], dtype=float),
        config=load_production_tabletop_config(),
    )


def benchmark_generator(
    fixture,
    iterations=100,
    warmups=10,
    generate=generate_tabletop_proposals_from_fixture,
    clock=time.perf_counter,
):
    if type(iterations) is not int or iterations <= 0:
        raise ValueError('iterations must be a positive integer')
    if type(warmups) is not int or warmups < 0:
        raise ValueError('warmups must be a non-negative integer')
    for _ in range(warmups):
        generate(fixture)
    elapsed_ms = []
    for _ in range(iterations):
        started = clock()
        generate(fixture)
        elapsed_ms.append((clock() - started) * 1000.0)
    return (
        float(np.median(elapsed_ms)),
        float(np.percentile(elapsed_ms, 95)),
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Capture or benchmark RealSense tabletop geometry.'
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--fixture', type=pathlib.Path)
    mode.add_argument('--dump-tabletop-fixture', type=pathlib.Path)
    parser.add_argument('--iterations', type=int, default=100)
    parser.add_argument('--max-median-ms', type=float, default=30.0)
    parser.add_argument('--capture-timeout-sec', type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.dump_tabletop_fixture is not None:
            fixture = capture_tabletop_fixture(
                args.dump_tabletop_fixture,
                timeout_sec=args.capture_timeout_sec,
            )
            print('captured tabletop fixture points=%d' % len(
                fixture['object_points_base']
            ))
            return 0
        fixture = load_fixture(args.fixture)
        median_ms, p95_ms = benchmark_generator(
            fixture, iterations=args.iterations, warmups=10
        )
        print('tabletop geometry median_ms=%.3f p95_ms=%.3f iterations=%d' % (
            median_ms, p95_ms, args.iterations
        ))
        return 1 if median_ms > args.max_median_ms else 0
    except (OSError, RuntimeError, ValueError) as error:
        print('error: %s' % error, file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
