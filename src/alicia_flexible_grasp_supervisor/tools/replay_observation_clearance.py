#!/usr/bin/env python3
"""Replay gripper-mesh clearance against a frozen grasp audit offline.

The tool deliberately has no ROS imports and opens no device.  It evaluates
the checked-in URDF collision meshes at caller-supplied joint coordinates
against the support plane stored in one gate-audit report.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text, length, name):
    values = np.asarray([float(value) for value in str(text).split()], dtype=float)
    if values.shape != (length,) or not np.all(np.isfinite(values)):
        raise ValueError('%s must contain %d finite values' % (name, length))
    return values


def _rotation_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('joint axis must be finite and non-zero')
    x, y, z = axis / norm
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    one_minus_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=float,
    )


def _rpy_rotation(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    rx = _rotation_axis_angle([1.0, 0.0, 0.0], roll)
    ry = _rotation_axis_angle([0.0, 1.0, 0.0], pitch)
    rz = _rotation_axis_angle([0.0, 0.0, 1.0], yaw)
    return rz @ ry @ rx


def _transform(rotation=None, translation=None):
    output = np.eye(4, dtype=float)
    if rotation is not None:
        output[:3, :3] = np.asarray(rotation, dtype=float)
    if translation is not None:
        output[:3, 3] = np.asarray(translation, dtype=float)
    return output


class OfflineUrdfKinematics:
    def __init__(self, urdf_path):
        self.path = Path(urdf_path)
        root = ET.parse(str(self.path)).getroot()
        self.links = {}
        for link in root.findall('link'):
            meshes = []
            for collision in link.findall('collision'):
                origin = collision.find('origin')
                xyz = _vector(
                    '0 0 0' if origin is None else origin.get('xyz', '0 0 0'),
                    3,
                    'collision xyz',
                )
                rpy = _vector(
                    '0 0 0' if origin is None else origin.get('rpy', '0 0 0'),
                    3,
                    'collision rpy',
                )
                mesh = collision.find('geometry/mesh')
                if mesh is not None:
                    meshes.append(
                        {
                            'filename': str(mesh.get('filename', '')),
                            'T_link_mesh': _transform(_rpy_rotation(rpy), xyz),
                        }
                    )
            self.links[str(link.get('name'))] = meshes
        self.parent_joint = {}
        for joint in root.findall('joint'):
            child = str(joint.find('child').get('link'))
            origin = joint.find('origin')
            axis = joint.find('axis')
            mimic = joint.find('mimic')
            self.parent_joint[child] = {
                'name': str(joint.get('name')),
                'type': str(joint.get('type')),
                'parent': str(joint.find('parent').get('link')),
                'xyz': _vector(
                    '0 0 0' if origin is None else origin.get('xyz', '0 0 0'),
                    3,
                    'joint xyz',
                ),
                'rpy': _vector(
                    '0 0 0' if origin is None else origin.get('rpy', '0 0 0'),
                    3,
                    'joint rpy',
                ),
                'axis': _vector(
                    '1 0 0' if axis is None else axis.get('xyz', '1 0 0'),
                    3,
                    'joint axis',
                ),
                'mimic': None if mimic is None else {
                    'joint': str(mimic.get('joint')),
                    'multiplier': float(mimic.get('multiplier', '1')),
                    'offset': float(mimic.get('offset', '0')),
                },
            }

    def _joint_position(self, joint, positions):
        mimic = joint['mimic']
        if mimic is None:
            return float(positions.get(joint['name'], 0.0))
        return (
            mimic['multiplier'] * float(positions.get(mimic['joint'], 0.0))
            + mimic['offset']
        )

    def link_transform(self, link_name, positions):
        chain = []
        child = str(link_name)
        while child in self.parent_joint:
            joint = self.parent_joint[child]
            chain.append(joint)
            child = joint['parent']
        transform = np.eye(4, dtype=float)
        for joint in reversed(chain):
            transform = transform @ _transform(
                _rpy_rotation(joint['rpy']),
                joint['xyz'],
            )
            position = self._joint_position(joint, positions)
            if joint['type'] in ('revolute', 'continuous'):
                transform = transform @ _transform(
                    _rotation_axis_angle(joint['axis'], position)
                )
            elif joint['type'] == 'prismatic':
                transform = transform @ _transform(
                    translation=joint['axis'] * position
                )
            elif joint['type'] != 'fixed':
                raise ValueError('unsupported joint type %s' % joint['type'])
        return transform


def _load_stl_vertices(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError('STL is too short: %s' % path)
    triangle_count = struct.unpack_from('<I', data, 80)[0]
    expected_size = 84 + 50 * triangle_count
    if expected_size != len(data):
        raise ValueError('only strict binary STL is supported: %s' % path)
    record_dtype = np.dtype(
        [
            ('normal', '<f4', (3,)),
            ('vertices', '<f4', (3, 3)),
            ('attribute', '<u2'),
        ]
    )
    records = np.frombuffer(
        data,
        dtype=record_dtype,
        count=triangle_count,
        offset=84,
    )
    return np.asarray(records['vertices'], dtype=float).reshape(-1, 3)


def _resolve_mesh(mesh_directory, filename):
    prefix = 'package://alicia_d_descriptions/meshes/'
    if not filename.startswith(prefix):
        raise ValueError('unexpected mesh URI: %s' % filename)
    name = filename[len(prefix):]
    path = Path(mesh_directory) / name
    if not path.is_file():
        raise ValueError('mesh does not exist: %s' % path)
    return path


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _quaternion_xyzw(rotation):
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.asarray(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ],
            dtype=float,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        following = (index + 1) % 3
        remaining = (index + 2) % 3
        scale = 2.0 * math.sqrt(
            max(0.0, 1.0 + matrix[index, index] - matrix[following, following] - matrix[remaining, remaining])
        )
        quaternion = np.zeros(4, dtype=float)
        quaternion[index] = 0.25 * scale
        quaternion[following] = (matrix[following, index] + matrix[index, following]) / scale
        quaternion[remaining] = (matrix[remaining, index] + matrix[index, remaining]) / scale
        quaternion[3] = (matrix[remaining, following] - matrix[following, remaining]) / scale
    quaternion /= float(np.linalg.norm(quaternion))
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return quaternion


def replay(args):
    audit_path = Path(args.audit)
    audit = json.loads(audit_path.read_text(encoding='utf-8'))
    snapshot = np.asarray(
        audit['snapshot_transform']['T_base_camera_link'],
        dtype=float,
    )
    normal_camera = np.asarray(audit['support_plane_normal_camera'], dtype=float)
    point_camera = np.asarray(audit['support_plane_point_camera'], dtype=float)
    normal_base = snapshot[:3, :3] @ normal_camera
    normal_base /= float(np.linalg.norm(normal_base))
    point_base = snapshot[:3, :3] @ point_camera + snapshot[:3, 3]
    offset_base = -float(normal_base @ point_base)

    urdf = OfflineUrdfKinematics(args.urdf)
    arm = np.radians(np.asarray(args.arm_joints_deg, dtype=float))
    positions = {'Joint%d' % (index + 1): float(value) for index, value in enumerate(arm)}
    positions['right_finger'] = float(args.right_finger_m)
    tool0 = urdf.link_transform('tool0', positions)

    mesh_results = []
    all_clearances = []
    for link_name in args.links:
        link_transform = urdf.link_transform(link_name, positions)
        for mesh in urdf.links.get(link_name, []):
            mesh_path = _resolve_mesh(args.mesh_directory, mesh['filename'])
            vertices = _load_stl_vertices(mesh_path)
            transform = link_transform @ mesh['T_link_mesh']
            vertices_base = (
                vertices @ transform[:3, :3].T + transform[:3, 3]
            )
            clearances = vertices_base @ normal_base + offset_base
            minimum_index = int(np.argmin(clearances))
            all_clearances.append(clearances)
            mesh_results.append(
                {
                    'link': link_name,
                    'mesh': mesh_path.name,
                    'mesh_sha256': _sha256(mesh_path),
                    'minimum_clearance_m': float(clearances[minimum_index]),
                    'maximum_clearance_m': float(np.max(clearances)),
                    'minimum_vertex_base_m': vertices_base[minimum_index].tolist(),
                    'vertex_count_with_duplicates': int(len(vertices)),
                }
            )
    if not all_clearances:
        raise ValueError('no collision mesh vertices were evaluated')
    combined = np.concatenate(all_clearances)
    return {
        'audit': {
            'path': str(audit_path),
            'file_sha256': _sha256(audit_path),
            'plan_id': str(audit.get('plan_id', '')),
            'request_id': int(audit.get('request_id', -1)),
            'snapshot_stamp_sec': float(audit.get('snapshot_stamp_sec', 0.0)),
        },
        'urdf': {
            'path': str(Path(args.urdf)),
            'file_sha256': _sha256(args.urdf),
        },
        'joint_positions': {
            'arm_degrees': [float(value) for value in args.arm_joints_deg],
            'right_finger_m': float(args.right_finger_m),
        },
        'support_plane_base': {
            'normal': normal_base.tolist(),
            'point_m': point_base.tolist(),
            'offset_m': offset_base,
        },
        'tool0_base': {
            'position_m': tool0[:3, 3].tolist(),
            'quaternion_xyzw': _quaternion_xyzw(tool0[:3, :3]).tolist(),
        },
        'meshes': mesh_results,
        'minimum_mesh_clearance_m': float(np.min(combined)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--audit', required=True)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--mesh-directory', required=True)
    parser.add_argument(
        '--arm-joints-deg',
        required=True,
        nargs=6,
        type=float,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
    )
    parser.add_argument('--right-finger-m', type=float, default=0.0)
    parser.add_argument(
        '--links',
        nargs='+',
        default=['Link6', 'Grasp_base', 'Link7', 'Link8'],
    )
    args = parser.parse_args()
    print(json.dumps(replay(args), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
