#!/usr/bin/env python3
"""Compare frozen and pinned URDFs without live ROS or execution authority.

Same named q, same base_link, and a ZERO-POSE-derived terminal frame change:
no optimization of joint signs, offsets, TF, or calibration against failed data.
Numerical samples are not a global equivalence proof or physical calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk


def mjcf_arm_fk(xml):
    """Restricted audit of the checked-in MJCF's WORLD-frame arm kinematics.

    The server uses world coordinates and the midpoint of Link7/Link8 origins,
    not the separate local v5.6 URDF's tool0. No dynamics/mesh certificate.
    Reject unsupported defaults, frames, moving bases or joint references.
    """
    from tf.transformations import quaternion_matrix
    root = ET.fromstring(xml)
    if (root.tag != 'mujoco' or root.find('default') is not None
            or root.find('.//include') is not None or root.find('.//frame') is not None):
        raise ValueError('only explicit MJCF without defaults is supported')
    compiler = root.find('compiler')
    if compiler is None or compiler.get('angle') != 'radian' or compiler.get('coordinate', 'local') != 'local':
        raise ValueError('explicit radian/local MJCF required')
    base = root.find("worldbody/body[@name='base_link']")
    if base is None or base.find('joint') is not None or base.find('freejoint') is not None:
        raise ValueError('fixed named MJCF base required')
    fk = object.__new__(SerialUrdfFk)
    fk.model_sha256 = hashlib.sha256(xml.encode()).hexdigest()
    fk.names, fk.chain, limits = ARM_NAMES, [], []

    def body_transform(body):
        if (body.get('mocap', 'false') != 'false' or body.find('freejoint') is not None
                or any(key in body.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis', 'childclass'))):
            raise ValueError('unsupported MJCF body convention')
        quat = np.asarray([float(v) for v in body.get('quat', '1 0 0 0').split()])
        xyz = np.asarray([float(v) for v in body.get('pos', '0 0 0').split()])
        if quat.shape != (4,) or xyz.shape != (3,) or not np.isfinite(quat).all() or not np.isfinite(xyz).all() or np.linalg.norm(quat) < 1e-12:
            raise ValueError('invalid MJCF body transform')
        transform = quaternion_matrix(quat[[1, 2, 3, 0]])
        transform[:3, 3] = xyz
        return transform

    fk.chain.append((body_transform(base), None, None))
    body = base
    for index, name in enumerate(ARM_NAMES):
        children = body.findall('body')
        if len(children) != 1:
            raise ValueError('explicit serial MJCF arm required')
        body = children[0]
        joints = body.findall('joint')
        if len(joints) != 1 or joints[0].get('name') != name:
            raise ValueError('named MJCF arm order mismatch')
        joint = joints[0]
        if (joint.get('type', 'hinge') != 'hinge' or joint.get('ref', '0') != '0'
                or any(key in joint.attrib for key in ('class', 'springref'))
                or any(float(v) != 0 for v in joint.get('pos', '0 0 0').split())):
            raise ValueError('unsupported MJCF joint offset/reference')
        axis = np.asarray([float(v) for v in joint.get('axis', '0 0 1').split()])
        bounds = [float(v) for v in joint.get('range', '').split()]
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12 or len(bounds) != 2:
            raise ValueError('invalid MJCF joint axis/range')
        limits.append(bounds)
        fk.chain.append((body_transform(body), index, axis/np.linalg.norm(axis)))
    fingers = [body.find("body[@name='%s']" % name) for name in ('Link7', 'Link8')]
    if any(f is None for f in fingers):
        raise ValueError('server finger-center bodies missing')
    for finger in fingers:
        joints = finger.findall('joint')
        if (len(joints) != 1 or joints[0].get('type') != 'slide'
                or joints[0].get('ref', '0') != '0'):
            raise ValueError('explicit zero-reference slider fingers required')
    # Audit zero finger positions only (the server's nominal fully open jaw).
    # Rounded body quaternions can make the center weakly opening-dependent.
    center = np.mean([body_transform(f)[:3, 3] for f in fingers], axis=0)
    suffix = np.eye(4)
    suffix[:3, 3] = center
    fk.chain.append((suffix, None, None))
    fk.limits = np.asarray(limits)
    if not np.isfinite(fk.limits).all() or np.any(fk.limits[:, 0] >= fk.limits[:, 1]):
        raise ValueError('invalid MJCF limits')
    return fk


def rotation_angle_deg(rotation):
    # atan2 is stable near identity and pi, unlike trace-only acos.
    skew = np.array([rotation[2, 1]-rotation[1, 2],
                     rotation[0, 2]-rotation[2, 0],
                     rotation[1, 0]-rotation[0, 1]]) / 2
    return float(np.degrees(np.arctan2(np.linalg.norm(skew),
                                     (np.trace(rotation)-1)/2)))


def pose_error(first, second):
    return {'translation_mm': float(np.linalg.norm(first[:3, 3]-second[:3, 3])*1000),
            'rotation_deg': rotation_angle_deg(first[:3, :3].T @ second[:3, :3])}


def zero_axes(fk):
    transform = np.eye(4)
    result = {}
    for origin, index, axis in fk.chain:
        transform = transform @ origin
        if index is not None:
            direction = transform[:3, :3] @ axis
            point = transform[:3, 3]
            result[fk.names[index]] = {
                'direction_base': direction.tolist(), 'origin_base_m': point.tolist(),
                'line_moment_base_m': np.cross(point, direction).tolist()}
    return result


def definitions(xml):
    result = {}
    for joint in ET.fromstring(xml).findall('joint'):
        if joint.get('name') in ARM_NAMES or joint.get('type') == 'fixed':
            result[joint.get('name')] = {'type': joint.get('type'), **{
                child.tag: dict(child.attrib) for child in joint
                if child.tag in ('origin', 'axis', 'parent', 'child', 'limit')}}
    return result


def compare(reference, candidate, samples):
    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 6 or not len(samples):
        raise ValueError('nonempty N by 6 samples required')
    if not np.all(np.isfinite(samples)):
        raise ValueError('nonfinite model samples')
    # Defined from model home transforms only, NOT fitted to measured errors.
    frame_change = np.linalg.inv(candidate(np.zeros(6))) @ reference(np.zeros(6))
    rotation_only = frame_change.copy()
    rotation_only[:3, 3] = 0
    rows = []
    for q in samples:
        old, new = reference(q), candidate(q)
        rows.append({'raw': pose_error(old, new),
                     'terminal_rotation_only': pose_error(old, new @ rotation_only),
                     'zero_anchored_frame': pose_error(old, new @ frame_change)})
    statistics = {}
    for kind in rows[0]:
        statistics[kind] = {}
        for metric in rows[0][kind]:
            values = [row[kind][metric] for row in rows]
            worst = int(np.argmax(values))
            statistics[kind][metric] = {
                'maximum': max(values), 'median': float(np.median(values)),
                'worst_sample_q_rad': samples[worst].tolist()}
    old_axes, new_axes = zero_axes(reference), zero_axes(candidate)
    axes = {}
    for name in ARM_NAMES:
        old, new = old_axes[name], new_axes[name]
        u, v = np.asarray(old['direction_base']), np.asarray(new['direction_base'])
        axes[name] = {
            'signed_direction_dot': float(u @ v),
            'direction_difference_deg': float(np.degrees(np.arctan2(np.linalg.norm(np.cross(u, v)), u @ v))),
            'line_moment_difference_mm': float(np.linalg.norm(
                np.asarray(old['line_moment_base_m'])-new['line_moment_base_m'])*1000)}
    return {'sample_count': len(rows), 'joint_mapping': 'same names, same radians; no sign/offset fit',
            'zero_derived_candidate_tool_to_reference_tool': frame_change.tolist(),
            'zero_axes': axes, 'statistics': statistics,
            'physical_calibration_or_global_equivalence_proven': False}


def read_bag_samples(path):
    import rosbag  # Deserializes closed historical files; never init_node.
    if str(path).endswith('.active'):
        raise ValueError('closed bag required')
    rows = set()
    counts = {}
    with rosbag.Bag(str(path), 'r') as bag:
        for topic, msg, _stamp in bag.read_messages(topics=[
                '/alicia_d/sdk_command', '/alicia_d/accepted_joint_states']):
            expected = 'sdk_transmitted' if topic.endswith('sdk_command') else 'sdk_measured'
            if msg.header.frame_id != expected or msg.header.stamp.to_nsec() <= 0:
                raise ValueError('invalid recorded joint provenance')
            if len(msg.name) != len(set(msg.name)) or len(msg.name) != len(msg.position):
                raise ValueError('ambiguous recorded joints')
            named = dict(zip(msg.name, msg.position))
            rows.add(tuple(float(named[name]) for name in ARM_NAMES))
            counts[topic] = counts.get(topic, 0) + 1
    if not rows:
        raise ValueError('missing recorded joint samples')
    return np.asarray(sorted(rows)), counts


def support_sensitivity(reference, candidate, original):
    """Transform the ORIGINAL fused planes, not refit/select a better ROI.

    Fused-window representative poses only; this is a model sensitivity check,
    not a replacement for per-frame RGB-D reconstruction or registration.
    """
    baseline = next(e for e in original['experiments']
                    if e['support_bbox_expand_ratio'] == original['production_expand_ratio'])
    frame_change = np.linalg.inv(candidate(np.zeros(6))) @ reference(np.zeros(6))
    normals, movements = {}, {}
    for name, view in baseline['views'].items():
        q = view['detail']['joint_positions'][:6]
        old = reference(q)
        new = candidate(q) @ frame_change
        delta = new @ np.linalg.inv(old)
        normals[name] = delta[:3, :3] @ np.asarray(view['support_normal_base'])
        camera = np.asarray(view['detail']['T_base_optical'])
        movements[name] = pose_error(camera, delta @ camera)
    pairs = {}
    for a, b in [('far', 'near'), ('far', 'reached'), ('reached', 'near')]:
        pairs[a+'_to_'+b] = float(np.degrees(np.arctan2(
            np.linalg.norm(np.cross(normals[a], normals[b])), normals[a] @ normals[b])))
    return {'scope': 'representative_pose_model_sensitivity_not_refitted_registration',
            'roi_changed': False, 'zero_frame_change_applied_to_attachment': True,
            'camera_pose_changes': movements, 'normal_difference_deg': pairs,
            'original_comparisons': baseline['comparisons'], 'execution_authority': False}


def following_sensitivity(reference, candidate, original):
    """Same recorded command/encoder pair under each model, no new target."""
    if original.get('model_sha256') != reference.model_sha256:
        raise ValueError('following report belongs to a different frozen model')
    frame_change = np.linalg.inv(candidate(np.zeros(6))) @ reference(np.zeros(6))
    result = []
    for window in original['windows']:
        command, measured = window['sdk_positions_rad'], window['accepted_positions_rad']
        before = pose_error(reference(command), reference(measured))
        if not np.isclose(before['translation_mm'], window['position_error_m']*1000,
                          rtol=0, atol=1e-8):
            raise ValueError('following report scalar does not match recorded angles')
        result.append({'window': window['window'], 'reference': before,
                       'candidate_raw_tcp': pose_error(candidate(command), candidate(measured)),
                       'candidate_reference_tcp': pose_error(candidate(command) @ frame_change,
                                                             candidate(measured) @ frame_change)})
    return {'scope': 'same_historical_angles_no_compensation_or_physical_accuracy_claim',
            'windows': result, 'execution_authority': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parameters', required=True)
    parser.add_argument('--candidate', action='append', required=True, help='label=URDF path')
    parser.add_argument('--mjcf', help='optional local server MJCF; world-frame, open-finger midpoint')
    parser.add_argument('--bag', action='append', default=[])
    parser.add_argument('--support-report')
    parser.add_argument('--following-report')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    parameters = yaml.safe_load(Path(args.parameters).read_text())
    xml = parameters['robot_description']
    reference = SerialUrdfFk(xml, ARM_NAMES)
    models = {}
    for entry in args.candidate:
        label, path = entry.split('=', 1)
        if label in models:
            raise ValueError('duplicate model label')
        text = Path(path).read_text()
        models[label] = (path, text, SerialUrdfFk(text, ARM_NAMES))
    if args.mjcf:
        if 'local_server_mjcf' in models:
            raise ValueError('reserved MJCF label')
        text = Path(args.mjcf).read_text()
        models['local_server_mjcf'] = (args.mjcf, text, mjcf_arm_fk(text))
    limits = np.stack([f.limits for f in [reference]+[v[2] for v in models.values()]])
    lower, upper = limits[:, :, 0].max(axis=0), limits[:, :, 1].min(axis=0)
    if not np.all(np.isfinite([lower, upper])) or np.any(lower >= upper):
        raise ValueError('empty/nonfinite shared joint domain')
    random = np.random.default_rng(20260914).uniform(lower, upper, size=(1000, 6))
    zero = np.zeros((1, 6))
    single = np.vstack([np.eye(6)*.1, -np.eye(6)*.1])
    samples = {'zero_and_single_joint': np.vstack([zero, single]), 'multiaxis_seed_20260914': random}
    bag_counts = {}
    for path in args.bag:
        samples[str(path)], bag_counts[str(path)] = read_bag_samples(path)
    report = {'read_only': True, 'execution_authority': False,
              'parameters_path': args.parameters,
              'parameters_sha256': hashlib.sha256(Path(args.parameters).read_bytes()).hexdigest(),
              'reference_model_sha256': reference.model_sha256,
              'reference_definitions': definitions(xml), 'bag_message_counts': bag_counts,
              'shared_sampling_limits_rad': np.stack([lower, upper], axis=1).tolist(), 'candidates': {}}
    for label, (path, text, fk) in models.items():
        result = {'path': path, 'model_sha256': fk.model_sha256, 'definitions': definitions(text),
                  'samples': {name: compare(reference, fk, rows) for name, rows in samples.items()}}
        if args.support_report and label != 'local_server_mjcf':
            result['support_sensitivity'] = support_sensitivity(reference, fk,
                json.loads(Path(args.support_report).read_text()))
        if args.following_report:
            result['following_sensitivity'] = following_sensitivity(reference, fk,
                json.loads(Path(args.following_report).read_text()))
        report['candidates'][label] = result
    with Path(args.output).open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({k: {'samples': {n: {kind: {metric: values['maximum']
            for metric, values in stats.items()} for kind, stats in v['statistics'].items()}
            for n, v in c['samples'].items()},
            'support_normal_difference_deg': c.get('support_sensitivity', {}).get('normal_difference_deg')}
                      for k, c in report['candidates'].items()}, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
