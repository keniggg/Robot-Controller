#!/usr/bin/env python3
import os

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped


def resolve_transform_config(cfg):
    resolved = {
        'parent_frame': cfg.get('parent_frame', cfg.get('base_frame', 'base_link')),
        'child_frame': cfg.get('child_frame', cfg.get('camera_frame', 'camera_color_optical_frame')),
        'translation_xyz': [float(v) for v in cfg.get('translation_xyz', [0, 0, 0])[:3]],
        'rotation_xyzw': [float(v) for v in cfg.get('rotation_xyzw', [0, 0, 0, 1])[:4]],
        'source': 'handeye.yaml',
    }
    calibration_file = cfg.get('calibration_file', '')
    if calibration_file:
        path = os.path.expanduser(os.path.expandvars(str(calibration_file)))
        if os.path.exists(path):
            with open(path, 'r') as handle:
                data = yaml.safe_load(handle) or {}
            params = data.get('parameters', {})
            transform = data.get('transformation', {})
            eye_on_hand = bool(params.get('eye_on_hand', False))
            parent_key = 'robot_effector_frame' if eye_on_hand else 'robot_base_frame'
            resolved['parent_frame'] = cfg.get('parent_frame') or params.get(parent_key) or resolved['parent_frame']
            resolved['child_frame'] = cfg.get('child_frame') or params.get('tracking_base_frame') or resolved['child_frame']
            resolved['translation_xyz'] = [
                float(transform.get('x', resolved['translation_xyz'][0])),
                float(transform.get('y', resolved['translation_xyz'][1])),
                float(transform.get('z', resolved['translation_xyz'][2])),
            ]
            resolved['rotation_xyzw'] = [
                float(transform.get('qx', resolved['rotation_xyzw'][0])),
                float(transform.get('qy', resolved['rotation_xyzw'][1])),
                float(transform.get('qz', resolved['rotation_xyzw'][2])),
                float(transform.get('qw', resolved['rotation_xyzw'][3])),
            ]
            resolved['source'] = path
        else:
            resolved['source'] = 'missing:%s' % path
    return resolved


def make_transform(resolved):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = resolved['parent_frame']
    t.child_frame_id = resolved['child_frame']
    xyz = resolved['translation_xyz']
    q = resolved['rotation_xyzw']
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    return t


def main():
    rospy.init_node('handeye_transform_node')
    cfg = rospy.get_param('/handeye', {})
    if not cfg.get('publish_static_tf', True):
        rospy.spin()
        return
    resolved = resolve_transform_config(cfg)
    if str(resolved.get('source', '')).startswith('missing:'):
        rospy.logwarn('Handeye calibration file not found, using values from handeye.yaml: %s', resolved['source'][8:])
    br = tf2_ros.StaticTransformBroadcaster()
    t = make_transform(resolved)
    br.sendTransform(t)
    rospy.loginfo(
        'Published static handeye TF %s -> %s xyz=%s q=%s source=%s',
        t.header.frame_id,
        t.child_frame_id,
        resolved['translation_xyz'],
        resolved['rotation_xyzw'],
        resolved.get('source', 'handeye.yaml'),
    )
    rospy.spin()


if __name__ == '__main__':
    main()
