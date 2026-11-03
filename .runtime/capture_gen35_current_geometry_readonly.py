"""Read-only exact RGB-D/TF/configuration evidence, with no target authority."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
mount = Path('/media/zhuyupei/SanStand')
out = args.output.resolve()
assert mount.is_mount()
out.relative_to(mount / '实验记录/Robot-Controller/ros_log')
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(out / 'ros')
import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('capture_gen35_geometry_readonly', anonymous=True, disable_rosout=True)
keys = ('/robot_description', '/camera', '/handeye', '/perception',
        '/unknown_perception', '/grasp', '/grasp_6d/remote')
def configuration():
    return {key: rospy.get_param(key, None) for key in keys}
config_before = configuration()
assert rospy.get_param('/gui/joint_direct_mode') is True
selection = rospy.get_param('/grasp_mode/selection')
started_ns = rospy.Time.now().to_nsec()
buffer = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
listener = tf2_ros.TransformListener(buffer)
lock = threading.Lock()
pairs, states, latest = {}, [], {}
def receive(kind, message):
    with lock:
        if kind in ('color', 'depth'):
            stamp = message.header.stamp.to_nsec()
            if stamp >= started_ns:
                pairs.setdefault(stamp, {})[kind] = message
            for old in sorted(pairs)[:-20]:
                del pairs[old]
        elif kind == 'q':
            states.append(message)
            del states[:-120]
        else:
            latest[kind] = message
subscriptions = [rospy.Subscriber(topic, kind, lambda m, k=key: receive(k, m),
    queue_size=12, buff_size=2**24) for key, topic, kind in (
    ('color', '/supervisor/camera/color/image_raw', Image),
    ('depth', '/supervisor/camera/depth/image_raw', Image),
    ('q', '/alicia_d/accepted_joint_states', JointState),
    ('epoch', '/alicia_d/control_reference_epoch', Header),
    ('task', '/grasp/state', GraspState))]
try:
    captured = None
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        with lock:
            candidates = [(stamp, dict(pair)) for stamp, pair in sorted(pairs.items())
                          if len(pair) == 2]
            history, context = list(states), dict(latest)
        if len(history) < 20 or not {'epoch', 'task'} <= set(context):
            time.sleep(.03)
            continue
        assert not context['task'].active
        for stamp, pair in candidates:
            if not 0 <= rospy.Time.now().to_nsec()-stamp <= 800_000_000:
                continue
            q = min(history, key=lambda item: abs(item.header.stamp.to_nsec()-stamp))
            if q.header.frame_id != 'sdk_measured' or abs(q.header.stamp.to_nsec()-stamp) > 50_000_000:
                continue
            camera_frame = config_before['/handeye']['camera_frame']
            try:
                transforms = [buffer.lookup_transform(parent, child,
                    rospy.Time(stamp//10**9, stamp % 10**9), rospy.Duration(.15))
                    for parent, child in (('base_link', camera_frame),
                        ('base_link', 'tool0'), ('tool0', camera_frame))]
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue
            captured = stamp, pair, q, history, context, transforms
            break
        if captured:
            break
        time.sleep(.03)
    assert captured is not None, 'No exact current image-stamp TF and RGB-D pair'
    stamp, pair, q, history, context, transforms = captured
    config_after = configuration()
    # The camera publishes delayed per-frame region diagnostics here. Keep
    # both original trees; ignore only this non-control field for comparison.
    # See camera_node._publish_temporal_status: geometry/control do not read it.
    comparable = []
    for configuration_value in (config_before, config_after):
        item = copy.deepcopy(configuration_value)
        item['/camera'].get('runtime_profile', {}).get('stationary_temporal', {}).pop('regions', None)
        comparable.append(item)
    assert comparable[0] == comparable[1], 'Planning configuration changed during capture'
    assert rospy.get_param('/gui/joint_direct_mode') is True
    assert rospy.get_param('/grasp_mode/selection') == selection
    with lock:
        assert not latest['task'].active
        assert latest['epoch'].stamp.to_nsec() == context['epoch'].stamp.to_nsec()
    def joint_dict(item):
        return dict(stamp_ns=item.header.stamp.to_nsec(), frame_id=item.header.frame_id,
                    names=list(item.name), positions=list(item.position))
    reference = dict(zip(q.name, q.position))
    assert all(set(reference) == set(item.name) for item in history)
    delta = max(abs(reference[name]-value) for item in history
                for name, value in zip(item.name, item.position))
    assert delta <= .004, 'Pose changed during capture; diagnostic capture rejected'
    def transform_dict(item):
        t, r = item.transform.translation, item.transform.rotation
        return dict(parent=item.header.frame_id, child=item.child_frame_id,
            stamp_ns=item.header.stamp.to_nsec(), translation=[t.x,t.y,t.z],
            rotation_xyzw=[r.x,r.y,r.z,r.w])
    bridge = CvBridge()
    assert cv2.imwrite(str(out/'current_scene.png'), bridge.imgmsg_to_cv2(pair['color'], 'bgr8'))
    np.savez_compressed(out/'current_depth.npz',
        depth=bridge.imgmsg_to_cv2(pair['depth'], 'passthrough'), stamp_ns=stamp)
    (out/'configuration.json').write_text(json.dumps(config_before, ensure_ascii=False, indent=2)+'\n')
    (out/'configuration_after.json').write_text(json.dumps(config_after, ensure_ascii=False, indent=2)+'\n')
    result = dict(kind='readonly_current_geometry_capture', image_stamp_ns=stamp,
        captured_ns=rospy.Time.now().to_nsec(), image_frames={k:v.header.frame_id for k,v in pair.items()},
        transforms=[transform_dict(item) for item in transforms],
        sdk=joint_dict(q), sdk_samples=[joint_dict(item) for item in history],
        maximum_joint_delta_rad=delta, selection=selection,
        epoch_ns=context['epoch'].stamp.to_nsec(), configuration_unchanged=config_before == config_after,
        planning_configuration_unchanged=True,
        configuration_comparison_excludes=['/camera/runtime_profile/stationary_temporal/regions'],
        task_active=False, manual_mode=True, motion_commands_sent=False,
        temperature_queried=False, target_identity_assigned=False, execution_authorized=False)
    result['files_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (out/'current_scene.png', out/'current_depth.npz', out/'configuration.json', out/'configuration_after.json')}
    result['capture_script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(out), image_stamp_ns=stamp,
        maximum_joint_delta_rad=delta, transforms=len(transforms), execution_authorized=False)))
finally:
    for subscriber in subscriptions:
        subscriber.unregister()
