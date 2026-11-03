"""Offline recorded-config and actual geometry support for the three-owner replay.

No ROS initialization, subscriptions, workers, service calls, or robot interfaces.
Initial surface reconstruction is historical evidence, not arrival qualification.
"""
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5')
F = N/'verification/gen14_arrival_independent'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_recorded_parameters(camera, unknown, selection):
    """Parse the recorded launch locally, then apply saved runtime snapshots.

    The remote/perception baseline is reconstruction from SHA-bound launch
    inputs, not a claimed complete runtime parameter dump. The audit says so.
    """
    import roslaunch.config
    import roslaunch.xmlloader
    startup = json.loads((N/'startup.json').read_text())
    checked = {}
    for name, expected in startup['startup_dependencies_sha256'].items():
        if name.endswith('.launch'):
            actual = digest(W/name)
            assert actual == expected, (name, actual, expected)
            checked[name] = actual
    config = roslaunch.config.ROSLaunchConfig()
    launch = W/'.runtime/full_latest_frozen_two_observations_20261011.launch'
    roslaunch.xmlloader.XmlLoader().load(str(launch), config, verbose=False)
    tree = {}
    def assign(key, value):
        cursor = tree
        parts = key.strip('/').split('/')
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = deepcopy(value)
    for key, parameter in config.params.items():
        assign(key, parameter.value)
    saved_path = R/'perception_execution_configuration.json'
    saved = json.loads(saved_path.read_text())['parameters']
    for key, value in saved.items():
        assign(key, value)
    context_path = N/'readonly_full_frame_probe_002/context.json'
    context = json.loads(context_path.read_text())['parameters']
    for key in ('/handeye', '/robot_description'):
        assign(key, context[key])
    for key, value in {'/camera': camera, '/unknown_perception': unknown,
                       '/grasp_mode/selection': selection,
                       '/gui/joint_direct_mode': False}.items():
        assign(key, value)
    params = {}
    def flatten(node, path=''):
        for key, value in node.items():
            full = path+'/'+key
            params[full] = deepcopy(value)
            if isinstance(value, dict):
                flatten(value, full)
    flatten(tree)
    audit = dict(
        policy='static_launch_baseline_plus_original_saved_runtime_overrides',
        no_ros_master_access=True,
        runtime_snapshot=str(saved_path), runtime_snapshot_sha256=digest(saved_path),
        context_path=str(context_path), context_sha256=digest(context_path),
        startup_bound_launch_sha256=checked,
        parsed_launch_sha256={str(Path(p).resolve()):digest(p) for p in config.roslaunch_files},
        limitation='The record did not contain a complete remote/perception runtime parameter dump; unrecorded values are reconstructed from the recorded launch baseline, not asserted to be separately measured runtime values.',
        selected_parameters={k:params[k] for k in ('/grasp_6d/remote','/perception')})
    return params, audit


def configure_remote_geometry(node, base, parameters, lookup_transform):
    """Evaluate only named pure config assignments from the original __init__.

    This avoids copying constructor defaults while excluding all ROS I/O setup.
    Calls to rospy.get_param are handled by the enclosing saved-parameter patch.
    """
    source = Path(base.__file__).read_text()
    cls = next(x for x in ast.parse(source).body if isinstance(x, ast.ClassDef)
               and x.name == 'RemoteGrasp6DNode')
    init = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name == '__init__')
    names = {
        'camera_depth_scale', 'gripper_geometry', 'gripper_tool_jaw_axis',
        'gripper_tool_finger_length_axis', 'adaptive_stage_limits',
        'multiview_registration_config', 'planning_snapshot_frames',
        'near_field_planning_snapshot_frames', 'planning_snapshot_timeout_sec',
        'planning_snapshot_max_age_sec', 'planning_snapshot_max_inference_latency_sec',
        'planning_snapshot_max_span_sec', 'planning_mask_min_iou',
        'planning_mask_max_centroid_shift_px', 'planning_max_joint_delta_rad',
        'mask_erosion_px', 'mask_internal_hole_max_area_px', 'depth_mad_scale',
        'depth_mad_absolute_floor_m', 'geometry_support_bbox_expand_ratio',
        'geometry_support_distance_threshold_m', 'geometry_voxel_size_m',
        'geometry_outlier_neighbors', 'geometry_outlier_std_ratio',
        'geometry_min_support_points', 'geometry_min_object_points',
        'geometry_min_size_m', 'geometry_max_size_m', 'geometry_max_height_m',
    }
    statements=[];found=set()
    for statement in init.body:
        if not isinstance(statement, ast.Assign) or len(statement.targets)!=1:
            continue
        target=statement.targets[0]
        if isinstance(target,ast.Attribute) and isinstance(target.value,ast.Name) \
                and target.value.id=='self' and target.attr in names:
            statements.append(statement);found.add(target.attr)
    assert found==names, names-found
    remote_cfg=deepcopy(parameters['/grasp_6d/remote'])
    scope=dict(vars(base),self=node,remote_cfg=remote_cfg,
               cam_cfg=parameters['/camera'],
               gripper_geometry_cfg=dict(remote_cfg.get('gripper_geometry',{}) or {}))
    pure=ast.fix_missing_locations(ast.Module(body=statements,type_ignores=[]))
    exec(compile(pure, str(base.__file__)+' [pure config assignments]', 'exec'),scope)
    node.previous_object_axes_base=None
    node._geometry_invalidation_generation=0
    node._last_geometry_invalidation_code=''
    node._latest_target_observation=None
    node._latest_geometry_estimate=None
    node.tf_buffer=SimpleNamespace(lookup_transform=lookup_transform)
    node.pose_estimator=SimpleNamespace(tf_timeout_sec=.2,camera_frame=parameters['/camera']['frame_id'])
    node.frames.configure_retention(node.planning_snapshot_max_age_sec,
                                    node.planning_snapshot_max_inference_latency_sec)
    return dict(original_constructor_path=str(base.__file__),
                original_constructor_sha256=digest(base.__file__),
                evaluated_attribute_names=sorted(names),
                evaluated_constructor_ranges=[[s.lineno,s.end_lineno] for s in statements],
                constructor_io_executed=False)


def freeze_initial_surface(node, base, fixture_directories):
    """Reconstruct the exact saved initial view through original RGBD/geometry.

    Uses a separate historical buffer. Its clock is the saved initial message
    reception; this setup is outside the arrival replay, and is audited as such.
    """
    import rospy
    import tf2_py
    import roslib.message
    from alicia_grasp_modes.observation_information_uncertainty import observation_information_uncertainty_m
    manifests=[];events=[]
    for directory in fixture_directories:
        path=Path(directory)/'manifest.json';manifest=json.loads(path.read_text())
        assert digest(manifest['file'])==manifest['sha256']
        manifests.append((path,manifest))
        for row in manifest['rows']:
            raw=Path(row['file']).read_bytes()
            assert hashlib.sha256(raw).hexdigest()==row['sha256']
            cls=roslib.message.get_message_class(row['message_type'])
            message=cls().deserialize(raw)
            assert message._md5sum==row['md5sum']
            events.append((row['received_ns'],row['kind'],message))
    manifests.sort(key=lambda pair:pair[1]['source_ns'])
    sources=[m['source_ns'] for _,m in manifests]
    assert sources==[1791860167282787322,1791860167481009244,1791860167684837341]
    historical_now=[min(row[0] for row in events)]
    frames=base.SynchronizedRgbdBuffer(source_clock_ns=lambda:historical_now[0],
                                      monotonic_clock=lambda:historical_now[0]*1e-9)
    frames.configure_retention(node.planning_snapshot_max_age_sec,
                               node.planning_snapshot_max_inference_latency_sec)
    tf=tf2_py.BufferCore(rospy.Duration(600))
    identity=node._current_stream_target_identity(); samples=[];admission=[]
    # Multiple source fixtures can repeat the same original SDK/TF message.
    seen=set()
    for at,kind,msg in sorted(events,key=lambda e:e[0]):
        key=(at,kind,repr(getattr(msg,'header',None)))
        if key in seen:continue
        seen.add(key);historical_now[0]=at
        if kind.startswith('sdk'):
            by_name=dict(zip(msg.name,msg.position))
            frames.update_joints([by_name['Joint%d'%i] for i in range(1,7)])
        elif kind.startswith('tf'):
            setter=tf.set_transform_static if kind=='tf_static' else tf.set_transform
            for transform in msg.transforms:setter(transform,'original_sealed_history')
        elif kind=='rgb':frames.update_color(node.bridge.imgmsg_to_cv2(msg,'bgr8'),msg.header.stamp,msg.header.frame_id)
        elif kind=='depth':frames.update_depth(node.bridge.imgmsg_to_cv2(msg,'passthrough'),msg.header.stamp,msg.header.frame_id)
        elif kind=='mask':frames.update_mask(node.bridge.imgmsg_to_cv2(msg,'mono8'),msg.header.stamp,msg.header.frame_id)
        elif kind=='object':
            assert msg.detected
            frames.update_object(msg,msg.header.stamp,identity.epoch,identity)
        else:continue
        result=frames.wait_for_samples(node.near_field_planning_snapshot_frames,0.,
            require_mask=node._active_profile_requires_mask(),
            max_age_sec=node.planning_snapshot_max_age_sec,
            collection_span_sec=node.planning_snapshot_max_span_sec,
            max_inference_latency_sec=node.planning_snapshot_max_inference_latency_sec,
            newest_after_ns=node._mode_selection['stamp_ns'],target_identity=identity,
            require_all_after_ns=True)
        admission.append(dict(received_ns=at,kind=kind,source_stamps_ns=[s.stamp_ns for s in result]))
        if result:samples=result
    assert [sample.stamp_ns for sample in samples]==sources
    depth_scale,depth_min,depth_max=node._snapshot_depth_config()
    snapshot=base.fuse_stable_samples(samples,require_mask=node._active_profile_requires_mask(),
        min_mask_iou=node.planning_mask_min_iou,
        max_centroid_shift_px=node.planning_mask_max_centroid_shift_px,
        max_joint_delta_rad=node.planning_max_joint_delta_rad,
        erosion_px=node.mask_erosion_px,depth_scale=depth_scale,
        depth_min_m=depth_min,depth_max_m=depth_max,mad_scale=node.depth_mad_scale,
        mad_absolute_floor_m=node.depth_mad_absolute_floor_m,
        internal_hole_max_area_px=node.mask_internal_hole_max_area_px)
    assert snapshot.ok,(snapshot.failure_code,snapshot.failure_reason)
    prior_buffer=node.tf_buffer;prior_active=node.near_field_planning_active
    node.tf_buffer=SimpleNamespace(lookup_transform=lambda target,source,when,timeout:tf.lookup_transform_core(target,source,when))
    node.near_field_planning_active=False
    try:
        estimate,_depth,transform=node._prepare_snapshot_geometry(
            snapshot,rospy.Time(sources[-1]//10**9,sources[-1]%10**9))
    finally:
        node.tf_buffer=prior_buffer;node.near_field_planning_active=prior_active
    assert estimate.ok,(estimate.failure_code,estimate.failure_reason)
    observation=base.observation_from_snapshot(snapshot,estimate,transform,'base_link')
    reference=base.surface_view_from_observation(observation)
    surface=base.fused_surface_from_view(reference,voxel_size_m=node.geometry_voxel_size_m)
    audit_path=R/'keyframes_audits/0e4b37a0810b68c784653ec71741d0a15c70bee954ea112c830e5a3c325e09cc.json'
    original=json.loads(audit_path.read_text())['observe_return']
    gap=original['measured_missing_sides'][0]
    center_error=float(np.max(np.abs(np.asarray(estimate.center_base)-gap['target_center_base_m'])))
    height=node._geometry_object_height_m(estimate)
    height_error=abs(height-gap['object_height_m'])
    camera_error=float(np.max(np.abs(transform[:3,3]-gap['initial_camera_origin_base_m'])))
    assert center_error<1e-10 and height_error<1e-10 and camera_error<1e-12,(
        'historical three-frame geometry does not reproduce recorded initial geometry',
        center_error,height_error,camera_error)
    node._observe_return_saved=(surface,reference)
    node._observe_return_initial_stamp=sources[-1]
    node._observe_return_view_gaps=deepcopy(original['measured_missing_sides'])
    node._observe_return_trigger=original['trigger_code']
    node._observe_return_trigger_evidence=deepcopy(original['trigger_evidence'])
    node._observe_return_view_uncertainty_m=observation_information_uncertainty_m(
        node._adaptive_stage_profiles(estimate,snapshot))
    node._latest_target_observation=None
    return dict(scope='Historical initial three-frame RGBD fusion and actual geometry reconstruction only; no arrival qualification or registration seeded.',
        manifest_sha256={str(path):digest(path) for path,_ in manifests},
        source_stamps_ns=sources,admission=admission,
        snapshot_quality=vars(snapshot.quality),
        target_depth_sha256=hashlib.sha256(np.ascontiguousarray(snapshot.target_depth_raw).tobytes()).hexdigest(),
        reference_point_count=len(reference.points_base),fused_point_count=len(surface.points_base),
        reconstructed_points_sha256=hashlib.sha256(np.ascontiguousarray(surface.points_base).tobytes()).hexdigest(),
        pointcloud_sha_is_reconstructed_not_original=True,
        original_audit_path=str(audit_path),original_audit_sha256=digest(audit_path),
        center_residual_m=center_error,height_residual_m=height_error,camera_origin_residual_m=camera_error,
        measured_height_m=height,center_base=np.asarray(estimate.center_base).tolist(),
        depth_uncertainty_m=node._observe_return_view_uncertainty_m,
        missing_depth_filled=False,arrival_qualification_seeded=False)
