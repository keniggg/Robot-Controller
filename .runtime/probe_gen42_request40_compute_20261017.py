"""Bounded offline request40 calculation and held-lock timing only.

No node constructor, worker, master, publisher, client, IK, controller or device
is started. Restores all three original RGB-D-mask samples and preappend CAS.
The calculation must match archived CAS points and registration exactly before
candidate calculation. Timings are new offline observations, never a historical
callback trace, execution permission or a physical grasp result.
"""
import argparse
import ast
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time
import traceback
from types import SimpleNamespace as NS

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
E = MOUNT/'实验记录/Robot-Controller'
D = E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
V = D/'verification'
INPUT = V/'gen42_request40_probe_preparation_r4'
TRACE = V/'gen41_actual_attempt1_handoff_trace_r1'
STAMPS = (1792237678248981237, 1792237678533539533, 1792237679451562404)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def plain(value):
    if isinstance(value, dict) or hasattr(value, 'items'):
        return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if hasattr(value, 'tolist'):
        return value.tolist()
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    a = parser.parse_args()
    output = a.output.resolve()
    assert MOUNT.is_mount() and E/'ros_log' in output.parents
    assert output.parent.exists() and output.parent.stat().st_dev == MOUNT.stat().st_dev
    output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(output/'ros')
    os.environ['TMPDIR'] = str(output/'tmp')
    (output/'tmp').mkdir()
    sys.dont_write_bytecode = True
    sys.excepthook = sys.__excepthook__
    import platform
    platform.architecture = lambda *args, **kwargs: (str(struct.calcsize('P')*8)+'bit', 'ELF')
    platform._syscmd_uname = lambda option, default='': default
    socket.has_ipv6 = False
    violations, locks, durations = [], [], []
    result = dict(passed=False, physical_grasp_verified=False, execution_authorized=False,
        historical_callback_timing_reconstructed=False, request_id=40, actual_source_stamps_ns=STAMPS,
        helper_sha256=sha(__file__),
        timing_kind='new offline measurements with tracing overhead; no historical lock trace exists')

    def save(name, value):
        assert MOUNT.is_mount() and output.stat().st_dev == MOUNT.stat().st_dev
        with (output/name).open('x') as f:
            json.dump(plain(value), f, ensure_ascii=False, indent=2, allow_nan=False)
            f.write('\n')

    class Boundary(BaseException):
        pass

    def forbidden(*args, **kwargs):
        violations.append(repr(args)[:500])
        raise Boundary('unsimulated offline boundary: '+repr(args)[:300])

    for name in ('connect','connect_ex','sendto','bind','listen'):
        setattr(socket.socket, name, forbidden)
    socket.create_connection = forbidden
    socket.getaddrinfo = forbidden

    def guard(event, args):
        if event in ('subprocess.Popen','os.system','os.posix_spawn','os.exec'):
            forbidden(event, args)
        if event == 'open' and args and isinstance(args[0], (str, bytes, os.PathLike)):
            path = os.path.realpath(os.fsdecode(args[0]))
            if path.startswith('/dev/') and path not in (
                    '/dev/null','/dev/zero','/dev/random','/dev/urandom') and not path.startswith(
                    ('/dev/shm/','/dev/fd/','/dev/pts/')):
                forbidden('physical_device_open', path)
    sys.addaudithook(guard)
    start = time.perf_counter_ns()

    class TracedRLock:
        def __init__(self, name):
            self.lock, self.name, self.depth, self.outer = threading.RLock(), name, 0, None
        def acquire(self, *args, **kwargs):
            started = time.perf_counter_ns()
            ok = self.lock.acquire(*args, **kwargs)
            if ok:
                now = time.perf_counter_ns()
                self.depth += 1
                caller = sys._getframe(1)
                row = dict(lock=self.name, kind='acquire', at_ns=now-start, depth=self.depth,
                    wait_ns=now-started, caller=caller.f_code.co_name, line=caller.f_lineno)
                locks.append(row)
                if self.depth == 1:
                    self.outer = row
            return ok
        def release(self):
            now = time.perf_counter_ns()
            if self.depth == 1:
                locks.append(dict(lock=self.name, kind='outer_release', at_ns=now-start,
                    held_ns=now-start-self.outer['at_ns'], acquired_by=self.outer['caller'],
                    acquired_line=self.outer['line']))
            self.depth -= 1
            self.lock.release()
        def __enter__(self):
            self.acquire()
            return self
        def __exit__(self, *args):
            self.release()
        def _is_owned(self):
            return self.lock._is_owned()

    def timed(label, function, *args, **kwargs):
        t = time.perf_counter_ns()
        try:
            return function(*args, **kwargs)
        finally:
            durations.append(dict(label=label, start_ns=t-start, elapsed_ns=time.perf_counter_ns()-t))

    try:
        release_path = V/'gen41_release_manifest.json'
        release = read(release_path)
        assert sha(release_path) == 'b98c25da01b77582bff4567a791afed8b8c908fcfe391b3b84601276c9ed706d'
        sources = release['source_sha256']
        assert len(sources) == 1032 and all(sha(W/n) == h for n,h in sources.items())
        ready = read(INPUT/'data_readiness.json')
        assert ready['passed'] and ready['requested_stamps_ns'] == list(STAMPS)
        assert ready['boundary_attempts'] == [] and ready['source_count'] == 1032
        for name, row in ready['arrays'].items():
            assert sha(INPUT/name) == row['sha256']
        for path, digest in ready['input_sha256'].items():
            assert sha(path) == digest
        assert read(INPUT/'cas_before.json')['request_binding']['current_source_not_appended'] is True
        input_files = [p for p in INPUT.iterdir() if p.is_file()]
        input_files += [TRACE/'events.json', TRACE/'accepted_samples.json']
        input_hashes = {str(p):sha(p) for p in input_files}
        import numpy as np
        import cv2
        import yaml
        import rospy
        for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy',
                     'wait_for_service','set_param','delete_param','get_master'):
            setattr(rospy, name, forbidden)
        for name in ('loginfo','logwarn','logerr','logdebug','loginfo_throttle',
                     'logwarn_throttle','logerr_throttle','logdebug_throttle'):
            setattr(rospy, name, lambda *args, **kwargs: None)
        cv2.setNumThreads(1)
        source_path = W/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
        spec = importlib.util.spec_from_file_location('gen42_archived_original_remote', source_path)
        remote = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = remote
        spec.loader.exec_module(remote)
        from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
        from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
        from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView, RegistrationConfig
        from alicia_flexible_grasp.vision.fused_scene_geometry import _array_digest
        from alicia_flexible_grasp.utils.transform_utils import matrix_from_xyz_quat

        cfg = yaml.safe_load((W/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())
        archived = read(INPUT/'perception_execution_configuration.json')['parameters']
        remote_cfg = deepcopy(cfg['grasp_6d']['remote'])
        remote_cfg['multiview'] = archived['/grasp_6d/remote/multiview']
        camera = read(INPUT/'camera_configuration.json')
        K = NS(**{k:camera[k] for k in ('width','height','fx','fy','cx','cy','depth_scale')})
        before, expected_ref, expected_snapshot = [read(INPUT/('cas_'+n+'.json')) for n in ('before','reference','snapshot')]
        identity = TargetTrackIdentity(**before['identity'])
        events = read(TRACE/'events.json')
        errors = [r for r in events if r['topic']=='/experiment/runtime_execution_error_snapshot'
            and r['bag_receive_ns'] <= STAMPS[-1]]
        assert errors
        error = errors[-1]['data']['json']
        handeye = yaml.safe_load((W/'.runtime/candidate_models/expanded_metric_20260929/handeye_candidate_config.yaml').read_text())['handeye']
        urdf = W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
        params = {'/grasp':archived['/grasp'], '/grasp_6d/remote':remote_cfg,
            '/grasp_6d/runtime_execution_error':error, '/camera':camera, '/handeye':handeye,
            '/gripper':cfg['gripper'], '/robot_description':urdf.read_text(),
            '/grasp/unknown_require_measured_bilateral_contact':archived['/grasp']['unknown_require_measured_bilateral_contact']}
        reads = []
        def get_param(name, default=None):
            reads.append(name)
            if name in params:
                return deepcopy(params[name])
            for root in ('/grasp_6d/remote','/grasp','/camera','/handeye','/gripper'):
                if name.startswith(root+'/'):
                    value = params[root]
                    try:
                        for part in name[len(root)+1:].split('/'):
                            value = value[part]
                        return deepcopy(value)
                    except (KeyError, TypeError):
                        return default
            raise ValueError('unprovided archived parameter: '+name)
        rospy.get_param = rospy.get_param_cached = get_param
        rospy.Time.now = staticmethod(lambda: rospy.Time(*divmod(STAMPS[-1],10**9)))
        node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
        namespace = dict(vars(remote), self=node, remote_cfg=remote_cfg, gcfg=archived['/grasp'],
            grasp_cfg=archived['/grasp'], cam_cfg=camera, hcfg=handeye,
            gripper_cfg=cfg['gripper'], twin_cfg=cfg.get('mujoco_digital_twin',{}),
            gripper_geometry_cfg=dict(remote_cfg['gripper_geometry']),
            default_soft_weights=remote.SoftScoreWeights(), soft_weight_cfg=remote_cfg.get('soft_score_weights',{}),
            startup_continuous_config=remote.load_continuous_runtime_config(remote_cfg,get_param=get_param))
        names = {'gripper_geometry','opening_fit_clearance_each_side_m','camera_depth_scale',
            'tabletop_geometry_enabled','tabletop_geometry_config','hybrid_merge_config',
            'adaptive_stage_limits','runtime_execution_error_max_age_sec','gripper_tool_jaw_axis',
            'gripper_tool_finger_length_axis','soft_score_weights','target_absolute_sanity_distance_m',
            'grasp_config','handeye_parent_frame','handeye_camera_frame','handeye_translation_xyz',
            'handeye_rotation_xyzw','handeye_allow_static_fallback','_cached_tool_from_camera',
            '_cached_tool_from_camera_stamp_ns','gripper_physical_open_width_m',
            'candidate_min_downward_approach_cos','candidate_max_final_approach_lateral_m',
            'planning_mask_min_iou','planning_mask_max_centroid_shift_px','planning_max_joint_delta_rad',
            'mask_erosion_px','mask_internal_hole_max_area_px','depth_mad_scale','depth_mad_absolute_floor_m',
            'geometry_support_bbox_expand_ratio','geometry_support_distance_threshold_m',
            'geometry_voxel_size_m','geometry_outlier_neighbors','geometry_outlier_std_ratio',
            'geometry_min_support_points','geometry_min_object_points','geometry_min_size_m',
            'geometry_max_size_m','geometry_max_height_m'}
        tree = ast.parse(source_path.read_text())
        original_init = next(f for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RemoteGrasp6DNode'
            for f in c.body if isinstance(f,ast.FunctionDef) and f.name=='__init__')
        configured = []
        for statement in original_init.body:
            if not isinstance(statement,ast.Assign):
                continue
            attributes = [n.attr for target in statement.targets for n in ast.walk(target)
                if isinstance(n,ast.Attribute) and isinstance(n.value,ast.Name) and n.value.id=='self']
            if attributes and all(n in names for n in attributes):
                exec(compile(ast.Module(body=[statement], type_ignores=[]), str(source_path), 'exec'),namespace)
                configured.append(dict(line=statement.lineno, names=attributes))
        # Production mode reset explicitly selects zero hole filling for the
        # unknown mode (mode_aware_remote_grasp6d.py:183-190). This is not the
        # carton yaml default and no handwritten pixel fusion is used.
        mode_path = W/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
        mode_tree = ast.parse(mode_path.read_text())
        reset = next(f for c in mode_tree.body if isinstance(c,ast.ClassDef)
            for f in c.body if isinstance(f,ast.FunctionDef) and f.name=='_reset_mode')
        node._carton_internal_hole_area_px = node.mask_internal_hole_max_area_px
        node._mode_selection = dict(mode='unknown',strategy='observe_return',generation=6)
        selected_assignment = next(n for n in ast.walk(reset) if isinstance(n,ast.Assign)
            and any(isinstance(t,ast.Attribute) and t.attr=='mask_internal_hole_max_area_px' for t in n.targets))
        exec(compile(ast.Module(body=[selected_assignment], type_ignores=[]),str(mode_path),'exec'),
            dict(vars(remote), self=node, selected=node._mode_selection))
        assert node.mask_internal_hole_max_area_px == 0
        node.multiview_registration_config = RegistrationConfig(**archived['/grasp_6d/remote/multiview'])
        node._stream_condition, node._geometry_state_lock = TracedRLock('stream'), TracedRLock('geometry')
        node._stream_generation = before['request_binding']['generation']
        node._near_field_phase_id = before['request_binding']['phase_id']
        node.near_field_planning_active = True
        node._current_stream_target_identity = lambda: identity
        node._camera_intrinsics = lambda: K
        node._observe_return_session = dict(session_id=before['request_binding']['session_id'])
        def restore_surface(row):
            return FusedTargetSurface(identity,row['points_base'],row['view_indices'],row['view_stamps_ns'],view_sources=row['view_sources'])
        node._near_field_fused_surface = restore_surface(before)
        ref = before['reference']
        node._near_field_reference_view = SurfaceView(identity,ref['stamp_ns'],ref['points_base'],
            ref['support_normal_base'],ref['support_offset_m'],ref['edge_clearance_px'])
        node._near_field_surface_generation = node._stream_generation
        node._near_field_surface_phase_id = node._near_field_phase_id
        node._latest_registration_evidence = None
        old = [r['data'] for r in events if r['topic']=='/grasp_6d/object_geometry'
            and r['data'].get('valid') and r['data']['header']['stamp']['stamp_ns'] <= STAMPS[-1]][-1]
        q = old['pose_base']['orientation']
        node.previous_object_axes_base = matrix_from_xyz_quat([0.,0.,0.],[q[k] for k in ('x','y','z','w')])[:3,:3]
        accepted = read(TRACE/'accepted_samples.json')
        objects = read(INPUT/'object_messages.json')
        samples, source_joints = [], []
        for stamp in STAMPS:
            js = max((r for r in accepted if r['stamp_ns']<=stamp),key=lambda r:r['stamp_ns'])
            assert js['frame_id']=='sdk_measured' and stamp-js['stamp_ns']<200_000_000
            source_joints.append(js)
            row = objects[str(stamp)]
            arrays = {role:np.load(INPUT/(str(stamp)+'_'+role+'.npy'),allow_pickle=False) for role in ('color','depth','mask')}
            qmap = dict(zip(js['names'],js['positions']))
            samples.append(RgbdSample(arrays['color'],arrays['depth'],arrays['mask'],tuple(row['bbox']),
                NS(detected=row['detected'],label=row['label'],confidence=row['confidence']),
                stamp/1e9,'camera_link',np.asarray([qmap['Joint'+str(i)] for i in range(1,7)]),stamp,identity.epoch,identity))
        depth_min,depth_max = camera['runtime_profile']['depth_filters']['range_m']
        snapshot = timed('fuse_stable_samples_actual_three_sources',fuse_stable_samples,samples,True,
            node.planning_mask_min_iou,node.planning_mask_max_centroid_shift_px,node.planning_max_joint_delta_rad,
            node.mask_erosion_px,K.depth_scale,depth_min,depth_max,node.depth_mad_scale,
            node.depth_mad_absolute_floor_m,internal_hole_max_area_px=node.mask_internal_hole_max_area_px)
        assert snapshot.ok, (snapshot.failure_code,snapshot.failure_reason)
        assert snapshot.sample_stamp_ns == STAMPS and snapshot.quality.fused_frames == 3
        assert snapshot.original_sample.stamp_ns == STAMPS[-1]
        ft = read(INPUT/'exact_tf.json')[str(STAMPS[-1])]
        base_camera = matrix_from_xyz_quat(ft['translation'],ft['quaternion_xyzw'])
        source_optical = np.eye(4)
        source_optical[:3,:3] = remote.OPTICAL_TO_ROS_CAMERA
        transform = base_camera@source_optical
        node._snapshot_base_optical_transform = lambda *args: transform
        node.latest_joint_state = None  # no invented request-owned tool audit
        for label in ('estimate_object_geometry','qualify_regular_box_snapshot',
                      'append_registered_view','append_box_consistent_registered_view',
                      'enclose_registered_surface','surface_data','enclose_inferred_box_geometry'):
            actual = getattr(remote,label)
            setattr(remote,label,lambda *args,_f=actual,_label=label,**kwargs: timed(_label,_f,*args,**kwargs))
        for label in ('_register_near_field_surface','_snapshot_multiview_surface',
                      '_contact_boundary_tilts','_materialize_tabletop_union'):
            actual = getattr(node,label)
            setattr(node,label,lambda *args,_f=actual,_label=label,**kwargs: timed(_label,_f,*args,**kwargs))
        geometry, target_depth, actual_transform = timed('_prepare_snapshot_geometry',
            node._prepare_snapshot_geometry,snapshot,rospy.Time(*divmod(STAMPS[-1],10**9)))
        assert geometry.ok, (geometry.failure_code,geometry.failure_reason)
        actual_ref = node._near_field_fused_surface
        np.testing.assert_array_equal(actual_ref.points_base,np.asarray(expected_ref['points_base']))
        np.testing.assert_array_equal(actual_ref.view_indices,np.asarray(expected_ref['view_indices']))
        assert actual_ref.view_stamps_ns==tuple(expected_ref['view_stamps_ns'])
        assert actual_ref.view_sources==tuple(tuple(r) for r in expected_ref['view_sources'])
        np.testing.assert_array_equal(node._latest_registration_evidence.result.transform_base,
            np.asarray(expected_ref['snapshot_to_reference']))
        actual_snapshot,_ = node._snapshot_multiview_surface()
        np.testing.assert_array_equal(actual_snapshot.points_base,np.asarray(expected_snapshot['points_base']))
        digest = _array_digest(geometry.object_points_base)
        assert digest==expected_snapshot['request_binding']['object_points_sha256'],digest
        result['exact_actual_cas_matched_before_candidate_calculation']=True
        result['enclosed_object_points_sha256']=digest
        result['full_object_point_count']=len(geometry.object_points_base)
        result['before_surface_point_count']=len(before['points_base'])
        result['after_surface_point_count']=len(actual_ref.points_base)
        save('geometry_and_source_binding.json',dict(raw=asdict(node._latest_raw_geometry_source['geometry']),
            fused=asdict(geometry),snapshot_quality=asdict(snapshot.quality),source_joints=source_joints,
            source_stamp_ns=snapshot.sample_stamp_ns,configured_original_assignments=configured,
            camera_transform=actual_transform,registration=asdict(node._latest_registration_evidence.result)))
        planning = timed('_prepare_regular_box_snapshot',node._prepare_regular_box_snapshot,snapshot,geometry,actual_transform)
        # Exactly the observe_return qualified supplementary-view branch of
        # mode_aware_remote_grasp6d.py:367-387 delegates to this base function.
        candidates,audit = timed('_generate_tabletop_candidates_qualified_observe_return',
            node._generate_tabletop_candidates,planning,snapshot=snapshot,contact_execution_phase=True,
            target_label=objects[str(STAMPS[-1])]['label'],refine_obb_neighbor_angles=True)
        save('candidate_diagnostics.json',audit)
        save('candidate_summary.json',[dict(source_index=c.source_index,variant_index=c.variant_index,
            audit=dict(c.audit),T_base_tool0=c.T_base_tool0) for c in candidates])
        result.update(passed=True,candidate_count=len(candidates),parameter_reads=sorted(set(reads)),
            snapshot_quality=asdict(snapshot.quality),raw_retained_source_stamp_ns=snapshot.original_sample.stamp_ns,
            original_source_fusion_used=True,mask_hole_policy='actual unknown-mode reset: zero',
            configuration_basis='sealed YAML plus actual archived /grasp and multiview; live full remote parameter dump was not recorded',
            no_final_request40_candidate_audit_exists=True,current_tool_reorientation_diagnostic_available=False,
            helper_sha256=sha(__file__),input_sha256=input_hashes,source_count=len(sources))
        assert all(sha(W/n)==h for n,h in sources.items())
        assert all(sha(path)==digest for path,digest in input_hashes.items())
        assert not violations
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        result.update(boundary_attempts=violations,ROS_initialized=False,hardware_contacted=False,
            computation_wall_ns=time.perf_counter_ns()-start,recorded_ns=time.time_ns())
        save('lock_intervals.json',locks)
        save('function_durations.json',durations)
        save('result.json',result)
        print(json.dumps(dict(passed=result['passed'],output=str(output),error=result.get('error'))),flush=True)


if __name__ == '__main__':
    main()
