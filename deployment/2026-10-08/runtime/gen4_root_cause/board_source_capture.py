"""Exact-header RGB board evidence for a finite diagnostic; no motion publisher."""
import copy
import hashlib
import json
import math
import importlib.util
import time
import threading
from pathlib import Path
import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_matrix
from collect_high_posture import BoundRecorder
from collect_single_axis import C
from bounded_async_bag_writer import BoundedAsyncBagWriter
import gen4_single_axis_plan as base

BOARD_FRAME = 'charuco_fixed_board_20261003'
_recorder = None


def register(recorder):
    global _recorder
    _recorder = recorder


def _tracker_module():
    path = base.ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'
    module = getattr(current_model_evidence, '_module', None)
    if module is None:
        spec = importlib.util.spec_from_file_location('board_rgb_model_math_only', str(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        current_model_evidence._module = module
    return module


def make_model(evidence):
    return _tracker_module().RuntimeColorModel(evidence)


def current_model_evidence():
    if _recorder is None:
        raise RuntimeError('registered current board recorder required')
    module = _tracker_module()
    evidence = module.runtime_color_model_evidence(rospy.get_param('/camera'), 'camera_link', '/supervisor/camera/color/image_raw')
    model = module.RuntimeColorModel(evidence)
    handeye = rospy.get_param('/handeye')
    X = quaternion_matrix(handeye['rotation_xyzw'])
    X[:3, 3] = handeye['translation_xyz']
    return dict(urdf_sha256=hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest(),
                T_tool_camera=X.tolist(), camera_model_evidence=evidence,
                camera_model_fingerprint=model.fingerprint, epoch_ns=_recorder.epoch_ns,
                depth_scale=rospy.get_param('/camera/depth_scale'))


def transform_matrix(transform):
    t = transform.translation
    q = transform.rotation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def require_exact_binding(ns, quality, transform):
    if quality.get('source_stamp_ns') != ns or transform.header.stamp.to_nsec() != ns:
        raise RuntimeError('RGB / quality / board TF source mismatch')
    if transform.header.frame_id != 'camera_link' or transform.child_frame_id != BOARD_FRAME:
        raise RuntimeError('board transform frame mismatch')
    expected = dict(camera_frame='camera_link', board_frame=BOARD_FRAME, intrinsics_source='realsense_runtime',
                    board_size=[12,9], dictionary_id='DICT_5X5_100', square_length_m=.015, marker_length_m=.01125)
    if any(quality.get(k) != v for k,v in expected.items()):
        raise RuntimeError('RGB board specification / frame / model provenance mismatch')
    if not quality.get('pose_ok') or quality.get('camera_model_error') or quality.get('camera_model_change_latched'):
        raise RuntimeError('RGB board pose/model not qualified')
    rms = quality.get('reprojection_rms_px')
    edge = quality.get('edge_clearance_px')
    if (quality.get('charuco_count',0) < 35 or not isinstance(rms,(int,float)) or not math.isfinite(rms)
            or rms > 1.5 or not isinstance(edge,(int,float)) or not math.isfinite(edge) or edge < 20.):
        raise RuntimeError('RGB board corner / residual / image margin not qualified')


class BoardRecorder(BoundRecorder):
    def __init__(self, out):
        self._archive_local = threading.local()
        self._callback_condition = threading.Condition()
        self._callback_inflight = 0
        self._accepting_callbacks = True
        self._event_lock = threading.Lock()
        self._receipt_samples = {}
        self._receipt_maxima = {}
        self.board_quality = {}
        self.board_tf = {}
        self.board_snapshots = {}
        super().__init__(out)
        self.subs.extend([
            rospy.Subscriber('/charuco/quality', String, self.callback('/charuco/quality'), queue_size=16),
            rospy.Subscriber('/alicia_d/protection_latched', Bool, self.callback('/alicia_d/protection_latched'), queue_size=1),
            rospy.Subscriber('/demonstration', Bool, self.callback('/demonstration'), queue_size=1),
        ])
        register(self)

    def start_archive_writer(self):
        # The inherited constructor opened a fresh, still-empty bag before any
        # subscriptions. Close that descriptor; one spawned process now owns
        # compression/chunk writes, so native archive work cannot hold this
        # process's GIL while feedback/image callbacks need to run.
        from process_bag_sink import ProcessBagSink
        self.bag.close()
        self.bag = None
        self.process_sink = ProcessBagSink(self.out/'samples.bag')
        def failure(reason):
            with self.lock:
                self.fatal = 'diagnostic archive: '+str(reason)
        self.archive_writer = BoundedAsyncBagWriter(self.process_sink.write, failure,
                                                   maximum_bytes=32*1024*1024, maximum_items=512)

    def archive_message(self, topic, msg, stamp):
        if getattr(self._archive_local, 'skip_receive_archive', False):
            return
        return super().archive_message(topic, msg, stamp)

    def callback(self, topic):
        def cb(msg):
            receipt = rospy.Time.now()
            with self._callback_condition:
                if not self._accepting_callbacks:
                    return
                self._callback_inflight += 1
            try:
                # Preserve admission-before-use, but move serialization and IPC
                # queue admission out of the state RLock. The old receive chain
                # then performs its exact state/provenance checks only once.
                self.archive_message(topic, msg, receipt)
                with self.lock:
                    self._archive_local.skip_receive_archive = True
                    self._archive_local.receipt_ns = receipt.to_nsec()
                    try:
                        self.receive(topic, msg)
                    finally:
                        self._archive_local.skip_receive_archive = False
                        self._archive_local.receipt_ns = None
            except Exception as error:
                with self.lock:
                    self.fatal = 'recording callback failed: '+str(error)
            finally:
                with self._callback_condition:
                    self._callback_inflight -= 1
                    self._callback_condition.notify_all()
        return cb

    def mark(self, event, **values):
        row = dict(event=event, time_ns=rospy.Time.now().to_nsec(), **values)
        payload = json.dumps(row)
        # Large geometry proofs must not hold the feedback/image state lock
        # during JSON encoding, file I/O, or event archive serialization.
        with self._event_lock:
            with (self.out/'events.jsonl').open('a') as stream:
                stream.write(payload+'\n')
            self.archive_message('/support_diagnostic/event',String(data=payload),rospy.Time(*divmod(row['time_ns'],10**9)))
        print(json.dumps({k:v for k,v in row.items() if k != 'audit'}),flush=True)

    def close(self):
        with self._callback_condition:
            self._accepting_callbacks = False
        for subscriber in self.subs:
            subscriber.unregister()
        deadline = time.monotonic()+5.
        with self._callback_condition:
            while self._callback_inflight and time.monotonic() < deadline:
                self._callback_condition.wait(min(.1,max(0.,deadline-time.monotonic())))
            if self._callback_inflight:
                self.archive_writer.latch_failure('board callback drain deadline exhausted')
        try:
            super().close()
        finally:
            try:
                sink = getattr(self,'process_sink',None)
                if sink is not None:
                    try:
                        summary = sink.close(timeout=30.)
                        base.write(self.out/'process_archive.json',summary)
                    except BaseException as error:
                        base.write(self.out/'process_archive.json',dict(archive_complete=False,failure=str(error)))
                        count_path = self.out/'counts.json'
                        if count_path.is_file():
                            counts = json.loads(count_path.read_text())
                            counts['archive_writer']['archive_complete'] = False
                            counts['process_archive_close_error'] = str(error)
                            base.write(count_path,counts)
                        raise
            finally:
                # Diagnostic-file I/O must not prevent the child owning the bag
                # from closing, even if the evidence drive reports a write error.
                base.write(self.out/'receipt_latency.json',dict(
                    clock='ROS source header to actual callback entry; not independently calibrated exposure latency',
                    maximum_sec_by_topic=self._receipt_maxima,
                    last_512_samples_sec_by_topic=self._receipt_samples,
                    compression_owner='independent_spawn_process',
                    state_lock_contains_message_serialization=False))

    def receive(self, topic, msg):
        super().receive(topic, msg)
        if not self.active:
            return
        now = getattr(self._archive_local,'receipt_ns',None) or rospy.Time.now().to_nsec()
        self.latest_inputs[topic] = (msg,now)
        header = getattr(msg,'header',None)
        if header is not None and header.stamp.to_nsec() > 0:
            age = (now-header.stamp.to_nsec())*1e-9
            ages = self._receipt_samples.setdefault(topic,[])
            ages.append(age)
            del ages[:-512]
            self._receipt_maxima[topic] = max(age,self._receipt_maxima.get(topic,age))
        if topic == '/charuco/quality':
            quality = json.loads(msg.data)
            ns = quality.get('source_stamp_ns')
            if isinstance(ns, int) and not isinstance(ns, bool):
                self.board_quality[ns] = (quality, now)
                for old in sorted(self.board_quality)[:-48]:
                    del self.board_quality[old]
        if topic == '/tf':
            for transform in msg.transforms:
                if transform.child_frame_id == BOARD_FRAME:
                    self.board_tf[transform.header.stamp.to_nsec()] = (transform, now)
                    for old in sorted(self.board_tf)[:-48]:
                        del self.board_tf[old]

    def snapshot(self, label, after_ns=0):
        end = time.monotonic() + 10.
        while time.monotonic() < end:
            self.guard()
            now = rospy.Time.now().to_nsec()
            with self.lock:
                records = [(ns, q) for ns, q in self.states if ns > after_ns and 0 <= now-ns <= 1000000000]
                pairs = [(ns, pair.copy(), self.board_quality[ns], self.board_tf[ns])
                         for ns, pair in self.frames.items() if len(pair) == 2 and ns > after_ns
                         and 100000000 <= now-ns <= 350000000
                         and ns in self.board_quality and ns in self.board_tf]
                frozen_inputs = self.latest_inputs.copy()
            recent = np.asarray([q[:6] for ns, q in records])
            stable = (len(records) >= 7 and records[-1][0]-records[0][0] >= 600000000
                      and records[-1][0]-records[0][0] >= 700000000 and now-records[-1][0] <= 300000000
                      and np.max(np.ptp(recent, axis=0)) <= math.radians(.15))
            if not stable or not pairs:
                time.sleep(.025)
                continue
            ns, pair, (quality, quality_received), (board_tf, board_received) = max(pairs, key=lambda x: x[0])
            try:
                require_exact_binding(ns, quality, board_tf)
            except RuntimeError:
                time.sleep(.025)
                continue
            if now-quality_received > 350000000 or now-board_received > 350000000:
                time.sleep(.025)
                continue
            before = [r for r in records if r[0] <= ns]
            after = [r for r in records if r[0] >= ns]
            if not before or not after:
                time.sleep(.025)
                continue
            lo, hi = before[-1], after[0]
            near = min((lo, hi), key=lambda r: abs(r[0]-ns))
            if hi[0]-lo[0] > 200000000 or abs(near[0]-ns) > 100000000:
                time.sleep(.025)
                continue
            try:
                tr = self.buffer.lookup_transform('base_link', 'camera_link', rospy.Time(*divmod(ns, 10**9)), rospy.Duration(.15))
            except Exception:
                time.sleep(.025)
                continue
            q, sdk, sdk_stamp = self.joint_and_sdk()
            required = ['/alicia_d/control_reference', '/alicia_d/actuation_status', '/alicia_d/feedback_ready',
                        '/alicia_d/protection_latched', '/gui/joint_direct_mode', '/grasp/state']
            if any(t not in frozen_inputs for t in required) or self.epoch_ns is None:
                time.sleep(.025)
                continue
            ref = frozen_inputs['/alicia_d/control_reference'][0]
            if base.words(sdk) != base.words(base.qmsg(ref)) or sdk[6] != base.qmsg(ref)[6]:
                raise RuntimeError('SDK / reference words disagree')
            if np.max(abs(q[:6]-np.asarray(near[1])[:6])) > math.radians(.15):
                raise RuntimeError('actual feedback changed during source binding')
            camera = rospy.get_param('/camera')
            model_now = current_model_evidence()
            if model_now['camera_model_fingerprint'] != quality['camera_model_fingerprint'] or camera['depth_scale'] != self.camera['depth_scale']:
                raise RuntimeError('camera geometric model changed during evidence capture')
            handeye = rospy.get_param('/handeye')
            X = quaternion_matrix(handeye['rotation_xyzw'])
            X[:3, 3] = handeye['translation_xyz']
            T = transform_matrix(tr.transform)
            B = transform_matrix(board_tf.transform)
            source_q = np.asarray(near[1], float)
            fk = base.SerialUrdfFk(self.xml, base.NAMES[:6])
            error = float(np.max(abs(fk(source_q[:6]) @ X-T)))
            if error > 1e-5:
                # TF interpolation and a non-identical quantized sample cannot
                # masquerade as exact FK evidence. Acquire a new complete source.
                time.sleep(.025)
                continue
            rgb = self.bridge.imgmsg_to_cv2(pair['color'], 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(pair['depth'], 'passthrough')
            path = self.out/(str(ns)+'_rgbd_tf.npz')
            binding = dict(rgb_stamp_ns=ns, depth_stamp_ns=pair['depth'].header.stamp.to_nsec(),
                           board_stamp_ns=ns, quality_stamp_ns=quality['source_stamp_ns'],
                           accepted_before_ns=lo[0], accepted_after_ns=hi[0], accepted_source_stamp_ns=near[0],
                           sdk_stamp_ns=sdk_stamp, reference_stamp_ns=ref.header.stamp.to_nsec(),
                           snapshot_received_ros_ns=now, quality_received_ros_ns=quality_received,
                           board_received_ros_ns=board_received,
                           accepted_window_span_deg=np.degrees(np.ptp(recent,axis=0)).tolist(),
                           accepted_window_duration_sec=(records[-1][0]-records[0][0])*1e-9,
                           image_timestamp_semantics='host_processed_read_completion_not_exposure')
            snapshot = dict(kind='RGB_PnP_board_observation_only', source_stamp_ns=ns,
                            accepted_source_stamp_ns=near[0], snapshot_received_ros_ns=now,
                            q_rad=source_q.tolist(), sdk_q_rad=sdk.tolist(), reference_q_rad=base.qmsg(ref).tolist(),
                            sdk_frame_id='sdk_transmitted', reference_frame_id=ref.header.frame_id,
                            epoch_ns=self.epoch_ns, actuation_status=frozen_inputs['/alicia_d/actuation_status'][0].data,
                            feedback_ready=frozen_inputs['/alicia_d/feedback_ready'][0].data,
                            protection_latched=frozen_inputs['/alicia_d/protection_latched'][0].data,
                            gui_direct_mode=frozen_inputs['/gui/joint_direct_mode'][0].data,
                            task_active=frozen_inputs['/grasp/state'][0].active,
                            accepted_window_span_deg=binding['accepted_window_span_deg'],
                            T_base_camera=T.tolist(), T_tool_camera=X.tolist(), T_camera_board=B.tolist(),
                            T_base_board=(T@B).tolist(), board_quality=copy.deepcopy(quality),
                            camera_model_evidence=copy.deepcopy(quality['camera_model']),
                            camera_model_fingerprint=quality['camera_model_fingerprint'],
                            K=[camera[k] for k in ('fx','fy','cx','cy')], width=rgb.shape[1], height=rgb.shape[0],
                            depth_scale=camera['depth_scale'], joint_min=rospy.get_param('/robot/joint_min_rad'),
                            joint_max=rospy.get_param('/robot/joint_max_rad'),
                            openings=sorted(set([float(source_q[6]),float(sdk[6])])),
                            urdf_sha256=hashlib.sha256(self.xml.encode()).hexdigest(), fk_tf_max_abs_error=error,
                            source_binding=binding, scene_contract={
                                'kind':'fixed_rigid_board_laid_flat_on_same_local_tabletop',
                                'board_printed_plane_is_local_table_upper_bound':True,
                                'evidence':'current same-source RGB shows flat rigid board on local mousemat tabletop; user confirms fixed board',
                                'old_board_thickness_reused':False,
                                'scope':'small bounded board-only diagnostic, not grasp authority'})
            board_spec = {k:copy.deepcopy(quality[k]) for k in ('board_size','dictionary_id','square_length_m','marker_length_m','detection_scale')}
            board_spec['legacy_pattern'] = True  # Explicit legacy board in current tracker source.
            if hasattr(self,'frozen_board_spec') and (self.frozen_board_spec != board_spec or self.frozen_model_fingerprint != quality['camera_model_fingerprint']):
                raise RuntimeError('RGB board / camera model changed during experiment')
            self.frozen_board_spec = board_spec
            self.frozen_model_fingerprint = quality['camera_model_fingerprint']
            base.write(self.out/'board_spec.json',board_spec)
            np.savez_compressed(str(path), image_bgr=rgb, depth_raw=depth, depth_scale=camera['depth_scale'],
                                K=self.K, T_base_ros_camera=T, T_base_optical=T@C,
                                T_camera_board=B, T_base_board=T@B, T_tool_camera=X,
                                source_stamp_ns=np.int64(ns), q_source_before=np.asarray(lo[1]),
                                q_source_after=np.asarray(hi[1]), q_rad=source_q, sdk_q_rad=sdk,
                                board_quality_json=np.asarray(json.dumps(quality)),
                                source_binding_json=np.asarray(json.dumps(binding)))
            cv2.imwrite(str(self.out/(str(ns)+'_rgb.png')), rgb)
            base.write(self.out/(str(ns)+'_board_binding.json'), snapshot)
            self.board_snapshots[ns] = snapshot
            self.mark('stationary_snapshot', label=label, source_stamp_ns=ns, q_source_before=lo,
                      q_source_after=hi, span_deg=binding['accepted_window_span_deg'], file=str(path),
                      charuco_count=quality['charuco_count'], board_rgb_reprojection_error_px=quality['reprojection_rms_px'])
            return ns
        raise RuntimeError('No exact-header stationary RGB board / RGB-D / TF / q source window')


def capture(directory):
    if _recorder is None:
        raise RuntimeError('registered current board recorder required')
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    ns = _recorder.snapshot('factory_board_source')
    snapshot = copy.deepcopy(_recorder.board_snapshots[ns])
    raw = _recorder.out/(str(ns)+'_rgbd_tf.npz')
    (directory/'source.npz').write_bytes(raw.read_bytes())
    (directory/'source_rgb.png').write_bytes((_recorder.out/(str(ns)+'_rgb.png')).read_bytes())
    base.write(directory/'source_board_binding.json',snapshot['source_binding'])
    base.write(directory/'snapshot.json',snapshot)
    base.write(directory/'camera_parameters.json',_recorder.camera)
    (directory/'robot_description.urdf').write_text(_recorder.xml)
    return snapshot,_recorder.xml
