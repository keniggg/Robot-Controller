#!/usr/bin/env python3
import copy
from collections import deque
import json
import math
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, JointState
try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None
from alicia_flexible_grasp.vision.realsense_manager import RealSenseManager


def _compact_read_evidence(diagnostic):
    """Detached bounded metadata only; never inspect or copy image arrays."""
    if not isinstance(diagnostic, dict) or diagnostic.get('result') != 'complete':
        return dict(available=False, reason='complete_read_diagnostic_unavailable')
    def sources(raw):
        keys = ('present', 'frame_number', 'device_timestamp_ms', 'timestamp_domain',
                'stream_fps', 'metadata_raw', 'unavailable')
        return {stream: {key: copy.deepcopy(row[key]) for key in keys if key in row}
                for stream, row in (raw or {}).items()
                if stream in ('color', 'depth') and isinstance(row, dict)}
    keys = ('schema', 'result', 'failed_stage', 'total_sec', 'host_wait', 'pair_timing',
            'stages_sec', 'sdk_color_alignment_applied',
            'stationary_pair_filter_host_monotonic', 'regional_stationary_evidence')
    result = {key: copy.deepcopy(diagnostic[key]) for key in keys if key in diagnostic}
    result.update(available=True, raw_frames=sources(diagnostic.get('raw_frames')))
    history = diagnostic.get('stationary_pair_history')
    if isinstance(history, dict):
        raw_keys = ('last_complete_raw_frames', 'rejected_raw_frames',
                    'latest_rejected_raw_frames', 'next_accepted_raw_frames')
        result['stationary_pair_history'] = {
            key: sources(value) if key in raw_keys and value is not None else copy.deepcopy(value)
            for key, value in history.items()}
    return result

class CameraNode:
    # Diagnostic only: this is not a frame freshness or recovery threshold.
    SLOW_CYCLE_WARN_SEC = 0.2
    RECENT_SUCCESSFUL_READ_CAPACITY = 32

    def __init__(self):
        self.cfg = copy.deepcopy(rospy.get_param('/camera', {}))
        self.color_topic = self.cfg.get('color_topic', '/supervisor/camera/color/image_raw')
        self.depth_topic = self.cfg.get('depth_topic', '/supervisor/camera/depth/image_raw')
        self.depth_preview_topic = str(self.cfg.get('depth_preview_topic', '') or '')
        self.depth_preview_width = max(1, int(self.cfg.get('depth_preview_width', 320)))
        self.depth_preview_height = max(1, int(self.cfg.get('depth_preview_height', 240)))
        self.frame_id = self.cfg.get('frame_id', 'camera_color_optical_frame')
        self.width = self.cfg.get('width', 640)
        self.height = self.cfg.get('height', 480)
        # Published rectified dimensions never become a recovered SDK request.
        self.stream_width = self.cfg.get('stream_width', self.width)
        self.stream_height = self.cfg.get('stream_height', self.height)
        self.fps = self.cfg.get('fps', 30)
        self.publish_fps = min(
            float(self.fps),
            max(0.1, float(self.cfg.get('publish_fps', self.fps))),
        )
        self._publish_period_sec = 1.0 / self.publish_fps
        self._last_publish_monotonic = None
        self._monotonic = time.monotonic
        self._diagnostic_clock = time.monotonic
        self._async_publication_enabled = bool(self.cfg.get('async_publication_enabled', True))
        self._publication_worker = None
        self._runtime_status_timer = None
        self._runtime_params_lock = threading.Lock()
        self._publication_lifecycle_lock = threading.Lock()
        self._recent_read_lock = threading.Lock()
        self._recent_successful_reads = deque(maxlen=self.RECENT_SUCCESSFUL_READ_CAPACITY)
        self._recent_read_generation = 0
        self._recent_read_copy_errors = 0
        self._last_recent_read_parameters = None
        self._shutdown_requested = False
        self._runtime_geometry_ready = False
        self.align_depth_to_color = self.cfg.get('align_depth_to_color', True)
        self.fallback_to_simulation = bool(self.cfg.get('fallback_to_simulation', False))
        self.read_failure_limit = max(1, int(self.cfg.get('read_failure_limit', 3)))
        self._read_failures = 0
        self.bridge = CvBridge() if CvBridge else None
        self.pub_color = rospy.Publisher(self.color_topic, Image, queue_size=2)
        self.pub_depth = rospy.Publisher(self.depth_topic, Image, queue_size=2)
        self.pub_depth_preview = (
            rospy.Publisher(self.depth_preview_topic, Image, queue_size=1)
            if self.depth_preview_topic else None
        )
        simulate = bool(self.cfg.get('simulate', False))
        self.cam = None
        self._stationary_window = None
        self._last_temporal_status = None
        if self.cfg.get('mode_depth_preset', {}).get('stationary_temporal_enabled', False):
            from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
            self._stationary_window = StationaryDepthWindow()
            self._stationary_sdk_sub = rospy.Subscriber(
                '/alicia_d/sdk_command', JointState,
                lambda msg: self._stationary_window.observe('sdk', msg), queue_size=10)
            self._stationary_encoder_sub = rospy.Subscriber(
                '/alicia_d/accepted_joint_states', JointState,
                lambda msg: self._stationary_window.observe('accepted', msg), queue_size=10)
        self._camera_started = False
        self._start_camera_or_defer(simulate)
        if self._async_publication_enabled:
            self._runtime_status_timer = rospy.Timer(rospy.Duration(.5), self._publish_temporal_status)
        self.rate = rospy.Rate(float(self.fps))

    def _make_camera(self, simulate):
        depth_filter_cfg = dict(self.cfg.get('depth_filter', {}) or {})
        perception_cfg = rospy.get_param('/perception', {})
        depth_filter_cfg.setdefault('depth_min_m', perception_cfg.get('depth_min_m', 0.03))
        depth_filter_cfg.setdefault('depth_max_m', perception_cfg.get('depth_max_m', 2.0))
        camera_args = dict(
            width=self.stream_width,
            height=self.stream_height,
            fps=self.fps,
            align_depth_to_color=self.align_depth_to_color,
            simulate=simulate,
            depth_filter_cfg=depth_filter_cfg,
        )
        projection_cfg = dict(self.cfg.get('color_projection_correction', {}) or {})
        if projection_cfg.get('enabled', False):
            target = projection_cfg['sdk_intrinsics']
            for key in ('fx', 'fy', 'cx', 'cy'):
                if not np.isclose(float(self.cfg[key]), float(target[key]), rtol=0, atol=1e-6):
                    raise ValueError('camera/%s must match the corrected RGB-D SDK projection' % key)
            camera_args['color_projection_cfg'] = projection_cfg
        rectification_cfg = dict(self.cfg.get('pair_rectification', {}) or {})
        if rectification_cfg:
            camera_args['pair_rectification_cfg'] = rectification_cfg
        preset_cfg = dict(self.cfg.get('mode_depth_preset', {}) or {})
        if preset_cfg.get('enabled', False):
            if simulate:
                raise ValueError('mode depth preset requires a real camera')
            camera_args['mode_depth_preset_cfg'] = preset_cfg
        return RealSenseManager(**camera_args)

    def _start_camera(self, simulate):
        self._raise_if_shutdown()
        # An already admitted publisher cannot be cancelled. Finish it before
        # replacing camera geometry; otherwise old pixels could arrive under K
        # and scale belonging to the recovered stream.
        self._quiesce_publication_for_geometry()
        self._raise_if_shutdown()
        self.cam = self._make_camera(simulate)
        self.cam.publication_diagnostics_provider = self._publication_diagnostics
        try:
            self.cam.start()
            self._raise_if_shutdown()
            if getattr(self, '_stationary_window', None) is not None:
                self.cam.stationary_depth_provider = lambda: self._stationary_window.stationary(rospy.get_time())
            self._sync_depth_preset(publish=False)
            self._publish_runtime_camera_params()
            with self._publication_lifecycle_lock:
                self._raise_if_shutdown()
                self._camera_started = True
        except BaseException:
            # start() can return after shutdown already called stop(). Clean
            # up again so a late SDK start cannot leave an unowned stream.
            self._stop_camera()
            raise
        mode = 'simulated' if simulate else 'real'
        rospy.loginfo('Camera started (%s): color=%s depth=%s', mode, self.color_topic, self.depth_topic)
        return True

    def _start_camera_or_defer(self, simulate):
        if self._shutdown_requested or rospy.is_shutdown():
            return False
        try:
            return self._start_camera(simulate)
        except Exception as exc:
            self._camera_started = False
            self._stop_camera()
            if self._shutdown_requested or rospy.is_shutdown():
                return False
            if self.fallback_to_simulation and not simulate:
                rospy.logerr('Camera start failed: %s. Falling back to simulated camera.', exc)
                return self._start_camera(True)
            rospy.logerr(
                'Camera start failed: %s. Node stays alive and will retry real camera; '
                'check for stale camera_node/RealSense viewers if the device is busy.',
                exc,
            )
            return False

    def _stop_camera(self):
        self._clear_recent_successful_reads()
        worker = getattr(self, '_publication_worker', None)
        if worker is not None:
            worker.discard_pending()
        if self.cam is None:
            return
        try:
            self.cam.stop()
        except Exception as exc:
            rospy.logwarn('Camera stop failed during recovery: %s', exc)
        finally:
            self._camera_started = False

    def request_shutdown(self):
        # rospy can invoke this hook from Python's signal handler while this
        # same thread is inside SDK read/filter/start or holding a lifecycle
        # lock. Do not re-enter SDK stop, join workers, or acquire locks here.
        # main() performs synchronous cleanup after spin has unwound.
        self._shutdown_requested = True

    def shutdown(self):
        with self._publication_lifecycle_lock:
            self._shutdown_requested = True
            self._runtime_geometry_ready = False
            worker = self._publication_worker
        timer = getattr(self, '_runtime_status_timer', None)
        if timer is not None:
            timer.shutdown()
        if worker is not None:
            result = worker.close(timeout=1.)
            if result.get('worker_alive'):
                rospy.logwarn('Camera publication worker remains blocked at bounded shutdown')
        self._stop_camera()

    def _publication_diagnostics(self):
        worker = getattr(self, '_publication_worker', None)
        return None if worker is None else worker.summary()

    def _raise_if_shutdown(self):
        if self._shutdown_requested or rospy.is_shutdown():
            raise RuntimeError('camera node is shutting down')

    def _quiesce_publication_for_geometry(self):
        self._runtime_geometry_ready = False
        self._clear_recent_successful_reads()
        worker = self._publication_worker
        if worker is None:
            return
        result = worker.close(timeout=1.)
        if result.get('worker_alive'):
            raise RuntimeError('old RGB-D publication still in flight; camera geometry unchanged')
        self._publication_worker = None

    def _publish_snapshot(self, snapshot):
        # The source clock is captured by the read thread, before the handoff.
        # It is host read completion, NOT a calibrated exposure timestamp.
        stamp = rospy.Time(*divmod(snapshot.host_read_completed_stamp_ns, 10**9))
        started = self._diagnostic_clock()
        self.publish_image(self.pub_color, snapshot.color, 'bgr8', stamp)
        self.publish_image(self.pub_depth, snapshot.depth, '16UC1', stamp)
        self.publish_depth_preview(snapshot.depth, stamp)
        self._report_slow_cycle(started, {'image_publish': self._diagnostic_clock()-started}, 'async_published')

    def _clear_recent_successful_reads(self):
        with self._recent_read_lock:
            self._recent_successful_reads.clear()
            self._recent_read_generation += 1

    def _remember_successful_read(self, source_stamp_ns, read_completed_monotonic,
                                  *, camera=None, geometry_generation=None):
        # Diagnostics must not turn a successful SDK read into recovery or
        # delay it behind XMLRPC. Entries are immutable after this append.
        try:
            camera = self.cam if camera is None else camera
            if not isinstance(getattr(camera, 'runtime_profile', {}).get('stationary_temporal'), dict):
                return
            if (type(source_stamp_ns) is not int or source_stamp_ns <= 0
                    or not math.isfinite(read_completed_monotonic)):
                raise ValueError('invalid diagnostic source clock')
            with self._recent_read_lock:
                generation = (self._recent_read_generation if geometry_generation is None
                              else geometry_generation)
                if generation != self._recent_read_generation or camera is not self.cam:
                    self._recent_read_copy_errors += 1
                    return
            row = dict(source_stamp_ns=source_stamp_ns,
                read_completed_monotonic=read_completed_monotonic,
                geometry_generation=generation, frame_id=self.frame_id,
                geometry_activation_stamp_ns=self.cfg.get('runtime_profile', {}).get('geometry_activation_stamp_ns'),
                image_header_clock='host_read_completion', exposure_clock_calibrated=False,
                publication_outcome_unknown=True,
                evidence=_compact_read_evidence(getattr(camera, 'last_read_diagnostics', None)))
            with self._recent_read_lock:
                if generation != self._recent_read_generation or camera is not self.cam:
                    self._recent_read_copy_errors += 1
                    return
                self._recent_successful_reads.append(row)
        except Exception:
            with self._recent_read_lock:
                self._recent_read_copy_errors += 1

    def _recent_successful_read_parameters(self):
        with self._recent_read_lock:
            # A list of immutable row references is enough under the lock.
            # Deep conversion below never blocks the read thread on XMLRPC.
            entries = list(self._recent_successful_reads)
            generation, errors = self._recent_read_generation, self._recent_read_copy_errors
        payload = dict(schema=1,
            capacity=self.RECENT_SUCCESSFUL_READ_CAPACITY,
            geometry_generation=generation, copy_errors=errors,
            frame_id=self.frame_id,
            geometry_activation_stamp_ns=self.cfg.get('runtime_profile', {}).get('geometry_activation_stamp_ns'),
            diagnostic_only=True, motion_authorized=False,
            scope='successful read candidates; match exact source stamp to a received RGB-D pair; publication is not guaranteed',
            entries=entries)
        # A single compact JSON string avoids thousands of XMLRPC members and
        # exactly preserves SDK integers/nulls. Consumers decode it once for
        # diagnosis; geometry and control never consult this parameter.
        return dict(schema=1, encoding='json',
            payload_json=json.dumps(payload,ensure_ascii=False,allow_nan=False,separators=(',',':')))

    def _publish_temporal_status(self, _event):
        # Informational metadata only. The live filter gate runs in read(),
        # never consults this delayed parameter and never waits for XMLRPC.
        try:
            with self._runtime_params_lock:
                if self._shutdown_requested or not self._runtime_geometry_ready:
                    return
                status = copy.deepcopy(getattr(self.cam, 'runtime_profile', {}).get('stationary_temporal'))
                if status is None:
                    return
                if status != self._last_temporal_status:
                    rospy.set_param('/camera/runtime_profile/stationary_temporal', status)
                    self._last_temporal_status = status
                # Unknown perception reads the /camera tree for every frame.
                # Keep this bounded history OUTSIDE that execution namespace.
                recent = self._recent_successful_read_parameters()
                if recent != self._last_recent_read_parameters:
                    rospy.set_param('/camera_read_diagnostics', recent)
                    self._last_recent_read_parameters = recent
        except Exception as exc:
            rospy.logwarn_throttle(2., 'Camera temporal-status diagnostic update failed: %s', exc)

    def _recover_from_read_error(self, exc):
        if self._shutdown_requested or rospy.is_shutdown():
            return False
        self._read_failures += 1
        rospy.logwarn_throttle(
            2.0,
            'Camera read failed (%d/%d): %s',
            self._read_failures,
            self.read_failure_limit,
            exc
        )
        if self._read_failures < self.read_failure_limit:
            return False
        self._stop_camera()
        try:
            self._start_camera(bool(self.fallback_to_simulation))
            self._read_failures = 0
            if self.fallback_to_simulation:
                rospy.logwarn('Camera stream switched to simulated fallback after read failures.')
            else:
                rospy.logwarn('Camera stream restarted after read failures.')
            return True
        except Exception as restart_exc:
            if self._shutdown_requested or rospy.is_shutdown():
                return False
            rospy.logerr(
                'Camera restart failed after read error: %s. Keeping real camera mode; no simulated frames will be published.',
                restart_exc
            )
            self._read_failures = self.read_failure_limit
            return False

    def publish_image(self, pub, cv_img, encoding, stamp):
        if self.bridge is None:
            return
        msg = self.bridge.cv2_to_imgmsg(cv_img, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        pub.publish(msg)

    @staticmethod
    def _resize_depth_preview(depth, width, height):
        array = np.asanyarray(depth)
        source_height, source_width = array.shape[:2]
        width = max(1, int(width))
        height = max(1, int(height))
        if (source_width, source_height) == (width, height):
            return array
        x_indices = np.linspace(
            0,
            source_width - 1,
            width,
        ).astype(np.intp)
        y_indices = np.linspace(
            0,
            source_height - 1,
            height,
        ).astype(np.intp)
        return array[np.ix_(y_indices, x_indices)]

    def publish_depth_preview(self, depth, stamp):
        pub = self.pub_depth_preview
        if pub is None:
            return
        connection_count = getattr(pub, 'get_num_connections', lambda: 1)()
        if int(connection_count) <= 0:
            return
        preview = self._resize_depth_preview(
            depth,
            self.depth_preview_width,
            self.depth_preview_height,
        )
        self.publish_image(pub, preview, '16UC1', stamp)

    def _publish_runtime_camera_params(self):
        self._raise_if_shutdown()
        self._quiesce_publication_for_geometry()
        with self._runtime_params_lock:
            self._raise_if_shutdown()
            self._publish_runtime_camera_params_locked()
        # Serialize only worker creation/ownership with shutdown. Neither SDK
        # operations nor XMLRPC hold this short lifecycle lock.
        with self._publication_lifecycle_lock:
            self._raise_if_shutdown()
            if self._async_publication_enabled:
                from alicia_flexible_grasp.vision.latest_rgbd_publication import LatestRGBDPublication
                self._publication_worker = LatestRGBDPublication(
                    self._publish_snapshot, monotonic=self._monotonic)
            self._runtime_geometry_ready = True

    def _publish_runtime_camera_params_locked(self):
        # One namespace replacement publishes K and scale together before the
        # first image, including after a stream recovery. Keep unrelated config.
        current = copy.deepcopy(rospy.get_param('/camera', self.cfg))
        if hasattr(self.cam, 'depth_scale'):
            current['depth_scale'] = float(self.cam.depth_scale)
        if bool(getattr(self.cam, 'simulate', False)):
            current['runtime_profile'] = {'source': 'simulation', 'schema_version': 1}
            current['intrinsics_source'] = 'configured_simulation'
        else:
            profile = copy.deepcopy(self.cam.runtime_profile)
            if profile.get('source') != 'realsense_active_profile':
                raise ValueError('real camera has no active SDK geometry profile')
            intrinsic = profile['geometry_intrinsics']
            for key in ('fx', 'fy', 'cx', 'cy'):
                current[key] = float(intrinsic[key])
            rectification = profile.get('rectification')
            if rectification is not None:
                from alicia_flexible_grasp.vision.rgbd_rectification import validate_rectification_contract
                validate_rectification_contract(rectification, output_intrinsics=intrinsic)
                source = rectification['source_intrinsics']
                if (int(source['width']) != int(self.stream_width)
                        or int(source['height']) != int(self.stream_height)
                        or source != profile['color_intrinsics']):
                    raise ValueError('rectified SDK source dimensions differ from requested stream')
                current['stream_width'], current['stream_height'] = int(self.stream_width), int(self.stream_height)
                current['width'], current['height'] = int(intrinsic['width']), int(intrinsic['height'])
                self.width, self.height = current['width'], current['height']
                current['model'], current['coeffs'] = 'none', [0.]*5
                current['distortion_model'], current['distortion_coefficients'] = 'none', [0.]*5
            elif (int(intrinsic['width']) != int(self.stream_width)
                    or int(intrinsic['height']) != int(self.stream_height)):
                raise ValueError('active SDK geometry dimensions differ from requested images')
            elif 'stream_width' in current or 'stream_height' in current:
                current['width'], current['height'] = int(intrinsic['width']), int(intrinsic['height'])
                self.width, self.height = current['width'], current['height']
                current['model'] = intrinsic.get('model', 'none')
                current['coeffs'] = copy.deepcopy(intrinsic.get('coeffs', [0.]*5))
                current['distortion_model'], current['distortion_coefficients'] = current['model'], current['coeffs']
            # Keep the boundary when returning to raw geometry too. A preset
            # telemetry update reuses it; each new camera owner or map changes it.
            if (rectification is not None or current.get('runtime_profile', {}).get('geometry_activation_stamp_ns') is not None):
                geometry_key = (rectification['digest'] if rectification is not None else
                                json.dumps(intrinsic, sort_keys=True, allow_nan=False))
                key = (self.cam, geometry_key, current.get('depth_scale'))
                retained = getattr(self, '_rectified_geometry_activation', None)
                if (retained is not None and retained[0][0] is self.cam
                        and retained[0][1:] == key[1:]):
                    activation = retained[1]
                else:
                    activation = rospy.Time.now().to_nsec()
                    if type(activation) is not int or activation <= 0:
                        raise ValueError('camera geometry requires a positive activation source clock')
                    self._rectified_geometry_activation = (key, activation)
                # ROS XMLRPC parameter integers are signed 32-bit. Preserve
                # exact nanoseconds as the existing canonical decimal wire form.
                profile['geometry_activation_stamp_ns'] = str(activation)
            current['runtime_profile'] = profile
            current['intrinsics_source'] = profile['geometry_intrinsics_source']
        current['runtime_profile']['publication'] = dict(
            async_enabled=self._async_publication_enabled,
            max_pending_pairs=1 if self._async_publication_enabled else 0,
            max_host_read_to_publication_start_sec=.15 if self._async_publication_enabled else None,
            image_header_clock='host_read_completion' if self._async_publication_enabled else 'host_publication',
            exposure_clock_calibrated=False,
            temporal_status_is_diagnostic_only=True,
            temporal_status_max_update_hz=2. if self._async_publication_enabled else None)
        rospy.set_param('/camera', current)
        self.cfg = copy.deepcopy(current)
        self._last_temporal_status = copy.deepcopy(current['runtime_profile'].get('stationary_temporal'))
        rospy.loginfo('Camera runtime geometry: source=%s fx=%.6f fy=%.6f depth_scale=%.7f m/unit',
                      current['intrinsics_source'], float(current.get('fx', 0.)),
                      float(current.get('fy', 0.)), float(current.get('depth_scale', 0.)))

    def _sync_depth_preset(self, publish=True, timings=None):
        if not self.cfg.get('mode_depth_preset', {}).get('enabled', False):
            return
        started = self._diagnostic_clock() if timings is not None else None
        try:
            # The first call subscribes with the master; later calls read the
            # cache maintained by paramUpdate, including actual mode changes.
            selection = rospy.get_param_cached('/grasp_mode/selection', {'mode': 'carton'})
            changed = self.cam.set_depth_preset_mode(selection.get('mode'))
        finally:
            if timings is not None:
                timings['sync_mode'] = self._diagnostic_clock() - started
        if changed and publish:
            self._timed_stage(timings, 'runtime_params', self._publish_runtime_camera_params)

    def _timed_stage(self, timings, stage, callback):
        if timings is None:
            return callback()
        started = self._diagnostic_clock()
        try:
            return callback()
        finally:
            timings[stage] = timings.get(stage, 0.) + self._diagnostic_clock() - started

    def _report_slow_cycle(self, started, timings, result):
        elapsed = self._diagnostic_clock() - started
        if elapsed <= self.SLOW_CYCLE_WARN_SEC:
            return
        rospy.logwarn(
            'Camera slow cycle: total=%.6fs sync_mode=%.6fs read=%.6fs '
            'runtime_params=%.6fs image_publish=%.6fs result=%s',
            elapsed, timings.get('sync_mode', 0.), timings.get('read', 0.),
            timings.get('runtime_params', 0.), timings.get('image_publish', 0.), result)
        if 'read' in timings:
            details = getattr(self.cam, 'last_read_diagnostics', None)
            if details is not None:
                try:
                    serialized = json.dumps(details, sort_keys=True, allow_nan=False)
                except Exception:
                    # An optional diagnostic must not alter publication or
                    # hide the original read exception/recovery decision.
                    return
                rospy.logwarn('Camera backend read stages: %s', serialized)

    def _publication_is_due(self):
        now = self._monotonic()
        previous = self._last_publish_monotonic
        if previous is None or now < previous:
            self._last_publish_monotonic = now
            return True

        # Keep the ideal publication clock instead of rebasing it to the
        # hardware-frame timestamp. Rebasing quantizes a 30 FPS camera with a
        # 20 FPS publication target to every second frame (about 15 FPS).
        # Advancing the scheduled clock produces the intended alternating
        # one-frame/two-frame cadence without publishing bursts after a stall.
        scheduled = previous + self._publish_period_sec
        epsilon = min(1e-6, self._publish_period_sec * 1e-3)
        if now + epsilon < scheduled:
            return False
        elapsed_periods = max(
            0,
            int((now - scheduled + epsilon) / self._publish_period_sec),
        )
        self._last_publish_monotonic = (
            scheduled + elapsed_periods * self._publish_period_sec
        )
        return True

    def _pace_simulated_camera(self):
        if bool(getattr(self.cam, 'simulate', False)):
            self.rate.sleep()

    def spin(self):
        while not self._shutdown_requested and not rospy.is_shutdown():
            if not self._camera_started:
                if not self._start_camera_or_defer(False):
                    self.rate.sleep()
                    continue
            cycle_started = self._diagnostic_clock()
            timings = {}
            try:
                if not self._runtime_geometry_ready:
                    self._timed_stage(timings, 'runtime_params', self._publish_runtime_camera_params)
                self._sync_depth_preset(timings=timings)
                read_camera = self.cam
                with self._recent_read_lock:
                    read_geometry_generation = self._recent_read_generation
                color, depth = self._timed_stage(timings, 'read', read_camera.read)
                read_completed_monotonic = self._monotonic() if self._async_publication_enabled else None
                read_completed_stamp = rospy.Time.now() if self._async_publication_enabled and color is not None and depth is not None else None
                if read_completed_stamp is not None:
                    self._remember_successful_read(read_completed_stamp.to_nsec(), read_completed_monotonic,
                        camera=read_camera, geometry_generation=read_geometry_generation)
                if color is None or depth is None:
                    self._read_failures = 0
                    details = getattr(self.cam, 'last_read_diagnostics', None)
                    # Preserve rejection evidence even for a fast SDK read.
                    # Throttling bounds log I/O during a run of invalid pairs.
                    try:
                        evidence = json.dumps(details, sort_keys=True, allow_nan=False)
                    except (TypeError, ValueError):
                        evidence = '{"result": "pair_unavailable", "diagnostic_serialization": "failed"}'
                    rospy.logwarn_throttle(2.0, 'Camera RGB-D pair unavailable: %s', evidence)
                    self._report_slow_cycle(cycle_started, timings, 'pair_unavailable')
                    self._pace_simulated_camera()
                    continue
                temporal_status = getattr(self.cam, 'runtime_profile', {}).get('stationary_temporal')
                if not self._async_publication_enabled and temporal_status != getattr(self, '_last_temporal_status', None):
                    self._timed_stage(timings, 'runtime_params', self._publish_runtime_camera_params)
                    self._last_temporal_status = copy.deepcopy(temporal_status)
                self._read_failures = 0
            except Exception as exc:
                self._report_slow_cycle(cycle_started, timings, 'read_or_sync_error')
                recovered = self._recover_from_read_error(exc)
                if not recovered:
                    self.rate.sleep()
                continue
            if not self._publication_is_due():
                self._report_slow_cycle(cycle_started, timings, 'publication_not_due')
                self._pace_simulated_camera()
                continue
            publish_started = self._diagnostic_clock()
            if self._async_publication_enabled:
                accepted = self._publication_worker.submit(
                    color, depth, read_completed_monotonic, read_completed_stamp.to_nsec())
                if not accepted:
                    summary = self._publication_worker.summary()
                    if summary.get('failure'):
                        rospy.logerr_throttle(2., 'Camera publication worker failed: %s', summary['failure'])
                timings['publication_handoff'] = self._diagnostic_clock()-publish_started
                self._report_slow_cycle(cycle_started, timings, 'publication_handoff')
                self._pace_simulated_camera()
                continue
            # This remains a host publication stamp, not an exposure stamp.
            # RealSense pair validity is checked before read() returns pixels;
            # global_time is not calibrated here into a ROS source clock.
            stamp = rospy.Time.now()
            if color is not None:
                self.publish_image(self.pub_color, color, 'bgr8', stamp)
            if depth is not None:
                self.publish_image(self.pub_depth, depth, '16UC1', stamp)
                self.publish_depth_preview(depth, stamp)
            timings['image_publish'] = self._diagnostic_clock() - publish_started
            self._report_slow_cycle(cycle_started, timings, 'published')
            # RealSense wait_for_frames() already blocks until the next hardware
            # frame. Sleeping again here halves the effective camera rate.
            # Simulated reads return immediately, so they still need pacing.
            self._pace_simulated_camera()

def main():
    # Small regional image operations do not benefit from a CPU-sized pool.
    # Bound this process before camera acquisition or ROS callbacks can start.
    cv2.setNumThreads(1)
    rospy.init_node('camera_node')
    rospy.loginfo('Camera OpenCV worker threads: %s', cv2.getNumThreads())
    node = CameraNode()
    try:
        rospy.on_shutdown(node.request_shutdown)
        node.spin()
    finally:
        # Includes ordinary return, ROS interruption and unexpected failures.
        # Explicit callers of node.shutdown() retain synchronous cleanup.
        node.shutdown()


if __name__ == '__main__':
    main()
