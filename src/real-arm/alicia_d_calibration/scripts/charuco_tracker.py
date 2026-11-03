#!/usr/bin/env python3
"""
ChArUco Tracker Node for Eye-on-Hand Calibration

This node detects ChArUco boards and publishes their pose as a TF frame,
making it compatible with easy_handeye calibration system.

It also publishes a debug image with the pose axis drawn on it to the
/charuco/result topic.
"""

import json
import hashlib
import threading
from copy import deepcopy

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import yaml


def prepare_detection_image(gray, scale):
    """Upscale only the detector input; pose coordinates stay in source pixels."""
    scale = float(scale)
    if not np.isfinite(scale) or scale < 1.0 or scale > 4.0:
        raise ValueError("detection_scale must be finite and in [1, 4]")
    if abs(scale - 1.0) < 1e-9:
        return gray, 1.0
    return (
        cv2.resize(
            gray,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC,
        ),
        scale,
    )


def rescale_corners_to_source(corners, scale, offset_xy=(0.0, 0.0)):
    if corners is None:
        return None
    restored = np.asarray(corners, dtype=np.float32) / float(scale)
    restored[..., 0] += float(offset_xy[0])
    restored[..., 1] += float(offset_xy[1])
    return restored


def marker_roi(corners, image_shape, padding_px):
    """Build a source-pixel ROI from detected marker corners."""
    if not corners:
        return None
    height, width = image_shape[:2]
    points = np.concatenate(
        [np.asarray(item, dtype=float).reshape(-1, 2) for item in corners], axis=0
    )
    if points.size == 0 or not np.all(np.isfinite(points)):
        return None
    padding = max(0.0, float(padding_px))
    x0 = max(0, int(np.floor(np.min(points[:, 0]) - padding)))
    y0 = max(0, int(np.floor(np.min(points[:, 1]) - padding)))
    x1 = min(width, int(np.ceil(np.max(points[:, 0]) + padding)) + 1)
    y1 = min(height, int(np.ceil(np.max(points[:, 1]) + padding)) + 1)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    return x0, y0, x1, y1


def runtime_color_model_evidence(parameters, camera_frame, image_topic):
    """Validate the published RGB model; no camera/device API is opened."""
    if not isinstance(parameters, dict):
        raise ValueError('RUNTIME_CAMERA_PARAMETERS_MISSING')
    profile = parameters.get('runtime_profile')
    if (not isinstance(profile, dict) or profile.get('source') != 'realsense_active_profile'
            or profile.get('schema_version') != 1):
        raise ValueError('RUNTIME_CAMERA_PROFILE_UNAVAILABLE')
    if parameters.get('frame_id') != camera_frame:
        raise ValueError('RUNTIME_CAMERA_FRAME_MISMATCH')
    if parameters.get('color_topic', '/supervisor/camera/color/image_raw') != image_topic:
        raise ValueError('RUNTIME_CAMERA_COLOR_TOPIC_MISMATCH')
    correction = profile.get('color_projection_correction')
    if not isinstance(correction, dict) or correction.get('enabled') is not False:
        raise ValueError('RUNTIME_CAMERA_RGB_RESAMPLING_UNSUPPORTED')
    rectification = None
    activation = profile.get('geometry_activation_stamp_ns')
    pair_settings = parameters.get('pair_rectification')
    declared = (profile.get('rectification') is not None
                or 'rectified' in str(profile.get('geometry_intrinsics_source', ''))
                or 'rectified' in str(parameters.get('intrinsics_source', ''))
                or (isinstance(pair_settings, dict) and pair_settings.get('enabled') is True))
    if declared or activation is not None:
        from alicia_flexible_grasp.vision.rgbd_rectification import (
            rectification_contract, require_rectified_source_after_activation)
        rectification = rectification_contract(parameters)
        # Validate canonical, bounded activation even before an image arrives.
        # The actual source stamp is checked again at both frame admission and
        # the final TF publication barrier; this is not a freshness substitute.
        require_rectified_source_after_activation(parameters, 2**63-1)
    color = profile.get('geometry_intrinsics' if rectification is not None else 'color_intrinsics')
    if not isinstance(color, dict):
        raise ValueError('RUNTIME_CAMERA_COLOR_INTRINSICS_MISSING')
    color = deepcopy(color)
    for key in ('width', 'height'):
        if type(color.get(key)) is not int or color[key] <= 0 or parameters.get(key) != color[key]:
            raise ValueError('RUNTIME_CAMERA_DIMENSIONS_INVALID')
    for key in ('fx', 'fy', 'cx', 'cy'):
        if type(color.get(key)) not in (int, float) or not np.isfinite(color[key]):
            raise ValueError('RUNTIME_CAMERA_INTRINSICS_INVALID')
    if min(color['fx'], color['fy']) <= 0:
        raise ValueError('RUNTIME_CAMERA_FOCAL_LENGTH_INVALID')
    coefficients = color.get('coeffs')
    if (not isinstance(coefficients, (list, tuple)) or len(coefficients) != 5
            or any(type(v) not in (int, float) or not np.isfinite(v) for v in coefficients)):
        raise ValueError('RUNTIME_CAMERA_DISTORTION_INVALID')
    if color.get('model') not in ('none', 'brown_conrady', 'inverse_brown_conrady'):
        raise ValueError('RUNTIME_CAMERA_DISTORTION_UNSUPPORTED')
    device = profile.get('device', {})
    serial = device.get('serial_number') if isinstance(device, dict) else None
    version = profile.get('sdk_version')
    if (not isinstance(serial, str) or serial in ('', 'unavailable')
            or not isinstance(version, str) or version in ('', 'unavailable')):
        raise ValueError('RUNTIME_CAMERA_PROVENANCE_MISSING')
    if rectification is not None:
        if (color['model'] != 'none' or list(color['coeffs']) != [0.]*5
                or any(parameters.get(key) != color[key] for key in ('fx', 'fy', 'cx', 'cy'))
                or profile.get('color_intrinsics') != rectification['source_intrinsics']
                or rectification.get('sdk_version') != version
                or rectification.get('physical_camera_axes_changed') is not False):
            raise ValueError('RUNTIME_CAMERA_RECTIFICATION_SOURCE_MISMATCH')
    # Depth presets/temporal-filter status can change without changing RGB
    # geometry; they must not manufacture a calibration-model change.
    result = dict(source='realsense_runtime', source_parameter='/camera',
                source_field='runtime_profile.color_intrinsics',
                color_intrinsics=color, camera_frame=camera_frame,
                image_topic=image_topic, serial_number=serial, sdk_version=version,
                color_projection_correction=False,
                backend='realsense_sdk_rays_normalized_pnp',
                pixel_error_model='realsense_sdk_projection_original_pixels',
                image_time_association='parameter_snapshot_not_capture_metadata')
    if activation is not None:
        result['geometry_activation_stamp_ns'] = str(int(activation))
        result['image_time_association'] = 'source_stamp_after_geometry_activation_and_parameter_snapshot'
    if rectification is not None:
        result.update(source_field='runtime_profile.geometry_intrinsics',
            raw_color_intrinsics=deepcopy(profile['color_intrinsics']),
            rectification=deepcopy(rectification),
            pixel_error_model='realsense_sdk_none_projection_rectified_pixels',
            rgb_domain_policy='complete_observed_marker_corner_roi_with_existing_detection_padding')
    return result


class RuntimeColorModel:
    """Pure SDK projection/ray math; never create a context, pipeline or device."""

    def __init__(self, evidence, sdk=None):
        if sdk is None:
            import pyrealsense2 as sdk
        backend_version = getattr(sdk, '__version__', None)
        if not isinstance(backend_version, str) or backend_version != evidence['sdk_version']:
            raise ValueError('RUNTIME_CAMERA_SDK_VERSION_MISMATCH_OR_UNKNOWN')
        self.backend_sdk_version = backend_version
        self.backend_sdk_module = str(getattr(sdk, '__file__', 'unavailable'))
        self.sdk = sdk
        self.evidence = deepcopy(evidence)
        self.fingerprint = hashlib.sha256(json.dumps(
            evidence, sort_keys=True, allow_nan=False).encode('utf-8')).hexdigest()
        color = evidence['color_intrinsics']
        intrinsics = sdk.intrinsics()
        for key in ('width', 'height', 'fx', 'fy'):
            setattr(intrinsics, key, color[key])
        intrinsics.ppx, intrinsics.ppy = color['cx'], color['cy']
        intrinsics.model = getattr(sdk.distortion, color['model'])
        intrinsics.coeffs = color['coeffs']
        self.intrinsics = intrinsics
        self.rgb_valid = None
        if evidence.get('rectification') is not None:
            from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
            self.rgb_valid = rectification_rgb_valid_mask(dict(runtime_profile=dict(
                rectification=evidence['rectification'], geometry_intrinsics=color)),
                (color['height'], color['width']))

    def validate_rgb_points(self, pixels):
        """Every contributing subpixel sampling cell must have observed RGB."""
        if self.rgb_valid is None:
            return
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        height, width = self.rgb_valid.shape
        if (not len(pixels) or not np.all(np.isfinite(pixels))
                or np.any(pixels < 0) or np.any(pixels[:, 0] > width-1)
                or np.any(pixels[:, 1] > height-1)):
            raise ValueError('RUNTIME_CAMERA_RGB_SAMPLE_UNOBSERVED')
        low, high = np.floor(pixels).astype(int), np.ceil(pixels).astype(int)
        for x in (low[:, 0], high[:, 0]):
            for y in (low[:, 1], high[:, 1]):
                if not self.rgb_valid[y, x].all():
                    raise ValueError('RUNTIME_CAMERA_RGB_SAMPLE_UNOBSERVED')

    def validate_rgb_detection_region(self, marker_corners, charuco_corners, padding_px):
        """Padding is the existing detector ROI setting, not a board threshold."""
        if self.rgb_valid is None:
            return
        corners = list(marker_corners or []) + [charuco_corners]
        points = np.concatenate([np.asarray(c, dtype=float).reshape(-1, 2) for c in corners])
        self.validate_rgb_points(points)
        roi = marker_roi([points], self.rgb_valid.shape, padding_px)
        if roi is None or not self.rgb_valid[roi[1]:roi[3], roi[0]:roi[2]].all():
            raise ValueError('RUNTIME_CAMERA_RGB_DETECTION_SUPPORT_UNOBSERVED')

    def normalized_points(self, pixels):
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        if not np.all(np.isfinite(pixels)):
            raise ValueError('RUNTIME_CAMERA_PIXELS_NONFINITE')
        self.validate_rgb_points(pixels)
        rays = np.asarray([self.sdk.rs2_deproject_pixel_to_point(
            self.intrinsics, p.tolist(), 1.) for p in pixels], dtype=float)
        if not np.all(np.isfinite(rays)) or np.any(rays[:, 2] <= 0):
            raise ValueError('RUNTIME_CAMERA_RAYS_INVALID')
        return np.ascontiguousarray(rays[:, :2] / rays[:, 2, None]).reshape(-1, 1, 2)

    def project(self, object_points, rvec, tvec):
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
        points = np.asarray(object_points, dtype=float).reshape(-1, 3)
        camera_points = points @ rotation.T + np.asarray(tvec, dtype=float).reshape(1, 3)
        if not np.all(np.isfinite(camera_points)) or np.any(camera_points[:, 2] <= 0):
            raise ValueError('RUNTIME_CAMERA_POINTS_BEHIND_OR_NONFINITE')
        pixels = np.asarray([self.sdk.rs2_project_point_to_pixel(
            self.intrinsics, p.tolist()) for p in camera_points], dtype=float)
        if not np.all(np.isfinite(pixels)):
            raise ValueError('RUNTIME_CAMERA_PROJECTION_NONFINITE')
        return pixels.reshape(-1, 2)

    def solve(self, object_points, pixels):
        points = np.ascontiguousarray(object_points, dtype=float).reshape(-1, 1, 3)
        observed = np.asarray(pixels, dtype=float).reshape(-1, 2)
        if len(points) < 4 or len(points) != len(observed) or not np.all(np.isfinite(points)):
            raise ValueError('RUNTIME_CAMERA_PNP_POINTS_INVALID')
        normalized = self.normalized_points(observed)
        identity = np.eye(3)
        flag = getattr(cv2, 'SOLVEPNP_SQPNP', cv2.SOLVEPNP_EPNP)
        ok, rvec, tvec = cv2.solvePnP(points, normalized, identity, None, flags=flag)
        if not ok:
            raise ValueError('RUNTIME_CAMERA_PNP_FAILED')
        if hasattr(cv2, 'solvePnPRefineLM'):
            rvec, tvec = cv2.solvePnPRefineLM(points, normalized, identity, None, rvec, tvec)
        projected = self.project(points, rvec, tvec)
        rms = float(np.sqrt(np.mean(np.sum((projected-observed)**2, axis=1))))
        if not np.isfinite(rms):
            raise ValueError('RUNTIME_CAMERA_REPROJECTION_INVALID')
        return rvec, tvec, rms


def read_runtime_camera_parameters(namespace, caller_id, master_uri):
    """Private, time-limited XMLRPC: no rospy shared proxy or hardware call."""
    from xmlrpc.client import ServerProxy, Transport

    class TimedTransport(Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = .2
            return connection

    with ServerProxy(master_uri, transport=TimedTransport()) as master:
        code, message, parameters = master.getParam(caller_id, namespace)
    if code != 1:
        raise ValueError('RUNTIME_CAMERA_PARAMETER_READ_FAILED: ' + str(message))
    return parameters


class RuntimeParameterReader:
    """One daemon; every frame requests new evidence with a bounded wait.

    A stalled read is never replaced by more workers or treated as fresh cached
    evidence. Late results belong to the dropped frame and are discarded.
    """

    def __init__(self, read):
        self._read = read
        self._lock = threading.Lock()
        self._consumer = threading.Lock()
        self._requested, self._ready = threading.Event(), threading.Event()
        self._outstanding = self._closed = False
        self._result = None
        self._worker = threading.Thread(target=self._run, daemon=True,
                                        name='charuco-runtime-parameters')
        self._worker.start()

    @property
    def closed(self):
        with self._lock:
            return self._closed

    def read(self, timeout=.1):
        if not self._consumer.acquire(blocking=False):
            raise ValueError('RUNTIME_CAMERA_PARAMETER_READER_BUSY')
        try:
            return self._read_one(timeout)
        finally:
            self._consumer.release()

    def _read_one(self, timeout):
        with self._lock:
            if self._closed:
                raise ValueError('RUNTIME_CAMERA_READER_CLOSED')
            if self._outstanding and not self._ready.is_set():
                raise ValueError('RUNTIME_CAMERA_PARAMETER_READ_PENDING')
            self._result = None  # Never reuse a late result from an earlier frame.
            self._ready.clear()
            self._outstanding = True
            self._requested.set()
        if not self._ready.wait(timeout):
            raise ValueError('RUNTIME_CAMERA_PARAMETER_READ_TIMEOUT')
        with self._lock:
            if self._closed:
                raise ValueError('RUNTIME_CAMERA_READER_CLOSED')
            result, error = self._result
            self._outstanding = False
            self._result = None
            self._ready.clear()
        if error is not None:
            raise ValueError('RUNTIME_CAMERA_PARAMETER_READ_FAILED: ' + error)
        return result

    def close(self):
        with self._lock:
            self._closed = True
            self._requested.set()
            self._ready.set()
        # No join: shutdown cannot wait indefinitely for an RPC.

    def _run(self):
        while True:
            self._requested.wait()
            self._requested.clear()
            with self._lock:
                if self._closed:
                    return
            result, error = None, None
            try:
                result = self._read()
            except Exception as exc:
                error = '%s: %s' % (type(exc).__name__, exc)
            with self._lock:
                if self._closed:
                    return
                self._result = (result, error)
                self._ready.set()


class LatestImageWorker:
    """One processing thread and one replaceable pending source image.

    The ROS transport callback must keep draining the socket while detection
    runs. Only unprocessed pending images are replaced; source stamps and the
    existing callback's model/freshness checks remain untouched.
    """

    def __init__(self, process, on_error):
        self._process, self._on_error = process, on_error
        self._condition = threading.Condition()
        self._pending = None
        self._closed = False
        self._worker = threading.Thread(target=self._run, daemon=True,
                                        name='charuco-latest-image')
        self._worker.start()

    def submit(self, image):
        with self._condition:
            if self._closed:
                return False
            self._pending = image
            self._condition.notify()
            return True

    def close(self):
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify()
        # Never wait on detection during shutdown. The runtime reader is
        # closed separately so in-flight work cannot publish a new valid pose.

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                image, self._pending = self._pending, None
            try:
                self._process(image)
            except Exception as exc:
                self._on_error(exc)


class CharucoTracker:
    def __init__(self):
        rospy.init_node('charuco_tracker', anonymous=True)
        
        # Parameters
        board_size_param = rospy.get_param('~board_size', [11, 8])  # Width x Height
        # Handle both string and list formats for board_size
        if isinstance(board_size_param, str):
            # Parse string format like "[11, 8]"
            import ast
            try:
                self.board_size = ast.literal_eval(board_size_param)
            except:
                rospy.logwarn(f"Failed to parse board_size string: {board_size_param}, using default [11, 8]")
                self.board_size = [11, 8]
        else:
            self.board_size = board_size_param
            
        # Ensure board_size is a tuple of exactly 2 integers
        if not isinstance(self.board_size, (list, tuple)) or len(self.board_size) != 2:
            rospy.logwarn(f"Invalid board_size format: {self.board_size}, using default [11, 8]")
            self.board_size = [11, 8]
        
        self.board_size = tuple(map(int, self.board_size))  # Convert to tuple of ints
        
        self.square_length = rospy.get_param('~square_length', 0.015)  # 15mm
        self.marker_length = rospy.get_param('~marker_length', 0.011)  # 11mm
        self.dictionary_id = rospy.get_param('~dictionary_id', 'DICT_4X4_100')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.board_frame = rospy.get_param('~board_frame', 'charuco_board')
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/usb_cam/camera_info')
        self.camera_info_yaml = rospy.get_param('~camera_info_yaml', '')
        self.intrinsics_source = rospy.get_param('~intrinsics_source', 'legacy')
        if self.intrinsics_source not in ('legacy', 'realsense_runtime'):
            raise ValueError('unsupported ~intrinsics_source')
        if self.intrinsics_source == 'realsense_runtime' and self.camera_info_yaml:
            raise ValueError('runtime intrinsics cannot use ~camera_info_yaml')
        self.runtime_model = None
        self.runtime_model_rejected = None
        self.runtime_model_error = None
        self.runtime_parameter_reader = None
        self.runtime_frame_worker = None
        self.runtime_callback_lock = threading.Lock()
        if self.intrinsics_source == 'realsense_runtime':
            namespace = rospy.get_param('~camera_parameter_namespace', '/camera')
            if namespace != '/camera':
                raise ValueError('current-stream intrinsics require /camera')
            from rosgraph import get_master_uri
            caller_id, master_uri = rospy.get_name(), get_master_uri()
            self.runtime_parameter_reader = RuntimeParameterReader(
                lambda: read_runtime_camera_parameters(namespace, caller_id, master_uri))
            rospy.on_shutdown(self.runtime_parameter_reader.close)
        self.quality_topic = rospy.get_param('~quality_topic', '/charuco/quality')
        self.max_input_age_sec = float(rospy.get_param('~max_input_age_sec', 0.35))
        self.detection_scale = float(rospy.get_param('~detection_scale', 1.0))
        if not np.isfinite(self.detection_scale) or not 1.0 <= self.detection_scale <= 4.0:
            raise ValueError("~detection_scale must be finite and in [1, 4]")
        self.detection_roi_padding_px = float(
            rospy.get_param('~detection_roi_padding_px', 45.0)
        )
        if not np.isfinite(self.detection_roi_padding_px) or self.detection_roi_padding_px < 0.0:
            raise ValueError("~detection_roi_padding_px must be finite and non-negative")
        
        # Camera calibration parameters (should be loaded from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_marker_count = 0
        self.last_marker_corners = []
        self.last_marker_ids = None
        self.detection_roi = None
        self.last_charuco_count = 0
        self.last_edge_clearance_px = None
        self.last_reprojection_rms_px = None
        self.last_pose_ok = False
        self.stale_drop_count = 0
        
        # ChArUco board setup
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, self.dictionary_id))
        if hasattr(aruco, 'CharucoBoard'):
            self.board = aruco.CharucoBoard(self.board_size, self.square_length, self.marker_length, self.dictionary)
        else:
            self.board = aruco.CharucoBoard_create(
                self.board_size[0], self.board_size[1],
                self.square_length, self.marker_length, self.dictionary
            )
        
        # Set legacy pattern for OpenCV 4.x compatibility
        if hasattr(self.board, 'setLegacyPattern'):
            self.board.setLegacyPattern(True)
            rospy.loginfo("Legacy pattern enabled for OpenCV 4.x compatibility")
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Publisher for the result image
        self.result_pub = rospy.Publisher('/charuco/result', Image, queue_size=1)
        self.quality_pub = rospy.Publisher(self.quality_topic, String, queue_size=1)

        if self.camera_info_yaml:
            self._load_camera_info_yaml(self.camera_info_yaml)
        
        # Runtime detection runs outside the socket reader. queue_size=1 alone
        # does not prevent a slow synchronous callback from delaying reception.
        if self.intrinsics_source == 'realsense_runtime':
            self.runtime_frame_worker = LatestImageWorker(
                lambda image: self.callback(image, None),
                lambda exc: rospy.logerr('ChArUco image worker failed: %s', exc))
            rospy.on_shutdown(self._close_runtime_pipeline)

        # Subscribers
        if self.intrinsics_source == 'realsense_runtime' or self.camera_matrix is not None:
            self.image_sub = rospy.Subscriber(
                self.image_topic,
                Image,
                self.image_callback,
                queue_size=1,
                buff_size=2 ** 24,
                tcp_nodelay=True,
            )
            self.camera_info_sub = None
            self.ts = None
        else:
            self.image_sub = message_filters.Subscriber(self.image_topic, Image)
            self.camera_info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)

            # Synchronize image and camera info
            self.ts = message_filters.TimeSynchronizer([self.image_sub, self.camera_info_sub], 10)
            self.ts.registerCallback(self.callback)
        
        rospy.loginfo("ChArUco tracker initialized")
        rospy.loginfo(f"Board size: {self.board_size[0]}x{self.board_size[1]}")
        rospy.loginfo(f"Square length: {self.square_length*1000:.1f}mm")
        rospy.loginfo(f"Marker length: {self.marker_length*1000:.1f}mm")
        rospy.loginfo(f"Image topic: {self.image_topic}")
        rospy.loginfo(f"Camera info topic: {self.camera_info_topic}")
        rospy.loginfo(f"Quality topic: {self.quality_topic}")
        rospy.loginfo(f"Max input age: {self.max_input_age_sec:.3f}s")
        rospy.loginfo(f"Detection scale: {self.detection_scale:.2f}x")
        rospy.loginfo(f"Detection ROI padding: {self.detection_roi_padding_px:.1f}px")

    def _load_camera_info_yaml(self, path):
        """Load a static CameraInfo YAML so this tracker can reuse an existing image stream."""
        try:
            with open(path, 'r') as handle:
                data = yaml.safe_load(handle)
            self.camera_matrix = np.array(
                data['camera_matrix']['data'], dtype=float
            ).reshape(3, 3)
            self.dist_coeffs = np.array(
                data.get('distortion_coefficients', {}).get('data', []), dtype=float
            )
            rospy.loginfo("Camera parameters loaded from YAML: %s", path)
        except Exception as exc:
            self.camera_matrix = None
            self.dist_coeffs = None
            rospy.logwarn("Failed to load camera_info_yaml %s: %s", path, exc)

    def _close_runtime_pipeline(self):
        # Close the model reader first: an image already being processed must
        # fail the existing before-publication closed checks as well.
        if self.runtime_parameter_reader is not None:
            self.runtime_parameter_reader.close()
        if self.runtime_frame_worker is not None:
            self.runtime_frame_worker.close()

    def image_callback(self, image_msg):
        if self.intrinsics_source == 'realsense_runtime':
            self.runtime_frame_worker.submit(image_msg)
            return
        self.callback(image_msg, None)
    
    def callback(self, image_msg, camera_info_msg):
        if self.intrinsics_source != 'realsense_runtime':
            return self._process_callback(image_msg, camera_info_msg)
        # ROS can invoke callbacks concurrently when a topic has multiple
        # publishers. Drop busy frames rather than mixing per-frame evidence.
        if not self.runtime_callback_lock.acquire(blocking=False):
            return
        try:
            if self.runtime_parameter_reader.closed or rospy.is_shutdown():
                return
            return self._process_callback(image_msg, camera_info_msg)
        finally:
            self.runtime_callback_lock.release()

    def _process_callback(self, image_msg, camera_info_msg):
        """Process synchronized image and camera info messages."""

        if self._drop_if_stale(image_msg.header.stamp):
            return
        
        self.last_pose_ok = False
        self.last_reprojection_rms_px = None
        if self.intrinsics_source == 'realsense_runtime':
            if not self._validate_runtime_model(image_msg):
                self.publish_quality(image_msg.header.stamp, (image_msg.height, image_msg.width), False)
                return

        # Update camera parameters if needed
        if self.camera_matrix is None and camera_info_msg is not None:
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            rospy.loginfo("Camera parameters loaded")
        if self.camera_matrix is None and self.intrinsics_source != 'realsense_runtime':
            rospy.logwarn_throttle(
                2.0,
                "No camera intrinsics available; provide CameraInfo or ~camera_info_yaml.",
            )
            self.publish_quality(image_msg.header.stamp, None, False)
            return
        
        # Convert ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Failed to convert image: {e}")
            return
        
        # Detect ChArUco board
        self.last_reprojection_rms_px = None
        charuco_corners, charuco_ids = self.detect_charuco_board(cv_image)
        pose_ok = False
        if (charuco_corners is not None and self.intrinsics_source == 'realsense_runtime'
                and self.runtime_model.rgb_valid is not None):
            try:
                self.runtime_model.validate_rgb_detection_region(
                    self.last_marker_corners, charuco_corners, self.detection_roi_padding_px)
            except ValueError as exc:
                self.runtime_model_error = str(exc)
                charuco_corners, charuco_ids = None, None

        # Marker boxes and IDs remain useful even before four ChArUco corners
        # are available for pose estimation.
        if self.last_marker_corners and self.last_marker_ids is not None:
            aruco.drawDetectedMarkers(
                cv_image, self.last_marker_corners, self.last_marker_ids
            )
        
        if charuco_corners is not None and charuco_ids is not None:
            # Always show detected ChArUco corners.  This keeps the debug image
            # useful even when camera calibration or pose estimation fails.
            if hasattr(aruco, 'drawDetectedCornersCharuco'):
                aruco.drawDetectedCornersCharuco(
                    cv_image, charuco_corners, charuco_ids, (0, 255, 0)
                )

            # Estimate pose
            pose, rvec, tvec = self.estimate_pose(charuco_corners, charuco_ids)
            if pose is not None and self.intrinsics_source == 'realsense_runtime':
                # The detector/PnP may be slow or the profile may change while
                # working. Re-read before publishing any new TF or valid quality.
                if not self._validate_runtime_model(image_msg) or self._drop_if_stale(image_msg.header.stamp):
                    pose = None
                if self.runtime_parameter_reader.closed or rospy.is_shutdown():
                    self.runtime_model_error = 'RUNTIME_CAMERA_CLOSED_OR_SHUTDOWN'
                    pose = None
            if pose is not None:
                pose_ok = True
                # Publish TF transform
                self.publish_tf(pose, image_msg.header.stamp)
                rospy.loginfo_throttle(
                    2.0,
                    "ChArUco pose OK: markers=%d corners=%d t=[%.3f, %.3f, %.3f] m",
                    self.last_marker_count,
                    self.last_charuco_count,
                    tvec[0][0],
                    tvec[1][0],
                    tvec[2][0],
                )
                
                # Draw the pose axis on the image for visualization
                try:
                    # Check if axes will be within image bounds before drawing
                    if self.intrinsics_source == 'realsense_runtime':
                        self._draw_runtime_axes(cv_image, rvec, tvec)
                    elif self._can_draw_axes_safely(rvec, tvec, cv_image.shape):
                        # Use a fixed axis length that's appropriate for visualization
                        # 0.03 meters (3cm) is a good compromise between visibility and staying in frame
                        axis_length = 0.03
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)
                    else:
                        # If axes would go out of frame, just draw a small marker at the origin
                        origin_2d, _ = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec, tvec, 
                                                       self.camera_matrix, self.dist_coeffs)
                        origin_2d = tuple(map(int, origin_2d[0, 0]))
                        cv2.circle(cv_image, origin_2d, 5, (0, 255, 0), -1)  # Green circle at origin
                except AttributeError:
                    # Fallback to the older aruco.drawAxis for compatibility
                    if self._can_draw_axes_safely(rvec, tvec, cv_image.shape):
                        aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                    else:
                        # Draw origin marker instead
                        origin_2d, _ = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec, tvec, 
                                                       self.camera_matrix, self.dist_coeffs)
                        origin_2d = tuple(map(int, origin_2d[0, 0]))
                        cv2.circle(cv_image, origin_2d, 5, (0, 255, 0), -1)

        if not pose_ok:
            if self.last_marker_count == 0:
                rospy.logwarn_throttle(
                    2.0,
                    "No ArUco markers detected; keep the ChArUco board fully visible and well lit.",
                )
            elif self.last_charuco_count < 4:
                rospy.logwarn_throttle(
                    2.0,
                    "Detected %d ArUco markers but only %d ChArUco corners; need at least 4 stable corners.",
                    self.last_marker_count,
                    self.last_charuco_count,
                )
            else:
                rospy.logwarn_throttle(
                    2.0,
                    "Detected markers=%d corners=%d but pose estimation failed.",
                    self.last_marker_count,
                    self.last_charuco_count,
                )

        # Publish the final image with annotations (if any)
        try:
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            result_msg.header = image_msg.header
            self.result_pub.publish(result_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish result image: {e}")
        if (self.intrinsics_source == 'realsense_runtime'
                and (self.runtime_parameter_reader.closed or rospy.is_shutdown())):
            self.runtime_model_error = 'RUNTIME_CAMERA_CLOSED_OR_SHUTDOWN'
            pose_ok = False
        self.last_pose_ok = pose_ok
        self.publish_quality(image_msg.header.stamp, cv_image.shape, pose_ok)

    def _validate_runtime_model(self, image_msg):
        try:
            if self.runtime_model_rejected:
                raise ValueError(self.runtime_model_rejected)
            parameters = self.runtime_parameter_reader.read()
            evidence = runtime_color_model_evidence(parameters, self.camera_frame, self.image_topic)
            fingerprint = hashlib.sha256(json.dumps(
                evidence, sort_keys=True, allow_nan=False).encode('utf-8')).hexdigest()
            if self.runtime_model is not None and fingerprint != self.runtime_model.fingerprint:
                self.runtime_model_rejected = 'RUNTIME_CAMERA_MODEL_CHANGED_RESTART_REQUIRED'
                raise ValueError(self.runtime_model_rejected)
            if 'geometry_activation_stamp_ns' in evidence:
                from alicia_flexible_grasp.vision.rgbd_rectification import require_rectified_source_after_activation
                require_rectified_source_after_activation(parameters, image_msg.header.stamp.to_nsec())
            color = evidence['color_intrinsics']
            if ((image_msg.width, image_msg.height) != (color['width'], color['height'])
                    or image_msg.header.frame_id != self.camera_frame):
                raise ValueError('RUNTIME_CAMERA_IMAGE_DIMENSIONS_OR_FRAME_MISMATCH')
            if self.runtime_model is None:
                self.runtime_model = RuntimeColorModel(evidence)
            self.runtime_model_error = None
            return True
        except Exception as exc:
            self.runtime_model_error = str(exc)
            self.last_pose_ok = False
            self.last_reprojection_rms_px = None
            rospy.logwarn_throttle(2., 'Current-stream camera model rejected: %s', exc)
            return False

    def _board_points(self, charuco_corners, charuco_ids):
        if hasattr(self.board, 'matchImagePoints'):
            return self.board.matchImagePoints(charuco_corners, charuco_ids)
        # Older OpenCV has no Board.matchImagePoints, but board order is public.
        points = (self.board.getChessboardCorners() if hasattr(self.board, 'getChessboardCorners')
                  else self.board.chessboardCorners)
        indices = np.asarray(charuco_ids).reshape(-1)
        if not np.issubdtype(indices.dtype, np.integer) or np.any(indices < 0) or np.any(indices >= len(points)):
            raise ValueError('CHARUCO_IDS_INVALID')
        return np.asarray(points)[indices], np.asarray(charuco_corners)

    def _draw_runtime_axes(self, image, rvec, tvec):
        try:
            pixels = self.runtime_model.project(
                [[0., 0., 0.], [.03, 0., 0.], [0., .03, 0.], [0., 0., .03]], rvec, tvec)
            height, width = image.shape[:2]
            inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < width)
                      & (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
            if inside.all():
                points = np.rint(pixels).astype(int)
                for index, color in enumerate(((0, 0, 255), (0, 255, 0), (255, 0, 0)), 1):
                    cv2.line(image, tuple(points[0]), tuple(points[index]), color, 2)
            elif inside[0]:
                cv2.circle(image, tuple(np.rint(pixels[0]).astype(int)), 5, (0, 255, 0), -1)
        except Exception as exc:
            rospy.logwarn_throttle(2., 'Runtime axis overlay unavailable: %s', exc)
            # Visual overlay is optional; no OpenCV distortion fallback.

    def _drop_if_stale(self, stamp):
        if self.max_input_age_sec <= 0.0 or stamp is None or stamp.to_sec() <= 0.0:
            return False
        age = (rospy.Time.now() - stamp).to_sec()
        if age <= self.max_input_age_sec:
            return False
        self.stale_drop_count += 1
        rospy.logwarn_throttle(
            2.0,
            "Dropping stale ChArUco input frame: age=%.3fs limit=%.3fs dropped=%d",
            age,
            self.max_input_age_sec,
            self.stale_drop_count,
        )
        return True

    def detect_charuco_board(self, image):
        """Detect ChArUco board in the image."""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers first
            if hasattr(aruco, 'DetectorParameters'):
                detector_params = aruco.DetectorParameters()
            else:
                detector_params = aruco.DetectorParameters_create()
            detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE  # Important for ChArUco

            marker_corners, marker_ids, detection_gray, detection_scale, offset = (
                self._detect_markers_in_roi(gray, detector_params, self.detection_roi)
            )
            # A moving board can leave the cached ROI. Retry the complete
            # source image immediately and rebuild the ROI from that evidence.
            if (marker_ids is None or len(marker_ids) == 0) and self.detection_roi is not None:
                marker_corners, marker_ids, detection_gray, detection_scale, offset = (
                    self._detect_markers_in_roi(gray, detector_params, None)
                )
            
            if marker_ids is None or len(marker_ids) == 0:
                self.last_marker_count = 0
                self.last_marker_corners = []
                self.last_marker_ids = None
                self.detection_roi = None
                self.last_charuco_count = 0
                self.last_edge_clearance_px = None
                return None, None
            self.last_marker_count = len(marker_ids)
            self.last_marker_corners = [
                rescale_corners_to_source(corners, detection_scale, offset)
                for corners in marker_corners
            ]
            self.last_marker_ids = np.asarray(marker_ids, dtype=np.int32)
            self.detection_roi = marker_roi(
                self.last_marker_corners,
                gray.shape,
                self.detection_roi_padding_px,
            )
            
            # Detect ChArUco corners
            if hasattr(aruco, 'CharucoDetector'):
                charuco_detector = cv2.aruco.CharucoDetector(self.board, detectorParams=detector_params)
                # Reuse the exact marker result above. Calling detectBoard
                # without these inputs repeats the most expensive search.
                charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(
                    detection_gray, None, None, marker_corners, marker_ids
                )
            else:
                _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, detection_gray, self.board
                )
            charuco_corners = rescale_corners_to_source(
                charuco_corners, detection_scale, offset
            )
            self.last_charuco_count = 0 if charuco_ids is None else len(charuco_ids)
            self.last_edge_clearance_px = self._edge_clearance_px(gray.shape, charuco_corners)
            
            return charuco_corners, charuco_ids
            
        except Exception as e:
            rospy.logwarn(f"ChArUco detection failed: {e}")
            return None, None

    def _detect_markers_in_roi(self, gray, detector_params, roi):
        if roi is None:
            x0, y0, x1, y1 = 0, 0, gray.shape[1], gray.shape[0]
        else:
            x0, y0, x1, y1 = roi
        source = gray[y0:y1, x0:x1]
        detection_gray, detection_scale = prepare_detection_image(
            source, self.detection_scale
        )
        if hasattr(aruco, 'ArucoDetector'):
            detector = aruco.ArucoDetector(self.dictionary, detector_params)
            marker_corners, marker_ids, _ = detector.detectMarkers(detection_gray)
        else:
            marker_corners, marker_ids, _ = aruco.detectMarkers(
                detection_gray, self.dictionary, parameters=detector_params
            )
        return (
            marker_corners,
            marker_ids,
            detection_gray,
            detection_scale,
            (float(x0), float(y0)),
        )
    
    def estimate_pose(self, charuco_corners, charuco_ids):
        """Estimate pose of the ChArUco board."""
        try:
            if self.intrinsics_source == 'realsense_runtime':
                if self.runtime_model is None or self.runtime_model_rejected:
                    raise ValueError('RUNTIME_CAMERA_MODEL_UNAVAILABLE')
                object_points, image_points = self._board_points(charuco_corners, charuco_ids)
                rvec, tvec, rms = self.runtime_model.solve(object_points, image_points)
                self.last_reprojection_rms_px = rms
                rotation, _ = cv2.Rodrigues(rvec)
                pose = np.eye(4)
                pose[:3, :3], pose[:3, 3] = rotation, tvec.reshape(3)
                return pose, rvec, tvec
            # OpenCV 4.13 removed estimatePoseCharucoBoard from the Python
            # bindings.  Keep the old API when available and use the Board
            # point matcher + solvePnP on newer OpenCV releases.
            if hasattr(aruco, 'estimatePoseCharucoBoard'):
                retval, rvec, tvec = aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board,
                    self.camera_matrix, self.dist_coeffs,
                    None, None, useExtrinsicGuess=False
                )
            else:
                object_points, image_points = self.board.matchImagePoints(
                    charuco_corners, charuco_ids
                )
                if object_points is None or len(object_points) < 4:
                    return None, None, None

                # SQPnP is stable for the sparse, planar corner sets that can
                # occur when the board is small in the D405 image.  ITERATIVE
                # can converge to the mirrored, behind-camera solution here.
                pnp_flag = getattr(cv2, 'SOLVEPNP_SQPNP', cv2.SOLVEPNP_EPNP)
                retval, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=pnp_flag,
                )
                if retval and hasattr(cv2, 'solvePnPRefineLM'):
                    rvec, tvec = cv2.solvePnPRefineLM(
                        object_points,
                        image_points,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                    )

                # A valid board observed by the camera must lie in front of it.
                if retval and float(tvec.reshape(-1)[2]) <= 0.0:
                    rospy.logwarn_throttle(
                        2.0, "Rejected mirrored ChArUco pose behind the camera."
                    )
                    retval = False
            
            if retval:
                self.last_reprojection_rms_px = self._reprojection_rms_px(
                    charuco_corners, charuco_ids, rvec, tvec
                )
                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)
                
                # Create transformation matrix
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.flatten()
                
                return T, rvec, tvec
            else:
                return None, None, None
                
        except Exception as e:
            rospy.logwarn(f"Pose estimation failed: {e}")
            return None, None, None

    def _edge_clearance_px(self, image_shape, charuco_corners):
        if charuco_corners is None or len(charuco_corners) == 0:
            return None
        height, width = image_shape[:2]
        points = np.asarray(charuco_corners, dtype=float).reshape(-1, 2)
        x_min = float(np.min(points[:, 0]))
        x_max = float(np.max(points[:, 0]))
        y_min = float(np.min(points[:, 1]))
        y_max = float(np.max(points[:, 1]))
        return float(min(x_min, y_min, width - 1.0 - x_max, height - 1.0 - y_max))

    def _reprojection_rms_px(self, charuco_corners, charuco_ids, rvec, tvec):
        try:
            if self.intrinsics_source == 'realsense_runtime':
                object_points, image_points = self._board_points(charuco_corners, charuco_ids)
                projected = self.runtime_model.project(object_points, rvec, tvec)
                residual = projected - np.asarray(image_points, dtype=float).reshape(-1, 2)
                rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
                return rms if np.isfinite(rms) else None
            object_points, image_points = self.board.matchImagePoints(
                charuco_corners, charuco_ids
            )
            if object_points is None or image_points is None or len(object_points) < 4:
                return None
            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs,
            )
            projected = np.asarray(projected, dtype=float).reshape(-1, 2)
            image_points = np.asarray(image_points, dtype=float).reshape(-1, 2)
            residual = projected - image_points
            return float(np.sqrt(np.mean(np.sum(np.square(residual), axis=1))))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to compute ChArUco reprojection RMS: %s", exc)
            return None

    def publish_quality(self, timestamp, image_shape, pose_ok):
        height = None
        width = None
        if image_shape is not None:
            height, width = image_shape[:2]
        payload = {
            'stamp': float(timestamp.to_sec()) if timestamp else None,
            'source_stamp_ns': int(timestamp.to_nsec()) if timestamp else None,
            'pose_ok': bool(pose_ok),
            'marker_count': int(self.last_marker_count),
            'charuco_count': int(self.last_charuco_count),
            'edge_clearance_px': self.last_edge_clearance_px,
            'reprojection_rms_px': self.last_reprojection_rms_px,
            'image_width': int(width) if width is not None else None,
            'image_height': int(height) if height is not None else None,
            'camera_frame': self.camera_frame,
            'board_frame': self.board_frame,
            'board_size': list(self.board_size),
            'dictionary_id': self.dictionary_id,
            'square_length_m': float(self.square_length),
            'marker_length_m': float(self.marker_length),
            'detection_scale': self.detection_scale,
            'detection_roi': list(self.detection_roi) if self.detection_roi else None,
        }
        if self.intrinsics_source == 'realsense_runtime':
            model = self.runtime_model
            payload.update(intrinsics_source=self.intrinsics_source,
                           camera_model_fingerprint=model.fingerprint if model else None,
                           camera_model=deepcopy(model.evidence) if model else None,
                           backend_sdk_version=model.backend_sdk_version if model else None,
                           backend_sdk_module=model.backend_sdk_module if model else None,
                           camera_model_error=self.runtime_model_error,
                           camera_model_change_latched=bool(self.runtime_model_rejected))
        self.quality_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def publish_tf(self, pose_matrix, timestamp):
        """Publish the board pose as a TF transform."""
        self._publish_tf_manual(pose_matrix, timestamp)
    
    def _publish_tf_manual(self, pose_matrix, timestamp):
        """Fallback manual TF publishing without scipy."""
        try:
            # Extract rotation and translation
            R = pose_matrix[:3, :3]
            t = pose_matrix[:3, 3]
            
            # Apply coordinate system transformation
            T_cv_to_ros = np.array([
                [0, 0, 1],    # OpenCV Z -> ROS X
                [-1, 0, 0],   # OpenCV -X -> ROS Y  
                [0, -1, 0]    # OpenCV -Y -> ROS Z
            ])
            
            R_ros = T_cv_to_ros @ R
            t_ros = T_cv_to_ros @ t
            
            # Manual rotation matrix to quaternion conversion
            trace = np.trace(R_ros)
            if trace > 0:
                S = np.sqrt(trace + 1.0) * 2
                w = 0.25 * S
                x = (R_ros[2, 1] - R_ros[1, 2]) / S
                y = (R_ros[0, 2] - R_ros[2, 0]) / S
                z = (R_ros[1, 0] - R_ros[0, 1]) / S
            else:
                if R_ros[0, 0] > R_ros[1, 1] and R_ros[0, 0] > R_ros[2, 2]:
                    S = np.sqrt(1.0 + R_ros[0, 0] - R_ros[1, 1] - R_ros[2, 2]) * 2
                    w = (R_ros[2, 1] - R_ros[1, 2]) / S
                    x = 0.25 * S
                    y = (R_ros[0, 1] + R_ros[1, 0]) / S
                    z = (R_ros[0, 2] + R_ros[2, 0]) / S
                elif R_ros[1, 1] > R_ros[2, 2]:
                    S = np.sqrt(1.0 + R_ros[1, 1] - R_ros[0, 0] - R_ros[2, 2]) * 2
                    w = (R_ros[0, 2] - R_ros[2, 0]) / S
                    x = (R_ros[0, 1] + R_ros[1, 0]) / S
                    y = 0.25 * S
                    z = (R_ros[1, 2] + R_ros[2, 1]) / S
                else:
                    S = np.sqrt(1.0 + R_ros[2, 2] - R_ros[0, 0] - R_ros[1, 1]) * 2
                    w = (R_ros[1, 0] - R_ros[0, 1]) / S
                    x = (R_ros[0, 2] + R_ros[2, 0]) / S
                    y = (R_ros[1, 2] + R_ros[2, 1]) / S
                    z = 0.25 * S
            
            # Create transform message
            transform = TransformStamped()
            transform.header.stamp = timestamp
            transform.header.frame_id = self.camera_frame
            transform.child_frame_id = self.board_frame
            
            transform.transform.translation.x = t_ros[0]
            transform.transform.translation.y = t_ros[1]
            transform.transform.translation.z = t_ros[2]
            
            transform.transform.rotation.x = x
            transform.transform.rotation.y = y
            transform.transform.rotation.z = z
            transform.transform.rotation.w = w
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish TF manually: {e}")
    
    def _can_draw_axes_safely(self, rvec, tvec, image_shape):
        """Check if drawing axes will keep endpoints within image bounds."""
        try:
            # Test with a reasonable axis length (3cm)
            axis_length = 0.03
            height, width = image_shape[:2]
            
            # Define axis endpoints in 3D (origin + X, Y, Z axes)
            axis_points = np.array([
                [0.0, 0.0, 0.0],           # Origin
                [axis_length, 0.0, 0.0],   # X-axis end
                [0.0, axis_length, 0.0],   # Y-axis end
                [0.0, 0.0, axis_length]    # Z-axis end
            ], dtype=np.float32)
            
            # Project 3D points to 2D image coordinates
            if self.intrinsics_source == 'realsense_runtime':
                projected_points = self.runtime_model.project(axis_points, rvec, tvec)
            else:
                projected_points, _ = cv2.projectPoints(axis_points, rvec, tvec,
                                                      self.camera_matrix, self.dist_coeffs)
                projected_points = projected_points.reshape(-1, 2)
            
            # Check if all projected points are within image bounds
            # Add a small margin to avoid drawing very close to edges
            margin = 10
            for point in projected_points:
                x, y = point
                if (x < margin or x > width - margin or 
                    y < margin or y > height - margin):
                    return False
            
            return True
            
        except Exception as e:
            rospy.logwarn(f"Error checking axis bounds: {e}")
            return False  # If we can't check, don't draw axes

if __name__ == '__main__':
    try:
        tracker = CharucoTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
