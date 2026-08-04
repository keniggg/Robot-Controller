#!/usr/bin/env python3
"""
ChArUco Tracker Node for Eye-on-Hand Calibration

This node detects ChArUco boards and publishes their pose as a TF frame,
making it compatible with easy_handeye calibration system.

It also publishes a debug image with the pose axis drawn on it to the
/charuco/result topic.
"""

import json

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
        
        # Subscribers
        if self.camera_matrix is not None:
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

    def image_callback(self, image_msg):
        self.callback(image_msg, None)
    
    def callback(self, image_msg, camera_info_msg):
        """Process synchronized image and camera info messages."""

        if self._drop_if_stale(image_msg.header.stamp):
            return
        
        # Update camera parameters if needed
        if self.camera_matrix is None and camera_info_msg is not None:
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            rospy.loginfo("Camera parameters loaded")
        if self.camera_matrix is None:
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
                    if self._can_draw_axes_safely(rvec, tvec, cv_image.shape):
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
        self.last_pose_ok = pose_ok
        self.publish_quality(image_msg.header.stamp, cv_image.shape, pose_ok)

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
            'detection_scale': self.detection_scale,
            'detection_roi': list(self.detection_roi) if self.detection_roi else None,
        }
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
