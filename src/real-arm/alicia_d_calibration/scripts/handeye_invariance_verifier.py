#!/usr/bin/env python3
"""Passive hand-eye invariance verifier.

The verifier never commands the robot.  It only samples TF and ChArUco quality
while the operator manually places the eye-on-hand camera at several settled
poses around one fixed board.
"""

import json
import os
import sys
import time

import numpy as np
import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handeye_invariance_metrics import mean_transform, quaternion_from_matrix, summarize_transforms


class HandeyeInvarianceVerifier:
    def __init__(self):
        rospy.init_node("handeye_invariance_verifier", anonymous=False)

        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tool_frame = rospy.get_param("~tool_frame", "tool0")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.board_frame = rospy.get_param("~board_frame", "charuco_board")
        self.quality_topic = rospy.get_param("~quality_topic", "/charuco/quality")

        self.sample_window_sec = float(rospy.get_param("~sample_window_sec", 1.0))
        self.sample_rate_hz = float(rospy.get_param("~sample_rate_hz", 15.0))
        self.min_window_samples = int(rospy.get_param("~min_window_samples", 8))
        self.min_pose_samples = int(rospy.get_param("~min_pose_samples", 6))
        self.max_tf_age_sec = float(rospy.get_param("~max_tf_age_sec", 0.5))
        self.min_charuco_corners = int(rospy.get_param("~min_charuco_corners", 35))
        self.min_edge_clearance_px = float(rospy.get_param("~min_edge_clearance_px", 12.0))
        self.max_reprojection_rms_px = float(rospy.get_param("~max_reprojection_rms_px", 1.5))
        self.max_tool_window_motion_m = float(rospy.get_param("~max_tool_window_motion_m", 0.0015))
        self.pass_translation_rms_m = float(rospy.get_param("~pass_translation_rms_m", 0.003))
        self.pass_translation_max_m = float(rospy.get_param("~pass_translation_max_m", 0.005))
        self.pass_orientation_rms_deg = float(rospy.get_param("~pass_orientation_rms_deg", 1.0))
        self.pass_orientation_max_deg = float(rospy.get_param("~pass_orientation_max_deg", 2.0))
        default_output_dir = os.path.expanduser("~/.ros/alicia_d_calibration/handeye_verification")
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", default_output_dir))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.latest_quality = None
        self.latest_joint_state = None
        self.captured = []

        self.quality_sub = rospy.Subscriber(self.quality_topic, String, self.quality_cb, queue_size=5)
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb, queue_size=5)
        self.capture_srv = rospy.Service("~capture", Trigger, self.capture_cb)
        self.report_srv = rospy.Service("~report", Trigger, self.report_cb)
        self.clear_srv = rospy.Service("~clear", Trigger, self.clear_cb)

        rospy.loginfo(
            "Passive hand-eye verifier ready: %s->%s->%s->%s, quality=%s",
            self.base_frame,
            self.tool_frame,
            self.camera_frame,
            self.board_frame,
            self.quality_topic,
        )

    def quality_cb(self, msg):
        try:
            self.latest_quality = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Invalid ChArUco quality JSON: %s", exc)

    def joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def capture_cb(self, _request):
        ok, reason = self._quality_gate()
        if not ok:
            return TriggerResponse(success=False, message=reason)

        deadline = rospy.Time.now() + rospy.Duration(self.sample_window_sec)
        rate = rospy.Rate(self.sample_rate_hz)
        samples = []
        tool_positions = []
        failures = []
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                sample = self._read_sample()
                q_ok, q_reason = self._quality_gate()
                if not q_ok:
                    failures.append(q_reason)
                else:
                    samples.append(sample)
                    tool_positions.append(sample["T_base_tool"][:3, 3])
            except Exception as exc:
                failures.append(str(exc))
            rate.sleep()

        if len(samples) < self.min_window_samples:
            suffix = ("; last failure: " + failures[-1]) if failures else ""
            return TriggerResponse(
                success=False,
                message="CAPTURE_REJECTED: only %d valid samples%s"
                % (len(samples), suffix),
            )

        tool_motion = self._max_position_span(tool_positions)
        if tool_motion > self.max_tool_window_motion_m:
            return TriggerResponse(
                success=False,
                message="CAPTURE_REJECTED: tool moved %.1f mm during sample window"
                % (tool_motion * 1000.0),
            )

        T_base_board_mean = mean_transform([s["T_base_board"] for s in samples])
        window_summary = summarize_transforms([s["T_base_board"] for s in samples])
        quality = dict(self.latest_quality or {})
        record = {
            "index": len(self.captured) + 1,
            "stamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "valid_window_samples": len(samples),
            "tool_window_motion_m": tool_motion,
            "window_summary": window_summary,
            "quality": quality,
            "T_base_board": T_base_board_mean.tolist(),
            "T_base_tool": mean_transform([s["T_base_tool"] for s in samples]).tolist(),
            "T_tool_camera": mean_transform([s["T_tool_camera"] for s in samples]).tolist(),
            "T_camera_board": mean_transform([s["T_camera_board"] for s in samples]).tolist(),
        }
        self.captured.append(record)
        message = (
            "captured pose %d/%d: window rms %.2f mm, max %.2f mm, corners=%s"
            % (
                len(self.captured),
                self.min_pose_samples,
                window_summary["translation_rms_m"] * 1000.0,
                window_summary["translation_max_m"] * 1000.0,
                quality.get("charuco_count"),
            )
        )
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)

    def report_cb(self, _request):
        if len(self.captured) < 2:
            return TriggerResponse(
                success=False,
                message="REPORT_REJECTED: capture at least 2 poses first",
            )

        summary = summarize_transforms([np.array(s["T_base_board"], dtype=float) for s in self.captured])
        passed = (
            len(self.captured) >= self.min_pose_samples
            and summary["translation_rms_m"] <= self.pass_translation_rms_m
            and summary["translation_max_m"] <= self.pass_translation_max_m
            and summary["orientation_rms_deg"] <= self.pass_orientation_rms_deg
            and summary["orientation_max_deg"] <= self.pass_orientation_max_deg
        )
        report = {
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "frames": {
                "base_frame": self.base_frame,
                "tool_frame": self.tool_frame,
                "camera_frame": self.camera_frame,
                "board_frame": self.board_frame,
            },
            "thresholds": {
                "min_pose_samples": self.min_pose_samples,
                "translation_rms_m": self.pass_translation_rms_m,
                "translation_max_m": self.pass_translation_max_m,
                "orientation_rms_deg": self.pass_orientation_rms_deg,
                "orientation_max_deg": self.pass_orientation_max_deg,
            },
            "passed": bool(passed),
            "summary": summary,
            "samples": self.captured,
        }
        path = self._write_report(report)
        verdict = "PASS" if passed else "FAIL"
        message = (
            "%s poses=%d rms=%.2f mm max=%.2f mm rot_rms=%.2f deg rot_max=%.2f deg report=%s"
            % (
                verdict,
                len(self.captured),
                summary["translation_rms_m"] * 1000.0,
                summary["translation_max_m"] * 1000.0,
                summary["orientation_rms_deg"],
                summary["orientation_max_deg"],
                path,
            )
        )
        rospy.loginfo(message)
        return TriggerResponse(success=bool(passed), message=message)

    def clear_cb(self, _request):
        count = len(self.captured)
        self.captured = []
        return TriggerResponse(success=True, message="cleared %d captured poses" % count)

    def _read_sample(self):
        T_base_tool, base_tool_stamp = self._lookup_matrix(self.base_frame, self.tool_frame)
        T_tool_camera, tool_camera_stamp = self._lookup_matrix(self.tool_frame, self.camera_frame)
        T_camera_board, camera_board_stamp = self._lookup_matrix(self.camera_frame, self.board_frame)
        now = rospy.Time.now()
        for name, stamp in (
            ("base_tool", base_tool_stamp),
            ("tool_camera", tool_camera_stamp),
            ("camera_board", camera_board_stamp),
        ):
            if stamp.to_sec() <= 0.0:
                continue
            age = abs((now - stamp).to_sec())
            if age > self.max_tf_age_sec:
                raise RuntimeError("%s TF is stale: %.3f sec" % (name, age))
        return {
            "T_base_tool": T_base_tool,
            "T_tool_camera": T_tool_camera,
            "T_camera_board": T_camera_board,
            "T_base_board": T_base_tool.dot(T_tool_camera).dot(T_camera_board),
        }

    def _lookup_matrix(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rospy.Time(0),
            rospy.Duration(0.2),
        )
        return self._matrix_from_transform(transform.transform), transform.header.stamp

    def _quality_gate(self):
        if not self.latest_quality:
            return False, "CHARUCO_QUALITY_MISSING"
        if not self.latest_quality.get("pose_ok", False):
            return False, "CHARUCO_POSE_NOT_READY"
        corners = int(self.latest_quality.get("charuco_count") or 0)
        if corners < self.min_charuco_corners:
            return False, "CHARUCO_CORNERS_LOW: %d < %d" % (corners, self.min_charuco_corners)
        edge = self.latest_quality.get("edge_clearance_px")
        if edge is None or float(edge) < self.min_edge_clearance_px:
            return False, "CHARUCO_EDGE_CLIPPED: %s px" % edge
        reproj = self.latest_quality.get("reprojection_rms_px")
        if reproj is None or float(reproj) > self.max_reprojection_rms_px:
            return False, "CHARUCO_REPROJECTION_HIGH: %s px" % reproj
        stamp = self.latest_quality.get("stamp")
        if stamp is not None:
            age = abs(rospy.Time.now().to_sec() - float(stamp))
            if age > self.max_tf_age_sec:
                return False, "CHARUCO_QUALITY_STALE: %.3f sec" % age
        return True, "OK"

    @staticmethod
    def _matrix_from_transform(transform):
        translation = transform.translation
        rotation = transform.rotation
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w], dtype=float)
        x, y, z, w = q / np.linalg.norm(q)
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return matrix

    @staticmethod
    def _max_position_span(positions):
        if not positions:
            return 0.0
        points = np.array(positions, dtype=float)
        max_span = 0.0
        for index in range(len(points)):
            distances = np.linalg.norm(points[index + 1 :] - points[index], axis=1)
            if len(distances):
                max_span = max(max_span, float(np.max(distances)))
        return max_span

    def _write_report(self, report):
        os.makedirs(self.output_dir, exist_ok=True)
        filename = "handeye_invariance_%s.yaml" % time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.output_dir, filename)
        with open(path, "w") as handle:
            yaml.safe_dump(report, handle, default_flow_style=False, sort_keys=False)
        return path


if __name__ == "__main__":
    try:
        HandeyeInvarianceVerifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
