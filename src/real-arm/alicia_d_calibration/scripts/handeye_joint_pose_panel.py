#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal joint pose panel for manual hand-eye verification poses.

This panel is only a small jog and sample helper for operator-guided
calibration poses.
"""

import json
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

from easy_handeye_msgs.srv import TakeSample
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


ARM_JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
DEFAULT_JOINT_MIN_RAD = [-3.14, -2.5, -2.5, -3.14, -2.5, -3.14]
DEFAULT_JOINT_MAX_RAD = [3.14, 2.5, 2.5, 3.14, 2.5, 3.14]
DEFAULT_STEP_DEG = 2.0
DEFAULT_MIN_STEP_DEG = 0.1
DEFAULT_MAX_STEP_DEG = 10.0
DEFAULT_DRIVER_SYNC_SETTLE_SEC = 0.15
FULLWIDTH_DIGIT_TRANSLATION = str.maketrans(
    {
        "０": "0",
        "１": "1",
        "２": "2",
        "３": "3",
        "４": "4",
        "５": "5",
        "６": "6",
        "７": "7",
        "８": "8",
        "９": "9",
        "．": ".",
        "。": ".",
        "，": ",",
        "＋": "+",
        "－": "-",
    }
)


def extract_joint_snapshot(names, positions):
    """Return six arm joints and the optional gripper from a JointState."""
    values = list(positions)
    name_to_position = dict(zip(names, values))
    arm = []
    for index, name in enumerate(ARM_JOINT_NAMES):
        if names:
            value = name_to_position.get(name)
        else:
            value = values[index] if index < len(values) else None
        if value is None or not math.isfinite(float(value)):
            return None, None
        arm.append(float(value))
    gripper = name_to_position.get(
        "right_finger", values[6] if len(values) > 6 else None
    )
    if gripper is not None:
        gripper = float(gripper)
        if not math.isfinite(gripper):
            gripper = None
    return arm, gripper


def make_jog_target(current, joint_index, step_rad, lower, upper):
    """Create one bounded single-joint target from measured feedback."""
    if len(current) != 6 or not 0 <= joint_index < 6:
        raise ValueError("six current arm joints and a valid joint index are required")
    target = list(current)
    requested = target[joint_index] + float(step_rad)
    target[joint_index] = max(float(lower[joint_index]), min(float(upper[joint_index]), requested))
    return target, target[joint_index] != requested


def parse_step_degrees(value, lower=DEFAULT_MIN_STEP_DEG, upper=DEFAULT_MAX_STEP_DEG):
    """Parse the operator-entered step size and clamp it to panel limits."""
    text = str(value).strip().translate(FULLWIDTH_DIGIT_TRANSLATION)
    text = text.replace("°", "").replace("度", "")
    text = text.replace("deg", "").replace("DEG", "")
    text = text.replace(" ", "").replace("\t", "")
    if "," in text and "." not in text:
        text = text.replace(",", ".")
    if not text:
        raise ValueError("Step deg is empty")
    step = float(text)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("Step deg must be a positive number")
    lower = float(lower)
    upper = float(upper)
    if not math.isfinite(lower) or not math.isfinite(upper) or lower <= 0.0 or upper < lower:
        raise ValueError("invalid step limits")
    clamped = False
    if step < lower:
        step = lower
        clamped = True
    elif step > upper:
        step = upper
        clamped = True
    return step, clamped


def format_step_degrees(value):
    return ("%.3f" % float(value)).rstrip("0").rstrip(".")


def actuation_status_needs_driver_sync(value):
    text = str(value)
    return not (
        text.startswith("CONFIRMED:")
        or "COMMAND_SYNCHRONIZED" in text
        or "MEASURED_DIRECTIONAL_RESPONSE" in text
    )


class HandeyeJointPosePanel:
    def __init__(self):
        rospy.init_node("handeye_joint_pose_panel", anonymous=True)

        self.calibration_namespace = rospy.get_param(
            "~calibration_namespace",
            "/d405_v4l2_charuco_recalib_20260725_eye_on_hand",
        ).rstrip("/")
        self.quality_topic = rospy.get_param("~quality_topic", "/charuco/quality")
        self.max_quality_age_ms = float(rospy.get_param("~max_quality_age_ms", 500.0))
        self.min_charuco_corners = int(rospy.get_param("~min_charuco_corners", 35))
        self.min_edge_clearance_px = float(rospy.get_param("~min_edge_clearance_px", 12.0))
        self.max_reprojection_rms_px = float(rospy.get_param("~max_reprojection_rms_px", 1.5))
        self.stable_window_sec = float(rospy.get_param("~stable_window_sec", 2.0))
        self.stable_required_hits = int(rospy.get_param("~stable_required_hits", 4))
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")
        self.joint_command_topic = rospy.get_param("~joint_command_topic", "/joint_commands")
        self.feedback_max_age_sec = float(rospy.get_param("~feedback_max_age_sec", 2.0))
        self.motion_response_timeout_sec = float(
            rospy.get_param("~motion_response_timeout_sec", 2.0)
        )
        self.measured_response_min_delta_rad = float(
            rospy.get_param("~measured_response_min_delta_rad", 0.003)
        )
        self.driver_sync_settle_sec = max(
            0.0,
            float(
                rospy.get_param(
                    "~driver_sync_settle_sec", DEFAULT_DRIVER_SYNC_SETTLE_SEC
                )
            ),
        )
        self.min_step_deg = float(rospy.get_param("~min_step_deg", DEFAULT_MIN_STEP_DEG))
        self.max_step_deg = float(rospy.get_param("~max_step_deg", DEFAULT_MAX_STEP_DEG))
        default_step_deg = float(rospy.get_param("~default_step_deg", DEFAULT_STEP_DEG))
        default_step_deg, _ = parse_step_degrees(
            default_step_deg, self.min_step_deg, self.max_step_deg
        )
        self.joint_min_rad = list(
            rospy.get_param("/robot/joint_min_rad", DEFAULT_JOINT_MIN_RAD)
        )
        self.joint_max_rad = list(
            rospy.get_param("/robot/joint_max_rad", DEFAULT_JOINT_MAX_RAD)
        )
        if len(self.joint_min_rad) != 6 or len(self.joint_max_rad) != 6:
            raise ValueError("/robot joint limits must each contain six values")

        self.busy = False
        self.stability_busy = False
        self.saving_busy = False
        self.can_save = False
        self.latest_quality = None
        self.quality_lock = threading.Lock()
        self.joint_lock = threading.Lock()
        self.latest_arm_joints = None
        self.latest_gripper = None
        self.latest_joint_monotonic = 0.0
        self.command_target = None
        self.command_gripper = None
        self.driver_sync_needed = True
        self.last_driver_sync_monotonic = 0.0
        self.take_sample_service = self.calibration_namespace + "/take_sample"
        self.get_sample_service = self.calibration_namespace + "/get_sample_list"
        self.root = tk.Tk()
        self.root.title("Hand-eye Pose Jog")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # 2 deg is above the driver's 0.02 rad post-enable response probe.
        self.step_deg_text = tk.StringVar(value=format_step_degrees(default_step_deg))
        self.status = tk.StringVar(value="等待最新关节反馈")
        self.actuation_status = tk.StringVar(value="使能确认: 等待状态")
        self.quality_status = tk.StringVar(value="Quality: waiting for ChArUco")
        self.save_status = tk.StringVar(value="Can save: no")
        self.sample_status = tk.StringVar(value="Samples: -")
        self.joint_values = [tk.StringVar(value="-") for _ in range(6)]

        self.command_pub = rospy.Publisher(
            self.joint_command_topic, JointState, queue_size=2
        )
        self.joint_sub = rospy.Subscriber(
            self.joint_state_topic, JointState, self._joint_state_cb, queue_size=1
        )
        self.actuation_sub = rospy.Subscriber(
            "/alicia_d/actuation_status", String, self._actuation_cb, queue_size=1
        )
        self.quality_sub = rospy.Subscriber(
            self.quality_topic, String, self._quality_cb, queue_size=1
        )
        self._build_ui()
        self._refresh_sample_count_async()
        self._refresh_quality_display()
        self._refresh_joint_display()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="nsew")

        ttk.Label(top, text="Step deg").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(
            top,
            from_=self.min_step_deg,
            to=self.max_step_deg,
            increment=0.2,
            textvariable=self.step_deg_text,
            width=8,
        ).grid(row=0, column=1)
        ttk.Button(top, text="刷新反馈", command=self.refresh).grid(row=0, column=2, padx=4)
        self.sync_btn = ttk.Button(top, text="同步当前关节", command=self.sync_current_joints)
        self.sync_btn.grid(row=0, column=3, padx=4)

        for index in range(6):
            row = index + 1
            ttk.Label(top, text="J%d" % (index + 1)).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Button(top, text="-", width=4, command=lambda i=index: self.jog(i, -1)).grid(row=row, column=1, padx=2)
            ttk.Button(top, text="+", width=4, command=lambda i=index: self.jog(i, 1)).grid(row=row, column=2, padx=2)
            ttk.Label(top, textvariable=self.joint_values[index], width=10).grid(row=row, column=3, padx=6)

        ttk.Label(
            top,
            textvariable=self.status,
            width=42,
        ).grid(row=7, column=0, columnspan=4, pady=8, sticky="w")
        ttk.Label(top, textvariable=self.actuation_status, width=58).grid(
            row=8, column=0, columnspan=4, sticky="w"
        )

        ttk.Separator(top, orient="horizontal").grid(row=9, column=0, columnspan=4, sticky="ew", pady=4)
        self.stable_btn = ttk.Button(top, text="已稳定", command=self.check_stable)
        self.stable_btn.grid(row=10, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
        self.save_btn = ttk.Button(top, text="保存", command=self.save_sample)
        self.save_btn.grid(row=10, column=2, columnspan=2, sticky="ew", padx=2, pady=2)
        self.save_btn.state(["disabled"])
        ttk.Label(top, textvariable=self.save_status, width=42).grid(row=11, column=0, columnspan=4, sticky="w")
        ttk.Label(top, textvariable=self.sample_status, width=42).grid(row=12, column=0, columnspan=4, sticky="w")
        ttk.Label(top, textvariable=self.quality_status, width=58).grid(row=13, column=0, columnspan=4, sticky="w")

    def refresh(self):
        snapshot = self._fresh_joint_snapshot()
        if snapshot is None:
            self.status.set("刷新失败：没有新鲜的 /joint_states")
            return
        arm, _gripper = snapshot
        for index, value in enumerate(arm):
            self.joint_values[index].set("%.1f" % math.degrees(value))
        self.status.set("反馈已刷新")

    def _joint_state_cb(self, msg):
        arm, gripper = extract_joint_snapshot(msg.name, msg.position)
        if arm is None:
            return
        first_feedback = False
        with self.joint_lock:
            self.latest_arm_joints = arm
            self.latest_gripper = gripper
            self.latest_joint_monotonic = time.monotonic()
            # First valid feedback is a non-commanding synchronization only.
            if self.command_target is None:
                self.command_target = list(arm)
                self.command_gripper = gripper
                first_feedback = True
        if first_feedback:
            self.root.after(
                0,
                lambda: self.status.set(
                    "当前关节已自动同步；关节按钮使用 /joint_commands 直控"
                ),
            )

    def _actuation_cb(self, msg):
        value = str(msg.data)
        with self.joint_lock:
            self.driver_sync_needed = actuation_status_needs_driver_sync(value)
        self.root.after(0, lambda text=value: self.actuation_status.set("使能确认: " + text))

    def _fresh_joint_snapshot(self):
        with self.joint_lock:
            if self.latest_arm_joints is None:
                return None
            if time.monotonic() - self.latest_joint_monotonic > self.feedback_max_age_sec:
                return None
            return list(self.latest_arm_joints), self.latest_gripper

    def sync_current_joints(self):
        snapshot = self._fresh_joint_snapshot()
        if snapshot is None:
            self.status.set("同步失败：没有新鲜的 /joint_states")
            return
        arm, gripper = snapshot
        with self.joint_lock:
            self.command_target = list(arm)
            self.command_gripper = gripper
        for index, value in enumerate(arm):
            self.joint_values[index].set("%.1f" % math.degrees(value))
        if self._publish_driver_sync_baseline(arm, gripper, "operator_sync"):
            self.status.set("已同步当前关节，并发送当前位置基线到驱动")
        else:
            self.status.set("已同步当前关节；/joint_commands 暂无驱动连接")

    def _make_joint_command(self, arm, gripper):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = list(arm)
        if gripper is not None:
            msg.name.append("right_finger")
            msg.position.append(gripper)
        return msg

    def _publish_driver_sync_baseline(self, arm, gripper, reason):
        if self.command_pub.get_num_connections() < 1:
            return False
        msg = self._make_joint_command(arm, gripper)
        rospy.loginfo(
            "Panel publishing no-motion driver sync baseline: reason=%s target_deg=%s",
            reason,
            ["%.2f" % math.degrees(value) for value in arm],
        )
        self.command_pub.publish(msg)
        with self.joint_lock:
            self.command_target = list(arm)
            self.command_gripper = gripper
            self.driver_sync_needed = False
            self.last_driver_sync_monotonic = time.monotonic()
        return True

    def _refresh_joint_display(self):
        snapshot = self._fresh_joint_snapshot()
        if snapshot is not None:
            arm, _gripper = snapshot
            for index, value in enumerate(arm):
                self.joint_values[index].set("%.1f" % math.degrees(value))
        if not rospy.is_shutdown():
            self.root.after(250, self._refresh_joint_display)

    def jog(self, joint_index, direction):
        if self.busy:
            self.status.set("上一条关节直控命令仍在等待编码器响应")
            return
        self._set_can_save(False, "Can save: no, pose changed")

        snapshot = self._fresh_joint_snapshot()
        if snapshot is None:
            self.status.set("未发送：没有新鲜的 /joint_states")
            return
        if self.command_pub.get_num_connections() < 1:
            self.status.set("未发送：/joint_commands 当前没有驱动订阅连接")
            return
        try:
            step_deg, step_clamped = parse_step_degrees(
                self.step_deg_text.get(), self.min_step_deg, self.max_step_deg
            )
            if step_clamped:
                self.step_deg_text.set(format_step_degrees(step_deg))
            step = math.radians(step_deg) * direction
            baseline, gripper = snapshot
            target, clamped = make_jog_target(
                baseline,
                joint_index,
                step,
                self.joint_min_rad,
                self.joint_max_rad,
            )
        except Exception as exc:
            self.status.set("未发送：%s" % exc)
            return
        actual_step = target[joint_index] - baseline[joint_index]
        if abs(actual_step) < 1e-9:
            self.status.set("未发送：J%d 已到关节限位" % (joint_index + 1))
            return
        with self.joint_lock:
            driver_sync_needed = self.driver_sync_needed

        def worker():
            try:
                if driver_sync_needed:
                    self._publish_driver_sync_baseline(
                        baseline, gripper, "pre_jog_reconnect_sync"
                    )
                    if self.driver_sync_settle_sec > 0.0:
                        time.sleep(self.driver_sync_settle_sec)
                msg = self._make_joint_command(target, gripper)
                rospy.loginfo(
                    "Panel publishing joint jog: joint=J%d requested_step_deg=%.3f "
                    "actual_step_deg=%.3f target_deg=%s",
                    joint_index + 1,
                    step_deg * direction,
                    math.degrees(actual_step),
                    ["%.2f" % math.degrees(value) for value in target],
                )
                self.command_pub.publish(msg)
                with self.joint_lock:
                    self.command_target = list(target)
                    self.command_gripper = gripper
                suffixes = []
                if step_clamped:
                    suffixes.append("步长已限制到 %.1f°" % step_deg)
                if clamped:
                    suffixes.append("已限制到关节限位")
                suffix = "（%s）" % "，".join(suffixes) if suffixes else ""
                self.root.after(
                    0,
                    lambda: self.status.set(
                        "已发送 J%d %+.1f° 到 /joint_commands%s；等待编码器响应"
                        % (joint_index + 1, math.degrees(actual_step), suffix)
                    ),
                )
                deadline = time.monotonic() + self.motion_response_timeout_sec
                responded = False
                measured_delta = 0.0
                while time.monotonic() < deadline and not rospy.is_shutdown():
                    current_snapshot = self._fresh_joint_snapshot()
                    if current_snapshot is not None:
                        current_arm, _ = current_snapshot
                        measured_delta = current_arm[joint_index] - baseline[joint_index]
                        if (
                            abs(measured_delta) >= self.measured_response_min_delta_rad
                            and measured_delta * actual_step > 0.0
                        ):
                            responded = True
                            break
                    time.sleep(0.05)
                if responded:
                    self.root.after(
                        0,
                        lambda: self.status.set(
                            "J%d 编码器已按命令方向响应 %+.2f°"
                            % (joint_index + 1, math.degrees(measured_delta))
                        ),
                    )
                else:
                    self.root.after(
                        0,
                        lambda: self.status.set(
                            "J%d 命令已发送，但 %.1fs 内编码器无方向响应"
                            % (joint_index + 1, self.motion_response_timeout_sec)
                        ),
                    )
            except Exception as exc:
                self.root.after(
                    0, lambda error=exc: self.status.set("直控发送失败：%s" % error)
                )
            finally:
                self.busy = False

        self.busy = True
        threading.Thread(target=worker, daemon=True).start()

    def _quality_cb(self, msg):
        try:
            quality = json.loads(msg.data)
        except Exception:
            return
        with self.quality_lock:
            self.latest_quality = quality

    def _quality_snapshot(self):
        with self.quality_lock:
            return dict(self.latest_quality) if self.latest_quality else None

    def _quality_gate(self):
        quality = self._quality_snapshot()
        if not quality:
            return False, "waiting for ChArUco", {}
        stamp = float(quality.get("stamp") or 0.0)
        age_ms = (rospy.Time.now().to_sec() - stamp) * 1000.0 if stamp > 0.0 else 1e9
        corners = int(quality.get("charuco_count") or 0)
        markers = int(quality.get("marker_count") or 0)
        edge = quality.get("edge_clearance_px")
        reproj = quality.get("reprojection_rms_px")
        edge_value = float(edge) if edge is not None else -1.0
        reproj_value = float(reproj) if reproj is not None else -1.0
        detail = {
            "age_ms": age_ms,
            "corners": corners,
            "markers": markers,
            "edge": edge_value,
            "reproj": reproj_value,
        }
        if not bool(quality.get("pose_ok")):
            return False, "ChArUco not visible", detail
        if age_ms > self.max_quality_age_ms:
            return False, "quality frame is stale", detail
        if corners < self.min_charuco_corners:
            return False, "not enough corners", detail
        if edge_value < self.min_edge_clearance_px:
            return False, "board too close to image edge", detail
        if reproj_value < 0.0 or reproj_value > self.max_reprojection_rms_px:
            return False, "reprojection error too high", detail
        return True, "OK", detail

    def _format_quality(self, ok, reason, detail):
        if not detail:
            return "Quality: %s" % reason
        return (
            "Quality: %s | age %.0fms corners %d markers %d edge %.1fpx reproj %.3fpx"
            % (
                "OK" if ok else reason,
                detail["age_ms"],
                detail["corners"],
                detail["markers"],
                detail["edge"],
                detail["reproj"],
            )
        )

    def _refresh_quality_display(self):
        ok, reason, detail = self._quality_gate()
        self.quality_status.set(self._format_quality(ok, reason, detail))
        if not self.can_save and not self.stability_busy:
            self.save_status.set("Can save: no" if ok else "Can save: no, %s" % reason)
        self.root.after(500, self._refresh_quality_display)

    def _set_can_save(self, enabled, text):
        self.can_save = bool(enabled)
        self.save_status.set(text)
        if enabled:
            self.save_btn.state(["!disabled"])
        else:
            self.save_btn.state(["disabled"])

    def check_stable(self):
        if self.stability_busy:
            return
        self._set_can_save(False, "Can save: checking stability")

        def worker():
            self.stability_busy = True
            self.root.after(0, lambda: self.stable_btn.state(["disabled"]))
            hits = 0
            deadline = time.monotonic() + self.stable_window_sec
            last_text = "waiting"
            while time.monotonic() < deadline and not rospy.is_shutdown():
                ok, reason, detail = self._quality_gate()
                hits = hits + 1 if ok else 0
                last_text = self._format_quality(ok, reason, detail)
                self.root.after(0, lambda h=hits, t=last_text: self.save_status.set(
                    "Can save: checking %d/%d | %s" % (h, self.stable_required_hits, t)
                ))
                if hits >= self.stable_required_hits:
                    self.root.after(0, lambda: self._set_can_save(True, "Can save: yes"))
                    self.root.after(0, lambda: self.status.set("Stable quality confirmed"))
                    break
                time.sleep(0.25)
            else:
                self.root.after(0, lambda t=last_text: self._set_can_save(False, "Can save: no | %s" % t))
                self.root.after(0, lambda: self.status.set("Stable check failed"))
            self.stability_busy = False
            self.root.after(0, lambda: self.stable_btn.state(["!disabled"]))

        threading.Thread(target=worker, daemon=True).start()

    def save_sample(self):
        if self.saving_busy:
            return
        if not self.can_save:
            self.status.set("Not allowed to save yet")
            return

        def worker():
            self.saving_busy = True
            self.root.after(0, lambda: self.save_btn.state(["disabled"]))
            self.root.after(0, lambda: self.status.set("Saving calibration sample..."))
            try:
                rospy.wait_for_service(self.take_sample_service, timeout=2.0)
                proxy = rospy.ServiceProxy(self.take_sample_service, TakeSample)
                response = proxy()
                count = self._sample_count(response.samples)
                self.root.after(0, lambda c=count: self.sample_status.set("Samples: %d" % c))
                self.root.after(0, lambda c=count: self.status.set("Saved sample %d" % c))
                self.root.after(0, lambda: self._set_can_save(False, "Can save: no, move to next pose"))
            except Exception as exc:
                self.root.after(0, lambda e=exc: self.status.set("Save failed: %s" % e))
                self.root.after(0, lambda: self._set_can_save(False, "Can save: no, save failed"))
            finally:
                self.saving_busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _sample_count(self, samples):
        hand_samples = getattr(samples, "hand_world_samples", [])
        return len(hand_samples)

    def _refresh_sample_count_async(self):
        def worker():
            try:
                rospy.wait_for_service(self.get_sample_service, timeout=1.0)
                proxy = rospy.ServiceProxy(self.get_sample_service, TakeSample)
                response = proxy()
                count = self._sample_count(response.samples)
                self.root.after(0, lambda: self.sample_status.set("Samples: %d" % count))
            except Exception:
                self.root.after(0, lambda: self.sample_status.set("Samples: service not ready"))

        threading.Thread(target=worker, daemon=True).start()

    def run(self):
        self.root.mainloop()

    def close(self):
        # Closing this helper unregisters ROS endpoints only. It deliberately
        # publishes no stop, controller switch, torque-off, or disable command.
        for subscriber in (self.joint_sub, self.actuation_sub, self.quality_sub):
            try:
                subscriber.unregister()
            except Exception:
                pass
        try:
            self.command_pub.unregister()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    HandeyeJointPosePanel().run()
