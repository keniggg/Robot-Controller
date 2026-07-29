#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal joint pose panel for manual hand-eye verification poses.

This panel is only a small jog and sample helper for operator-guided
calibration poses.
"""

import json
import math
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

from easy_handeye_msgs.srv import TakeSample
import moveit_commander
import rospy
from std_msgs.msg import String


class HandeyeJointPosePanel:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("handeye_joint_pose_panel", anonymous=True)

        group_name = rospy.get_param("~group", "alicia")
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_max_velocity_scaling_factor(float(rospy.get_param("~velocity_scale", 0.08)))
        self.group.set_max_acceleration_scaling_factor(float(rospy.get_param("~accel_scale", 0.06)))
        self.group.set_planning_time(float(rospy.get_param("~planning_time", 3.0)))
        self.group.set_num_planning_attempts(int(rospy.get_param("~planning_attempts", 3)))
        self.group.allow_replanning(False)

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

        self.busy = False
        self.stability_busy = False
        self.saving_busy = False
        self.can_save = False
        self.latest_quality = None
        self.quality_lock = threading.Lock()
        self.take_sample_service = self.calibration_namespace + "/take_sample"
        self.get_sample_service = self.calibration_namespace + "/get_sample_list"
        self.root = tk.Tk()
        self.root.title("Hand-eye Pose Jog")
        self.root.protocol("WM_DELETE_WINDOW", self.root.destroy)

        self.step_deg = tk.DoubleVar(value=1.0)
        self.status = tk.StringVar(value="Ready")
        self.quality_status = tk.StringVar(value="Quality: waiting for ChArUco")
        self.save_status = tk.StringVar(value="Can save: no")
        self.sample_status = tk.StringVar(value="Samples: -")
        self.joint_values = [tk.StringVar(value="-") for _ in range(6)]

        self.quality_sub = rospy.Subscriber(self.quality_topic, String, self._quality_cb, queue_size=1)
        self._build_ui()
        self.refresh()
        self._refresh_sample_count_async()
        self._refresh_quality_display()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="nsew")

        ttk.Label(top, text="Step deg").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(top, from_=0.2, to=3.0, increment=0.2, textvariable=self.step_deg, width=6).grid(row=0, column=1)
        ttk.Button(top, text="Refresh", command=self.refresh).grid(row=0, column=2, padx=4)

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

        ttk.Separator(top, orient="horizontal").grid(row=8, column=0, columnspan=4, sticky="ew", pady=4)
        self.stable_btn = ttk.Button(top, text="已稳定", command=self.check_stable)
        self.stable_btn.grid(row=9, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
        self.save_btn = ttk.Button(top, text="保存", command=self.save_sample)
        self.save_btn.grid(row=9, column=2, columnspan=2, sticky="ew", padx=2, pady=2)
        self.save_btn.state(["disabled"])
        ttk.Label(top, textvariable=self.save_status, width=42).grid(row=10, column=0, columnspan=4, sticky="w")
        ttk.Label(top, textvariable=self.sample_status, width=42).grid(row=11, column=0, columnspan=4, sticky="w")
        ttk.Label(top, textvariable=self.quality_status, width=58).grid(row=12, column=0, columnspan=4, sticky="w")

    def refresh(self):
        try:
            values = self.group.get_current_joint_values()[:6]
            for index, value in enumerate(values):
                self.joint_values[index].set("%.1f" % math.degrees(value))
            self.status.set("Ready")
        except Exception as exc:
            self.status.set("Refresh failed: %s" % exc)

    def jog(self, joint_index, direction):
        if self.busy:
            self.status.set("Busy")
            return
        self._set_can_save(False, "Can save: no, pose changed")

        def worker():
            self.busy = True
            try:
                step = math.radians(float(self.step_deg.get())) * direction
                target = self.group.get_current_joint_values()
                target[joint_index] += step
                self.status.set("Planning J%d %+.1f deg" % (joint_index + 1, math.degrees(step)))
                self.group.set_joint_value_target(target)
                ok = self.group.go(wait=True)
                self.status.set("Move OK" if ok else "Move failed")
            except Exception as exc:
                self.status.set("Move failed: %s" % exc)
            finally:
                self.busy = False
                self.root.after(0, self.refresh)

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


if __name__ == "__main__":
    HandeyeJointPosePanel().run()
