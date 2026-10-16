#!/usr/bin/env python3
import math
import sys
import threading
import tkinter as tk
from tkinter import ttk

import moveit_commander
import rospy


class JointJogGui:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("alicia_joint_jog_gui", anonymous=True)

        self.group = moveit_commander.MoveGroupCommander("alicia")
        self.group.set_max_velocity_scaling_factor(0.12)
        self.group.set_max_acceleration_scaling_factor(0.08)
        self.group.set_planning_time(3.0)
        self.group.set_num_planning_attempts(3)
        self.group.allow_replanning(True)

        self.busy = False
        self.root = tk.Tk()
        self.root.title("Alicia-D Joint Jog")

        self.step_deg = tk.DoubleVar(value=2.0)
        self.status = tk.StringVar(value="Ready")
        self.joint_values = [tk.StringVar(value="-") for _ in range(6)]

        self._build_ui()
        self.refresh()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="nsew")

        ttk.Label(top, text="Step deg").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(top, from_=0.2, to=10.0, increment=0.2, textvariable=self.step_deg, width=6).grid(row=0, column=1)
        ttk.Button(top, text="Refresh", command=self.refresh).grid(row=0, column=2, padx=4)

        for i in range(6):
            row = i + 1
            ttk.Label(top, text=f"J{i + 1}").grid(row=row, column=0, sticky="w", pady=2)
            ttk.Button(top, text="-", width=4, command=lambda idx=i: self.jog(idx, -1)).grid(row=row, column=1, padx=2)
            ttk.Button(top, text="+", width=4, command=lambda idx=i: self.jog(idx, 1)).grid(row=row, column=2, padx=2)
            ttk.Label(top, textvariable=self.joint_values[i], width=10).grid(row=row, column=3, padx=6)

        ttk.Button(top, text="Stop", command=self.stop).grid(row=7, column=0, pady=8, sticky="ew")
        ttk.Label(top, textvariable=self.status, width=34).grid(row=7, column=1, columnspan=3, sticky="w")

    def refresh(self):
        try:
            vals = self.group.get_current_joint_values()[:6]
            for i, value in enumerate(vals):
                self.joint_values[i].set(f"{math.degrees(value):.1f}")
            self.status.set("Ready")
        except Exception as exc:
            self.status.set(f"Refresh failed: {exc}")

    def stop(self):
        try:
            self.group.stop()
            self.status.set("Stopped")
        except Exception as exc:
            self.status.set(f"Stop failed: {exc}")

    def jog(self, joint_index, direction):
        if self.busy:
            return

        def worker():
            self.busy = True
            try:
                step = math.radians(float(self.step_deg.get())) * direction
                target = self.group.get_current_joint_values()
                target[joint_index] += step
                self.status.set(f"Moving J{joint_index + 1}...")
                self.group.set_joint_value_target(target)
                ok = self.group.go(wait=True)
                self.group.stop()
                self.status.set("Move OK" if ok else "Move failed")
            except Exception as exc:
                self.status.set(f"Move failed: {exc}")
            finally:
                self.busy = False
                self.root.after(0, self.refresh)

        threading.Thread(target=worker, daemon=True).start()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    JointJogGui().run()
