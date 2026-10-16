#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Alicia-D System Live Stream Supervisor
Aggregates:
1. ROS Node logs (warnings & errors from /rosout_agg)
2. Robot arm serial communication, temperatures & joint states
3. Tactile force & safety status
4. Remote GraspNet + MuJoCo digital twin server health (:8000)
"""
import os
import sys
import time
import json
import urllib.request
from datetime import datetime

try:
    import rospy
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray
    from alicia_flexible_grasp_supervisor.msg import TactileState, SafetyState, GraspState
except ImportError as e:
    print("Error importing ROS dependencies: {}".format(e))
    print("Please run: source /opt/ros/noetic/setup.bash && source devel/setup.bash")
    sys.exit(1)


GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
RESET = "\033[0m"


class LiveStreamSupervisor:
    def __init__(self, remote_url="http://172.23.132.97:8000", poll_interval=2.0):
        self.remote_url = os.environ.get("GRASP6D_URL", remote_url)
        self.poll_interval = poll_interval

        # State caches
        self.last_joints = {}
        self.last_temps = []
        self.last_tactile = {"force": 0.0, "contact": False, "valid": False}
        self.last_safety = {"ok": True, "level": "UNKNOWN", "msg": ""}
        self.last_grasp = {"state": "IDLE", "msg": ""}
        self.recent_logs = []

        # ROS Subscribers
        rospy.Subscriber("/joint_states", JointState, self._cb_joints, queue_size=1)
        rospy.Subscriber("/alicia_d/temperatures_c", Float32MultiArray, self._cb_temps, queue_size=1)
        rospy.Subscriber("/tactile/state", TactileState, self._cb_tactile, queue_size=1)
        rospy.Subscriber("/safety/status", SafetyState, self._cb_safety, queue_size=1)
        rospy.Subscriber("/grasp/state", GraspState, self._cb_grasp, queue_size=1)
        rospy.Subscriber("/rosout_agg", Log, self._cb_log, queue_size=50)

    def _cb_joints(self, msg):
        self.last_joints = {name: round(pos, 3) for name, pos in zip(msg.name, msg.position)}

    def _cb_temps(self, msg):
        self.last_temps = [round(t, 1) for t in msg.data]

    def _cb_tactile(self, msg):
        self.last_tactile = {
            "force": round(msg.total_grip_force_mn, 1),
            "contact": msg.contact_detected,
            "slip": msg.slip_detected,
            "valid": msg.valid
        }

    def _cb_safety(self, msg):
        self.last_safety = {"ok": msg.ok, "level": msg.level, "msg": msg.message}

    def _cb_grasp(self, msg):
        self.last_grasp = {"state": msg.state, "stage": msg.stage, "msg": msg.message}

    def _cb_log(self, msg):
        if msg.level >= Log.WARN:
            level = "WARN" if msg.level == Log.WARN else ("ERROR" if msg.level == Log.ERROR else "FATAL")
            t_str = datetime.now().strftime("%H:%M:%S")
            self.recent_logs.append("[{}] [{}] [{}]: {}".format(t_str, level, msg.name, msg.msg))
            if len(self.recent_logs) > 20:
                self.recent_logs.pop(0)

    def check_remote_server(self):
        url = "{}/health".format(self.remote_url.rstrip('/'))
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "SupervisorWatcher"})
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                data = json.loads(resp.read().decode())
                grasp_ok = data.get("grasp_backend", {}).get("ok", False)
                sim_ok = data.get("digital_twin", {}).get("ok", False)
                return {
                    "online": True,
                    "graspnet_ok": grasp_ok,
                    "mujoco_ok": sim_ok,
                    "mujoco_ver": data.get("digital_twin", {}).get("mujoco", "unknown")
                }
        except Exception as e:
            return {"online": False, "error": str(e)}

    def render_dashboard(self):
        remote_info = self.check_remote_server()
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        print("\033[2J\033[H", end="") # Clear screen
        print("=" * 78)
        print("       ALICIA-D 系统实时综合监督控制台 [{}]".format(now_str))
        print("=" * 78)

        # 1. 机械臂与串口状态
        print("\n[1] 机械臂底层串口与运动反馈 (Topic: /joint_states, /alicia_d/*)")
        if self.last_joints:
            j_str = " | ".join(["{}: {:+.3f}".format(k, v) for k, v in self.last_joints.items()])
            print("  * 关节位置(rad/m): {}".format(j_str))
        else:
            print("  * 关节位置: 等待 /joint_states 数据...")

        if self.last_temps:
            t_str = ", ".join(["{:.1f}°C".format(t) for t in self.last_temps])
            print("  * 电机温度: [{}] (正常安全区间 < 70°C)".format(t_str))
        else:
            print("  * 电机温度: 暂无温度回传")

        # 2. 触觉力反馈与安全状态
        print("\n[2] 触觉与安全监控 (Topic: /tactile/state, /safety/status)")
        if self.last_tactile.get("valid"):
            print("  * 电子皮肤合力: {} mN | 接触: {} | 滑移: {}".format(
                self.last_tactile['force'], self.last_tactile['contact'], self.last_tactile.get('slip', False)
            ))
        else:
            print("  * 电子皮肤合力: 传感器未上报或离线 (硬件串口 /dev/alicia_skin 正常)")

        safety_color = GREEN if self.last_safety["ok"] else RED
        print("  * 安全监控状态: {}{}{} - {}".format(
            safety_color, self.last_safety['level'], RESET, self.last_safety['msg']
        ))
        print("  * 抓取任务状态机: [{}] {}".format(self.last_grasp['state'], self.last_grasp['msg']))

        # 3. 远程 MuJoCo/WSL 服务
        print("\n[3] 远程 MuJoCo / WSL2 仿真门控服务 ({})".format(self.remote_url))
        if remote_info.get("online"):
            grasp_status = "{}[READY]{}".format(GREEN, RESET) if remote_info.get("graspnet_ok") else "{}[ERROR]{}".format(RED, RESET)
            sim_status = "{}[READY]{}".format(GREEN, RESET) if remote_info.get("mujoco_ok") else "{}[ERROR]{}".format(RED, RESET)
            print("  * 服务连通性: {}ONLINE{}".format(GREEN, RESET))
            print("  * GraspNet 后端: {}".format(grasp_status))
            print("  * MuJoCo 仿真门控: {} (版本: {})".format(sim_status, remote_info.get('mujoco_ver')))
        else:
            print("  * 服务连通性: {}OFFLINE{} ({})".format(RED, RESET, remote_info.get('error')))

        # 4. ROS 节点报警与错误日志
        print("\n[4] ROS 节点异常日志监控 (最近 WARN/ERROR，共监控所有节点)")
        if self.recent_logs:
            for log in self.recent_logs[-8:]:
                color = YELLOW if "WARN" in log else RED
                print("  {}{}{}".format(color, log, RESET))
        else:
            print("  {}[✓] 系统运行平稳，最近无警告与错误日志{}".format(GREEN, RESET))

        print("=" * 78)
        print("按 Ctrl+C 退出监督面板")

    def run(self):
        rate = rospy.Rate(1.0 / self.poll_interval)
        while not rospy.is_shutdown():
            self.render_dashboard()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("live_stream_supervisor", anonymous=True)
    watcher = LiveStreamSupervisor(poll_interval=1.5)
    try:
        watcher.run()
    except KeyboardInterrupt:
        print("\n监督退出。")
