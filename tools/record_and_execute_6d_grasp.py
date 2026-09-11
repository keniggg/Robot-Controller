#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Alicia-D 6D Grasp Autonomous Executor & Full Telemetry Recorder
Records:
- Wall-clock timestamps (ISO 8601)
- Grasp state transitions (/grasp/state)
- Joint positions and velocities (/joint_states)
- Motor temperatures (/alicia_d/temperatures_c)
- MuJoCo Gate audit logs (/grasp_6d/gate_audit)
- Enriched plan details (/grasp_6d/plan_enriched)
- Preview plan details (/grasp_6d/preview_plan_enriched)
- Object detection updates (/perception/object)
- ROS system logs (/rosout_agg)
"""
import os
import sys
import time
import json
import threading
from datetime import datetime

import rospy
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan, GraspState, ObjectPose
from alicia_flexible_grasp_supervisor.srv import StartGrasp, TriggerZero

TELEMETRY_LOG_PATH = os.path.expanduser("~/.ros/grasp_execution_telemetry_latest.jsonl")


class GraspTelemetryExecutor:
    def __init__(self):
        rospy.init_node("grasp_telemetry_executor", anonymous=True)
        self.log_file = open(TELEMETRY_LOG_PATH, "w", encoding="utf-8")

        self.init_time = rospy.Time.now().to_sec()
        self.latest_plan = None
        self.latest_plan_id = None
        self.latest_preview = None
        self.latest_state = None
        self.is_done = False
        self.motion_started = False
        self.lock = threading.Lock()

        # Subscriptions
        rospy.Subscriber("/grasp/state", GraspState, self._cb_state, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self._cb_joints, queue_size=10)
        rospy.Subscriber("/alicia_d/temperatures_c", Float32MultiArray, self._cb_temps, queue_size=5)
        rospy.Subscriber("/grasp_6d/gate_audit", String, self._cb_audit, queue_size=10)
        rospy.Subscriber("/grasp_6d/plan_enriched", Grasp6DPlan, self._cb_plan, queue_size=5)
        rospy.Subscriber("/grasp_6d/preview_plan_enriched", Grasp6DPlan, self._cb_preview, queue_size=5)
        rospy.Subscriber("/perception/object", ObjectPose, self._cb_object, queue_size=5)
        rospy.Subscriber("/rosout_agg", Log, self._cb_rosout, queue_size=50)

        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] Telemetry recording to: {TELEMETRY_LOG_PATH}")

    def log_event(self, event_type, data):
        record = {
            "timestamp": time.time(),
            "iso_time": datetime.now().isoformat(),
            "type": event_type,
            "data": data
        }
        with self.lock:
            self.log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.log_file.flush()

    def _cb_state(self, msg):
        self.latest_state = (msg.stage, msg.state, msg.message)
        t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        print(f"[{t_str}] [GRASP_STATE] Stage={msg.stage} ({msg.state}): {msg.message}")
        self.log_event("grasp_state", {
            "stage": msg.stage,
            "state": msg.state,
            "message": msg.message
        })
        if msg.stage in (1, 2, 3, 4, 5):
            self.motion_started = True
        if msg.stage in (6, 7, 8):  # SUCCESS=6, FAILED=7, EMERGENCY_STOP=8
            self.is_done = True

    def _cb_joints(self, msg):
        positions = {name: round(pos, 4) for name, pos in zip(msg.name, msg.position)}
        self.log_event("joint_states", positions)

    def _cb_temps(self, msg):
        self.log_event("temperatures", [round(t, 1) for t in msg.data])

    def _cb_audit(self, msg):
        t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        try:
            audit_json = json.loads(msg.data)
            status = audit_json.get("status")
            promotion = audit_json.get("promotion")
            outcome = audit_json.get("outcome", {})
            tracking = audit_json.get("summary", {}).get("tracking_evidence", {})
            max_hits = tracking.get("max_hit_count", 0)
            req_hits = tracking.get("required_hits", 3)
            code = status or outcome.get("code")
            if code or max_hits:
                print(f"[{t_str}] [GATE_AUDIT] status={code} hits={max_hits}/{req_hits} promo={promotion.get('code') if isinstance(promotion, dict) else promotion}")
        except Exception:
            pass
        self.log_event("gate_audit", msg.data)

    def _cb_plan(self, msg):
        if not bool(getattr(msg, 'valid', False)):
            return
        now_sec = rospy.Time.now().to_sec()
        stamp_sec = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        age = now_sec - stamp_sec
        # Accept valid plan within validity window (under 110s)
        if age < 110.0:
            self.latest_plan = msg
            self.latest_plan_id = msg.plan_id
            t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"[{t_str}] [ENRICHED_PLAN] ===> VALID 6D PLAN COMMITTED: plan_id={msg.plan_id}, score={msg.score:.1f}, valid={msg.valid}, age={age:.2f}s")
            self.log_event("plan_enriched", {
                "plan_id": msg.plan_id,
                "score": msg.score,
                "valid": msg.valid,
                "open_width_m": msg.required_open_width_m,
                "poses_count": len(msg.poses.poses),
                "age_sec": age
            })

    def _cb_preview(self, msg):
        if not bool(getattr(msg, 'valid', False)):
            return
        now_sec = rospy.Time.now().to_sec()
        stamp_sec = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        age = now_sec - stamp_sec
        if age < 110.0:
            self.latest_preview = msg
            t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"[{t_str}] [PREVIEW_PLAN] preview plan_id={msg.plan_id}, score={msg.score:.1f}, age={age:.2f}s")
            self.log_event("preview_plan", {
                "plan_id": msg.plan_id,
                "score": msg.score,
                "age_sec": age
            })
            # 如果当前富计划尚未就绪或较旧，尝试触发权威晋升
            if self.latest_plan is None:
                try:
                    replan_srv = rospy.ServiceProxy("/grasp_6d/replan_execution", TriggerZero)
                    replan_srv(True)
                except Exception:
                    pass

    def _cb_object(self, msg):
        if bool(msg.detected):
            self.log_event("perception_object", {
                "detected": True,
                "label": msg.label,
                "confidence": round(msg.confidence, 3),
                "depth_m": round(msg.depth_m, 4),
                "base_x": round(msg.pose_base.pose.position.x, 4),
                "base_y": round(msg.pose_base.pose.position.y, 4),
                "base_z": round(msg.pose_base.pose.position.z, 4)
            })

    def _cb_rosout(self, msg):
        if msg.level >= Log.WARN:
            level_str = "WARN" if msg.level == Log.WARN else ("ERROR" if msg.level == Log.ERROR else "FATAL")
            t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"[{t_str}] [{level_str}] [{msg.name}]: {msg.msg}")
            self.log_event("rosout", {
                "level": level_str,
                "node": msg.name,
                "msg": msg.msg
            })

    def execute(self):
        print("\n" + "="*75)
        print("         ALICIA-D 6D 抓取任务自主执行与全数据流同步记录器")
        print("="*75)

        # 1. 检查当前是否已有 VALID 计划
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] [Step 1] 检查权威计划状态 (/grasp/current_plan)...")
        plan_id = None
        try:
            rospy.wait_for_service("/grasp/current_plan", timeout=3.0)
            cur_res = rospy.ServiceProxy("/grasp/current_plan", TriggerZero)(True)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] /grasp/current_plan 返回: success={cur_res.success}, message={cur_res.message}")
            if cur_res.success and "validation=VALID" in cur_res.message:
                parts = dict(kv.split("=") for kv in cur_res.message.split() if "=" in kv)
                plan_id = parts.get("plan_id")
                print(f"[{datetime.now().strftime('%H:%M:%S')}] 当前已存在通过门控且有效的执行计划: {plan_id}")
        except Exception as e:
            print(f"[WARN] 查询 /grasp/current_plan 异常: {e}")

        # 如果当前没有有效计划，则等待或请求晋升
        if not plan_id:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 请求 /grasp_6d/replan_execution 促进权威晋升...")
            try:
                rospy.wait_for_service("/grasp_6d/replan_execution", timeout=3.0)
                replan_res = rospy.ServiceProxy("/grasp_6d/replan_execution", TriggerZero)(True)
                print(f"[{datetime.now().strftime('%H:%M:%S')}] replan_execution 响应: success={replan_res.success}, message={replan_res.message}")
            except Exception as e:
                print(f"[WARN] replan_execution 异常: {e}")

            # 等待计划生成
            start_wait = time.time()
            while self.latest_plan is None and (time.time() - start_wait) < 15.0:
                time.sleep(0.3)
                if self.latest_preview is not None and self.latest_plan is None:
                    try:
                        rospy.ServiceProxy("/grasp_6d/replan_execution", TriggerZero)(True)
                    except Exception:
                        pass

            if self.latest_plan is not None:
                plan_id = self.latest_plan.plan_id

        if not plan_id:
            # 再次查询 current_plan
            try:
                cur_res = rospy.ServiceProxy("/grasp/current_plan", TriggerZero)(True)
                if cur_res.success and "validation=VALID" in cur_res.message:
                    parts = dict(kv.split("=") for kv in cur_res.message.split() if "=" in kv)
                    plan_id = parts.get("plan_id")
            except Exception:
                pass

        if not plan_id:
            print(f"\n[ERROR] 未能获取到有效的 6D 抓取计划 ID！")
            self.log_file.close()
            return False

        print("\n" + "*"*75)
        print("  【拍摄准备通知】目标 6D 抓取计划完全就绪，通过了 MuJoCo 动力学验证！")
        print(f"  有效计划 ID: {plan_id}")
        print("  机械臂物理运动即刻触发！请用户保持拍摄视角对准机械臂末端与目标物体！")
        print("*"*75 + "\n")

        # 2. 立即调用 /grasp/start 发起物理执行
        try:
            rospy.wait_for_service("/grasp/start", timeout=3.0)
            print(f"[{datetime.now().strftime('%H:%M:%S')}] [Step 2] 正在调用 /grasp/start(execute=True, plan_id='{plan_id}')...")
            self.log_event("start_grasp_call", {"execute": True, "plan_id": plan_id})

            # 异步或同步发起执行
            start_proxy = rospy.ServiceProxy("/grasp/start", StartGrasp)

            # 开启监听线程持续轮询状态
            exec_thread = threading.Thread(
                target=lambda: self._call_start_grasp(start_proxy, plan_id),
                daemon=True
            )
            exec_thread.start()

        except Exception as e:
            print(f"[ERROR] 调用 /grasp/start 发生异常: {e}")
            self.log_file.close()
            return False

        # 3. 持续监听物理执行全过程
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] [Step 3] 机械臂已触发物理抓取动作，终端数据流高频同步记录中...")
        exec_timeout_sec = 60.0
        start_exec = time.time()

        while not self.is_done and (time.time() - start_exec) < exec_timeout_sec:
            time.sleep(0.1)

        if not self.is_done:
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] [TIMEOUT] 抓取动作在 60 秒内未报告最终完成状态。")
        else:
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] [FINISHED] 6D 抓取动作流程已结束！最终状态: {self.latest_state}")

        time.sleep(1.0)
        self.log_file.close()
        print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 全量传感器与状态数据流已固化至:\n  --> {TELEMETRY_LOG_PATH}")
        return True

    def _call_start_grasp(self, start_proxy, plan_id):
        try:
            t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"[{t_str}] [GRASP_START_REQUEST] 发送 /grasp/start(execute=True, plan_id={plan_id})...")
            resp = start_proxy(execute=True, plan_id=plan_id)
            t_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            print(f"[{t_str}] [GRASP_START_RESPONSE] success={resp.success}, message={resp.message}")
            self.log_event("start_grasp_response", {"success": resp.success, "message": resp.message})
            if not resp.success:
                print(f"[ERROR] 抓取任务拒绝或失败: {resp.message}")
                self.is_done = True
        except Exception as e:
            print(f"[ERROR] _call_start_grasp 异常: {e}")
            self.is_done = True


if __name__ == "__main__":
    executor = GraspTelemetryExecutor()
    executor.execute()
