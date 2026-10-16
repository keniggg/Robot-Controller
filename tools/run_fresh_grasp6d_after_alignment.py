#!/usr/bin/env python3
"""Run one fresh, exact-plan-bound 6D grasp after operator alignment."""

import hashlib
import json
import math
import sys
import threading
import time

import rospy
from std_msgs.msg import String

from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import StartGrasp, TriggerZero


class FreshPreviewRunner:
    def __init__(self):
        self.condition = threading.Condition()
        self.preview = None
        self.committed_audits = {}
        self.minimum_stamp_ns = 0
        self.plan_validity_sec = 0.0
        self.request_plan = None

    @staticmethod
    def stamp_ns(message):
        stamp = message.header.stamp
        return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)

    def on_preview(self, message):
        stamp_ns = self.stamp_ns(message)
        now_ns = rospy.Time.now().to_nsec()
        age_sec = float(now_ns - stamp_ns) / 1.0e9
        if (
            not bool(message.valid)
            or not str(message.plan_id).strip()
            or len(message.poses) != 4
            or str(message.diagnostic) != "FAR_FIELD_OBSERVATION_PLAN"
            or stamp_ns < self.minimum_stamp_ns
            or age_sec < 0.0
            or age_sec > self.plan_validity_sec
        ):
            return
        with self.condition:
            if self.preview is None:
                self.preview = message
                print(
                    "FRESH_PREVIEW plan_id=%s stamp_ns=%d age_sec=%.6f"
                    % (message.plan_id, stamp_ns, age_sec),
                    flush=True,
                )
                self.condition.notify_all()

    @staticmethod
    def audit_error(report):
        selection = dict(report.get("moveit_selection", {}) or {})
        selected = report.get("selected")
        replay = dict(report.get("replay_geometry", {}) or {})
        outcome = dict(report.get("outcome", {}) or {})
        if outcome.get("preview_valid") is not True:
            return "audit does not authorize a valid preview"
        if not isinstance(selected, dict) or selected.get("selected") is not True:
            return "audit has no unique selected lineage"
        if selection.get("policy") != "FIRST_REACHABLE_BY_AUTHORITATIVE_RANK":
            return "far-field MoveIt selection policy is not authoritative"
        if int(selection.get("checked_count", 0)) < 1:
            return "no strict MoveIt candidate was checked"
        if int(selection.get("reachable_count", 0)) != 1:
            return "far-field audit must bind exactly one reachable optimum"
        if selection.get("terminated_early") is True and (
            selection.get("termination_reason")
            != "FIRST_REACHABLE_BY_AUTHORITATIVE_RANK"
        ):
            return "early convergence reason is not bound"
        if replay.get("available") is not True:
            return "request-bound replay geometry is unavailable"
        if int(replay.get("object_points_count", 0)) < 1:
            return "request-bound target point cloud is empty"
        if len(str(replay.get("object_points_sha256", ""))) != 64:
            return "request-bound target point-cloud hash is invalid"

        moveit = dict(selected.get("moveit", {}) or {})
        if moveit.get("reachable") is not True:
            return "selected observation is not strictly reachable"
        hard_states = tuple(
            moveit.get(name)
            for name in (
                "collision_free",
                "within_joint_limits",
                "ik_valid",
                "planning_success",
            )
        )
        structured_success = (
            all(value is True for value in hard_states)
            and not str(moveit.get("failure_code", "") or "")
            and not str(moveit.get("evidence_code", "") or "")
        )
        strict_service_success = (
            all(value is None for value in hard_states)
            and not str(moveit.get("failure_code", "") or "")
            and moveit.get("evidence_code") == "STRICT_SERVICE_SUCCESS"
        )
        if not (structured_success or strict_service_success):
            return "selected observation lacks strict MoveIt evidence"

        sequence = dict(selected.get("execution_sequence", {}) or {})
        stages = sequence.get("stages")
        if (
            sequence.get("available") is not True
            or sequence.get("kind") != "far_field_observation"
            or not isinstance(stages, list)
            or len(stages) != 1
            or dict(stages[0]).get("stage") != "observation"
        ):
            return "audit does not bind exactly one observation execution stage"

        view = dict(selected.get("observation_view", {}) or {})
        try:
            distance = float(view["actual_camera_target_distance_m"])
            minimum = float(view["min_camera_target_distance_m"])
            maximum = float(view["max_camera_target_distance_m"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return "camera-target distance evidence is malformed"
        if (
            not all(math.isfinite(value) for value in (distance, minimum, maximum))
            or minimum < 0.18
            or maximum > 0.22
            # Rigid-transform arithmetic can move a boundary by one ULP.
            # This picometre tolerance is numerical, not a motion margin.
            or distance < minimum - 1.0e-12
            or distance > maximum + 1.0e-12
        ):
            return "camera-target distance is outside the 18-22 cm observation band"

        envelope = dict(selected.get("observation_envelope", {}) or {})
        try:
            clearance = float(envelope["minimum_support_clearance_m"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return "observation support-clearance evidence is malformed"
        if (
            envelope.get("ok") is not True
            or not math.isfinite(clearance)
            or clearance < 0.003
        ):
            return "observation envelope does not preserve 3 mm support clearance"
        return ""

    def on_gate_audit(self, message):
        try:
            reference = json.loads(str(message.data))
            report_path = str(reference["report_path"])
            expected_sha256 = str(reference["report_sha256"])
            with open(report_path, "rb") as handle:
                payload = handle.read()
            if hashlib.sha256(payload).hexdigest() != expected_sha256:
                return
            report = json.loads(payload.decode("utf-8"))
            plan_id = str(report.get("plan_id", "")).strip()
            error = self.audit_error(report)
            if not plan_id or error:
                if error:
                    print("BOUND_AUDIT_REJECTED reason=%s" % error, flush=True)
                return
        except (KeyError, OSError, TypeError, ValueError):
            return
        with self.condition:
            self.committed_audits[plan_id] = report
            print(
                "BOUND_AUDIT_COMMITTED plan_id=%s mode=%s"
                % (plan_id, str(report.get("mode", ""))),
                flush=True,
            )
            self.condition.notify_all()

    def wait_for_preview(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        with self.condition:
            while self.preview is None and not rospy.is_shutdown():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.condition.wait(min(1.0, remaining))
            return self.preview

    def stop_candidate_computation(self):
        if self.request_plan is None:
            return
        response = self.request_plan(False)
        print(
            "CANDIDATE_COMPUTATION_STOP success=%s message=%s"
            % (response.success, response.message),
            flush=True,
        )

    @staticmethod
    def current_plan_matches(current_plan, plan_id):
        observed = current_plan(True)
        return bool(
            observed.success
            and ("plan_id=%s " % plan_id) in (observed.message + " ")
        ), observed

    @staticmethod
    def audit_matches_preview(report, preview):
        try:
            stage = dict(report["selected"]["execution_sequence"]["stages"][0])
            position = tuple(float(value) for value in stage["position_m"])
            quaternion = tuple(
                float(value) for value in stage["quaternion_xyzw"]
            )
            pose = preview.poses[0]
            preview_position = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            preview_quaternion = (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        except (KeyError, IndexError, TypeError, ValueError, OverflowError):
            return False
        return (
            max(abs(a - b) for a, b in zip(position, preview_position)) <= 1e-9
            and max(
                abs(a - b)
                for a, b in zip(quaternion, preview_quaternion)
            )
            <= 1e-9
        )

    def wait_for_execution_authority(
        self,
        current_plan,
        replan_execution,
        preview,
        plan_id,
        timeout_sec,
    ):
        deadline = time.monotonic() + timeout_sec
        replan_requested = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.condition:
                audit = self.committed_audits.get(plan_id)
            if audit is None or not self.audit_matches_preview(audit, preview):
                time.sleep(0.1)
                continue
            matches, observed = self.current_plan_matches(
                current_plan,
                plan_id,
            )
            if matches:
                return observed
            if not replan_requested:
                promoted = replan_execution(True)
                replan_requested = True
                print(
                    "PROMOTION_AFTER_BOUND_AUDIT success=%s message=%s"
                    % (promoted.success, promoted.message),
                    flush=True,
                )
                if not promoted.success:
                    return None
            time.sleep(0.1)
        return None

    def run(self):
        rospy.init_node("fresh_grasp6d_execution_runner", anonymous=True)
        rospy.wait_for_service("/grasp_6d/request_plan", timeout=15.0)
        rospy.wait_for_service("/grasp_6d/replan_execution", timeout=15.0)
        rospy.wait_for_service("/grasp/current_plan", timeout=15.0)
        rospy.wait_for_service("/grasp/start", timeout=15.0)

        self.request_plan = rospy.ServiceProxy(
            "/grasp_6d/request_plan",
            TriggerZero,
        )
        replan_execution = rospy.ServiceProxy(
            "/grasp_6d/replan_execution",
            TriggerZero,
        )
        current_plan = rospy.ServiceProxy("/grasp/current_plan", TriggerZero)
        start_grasp = rospy.ServiceProxy("/grasp/start", StartGrasp)

        rospy.Subscriber(
            "/grasp_6d/preview_plan_enriched",
            Grasp6DPlan,
            self.on_preview,
            queue_size=1,
        )
        rospy.Subscriber(
            "/grasp_6d/gate_audit",
            String,
            self.on_gate_audit,
            queue_size=4,
        )
        rospy.sleep(0.25)
        self.plan_validity_sec = max(
            0.0,
            float(rospy.get_param("/grasp_6d/plan_validity_sec")),
        )
        self.minimum_stamp_ns = rospy.Time.now().to_nsec()
        print(
            "FRESH_WINDOW minimum_stamp_ns=%d timeout_sec=300 "
            "plan_validity_sec=%.6f"
            % (self.minimum_stamp_ns, self.plan_validity_sec),
            flush=True,
        )
        response = self.request_plan(True)
        print(
            "CANDIDATE_COMPUTATION_START success=%s message=%s"
            % (response.success, response.message),
            flush=True,
        )
        if not response.success:
            return 2

        preview = self.wait_for_preview(300.0)
        if preview is None:
            self.stop_candidate_computation()
            print(
                "TIMEOUT no fresh valid four-stage preview within 300 seconds",
                flush=True,
            )
            return 3

        plan_id = str(preview.plan_id).strip()
        verified = self.wait_for_execution_authority(
            current_plan,
            replan_execution,
            preview,
            plan_id,
            60.0,
        )
        if verified is None:
            self.stop_candidate_computation()
            print(
                "EXECUTION_AUTHORITY_TIMEOUT expected_plan_id=%s" % plan_id,
                flush=True,
            )
            return 4

        # Keep inference and its opaque target identity alive across the
        # observation move. The task owns near-field phase transitions;
        # stopping here changes the track before its reached-view handoff.
        still_matches, verified_before_start = self.current_plan_matches(
            current_plan,
            plan_id,
        )
        if not still_matches:
            print(
                "PLAN_ID_VERIFY_BEFORE_START_FAILED "
                "expected_plan_id=%s" % plan_id,
                flush=True,
            )
            self.stop_candidate_computation()
            return 5
        print(
            "PLAN_ID_VERIFIED plan_id=%s message=%s"
            % (plan_id, verified_before_start.message),
            flush=True,
        )
        print("GRASP_START_CALL plan_id=%s" % plan_id, flush=True)
        result = start_grasp(True, plan_id)
        print(
            "GRASP_RESULT success=%s message=%s"
            % (result.success, result.message),
            flush=True,
        )
        self.stop_candidate_computation()
        return 0 if result.success else 6


if __name__ == "__main__":
    runner = FreshPreviewRunner()
    try:
        sys.exit(runner.run())
    except Exception as exc:
        try:
            runner.stop_candidate_computation()
        except Exception as stop_exc:
            print(
                "CANDIDATE_COMPUTATION_STOP_EXCEPTION %s" % stop_exc,
                flush=True,
            )
        print("RUNNER_EXCEPTION %s" % exc, flush=True)
        sys.exit(10)
