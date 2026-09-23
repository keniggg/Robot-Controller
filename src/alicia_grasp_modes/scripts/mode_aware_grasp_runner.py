#!/usr/bin/env python3
"""Execute one fresh audited plan for the currently selected mode and strategy."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import rospkg
import rospy
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import StartGrasp, TriggerZero
from alicia_flexible_grasp.grasp.rich_plan_integrity import plan_id_matches_content
from alicia_grasp_modes.selection import SELECTION_PARAM, parse_selection
from alicia_grasp_modes.runner_contract import (
    selection_audit_error, direct_audit_error, audit_matches_preview,
    unknown_observation_audit_error)


def _load_original():
    root = Path(rospkg.RosPack().get_path('alicia_flexible_grasp_supervisor'))
    path = root.parents[1] / 'tools' / 'run_fresh_grasp6d_after_alignment.py'
    spec = importlib.util.spec_from_file_location('_original_fresh_grasp_runner', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


original = _load_original()


class ModeAwareGraspRunner(original.FreshPreviewRunner):
    def __init__(self, expected_mode=None, expected_strategy=None):
        super().__init__()
        self.selection = None
        self.expected_mode = expected_mode
        self.expected_strategy = expected_strategy
        self.expected_model = ''
        self.execution_inflight = False
        self.selection_changed = False
        self.terminal_reason = ''

    def selection_current(self):
        try:
            current = parse_selection(rospy.get_param(SELECTION_PARAM, None))
            same = current == self.selection and not self.selection_changed
        except Exception:
            same = False
        if not same:
            self.selection_changed = True
        return same

    def on_selection(self, message):
        try:
            incoming = parse_selection(message.data)
        except (TypeError, ValueError):
            return
        # The parameter is authoritative; an old latched topic is harmless.
        if incoming != self.selection and not self.selection_current():
            with self.condition:
                self.condition.notify_all()

    def on_preview(self, message):
        if not self.selection or not self.selection_current():
            return
        expected_phase = ('CONTACT_EXECUTION_PLAN' if self.selection['strategy'] == 'direct'
                          else 'FAR_FIELD_OBSERVATION_PLAN')
        stamp_ns = self.stamp_ns(message)
        age = (rospy.Time.now().to_nsec() - stamp_ns) / 1e9
        if (self.selection['strategy'] == 'direct' and not message.valid
                and message.candidate_source == 'near_field_terminal'
                and stamp_ns >= self.minimum_stamp_ns and 0 <= age <= self.plan_validity_sec):
            with self.condition:
                self.terminal_reason = str(message.diagnostic)
                print('DIRECT_PLANNING_TERMINAL %s' % self.terminal_reason, flush=True)
                self.condition.notify_all()
            return
        if (not message.valid or message.diagnostic != expected_phase or len(message.poses) != 4
                or message.model_choice != self.expected_model
                or stamp_ns < self.minimum_stamp_ns or stamp_ns <= self.selection['stamp_ns']
                or not 0 <= age <= self.plan_validity_sec
                or not plan_id_matches_content(message)):
            return
        with self.condition:
            if self.preview is None:
                self.preview = message
                print('FRESH_MODE_PREVIEW strategy=%s plan_id=%s stamp_ns=%d' % (
                    self.selection['strategy'], message.plan_id, stamp_ns), flush=True)
                self.condition.notify_all()

    def audit_error(self, report):
        error = selection_audit_error(report, self.selection, self.minimum_stamp_ns)
        if error:
            return error
        if self.selection['strategy'] == 'direct':
            return direct_audit_error(report)
        if self.selection['mode'] == 'unknown':
            return unknown_observation_audit_error(report)
        return original.FreshPreviewRunner.audit_error(report)

    def audit_matches_preview(self, report, preview):
        return audit_matches_preview(report, preview, self.selection['strategy'] == 'direct')

    def wait_for_preview(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if not self.selection_current():
                return None
            with self.condition:
                if self.terminal_reason:
                    return None
                if self.preview is not None:
                    return self.preview
                self.condition.wait(min(.25, max(0., deadline - time.monotonic())))
        return None

    def wait_for_execution_authority(self, current_plan, replan_execution, preview, plan_id, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        replan_requested = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if not self.selection_current():
                return None
            with self.condition:
                report = self.committed_audits.get(plan_id)
            if report is not None and not self.audit_error(report) and self.audit_matches_preview(report, preview):
                matches, observed = self.current_plan_matches(current_plan, plan_id)
                if matches and self.selection_current():
                    return observed
                if not replan_requested:
                    promoted = replan_execution(True)
                    replan_requested = True
                    print('MODE_PLAN_PROMOTION success=%s message=%s' % (
                        promoted.success, promoted.message), flush=True)
                    if not promoted.success:
                        return None
            time.sleep(.1)
        return None

    def stop_candidate_computation(self):
        # A lost service response can coexist with physical execution. Retain
        # its computation/target identity until the task's owner resolves it.
        if self.execution_inflight or not self.selection_current():
            return
        return super().stop_candidate_computation()

    def run(self):
        rospy.init_node('mode_aware_grasp_runner', anonymous=True)
        self.selection = parse_selection(rospy.get_param(SELECTION_PARAM, None))
        if ((self.expected_mode and self.selection['mode'] != self.expected_mode)
                or (self.expected_strategy and self.selection['strategy'] != self.expected_strategy)):
            raise ValueError('active mode/strategy differs from requested attempt')
        self.expected_model = ('unknown_tabletop' if self.selection['mode'] == 'unknown'
                               else str(rospy.get_param('/perception/yolo_model_choice', 'original')))
        for name in ('/grasp_6d/request_plan', '/grasp_6d/replan_execution', '/grasp/current_plan', '/grasp/start'):
            rospy.wait_for_service(name, timeout=15.)
        self.request_plan = rospy.ServiceProxy('/grasp_6d/request_plan', TriggerZero)
        replan_execution = rospy.ServiceProxy('/grasp_6d/replan_execution', TriggerZero)
        current_plan = rospy.ServiceProxy('/grasp/current_plan', TriggerZero)
        start_grasp = rospy.ServiceProxy('/grasp/start', StartGrasp)
        self.plan_validity_sec = float(rospy.get_param('/grasp_6d/plan_validity_sec'))
        if not math.isfinite(self.plan_validity_sec) or self.plan_validity_sec <= 0:
            raise ValueError('plan validity must be finite and positive')
        self.minimum_stamp_ns = max(rospy.Time.now().to_nsec(), self.selection['stamp_ns'] + 1)
        rospy.Subscriber('/grasp_mode/selection', String, self.on_selection, queue_size=2)
        rospy.Subscriber('/grasp_6d/preview_plan_enriched', Grasp6DPlan, self.on_preview, queue_size=2)
        rospy.Subscriber('/grasp_6d/gate_audit', String, self.on_gate_audit, queue_size=4)
        print('MODE_ATTEMPT selection=%s minimum_stamp_ns=%d' % (
            json.dumps(self.selection, sort_keys=True), self.minimum_stamp_ns), flush=True)
        if not self.selection_current():
            return 7
        response = self.request_plan(True)
        print('CANDIDATE_COMPUTATION_START success=%s message=%s' % (
            response.success, response.message), flush=True)
        if not response.success:
            return 2
        preview = self.wait_for_preview(300.)
        if preview is None:
            print('NO_FRESH_MODE_PREVIEW selection_changed=%s terminal=%s' % (
                self.selection_changed, self.terminal_reason), flush=True)
            self.stop_candidate_computation()
            return 7 if self.selection_changed else 3
        plan_id = preview.plan_id
        verified = self.wait_for_execution_authority(current_plan, replan_execution, preview, plan_id, 60.)
        if verified is None:
            print('MODE_EXECUTION_AUTHORITY_UNAVAILABLE plan_id=%s' % plan_id, flush=True)
            self.stop_candidate_computation()
            return 7 if self.selection_changed else 4
        matches, observed = self.current_plan_matches(current_plan, plan_id)
        with self.condition:
            report = self.committed_audits.get(plan_id)
        if (not matches or not self.selection_current() or report is None
                or self.audit_error(report) or not self.audit_matches_preview(report, preview)):
            print('MODE_BINDING_VERIFY_BEFORE_START_FAILED plan_id=%s' % plan_id, flush=True)
            self.stop_candidate_computation()
            return 5
        # Router and task independently bind the same selection at their
        # service/action boundary, closing the remaining transport race.
        print('GRASP_START_CALL mode=%s strategy=%s generation=%d plan_id=%s' % (
            self.selection['mode'], self.selection['strategy'], self.selection['generation'], plan_id), flush=True)
        self.execution_inflight = True
        result = start_grasp(True, plan_id)
        self.execution_inflight = False
        print('GRASP_RESULT success=%s message=%s' % (result.success, result.message), flush=True)
        self.stop_candidate_computation()
        return 0 if result.success else 6


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('carton', 'unknown'))
    parser.add_argument('--strategy', choices=('two_stage', 'direct'))
    args = parser.parse_args(rospy.myargv()[1:])
    runner = ModeAwareGraspRunner(args.mode, args.strategy)
    try:
        sys.exit(runner.run())
    except Exception as exc:
        try:
            runner.stop_candidate_computation()
        except Exception as stop_error:
            print('CANDIDATE_COMPUTATION_STOP_EXCEPTION %s' % stop_error, flush=True)
        print('RUNNER_EXCEPTION %s execution_response_unknown=%s' % (
            exc, runner.execution_inflight), flush=True)
        sys.exit(10)
