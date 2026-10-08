"""Task-owned observe/return handoff; motion remains in the strict gateway."""
from copy import deepcopy
import hashlib
import json
import math
import os
import re
import threading
import time

import rospy
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.srv import CompensatePregrasp
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.observe_return_session import exact_ns, parse_session, VIEW_FIELDS
from alicia_grasp_modes.active_observation_policy import observation_budget, evidence_digest, information_gain_valid
from alicia_grasp_modes.runner_contract import (observe_return_audit_error, audit_matches_preview,
                                               unknown_observation_audit_error)


_EXPERIMENT_MOUNT = '/media/zhuyupei/SanStand'
_EXPERIMENT_ROOT = _EXPERIMENT_MOUNT + '/实验记录/Robot-Controller'
_SILHOUETTE_TRIGGER = 'RGB_DEPTH_SILHOUETTE_AMBIGUOUS'


def positive_ns(value):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value <= 0:
        raise ValueError('positive exact source timestamp required')
    return value


class ObserveReturnTask:
    def _initialize_observe_return_state(self):
        self._observe_return_condition = threading.Condition(threading.RLock())
        self._observe_return_reports = {}
        self._observe_return_report_references = {}
        self._diagnostic_observation_consumed_sources = set()
        self._observe_return_acks = {}
        self._clear_observe_return_attempt()

    def _clear_observe_return_attempt(self):
        """Retire one finished attempt without discarding a fresh plan audit."""
        with self._observe_return_condition:
            self._observe_return_session = None
            self._observe_return_entry_phase = None
            self._observe_return_origin_proof = None
            self._observe_return_origin_plan = None
            self._observe_return_active_view_plan = None
            self._observe_return_submitted_view_requests = set()
            self._observe_return_bound_report = None
            self._observe_return_bound_reference = None
            self._diagnostic_observation_sample_reference = None
            self._observe_return_acks.clear()

    def _setup_observe_return_io(self):
        self._observe_return_pub = rospy.Publisher(
            '/grasp_mode/observe_return_session', String, queue_size=4, latch=True)
        self._observe_return_ack_sub = rospy.Subscriber(
            '/grasp_mode/observe_return_ack', String, self._observe_return_ack_cb, queue_size=8)
        self._observe_return_audit_sub = rospy.Subscriber(
            '/grasp_6d/gate_audit', String, self._observe_return_audit_cb, queue_size=8)

    def _observe_return_audit_cb(self, msg):
        try:
            reference = json.loads(msg.data)
            digest = reference['report_sha256']
            if not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest):
                return
            with open(reference['report_path'], 'rb') as source:
                payload = source.read()
            if hashlib.sha256(payload).hexdigest() != digest:
                return
            report = json.loads(payload.decode('utf-8'))
            if not isinstance(report, dict) or not report.get('plan_id'):
                return
            with self._observe_return_condition:
                self._observe_return_reports[report['plan_id']] = report
                self._observe_return_report_references[report['plan_id']] = dict(
                    path=os.path.realpath(reference['report_path']), sha256=digest)
                while len(self._observe_return_reports) > 16:
                    oldest = next(iter(self._observe_return_reports))
                    del self._observe_return_reports[oldest]
                    self._observe_return_report_references.pop(oldest, None)
                self._observe_return_condition.notify_all()
        except (KeyError, OSError, TypeError, ValueError, AttributeError):
            return

    def _observe_return_ack_cb(self, msg):
        try:
            ack = json.loads(msg.data)
            key = (str(ack['session_id']), str(ack['phase']), positive_ns(ack['stamp_ns']))
            if type(ack['accepted']) is not bool:
                return
            with self._observe_return_condition:
                self._observe_return_acks[key] = ack
                while len(self._observe_return_acks) > 16:
                    del self._observe_return_acks[next(iter(self._observe_return_acks))]
                self._observe_return_condition.notify_all()
        except (KeyError, TypeError, ValueError, AttributeError):
            return

    def _observe_return_live(self):
        if (not self.active or rospy.is_shutdown()
                or not self._selection_current(self._mode_bound_selection)
                or rospy.get_param('/gui/joint_direct_mode', False) is True):
            raise ValueError('OBSERVE_RETURN_OWNER_CHANGED')
        session = self._observe_return_session
        if session is not None and rospy.Time.now().to_nsec() >= positive_ns(session['deadline_ns']):
            raise ValueError('OBSERVE_RETURN_SESSION_EXPIRED')

    def _observe_return_report(self, plan, allow_returned=False):
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            self._observe_return_live()
            with self._observe_return_condition:
                report = deepcopy(self._observe_return_reports.get(plan.plan_id))
                if report is None:
                    self._observe_return_condition.wait(.05)
                    continue
            if parse_selection(report.get('grasp_mode_selection')) != self._mode_bound_selection:
                raise ValueError('OBSERVE_RETURN_AUDIT_SELECTION_CHANGED')
            reason = observe_return_audit_error(report, allow_returned=allow_returned)
            if reason:
                raise ValueError('OBSERVE_RETURN_AUDIT_INVALID: ' + reason)
            if (report['observe_return']['target_track_id'] != plan.target_track_id
                    or not audit_matches_preview(report, plan, plan.diagnostic == 'CONTACT_EXECUTION_PLAN')):
                raise ValueError('OBSERVE_RETURN_AUDIT_PLAN_MISMATCH')
            return report
        raise ValueError('OBSERVE_RETURN_BOUND_AUDIT_UNAVAILABLE')

    def _observe_return_publish(self, message):
        self._observe_return_live()
        key = (message['session_id'], message['phase'], positive_ns(message['stamp_ns']))
        self._observe_return_pub.publish(String(data=json.dumps(message, allow_nan=False)))
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            self._observe_return_live()
            with self._observe_return_condition:
                ack = self._observe_return_acks.get(key)
                if ack is None:
                    self._observe_return_condition.wait(.05)
                    continue
            if ack['accepted'] is not True:
                raise ValueError('OBSERVE_RETURN_REMOTE_REJECTED: ' + str(ack.get('reason', '')))
            try:
                same = (type(ack.get('version')) is int and ack['version'] == message['version']
                    and parse_selection(ack['selection']) == parse_selection(message['selection'])
                    and type(ack.get('target_epoch')) is int
                    and ack['target_epoch'] == message['target_epoch']
                    and all(ack[name] == message[name] for name in (
                        'session_id', 'phase', 'target_track_id', 'origin_plan_id'))
                    and all(exact_ns(ack[name], name, 0) == exact_ns(message.get(name, 0), name, 0)
                        for name in ('stamp_ns', 'deadline_ns', 'origin_snapshot_stamp_ns',
                                     'return_verified_stamp_ns', 'contact_deadline_ns')))
            except (KeyError, TypeError, ValueError):
                same = False
            if same and message['version'] == 2:
                try:
                    normalized = parse_session(message, self._mode_bound_selection,
                        rospy.Time.now().to_nsec(), self._observe_return_session
                        if self._observe_return_session != message else None)
                    same = all(ack.get(key) == normalized[key] for key in (*VIEW_FIELDS, 'max_observations'))
                except (KeyError, TypeError, ValueError):
                    same = False
            if not same:
                raise ValueError('OBSERVE_RETURN_REMOTE_ACK_BINDING_INVALID')
            self._observe_return_live()
            return ack
        raise ValueError('OBSERVE_RETURN_REMOTE_ACK_TIMEOUT')

    def _observe_return_gateway(self, name, plan, *, bound_plan=None, gcfg=None):
        self._observe_return_live()
        rospy.wait_for_service(name, timeout=3.)
        call = rospy.ServiceProxy(name, CompensatePregrasp)
        if bound_plan is None:
            response = call(plan, True)
        else:
            # Waits must remain outside the callback lock. Recheck the active
            # view at the actual return submission, while the gateway retains
            # the immutable origin plan as its return destination.
            validation, response = self._invoke_plan_bound_action(bound_plan, gcfg,
                'observe-return origin submission', lambda: call(plan, True))
            if not validation.ok:
                raise ValueError(validation.code + ': ' + validation.reason)
        if not response.success:
            raise ValueError('OBSERVE_RETURN_GATEWAY_FAILED: ' + response.message)
        proof = json.loads(response.message)
        if (not isinstance(proof, dict) or proof.get('plan_id') != plan.plan_id
                or proof.get('target_track_id') != plan.target_track_id):
            raise ValueError('OBSERVE_RETURN_GATEWAY_BINDING_INVALID')
        self._observe_return_live()
        return proof

    @staticmethod
    def _observe_return_origin_valid(proof, plan, now_ns):
        """Require the measured origin fields supplied by the strict gateway."""
        try:
            def finite_vector(values, size):
                return (isinstance(values, (list, tuple)) and len(values) == size
                    and all(type(value) in (int, float) and math.isfinite(value) for value in values))
            pose = proof['origin_tool_pose']
            return (proof.get('code') in ('OBSERVE_RETURN_ORIGIN_CAPTURED', 'OBSERVE_RETURN_ORIGIN_REUSED')
                and proof.get('physical_motion_submitted') is False
                and isinstance(proof.get('session_id'), str) and bool(proof['session_id'])
                and proof['session_id'] == proof['session_id'].strip()
                and plan.header.stamp.to_nsec() <= positive_ns(proof['origin_accepted_stamp_ns']) <= now_ns
                and positive_ns(proof['epoch_ns']) <= positive_ns(proof['origin_accepted_stamp_ns'])
                and all(isinstance(proof.get(key), str) and re.fullmatch('[0-9a-f]{64}', proof[key])
                    for key in ('plan_digest', 'model_sha256', 'origin_goal_sha256'))
                and finite_vector(proof['origin_joint_positions_rad'], 6)
                and pose['frame_id'] == 'base_link'
                and finite_vector(pose['position_m'], 3)
                and finite_vector(pose['quaternion_xyzw'], 4)
                and math.isclose(sum(value*value for value in pose['quaternion_xyzw']), 1., abs_tol=1e-6))
        except (AttributeError, KeyError, TypeError, ValueError):
            return False

    def _begin_observe_return(self, plan, gcfg):
        report = self._observe_return_report(plan)
        metadata = report['observe_return']
        if metadata.get('phase') != 'observation_prefix':
            raise ValueError('OBSERVE_RETURN_OBSERVATION_PREFIX_REQUIRED')
        with self._observe_return_condition:
            if self._observe_return_reports.get(plan.plan_id) != report:
                raise ValueError('OBSERVE_RETURN_BOUND_AUDIT_CHANGED')
            self._observe_return_bound_report = deepcopy(report)
            self._observe_return_bound_reference = deepcopy(
                self._observe_return_report_references.get(plan.plan_id))
        self.set_state(GraspStages.MOVE_PREGRASP, 'capturing initial observation pose')
        proof = self._observe_return_gateway('/supervisor/capture_observe_return_origin', plan)
        now = rospy.Time.now().to_nsec()
        if not self._observe_return_origin_valid(proof, plan, now):
            raise ValueError('OBSERVE_RETURN_ORIGIN_PROOF_INVALID')
        limit = gcfg.get('unknown_observe_return_session_timeout_sec', 600.)
        if (isinstance(limit, bool) or not isinstance(limit, (float, int))
                or not math.isfinite(limit) or not 30. <= limit <= 600.):
            raise ValueError('OBSERVE_RETURN_SESSION_BUDGET_INVALID')
        epoch = metadata['target_epoch']
        if type(epoch) is not int or epoch < 0 or metadata['target_track_id'] != plan.target_track_id:
            raise ValueError('OBSERVE_RETURN_TARGET_BINDING_INVALID')
        message = dict(version=1, session_id=proof['session_id'],
            selection=deepcopy(self._mode_bound_selection), phase='observing',
            stamp_ns=str(now), deadline_ns=str(now + int(limit * 1e9)),
            target_epoch=epoch, target_track_id=plan.target_track_id,
            origin_plan_id=plan.plan_id,
            origin_snapshot_stamp_ns=str(positive_ns(metadata['origin_snapshot_stamp_ns'])),
            return_verified_stamp_ns='0')
        diagnostic = metadata.get('diagnostic_observation_only', False)
        version = 1 if diagnostic else gcfg.get('unknown_observe_return_session_version', 2)
        if type(version) is not int or version not in (1, 2):
            raise ValueError('OBSERVE_RETURN_SESSION_VERSION_INVALID')
        if version == 2:
            message.update(version=2,
                max_observations=observation_budget(gcfg.get('unknown_observe_return_max_observations', 3)),
                view_index=1, view_plan_id=plan.plan_id,
                view_source_stamp_ns=str(plan.header.stamp.to_nsec()),
                view_request_id='initial:' + plan.plan_id, previous_gain_sha256='0' * 64)
        self._observe_return_session = message
        self._observe_return_origin_proof = proof
        self._observe_return_origin_plan = deepcopy(plan)
        self._observe_return_active_view_plan = deepcopy(plan)
        self._observe_return_publish(message)
        rospy.loginfo('Observe-return origin captured: %s', json.dumps(proof))

    def _coordinated_observation_views(self):
        """A session owns every view: v1 has one; v2 has at most three.

        Neither can borrow an uncounted motion from the legacy recovery path.
        """
        session = self._observe_return_session or {}
        return session.get('version') in (1, 2)

    def _observe_return_view_is_fresh(self, plan, gcfg):
        validity = self._configured_plan_validity(gcfg)
        age = (rospy.Time.now().to_nsec() - positive_ns(plan.header.stamp.to_nsec())) * 1e-9
        if not math.isfinite(validity) or validity <= 0. or not 0. <= age <= validity:
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_PLAN_STALE_OR_FUTURE')

    def _observe_return_supplement_report(self, plan, report):
        session = self._observe_return_session
        metadata = report['observe_return']
        if (parse_selection(report['grasp_mode_selection']) != self._mode_bound_selection
                or report.get('execution_strategy') != 'observe_return'
                or report.get('planning_origin') != 'observe_return_supplemental_observation'
                or report.get('observation_motion_performed') is not True
                or metadata.get('phase') != 'supplemental_observation'
                or metadata.get('version') != 2
                or metadata.get('diagnostic_observation_only', False) is not False
                or metadata.get('trigger_code') != 'BILATERAL_SURFACE_EVIDENCE_MISSING'
                or metadata.get('contact_or_motion_authorized') is not False
                or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or plan.target_track_id != session['target_track_id']
                or metadata['view_plan_id'] != plan.plan_id
                or metadata['view_source_stamp_ns'] != plan.header.stamp.to_nsec()
                or metadata['previous_view_index'] != session['view_index']
                or metadata['view_index'] != session['view_index'] + 1
                or any(metadata[key] != session[key] for key in (
                    'session_id', 'origin_plan_id', 'target_epoch', 'target_track_id', 'max_observations'))
                or any(positive_ns(metadata[key]) != positive_ns(session[key])
                    for key in ('origin_snapshot_stamp_ns', 'deadline_ns'))
                or not information_gain_valid(metadata['previous_gain'])
                or evidence_digest(metadata['previous_gain']) != metadata['previous_gain_sha256']
                or unknown_observation_audit_error(report)
                or not audit_matches_preview(report, plan, False)):
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_AUDIT_INVALID')
        gain = metadata['previous_gain']
        failure = metadata['failed_search']
        opportunity = metadata['visual_opportunity']
        if (gain['target_epoch'] != session['target_epoch']
                or gain['target_track_id'] != session['target_track_id']
                or positive_ns(gain['source_stamp_ns']) <= positive_ns(session['view_source_stamp_ns'])
                or positive_ns(failure['snapshot_stamp_ns']) < positive_ns(gain['source_stamp_ns'])
                or failure['target_epoch'] != session['target_epoch']
                or failure['selected'] is not False or failure['terminated_early'] is not False
                or failure['reachable_count'] != 0
                or failure['checked_count'] != failure['shortlist_count']
                or failure['contact_search_code'] not in ('NEAR_FIELD_NO_HARD_SAFE_CANDIDATE', 'NEAR_FIELD_NO_REACHABLE_CANDIDATE')
                or any(opportunity[key] != failure[key] for key in (
                    'request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns', 'contact_search_code'))
                or opportunity.get('contact_or_motion_authorized') is not False
                or opportunity.get('policy') != 'current_request_independent_bilateral_view_gap'):
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_EVIDENCE_INVALID')
        request_fields = ('session_id', 'view_index', 'previous_view_index', 'origin_plan_id',
            'origin_snapshot_stamp_ns', 'target_epoch', 'target_track_id', 'deadline_ns',
            'max_observations', 'failed_search', 'visual_opportunity', 'previous_gain',
            'previous_gain_sha256', 'contact_or_motion_authorized')
        if evidence_digest({key: metadata[key] for key in request_fields}) != metadata['view_request_id']:
            raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_REQUEST_CHANGED')
        return metadata

    def _before_near_field_terminal_recovery(self, last_result, current_plan, gcfg,
                                             move_pose, strict_execute_pose):
        """Consume a separately typed supplemental preview, never an error alone."""
        del last_result
        session = self._observe_return_session or {}
        if session.get('version') != 2 or session.get('phase') != 'observing':
            return None
        with self._grasp6d_plan_guard():
            candidate = deepcopy(getattr(self, 'latest_grasp6d_preview_plan', None))
        if (candidate is None or not candidate.valid
                or candidate.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or candidate.plan_id == session['view_plan_id']):
            return None
        with self._observe_return_condition:
            report = deepcopy(self._observe_return_reports.get(candidate.plan_id))
        if report is None:
            return dict(waiting=True)
        if report.get('observe_return', {}).get('phase') != 'supplemental_observation':
            return None
        try:
            self._observe_return_live()
            if (not callable(move_pose) or not callable(strict_execute_pose)
                    or session['view_index'] >= session['max_observations']
                    or current_plan.plan_id != session['view_plan_id']):
                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_CONTEXT_INVALID')
            if not self._diagnostic_observation_checkpoint(current_plan, gcfg, 'supplemental observation admission'):
                return dict(failed=True)
            failed_execution = getattr(self, '_last_observation_execution_failure', None)
            if (isinstance(failed_execution, dict)
                    and failed_execution.get('plan_id') == current_plan.plan_id):
                raise ValueError('OBSERVE_RETURN_AFTER_FAILED_EXECUTION_FORBIDDEN')
            self._observe_return_view_is_fresh(candidate, gcfg)
            metadata = self._observe_return_supplement_report(candidate, report)
            request_id = metadata['view_request_id']
            if request_id in self._observe_return_submitted_view_requests:
                raise ValueError('OBSERVE_RETURN_VIEW_ALREADY_ATTEMPTED')
            next_session = dict(session, phase='observing', stamp_ns=str(rospy.Time.now().to_nsec()))
            next_session.update({key: metadata[key] for key in VIEW_FIELDS})
            parse_session(next_session, self._mode_bound_selection,
                          rospy.Time.now().to_nsec(), session)
            # Remote acknowledgement is not execution authority. Validate the
            # old frozen authority after its asynchronous wait, then replace
            # plan and local step under the same callback lock. Revocation
            # while waiting must never be cleared by _freeze_execution_plan.
            self._observe_return_publish(next_session)
            self.set_state(GraspStages.MOVE_PREGRASP, 'admitting supplemental observation view')
            proof = self._observe_return_gateway('/supervisor/admit_observe_return_view', candidate)
            origin = self._observe_return_origin_proof or {}
            if (proof.get('code') != 'OBSERVE_RETURN_VIEW_ADMITTED'
                    or proof.get('physical_motion_submitted') is not False
                    or proof.get('origin_plan_id') != session['origin_plan_id']
                    or proof.get('origin_plan_digest') != origin.get('plan_digest')
                    or proof.get('origin_goal_sha256') != origin.get('origin_goal_sha256')
                    or proof.get('epoch_ns') != origin.get('epoch_ns')
                    or proof.get('previous_view_plan_id') != session['view_plan_id']
                    or proof.get('plan_digest') != committed_plan_digest(candidate)
                    or any(proof.get(key) != next_session[key] for key in
                        ('session_id', 'view_index', 'view_request_id', 'previous_gain_sha256'))
                    or any(positive_ns(proof[key]) != positive_ns(next_session[key]) for key in
                        ('view_source_stamp_ns', 'deadline_ns'))):
                raise ValueError('OBSERVE_RETURN_VIEW_ADMISSION_PROOF_INVALID')
            def bind_next_view():
                self._observe_return_view_is_fresh(candidate, gcfg)
                frozen = self._freeze_execution_plan(candidate)
                self._observe_return_session = next_session
                self._observe_return_active_view_plan = deepcopy(frozen)
                self._observe_return_submitted_view_requests.add(request_id)
                return frozen
            validation, frozen = self._invoke_plan_bound_action(current_plan, gcfg,
                'supplemental observation rebind', bind_next_view)
            if not validation.ok:
                raise ValueError(validation.code + ': ' + validation.reason)
            if not self._diagnostic_observation_checkpoint(frozen, gcfg, 'supplemental observation frozen'):
                return dict(failed=True)
            from geometry_msgs.msg import PoseStamped
            observation = PoseStamped(header=deepcopy(frozen.header), pose=deepcopy(frozen.poses[0]))
            self._last_motion_settle_sample = None
            if not self._plan_and_execute_pose(GraspStages.MOVE_PREGRASP,
                    'supplemental observation %d' % next_session['view_index'],
                    observation, move_pose, 'supplemental observation',
                    execution_plan_id=frozen.plan_id, execution_plan=frozen, gcfg=gcfg,
                    execute_pose=strict_execute_pose, allow_post_failure_observation_validation=False):
                return dict(failed=True)
            settled = deepcopy(getattr(self, '_last_motion_settle_sample', None))
            cutoff = rospy.Time.now().to_nsec()
            if (not isinstance(settled, dict)
                    or not positive_ns(settled['stationary_window_start_ns'])
                       < positive_ns(settled['stationary_window_end_ns']) <= cutoff):
                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_SETTLE_MISSING')
            if (not self._diagnostic_observation_checkpoint(frozen, gcfg, 'supplemental observation reached')
                    or not self._validate_measured_observation_envelope(frozen, gcfg, 'supplemental observation')):
                return dict(failed=True)
            reached = self._wait_for_fresh_observation_camera_target_range(frozen, gcfg, cutoff)
            if not reached.ok:
                raise ValueError(reached.code + ': ' + reached.reason)
            camera = getattr(self, '_last_observation_camera_range_evidence', None)
            if (not isinstance(camera, dict) or positive_ns(camera['image_source_stamp_ns']) < cutoff
                    or positive_ns(camera['observation_stamp_ns']) < cutoff):
                raise ValueError('OBSERVE_RETURN_SUPPLEMENTAL_FRESH_SOURCE_MISSING')
            if not self._diagnostic_observation_checkpoint(frozen, gcfg, 'supplemental observation sampled'):
                return dict(failed=True)
            now = rospy.Time.now().to_nsec()
            budget = gcfg.get('near_field_replan_timeout_sec', 90.)
            if isinstance(budget, bool) or not isinstance(budget, (int, float)) or not math.isfinite(budget) or not 0. < budget <= 300.:
                raise ValueError('OBSERVE_RETURN_CONTACT_BUDGET_INVALID')
            deadline = min(positive_ns(session['deadline_ns']), now + int(budget * 1e9))
            if deadline <= now:
                raise ValueError('OBSERVE_RETURN_SESSION_EXPIRED')
            self._set_near_field_active(True, force=True, budget_sec=(deadline-now)/1e9,
                reference_plan=frozen, absolute_deadline_sec=deadline/1e9)
            self.set_state(GraspStages.PLAN_PREGRASP,
                'supplemental observation sampled; requiring measured registration and fresh contact evidence')
            rospy.loginfo('Supplemental observation completed: %s', json.dumps(dict(
                session_id=session['session_id'], view_index=next_session['view_index'],
                view_request_id=request_id, view_plan_id=frozen.plan_id,
                origin_plan_id=session['origin_plan_id'], deadline_ns=session['deadline_ns'],
                sampling_after_ns=cutoff, camera_evidence=camera), allow_nan=False))
            return dict(retry=True, current_plan=frozen,
                minimum_stamp_ns=max(now, cutoff + 1), phase_deadline_sec=deadline/1e9)
        except Exception as exc:
            self.set_state(GraspStages.FAILED, 'OBSERVE_RETURN_SUPPLEMENTAL_FAILED: ' + str(exc))
            return dict(failed=True)

    def _execution_result_message(self, result, message, slot_released=False):
        reference = self._diagnostic_observation_sample_reference
        if result and reference is not None:
            # The recorder consumes the first inactive terminal publication.
            # Classification must be correct during base start_cb cleanup,
            # before control returns to the mode wrapper.
            return ('DIAGNOSTIC_OBSERVATION_COMPLETE: audit_path=%s audit_sha256=%s; '
                    'no grasp performed' % (reference['path'], reference['sha256']))
        return super()._execution_result_message(result, message, slot_released=slot_released)

    def _diagnostic_observation_only(self):
        report = self._observe_return_bound_report or {}
        metadata = report.get('observe_return', {})
        diagnostic = metadata.get('diagnostic_observation_only', False)
        trigger = metadata.get('trigger_code')
        if trigger == _SILHOUETTE_TRIGGER or diagnostic is not False:
            if (trigger != _SILHOUETTE_TRIGGER or diagnostic is not True
                    or metadata.get('phase') != 'observation_prefix'):
                raise ValueError('DIAGNOSTIC_OBSERVATION_TRIGGER_INVALID')
            return True
        return False

    def _diagnostic_observation_checkpoint(self, plan, gcfg, label):
        self._observe_return_live()
        if not self._execution_checkpoint(plan, gcfg, label):
            return False
        # Preserve the same-generation loss latch, including the original
        # 4 mm support-plane contradiction; a later good frame cannot erase it.
        failure = self._near_field_wait_failure(plan, gcfg)
        if failure is not None:
            self.set_state(GraspStages.FAILED, '%s: %s' % (failure.code, failure.reason))
            return False
        return True

    def _diagnostic_observation_output_directory(self):
        reference = self._observe_return_bound_reference
        if not isinstance(reference, dict):
            raise ValueError('DIAGNOSTIC_OBSERVATION_AUDIT_REFERENCE_MISSING')
        path = os.path.realpath(reference['path'])
        root = os.path.realpath(_EXPERIMENT_ROOT)
        if (not os.path.ismount(_EXPERIMENT_MOUNT)
                or os.path.commonpath((root, path)) != root):
            raise ValueError('DIAGNOSTIC_OBSERVATION_EXTERNAL_STORAGE_UNAVAILABLE')
        with open(path, 'rb') as source:
            payload = source.read()
        if (hashlib.sha256(payload).hexdigest() != reference['sha256']
                or json.loads(payload) != self._observe_return_bound_report):
            raise ValueError('DIAGNOSTIC_OBSERVATION_BOUND_AUDIT_CHANGED')
        return os.path.dirname(path)

    def _execute_diagnostic_observation(self, plan, gcfg, observation_pose,
                                        move_pose, strict_execute_pose, write_audit):
        """One strict observation, then source-bound evidence only.

        Never enter the grasp executor: no gripper command, range correction,
        contact replan or return. A failed gate is terminal, not a retry.
        """
        try:
            if not self._diagnostic_observation_only():
                raise ValueError('DIAGNOSTIC_OBSERVATION_TRIGGER_REQUIRED')
            report = self._observe_return_bound_report
            session = self._observe_return_session
            metadata = report['observe_return']
            if (self._observe_return_report(plan) != report
                    or session is None or session['phase'] != 'observing'
                    or session['origin_plan_id'] != plan.plan_id
                    or session['target_track_id'] != plan.target_track_id
                    or session['target_epoch'] != metadata['target_epoch']
                    or not callable(strict_execute_pose)):
                raise ValueError('DIAGNOSTIC_OBSERVATION_BOUND_CONTEXT_INVALID')
            if gcfg.get('observation_camera_target_range_check_enabled') is not True:
                raise ValueError('DIAGNOSTIC_OBSERVATION_FRESH_RANGE_REQUIRED')
            directory = self._diagnostic_observation_output_directory()
            source_key = (json.dumps(self._mode_bound_selection, sort_keys=True),
                          metadata['target_epoch'], metadata['origin_snapshot_stamp_ns'])
            if source_key in self._diagnostic_observation_consumed_sources:
                raise ValueError('DIAGNOSTIC_OBSERVATION_ALREADY_ATTEMPTED')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation entry'):
                return False
            self._diagnostic_observation_consumed_sources.add(source_key)
            self._last_motion_settle_sample = None
            self._last_measured_observation_envelope_sample = None
            self._last_observation_camera_range_evidence = None
            if not self._plan_and_execute_pose(
                    GraspStages.MOVE_PREGRASP, 'diagnostic 6D observation',
                    observation_pose, move_pose, 'diagnostic 6D observation',
                    execution_plan_id=plan.plan_id, execution_plan=plan, gcfg=gcfg,
                    execute_pose=strict_execute_pose,
                    allow_post_failure_observation_validation=False):
                return False
            # The shared executor already requires a fresh independent
            # post-action stationary window; sampling adds no physical action.
            settled = deepcopy(getattr(self, '_last_motion_settle_sample', None))
            if not isinstance(settled, dict):
                raise ValueError('DIAGNOSTIC_OBSERVATION_SETTLE_EVIDENCE_MISSING')
            cutoff = rospy.Time.now().to_nsec()
            if (positive_ns(settled['stationary_window_end_ns']) > cutoff
                    or positive_ns(settled['stationary_window_start_ns'])
                       >= positive_ns(settled['stationary_window_end_ns'])):
                raise ValueError('DIAGNOSTIC_OBSERVATION_SETTLE_EVIDENCE_INVALID')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation arrival'):
                return False
            if not self._validate_measured_observation_envelope(
                    plan, gcfg, 'diagnostic 6D observation'):
                return False
            result = self._wait_for_fresh_observation_camera_target_range(plan, gcfg, cutoff)
            if not result.ok:
                self.set_state(GraspStages.FAILED, '%s: %s' % (result.code, result.reason))
                return False
            camera = deepcopy(getattr(self, '_last_observation_camera_range_evidence', None))
            if (not isinstance(camera, dict)
                    or positive_ns(camera['image_source_stamp_ns']) < cutoff
                    or positive_ns(camera['observation_stamp_ns']) < cutoff):
                raise ValueError('DIAGNOSTIC_OBSERVATION_FRESH_SOURCE_MISSING')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation sampling'):
                return False
            self._diagnostic_observation_output_directory()
            marker = dict(version=1, code='DIAGNOSTIC_OBSERVATION_COMPLETE',
                diagnostic_observation_only=True, grasp_completed=False,
                contact_and_return_authorized=False, gripper_command_submitted=False,
                additional_motion_submitted=False, selection=deepcopy(self._mode_bound_selection),
                plan_id=plan.plan_id, target_track_id=plan.target_track_id,
                target_epoch=metadata['target_epoch'], session_id=session['session_id'],
                origin_snapshot_stamp_ns=metadata['origin_snapshot_stamp_ns'],
                source_stamp_ns=camera['image_source_stamp_ns'], sampling_after_ns=cutoff,
                sampling_marker_only=True, physical_contour_certified=False,
                original_audit_reference=deepcopy(self._observe_return_bound_reference),
                stationary_window=settled, camera_range=camera,
                measured_observation_envelope=deepcopy(self._last_measured_observation_envelope_sample))
            session_digest = hashlib.sha256(session['session_id'].encode()).hexdigest()[:24]
            path = os.path.join(directory, 'diagnostic_observation_%s_%s_%d.json' % (
                plan.plan_id, session_digest, camera['image_source_stamp_ns']))
            if os.path.exists(path):
                raise ValueError('DIAGNOSTIC_OBSERVATION_SAMPLE_ALREADY_EXISTS')
            reference = write_audit(path, marker)
            with open(reference['path'], 'rb') as source:
                saved = source.read()
            if hashlib.sha256(saved).hexdigest() != reference['sha256'] or json.loads(saved) != marker:
                raise ValueError('DIAGNOSTIC_OBSERVATION_SAMPLE_WRITE_FAILED')
            if not self._diagnostic_observation_checkpoint(plan, gcfg, 'diagnostic observation saved'):
                return False
            self._diagnostic_observation_sample_reference = reference
            rospy.loginfo('Diagnostic observation source sample: %s', json.dumps(marker, allow_nan=False))
            self.set_state(GraspStages.SUCCESS, 'DIAGNOSTIC_OBSERVATION_COMPLETE: '
                'source_stamp_ns=%d audit_path=%s audit_sha256=%s; no grasp performed' % (
                    camera['image_source_stamp_ns'], reference['path'], reference['sha256']), True)
            return True
        except Exception as exc:
            self.set_state(GraspStages.FAILED, 'DIAGNOSTIC_OBSERVATION_FAILED: ' + str(exc))
            return False

    def _before_near_field_contact_freeze(self, candidate, current_plan, gcfg):
        """None permits freezing; a retry advances only the new strategy phase."""
        selection = self._mode_bound_selection or {}
        if selection.get('strategy') != 'observe_return':
            return None
        try:
            if self._diagnostic_observation_only():
                raise ValueError('DIAGNOSTIC_OBSERVATION_CONTACT_AND_RETURN_FORBIDDEN')
            self._observe_return_live()
            session = self._observe_return_session
            origin_plan = self._observe_return_origin_plan or current_plan
            active_plan_id = (session.get('view_plan_id', session['origin_plan_id']) if session else None)
            if session is None or current_plan.plan_id != active_plan_id:
                raise ValueError('OBSERVE_RETURN_ORIGIN_CONTEXT_MISSING')
            if session['phase'] == 'returned':
                report = self._observe_return_report(candidate, allow_returned=True)
                metadata = report['observe_return']
                if (metadata.get('phase') != 'returned_contact'
                        or metadata.get('session_id') != session['session_id']
                        or metadata.get('origin_plan_id') != origin_plan.plan_id
                        or type(metadata.get('target_epoch')) is not int
                        or metadata['target_epoch'] != session['target_epoch']
                        or any(positive_ns(metadata[key]) != positive_ns(session[key])
                               for key in ('origin_snapshot_stamp_ns', 'return_verified_stamp_ns',
                                           'deadline_ns', 'contact_deadline_ns'))
                        or candidate.target_track_id != session['target_track_id']
                        or candidate.header.stamp.to_nsec() <= positive_ns(session['return_verified_stamp_ns'])
                        or rospy.Time.now().to_nsec() >= positive_ns(session['contact_deadline_ns'])):
                    raise ValueError('OBSERVE_RETURN_FINAL_CONTACT_BINDING_INVALID')
                return None
            if session['phase'] != 'observing' or candidate.fused_view_count < 2:
                raise ValueError('OBSERVE_RETURN_ADDITIONAL_VIEW_REQUIRED')
            if candidate.target_track_id != session['target_track_id']:
                raise ValueError('OBSERVE_RETURN_TARGET_CHANGED')
            returning = dict(session, phase='returning', stamp_ns=str(rospy.Time.now().to_nsec()))
            self._observe_return_publish(returning)
            self._observe_return_session = returning
            self.set_state(GraspStages.MOVE_PREGRASP, 'returning to initial observation pose')
            proof = self._observe_return_gateway('/supervisor/return_observe_return_origin', origin_plan,
                bound_plan=current_plan, gcfg=gcfg)
            origin_proof = self._observe_return_origin_proof or {}
            if (proof.get('code') != 'OBSERVE_RETURN_MEASURED_ARRIVAL'
                    or proof.get('session_id') != session['session_id']
                    or proof.get('post_command_feedback_valid') is not True
                    or proof.get('controller_action_completed') is not True
                    or proof.get('command_submission_attempted') is not True
                    or any(key not in origin_proof or proof.get(key) != origin_proof[key]
                           for key in ('epoch_ns', 'plan_digest', 'model_sha256',
                                       'origin_accepted_stamp_ns', 'origin_joint_positions_rad',
                                       'origin_tool_pose', 'origin_goal_sha256'))):
                raise ValueError('OBSERVE_RETURN_MEASURED_RETURN_REQUIRED')
            verified = positive_ns(proof['accepted_stamp_ns'])
            completed = positive_ns(proof['completion_stamp_ns'])
            now = rospy.Time.now().to_nsec()
            if not positive_ns(returning['stamp_ns']) < completed < verified <= now:
                raise ValueError('OBSERVE_RETURN_FRESH_RETURN_PROOF_REQUIRED')
            budget = float(gcfg.get('near_field_replan_timeout_sec', 90.))
            if not math.isfinite(budget) or not 0. < budget <= 300.:
                raise ValueError('OBSERVE_RETURN_CONTACT_BUDGET_INVALID')
            contact_deadline = min(positive_ns(session['deadline_ns']), now + int(budget * 1e9))
            if contact_deadline <= now:
                raise ValueError('OBSERVE_RETURN_SESSION_EXPIRED')
            returned = dict(returning, phase='returned', stamp_ns=str(now),
                return_verified_stamp_ns=str(verified), contact_deadline_ns=str(contact_deadline))
            self._observe_return_publish(returned)
            self._observe_return_session = returned
            self._set_near_field_active(True, force=True, budget_sec=(contact_deadline-now)/1e9,
                reference_plan=current_plan, absolute_deadline_sec=contact_deadline/1e9)
            self.set_state(GraspStages.PLAN_PREGRASP,
                'initial pose measured; replanning complete grasp from returned state')
            rospy.loginfo('Observe-return measured arrival: %s', json.dumps(proof))
            return dict(retry=True, minimum_stamp_ns=max(verified + 1, now),
                        phase_deadline_sec=contact_deadline/1e9)
        except Exception as exc:
            self.set_state(GraspStages.FAILED, 'OBSERVE_RETURN_FAILED: ' + str(exc))
            return dict(failed=True)
