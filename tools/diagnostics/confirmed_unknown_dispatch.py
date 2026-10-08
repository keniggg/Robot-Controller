"""Select one existing unknown-target entry point; never grant motion authority.

The metric runner and RGB gateway keep all original admission/execution gates.
This module only binds a finite, serial handoff to the confirmed selection.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.rgb_observation_source import camera_geometry_snapshot, digest
from alicia_flexible_grasp.robot.observation_view_limits import DEFAULT_OBSERVATION_VIEW_PARAMETERS


ENTRY_WAIT_SEC = 20.
RGB_WAIT_SEC = 20.
RGB_MAX_PLANS = 3


def positive_ns(value):
    if isinstance(value, str) and value.isdecimal():
        value = int(value)
    if type(value) is not int or value <= 0:
        raise ValueError('invalid_dispatch_timestamp')
    return value


def context_parameters(get):
    """Comparable live parameters; exact TF/FK/source proof remains downstream."""
    model = get('/robot_description', '')
    if not isinstance(model, str) or not model.strip():
        raise ValueError('dispatch_robot_model_unavailable')
    return deepcopy(dict(camera=camera_geometry_snapshot(get('/camera')),
        perception=get('/unknown_perception', {}), handeye=get('/handeye', {}),
        model_sha256=hashlib.sha256(model.encode()).hexdigest(),
        observation_view_parameters={key: get('/grasp_6d/remote/'+key, value)
            for key, value in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()}))


class EntryDecision:
    """Caller serializes callbacks and decisions with its condition lock."""
    def __init__(self, selection, max_source_age_sec=.8):
        self.selection = parse_selection(selection)
        if self.selection['mode'] != 'unknown' or self.selection['strategy'] != 'observe_return':
            raise ValueError('confirmed_unknown_observe_return_required')
        if (isinstance(max_source_age_sec, bool) or not isinstance(max_source_age_sec, (float, int))
                or not math.isfinite(max_source_age_sec) or max_source_age_sec <= 0):
            raise ValueError('invalid_dispatch_source_age_limit')
        self.max_age_ns = int(min(.8, max_source_age_sec)*1e9)
        self.status = None
        self.epoch_ns = None
        self.terminal = ''

    def observe_epoch(self, stamp_ns):
        stamp_ns = positive_ns(stamp_ns)
        if self.epoch_ns is not None and stamp_ns != self.epoch_ns:
            self.terminal = 'dispatch_control_reference_epoch_changed'
        self.epoch_ns = stamp_ns

    def observe(self, status):
        if not isinstance(status, dict):
            return
        if (status.get('mode') != self.selection['mode']
                or type(status.get('generation')) is not int
                or status['generation'] != self.selection['generation']
                or type(status.get('selection_stamp_ns')) is not int
                or status['selection_stamp_ns'] != self.selection['stamp_ns']):
            return  # A latched status from another selection is not this attempt.
        self.status = deepcopy(status)
        if status.get('target_lost') is True:
            self.terminal = 'dispatch_target_lost'

    def require_current(self, selection):
        if parse_selection(selection) != self.selection:
            self.terminal = 'dispatch_selection_changed'
        if self.terminal:
            raise ValueError(self.terminal)

    def fresh(self, stamp_ns, now_ns):
        return (type(stamp_ns) is int and type(now_ns) is int
            and self.epoch_ns is not None
            and max(self.selection['stamp_ns'], self.epoch_ns) < stamp_ns <= now_ns
            and now_ns-stamp_ns <= self.max_age_ns)

    def choose(self, selection, now_ns):
        self.require_current(selection)
        status = self.status
        if status is None or self.epoch_ns is None:
            return 'wait'
        if any(type(status.get(key)) is not bool for key in ('target_locked', 'target_lost')):
            raise ValueError('dispatch_target_state_malformed')
        if status['target_locked'] and status.get('state') != 'ready':
            # Hold/pending are deliberately false. Only the producer decides
            # whether its original .8 s identity reference survives; waiting
            # here neither renews it nor grants the initial RGB entry point.
            return 'wait'
        stamp = status.get('source_stamp_ns')
        if not self.fresh(stamp, now_ns):
            return 'wait'
        if status.get('color_stamp_ns') != stamp or status.get('depth_stamp_ns') != stamp:
            raise ValueError('dispatch_rgbd_source_mismatch')
        # Real metric detection has priority; interest never replaces a lock.
        if status['target_locked'] and status.get('state') == 'ready':
            return 'metric'
        interest = status.get('observation_interest') or {}
        if interest.get('observation_interest_ready') is not True:
            return 'wait'
        try:
            context, track = interest['source_context'], interest['rgb_track']
            stamps = track['source_stamps_ns']
            if (interest['source_stamp_ns'] != stamp
                    or interest['source_id'] != digest(context)
                    or parse_selection(context['selection']) != self.selection
                    or interest['generation'] != self.selection['generation']
                    or interest['selection_stamp_ns'] != self.selection['stamp_ns']
                    or context['epoch_ns'] != self.epoch_ns or interest['epoch_ns'] != self.epoch_ns
                    or track['confirmed'] is not True or track['source_id'] != interest['source_id']
                    or track['generation'] != self.selection['generation']
                    or not isinstance(stamps, (list, tuple)) or len(stamps) < 3
                    or any(type(value) is not int for value in stamps)
                    or not max(self.epoch_ns, self.selection['stamp_ns']) < stamps[0]
                    or stamps[-1] != stamp or any(a >= b for a, b in zip(stamps, stamps[1:]))
                    or interest.get('reference_id')):
                raise ValueError('dispatch_rgb_interest_context_changed')
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError('dispatch_rgb_interest_malformed') from exc
        return 'rgb'

    def require_live_parameters(self, original, current, *, rgb=False):
        if digest(original) != digest(current):
            raise ValueError('dispatch_source_parameters_changed')
        if rgb:
            context = self.status['observation_interest']['source_context']
            if any(digest(context.get(key)) != digest(value) for key, value in current.items()):
                raise ValueError('dispatch_rgb_interest_live_context_changed')

    def require_measured_handoff(self, report, selection, now_ns, monotonic_now):
        """A successful camera move/exit zero alone never starts grasp planning."""
        if self.choose(selection, now_ns) != 'metric':
            raise ValueError('dispatch_fresh_measured_target_required')
        try:
            if (report.get('preview_only') is not False or report.get('failure')
                    or report.get('measured_target_available') is not True
                    or report.get('observation_completed') is not True
                    or report.get('grasp_started') is not False
                    or report.get('torque_commands_sent') is not False):
                raise ValueError('dispatch_rgb_acquisition_not_measured')
            requests = report['requests']
            if not isinstance(requests, list) or not 1 <= len(requests) <= 6:
                raise ValueError('dispatch_rgb_receipt_missing')
            successful = [row for row in requests if row.get('success') is True]
            ledger = successful[-1]['ledger']
            settled = positive_ns(ledger['settled_ns'])
            if (ledger['phase'] != 'awaiting_measured_target'
                    or parse_selection(ledger['selection']) != self.selection
                    or ledger['duration_sec'] != 120. or ledger['max_observations'] != RGB_MAX_PLANS
                    or type(ledger['attempts']) is not int or not 1 <= ledger['attempts'] <= RGB_MAX_PLANS
                    or not now_ns < positive_ns(ledger['deadline_ns'])
                    or not math.isfinite(ledger['deadline_monotonic'])
                    or not monotonic_now < ledger['deadline_monotonic']):
                raise ValueError('dispatch_rgb_owner_or_budget_changed')
            reported, live = report['handoff'], self.status['rgb_acquisition_handoff']
            for handoff in (reported, live):
                if (handoff['state'] != 'measured_ready'
                        or handoff.get('metric_geometry_authority') is not True
                        or handoff.get('motion_authorized') is not False
                        or any(handoff.get(key) != ledger.get(key) or not ledger.get(key)
                            for key in ('request_id', 'reference_id', 'source_id', 'observation_index'))
                        or not self.fresh(handoff['source_stamp_ns'], now_ns)
                        or not settled < handoff['source_stamp_ns'] <= self.status['source_stamp_ns']):
                    raise ValueError('dispatch_measured_handoff_binding_changed')
            if reported['source_stamp_ns'] != live['source_stamp_ns']:
                raise ValueError('dispatch_measured_handoff_source_changed')
        except (KeyError, IndexError, TypeError, AttributeError) as exc:
            raise ValueError('dispatch_rgb_receipt_malformed') from exc
        return dict(reference_id=ledger['reference_id'], request_id=ledger['request_id'],
            source_id=ledger['source_id'], settled_ns=settled,
            handoff_source_stamp_ns=live['source_stamp_ns'],
            current_source_stamp_ns=self.status['source_stamp_ns'])


def acquisition_result(log_path, archive_root):
    """Read only this child's unique recorded result, never the latest directory."""
    root = Path(archive_root).resolve()
    sessions = []
    with Path(log_path).open() as stream:
        for line in stream:
            if line.startswith('SESSION '):
                sessions.append(Path(line[len('SESSION '):].strip()).resolve())
    if (len(sessions) != 1 or sessions[0].parent != root
            or not sessions[0].name.startswith('rgb_acquisition_run_')):
        raise ValueError('dispatch_exact_rgb_result_path_required')
    path = sessions[0]/'result.json'
    if path.resolve().parent != sessions[0] or path.stat().st_size > 8*1024*1024:
        raise ValueError('dispatch_rgb_result_artifact_invalid')
    raw = path.read_bytes()
    return json.loads(raw), dict(path=str(path), sha256=hashlib.sha256(raw).hexdigest())


def dispatch_attempt(decision, *, condition, selection, now_ns, monotonic, parameters,
                     run_rgb, run_metric, record, is_shutdown):
    """Wait only before admission; neither executing child receives a timeout.

    Callbacks keep recording loss/epoch changes while the RGB child owns its
    blocking RPC. They do not kill it or start a competing metric runner.
    """
    baseline = parameters()
    deadline = monotonic()+ENTRY_WAIT_SEC
    while True:
        current = selection()
        with condition:
            branch = decision.choose(current, now_ns())
            if branch != 'wait':
                break
            if is_shutdown() or monotonic() >= deadline:
                raise ValueError('dispatch_no_fresh_entry_within_budget')
            condition.wait(min(.05, max(0., deadline-monotonic())))
    current_parameters = parameters()
    current = selection()
    with condition:
        # Parameter reads and logging cannot make an earlier ready frame fresh.
        branch = decision.choose(current, now_ns())
        if branch == 'wait':
            raise ValueError('dispatch_entry_changed_before_launch')
        decision.require_live_parameters(baseline, current_parameters, rgb=branch == 'rgb')
    record(dict(phase=branch+'_entry', source_stamp_ns=decision.status['source_stamp_ns'],
        selection=decision.selection, motion_authorized=False,
        budget_policy='RGB acquisition and observe_return retain separate original budgets',
        rgb_acquisition_duration_sec=120., rgb_acquisition_max_plans=RGB_MAX_PLANS))
    current_parameters = parameters()
    current = selection()
    with condition:
        branch = decision.choose(current, now_ns())
        if branch == 'wait':
            raise ValueError('dispatch_entry_changed_before_launch')
        decision.require_live_parameters(baseline, current_parameters, rgb=branch == 'rgb')
    if branch == 'rgb':
        code, report, artifact = run_rgb(decision.selection)
        record(dict(phase='rgb_finished', returncode=code, artifact=artifact,
            measured_target_available=report.get('measured_target_available'),
            observation_completed=report.get('observation_completed'),
            failure=report.get('failure'), latest_interest_rejection=report.get('latest_interest_rejection'),
            request_messages=[row.get('message') for row in report.get('requests', [])]))
        if code != 0:
            raise ValueError('dispatch_rgb_acquisition_failed')
        if report.get('measured_target_available') is not True:
            raise ValueError('dispatch_rgb_acquisition_not_measured')
        # A transient false may race the child's successful handoff receipt.
        # Wait only within that SAME handoff source's remaining original age,
        # never for a renewed arbitrary status or another acquisition episode.
        stamp = (report.get('handoff') or {}).get('source_stamp_ns')
        if type(stamp) is not int:
            raise ValueError('dispatch_rgb_receipt_malformed')
        handoff_deadline = monotonic()+max(0., (stamp+decision.max_age_ns-now_ns())/1e9)
        while True:
            current = selection()
            with condition:
                branch_now = decision.choose(current, now_ns())
                if not decision.fresh(stamp, now_ns()):
                    raise ValueError('dispatch_measured_handoff_expired')
                if branch_now == 'metric':
                    break
                if is_shutdown() or monotonic() >= handoff_deadline:
                    raise ValueError('dispatch_fresh_measured_target_required')
                condition.wait(min(.05, max(0., handoff_deadline-monotonic())))
        current_parameters = parameters()
        current = selection()
        with condition:
            decision.require_current(current)
            decision.require_live_parameters(baseline, current_parameters)
            receipt = decision.require_measured_handoff(report, current, now_ns(), monotonic())
        record(dict(phase='measured_handoff_verified', **receipt))
    # The original runner performs fresh plan/audit/authority admission. Its
    # expected selection closes the process-start selection race independently.
    # Record intent first: disk I/O may block or deliver callbacks. Read the
    # live context again afterwards and leave no logging before child launch.
    record(dict(phase='metric_runner_start_requested', selection=decision.selection))
    current_parameters = parameters()
    current = selection()
    with condition:
        decision.require_live_parameters(baseline, current_parameters)
        if decision.choose(current, now_ns()) != 'metric':
            raise ValueError('dispatch_measured_entry_revoked')
        if branch == 'rgb':
            decision.require_measured_handoff(report, current, now_ns(), monotonic())
    code = run_metric(decision.selection)
    record(dict(phase='metric_runner_finished', returncode=code))
    return code
