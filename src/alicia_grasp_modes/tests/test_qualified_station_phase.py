"""Independent qualified-station phase lifecycle through the real task MRO.

Only transport, clock and the pre-existing origin fixture are synthetic.
Measurement parsing/ACK validation, target association, phase construction and
new phase ownership checks execute normally. This is not physical validation.
"""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS

import pytest

from test_observe_return_task import context, begin, at_ns, module, base
from alicia_grasp_modes import observe_return_measurement as contract
from alicia_grasp_modes.observe_return_task import committed_plan_digest
from alicia_flexible_grasp_supervisor.msg import ObjectPose, Grasp6DPlan
from geometry_msgs.msg import Pose


def actual_plan(source):
    def fill(target, value):
        for key, item in vars(value).items():
            if not hasattr(target, key):
                continue
            current = getattr(target, key)
            if isinstance(item, NS) and hasattr(current, '__slots__'):
                fill(current, item)
            elif key == 'poses':
                target.poses = [fill(Pose(), pose) for pose in item]
            else:
                setattr(target, key, deepcopy(item))
        return target
    plan = fill(Grasp6DPlan(), source)
    # The source fixture was audit-only and has no embedded geometry. Supply
    # explicit synthetic geometry, then run the original association gate.
    plan.object_geometry.header = deepcopy(plan.header)
    plan.object_geometry.valid = True
    plan.object_geometry.target_track_id = plan.target_track_id
    plan.object_geometry.pose_base.position.x = .1
    plan.object_geometry.pose_base.position.y = .02
    plan.object_geometry.pose_base.position.z = .3
    plan.object_geometry.pose_base.orientation.w = 1.
    plan.object_geometry.size_xyz_m.x = .03
    plan.object_geometry.size_xyz_m.y = .03
    plan.object_geometry.size_xyz_m.z = .04
    plan.object_geometry.support_normal_base.z = 1.
    return plan


@pytest.fixture
def qualified_context(context, monkeypatch):
    c = context
    n = c.node
    c.plan = actual_plan(c.plan)
    begin(c)
    n.__dict__.pop('_observation_measurement_scope')
    n.__dict__.pop('_set_near_field_active')
    n.__dict__.pop('set_state')
    c.events = []
    c.phases = []
    c.requests = []
    c.ack_filter = lambda participant, request: True
    c.after_ack = lambda participant, request: None
    c.cfg = dict(near_field_replan_enabled=True, near_field_replan_required=True,
                 near_field_strategy='single_snapshot_direct', near_field_replan_timeout_sec=12.)
    monkeypatch.setattr(module.rospy, 'get_param_cached',
                        lambda name, default=None: c.params.get(name, default))
    n._following_evidence_lock = threading.RLock()
    n._following_epoch_ns = c.proof['epoch_ns']
    n.stage = base.GraspStages.MOVE_PREGRASP
    n._bound_execution_plan = deepcopy(c.plan)
    n._bound_execution_plan_id = c.plan.plan_id
    n._bound_execution_plan_digest = committed_plan_digest(c.plan)
    n._execution_authority_revoked = False
    n.latest_target_geometry = None
    n._near_field_active = False
    n._near_field_phase_id = 0
    n.pub = NS(publish=lambda message: c.events.append(('state', message.state)))
    n.near_field_pub = NS(publish=lambda message: c.events.append(('active', message.data)))

    def phase_publish(message):
        c.events.append(('phase', message.active))
        c.phases.append(deepcopy(message))
    n.near_field_phase_pub = NS(publish=phase_publish)

    def publish(message):
        payload = json.loads(message.data)
        c.requests.append(payload)
        c.events.append(('request', payload['state']))
        stamps = []
        if payload['state'] == 'requalifying':
            stamps = [payload['arrival_stamp_ns'] + i*100_000_000 for i in (1, 2, 3)]
            c.now[0] = stamps[-1] + 1
            obj = ObjectPose()
            obj.header.stamp = at_ns(stamps[-1]); obj.header.frame_id = 'base_link'
            obj.pose_base.header = deepcopy(obj.header)
            obj.pose_base.pose.position = deepcopy(c.plan.object_geometry.pose_base.position)
            obj.detected = True; obj.label = 'unknown_small'; obj.confidence = .99
            n.latest_obj = obj
            assert n._observed_target_matches_plan(c.plan, obj, c.cfg)
        for participant in contract.PARTICIPANTS:
            if not c.ack_filter(participant, payload):
                continue
            ack = contract.measurement_ack(payload, participant, accepted=True,
                qualified=payload['state'] == 'requalifying', source_stamps_ns=stamps)
            n._observation_measurement_ack_cb(NS(data=json.dumps(ack)))
            c.events.append(('ack', payload['state'], participant))
            c.after_ack(participant, payload)
    n._observation_measurement_pub = NS(publish=publish)
    return c


def finish(c, view='first'):
    n = c.node
    if view == 'supplement':
        n._observe_return_session['view_index'] = 2
    elif view == 'return':
        n._observe_return_session['phase'] = 'returning'
    req = n._observation_measurement_begin(c.plan, 'return' if view == 'return' else 'observation')
    c.now[0] += 200_000_000
    n._last_motion_settle_sample = dict(epoch_ns=n._following_epoch_ns,
        stationary_window_start_ns=req['stamp_ns']+10_000_000,
        stationary_window_end_ns=req['stamp_ns']+100_000_000)
    n._observation_measurement_finish(c.plan, c.cfg)
    return deepcopy(n._observation_measurement_request)


@pytest.mark.parametrize('view', ['first', 'supplement', 'return'])
def test_qualified_pair_precedes_actual_phase_creation_and_reuse_never_renews(qualified_context, view):
    c = qualified_context
    done = finish(c, view)
    assert len(c.phases) == 1
    phase = c.phases[0]
    assert phase.active and phase.planning_stage == 'contact_replan'
    assert phase.reference_source_stamp.to_nsec() == done['source_stamps_ns'][-1]
    assert phase.reference_target_track_id == c.plan.target_track_id
    assert phase.reference_center_valid
    state_i = c.events.index(('state', 'PLAN_PREGRASP'))
    phase_i = c.events.index(('phase', True))
    for participant in contract.PARTICIPANTS:
        assert c.events.index(('ack', 'qualified', participant)) < state_i < phase_i
    assert phase.deadline.to_nsec() == min(done['deadline_ns'], phase.header.stamp.to_nsec()+12_000_000_000)
    c.now[0] += 1_000_000_000
    reused = c.node._ensure_qualified_station_phase(c.plan, c.cfg)
    assert reused == phase
    assert len(c.phases) == 1 and c.node._near_field_phase_id == 1
    assert c.node._observation_measurement_request == done
    assert len(c.gateways) == 1  # Only the fixture's pre-existing origin capture.


@pytest.mark.parametrize('change', ['lease', 'request', 'plan_digest', 'source', 'qualified_missing',
                                  'ack_missing', 'ack_replaced', 'phase_id', 'phase_deadline',
                                  'phase_reference', 'phase_inactive', 'internal_phase_id',
                                  'internal_phase_inactive', 'session_view', 'session_deadline'])
def test_reuse_rejects_changed_binding_without_new_phase(qualified_context, change):
    c = qualified_context; n = c.node
    done = finish(c)
    plan = deepcopy(c.plan)
    if change == 'lease': n._observation_measurement_request['lease_id'] += '-wrong'
    elif change == 'request': n._observation_measurement_request['request_id'] += '-wrong'
    elif change == 'plan_digest': plan.poses[0].position.x += .001
    elif change == 'source': n._observation_measurement_request['source_stamps_ns'][-1] += 1
    elif change == 'qualified_missing': n._observation_measurement_request['state'] = 'requalifying'
    elif change == 'ack_missing': n._observation_measurement_acks.pop((done['request_id'], 'remote'))
    elif change == 'ack_replaced': n._observation_measurement_acks[(done['request_id'], 'unknown')]['accepted'] = False
    elif change == 'phase_id': n._published_near_field_phase.phase_id += 1
    elif change == 'phase_deadline': n._published_near_field_phase.deadline += module.rospy.Duration(1.)
    elif change == 'phase_reference': n._published_near_field_phase.reference_source_stamp += module.rospy.Duration(.001)
    elif change == 'internal_phase_id': n._near_field_phase_id += 1
    elif change == 'internal_phase_inactive': n._near_field_active = False
    elif change == 'session_view': n._observe_return_session = dict(n._observe_return_session, view_index=2)
    elif change == 'session_deadline': n._observe_return_session = dict(n._observe_return_session, deadline_ns=str(c.now[0]+1_000_000_000))
    else: n._published_near_field_phase.active = False
    with pytest.raises((ValueError, RuntimeError)):
        n._ensure_qualified_station_phase(plan, c.cfg)
    assert len([phase for phase in c.phases if phase.active]) == 1


@pytest.mark.parametrize('fault', ['manual', 'inactive', 'shutdown', 'epoch', 'session_deadline',
                                 'phase_deadline', 'selection'])
def test_reuse_rechecks_current_owner_and_deadline(qualified_context, monkeypatch, fault):
    c = qualified_context; n = c.node
    finish(c)
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'inactive': n.active = False
    elif fault == 'shutdown': monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: True)
    elif fault == 'epoch': n._following_epoch_ns += 1
    elif fault == 'session_deadline': c.now[0] = int(n._observe_return_session['deadline_ns'])
    elif fault == 'phase_deadline': c.now[0] = c.phases[0].deadline.to_nsec()
    else: c.params[module.SELECTION_PARAM]['generation'] += 1
    with pytest.raises((ValueError, RuntimeError)):
        n._ensure_qualified_station_phase(c.plan, c.cfg)
    assert len([phase for phase in c.phases if phase.active]) == 1


def test_final_qualified_ack_owner_loss_does_not_publish_phase(qualified_context):
    c = qualified_context
    def invalidate(participant, request):
        if participant == 'remote' and request['state'] == 'qualified':
            c.params['/gui/joint_direct_mode'] = True
    c.after_ack = invalidate
    with pytest.raises((ValueError, RuntimeError)):
        finish(c)
    assert not c.phases and not any(event == ('state', 'PLAN_PREGRASP') for event in c.events)


def test_failed_qualification_never_publishes_phase(qualified_context):
    c = qualified_context
    def invalidate(participant, request):
        if participant == 'remote' and request['state'] == 'requalifying':
            c.node._observation_measurement_acks[(request['request_id'], participant)]['accepted'] = False
    c.after_ack = invalidate
    with pytest.raises((ValueError, RuntimeError)):
        finish(c)
    assert not c.phases


def test_original_finally_phase_deactivation_prevents_reuse(qualified_context):
    c = qualified_context
    finish(c)
    # The original start_cb finally owns deactivation; raw set_state never
    # claimed to retire a phase. Exercise that unchanged terminal operation.
    c.node._set_near_field_active(False)
    assert c.node._near_field_active is False
    with pytest.raises((ValueError, RuntimeError)):
        c.node._ensure_qualified_station_phase(c.plan, c.cfg)
    assert len([phase for phase in c.phases if phase.active]) == 1


def test_new_ready_between_actual_qualified_ack_and_phase_keeps_original_stamp(qualified_context):
    c = qualified_context
    def newer_frame(participant, request):
        if participant == 'remote' and request['state'] == 'qualified':
            newer = deepcopy(c.node.latest_obj)
            newer.header.stamp += module.rospy.Duration(0, 1)
            newer.pose_base.header.stamp = newer.header.stamp
            c.node.latest_obj = newer
            c.now[0] += 2
    c.after_ack = newer_frame
    done = finish(c)
    assert c.node.latest_obj.header.stamp.to_nsec() > done['source_stamps_ns'][-1]
    assert c.phases[0].reference_source_stamp.to_nsec() == done['source_stamps_ns'][-1]


@pytest.mark.parametrize('fault', ['request', 'ack', 'manual', 'epoch', 'expired', 'session_view', 'session_deadline'])
@pytest.mark.parametrize('existing', [False, True])
def test_owner_change_while_waiting_for_real_plan_lock_never_creates_or_reuses(
        qualified_context, monkeypatch, fault, existing):
    c = qualified_context; n = c.node
    if existing:
        done = finish(c)
    else:
        # Isolate construction after an actual completed measurement exchange;
        # automatic finish-to-phase ordering has separate unmodified tests.
        with monkeypatch.context() as delay_phase:
            delay_phase.setattr(n, '_ensure_qualified_station_phase', lambda *a: None)
            done = finish(c)
        assert not c.phases
    active_before = len([phase for phase in c.phases if phase.active])
    real = n._grasp6d_plan_guard()
    waiting = threading.Event()
    main_ident = threading.get_ident()
    class WaitingGuard:
        def __enter__(self):
            if threading.get_ident() != main_ident:
                waiting.set()
            real.acquire()
            return self
        def __exit__(self, *args):
            real.release()
    n._grasp6d_plan_lock = WaitingGuard()
    outcomes = []
    def worker():
        try:
            outcomes.append(('returned', n._ensure_qualified_station_phase(c.plan, c.cfg)))
        except Exception as error:
            outcomes.append(('rejected', str(error)))
    real.acquire()
    thread = threading.Thread(target=worker)
    thread.start()
    try:
        assert waiting.wait(2.)
        if fault == 'request':
            n._observation_measurement_request = dict(done, lease_id='new-lease')
        elif fault == 'ack':
            with n._observe_return_condition:
                n._observation_measurement_acks.pop((done['request_id'], 'remote'))
        elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch': n._following_epoch_ns += 1
        elif fault == 'session_view': n._observe_return_session = dict(n._observe_return_session, view_index=2)
        elif fault == 'session_deadline': n._observe_return_session = dict(n._observe_return_session, deadline_ns=str(c.now[0]+1_000_000_000))
        else: c.now[0] = (c.phases[0].deadline.to_nsec() if existing else int(done['deadline_ns']))
    finally:
        real.release()
        thread.join(3.)
    assert not thread.is_alive()
    assert len(outcomes) == 1 and outcomes[0][0] == 'rejected', outcomes
    assert len([phase for phase in c.phases if phase.active]) == active_before


@pytest.mark.parametrize('kind', ['ros', 'monotonic'])
def test_phase_budget_cannot_be_renewed_after_clock_expiry(qualified_context, monkeypatch, kind):
    c = qualified_context; n = c.node
    finish(c)
    phase = deepcopy(c.phases[0])
    if kind == 'ros':
        c.now[0] = phase.deadline.to_nsec()
    else:
        import alicia_grasp_modes.observe_return_task as task
        expired_monotonic = n._qualified_station_phase['monotonic_deadline']
        monkeypatch.setattr(task.time, 'monotonic', lambda: expired_monotonic)
    with pytest.raises((ValueError, RuntimeError)):
        n._ensure_qualified_station_phase(c.plan, dict(c.cfg, near_field_replan_timeout_sec=300.))
    assert len([p for p in c.phases if p.active]) == 1
    assert c.phases[0].deadline == phase.deadline


def test_returned_session_reuses_exact_returning_measurement_phase(qualified_context):
    c = qualified_context; n = c.node
    finish(c, 'return')
    phase = deepcopy(c.phases[0])
    n._observe_return_session = dict(n._observe_return_session, phase='returned',
        return_verified_stamp_ns=str(c.now[0]-1), contact_deadline_ns=str(phase.deadline.to_nsec()))
    c.now[0] += 200_000_000
    assert n._ensure_qualified_station_phase(c.plan, c.cfg) == phase
    assert len(c.phases) == 1 and n._near_field_phase_id == 1


@pytest.mark.parametrize('age_ns', [800_000_000, 800_000_001])
def test_newer_ready_cannot_refresh_expired_qualified_source_before_first_phase(
        qualified_context, monkeypatch, age_ns):
    c = qualified_context; n = c.node
    with monkeypatch.context() as delay_phase:
        delay_phase.setattr(n, '_ensure_qualified_station_phase', lambda *a: None)
        done = finish(c)
    c.now[0] = done['source_stamps_ns'][-1] + age_ns
    n.latest_obj = deepcopy(n.latest_obj)
    n.latest_obj.header.stamp = at_ns(c.now[0]-1)
    n.latest_obj.pose_base.header.stamp = at_ns(c.now[0]-1)
    with pytest.raises(ValueError, match='FRESH_COMPLETED_SOURCE_REQUIRED'):
        n._ensure_qualified_station_phase(c.plan, c.cfg)
    assert not any(p.active for p in c.phases)
    assert n._near_field_active is False


@pytest.mark.parametrize('fault', ['manual', 'inactive', 'epoch', 'selection'])
def test_owner_loss_during_actual_phase_publication_retires_phase(
        qualified_context, fault):
    c = qualified_context; n = c.node
    publish = n.near_field_phase_pub.publish
    def publish_then_invalidate(message):
        publish(message)
        if not message.active:
            return
        if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'inactive': n.active = False
        elif fault == 'epoch': n._following_epoch_ns += 1
        else: c.params[module.SELECTION_PARAM]['generation'] += 1
    n.near_field_phase_pub.publish = publish_then_invalidate
    with pytest.raises((ValueError, RuntimeError)):
        finish(c)
    assert [p.active for p in c.phases] == [True, False]
    assert n._near_field_active is False
    assert n._qualified_station_phase is None
    assert len(c.gateways) == 1  # Original fixture capture only; no contact action.
