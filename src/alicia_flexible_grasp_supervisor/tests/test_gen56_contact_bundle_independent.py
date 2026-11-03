"""Independent GEN56 handoff boundary checks under the isolated test runner.

Transport/feedback clocks are explicit test providers. The geometry negatives
below use the original complete CAD support validator with an actual mesh
vertex inside the target; they do not claim a physical grasp or replay R2.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes
from alicia_flexible_grasp.robot import observation_path_guard as geometry
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.robot.finger_cad_separation import load_finger_cad_model
from test_gen55_finger_cad_separation import XML
from test_gen9_height_three_build_independent import chain_fixture, base
from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, feedback
from test_station_compute_work_handoff import submitted, prepared_for
from test_gen11_contact_remote_receipt import contact_plan
from alicia_grasp_modes.station_contact_admission import KEY as CONTACT_RECEIPT
from alicia_grasp_modes.observe_return_measurement import measurement_digest
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from std_msgs.msg import String
from test_remote_modes import Publisher


def qualified_chain(monkeypatch, preliminary_cost=None):
    """Real builder/strict adapter/MuJoCo gate; explicit offline IO providers.

    Extend the older chain fixture's minimal transport response with the
    complete physical dependency schema. No production method is substituted.
    Those values are synthetic boundary inputs, not measured R2 feedback.
    """
    env = chain_fixture(monkeypatch)
    if preliminary_cost is not None:
        nominal = env.node._check_moveit_stable_candidate(env.candidate)
        nominal = replace(nominal, joint_path_cost=preliminary_cost[0],
                          joint_max_delta_rad=preliminary_cost[1])
        env.node._check_moveit_stable_candidate = lambda candidate: nominal
    env.node._geometry_invalidation_generation = env.prepared.request_invalidation_generation
    transport = base.rospy.ServiceProxy('offline_fixture', object)
    env.feedback_effect = [lambda evidence: None]
    env.params = {'/mujoco_digital_twin/unknown_lift_gate_enabled': False}
    monkeypatch.setattr(base.rospy, 'get_param',
                        lambda name, default=None: deepcopy(env.params.get(name, default)))

    def complete_response(request):
        response = transport(request)
        evidence = json.loads(response.evidence_json)
        stamp = len(env.service_calls) * 1_000_000
        sample = dict(epoch_ns=9_000_000_000, model_sha256='a' * 64,
            joint_names=['joint%d' % i for i in range(1, 7)],
            sdk_positions_rad=[0.] * 6, accepted_positions_rad=[0.] * 6,
            accepted_gripper_opening_m=.0498, source_stamp_ns=stamp,
            receipt_stamp_ns=stamp + 1)
        evidence.update(epoch_ns=sample['epoch_ns'], model_sha256=sample['model_sha256'],
            opening_m=.0498, selection=deepcopy(env.node._mode_selection),
            planning_scene_digest='b' * 64, source_sample=deepcopy(sample),
            completion_sample=deepcopy(sample))
        env.feedback_effect[0](evidence)
        response.evidence_json = json.dumps(evidence)
        return response

    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *a, **kw: complete_response)
    result = env.node._check_direct_registered_candidate(env.candidate)
    assert result.reachable, result.reason
    env.strict_result = result
    assert '_qualified_contact_bundle' in env.runtime
    assert len(env.builds) == 1 and len(env.plans) == 1 and not env.published
    env.initial_wire = base.wire_digest(env.builds[0])
    return env


def test_exact_qualified_bundle_is_reused_with_fresh_feedback_without_reconstruction(monkeypatch):
    env = qualified_chain(monkeypatch)
    initial_evidence = deepcopy(env.runtime['contact_sequence_precheck'])
    proposal = env.node._publish_selected_preview(env.candidate, ticket=env.prepared.ticket)
    assert len(env.builds) == 1 and len(env.published) == 1 and len(env.plans) == 1
    assert base.wire_digest(env.published[0]) == env.initial_wire
    assert base.wire_digest(proposal['rich_plan']) == env.initial_wire
    assert [r.plan.plan_id for r in env.service_calls][1:] == [env.published[0].plan_id] * 3
    assert len(env.service_calls) == 4
    current = env.runtime['contact_sequence_precheck']
    assert current['source_sample']['source_stamp_ns'] > initial_evidence['source_sample']['source_stamp_ns']
    assert current['completion_sample']['receipt_stamp_ns'] > initial_evidence['completion_sample']['receipt_stamp_ns']
    assert env.runtime['mujoco_selection']['passed'] is True
    assert all(r.deadline.to_sec() == pytest.approx(env.deadline) for r in env.service_calls)


FROZEN_CHANGES = (
    'plan_pose', 'plan_geometry', 'plan_id', 'legacy_pose', 'streaming_audit', 'proposal_metadata',
    'candidate_identity', 'candidate_aperture',
    'prepared_geometry', 'prepared_snapshot', 'prepared_refinement', 'prepared_deadline',
    'runtime_sequence', 'runtime_profile', 'grasp_config', 'gripper_geometry', 'jaw_axis',
    'mujoco_config', 'robot_model_param', 'grasp_param', 'gripper_param', 'selection_param',
    'geometry_generation', 'prepared_identity', 'ticket_request', 'ticket_generation', 'ticket_epoch',
)


def change_frozen_input(env, change):
    held_plan = env.runtime['_qualified_contact_bundle']['bundle']['rich_plan']
    if change == 'plan_pose': held_plan.poses[2].position.x += .001
    elif change == 'plan_geometry': held_plan.object_geometry.size_xyz_m.x += .001
    elif change == 'plan_id': held_plan.plan_id = 'changed-plan'
    elif change == 'legacy_pose':
        env.runtime['_qualified_contact_bundle']['bundle']['legacy_plan'].poses[2].position.x += .001
    elif change == 'streaming_audit':
        env.runtime['_qualified_contact_bundle']['bundle']['streaming_audit']['plan_id'] = 'changed-plan'
    elif change == 'proposal_metadata':
        env.runtime['_qualified_contact_bundle']['bundle']['proposal_metadata']['plan_id'] = 'changed-plan'
    elif change == 'candidate_identity': env.candidate.stable_candidate.target_track_id += '-changed'
    elif change == 'candidate_aperture':
        gate = replace(env.candidate.payload.geometry_gate, required_open_width_m=.041)
        env.candidate.payload = replace(env.candidate.payload, required_open_width_m=.041,
                                        geometry_gate=gate)
    elif change == 'prepared_geometry': env.prepared.geometry.changed_support = .001
    elif change == 'prepared_snapshot': env.prepared.snapshot.stamp_ns = 100
    elif change == 'prepared_refinement': env.prepared.refinement_binding = {'changed': True}
    elif change == 'prepared_deadline': env.prepared.near_field_phase_deadline_sec -= .01
    elif change == 'runtime_sequence': env.runtime['sequence'].grasp.pose.position.x += .001
    elif change == 'runtime_profile': env.runtime['adaptive_stage_profile'] = {'changed': True}
    elif change == 'grasp_config': env.node.grasp_config['changed'] = True
    elif change == 'gripper_geometry':
        env.node.gripper_geometry = replace(env.node.gripper_geometry,
            max_inner_gap_m=env.node.gripper_geometry.max_inner_gap_m-.001)
    elif change == 'jaw_axis': env.node.gripper_tool_jaw_axis = 'x'
    elif change == 'mujoco_config': env.node.mujoco_config['min_score'] += 1
    elif change == 'robot_model_param': env.params['/robot_description'] = '<changed-model/>'
    elif change == 'grasp_param': env.params['/grasp'] = {'changed': True}
    elif change == 'gripper_param': env.params['/gripper'] = {'changed': True}
    elif change == 'selection_param': env.params['/grasp_mode/selection'] = {'generation': 10}
    elif change == 'geometry_generation': env.node._geometry_invalidation_generation += 1
    elif change == 'prepared_identity': env.runtime['prepared'] = NS(**vars(env.prepared))
    elif change == 'ticket_request': env.prepared.ticket.request_id += 1
    elif change == 'ticket_generation': env.prepared.ticket.generation += 1
    elif change == 'ticket_epoch': env.prepared.ticket.target_epoch += 1
    else: raise AssertionError(change)


@pytest.mark.parametrize('change', FROZEN_CHANGES)
def test_frozen_plan_geometry_ticket_or_config_change_never_publishes(monkeypatch, change):
    env = qualified_chain(monkeypatch)
    calls = len(env.service_calls)
    change_frozen_input(env, change)
    with pytest.raises((base.CandidateContractError, base.StreamResultCancelled, RuntimeError)):
        env.node._publish_selected_preview(env.candidate, ticket=env.prepared.ticket)
    assert not env.published and len(env.builds) == 1
    assert len(env.service_calls) == calls


FEEDBACK_CHANGES = (
    'opening', 'epoch', 'model', 'scene', 'selection', 'source_sdk',
    'source_accepted', 'completion_sdk', 'completion_accepted', 'accepted_opening',
)


@pytest.mark.parametrize('change', FEEDBACK_CHANGES)
def test_fresh_strict_physical_dependency_change_never_publishes_old_bundle(monkeypatch, change):
    env = qualified_chain(monkeypatch)
    def alter(evidence):
        if change == 'opening': evidence['opening_m'] += .001
        elif change == 'epoch': evidence['epoch_ns'] += 1
        elif change == 'model': evidence['model_sha256'] = 'c' * 64
        elif change == 'scene': evidence['planning_scene_digest'] = 'd' * 64
        elif change == 'selection': evidence['selection']['generation'] += 1
        elif change == 'source_sdk': evidence['source_sample']['sdk_positions_rad'][0] += .001
        elif change == 'source_accepted': evidence['source_sample']['accepted_positions_rad'][0] += .001
        elif change == 'completion_sdk': evidence['completion_sample']['sdk_positions_rad'][0] += .001
        elif change == 'completion_accepted': evidence['completion_sample']['accepted_positions_rad'][0] += .001
        elif change == 'accepted_opening': evidence['completion_sample']['accepted_gripper_opening_m'] += .001
        else: raise AssertionError(change)
    env.feedback_effect[0] = alter
    with pytest.raises(base.CandidateContractError, match='changed'):
        env.node._publish_selected_preview(env.candidate, ticket=env.prepared.ticket)
    assert not env.published and len(env.builds) == 1 and len(env.service_calls) == 4


@pytest.mark.parametrize('fault', ['cancel', 'deadline'])
def test_qualified_bundle_does_not_extend_owner_or_phase_lifetime(monkeypatch, fault):
    env = qualified_chain(monkeypatch)
    if fault == 'cancel': env.state['revoked'] = True
    else: env.clock[0] = env.deadline
    with pytest.raises(base.StreamResultCancelled):
        env.node._publish_selected_preview(env.candidate, ticket=env.prepared.ticket)
    assert not env.published and len(env.service_calls) == 3


def test_mutating_a_returned_bundle_does_not_mutate_retained_plan(monkeypatch):
    env = qualified_chain(monkeypatch)
    copied = env.node._qualified_contact_preview_bundle(env.candidate, ticket=env.prepared.ticket)
    copied['rich_plan'].poses[2].position.x += .1
    copied['legacy_plan'].poses[2].position.y += .1
    copied['streaming_audit']['plan_id'] = 'changed'
    pristine = env.node._qualified_contact_preview_bundle(env.candidate, ticket=env.prepared.ticket)
    assert base.wire_digest(pristine['rich_plan']) == env.initial_wire
    assert pristine['proposal']['rich_plan'] is pristine['rich_plan']
    assert pristine['streaming_audit']['plan_id'] == pristine['rich_plan'].plan_id
    assert not env.published


@pytest.mark.parametrize('reported_score', ['final_strict', 'untrusted_changed'])
def test_final_metadata_uses_qualified_strict_cost_without_rebuilding_geometry(monkeypatch, reported_score):
    env = qualified_chain(monkeypatch, preliminary_cost=(7., 2.))
    expected = base.soft_candidate_cost(replace(env.candidate.soft_features,
        joint_path_cost=env.strict_result.joint_path_cost,
        joint_max_delta_rad=env.strict_result.joint_max_delta_rad), env.candidate.score_weights)
    selected = replace(env.candidate, moveit_result=env.strict_result,
        final_score=expected.total if reported_score == 'final_strict' else 1e6)
    bundle = env.node._qualified_contact_preview_bundle(selected, ticket=env.prepared.ticket)
    assert base.wire_digest(bundle['rich_plan']) == env.initial_wire and len(env.builds) == 1
    assert bundle['proposal']['score'] == pytest.approx(expected.total)
    assert bundle['proposal_metadata']['score'] == pytest.approx(expected.total)
    assert bundle['streaming_audit']['final_score'] == pytest.approx(expected.total)
    assert bundle['streaming_audit']['score_components'] == dict(expected.components)
    assert bundle['streaming_audit']['final_strict_moveit_result']['joint_path_cost'] == env.strict_result.joint_path_cost
    assert not env.published


def station_transaction(c, monkeypatch, tmp_path):
    """Actual station receipt, atomic audit and original preview publishers.

    Qualification is an explicit boundary provider here; the preceding tests
    exercise its real implementation and fresh strict-service adapter. The
    provider retains one frozen wire and rejects changed physical inputs only
    when the transaction asks for the mandatory fresh-feedback check.
    """
    node = c.node
    _, _, ticket = submitted(c)
    prepared = prepared_for(node, ticket)
    assert node._activate_prepared_geometry(prepared)
    plan = contact_plan(node, ticket)
    legacy = base.PoseArray()
    legacy.header = deepcopy(plan.header)
    legacy.poses = deepcopy(plan.poses)
    selected = NS(track_id=1, variant_index=0)
    selection = NS(selected=selected, checked=(), reachable=())
    bundle = dict(rich_plan=plan, legacy_plan=legacy, ticket=ticket,
        proposal=dict(rich_plan=plan, signature='independent-station', score=1., ticket=ticket,
                      expected_generation=prepared.request_invalidation_generation),
        proposal_metadata=dict(plan_id=plan.plan_id, signature='independent-station',
            score=1., expected_generation=prepared.request_invalidation_generation),
        streaming_audit=dict(plan_id=plan.plan_id, request_id=ticket.request_id))
    env = NS(node=node, ticket=ticket, prepared=prepared, selection=selection,
             bundle=bundle, events=[], physical_changed=False, after_commit=lambda: None)
    node.gate_audit_output_path = str(tmp_path / 'actual_atomic_preview_audit.json')
    node.mujoco_audit_output_path = str(tmp_path / 'unused_simulation_audit.json')

    def qualified(candidate, ticket=None, refresh_feedback=False):
        assert candidate is selected and ticket is env.ticket
        node._require_stream_ticket_current(ticket)
        env.events.append('fresh_strict' if refresh_feedback else 'frozen_inputs')
        if refresh_feedback and env.physical_changed:
            raise base.CandidateContractError('CONTACT_BUNDLE_INPUT_CHANGED',
                                             'fresh physical dependency changed')
        copied = node._copy_contact_bundle(bundle)
        copied['commit_callback'] = lambda: setattr(node, '_latest_streaming_audit',
                                                   deepcopy(copied['streaming_audit']))
        return copied
    monkeypatch.setattr(node, '_qualified_contact_preview_bundle', qualified)
    real_finalize = node._finalize_streaming_gate_audit
    def finalize(*args, **kwargs):
        result = real_finalize(*args, **kwargs)
        env.events.append('audit_committed')
        env.after_commit()
        return result
    monkeypatch.setattr(node, '_finalize_streaming_gate_audit', finalize)
    env.finalize = real_finalize
    for name, label in (('preview_plan_pub', 'legacy'), ('preview_rich_plan_pub', 'rich')):
        publisher = getattr(node, name)
        original_publish = publisher.publish
        def publish(message, _original=original_publish, _label=label):
            if _label == 'legacy' or getattr(message, 'valid', False):
                env.events.append(_label)
            return _original(message)
        monkeypatch.setattr(publisher, 'publish', publish)
    env.execution_before = len(node.rich_plan_pub.messages)
    env.valid_execution_before = sum(bool(m.valid) for m in node.rich_plan_pub.messages)
    env.valid_preview_before = sum(bool(m.valid) for m in node.preview_rich_plan_pub.messages)
    env.run = lambda: node._publish_audited_selected_contact_preview(
        prepared, selection, {}, {}, (), {'rows': []})
    return env


def assert_no_valid_station_publication(env):
    assert sum(bool(m.valid) for m in env.node.preview_rich_plan_pub.messages) == env.valid_preview_before
    assert sum(bool(m.valid) for m in env.node.rich_plan_pub.messages) == env.valid_execution_before
    proof = env.node._station_compute_work.get(int(env.ticket.request_id), {})
    assert not proof.get('contact_published') and not proof.get('contact_audit')


def test_actual_station_audit_is_committed_before_first_valid_preview(station, monkeypatch, tmp_path):
    env = station_transaction(station, monkeypatch, tmp_path)
    result = env.run()
    assert result['status'] == 'PREVIEW_READY'
    assert env.events.index('audit_committed') < env.events.index('legacy') < env.events.index('rich')
    assert len(env.node.rich_plan_pub.messages) == env.execution_before
    proof = env.node._station_compute_work[int(env.ticket.request_id)]
    assert proof['contact_published'] is True and proof['contact_audit']
    assert 'contact_prepared_audit' not in proof
    assert 'contact_audit_preparing_wire_sha256' not in proof
    reference = env.node._latest_gate_audit_reference
    assert reference['atomic_committed'] is True
    payload = (tmp_path / 'actual_atomic_preview_audit.json').read_bytes()
    assert hashlib.sha256(payload).hexdigest() == reference['report_sha256']
    report = json.loads(payload)
    assert report[CONTACT_RECEIPT] == proof['contact_audit']['receipt']
    assert report[CONTACT_RECEIPT]['motion_authorized'] is False


def test_prepared_committed_audit_is_not_a_published_contact_fact(station, monkeypatch, tmp_path):
    env = station_transaction(station, monkeypatch, tmp_path)
    env.node._stage_station_contact_preview_audit(env.bundle, env.ticket)
    env.finalize(env.prepared, env.selection, {}, 'PREVIEW_READY',
                 base_report={'rows': []}, publish_reference=False, preview_bundle=env.bundle)
    proof = env.node._station_compute_work[int(env.ticket.request_id)]
    assert proof['contact_prepared_audit']['reference']['atomic_committed'] is True
    env.node._require_station_contact_prepared_audit(env.bundle['rich_plan'], env.ticket)
    assert env.node._station_contact_audited_proposals({}) == []
    assert_no_valid_station_publication(env)
    env.node._abort_station_contact_preview_audit(env.ticket)
    assert 'contact_prepared_audit' not in proof and 'contact_proposal' not in proof


def prepared_publication_crossing(c, monkeypatch, tmp_path, boundary_index, crossing='geometry_ros'):
    """Run the real audit/publication transaction and cross its context clock.

    The original context, wait, quarantine, three-topic callbacks, atomic
    writer and publication guards execute. Qualification remains the explicit
    physical-transport provider used by station_transaction above.
    """
    from alicia_grasp_modes import station_compute_wait_remote as implementation
    env = station_transaction(c,monkeypatch,tmp_path)
    node = env.node
    env.crossed, env.results, env.wait_calls = [], [], []
    env.wait_entered, env.done = threading.Event(), threading.Event()
    env.prepared_checks = 0
    original_context = node._station_compute_context
    original_mono, mono_offset = implementation.time.monotonic, [0.]
    monkeypatch.setattr(implementation.time,'monotonic',lambda:original_mono()+mono_offset[0])
    def cross(contract):
        reference = node._station_compute_reference
        env.crossed.append(dict(contract=deepcopy(contract),reference=reference,
            geometry_monotonic=reference['geometry_monotonic'],
            identity_monotonic=reference['identity_monotonic'],
            audit_bytes=(tmp_path/'actual_atomic_preview_audit.json').read_bytes(),
            audit_reference=deepcopy(node._latest_gate_audit_reference),
            receipt=deepcopy(node._active_gate_audit_report[CONTACT_RECEIPT]),
            plan_wire=base.wire_digest(env.bundle['rich_plan'])))
        if crossing == 'geometry_ros': c.clock[0] = (contract['geometry_deadline_ns']+1_000_000)/1e9
        elif crossing == 'geometry_exact': c.clock[0] = contract['geometry_deadline_ns']/1e9
        elif crossing == 'geometry_mono':
            mono_offset[0] = reference['geometry_monotonic']-original_mono()+.001
        else: raise AssertionError(crossing)
    def context(contract, *args, **kwargs):
        result = original_context(contract,*args,**kwargs)
        if sys._getframe(1).f_code.co_name == '_require_station_contact_prepared_audit':
            env.prepared_checks += 1
            if env.prepared_checks == boundary_index and not env.crossed: cross(contract)
        return result
    monkeypatch.setattr(node,'_station_compute_context',context)
    if boundary_index=='commit_entry':
        original_certificate=node._station_compute_work_certificate
        def certificate(ticket):
            result=original_certificate(ticket)
            if (result and not env.crossed
                    and sys._getframe(1).f_code.co_name=='_publish_preview_plan'):
                cross(node._station_compute_reference['contract'])
            return result
        monkeypatch.setattr(node,'_station_compute_work_certificate',certificate)
    original_wait = node._stream_condition.wait
    def wait(*args, **kwargs):
        env.wait_calls.append(dict(now_ns=node._ros_source_clock_ns(),
            monotonic=implementation.time.monotonic(),timeout=kwargs.get('timeout',args[0] if args else None)))
        env.wait_entered.set()
        return original_wait(*args,**kwargs)
    monkeypatch.setattr(node._stream_condition,'wait',wait)
    def run():
        try: env.results.append(env.run())
        except Exception as exc: env.results.append(exc)
        finally: env.done.set()
    env.worker = threading.Thread(target=run,daemon=True)
    env.worker.start()
    return env


def wait_prepared_publication(env):
    # Event waits use real wall time; synthetic ROS time cannot hide a deadlock.
    for _ in range(300):
        if env.wait_entered.is_set() or env.done.is_set(): break
        env.done.wait(.01)
    assert env.wait_entered.is_set(), env.results
    assert not env.done.is_set(), env.results
    assert len(env.crossed) == 1
    for label,lock in [('stream',env.node._stream_condition),
                       ('geometry',env.node._geometry_state_guard())]:
        acquired=lock.acquire(timeout=.5)
        assert acquired, label+' lock retained during prepared-audit wait'
        lock.release()


def close_prepared_publication(env):
    if env.worker.is_alive():
        acquired=[]
        try:
            for lock in (env.node._stream_condition,env.node._geometry_state_guard()):
                assert lock.acquire(timeout=.5), 'prepared-audit cleanup lock unavailable'
                acquired.append(lock)
            env.node._station_compute_retire('independent prepared-publication test cleanup')
            env.node._stream_condition.notify_all()
        finally:
            for lock in reversed(acquired): lock.release()
    assert env.done.wait(3), env.results
    env.worker.join(3)
    assert not env.worker.is_alive()


@pytest.mark.parametrize('boundary_index',[1,2])
@pytest.mark.parametrize('crossing',['geometry_ros','geometry_exact','geometry_mono'])
def test_prepared_audit_context_crossing_rejoins_without_rewriting_committed_evidence(
        station,monkeypatch,tmp_path,boundary_index,crossing):
    from test_station_compute_work_handoff import next_row
    env = prepared_publication_crossing(station,monkeypatch,tmp_path,boundary_index,crossing)
    try:
        wait_prepared_publication(env)
        old = env.crossed[0]
        assert env.node._station_compute_reference is old['reference']
        assert env.node._station_compute_pending and env.node._station_compute_soft_wait
        assert old['reference']['contract'] == old['contract']
        assert old['reference']['geometry_monotonic'] == old['geometry_monotonic']
        assert old['reference']['identity_monotonic'] == old['identity_monotonic']
        assert_no_valid_station_publication(env)
        def partial(node):
            assert node._station_compute_pending
            assert not env.done.is_set()
            assert_no_valid_station_publication(env)
        env.events.append('new_source_delivery')
        successor = next_row(station,old['contract']['source_ns']+400_000_000,
                             old['contract'],partial=partial)
        assert env.done.wait(3), env.results
        assert len(env.results)==1 and isinstance(env.results[0],dict), env.results
        assert env.results[0]['status']=='PREVIEW_READY'
        assert (tmp_path/'actual_atomic_preview_audit.json').read_bytes()==old['audit_bytes']
        assert env.node._latest_gate_audit_reference==old['audit_reference']
        assert env.node._active_gate_audit_report[CONTACT_RECEIPT]==old['receipt']
        assert base.wire_digest(env.bundle['rich_plan'])==old['plan_wire']
        current = env.node._station_compute_reference['contract']
        assert current==successor['status']['station_compute_wait']
        assert current['phase_deadline_ns']==old['contract']['phase_deadline_ns']
        assert current['session_deadline_ns']==old['contract']['session_deadline_ns']
        assert env.node._ros_source_clock_ns()<current['geometry_deadline_ns']
        proof=env.node._station_compute_work[int(env.ticket.request_id)]
        assert proof['contact_audit']['receipt']==old['receipt']
        assert proof['contact_published'] is True and 'contact_prepared_audit' not in proof
        # Waiting after either prepared check must not turn the preceding
        # physical check into a reusable permission to publish later.
        tail=env.events[env.events.index('new_source_delivery')+1:]
        assert 'fresh_strict' in tail and tail.index('fresh_strict')<tail.index('legacy')
        assert len(env.node.rich_plan_pub.messages)==env.execution_before
    finally:
        close_prepared_publication(env)


@pytest.mark.parametrize('boundary_index',[1,2])
@pytest.mark.parametrize('fault',['manual','epoch','phase','identity','historical_source','feedback'])
def test_prepared_audit_rejoin_wait_preserves_owner_source_and_fresh_physical_rejection(
        station,monkeypatch,tmp_path,boundary_index,fault):
    from test_station_compute_work_handoff import next_row
    env = prepared_publication_crossing(station,monkeypatch,tmp_path,boundary_index)
    try:
        wait_prepared_publication(env)
        old=env.crossed[0]
        if fault=='manual': station.params['/gui/joint_direct_mode']=True
        elif fault=='epoch': env.node._measurement_remote_epoch_ns+=1
        elif fault=='phase': env.node._station_compute_task_state='MOVE_PREGRASP'
        elif fault=='identity': station.clock[0]=old['contract']['identity_deadline_ns']/1e9
        elif fault=='historical_source':
            ns=env.ticket.payload[0].sample_stamp_ns[0]
            changed=deepcopy(env.node._station_compute_ready_history[ns]['object'])
            changed.pose_base.pose.position.x+=.01
            env.node.object_cb(changed)
        elif fault=='feedback':
            env.node.latest_joint_state.position[0]+=.0001
            feedback(station,env.node._ros_source_clock_ns(),q=list(env.node.latest_joint_state.position))
            env.physical_changed=True
            env.events.append('new_source_delivery')
            next_row(station,old['contract']['source_ns']+400_000_000,old['contract'])
        with env.node._stream_condition: env.node._stream_condition.notify_all()
        assert env.done.wait(3), env.results
        assert len(env.results)==1 and isinstance(env.results[0],
            (base.StreamResultCancelled,base.CandidateContractError,ValueError)), env.results
        assert_no_valid_station_publication(env)
        assert (tmp_path/'actual_atomic_preview_audit.json').read_bytes()==old['audit_bytes']
        assert env.node._active_gate_audit_report[CONTACT_RECEIPT]==old['receipt']
        if fault=='feedback':
            assert 'fresh_strict' in env.events[env.events.index('new_source_delivery')+1:]
    finally:
        close_prepared_publication(env)


@pytest.mark.parametrize('boundary_index',[1,2])
@pytest.mark.parametrize('clock_kind',['ros','monotonic'])
def test_prepared_handoff_phase_limit_is_fixed_across_fresh_successors(
        station,monkeypatch,tmp_path,boundary_index,clock_kind):
    from test_station_compute_work_handoff import next_row
    from test_station_compute_wait_remote import clock
    from alicia_grasp_modes import station_compute_wait_remote as implementation
    env=prepared_publication_crossing(station,monkeypatch,tmp_path,boundary_index)
    try:
        wait_prepared_publication(env)
        old=env.crossed[0];parent=old['contract'];node=env.node
        end=min(parent['phase_deadline_ns'],parent['session_deadline_ns'])
        phase_mono=old['identity_monotonic']+(end-parent['identity_deadline_ns'])/1e9
        # Real source callbacks keep the same station qualified until just
        # before the original phase ends. No initial frame pins scheduling.
        assert node._stream_condition.acquire(timeout=.5)
        try:
            ns=parent['source_ns']+400_000_000
            while ns < end-900_000_000:
                row=next_row(station,ns,parent)
                parent=row['status']['station_compute_wait']
                ns+=400_000_000
            clock(station,end-10_000_000)
            with node._geometry_state_guard():node._station_compute_tick_locked()
            assert node._station_compute_pending and not node._station_compute_retired
            env.wait_entered.clear()
        finally:node._stream_condition.release()
        assert env.wait_entered.wait(1),env.results
        bounded_wait=env.wait_calls[-1]
        with node._stream_condition:
            if clock_kind=='ros':
                fixed_mono=implementation.time.monotonic()
                monkeypatch.setattr(implementation.time,'monotonic',lambda:fixed_mono)
                monkeypatch.setattr(implementation.rospy.Time,'now',staticmethod(
                    lambda:implementation.rospy.Time(end//10**9,end%10**9)))
            else:
                monkeypatch.setattr(implementation.time,'monotonic',lambda:phase_mono)
            node._stream_condition.notify_all()
        assert env.done.wait(1), 'fresh successors extended the original phase'
        assert len(env.results)==1 and isinstance(env.results[0],(ValueError,base.StreamResultCancelled)),env.results
        assert 0 < bounded_wait['timeout'] <= .010001
        assert_no_valid_station_publication(env)
        assert (tmp_path/'actual_atomic_preview_audit.json').read_bytes()==old['audit_bytes']
    finally:close_prepared_publication(env)


@pytest.mark.parametrize('boundary_index',[1,2])
def test_prepared_handoff_passes_first_frame_identity_deadline_with_continuous_fresh_sources(
        station,monkeypatch,tmp_path,boundary_index):
    from test_station_compute_work_handoff import next_row
    env=prepared_publication_crossing(station,monkeypatch,tmp_path,boundary_index)
    try:
        wait_prepared_publication(env)
        old=env.crossed[0];parent=old['contract'];node=env.node
        with node._stream_condition:
            ns=parent['source_ns']+400_000_000
            while ns <= old['contract']['identity_deadline_ns']+400_000_000:
                row=next_row(station,ns,parent)
                parent=row['status']['station_compute_wait']
                ns+=400_000_000
            env.events.append('extended_fresh_source_delivery')
            assert node._ros_source_clock_ns()>old['contract']['identity_deadline_ns']
            assert node._ros_source_clock_ns()<parent['geometry_deadline_ns']
        assert env.done.wait(3),env.results
        assert len(env.results)==1 and isinstance(env.results[0],dict),env.results
        assert env.results[0]['status']=='PREVIEW_READY'
        tail=env.events[env.events.index('extended_fresh_source_delivery')+1:]
        assert 'fresh_strict' in tail and tail.index('fresh_strict')<tail.index('legacy')
        assert (tmp_path/'actual_atomic_preview_audit.json').read_bytes()==old['audit_bytes']
        assert node._active_gate_audit_report[CONTACT_RECEIPT]==old['receipt']
        assert base.wire_digest(env.bundle['rich_plan'])==old['plan_wire']
        assert parent['phase_deadline_ns']==old['contract']['phase_deadline_ns']
        assert len(node.rich_plan_pub.messages)==env.execution_before
    finally:close_prepared_publication(env)


@pytest.mark.parametrize('boundary_index',[3,'commit_entry'])
@pytest.mark.parametrize('crossing',['geometry_ros','geometry_exact','geometry_mono'])
def test_final_prepared_commit_never_waits_or_refreshes_inside_publication(
        station,monkeypatch,tmp_path,boundary_index,crossing):
    env=prepared_publication_crossing(station,monkeypatch,tmp_path,boundary_index,crossing)
    try:
        assert env.done.wait(.5), 'final commit must fail closed without waiting for another source'
        assert len(env.crossed)==1
        assert len(env.results)==1 and isinstance(env.results[0],
            (base.StreamResultCancelled,ValueError)), env.results
        assert not env.wait_entered.is_set() and env.wait_calls==[]
        assert env.events.count('fresh_strict')==1
        assert_no_valid_station_publication(env)
        assert (tmp_path/'actual_atomic_preview_audit.json').read_bytes()==env.crossed[0]['audit_bytes']
    finally:
        close_prepared_publication(env)


@pytest.mark.parametrize('fault', ['cancel', 'deadline', 'reference', 'receipt', 'feedback'])
def test_change_during_atomic_audit_never_reaches_valid_plan(station, monkeypatch, tmp_path, fault):
    env = station_transaction(station, monkeypatch, tmp_path)
    def after_commit():
        if fault == 'cancel': station.params['/gui/joint_direct_mode'] = True
        elif fault == 'deadline': station.clock[0] = station.phase.deadline.to_sec() + .001
        elif fault == 'reference': env.node._latest_gate_audit_reference = {'atomic_committed': False}
        elif fault == 'receipt':
            env.node._active_gate_audit_report[CONTACT_RECEIPT]['ticket']['request_id'] += 1
        elif fault == 'feedback':
            # Below ordinary stationary tolerance: exact checked dependencies
            # still changed while the audit writer was finishing.
            env.node.latest_joint_state.position[0] += .0001
            feedback(station, env.node._ros_source_clock_ns(),
                     q=list(env.node.latest_joint_state.position))
            env.physical_changed = True
        else: raise AssertionError(fault)
    env.after_commit = after_commit
    with pytest.raises((base.CandidateContractError, base.StreamResultCancelled, ValueError)):
        env.run()
    assert 'audit_committed' in env.events
    assert_no_valid_station_publication(env)


@pytest.mark.parametrize('fault', ['receipt', 'disk', 'audit_reference_publish', 'legacy_publish'])
def test_audit_or_first_send_failure_cannot_leave_published_contact_authority(
        station, monkeypatch, tmp_path, fault):
    env = station_transaction(station, monkeypatch, tmp_path)
    def fail(*args, **kwargs):
        raise OSError('independent offline injected boundary failure')
    if fault == 'receipt': monkeypatch.setattr(env.node, '_station_contact_computation_receipt', fail)
    elif fault == 'disk': monkeypatch.setattr(env.node, '_write_gate_audit_report', fail)
    elif fault == 'audit_reference_publish': monkeypatch.setattr(env.node, '_publish_bounded_gate_audit', fail)
    else: monkeypatch.setattr(env.node.preview_plan_pub, 'publish', fail)
    with pytest.raises(OSError):
        env.run()
    assert_no_valid_station_publication(env)


def test_cancellation_after_legacy_send_prevents_rich_contact_plan(station, monkeypatch, tmp_path):
    env = station_transaction(station, monkeypatch, tmp_path)
    publish = env.node.preview_plan_pub.publish
    def cancel_after_legacy(message):
        result = publish(message)
        station.params['/gui/joint_direct_mode'] = True
        return result
    monkeypatch.setattr(env.node.preview_plan_pub, 'publish', cancel_after_legacy)
    with pytest.raises(base.StreamResultCancelled):
        env.run()
    assert 'legacy' in env.events and 'rich' not in env.events
    assert_no_valid_station_publication(env)


def test_promotable_transaction_observes_controller_once_when_execution_audit_rejects(
        station, monkeypatch, tmp_path):
    env = station_transaction(station, monkeypatch, tmp_path)
    controller = env.node._promotion_controller()
    observed = []
    def promotable(*args, **kwargs):
        observed.append((args, kwargs))
        return base.PromotionDecision(True, 'INDEPENDENT_BRANCH_PROVIDER',
                                      'synthetic promotion branch; original execution audit stays active')
    monkeypatch.setattr(controller, 'observe_preview', promotable)
    # This minimal station receipt fixture deliberately lacks execution audit
    # lineage. Its original strict execution schema must still refuse authority.
    with pytest.raises(base.CandidateContractError, match='audit'):
        env.run()
    assert len(observed) == 1
    assert 'audit_committed' in env.events
    assert sum(bool(m.valid) for m in env.node.rich_plan_pub.messages) == env.valid_execution_before


def contact_prepare_request(c, plan):
    node = c.node
    measured = node._measurement_remote_request
    now = node._ros_source_clock_ns()
    return dict(version=1, state='prepare', request_id='independent-contact-prepare',
        selection=deepcopy(node._mode_selection), session_id=measured['session_id'],
        target_track_id=measured['target_track_id'], target_epoch=measured['target_epoch'],
        control_epoch_ns=measured['control_epoch_ns'], measurement_sha256=measurement_digest(measured),
        plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan),
        plan_source_ns=plan.header.stamp.to_nsec(), phase_id=c.phase.phase_id,
        phase_started_ns=c.phase.header.stamp.to_nsec(), stage='MOVE_PREGRASP',
        stamp_ns=now, deadline_ns=now + 800_000_000, metric_geometry_authority=False,
        motion_authorized=False, controller_motion_confirmed=False)


def concurrent_callback_inside_rich_send(env, monkeypatch, callback):
    """Attempt a real callback after transport delivery but before send returns.

    A corrected implementation may serialize the callback on a commit lock;
    the publisher never waits for callback completion as a requirement. The
    short bounded wait only gives the formerly unlocked race a deterministic
    chance to run. All spawned threads are joined by the caller.
    """
    started, done = threading.Event(), threading.Event()
    errors, threads = [], []
    real_publish = env.node.preview_rich_plan_pub.publish
    def invoke():
        started.set()
        try:
            callback()
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()
    def publish(message):
        result = real_publish(message)
        if getattr(message, 'valid', False):
            assert not threads
            worker = threading.Thread(target=invoke, daemon=True)
            threads.append(worker)
            worker.start()
            assert started.wait(1.)
            env.events.append('callback_attempted_inside_rich_send')
            done.wait(.1)
        return result
    monkeypatch.setattr(env.node.preview_rich_plan_pub, 'publish', publish)
    def join():
        assert started.is_set() and done.wait(2.), 'callback did not complete after publication lock release'
        for worker in threads:
            worker.join(timeout=1.)
            assert not worker.is_alive()
        assert not errors, errors
    return join


def test_move_and_prepare_callback_inside_rich_send_sees_committed_published_fact(
        station, monkeypatch, tmp_path):
    env = station_transaction(station, monkeypatch, tmp_path)
    node = env.node
    node._contact_station_ack_pub = Publisher()
    request = contact_prepare_request(station, env.bundle['rich_plan'])
    def immediate_consumer():
        node.grasp_state_cb(NS(state='MOVE_PREGRASP', active=True))
        node._contact_station_request_cb(String(data=json.dumps(request)))
    join = concurrent_callback_inside_rich_send(env, monkeypatch, immediate_consumer)
    try:
        result = env.run()
    finally:
        join()
    response = json.loads(node._contact_station_ack_pub.messages[-1].data)
    assert response['accepted'] is True, response
    assert result['status'] == 'PREVIEW_READY'
    assert node._contact_station_prepared == request
    assert node._station_compute_retired is False
    assert sum(bool(m.valid) for m in node.rich_plan_pub.messages) == env.valid_execution_before


def test_real_cancel_callback_inside_rich_send_still_revokes_station_work(
        station, monkeypatch, tmp_path):
    env = station_transaction(station, monkeypatch, tmp_path)
    request = contact_prepare_request(station, env.bundle['rich_plan'])
    env.node._contact_station_ack_pub = Publisher()
    join = concurrent_callback_inside_rich_send(env, monkeypatch,
        lambda: env.node.grasp_state_cb(NS(state='CANCELLED', active=False)))
    try:
        try:
            env.run()
        except base.StreamResultCancelled:
            pass  # Cancellation can win either side of the completed send.
    finally:
        join()
    assert env.node.robot_execution_active is False
    assert env.node.streaming_enabled is False
    # A historical send fact may remain, but cancellation must revoke every
    # consumer capability. Do not require erasing a real publication receipt.
    assert not env.node._station_compute_work_certificate(env.ticket)
    assert env.node._station_contact_audited_proposals(request) == []
    env.node._contact_station_request_cb(String(data=json.dumps(request)))
    response = json.loads(env.node._contact_station_ack_pub.messages[-1].data)
    assert response['accepted'] is False, response
    assert sum(bool(m.valid) for m in env.node.rich_plan_pub.messages) == env.valid_execution_before


@pytest.mark.parametrize('body', ['left_finger', 'right_finger'])
@pytest.mark.parametrize('opening', [0., .023, .0498])
def test_real_complete_CAD_surface_intersection_still_rejects(body, opening):
    model = load_finger_cad_model(XML)
    assert model is not None
    gripper = contact_precheck_gripper({})
    transform = np.eye(4)
    angle = .37
    transform[:3, :3] = [[np.cos(angle), -np.sin(angle), 0.],
                         [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]]
    transform[:3, 3] = [.13, -.21, .4]
    part = model._body_components(body)[0]
    vertices = part.vertices_at_zero + opening * part.velocity_per_opening
    vertex = vertices[len(vertices)//3]
    scene = NS(center=transform[:3, :3] @ vertex + transform[:3, 3],
               rotation=transform[:3, :3].copy(), size=np.full(3, .0004))
    # An STL surface vertex is strictly inside this target OBB. It cannot
    # receive a sufficient-separation certificate at any tested opening.
    assert np.all(np.abs(scene.rotation.T @ (
        transform[:3, :3] @ vertex + transform[:3, 3] - scene.center)) < scene.size/2.)
    boxes = {name: (center, size) for name, center, size in
             _stage_boxes(transform, gripper, opening, 'y', 'z')}
    center, size = boxes[body]
    bound_fk = NS(finger_cad_model=model,
        model_sha256=model.provenance['robot_description_sha256'],
        base_frame='base_link', tool_frame='tool0')
    rejected, proof = geometry.gripper_target_overlap(
        bound_fk, gripper, opening, body, transform,
        center, size, scene)
    assert rejected is True
    assert proof is not None and proof['separated'] is False
    assert proof['provenance']['complete_mesh_vertices_used'] is True
    assert proof['padding_each_tool_axis_m'] == .0006
    assert proof['execution_authorized'] is False


@pytest.mark.parametrize('body', ['left_finger', 'right_finger'])
def test_missing_CAD_model_retains_conservative_overlap_rejection(body):
    gripper = contact_precheck_gripper({})
    transform = np.eye(4)
    boxes = {name: (center, size) for name, center, size in
             _stage_boxes(transform, gripper, .031, 'y', 'z')}
    center, size = boxes[body]
    scene = NS(center=np.array(center, copy=True), rotation=np.eye(3), size=np.full(3, .002))
    rejected, proof = geometry.gripper_target_overlap(
        NS(finger_cad_model=None), gripper, .031, body, transform,
        center, size, scene)
    assert rejected is True and proof is None
