"""Task/gateway supplemental transactions using real ROS plan serialization.

Only the physical motion and feedback providers are inert. The admission
handler, session checks, frozen scenes, and cross-component digests are real.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose

from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.observation_path_guard import (
    CommittedObservationContexts, committed_plan_digest, wire_digest)
from alicia_grasp_modes import observe_return_task as task_module
import test_grasp_task_sequence as sequence_fixture
from test_observe_return_gateway import fixture as gateway_fixture
from test_observe_return_task import context, at_ns, audit_reference, module
from test_observe_return_supplement_task import prepare, supplement, install, invoke


def real_plan(source, report, seq):
    """Populate the fixture's audit with a serialized, content-bound rich plan."""
    plan = sequence_fixture.GraspTaskSequenceTest()._rich_plan(stamp_sec=source.header.stamp.to_sec())
    for key in ('valid', 'diagnostic', 'model_choice', 'required_open_width_m',
                'target_track_id', 'candidate_source', 'fused_view_count'):
        setattr(plan, key, deepcopy(getattr(source, key)))
    plan.candidate_source_lineage = [plan.candidate_source]
    plan.has_candidate_model_width = False
    plan.candidate_width_m = 0.
    plan.refinement_status = "VALID_3D"
    plan.refinement_inlier_count = 300
    plan.refinement_overlap_fraction = .9
    plan.refinement_rmse_m = .001
    plan.header.stamp = deepcopy(source.header.stamp)
    plan.header.frame_id = source.header.frame_id
    plan.header.seq = seq
    plan.poses = []
    for original in source.poses:
        pose = Pose()
        for key in ('x', 'y', 'z'):
            setattr(pose.position, key, getattr(original.position, key))
        for key in ('x', 'y', 'z', 'w'):
            setattr(pose.orientation, key, getattr(original.orientation, key))
        plan.poses.append(pose)
    plan.object_geometry.header.stamp = deepcopy(plan.header.stamp)
    plan.object_geometry.target_track_id = plan.target_track_id
    plan.plan_id = compute_plan_id(plan)
    report['plan_id'] = plan.plan_id
    report['selected']['final_strict_plan_id'] = plan.plan_id
    if report['observe_return'].get('phase') == 'supplemental_observation':
        report['observe_return']['view_plan_id'] = plan.plan_id
    return plan


def connected(c, monkeypatch):
    c.plan = real_plan(c.plan, c.report, 21)
    c.node._observe_return_reports.clear()
    c.node._observe_return_audit_cb(audit_reference(c.tmp_path, c.report))
    gateway, _, _, state, params, physical = gateway_fixture(monkeypatch)
    params[module.SELECTION_PARAM] = deepcopy(c.selection)
    c.params.update(params)
    monkeypatch.setattr(module.rospy, 'get_param', lambda key, default=None: c.params.get(key, default))
    monkeypatch.setattr(module.rospy, 'get_time', lambda: c.now[0] / 1e9)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: at_ns(c.now[0])))
    gateway._observation_contexts = CommittedObservationContexts()
    gateway._observation_contexts.set_active(True)
    gateway._observation_contexts.ingest(c.plan)
    gateway._control_reference_epoch_ns = 1_000_000_000
    gateway._observation_task_state_stamp_ns = c.now[0]
    gateway._manual_control_active = lambda: c.params.get('/gui/joint_direct_mode', False)
    old_state = c.node.set_state
    def set_state(stage, text='', *args, **kwargs):
        old_state(stage, text, *args, **kwargs)
        gateway._pregrasp_task_stage = stage
        gateway._compensation_task_label = text
    c.node.set_state = set_state
    original_publish = c.node._observe_return_pub.publish
    def publish(message):
        original_publish(message)  # Real session parser and task ack binding.
        gateway._observe_return_view_session_cb(message)
    c.node._observe_return_pub.publish = publish
    responses = []
    def proxy(name, _kind):
        def call(plan, execute):
            state['now'] = c.now[0] / 1e9
            c.gateways.append((name, plan.plan_id, execute))
            request = NS(plan=plan, execute=execute)
            if name == '/supervisor/capture_observe_return_origin':
                response = gateway.handle_capture_observe_return_origin(request)
                c.proof = json.loads(response.message)
            else:
                assert name == '/supervisor/admit_observe_return_view'
                response = gateway.handle_admit_observe_return_view(request)
            responses.append(response)
            return response
        return call
    monkeypatch.setattr(module.rospy, 'ServiceProxy', proxy)
    calls, bound = prepare(c, monkeypatch)
    # prepare supplies inert execution interfaces. Restore the actual service
    # bridge and digest implementation rather than its lightweight stand-ins.
    monkeypatch.setattr(module.rospy, 'ServiceProxy', proxy)
    monkeypatch.setattr(task_module, 'committed_plan_digest', committed_plan_digest)
    return NS(gateway=gateway, state=state, physical=physical, calls=calls,
              bound=bound, responses=responses)


@pytest.mark.parametrize('seq', [1, 87, 2**32 - 1])
def test_real_admission_accepts_transport_sequence_without_changing_scene_or_origin(context, monkeypatch, seq):
    c = context
    link = connected(c, monkeypatch)
    original = link.gateway._observe_return_origin
    origin_goal = original['goal_digest']
    origin_deadline = original['deadline']
    c.now[0] += 2_000_000_000
    plan, report = supplement(c, 2)
    plan = real_plan(plan, report, seq)
    assert wire_digest(plan) != committed_plan_digest(plan)
    install(c, plan, report)
    result = invoke(c, c.plan)
    assert result.get('retry') is True, c.states
    assert [kind for kind, _ in link.calls] == ['freeze', 'move']
    proof = json.loads(link.responses[-1].message)
    assert proof['plan_digest'] == committed_plan_digest(plan)
    assert proof['physical_motion_submitted'] is False
    assert proof['origin_plan_digest'] == committed_plan_digest(c.plan)
    assert original['scene'].plan_id == c.plan.plan_id
    assert original['goal_digest'] == origin_goal and original['deadline'] == origin_deadline
    assert original['active_view_scene'].plan_id == result['current_plan'].plan_id
    assert c.node._observe_return_session['view_index'] == 2
    assert not link.physical and link.state['moves'] == 0


@pytest.mark.parametrize('changed', ['source', 'nested_sequence', 'geometry', 'pose'])
def test_real_admission_digest_cannot_lend_proof_to_changed_plan(context, monkeypatch, changed):
    c = context
    link = connected(c, monkeypatch)
    c.now[0] += 2_000_000_000
    plan, report = supplement(c, 2)
    plan = real_plan(plan, report, 87)
    install(c, plan, report)
    original_proxy = module.rospy.ServiceProxy
    def proxy(name, kind):
        handler = original_proxy(name, kind)
        def call(candidate, execute):
            response = handler(candidate, execute)
            assert response.success, response.message
            # Simulate payload corruption after gateway admission but before
            # task verifies the response; only outer transport seq is exempt.
            if changed == 'source':
                candidate.header.stamp = at_ns(candidate.header.stamp.to_nsec() + 1)
            elif changed == 'nested_sequence':
                candidate.object_geometry.header.seq += 1
            elif changed == 'geometry':
                candidate.object_geometry.size_xyz_m.x += .001
            else:
                candidate.poses[0].position.x += .001
            return response
        return call
    monkeypatch.setattr(module.rospy, 'ServiceProxy', proxy)
    assert invoke(c, c.plan) == {'failed': True}
    assert 'OBSERVE_RETURN_VIEW_ADMISSION_PROOF_INVALID' in c.states[-1][1]
    assert not link.calls and not link.physical and link.state['moves'] == 0
    assert c.node._observe_return_session['view_index'] == 1
