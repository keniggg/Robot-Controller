"""Bound final candidate selection; no ROS master or motion clients."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, asdict
from io import BytesIO
from types import SimpleNamespace as NS
import math

import pytest

from alicia_flexible_grasp.grasp.refinement_continuity import (
    binding_from_phase, validate_candidate_continuity, RefinementContinuityError,
)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.grasp.grasp6d_pipeline import bounded_moveit_select
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase
from test_final_refine_reference import setup_node
from test_task_contact_admission import contact_plan, task, fixtures
from test_registered_candidate_selection import selection_node
from test_grasp6d_pipeline import scored_candidate, reachable_moveit_result
from test_remote_grasp6d_streaming import streaming_node, remote_node


def phase_for(plan=None, stage='final_refinement'):
    plan = plan or contact_plan(10.)
    phase = NearFieldPlanningPhase()
    phase.active, phase.phase_id = True, 7
    phase.header.frame_id = plan.header.frame_id
    phase.header.stamp = task.rospy.Time.from_sec(20.)
    phase.deadline = task.rospy.Time.from_sec(40.)
    phase.reference_source_stamp = task.rospy.Time.from_sec(19.9)
    phase.reference_center_valid = True
    phase.reference_target_track_id = plan.target_track_id
    phase.reference_label = 'carton'
    phase.reference_contact_plan = deepcopy(plan)
    phase.planning_stage = stage
    return phase


def observed(roll=0., yaw=0., translation=0.):
    plan = contact_plan(20.1)
    # q_z(yaw) * q_x(roll), matching the independent task math.
    r, y = math.radians(roll)/2, math.radians(yaw)/2
    for pose in plan.poses:
        pose.position.x += translation
        for axis, value in zip('xyzw', (math.cos(y)*math.sin(r), math.sin(y)*math.sin(r),
                                        math.sin(y)*math.cos(r), math.cos(y)*math.cos(r))):
            setattr(pose.orientation, axis, value)
    return fixtures._mark_contact_geometry(plan)


def test_phase_transport_binds_exact_plan_and_freezes_reference():
    phase = phase_for()
    data = BytesIO(); phase.serialize(data)
    received = NearFieldPlanningPhase().deserialize(data.getvalue())
    echoed = BytesIO(); received.serialize(echoed)
    assert echoed.getvalue() == data.getvalue()
    assert received.planning_stage == phase.planning_stage
    assert compute_plan_id(received.reference_contact_plan) == phase.reference_contact_plan.plan_id
    bound = binding_from_phase(received, {})
    assert bound.plan_id == phase.reference_contact_plan.plan_id
    original = deepcopy(bound)
    phase.reference_contact_plan.poses[2].position.x += .1
    received.reference_contact_plan.poses[2].orientation.x = .5
    assert bound == original
    with pytest.raises(FrozenInstanceError): bound.phase_id = 8


@pytest.mark.parametrize('fault', ['hash', 'phase_id', 'phase_time', 'deadline', 'source_time',
    'reference_time', 'missing_reference', 'target', 'geometry_target', 'frame', 'invalid',
    'poses', 'diagnostic', 'normal', 'stage'])
def test_invalid_phase_cannot_supply_a_final_binding(fault):
    phase = phase_for()
    plan = phase.reference_contact_plan
    if fault == 'hash': plan.plan_id = 'corrupted'
    if fault == 'phase_id': phase.phase_id = 0
    if fault == 'phase_time': phase.header.stamp = task.rospy.Time.from_sec(19.)
    if fault == 'deadline': phase.deadline = phase.header.stamp
    if fault == 'source_time': plan.header.stamp = task.rospy.Time.from_sec(21.)
    if fault == 'reference_time': phase.reference_source_stamp = task.rospy.Time.from_sec(9.)
    if fault == 'missing_reference': phase.reference_center_valid = False
    if fault == 'target': phase.reference_target_track_id = 'g0-t2'
    if fault == 'geometry_target': plan.object_geometry.target_track_id = 'g0-t2'
    if fault == 'frame': phase.header.frame_id = 'camera_link'
    if fault == 'invalid': plan.valid = False
    if fault == 'poses': plan.poses = plan.poses[:3]
    if fault == 'diagnostic': plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    if fault == 'normal': plan.object_geometry.support_normal_base.z = 0.
    if fault == 'stage': phase.planning_stage = 'typo'
    with pytest.raises(RefinementContinuityError) as error:
        binding_from_phase(phase, {})
    assert error.value.code == 'FINAL_REFINE_PHASE_INVALID'


@pytest.mark.parametrize('key,value', [
    ('final_visual_refine_max_translation_m', float('nan')),
    ('final_visual_refine_max_yaw_deg', -1.),
    ('final_visual_refine_max_roll_pitch_change_deg', True),
])
def test_bad_limits_cannot_authorize_candidate_search(key, value):
    with pytest.raises(RefinementContinuityError): binding_from_phase(phase_for(), {key: value})


@pytest.mark.parametrize('roll,yaw,translation,code', [
    (0., 0., 0., None), (3.9, 9.9, .0249, None), (4.1, 0., 0., 'FINAL_REFINE_ROLL_PITCH_LIMIT'),
    (4.938309, .223496, .009514527, 'FINAL_REFINE_ROLL_PITCH_LIMIT'),
    (0., 10.1, 0., 'FINAL_REFINE_YAW_LIMIT'),
    (0., 0., .0251, 'FINAL_REFINE_TRANSLATION_LIMIT'),
])
def test_upstream_limits_match_task_and_never_modify_contact_evidence(roll, yaw, translation, code):
    phase = phase_for(); bound = binding_from_phase(phase, {})
    plan = observed(roll, yaw, translation); original = deepcopy(plan)
    task_result, refined, task_metrics = task.build_bounded_final_visual_refinement(
        phase.reference_contact_plan, plan, .025, math.radians(10.), math.radians(4.))
    if code:
        with pytest.raises(RefinementContinuityError) as error:
            validate_candidate_continuity(bound, plan, 7)
        assert error.value.code == task_result.code == code
        assert refined is None
    else:
        metrics = validate_candidate_continuity(bound, plan, 7)
        assert task_result.ok and refined == plan
        assert metrics['translation_m'] == pytest.approx(task_metrics['translation_m'])
        assert metrics['yaw_rad'] == pytest.approx(task_metrics['yaw_rad'])
        assert metrics['roll_pitch_rad'] == pytest.approx(task_metrics['observed_roll_pitch_change_rad'])
    assert plan == original


@pytest.mark.parametrize('fault,code', [('phase', 'FINAL_REFINE_PHASE_INVALID'),
    ('old', 'FINAL_REFINE_PHASE_INVALID'), ('late', 'FINAL_REFINE_PHASE_INVALID'),
    ('target', 'FINAL_REFINE_TARGET_CHANGED'), ('frame', 'FINAL_REFINE_TARGET_CHANGED')])
def test_candidate_cannot_cross_phase_or_target(fault, code):
    bound = binding_from_phase(phase_for(), {}); plan = observed()
    phase_id = 8 if fault == 'phase' else 7
    if fault == 'old': plan.header.stamp = task.rospy.Time.from_sec(19.9)
    if fault == 'late': plan.header.stamp = task.rospy.Time.from_sec(40.)
    if fault == 'target': plan.target_track_id = 'g0-t2'
    if fault == 'frame': plan.header.frame_id = 'camera_link'
    with pytest.raises(RefinementContinuityError) as error:
        validate_candidate_continuity(bound, plan, phase_id)
    assert error.value.code == code


def test_task_retains_ownership_of_complete_clear_view_source_rebind():
    phase = phase_for(); plan = observed()
    plan.candidate_source = 'tabletop_geometry'; plan.candidate_source_lineage = ['tabletop_geometry']
    plan.has_candidate_model_width = False; plan.candidate_width_m = 0.
    plan.plan_id = compute_plan_id(plan)
    metrics = validate_candidate_continuity(binding_from_phase(phase, {}), plan, 7)
    assert metrics['task_source_rebind_required'] is True
    result, refined, _ = task.build_bounded_final_visual_refinement(
        phase.reference_contact_plan, plan, .025, math.radians(10.), math.radians(4.))
    assert result.code == 'FINAL_REFINE_CANDIDATE_SWITCH' and refined is None


def test_ranked_search_skips_incompatible_family_and_keeps_next_measured_candidate():
    node = selection_node(); binding = binding_from_phase(phase_for(), {})
    prepared = NS(refinement_binding=binding, near_field_phase_id=7)
    plans = {1: observed(roll=4.938309), 2: observed(roll=1.)}
    checked=[]
    def build(candidate):
        checked.append(candidate.track_id)
        runtime = node._stable_variant_runtime[(candidate.track_id, 0)]
        node._check_refinement_candidate_continuity(plans[candidate.track_id], prepared, runtime)
        runtime['final_strict_moveit_result'] = asdict(reachable_moveit_result(.8, .4))
    node._build_selected_preview_bundle = build
    result = bounded_moveit_select([scored_candidate(i, i) for i in (1,2)],
        node._check_direct_registered_candidate, top_n=2, first_reachable_by_rank=True,
        ranking_key=lambda candidate:candidate.track_id)
    assert checked == [1,2] and result.selected.track_id == 2
    assert node._stable_variant_runtime[(1,0)]['final_contact_rejection']['code'] == 'FINAL_REFINE_ROLL_PITCH_LIMIT'
    assert node._stable_variant_runtime[(2,0)]['refinement_continuity']['reference_plan_id'] == binding.plan_id


def test_phase_deactivation_and_initial_contact_replan_clear_final_binding():
    node=streaming_node(start_worker=False)
    try:
        node.near_field_state_cb(phase_for())
        original=node._near_field_refinement_binding
        assert original is not None and node.near_field_planning_active
        next_phase=phase_for(stage='contact_replan');next_phase.phase_id += 1
        node.near_field_state_cb(next_phase)
        assert node._near_field_refinement_binding is None
        assert original.phase_id == 7
        node.near_field_state_cb(phase_for())
        off=phase_for();off.active=False
        node.near_field_state_cb(off)
        assert node._near_field_refinement_binding is None and not node.near_field_planning_active
    finally: node.shutdown_streaming_worker()


def test_task_final_phase_publishes_bound_contact_without_mutating_it(monkeypatch):
    node, _, clock = setup_node(monkeypatch, now=20.)
    plan=contact_plan(10.); original=deepcopy(plan)
    messages=[];node.near_field_phase_pub=NS(publish=messages.append)
    node._set_near_field_active(True,force=True,budget_sec=20.,reference_plan=plan,
                               planning_stage='final_refinement')
    assert len(messages)==1 and messages[0].reference_contact_plan==plan==original
    assert binding_from_phase(messages[0],{}).plan_id==plan.plan_id


@pytest.mark.parametrize('stage,active', [('contact_replan',True), ('',True), ('final_refinement',False)])
def test_unbound_stages_do_not_filter_original_planning(stage,active):
    phase=phase_for(stage=stage);phase.active=active
    assert binding_from_phase(phase,{}) is None
    assert validate_candidate_continuity(None,None,0) is None


@pytest.mark.parametrize('config', [
    {'final_visual_refine_registration': {'maximum_translation_m': .015, 'maximum_yaw_deg': 6.}},
    {'final_visual_refine_max_translation_m': .020, 'final_visual_refine_max_yaw_deg': 8.},
])
def test_candidate_search_uses_the_same_tightened_policy_as_task(config):
    binding=binding_from_phase(phase_for(), config)
    policy=task._final_refinement_registration_policy(config)
    assert binding.translation_limit_m == policy['maximum_translation_m']
    assert binding.yaw_limit_rad == math.radians(policy['maximum_yaw_deg'])


@pytest.mark.parametrize('degrees,size', [(0., (.020,.010,.012)),
    (37.,(.035,.022,.045)), (-62.,(.070,.018,.025)), (130.,(.048,.030,.060))])
@pytest.mark.parametrize('tilt,accepted', [(2.,True),(5.,False)])
def test_general_unknown_geometry_and_coordinate_rotations_preserve_limits(degrees,size,tilt,accepted):
    current,candidate=contact_plan(10.),observed(roll=tilt,yaw=3.,translation=.004)
    q=task._axis_angle_quaternion((1.,2.,3.),math.radians(degrees))
    # Same physical relative change in different target positions, orientations
    # and measured extents. No object name, fixed approach or height correction.
    for plan in (current,candidate):
        for pose in plan.poses:
            xyz=task._rotate_vector(q,tuple(getattr(pose.position,a) for a in 'xyz'))
            quat=task._quaternion_multiply(q,task._quaternion_xyzw(pose))
            for a,v in zip('xyz',xyz): setattr(pose.position,a,v)
            for a,v in zip('xyzw',quat): setattr(pose.orientation,a,v)
        normal=task._rotate_vector(q,(0.,0.,1.))
        for a,v in zip('xyz',normal): setattr(plan.object_geometry.support_normal_base,a,v)
        for a,v in zip('xyz',size): setattr(plan.object_geometry.size_xyz_m,a,v)
        fixtures._mark_contact_geometry(plan)
    phase=phase_for(current);binding=binding_from_phase(phase,{})
    if accepted:
        result=validate_candidate_continuity(binding,candidate,7)
        assert result['translation_m']==pytest.approx(.004)
        assert math.degrees(result['roll_pitch_rad'])==pytest.approx(tilt)
    else:
        with pytest.raises(RefinementContinuityError) as error:
            validate_candidate_continuity(binding,candidate,7)
        assert error.value.code=='FINAL_REFINE_ROLL_PITCH_LIMIT'
