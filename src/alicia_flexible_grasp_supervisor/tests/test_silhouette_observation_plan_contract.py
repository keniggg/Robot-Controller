"""One-view plans cannot acquire contact semantics or a gripper command."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from alicia_flexible_grasp.grasp import rich_plan_integrity as integrity
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, ObservationPathError
from test_rich_plan_candidate_source import neutral_refinement_ros_plan
from test_task_modes import base
from alicia_grasp_modes.runner_contract import audit_matches_preview


def observation_plan():
    plan = neutral_refinement_ros_plan()
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.candidate_source = 'silhouette_observation'
    plan.candidate_source_lineage = ['silhouette_observation']
    plan.has_candidate_model_width = False
    plan.candidate_width_m = plan.required_open_width_m = 0.
    plan.poses = plan.poses[:1]
    plan.plan_id = integrity.compute_plan_id(plan)
    return plan


def test_single_view_wire_roundtrip_preserves_id_and_full_scene():
    import io
    plan = observation_plan()
    stream = io.BytesIO()
    plan.serialize(stream)
    restored = type(plan)().deserialize(stream.getvalue())
    assert integrity.plan_id_matches_content(restored)
    assert len(restored.poses) == 1
    scene = FrozenObservationScene.from_plan(restored)
    assert list(scene.size) == [.20, .10, .08]
    result = base.validate_execution_plan(restored, 123.3, 2.)
    assert result.ok, (result.code, result.reason)
    assert base.observation_plan_pose(restored).pose == restored.poses[0]
    with pytest.raises(ValueError):
        base.split_rich_plan_poses(restored)
    with pytest.raises(ObservationPathError):
        FrozenObservationScene.from_contact_plan(restored)


@pytest.mark.parametrize('changes', [
    {'diagnostic':'CONTACT_EXECUTION_PLAN'}, {'model_choice':'carton'},
    {'required_open_width_m': .042}, {'required_open_width_m': float('nan')},
    {'required_open_width_m': float('inf')}, {'candidate_width_m': .042},
    {'has_candidate_model_width':True}, {'contact_geometry_evidence':'{}'},
    {'refinement_status':'VALID_3D'},
    {'candidate_source_lineage':['graspnet','silhouette_observation']},
    {'candidate_source_lineage':['silhouette_observation','tabletop_geometry']},
    {'poses':[]},
])
def test_observation_cannot_be_relabelled_as_contact_or_claim_width(changes):
    plan = observation_plan()
    for name,value in changes.items(): setattr(plan,name,value)
    with pytest.raises(ValueError): integrity.compute_plan_id(plan)
    assert not base.validate_execution_plan(plan,123.3,2.).ok


def test_extra_contact_poses_are_rejected():
    plan = observation_plan()
    plan.poses *= 4
    with pytest.raises(ValueError): integrity.compute_plan_id(plan)
    assert not base.validate_execution_plan(plan,123.3,2.).ok


def test_generic_grasp_executor_rejects_before_any_action():
    events=[]
    node=SimpleNamespace(set_state=lambda *args:events.append(args))
    def forbidden(*args,**kwargs): pytest.fail('observation invoked grasp hardware')
    assert not base.GraspTaskNode._execute_grasp6d_plan(node,{}, {},0.,
        forbidden,forbidden,forbidden,forbidden,observation_plan(),strict_execute_pose=forbidden)
    assert len(events)==1 and 'DIAGNOSTIC_OBSERVATION_EXECUTOR_REQUIRED' in events[0][1]


def bound_audit(plan):
    pose=plan.poses[0]
    return dict(snapshot_stamp_ns=plan.header.stamp.to_nsec(),plan_id=plan.plan_id,
        observe_return=dict(diagnostic_observation_only=True,
                            trigger_code='RGB_DEPTH_SILHOUETTE_AMBIGUOUS'),
        selected=dict(candidate_source='silhouette_observation',source_lineage=['silhouette_observation'],
            candidate_role='diagnostic_observation',contact_geometry_checked=False,required_open_width_m=0.,
            execution_sequence=dict(stages=[dict(stage='observation',
            frame_id=plan.header.frame_id,
            position_m=[getattr(pose.position,k) for k in 'xyz'],
            quaternion_xyzw=[getattr(pose.orientation,k) for k in 'xyzw'])])))


def test_single_observation_audit_binding_rejects_contact_or_other_trigger():
    plan=observation_plan(); report=bound_audit(plan)
    assert audit_matches_preview(report,plan,False)
    assert not audit_matches_preview(report,plan,True)
    for metadata in ({},dict(diagnostic_observation_only=False,trigger_code='RGB_DEPTH_SILHOUETTE_AMBIGUOUS'),
                     dict(diagnostic_observation_only=True,trigger_code='MISSING_SIDE')):
        bad=deepcopy(report);bad['observe_return']=metadata
        assert not audit_matches_preview(bad,plan,False)
    bad=deepcopy(report);bad['selected']['execution_sequence']['stages'][0]['position_m'][0]+=.001
    assert not audit_matches_preview(bad,plan,False)


def test_normal_contact_source_still_requires_positive_physical_opening():
    plan=neutral_refinement_ros_plan()
    for width in (0.,.051):
        plan.required_open_width_m=width
        plan.plan_id=integrity.compute_plan_id(plan)
        result=base.validate_execution_plan(plan,123.3,2.)
        assert not result.ok and result.code=='GRIPPER_TOO_NARROW'


def test_observation_cannot_bootstrap_actuation_or_return_to_origin():
    from alicia_flexible_grasp.robot.observe_return_gateway import ObserveReturnGateway
    plan=observation_plan(); events=[]
    node=SimpleNamespace(set_state=lambda *args:events.append(args))
    assert not base.GraspTaskNode._bootstrap_bound_actuation(node,plan,{})
    assert 'DIAGNOSTIC_OBSERVATION_REQUIRES_CONFIRMED_ACTUATION' in events[0][1]
    with pytest.raises(ObservationPathError,match='DIAGNOSTIC_OBSERVATION_RETURN_FORBIDDEN'):
        ObserveReturnGateway._validate_observe_return_context(object(),{'plan':plan},returning=True)


@pytest.mark.parametrize('field,value',[
    ('candidate_source','tabletop_geometry'),('source_lineage',['graspnet']),
    ('candidate_role','contact'),('contact_geometry_checked',True),('required_open_width_m',.042)])
def test_observation_preview_requires_matching_noncontact_source_audit(field,value):
    plan=observation_plan(); report=bound_audit(plan)
    report['selected'][field]=value
    assert not audit_matches_preview(report,plan,False)


def test_real_runner_receives_single_pose_then_accepts_same_source_authority(monkeypatch):
    from test_mode_runner import module
    plan=observation_plan()
    runner=module.ModeAwareGraspRunner('unknown','observe_return')
    runner.selection=dict(mode='unknown',strategy='observe_return',generation=20,stamp_ns=122000000000)
    runner.minimum_stamp_ns=123000000000; runner.plan_validity_sec=5.
    runner.expected_model='unknown_tabletop'
    monkeypatch.setattr(runner,'selection_current',lambda:True)
    monkeypatch.setattr(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time(124)))
    runner.on_preview(plan)
    assert runner.preview.plan_id==plan.plan_id
    assert runner._authority_preview_matches_attempt(plan,plan)
    assert not runner._authority_preview_matches_attempt(neutral_refinement_ros_plan(),plan)


@pytest.mark.parametrize('strategy',['direct','two_stage'])
def test_real_runner_rejects_observation_only_source_outside_diagnostic_strategy(monkeypatch,strategy):
    from test_mode_runner import module
    plan=observation_plan()
    runner=module.ModeAwareGraspRunner('unknown',strategy)
    runner.selection=dict(mode='unknown',strategy=strategy,generation=20,stamp_ns=122000000000)
    runner.minimum_stamp_ns=123000000000;runner.plan_validity_sec=5.;runner.expected_model='unknown_tabletop'
    monkeypatch.setattr(runner,'selection_current',lambda:True)
    monkeypatch.setattr(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time(124)))
    runner.on_preview(plan)
    assert runner.preview is None
    assert not runner._authority_preview_matches_attempt(plan,plan)


def test_gui_describes_the_single_view_without_claiming_grasp():
    from gui.widgets.grasp6d_control_widget import validate_enriched_plan
    plan=observation_plan()
    result=validate_enriched_plan(plan,now_sec=123.3,validity_sec=2.)
    assert result.fresh and '补充观察（不闭爪）' in result.text
    plan.required_open_width_m=.042
    assert not validate_enriched_plan(plan,now_sec=123.3,validity_sec=2.).fresh
