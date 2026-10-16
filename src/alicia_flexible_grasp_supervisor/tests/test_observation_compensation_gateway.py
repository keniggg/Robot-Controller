"""Real session/receipt callbacks and service, with simulated planning/hardware."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotState
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.robot import observation_compensation_gateway as module
from alicia_flexible_grasp.robot.observation_path_guard import CommittedObservationContexts,SerialUrdfFk
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from test_observe_return_gateway import fixture,capture
from test_observe_return_view_admission import initial_session,deliver
from test_observation_endpoint_compensation import view_inputs
from test_pregrasp_feedforward import case


def gateway_case(monkeypatch,moving=False,deliver_receipt=True,epoch_ns=None):
    n,p,plan,s,params,calls=fixture(monkeypatch)
    if epoch_ns is not None:
        n._control_reference_epoch_ns=epoch_ns
        s['sample_mutator']=lambda sample,after:sample.update(epoch_ns=epoch_ns)
        # Keep all source/session times after the supplied real control epoch.
        shift_sec=epoch_ns//1_000_000_000
        s['now']+=shift_sec
        plan.header.stamp=rospy.Time(plan.header.stamp.secs+shift_sec,plan.header.stamp.nsecs)
        plan.object_geometry.header.stamp=deepcopy(plan.header.stamp)
        n._observation_task_state_stamp_ns+=shift_sec*1_000_000_000
    # The root script wiring supplies the mixin in production; exercise its
    # actual methods on the existing actual gateway fixture here.
    if not isinstance(n,module.ObservationCompensationGateway):
        n.__class__=type('Gateway',(module.ObservationCompensationGateway,type(n)),{})
    params['/grasp']['observation_endpoint_compensation_enabled']=True
    base,nominal,sdk,actual=case()
    if moving:
        s['sdk']=sdk.copy();s['actual']=actual.copy()
        params['/robot_description']=base.contract.fk.xml if hasattr(base.contract.fk,'xml') else json.loads(
            (__import__('pathlib').Path(__file__).parent/'fixtures/pregrasp_following_20260925_actual.json').read_text())['robot_description']
        plan.poses[0]=deepcopy(base.goal)
    else:
        fk=SerialUrdfFk(params['/robot_description'],ARM_NAMES);tool=fk(s['actual'])
        plan.poses[0].position.x,plan.poses[0].position.y,plan.poses[0].position.z=tool[:3,3]
        q=quaternion_from_matrix(tool)
        plan.poses[0].orientation.x,plan.poses[0].orientation.y,plan.poses[0].orientation.z,plan.poses[0].orientation.w=q
    scene,config,valid=view_inputs(plan.poses[0])
    g=plan.object_geometry
    g.pose_base.position.x,g.pose_base.position.y,g.pose_base.position.z=scene.center
    q=quaternion_from_matrix(np.block([[scene.rotation,np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]))
    g.pose_base.orientation.x,g.pose_base.orientation.y,g.pose_base.orientation.z,g.pose_base.orientation.w=q
    g.size_xyz_m.x,g.size_xyz_m.y,g.size_xyz_m.z=scene.size
    plan.plan_id=compute_plan_id(plan)
    n._observation_contexts=CommittedObservationContexts();n._observation_contexts.set_active(True);n._observation_contexts.ingest(plan)
    assert capture(n,plan).success
    initial=initial_session(n,plan,s)
    if epoch_ns is not None:
        initial['deadline_ns']=str((190+shift_sec)*1_000_000_000)
    deliver(n,initial);session=n._observe_return_origin['view_session']
    receipt=dict(version=1,state='executing',selection=deepcopy(session['selection']),
        session_id=session['session_id'],target_epoch=session['target_epoch'],target_track_id=session['target_track_id'],
        origin_plan_id=plan.plan_id,origin_snapshot_stamp_ns=plan.header.stamp.to_nsec(),
        view_plan_id=plan.plan_id,view_source_stamp_ns=plan.header.stamp.to_nsec(),view_index=1,
        plan_sha256=n._observe_return_origin['scene'].digest,control_epoch_ns=n._control_reference_epoch_ns,
        deadline_ns=session['deadline_ns'],stamp_ns=rospy.Time.from_sec(s['now']).to_nsec(),lease_id='actual_fixture_lease',
        request_id='actual_fixture_lease:executing',motion_kind='observation',arrival_stamp_ns=0,
        capture_deadline_ns=0,metric_geometry_authority=False,motion_authorized=False)
    n._compensation_task_label=module.OBSERVATION_COMPENSATION_LABEL
    if deliver_receipt:
        n._observation_compensation_measurement_cb(NS(data=json.dumps(receipt)))
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(s['now'])))
    monkeypatch.setattr(rospy,'sleep',lambda t:s.update(now=s['now']+t))
    config.update(camera_configuration_snapshot={},camera_configuration_contract=_camera_configuration_contract({}),source_stamp_ns=plan.header.stamp.to_nsec())
    n._unknown_observation_view_configuration=lambda scene:deepcopy(config)
    if moving:
        # Isolate correction orchestration; projection and feedforward are
        # separately tested with real geometry. No fake success H/ROS calls.
        monkeypatch.setattr(module.ObservationEndpointCompensation,'current_view',lambda self,sample:None)
        state=RobotState();state.joint_state.name=list(ARM_NAMES)+['right_finger','left_finger']
        state.joint_state.position=s['actual'].tolist()+[.04975,-.04975]
        p.robot=NS(get_current_state=lambda:deepcopy(state))
        p._plan_cartesian_from_start_state=mock.Mock(return_value=(nominal,'nominal'))
        n._contact_command_collision_check=mock.Mock(return_value={'fixture':'collision_checker'})
        monkeypatch.setattr(module,'validate_selected_pregrasp_path',mock.Mock(return_value={'fixture':'dual_curve_checker'}))
        bias=sdk-actual;s['response']=lambda goal:goal-bias
    return n,p,plan,s,params,calls,receipt,config


@pytest.mark.parametrize('execute',[False,True])
def test_actual_callback_admits_same_plan_no_motion_and_no_controller_calls(monkeypatch,execute):
    n,p,plan,s,params,calls,receipt,cfg=gateway_case(monkeypatch)
    result=n.handle_compensate_observation(NS(plan=plan,execute=execute));d=json.loads(result.message)
    assert result.success,d
    assert d['code']=='OBSERVATION_ENDPOINT_READY' and d['no_motion']
    assert not d['physical_motion_submitted'] and not d['complete_execution_path_checked']
    assert d['lease_id']==receipt['lease_id'] and d['fixed_goal_pose'][0]==plan.poses[0].position.x
    assert not calls and s['moves']==0
    n._ensure_trajectory_controllers_started.assert_not_called()


@pytest.mark.parametrize('fault',['digest','epoch','session','source','selection','qualified','contact','disabled','manual','expired','rgb_hole'])
def test_invalid_authority_or_geometry_cannot_borrow_contact_service(monkeypatch,fault):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch)
    if fault=='digest':r['plan_sha256']='f'*64
    if fault=='epoch':r['control_epoch_ns']+=1
    if fault=='session':r['session_id']='other'
    if fault=='source':r['view_source_stamp_ns']+=1
    if fault=='selection':r['selection']['generation']+=1
    if fault=='qualified':r['state']='qualified'
    if fault=='contact':plan.diagnostic='CONTACT_EXECUTION_PLAN';plan.plan_id=compute_plan_id(plan)
    if fault=='disabled':params['/grasp']['observation_endpoint_compensation_enabled']=False
    if fault=='manual':n._manual_control_active=lambda:True
    if fault=='expired':s['now']=191.
    if fault=='rgb_hole':
        def invalid(*args):
            v=np.ones((480,640),dtype=bool);v[240,320]=False;return v
        monkeypatch.setattr(module,'rectification_rgb_valid_mask',invalid)
    n._observation_compensation_measurement_cb(NS(data=json.dumps(r)))
    result=n.handle_compensate_observation(NS(plan=plan,execute=True))
    assert not result.success,result.message
    assert not calls and s['moves']==0


def test_receipt_delivered_later_within_existing_short_admission_is_accepted(monkeypatch):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch,deliver_receipt=False)
    assert not getattr(n,'_observation_compensation_receipt_stamp_ns',0)
    def delayed(t):
        s['now']+=t;n._observation_compensation_measurement_cb(NS(data=json.dumps(r)))
    monkeypatch.setattr(rospy,'sleep',delayed)
    result=n.handle_compensate_observation(NS(plan=plan,execute=True))
    assert result.success,result.message
    assert 10.<s['now']<11. and not calls


def test_one_correction_uses_exact_goal_full_path_and_fresh_measured_response(monkeypatch):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch,True)
    result=n.handle_compensate_observation(NS(plan=plan,execute=True));d=json.loads(result.message)
    assert result.success,d
    assert s['moves']==1 and d['physical_motion_submitted'] and d['complete_execution_path_checked']
    assert d['accepted_stamp_ns']>d['completion_stamp_ns']
    assert d['response']['hardware_response_verified'] and d['view_audit']['rgb_identity_verified'] is False
    assert module.validate_selected_pregrasp_path.call_count==1
    n._validate_frozen_observation_path.assert_called_once()
    again=n.handle_compensate_observation(NS(plan=plan,execute=True))
    assert not again.success and 'CONSUMED' in again.message
    assert s['moves']==1


def test_full_path_collision_rejection_never_reaches_submission(monkeypatch):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch,True)
    module.validate_selected_pregrasp_path.side_effect=ValueError('recorded_test_full_curve_collision')
    result=n.handle_compensate_observation(NS(plan=plan,execute=True))
    report=json.loads(result.message)
    assert not result.success and 'recorded_test_full_curve_collision' in report['reason']
    module.validate_selected_pregrasp_path.assert_called_once()
    assert s['moves']==0 and not report['physical_motion_submitted']
    assert not any(k.startswith('/supervisor/observation_compensation_episodes') for k in params)


@pytest.mark.parametrize('fault',['manual','deadline','receipt','q','rgb_config'])
def test_changes_during_nominal_planning_never_submit(monkeypatch,fault):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch,True)
    original=p._plan_cartesian_from_start_state
    def changed(*args):
        result=original(*args)
        if fault=='manual':n._manual_control_active=lambda:True
        if fault=='deadline':s['now']=191.
        if fault=='receipt':r['state']='revoked';r['stamp_ns']+=1;n._observation_compensation_measurement_cb(NS(data=json.dumps(r)))
        if fault=='q':s['actual'][0]+=3*Q
        if fault=='rgb_config':cfg['camera']['fx']+=1.
        return result
    p._plan_cartesian_from_start_state=changed
    result=n.handle_compensate_observation(NS(plan=plan,execute=True))
    assert not result.success,result.message
    assert s['moves']==0 and not any(k.startswith('/supervisor/observation_compensation_episodes') for k in params)


def test_true_after_readonly_result_still_reads_current_owner(monkeypatch):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch)
    assert n.handle_compensate_observation(NS(plan=plan,execute=False)).success
    n._control_reference_epoch_ns+=1
    assert not n.handle_compensate_observation(NS(plan=plan,execute=True)).success
    assert not calls


@pytest.mark.parametrize('fault',['owner','deadline'])
def test_executor_late_success_is_not_accepted_after_original_authority_expires(monkeypatch,fault):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch,True)
    execute=p.plan_and_execute_joint_probe
    def late(*args,**kwargs):
        result=execute(*args,**kwargs)
        if fault=='owner':n._control_reference_epoch_ns+=1
        else:s['now']=191.
        return result
    p.plan_and_execute_joint_probe=late
    result=n.handle_compensate_observation(NS(plan=plan,execute=True))
    assert not result.success and s['moves']==1
    assert json.loads(result.message)['code']!='OBSERVATION_ENDPOINT_READY'


@pytest.mark.parametrize('fault',['revoked','malformed','same_stamp_changed'])
def test_late_executing_receipt_cannot_revive_revoked_or_invalidated_handoff(monkeypatch,fault):
    n,p,plan,s,params,calls,r,cfg=gateway_case(monkeypatch)
    closed=deepcopy(r)
    if fault=='revoked':closed.update(state='revoked',stamp_ns=r['stamp_ns']+1)
    if fault=='same_stamp_changed':closed['plan_sha256']='b'*64
    n._observation_compensation_measurement_cb(NS(data='bad' if fault=='malformed' else json.dumps(closed)))
    n._observation_compensation_measurement_cb(NS(data=json.dumps(r)))
    result=n.handle_compensate_observation(NS(plan=plan,execute=True))
    assert not result.success and not calls
