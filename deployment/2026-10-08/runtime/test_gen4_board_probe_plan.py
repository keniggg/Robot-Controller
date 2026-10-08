"""Pure source/geometry/SDK-math regressions: no ROS node, master or hardware."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE/'realsense-2.56.5')]
import pyrealsense2 as rs
import gen4_board_probe_plan as board


def fixture():
    # Synthetic kinematic model supports all six real interface axes; no fixture
    # assumption is an unknown-object parameter or a runtime robot correction.
    xml = '<robot name="fixture"><link name="base_link"/>'
    for index in range(1, 7):
        parent = 'base_link' if index == 1 else 'L%d'%(index-1)
        child = 'tool0' if index == 6 else 'L%d'%index
        xml += ('<link name="%s"/><joint name="Joint%d" type="revolute">'
                '<parent link="%s"/><child link="%s"/><origin xyz="0 0 %s"/>'
                '<axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>'
                '</joint>')%(child,index,parent,child,'.35' if index==1 else '0')
    xml += '</robot>'
    q = np.r_[np.zeros(6), .04975]
    X = np.eye(4)
    X[:3,:3] = [[0,-1,0],[0,0,1],[-1,0,0]]
    X[:3,3] = [.09,.0675,0.]
    fk = board.base.SerialUrdfFk(xml,board.base.NAMES[:6])
    C = fk(q[:6])@X
    Y = np.eye(4)
    B = np.linalg.inv(C)@Y
    c = dict(width=640,height=480,fx=400.,fy=400.,cx=320.,cy=240.,
             model='inverse_brown_conrady',coeffs=[-.05,.04,.001,.001,-.01])
    evidence = dict(source='realsense_runtime',source_parameter='/camera',
        source_field='runtime_profile.color_intrinsics',color_intrinsics=c,
        camera_frame='camera_link',image_topic='/supervisor/camera/color/image_raw',
        serial_number='fixture_device',sdk_version='2.56.5',color_projection_correction=False,
        backend='realsense_sdk_rays_normalized_pnp',pixel_error_model='realsense_sdk_projection_original_pixels',
        image_time_association='parameter_snapshot_not_capture_metadata')
    model = board.make_runtime_model(evidence,sdk=rs)
    ns = 2_000_000_000
    binding = dict(rgb_stamp_ns=ns,depth_stamp_ns=ns,board_stamp_ns=ns,quality_stamp_ns=ns,
        accepted_before_ns=ns-40_000_000,accepted_after_ns=ns+60_000_000,
        accepted_source_stamp_ns=ns-40_000_000,sdk_stamp_ns=ns+60_000_000,
        reference_stamp_ns=ns+60_000_000,snapshot_received_ros_ns=ns+100_000_000,
        quality_received_ros_ns=ns+30_000_000,board_received_ros_ns=ns+30_000_000,
        accepted_window_span_deg=[0.]*6,accepted_window_duration_sec=.7)
    quality = dict(source_stamp_ns=ns,pose_ok=True,charuco_count=35,camera_frame='camera_link',
        board_frame='fixed_fixture_board',board_size=[12,9],dictionary_id='DICT_5X5_100',
        square_length_m=.015,marker_length_m=.01125,reprojection_rms_px=1.5,
        edge_clearance_px=20.,intrinsics_source='realsense_runtime',camera_model_change_latched=False,
        camera_model_error=None,camera_model_fingerprint=model.fingerprint,camera_model=deepcopy(evidence),
        image_width=640,image_height=480)
    scene = dict(kind='fixed_rigid_board_laid_flat_on_same_local_tabletop',
        board_printed_plane_is_local_table_upper_bound=True,evidence='fixture explicitly declared flat fixed local tabletop',
        old_board_thickness_reused=False,scope='small bounded board-only diagnostic, not grasp authority')
    s = dict(source_stamp_ns=ns,accepted_source_stamp_ns=binding['accepted_source_stamp_ns'],
        snapshot_received_ros_ns=binding['snapshot_received_ros_ns'],epoch_ns=1_000_000_000,
        q_rad=q.tolist(),sdk_q_rad=q.tolist(),reference_q_rad=q.tolist(),sdk_frame_id='sdk_transmitted',
        reference_frame_id='sdk_manual',task_active=False,protection_latched=False,feedback_ready=True,
        gui_direct_mode=True,actuation_status='CONFIRMED: fixture',source_binding=binding,
        board_quality=quality,camera_model_evidence=deepcopy(evidence),camera_model_fingerprint=model.fingerprint,
        K=[400.,400.,320.,240.],width=640,height=480,depth_scale=.0001,
        T_tool_camera=X.tolist(),T_base_camera=C.tolist(),T_camera_board=B.tolist(),T_base_board=Y.tolist(),
        joint_min=[-3.14]*6,joint_max=[3.14]*6,openings=[.04975],
        urdf_sha256=hashlib.sha256(xml.encode()).hexdigest(),scene_contract=scene,
        independent_depth_support_diagnostic=dict(status='support_plane_inlier_ratio_low',ratio=.55))
    return s,xml,model


def model_now(s):
    return {key:deepcopy(s[key]) for key in ('urdf_sha256','T_tool_camera',
        'camera_model_evidence','camera_model_fingerprint','epoch_ns','depth_scale')}


def make_plan():
    s,xml,model=fixture()
    current=model_now(s)
    plan=board.BoardOrthogonalProbeAudit(s,xml,lambda q,g:True,model,lambda:current)
    return plan,s,current


def test_snapshot_is_immutable_and_does_not_manufacture_depth_success():
    s,xml,model=fixture(); frozen=board.build_board_snapshot(s,xml,model)
    s['T_base_board'][0][3]=10.;s['board_quality']['charuco_count']=0
    assert frozen['T_base_board'][0][3]==0.
    assert frozen['board_quality']['charuco_count']==35
    with pytest.raises(TypeError):frozen['T_base_board'][0][3]=2.
    assert frozen['board_plane_source']=='same_source_RGB_charuco_printed_face'
    assert not frozen['grasp_authorized']
    assert frozen['independent_depth_support_diagnostic']['status']=='support_plane_inlier_ratio_low'
    assert 'plane_base' not in frozen and 'plane_fit' not in frozen


@pytest.mark.parametrize('field', ['rgb_stamp_ns','depth_stamp_ns','board_stamp_ns','quality_stamp_ns'])
def test_cross_source_pair_never_admitted(field):
    s,xml,model=fixture();s['source_binding'][field]+=1
    with pytest.raises(RuntimeError,match='exact source'):board.build_board_snapshot(s,xml,model)


@pytest.mark.parametrize('mutate', [
    lambda s:s.update(epoch_ns=s['source_stamp_ns']),
    lambda s:s['source_binding'].update(snapshot_received_ros_ns=s['source_stamp_ns']+350_000_001),
    lambda s:s['source_binding'].update(quality_received_ros_ns=s['source_stamp_ns']-1),
    lambda s:s['source_binding'].update(accepted_before_ns=s['source_stamp_ns']+1),
    lambda s:s['source_binding'].update(accepted_after_ns=s['source_stamp_ns']+200_000_001),
    lambda s:s['source_binding'].update(accepted_source_stamp_ns=s['source_stamp_ns']-100_000_001),
    lambda s:s['source_binding'].update(sdk_stamp_ns=s['epoch_ns']-1),
    lambda s:s['source_binding'].update(reference_stamp_ns=s['source_stamp_ns']+400_000_000),
    lambda s:s['source_binding'].update(accepted_window_duration_sec=.69999),
    lambda s:s['source_binding'].update(accepted_window_span_deg=[.15001]*6),
])
def test_source_time_bracket_and_stationary_bounds_cannot_relax(mutate):
    s,xml,model=fixture();mutate(s)
    with pytest.raises(RuntimeError):board.build_board_snapshot(s,xml,model)


@pytest.mark.parametrize('field,value', [
    ('charuco_count',34),('reprojection_rms_px',1.500001),('edge_clearance_px',19.9999),
    ('pose_ok',False),('board_size',[11,9]),('square_length_m',.016),('marker_length_m',.012),
    ('dictionary_id','DICT_4X4_50'),('camera_model_change_latched',True),('camera_frame','optical'),
    ('camera_model_error','changed'),('image_width',639),('charuco_count',True),
])
def test_original_board_quality_gates_remain(field,value):
    s,xml,model=fixture();s['board_quality'][field]=value
    with pytest.raises(RuntimeError):board.build_board_snapshot(s,xml,model)


@pytest.mark.parametrize('field,value', [
    ('sdk_frame_id','sdk_measured'),('reference_frame_id','sdk_transmitted'),('feedback_ready',False),
    ('task_active',True),('protection_latched',True),('gui_direct_mode',False),('actuation_status','READY'),
    ('urdf_sha256','incorrect'),('depth_scale',0.),
])
def test_model_authority_and_sdk_reference_gates(field,value):
    s,xml,model=fixture();s[field]=value
    with pytest.raises(RuntimeError):board.build_board_snapshot(s,xml,model)


def test_held_sdk_word_or_gripper_change_rejected():
    for axis in (2,6):
        s,xml,model=fixture();s['reference_q_rad'][axis]+=board.base.QUANTUM if axis<6 else .0001
        with pytest.raises(RuntimeError,match='SDK/reference'):board.build_board_snapshot(s,xml,model)


@pytest.mark.parametrize('field', ['T_tool_camera','T_base_camera','T_camera_board','T_base_board'])
def test_full_exact_fk_tf_chain_rejected_after_translation_change(field):
    s,xml,model=fixture();s[field][0][3]+=.0001
    with pytest.raises(RuntimeError,match='TF.*FK'):board.build_board_snapshot(s,xml,model)


def test_rigid_transform_reflection_rejected():
    s,xml,model=fixture();s['T_camera_board'][0][0]*=-1
    # Change a nonzero entry; reflection changes determinant.
    s['T_camera_board'][1][0]*=-1;s['T_camera_board'][2][0]*=-1
    with pytest.raises(RuntimeError,match='proper rigid'):board.build_board_snapshot(s,xml,model)


@pytest.mark.parametrize('field,value', [
    ('board_printed_plane_is_local_table_upper_bound',False),('old_board_thickness_reused',True),
    ('evidence',''),('scope','grasp permission'),('kind','standalone_vertical_board'),
])
def test_board_never_silently_becomes_table_or_grasp_authority(field,value):
    s,xml,model=fixture();s['scene_contract'][field]=value
    with pytest.raises(RuntimeError,match='scene contract'):board.build_board_snapshot(s,xml,model)


def test_whole_board_margin_cannot_be_replaced_by_inner_corner_margin():
    s,xml,model=fixture();s['T_base_board'][0][3]=.18
    C=np.array(s['T_base_camera']);s['T_camera_board']=(np.linalg.inv(C)@s['T_base_board']).tolist()
    s['board_quality']['edge_clearance_px']=80.
    with pytest.raises(RuntimeError,match='full-board'):board.build_board_snapshot(s,xml,model)


def test_builder_does_not_double_convert_ros_board_tf():
    s,xml,model=fixture();out=board.build_board_snapshot(s,xml,model)
    np.testing.assert_allclose(out['T_base_board'],np.eye(4),rtol=0,atol=1e-12)
    assert out['initial_full_board_edge_lower_bound_px']>120.


@pytest.mark.parametrize('mutation', ['coefficients','sdk','serial','intrinsics'])
def test_full_distortion_sdk_device_or_intrinsics_mutation_detected(mutation):
    s,xml,model=fixture();audit=board.BoardGeometryAudit(s,xml,lambda q,g:True,model)
    if mutation=='coefficients':model.evidence['color_intrinsics']['coeffs'][0]+=.01
    if mutation=='sdk':model.backend_sdk_version='2.55.1'
    if mutation=='serial':model.evidence['serial_number']='other_device'
    if mutation=='intrinsics':model.intrinsics.fx+=1.
    with pytest.raises(RuntimeError,match='model|intrinsics'):audit.geometry(s['q_rad'])


def test_continuous_fov_proof_encloses_sdk_projected_board_interiors():
    s,xml,model=fixture();audit=board.BoardGeometryAudit(s,xml,lambda q,g:True,model)
    half=np.full(6,math.radians(.2));q=np.zeros(6)
    t=board.board_camera_box_taylor_bound(audit.fk,q,audit.X,audit.Y,half)
    rectangle=board.projection_corner_boxes(model,t['corner_lower_xyz_m'],t['corner_upper_xyz_m'])
    rng=np.random.RandomState(37)
    interior=np.c_[rng.uniform(0,.18,80),rng.uniform(0,.135,80),np.zeros(80),np.ones(80)]
    for delta in rng.uniform(-half,half,(25,6)):
        p=(np.linalg.inv(audit.fk(delta)@audit.X)@audit.Y@interior.T).T[:,:3]
        uv=np.array([rs.rs2_project_point_to_pixel(model.intrinsics,[-v[1],-v[2],v[0]]) for v in p])
        assert np.all(uv>=rectangle[:,0]) and np.all(uv<=rectangle[:,1])
    proof=audit.guard_box(-half,half)
    assert proof['continuous_model_box_certified'] and proof['minimum_clearance_m']>=.04
    assert not proof['whole_arm_continuous_collision_certified']


def test_current_moveit_rechecked_after_cache_clear():
    s,xml,model=fixture();state={'valid':True}
    audit=board.BoardGeometryAudit(s,xml,lambda q,g:state['valid'],model)
    assert audit.guard_segment(np.zeros(6),np.zeros(6))['all_moveit_valid']
    state['valid']=False;audit.cache.clear()
    assert not audit.guard_segment(np.zeros(6),np.zeros(6))['all_moveit_valid']


def test_40mm_cad_cannot_be_replaced_by_camera_distance():
    s,xml,model=fixture();audit=board.BoardGeometryAudit(s,xml,lambda q,g:True,model)
    first=audit.geometry(np.zeros(6))['clearance_m']
    s['T_base_board'][2][3]=first-.03999
    C=np.array(s['T_base_camera']);s['T_camera_board']=(np.linalg.inv(C)@s['T_base_board']).tolist()
    changed=board.BoardGeometryAudit(s,xml,lambda q,g:True,model)
    with pytest.raises(RuntimeError,match='40 mm'):changed.geometry(np.zeros(6))


def test_no_implicit_work_range_expansion():
    for key,value in [('board_working_depth_min_m',.11999),('board_working_depth_max_m',.50001)]:
        s,xml,model=fixture();s[key]=value
        with pytest.raises(RuntimeError,match='120..500'):board.build_board_snapshot(s,xml,model)


def test_existing_signed_repeat_words_and_exact_return_unchanged():
    plan,s,current=make_plan()
    assert len(plan.sequence)==16
    for i in range(0,16,2):
        first,last=plan.sequence[i:i+2]
        delta=np.array(first['target_words'])-board.base.words(plan.reference)
        assert np.count_nonzero(delta)==1 and abs(delta[first['axis']])==34
        np.testing.assert_array_equal(last['target_sdk'],plan.reference)
    assert plan.contract['response_extra_deg']==.8
    assert plan.contract['geometry_min_clearance_m']==.04
    assert not plan.contract['grasp_authorized']
    with pytest.raises(ValueError):plan.source_q.setflags(write=True)
    with pytest.raises(ValueError):plan.reference.setflags(write=True)


def test_full_readonly_audit_and_actual_step_include_independent_response_proofs():
    plan,s,current=make_plan();proof=plan.audit_nominal_sequence()
    assert proof['all_pass'] and len(proof['proof_steps'])==16
    plan.begin_step(0,s['q_rad'],s['sdk_q_rad'])
    result=plan.guard_segment(s['q_rad'],plan.sequence[0]['target_sdk'])
    assert result['continuous_physical_response']['continuous_model_box_certified']
    assert result['independent_nominal_sdk_bridge']['continuous_model_box_certified']
    assert result['continuous_physical_response']['upper_rad']!=result['independent_nominal_sdk_bridge']['upper_rad']
    assert not result['grasp_authorized']


@pytest.mark.parametrize('field,value', [
    ('urdf_sha256','wrong'),('epoch_ns',999),('camera_model_fingerprint','wrong'),('depth_scale',.001),
])
def test_fresh_model_guard_before_any_step(field,value):
    plan,s,current=make_plan();current[field]=value
    with pytest.raises(RuntimeError,match='changed'):plan.check_model()


def test_fresh_handeye_rotation_guard_not_only_translation():
    plan,s,current=make_plan();angle=.001
    R=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
    X=np.array(current['T_tool_camera']);X[:3,:3]=R@X[:3,:3];current['T_tool_camera']=X.tolist()
    with pytest.raises(RuntimeError,match='changed'):plan.check_model()


def test_immutable_baseline_y_not_updated_from_current_q_or_external_bundle():
    plan,s,current=make_plan();original=plan.audit.Y.copy()
    s['T_base_board'][2][3]=.02
    current['q_rad']=[.1]*7
    plan.check_model()
    np.testing.assert_array_equal(plan.audit.Y,original)
    plan.audit.Y[2,3]+=.02
    with pytest.raises(RuntimeError,match='frozen'):plan.check_model()


def test_mutable_step_target_change_detected():
    plan,s,current=make_plan();plan.sequence[0]['target_sdk'][3]+=board.base.QUANTUM
    with pytest.raises(RuntimeError,match='sequence'):plan.check_during_step(s['q_rad'])


def test_missing_online_model_reader_never_authorizes_submission():
    s,xml,model=fixture();plan=board.BoardOrthogonalProbeAudit(s,xml,lambda q,g:True,model)
    with pytest.raises(RuntimeError,match='reader'):plan.check_model()


def test_broader_runtime_limits_are_only_tightened_to_urdf_intersection():
    s,xml,model=fixture();s['joint_min']=[-3.1415]*6;s['joint_max']=[3.1415]*6
    frozen=board.build_board_snapshot(s,xml,model)
    np.testing.assert_array_equal(frozen['joint_min'],[-3.14]*6)
    np.testing.assert_array_equal(frozen['joint_max'],[3.14]*6)
    assert frozen['recorded_runtime_joint_min']==tuple([-3.1415]*6)
    assert frozen['recorded_runtime_joint_max']==tuple([3.1415]*6)


def test_rebinding_sdk_or_absolute_control_domain_is_detected():
    for field in ('reference','absolute_lo','absolute_hi','source_q'):
        plan,s,current=make_plan();changed=np.array(getattr(plan,field));changed[0]+=.01
        setattr(plan,field,changed)
        with pytest.raises(RuntimeError,match='frozen'):plan.check_during_step(s['q_rad'])


def test_model_change_during_geometry_reply_rejected_before_returning_proof():
    plan,s,current=make_plan();plan.audit_nominal_sequence(False)
    plan.begin_step(0,s['q_rad'],s['sdk_q_rad'])
    original=plan.audit.guard_box
    def mutate_after_reply(*args,**kwargs):
        result=original(*args,**kwargs)
        current['epoch_ns']+=1
        return result
    plan.audit.guard_box=mutate_after_reply
    with pytest.raises(RuntimeError,match='changed'):
        plan.guard_segment(s['q_rad'],plan.sequence[0]['target_sdk'])
