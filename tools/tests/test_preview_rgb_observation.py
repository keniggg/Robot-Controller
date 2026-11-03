"""Verify diagnostic preview evidence and finite candidate scope without ROS."""
import ast
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

WORK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORK/'src/alicia_grasp_modes/src'))
sys.path.insert(0, str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
sys.path.insert(0, str(WORK/'tools/diagnostics'))
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume, SupportPlaneProvenance

SCRIPT = WORK/'tools/diagnostics/preview_rgb_observation.py'
spec = importlib.util.spec_from_file_location('rgb_preview_test', SCRIPT)
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def record(tmp_path):
    stamp = 1_000_000_000
    T = np.array([[0.,-1.,0.,0.],[0.,0.,1.,0.],[-1.,0.,0.,.5],[0.,0.,0.,1.]])
    source = tmp_path/'source.npz'
    support = SupportPlaneProvenance(stamp, (0.,0.,1.,0.), str(source), 800, .9, .001, 4)
    volume = RGBObservationVolume((300,220,40,40), (480,640),
        ((400.,0.,320.),(0.,400.,240.),(0.,0.,1.)), T,
        (0.,0.,1.,0.), support, .12, stamp, (.07,.5))
    rgb = np.zeros((480,640,3),dtype=np.uint8)
    depth = np.full((480,640),5000,dtype=np.uint16)
    camera = preview.camera_geometry_snapshot(dict(fx=400.,fy=400.,cx=320.,cy=240.,
        width=640,height=480,depth_scale=.0001,frame_id='camera_link',align_depth_to_color=True))
    selection = dict(mode='unknown',strategy='observe_return',generation=1,stamp_ns=100)
    context = dict(camera=camera,selection=selection,epoch_ns=200,
        tool_camera=np.eye(4).tolist(),current_tool_camera=np.eye(4).tolist(),
        observation_view_parameters=preview.DEFAULT_OBSERVATION_VIEW_PARAMETERS,
        perception={},handeye={},model_sha256='a'*64)
    source_id = preview.digest(context)
    np.savez(str(source), source_stamp_ns=np.int64(stamp), q_source=np.zeros(6), T_base_camera=T,
        T_tool_camera=np.eye(4),K=np.array([400.,400.,320.,240.]),depth_scale=.0001,
        color_bgr=rgb,depth_raw=depth)
    row = dict(source_stamp_ns=stamp, source_file=str(source),
        source_file_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        observation_interest_ready=True, source_context_unchanged=True,
        fresh_for_existing_frame_limit=True, motion_authorized=False,
        contact_geometry_authorized=False, interest_volume=asdict(volume),
        T_base_camera=T.tolist(), q_source=[0.]*6,config=asdict(preview.Config()),
        source_context=context,source_id=source_id,selection=selection,epoch_ns=200,
        source_rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
        source_depth_sha256=hashlib.sha256(depth.tobytes()).hexdigest(),
        accepted_before=[stamp-10_000_000,[0.]*6],accepted_after=[stamp+10_000_000,[0.]*6],
        rgb_track=dict(confirmed=True,source_id=source_id,generation=1,
                       source_stamps_ns=[stamp-200_000_000,stamp-100_000_000,stamp]),
        rgb_candidates=[dict(bbox_xywh=[300,220,40,40])],rgb_reason='observation_region_found',
        observation_working_limits=preview.effective_observation_view_limits(context['observation_view_parameters']))
    path = tmp_path/'result.json'
    path.write_text(json.dumps(dict(rows=[row])))
    return path, json.loads(path.read_text())['rows'][0]


def test_reload_reconstructs_checked_volume_but_never_authority(tmp_path):
    path, row = record(tmp_path)
    loaded, volume = preview.read_interest_record(path)
    assert loaded == row
    assert not volume.motion_authorized and not volume.usable_as_collision_geometry


@pytest.mark.parametrize('mutation', ['hash','stamp','vertices','q','not_ready',
    'rgb_hash','depth_hash','context','source_id','bbox','track_generation','before_feedback','working_depth'])
def test_changed_record_or_source_is_rejected(tmp_path, mutation):
    path, row = record(tmp_path)
    if mutation == 'hash': row['source_file_sha256'] = 'f'*64
    if mutation == 'stamp': row['source_stamp_ns'] += 1
    if mutation == 'vertices': row['interest_volume']['vertices_base_m'][0] = [0.,0.,1.]
    if mutation == 'q': row['q_source'][0] = 1.
    if mutation == 'not_ready': row['observation_interest_ready'] = False
    if mutation == 'rgb_hash': row['source_rgb_sha256'] = 'f'*64
    if mutation == 'depth_hash': row['source_depth_sha256'] = 'f'*64
    if mutation == 'context': row['source_context']['tool_camera'][0][3] = .01
    if mutation == 'source_id': row['source_id'] = 'f'*64
    if mutation == 'bbox': row['rgb_candidates'][0]['bbox_xywh'][0] += 1
    if mutation == 'track_generation': row['rgb_track']['generation'] += 1
    if mutation == 'before_feedback': row['accepted_before'][1][0] = .1
    if mutation == 'working_depth': row['interest_volume']['working_depth_range_m'][1] = .7
    path.write_text(json.dumps(dict(rows=[row])))
    with pytest.raises(ValueError): preview.read_interest_record(path)


def test_finite_candidates_cover_all_axes_and_respect_joint_limits():
    def fk(q):
        matrix = np.eye(4); matrix[0,3] = q[0]*.1
        return matrix
    result = preview.local_joint_candidates(np.zeros(6), [-1.]*6, [1.]*6, fk, np.eye(4))
    assert len(result) == 24 and len({row.candidate_id for row in result}) == 24
    for row in result:
        assert np.count_nonzero(row.joint_positions) == 1
        assert max(np.abs(row.joint_positions)) <= np.radians(10.)
        np.testing.assert_array_equal(row.T_base_camera, fk(row.joint_positions))
    clipped = preview.local_joint_candidates(np.zeros(6), [-.1]*6, [.1]*6, fk, np.eye(4))
    assert len(clipped) == 12


def trajectory(values, times):
    return SimpleNamespace(joint_names=preview.NAMES, points=[SimpleNamespace(
        positions=value, time_from_start=SimpleNamespace(to_sec=lambda t=t:t))
        for value,t in zip(values,times)])


def test_metrics_expose_large_intermediate_detour_not_only_endpoint():
    data = preview.trajectory_metrics(trajectory([[0.]*6,[1.]+[0.]*5,[.1]+[0.]*5], [0.,1.,2.]), [0.]*6)
    assert data['total_joint_travel_rad'] == pytest.approx(1.9)
    assert data['max_joint_excursion_deg'][0] == pytest.approx(np.degrees(1.))
    assert data['duration_sec'] == 2. and not data['motion_authorized']
    assert data['continuous_joint_excursion_bound_deg'][0] >= np.degrees(1.)


def test_metrics_expose_spline_excursion_hidden_between_equal_waypoints():
    path=trajectory([[0.]*6,[0.]*6],[0.,1.])
    path.points[0].velocities=[0.]*6
    path.points[1].velocities=[-30.]+[0.]*5
    data=preview.trajectory_metrics(path,[0.]*6)
    assert data['max_joint_excursion_deg'][0] == 0.
    # The actual cubic has q(s)=30*s^2*(1-s), with a 40/9 rad maximum.
    assert data['continuous_joint_excursion_bound_deg'][0] >= np.degrees(40./9.)
    assert 'not_physical_measurement' in data['continuous_bound_semantics']


def feedback(values, stamp=1_100_000_000, names=None):
    return SimpleNamespace(name=list(names or preview.NAMES),position=list(values),
        header=SimpleNamespace(frame_id='sdk_measured',stamp=SimpleNamespace(to_nsec=lambda:stamp)))


def test_move_away_and_back_during_rpc_still_revokes_following_previews(tmp_path):
    _,row=record(tmp_path)
    guard=preview.PreviewRevocationGuard(row)
    guard.joint_sample(feedback([0.]*6),1_100_000_000)
    guard.check()
    guard.joint_sample(feedback([.01]+[0.]*5),1_100_000_000)
    guard.joint_sample(feedback([0.]*6),1_100_000_000)
    with pytest.raises(ValueError,match='pose_or_feedback_changed'):
        guard.check()


@pytest.mark.parametrize('kind',['epoch','selection','manual','invalid_feedback'])
def test_transient_owner_change_cannot_be_cleared_by_returning_state(tmp_path,kind):
    _,row=record(tmp_path)
    guard=preview.PreviewRevocationGuard(row)
    if kind == 'epoch':
        guard.epoch_sample(row['epoch_ns']+1);guard.epoch_sample(row['epoch_ns'])
    elif kind == 'selection':
        guard.selection_sample(dict(row['selection'],generation=2));guard.selection_sample(row['selection'])
    elif kind == 'manual':
        guard.revoke('manual_control_active')
    else:
        guard.joint_sample(feedback([0.]*6,names=['Joint1']*6),1_100_000_000)
    with pytest.raises(ValueError,match='no_more_previews'):
        guard.check()


def test_context_uses_shared_geometry_binding_not_camera_filter_telemetry(tmp_path):
    _,row=record(tmp_path)
    params={'/grasp_mode/selection':row['selection'],'/camera':dict(row['source_context']['camera']),
        '/unknown_perception':{},'/handeye':{},'/robot_description':'test model'}
    get=lambda key,default=None:params.get(key,default)
    first=preview.current_context(get,np.eye(4))
    params['/camera']['temporal_filter_active']=True
    params['/camera']['frame_count']=100
    assert first == preview.current_context(get,np.eye(4))
    params['/handeye']['calibration_id']='changed'
    assert first != preview.current_context(get,np.eye(4))


@pytest.mark.parametrize('values,times', [([[0.]*6,[np.nan]*6],[0.,1.]),
    ([[0.]*6,[1.]*6],[1.,0.]), ([[0.]*5,[1.]*5],[0.,1.])])
def test_invalid_path_cannot_be_reported_valid(values,times):
    with pytest.raises(ValueError): preview.trajectory_metrics(trajectory(values,times), [0.]*6)


def test_only_planning_service_and_execute_false():
    tree = ast.parse(SCRIPT.read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    assert not any(isinstance(node.func, ast.Attribute) and node.func.attr in
                   ('Publisher','set_param','delete_param','send_goal') for node in calls)
    requests = [node for node in calls if isinstance(node.func, ast.Name) and node.func.id == 'SetTargetPoseRequest']
    assert len(requests) == 1
    assert next(item.value.value for item in requests[0].keywords if item.arg == 'execute') is False
