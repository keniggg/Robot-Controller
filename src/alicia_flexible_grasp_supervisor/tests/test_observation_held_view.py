"""Actual failed scene; fresh-response transport below is explicitly simulated."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib
import json
import math

import numpy as np
import pytest
from geometry_msgs.msg import Pose

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.observation_endpoint_compensation import (
    ObservationEndpointCompensation, validate_observation_correction_path_bounds)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from test_endpoint_correction import sample
from test_observation_endpoint_compensation import path


def recorded(scale=1.):
    fixtures=Path(__file__).with_name('fixtures');stem='observation_held_view_20261023'
    d=json.loads((fixtures/(stem+'.json')).read_text())
    root=Path(__file__).resolve().parents[3]
    xml=(root/d['model_relative_path']).read_bytes()
    assert hashlib.sha256(xml).hexdigest()==d['model_sha256']
    archive=fixtures/(stem+'.npz')
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==d['arrays_sha256']
    with np.load(archive) as a:valid=a['rgb_valid'].copy()
    fk=SerialUrdfFk(xml.decode(),ARM_NAMES)
    goal=Pose();goal.position.x,goal.position.y,goal.position.z=d['goal'][:3]
    goal.orientation.x,goal.orientation.y,goal.orientation.z,goal.orientation.w=d['goal'][3:]
    scene=NS(**{k:np.array(v) if k in ('center','rotation','size') else v for k,v in d['scene'].items()})
    scene.size=scene.size*scale
    # Exact recorded q pairs; timestamp/epoch packaging is a test fixture,
    # not a claim of current feedback or permission to send saved commands.
    state=sample(fk,d['initial_sdk'],d['initial_measured'])
    return ObservationEndpointCompensation(fk,state,goal,scene,d['configuration'],valid),state


@pytest.mark.parametrize('scale',[.7,.85,1.])
def test_same_bounded_guide_keeps_weak_joints_and_requires_pose_and_view(scale):
    c,state=recorded(scale);fixed=pose_values(c.goal)
    step,evidence=c.propose_held_axes()
    delta=np.array(step.target_counts)-step.baseline_counts
    assert delta[1]==delta[4]==0 and np.max(abs(delta))<=32
    assert evidence['predicted_position_error_m']<=.010
    assert evidence['predicted_orientation_error_rad']<=math.radians(5)
    assert evidence['predicted_view']['ok']
    assert evidence['predicted_view']['minimum_image_edge_distance_px']>=24.
    assert not evidence['hardware_response_verified'] and pose_values(c.goal)==fixed
    c.committed(step,12.)
    actual=np.array(state['accepted_positions_rad'])+delta*Q
    response=c.evaluate(sample(c.fk,step.positions,actual,14.))
    assert response['view_audit']['ok'] and response['hardware_response_verified']
    with pytest.raises(ValueError,match='ALREADY_PROPOSED'):c.propose_held_axes()


def test_pose_only_held_guide_still_fails_recorded_view():
    c,state=recorded()
    step,evidence=c.recovery.propose_held_axes()
    assert evidence['predicted_position_error_m']<.006
    with pytest.raises(ValueError,match='image margin'):
        c._check_predicted_view(step,evidence)


@pytest.mark.parametrize('fault',['stall','reverse','held_moved','stale'])
def test_new_view_prediction_never_replaces_measured_response(fault):
    c,state=recorded();step,_=c.propose_held_axes();c.committed(step,12.)
    delta=np.array(step.target_counts)-step.baseline_counts
    if fault=='stall':delta[:]=0
    if fault=='reverse':delta=-delta
    if fault=='held_moved':delta[1]=3
    after=sample(c.fk,step.positions,np.array(state['accepted_positions_rad'])+delta*Q,14.)
    if fault=='stale':after['stationary_window_start_ns']=11_999_999_999
    with pytest.raises(ValueError):c.evaluate(after)


def test_unobserved_rgb_cannot_become_a_qualified_correction():
    c,_=recorded()
    c.rgb_valid=np.zeros_like(c.rgb_valid)
    with pytest.raises(ValueError,match='RGB_REGION_UNOBSERVED'):c.propose_held_axes()


@pytest.mark.parametrize('axis',[1,4])
def test_held_command_cannot_move_between_unchanged_endpoints(axis):
    goal=[0.,0.,10*Q,0.,0.,0.]
    p=path(goal);assert validate_observation_correction_path_bounds(p,[0.]*6,goal,hold_weak_axes=True)['held_joints']==['Joint2','Joint5']
    p.joint_trajectory.points[-1].velocities[axis]=.01
    assert validate_observation_correction_path_bounds(p,[0.]*6,goal)['complete_timed_curve_checked']
    with pytest.raises(ValueError,match='MOVES_HELD_AXIS'):
        validate_observation_correction_path_bounds(p,[0.]*6,goal,hold_weak_axes=True)


@pytest.mark.parametrize('axis',[1,4])
def test_held_target_itself_cannot_change(axis):
    goal=[0.]*6;goal[axis]=Q
    with pytest.raises(ValueError,match='MOVES_HELD_AXIS'):
        validate_observation_correction_path_bounds(path(goal),[0.]*6,goal,hold_weak_axes=True)
