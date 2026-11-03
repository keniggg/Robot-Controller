"""Whole-box enclosure, derivatives, recorded failure and input faults."""
from copy import deepcopy
from itertools import product
from pathlib import Path
import json

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_arrival_reserve import (
    PreparedObservationArrivalReserve,observation_goal_error_bounds,arrival_reserve_from_source)
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES,SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.grasp.observation_working_volume import observation_obb_corners
from test_reachable_observation_seed_replay import load_recorded_scene


def recorded(stem='observation_joint_view_20261023'):
    d=json.loads((Path(__file__).with_name('fixtures')/(stem+'.json')).read_text())
    root=Path(__file__).resolve().parents[3]
    fk=SerialUrdfFk((root/d['model_relative_path']).read_text(),ARM_NAMES)
    assert fk.model_sha256==d['model_sha256']
    s=d['scene'];cfg=d['configuration']
    points=observation_obb_corners(np.array(s['center']),np.array(s['rotation']),np.array(s['size']))
    errors=observation_goal_error_bounds({n:dict(goal=.035) for n in ARM_NAMES},ARM_NAMES)
    bound=PreparedObservationArrivalReserve(fk,errors,cfg['tool_from_camera'],points,cfg['camera'],cfg['limits'])
    return bound,d


@pytest.mark.parametrize('stem',['observation_joint_view_20261023','observation_held_view_20261023'])
def test_recorded_nominal_endpoint_near_border_lacks_arrival_reserve(stem):
    bound,d=recorded(stem)
    report=bound.audit(d['initial_sdk'])
    assert report['nominal_minimum_image_edge_distance_px']>24
    assert not report['ok']
    assert min(report['halfspace_clearance_lower_bounds_m'][2:])<0
    assert report['actual_arrival_measurement_required'] and not report['execution_authority']


@pytest.mark.parametrize('posture',[0,1,2])
def test_analytic_camera_derivatives_match_independent_fk_differences(posture):
    bound,d=recorded();q=np.array(d['initial_sdk'])
    q+=posture*np.array([.09,.04,-.06,.05,.07,-.04])
    _,jacobian=bound.geometry(q);epsilon=1.e-6
    for i in range(6):
        delta=np.zeros(6);delta[i]=epsilon
        plus=bound.fk(q+delta)@bound.mount;minus=bound.fk(q-delta)@bound.mount
        diff=((bound.points-plus[:3,3])@plus[:3,:3]-(bound.points-minus[:3,3])@minus[:3,:3])/(2*epsilon)
        np.testing.assert_allclose(diff,jacobian[:,i],rtol=1.e-7,atol=1.e-9)


@pytest.mark.parametrize('error_scale',[.25,1.,2.])
@pytest.mark.parametrize('scene_scale',[.6,1.,1.6])
def test_all_joint_box_corners_and_interior_stay_in_taylor_enclosure(error_scale,scene_scale):
    bound,d=recorded();centre=np.mean(bound.points,axis=0)
    points=centre+(bound.points-centre)*scene_scale
    bound=PreparedObservationArrivalReserve(bound.fk,bound.errors*error_scale,bound.mount,
        points,bound.camera,bound.limits)
    q=np.array(d['initial_sdk'])+np.array([.15,.1,-.1,.2,.2,.1])
    geometry=bound.geometry(q);rng=np.random.default_rng(3912)
    normals=np.r_[np.eye(3),-np.eye(3),rng.normal(size=(6,3))]
    lower=bound._lower(geometry,normals,np.zeros(len(normals)))
    for direction in list(product((-1,1),repeat=6))+rng.uniform(-1,1,(80,6)).tolist():
        actual=q+np.array(direction)*bound.errors
        camera=bound.fk(actual)@bound.mount
        local=(points-camera[:3,3])@camera[:3,:3]
        assert np.all(local@normals.T>=lower-1.e-12)


def test_nominally_visible_rgb_hole_inside_error_envelope_is_rejected():
    args=load_recorded_scene('supplement_view_joint_reserve_20261023')
    errors=observation_goal_error_bounds({n:dict(goal=.035) for n in ARM_NAMES},ARM_NAMES)
    b=arrival_reserve_from_source(args['fk'],errors,args['source'])
    # A frozen replay's interior proposal, not an execution command.
    q=[-2.0924490288171715,-.39872216355348816,.9545993547855369,
       -.32847323639266685,-1.341857721348059,-.13757099724987473]
    valid=args['source']['arrival_projection']['rgb_valid'].copy()
    report=b.audit(q,rgb_valid=valid);assert report['ok']
    x0,y0,x1,y1=report['required_observed_box_xyxy'];valid[y0,x0]=False
    assert not b.audit(q,rgb_valid=valid)['ok']
    assert b.audit(q,rgb_valid=np.ones_like(valid))['ok']


@pytest.mark.parametrize('fault',['missing','zero','negative','disabled','nan','boolean'])
def test_missing_or_disabled_goal_tolerance_cannot_mean_zero_error(fault):
    constraints={n:dict(goal=.035) for n in ARM_NAMES}
    if fault=='missing':del constraints['Joint3']
    else:constraints['Joint3']['goal']={'zero':0,'negative':-.1,'disabled':-1,'nan':float('nan'),'boolean':True}[fault]
    with pytest.raises(ValueError):observation_goal_error_bounds(constraints,ARM_NAMES)


def test_tolerance_comes_from_current_config_and_inputs_are_immutable():
    constraints={n:dict(goal=.01+i*.002) for i,n in enumerate(ARM_NAMES)}
    errors=observation_goal_error_bounds(constraints,ARM_NAMES)
    np.testing.assert_allclose(errors,[.01+i*.002+Q/2 for i in range(6)])
    b,d=recorded();points=b.points.copy();mount=b.mount.copy();epsilon=b.errors.copy()
    other=PreparedObservationArrivalReserve(b.fk,epsilon,mount,points,b.camera,b.limits)
    before=other.audit(d['initial_sdk']);points[:]=0;mount[:]=0;epsilon[:]=0
    assert other.audit(d['initial_sdk'])==before


@pytest.mark.parametrize('fault',['model','error','points','mount','camera','limits'])
def test_invalid_bound_input_fails_closed(fault):
    b,_=recorded();args=[b.fk,b.errors.copy(),b.mount.copy(),b.points.copy(),deepcopy(b.camera),deepcopy(b.limits)]
    if fault=='model':args[0]=object()
    elif fault=='error':args[1][0]=np.nan
    elif fault=='points':args[3][0,0]=np.nan
    elif fault=='mount':args[2][0,0]=5
    elif fault=='camera':args[4]['fx']=0
    else:args[5]['roi_margin_px']=0
    with pytest.raises(ValueError):PreparedObservationArrivalReserve(*args)
