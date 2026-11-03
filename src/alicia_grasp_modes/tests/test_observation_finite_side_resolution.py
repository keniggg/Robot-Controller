"""Finite side rays and measured resolution, without robot/ROS authority."""
from copy import deepcopy
import math
import numpy as np
import pytest

from alicia_grasp_modes.observe_return_visual import missing_side_view_gain, missing_side_ray_projection
from alicia_grasp_modes.observation_information_uncertainty import observation_resolvable_gain_target_m
from alicia_grasp_modes.observation_view_utility import predicted_information_utility, information_motion_rank


def gap(span=.02,width=.04):
    return dict(angle_deg=0.,jaw_axis_base=[1.,0.,0.],object_height_m=span,
        whole_obb_width_m=width,target_center_base_m=[0.,0.,0.],
        initial_camera_origin_base_m=[-.1,0.,.2],missing_side_signs=[1],
        target_epoch=1,target_track_id='frozen-target',source_view_stamps_ns=[10])


@pytest.mark.parametrize('noise,target,expected',[(.003,.002,.003),(.001,.002,.002),(.003,None,.003),(0.,None,0.)])
def test_task_target_cannot_lower_visual_resolution(noise,target,expected):
    assert observation_resolvable_gain_target_m(noise,target)==pytest.approx(expected)


@pytest.mark.parametrize('bad',[True,'0.003',None,float('nan'),float('inf'),-.001])
def test_malformed_uncertainty_fails(bad):
    with pytest.raises(ValueError):observation_resolvable_gain_target_m(bad,.002)


def test_original_r18_selected_pose_overstates_exposure_at_box_center():
    # Minimal unchanged numeric excerpt of sealed 09c622... audit. This is
    # prediction reproduction, not evidence that its unobserved face exists.
    row=gap()
    row.update(angle_deg=40.000000000000014,
        jaw_axis_base=[.7652165821404155,.6437687726130119,.0023129690177450604],
        target_center_base_m=[-.08406230961687862,-.3470647226783133,.016046905051585247],
        initial_camera_origin_base_m=[-.05687386143097216,-.22189094538049686,.23105535654636442],
        object_height_m=.0075978197286329,whole_obb_width_m=.0474188018188931,missing_side_signs=[-1])
    camera=np.array([-.16872855669989192,-.33532755331504827,.19269648877981865])
    delta=camera-np.array(row['target_center_base_m']);jaw=np.array(row['jaw_axis_base'])
    old=row['object_height_m']*(-jaw@delta/np.linalg.norm(delta))
    finite=row['object_height_m']*missing_side_ray_projection(delta,jaw,.5*row['whole_obb_width_m'],-1)
    assert old==pytest.approx(.0022,abs=1e-12)
    assert finite==pytest.approx(.00131909554618653,abs=1e-12)
    assert not missing_side_view_gain([row],camera,.003,predicted_gain_target_m=.002)['ok']


def test_plane_grazing_camera_gets_no_large_credit_from_center_ray():
    row=gap(span=.04,width=.04)
    assert not missing_side_view_gain([row],[.0201,0.,.15],.003)['ok']
    good=missing_side_view_gain([row],[.15,0.,.12],.003)
    assert good['ok'] and good['predicted_new_side_gain_m']>.02
    assert good['effective_gain_target_m']==.003
    assert good['exposure_geometry_policy']=='finite_distance_missing_side_plane_ray_v2'
    assert not good['contact_or_motion_authorized']


@pytest.mark.parametrize('radius',[.05,.1,.2,.5])
def test_same_center_ray_never_gains_by_radial_motion(radius):
    row=gap();row['initial_camera_origin_base_m']=[.1,0.,.1]
    result=missing_side_view_gain([row],[radius,0.,radius],0.)
    assert not result['ok']


@pytest.mark.parametrize('angle',[0.,.4,1.3,2.7])
@pytest.mark.parametrize('scale',[.5,1.,1.8])
def test_finite_ray_is_rigid_covariant_and_scales_with_geometry(angle,scale):
    row=gap();camera=np.array([.15,.02,.16]);before=deepcopy(row)
    expected=missing_side_view_gain([row],camera,.003,predicted_gain_target_m=.002)
    axis=np.array([.3,.4,.5]);axis/=np.linalg.norm(axis)
    x,y,z=axis;K=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    R=np.eye(3)+math.sin(angle)*K+(1.-math.cos(angle))*(K@K);shift=np.array([.4,-.2,.5])
    transformed=deepcopy(row)
    for key in ('target_center_base_m','initial_camera_origin_base_m'):
        transformed[key]=(R@(scale*np.array(row[key]))+shift).tolist()
    transformed['jaw_axis_base']=(R@np.array(row['jaw_axis_base'])).tolist()
    for key in ('object_height_m','whole_obb_width_m'):transformed[key]*=scale
    actual=missing_side_view_gain([transformed],R@(scale*camera)+shift,scale*.003,predicted_gain_target_m=scale*.002)
    assert actual['ok'] and expected['ok']
    assert actual['predicted_new_side_gain_m']==pytest.approx(scale*expected['predicted_new_side_gain_m'])
    assert row==before


def test_low_task_target_does_not_make_subnoise_gain_compete_with_resolved_side():
    weak=predicted_information_utility(.0022,.003,.20547690458,395.60275,predicted_gain_target_m=.002)
    strong=predicted_information_utility(.007,.003,.23,395.60275,predicted_gain_target_m=.002)
    assert weak['utility']==0. and weak['predicted_gain_target_m']==.002
    assert weak['effective_gain_target_m']==.003 and weak['noise_px']>weak['gain_px']
    assert information_motion_rank(strong['utility'],18.)<information_motion_rank(weak['utility'],12.238)
