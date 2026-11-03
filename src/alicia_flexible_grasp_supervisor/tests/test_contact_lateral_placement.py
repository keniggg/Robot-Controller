"""Frozen r10 counterexamples and conservative generic planning proposals."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from alicia_flexible_grasp.grasp.contact_lateral_placement import contact_lateral_placement
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk, gripper_target_overlap
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes, _box_corners


def case(index=0):
    data=json.loads((Path(__file__).parent/'fixtures/compensated_contact_terminal_20261023.json').read_text())
    row=data['rows'][index]
    args=dict(failure=row['failure'],plan_id=row['failure']['plan_id'],source_stamp_ns=row['scene']['source_ns'],
        goal_pose=row['poses'][2],support_normal_base=row['scene']['normal'],model_xml=data['model_xml'],
        maximum_translation_m=.01)
    return row,args


@pytest.mark.parametrize('index',range(4))
def test_same_failed_family_proposal_retains_orientation_target_height_and_inputs(index):
    row,args=case(index); before=deepcopy(args)
    proposed,report=contact_lateral_placement(**args)
    assert proposed is not None and report['proposed'] and args==before
    original=pose_matrix(args['goal_pose']);delta=proposed[:3,3]-original[:3,3]
    np.testing.assert_array_equal(proposed[:3,:3],original[:3,:3])
    np.testing.assert_allclose(delta,report['translation_base_m'],atol=1e-17)
    assert abs(np.dot(delta,args['support_normal_base']))<1e-16
    assert 0 < np.linalg.norm(delta) <= args['maximum_translation_m']
    assert not report['motion_authorized'] and not report['target_relocated']
    assert not report['path_verified'] and not report['future_following_verified']


def endpoint_clear(fk,q,scene,gripper,opening):
    transform=fk(q)
    return all(not gripper_target_overlap(fk,gripper,opening,body,transform,center,size,scene)[0]
        and np.min(_box_corners(center,transform[:3,:3],size)@scene.normal+scene.offset)>=gripper.support_clearance_m
        for body,center,size in _stage_boxes(transform,gripper,opening,'y','z'))


@pytest.mark.parametrize('index',range(4))
def test_frozen_terminal_rejections_and_new_exact_ik_endpoint_clearance(index):
    row,args=case(index);fk=SerialUrdfFk(args['model_xml'],ARM_NAMES)
    scene=NS(**{k:np.array(v) if k in ('center','size','normal','rotation') else v for k,v in row['scene'].items()})
    gripper=contact_precheck_gripper(row['gripper_geometry']);opening=row['opening_m']
    sample=args['failure']['source_sample'];bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    command=np.array(args['failure']['contact_terminal_failure']['command_endpoint_rad']);nominal=command-bias
    assert endpoint_clear(fk,nominal,scene,gripper,opening)
    assert not endpoint_clear(fk,command,scene,gripper,opening)
    target,_=contact_lateral_placement(**args)
    def residual(q):
        t=fk(q)
        rotation=target[:3,:3].T@t[:3,:3]
        make=getattr(Rotation,'from_matrix',None) or Rotation.from_dcm
        return np.r_[1000*(t[:3,3]-target[:3,3]),100*make(rotation).as_rotvec()]
    lower=np.maximum(fk.limits[:,0],fk.limits[:,0]-bias)+1e-12
    upper=np.minimum(fk.limits[:,1],fk.limits[:,1]-bias)-1e-12
    solved=least_squares(residual,nominal,bounds=(lower,upper),max_nfev=80,ftol=1e-12,xtol=1e-12,gtol=1e-12)
    assert np.linalg.norm(residual(solved.x))<1e-6
    assert endpoint_clear(fk,solved.x,scene,gripper,opening)
    assert endpoint_clear(fk,solved.x+bias,scene,gripper,opening)
    # These are endpoint checks only; contact_lateral_placement grants no
    # continuous path, future-following, or physical execution authority.
    # A narrower opening or oversized target still has no endpoint authority.
    wide=deepcopy(scene);wide.size=scene.size*2.
    assert not endpoint_clear(fk,solved.x,wide,gripper,opening)
    assert not endpoint_clear(fk,solved.x,scene,gripper,.025)
    if index == 1:
        # These fixed negatives have original unpadded mesh vertices inside
        # the target OBB. This proves validated CAD-union intersection, not
        # active-hardware intersection with the actual physical object.
        transform = fk(solved.x)
        for target_scene, opened in ((wide, opening), (scene, .025)):
            parts = fk.finger_cad_model._body_components('left_finger')
            vertices = np.concatenate([
                part.vertices_at_zero + opened * part.velocity_per_opening
                for part in parts])
            world = vertices @ transform[:3, :3].T + transform[:3, 3]
            local = (world - target_scene.center) @ target_scene.rotation
            margin = target_scene.size / 2. - np.abs(local)
            assert np.any(np.all(margin > 1e-6, axis=1))


@pytest.mark.parametrize('normal',[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[.6,0.,.8]])
@pytest.mark.parametrize('scale',[-1.,.5,1.,1.3])
def test_varied_bias_direction_and_plane_have_same_balancing_rule(normal,scale):
    row,args=case();failure=args['failure'];sample=failure['source_sample']
    bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    q=np.array(failure['contact_terminal_failure']['command_endpoint_rad'])-bias
    sample['sdk_positions_rad']=(np.array(sample['accepted_positions_rad'])+scale*bias).tolist()
    failure['contact_terminal_failure']['command_endpoint_rad']=(q+scale*bias).tolist()
    args['support_normal_base']=normal;args['maximum_translation_m']=.02
    target,report=contact_lateral_placement(**args)
    assert target is not None
    fk=SerialUrdfFk(args['model_xml'],ARM_NAMES);n=np.array(normal)
    r=fk(q+scale*bias)[:3,3]-fk(q)[:3,3]
    np.testing.assert_allclose(report['translation_base_m'],-.5*(r-n*(n@r)),atol=1e-14)
    assert abs(n@np.array(report['translation_base_m']))<1e-15


@pytest.mark.parametrize('fault',['missing_terminal','wrong_plan','wrong_source','wrong_curve','not_terminal',
    'executing','success','wrong_model','wrong_epoch','wrong_names','bias_large','nan_q','bool_q',
    'goal_changed','normal_invalid','zero_bias','boolean_schema'])
def test_unbound_invalid_or_inapplicable_failure_cannot_propose(fault):
    _,args=case();f=args['failure'];t=f['contact_terminal_failure'];s=f['source_sample']
    if fault=='missing_terminal':f.pop('contact_terminal_failure')
    elif fault=='wrong_plan':t['plan_id']='different'
    elif fault=='wrong_source':t['source_stamp_ns']+=1
    elif fault=='wrong_curve':t['curve']='nominal'
    elif fault=='not_terminal':t['endpoint_screen_only']=False
    elif fault=='executing':t['execution_authorized']=True
    elif fault=='success':f['success']=True
    elif fault=='wrong_model':t['model_sha256']='changed'
    elif fault=='wrong_epoch':s['epoch_ns']+=1
    elif fault=='wrong_names':s['joint_names']=list(reversed(s['joint_names']))
    elif fault=='bias_large':s['sdk_positions_rad'][0]=s['accepted_positions_rad'][0]+.036
    elif fault=='nan_q':t['command_endpoint_rad'][0]=float('nan')
    elif fault=='bool_q':t['command_endpoint_rad'][0]=True
    elif fault=='goal_changed':args['goal_pose'][0]+=.001
    elif fault=='normal_invalid':args['support_normal_base']=[0.,0.,2.]
    elif fault=='zero_bias':
        t['command_endpoint_rad']=(np.array(t['command_endpoint_rad'])-np.array(s['sdk_positions_rad'])+s['accepted_positions_rad']).tolist()
        s['sdk_positions_rad']=list(s['accepted_positions_rad'])
    elif fault=='boolean_schema':t['schema_version']=True
    proposed,report=contact_lateral_placement(**args)
    assert proposed is None and not report['proposed'] and not report['motion_authorized']


@pytest.mark.parametrize('bound',[None,True,0.,-.1,float('nan'),float('inf'),.0001])
def test_existing_remaining_translation_bound_is_not_enlarged(bound):
    _,args=case();args['maximum_translation_m']=bound
    assert contact_lateral_placement(**args)[0] is None
