#!/usr/bin/env python3
"""Sealed contact band, analytic family and continuous gripper-path replay."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import numpy as np
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import evaluate_bilateral_surface_evidence_and_bounds, evaluate_explicit_candidate
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import contact_height_window_proposals
from alicia_flexible_grasp.grasp.measured_contact import evaluate_measured_contact
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, validate_observation_trajectory

spec=importlib.util.spec_from_file_location('gen15quantifier',Path(__file__).with_name('quantify_gen15_contact_prediction.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
d=json.loads(m.AUDIT.read_text());prior=json.loads((m.OUT/'endpoint_quantification.json').read_text())
npz=np.load(m.OUT/'fused_reconstruction.npz');meta=json.loads((m.OUT/'fused_reconstruction.json').read_text())
assert meta['last_transform_max_error']==0 and meta['actual_point_count']==meta['expected_point_count'] and meta['actual_view_point_counts']==meta['expected_view_point_counts']
G=d['replay_geometry'];REG=np.array(prior['registration']);N=np.array(G['support_normal_base']);OFFSET=G['support_offset_m'];C=REG[:3,:3]@G['obb_center_base_m']+REG[:3,3];R=REG[:3,:3]@G['R_base_obb'];S=np.array(G['obb_size_xyz_m'])
GRIPPER=m.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
FK=m.SerialUrdfFk(m.MODEL.read_text(),tuple('Joint%d'%i for i in range(1,7)))
surface=FusedTargetSurface(TargetTrackIdentity(d['target_epoch'],d['observe_return']['target_track_id']),npz['reference_points'],npz['view_indices'],tuple(meta['sources']))
scene=FrozenObservationScene('offline_gen15_counterfactual',hashlib.sha256(m.AUDIT.read_bytes()).hexdigest(),d['snapshot_stamp_ns'],N,OFFSET,C,R,S)

def matrix(stage):
    t=quaternion_matrix(stage['quaternion_xyzw']);t[:3,3]=stage['position_m'];return t

def solve(T,seed):
    def residual(q):
        pose=FK(q);return np.r_[pose[:3,3]-T[:3,3],.2*Rotation.from_dcm(T[:3,:3].T@pose[:3,:3]).as_rotvec()]
    fit=least_squares(residual,seed,bounds=(FK.limits[:,0],FK.limits[:,1]),max_nfev=100,xtol=1e-12,ftol=1e-12,gtol=1e-12)
    if np.linalg.norm(residual(fit.x))>1e-7:raise ValueError('offline IK failed')
    return fit.x

def path(first,last,seed,bias):
    # Explicit reconstructed smooth Cartesian line. The failed MoveIt curve was
    # not retained; these are independent IK waypoints, not an original replay.
    count=max(2,int(math.ceil(np.linalg.norm(last[:3,3]-first[:3,3])/.003))+1)
    nominal=RobotTrajectory();nominal.joint_trajectory.joint_names=list(FK.names)
    q=seed
    for i,a in enumerate(np.linspace(0.,1.,count)):
        T=first.copy();T[:3,3]=(1.-a)*first[:3,3]+a*last[:3,3];q=solve(T,q)
        nominal.joint_trajectory.points.append(JointTrajectoryPoint(positions=q.tolist(),velocities=[0.]*6,accelerations=[0.]*6,time_from_start=rospy.Duration(i)))
    shifted=deepcopy(nominal)
    for p in shifted.joint_trajectory.points:p.positions=(np.array(p.positions)+bias).tolist()
    result={}
    for name,tr in (('nominal',nominal),('predicted_command',shifted)):
        try:
            result[name]=dict(ok=True,audit=validate_observation_trajectory(tr,scene,FK,GRIPPER,.04975,tr.joint_trajectory.points[0]))
        except ValueError as exc:result[name]=dict(ok=False,reason=str(exc))
    return result

rows=[]
for item in prior['rows'][:2]:
    entry=d['stable_evaluations'][item['index']];binding=entry['lineage_binding']
    raw=next(r for r in d['rows'] if r.get('source_index')==binding['source_raw_candidate_index'] and r.get('variant_index')==binding['source_variant_index'] and r['candidate_source']=='tabletop_geometry')
    center=REG[:3,:3]@raw['contact_center_base_m']+REG[:3,3]
    Ts=[REG@matrix(s) for s in entry['execution_sequence']['stages']]
    evidence,bounds=evaluate_bilateral_surface_evidence_and_bounds(surface,center,Ts[2][:3,1],Ts[2][:3,2],GRIPPER,minimum_points_per_side=12,support_normal_base=npz['support_normal'])
    geo=dict(gripper=GRIPPER,contact_center_base=center,contact_height_axis_base=Ts[2][:3,2],contact_height_bounds_m=bounds,minimum_contact_overlap_m=.002,support_normal_base=N,support_offset_m=OFFSET,opening_width_m=.05)
    original=evaluate_contact_height_margin(translation_evidence_limit_m=entry['final_contact_height_margin']['translation_evidence_limit_m'],actual_T_base_tool0=Ts[2],planned_T_base_tool0=Ts[2],**geo)
    recorded=entry['final_contact_height_margin']
    match=dict(nominal_overlap_error_m=abs(original['nominal']['actual_overlap_m']-recorded['nominal']['actual_overlap_m']),allowed_interval_max_error_m=float(np.max(abs(np.array(original['allowed_normal_translation_intervals_m'])-recorded['allowed_normal_translation_intervals_m']))))
    assert evidence.ok and max(match.values())<1e-9,(evidence,match)
    trials=[]
    production_proposals=contact_height_window_proposals(translation_evidence_limit_m=recorded['translation_evidence_limit_m'],actual_T_base_tool0=Ts[2],planned_T_base_tool0=Ts[2],**geo)
    for delta in [0.]+[p[1]['normal_translation_m'] for p in production_proposals]:
        family=[t.copy() for t in Ts]
        for t in family:t[:3,3]+=delta*N
        height=evaluate_contact_height_margin(translation_evidence_limit_m=recorded['translation_evidence_limit_m'],actual_T_base_tool0=family[2],planned_T_base_tool0=family[2],**geo)
        gate=evaluate_explicit_candidate(gripper=GRIPPER,candidate_center_base=center,candidate_tool0_base=family[2][:3,3],R_base_tool=family[2][:3,:3],required_open_width_m=raw['required_open_width_m'],target_points_base=surface.points_base,obb_center_base=C,R_base_obb=R,obb_size_xyz_m=S,support_normal_base=N,support_offset_m=OFFSET,pregrasp_T_base_tool=family[0],approach_T_base_tool=family[1],grasp_T_base_tool=family[2],lift_T_base_tool=family[3],opening_fit_clearance_each_side_m=.0005,contact_height_bounds_m=bounds,bilateral_surface_evidence=evidence,contact_height_axis_base_override=family[2][:3,2],minimum_contact_patch_overlap_m=.002)
        curves=path(family[1],family[2],np.array(item['offline_ik']['q']),np.array(item['bias_sdk_minus_encoder_rad']))
        trials.append(dict(normal_translation_m=delta,height=height,full_four_stage_analytic_geometry=asdict(gate),reconstructed_contact_curve=curves,T_base_tool0=[t.tolist() for t in family],conservative_registration_plus_placement_displacement_m=d['multiview_surface']['latest_registration']['translation_m']+delta))
    rows.append(dict(index=item['index'],source_index=item['source_index'],variant_index=item['variant_index'],bilateral_evidence=asdict(evidence),contact_center_base=center.tolist(),bounds_m=bounds,original_height_match=match,trials=trials))
out=m.OUT/'production_height_full_geometry_and_curve.json';result=dict(rows=rows,model_sha256=FK.model_sha256,source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),fused_reconstruction_exact_to_available_audit_checks=True,limitations=['Curve is newly reconstructed from sealed goals using offline IK and explicit zero-velocity/acceleration waypoints; actual failed MoveIt path is unavailable.','Original continuous gripper geometry checker is used, but no MoveIt whole-arm scene check or executable plan issued.','Fused representative points/view indices reconstructed exactly by registration/pointcount/viewcount; source-set provenance not rebuilt in this consumer and does not enter these bilateral contact geometry functions.'])
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(result=str(out),rows=[dict(index=r['index'],match=r['original_height_match'],trials=[dict(delta=t['normal_translation_m'],height=t['height']['ok'],geometry=t['full_four_stage_analytic_geometry']['ok'],curves={k:v['ok'] for k,v in t['reconstructed_contact_curve'].items()}) for t in r['trials']]) for r in rows]),indent=2))
