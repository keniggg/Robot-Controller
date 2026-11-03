"""Same frozen target, diagnostic lateral placement only; no path authority."""
from pathlib import Path
from types import SimpleNamespace as NS
import os, json, gzip, time, hashlib
from dataclasses import asdict
W=Path(__file__).resolve().parents[1]
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
D=R/'supplemental_view_r10/contact_diagnosis'
assert Path('/media/zhuyupei/SanStand').is_mount()
os.environ['OFFLINE_BOUNDARY_LOG']=str(D/'lateral_probe_boundary.jsonl')
exec((R/'guard.py').read_bytes())
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk, gripper_target_overlap
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.grasp.gripper_geometry import _stage_boxes, _box_corners
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.gripper_geometry import evaluate_explicit_candidate, evaluate_bilateral_surface_evidence_and_bounds
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

index=json.loads((D/'failed_prechecks.json').read_text());rows=[];started=time.monotonic()
audit=json.loads(Path(index['audit_path']).read_text())
ref=audit['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['reference']
raw=gzip.decompress((Path(ref['store_root'])/ref['relative_path']).read_bytes())
assert hashlib.sha256(raw).hexdigest()==ref['object_sha256']
s=json.loads(raw);surface=FusedTargetSurface(TargetTrackIdentity(**s['identity']),np.array(s['points_base']),
    np.array(s['view_indices']),tuple(s['view_stamps_ns']),view_sources=tuple(tuple(v) for v in s['view_sources']))
for row in index['rows']:
    ref=row['contact_sequence_precheck']['contact_failure_replay']['reference']
    raw=gzip.decompress((Path(ref['store_root'])/ref['relative_path']).read_bytes())
    assert hashlib.sha256(raw).hexdigest()==ref['object_sha256']
    p=json.loads(raw);fk=SerialUrdfFk(p['model']['xml'],ARM_NAMES)
    g=contact_precheck_gripper(p['config']['gripper_geometry']);opening=p['opening_m']
    scene=NS(**{k:np.array(v) if k in ('center','size','normal','rotation') else v for k,v in p['scene'].items()})
    bias=np.array(p['following_sample']['sdk_positions_rad'])-p['following_sample']['accepted_positions_rad']
    qc=np.array(p['first_failed_sample']['joint_positions_rad']);qn=qc-bias
    goal=pose_matrix(p['plan_data']['poses_xyz_xyzw'][2]);command=fk(qc)
    displacement=-(command[:3,3]-goal[:3,3]);displacement-=scene.normal*(scene.normal@displacement)
    e=json.loads(p['plan_data']['contact_geometry_evidence'])
    bilateral,bounds=evaluate_bilateral_surface_evidence_and_bounds(surface,e['contact_center_base'],
        goal[:3,1],goal[:3,2],g,support_normal_base=s['reference']['support_normal_base'])
    assert bilateral.ok
    np.testing.assert_allclose(bounds,e['contact_height_bounds_m'],rtol=0,atol=1e-12)
    geom=dict(gripper=g,contact_center_base=e['contact_center_base'],
        contact_height_axis_base=e['contact_height_axis_base'],contact_height_bounds_m=e['contact_height_bounds_m'],
        minimum_contact_overlap_m=e['minimum_contact_overlap_m'],support_normal_base=scene.normal,
        support_offset_m=scene.offset,opening_width_m=opening)
    limits=np.column_stack((np.maximum(fk.limits[:,0],fk.limits[:,0]-bias),np.minimum(fk.limits[:,1],fk.limits[:,1]-bias)))
    trials=[]
    for factor in (0.,.25,.5,.75,1.):
        target=goal.copy();target[:3,3]+=factor*displacement
        def residual(q):
            t=fk(q);return np.r_[1000*(t[:3,3]-target[:3,3]),100*Rotation.from_dcm(target[:3,:3].T@t[:3,:3]).as_rotvec()]
        z=least_squares(residual,qn,bounds=(limits[:,0]+1e-12,limits[:,1]-1e-12),max_nfev=80,ftol=1e-12,xtol=1e-12,gtol=1e-12)
        checks=[]
        for label,q in [('nominal',z.x),('command',z.x+bias)]:
            t=fk(q)
            for body,c,size in _stage_boxes(t,g,opening,'y','z'):
                collision,proof=gripper_target_overlap(fk,g,opening,body,t,c,size,scene)
                support=float(np.min(_box_corners(c,t[:3,:3],size)@scene.normal+scene.offset))
                checks.append(dict(curve=label,body=body,overlap=bool(collision),support_clearance_m=support,gap_m=proof.get('gap_m') if proof else None))
        height=evaluate_contact_height_margin(translation_evidence_limit_m=.001,actual_T_base_tool0=target,planned_T_base_tool0=target,**geom)
        stages=[pose_matrix(v) for v in p['plan_data']['poses_xyz_xyzw']]
        for t in stages:t[:3,3]+=factor*displacement
        gate=evaluate_explicit_candidate(gripper=g,candidate_center_base=e['contact_center_base'],
            candidate_tool0_base=target[:3,3],R_base_tool=target[:3,:3],required_open_width_m=e['required_open_width_m'],
            target_points_base=surface.points_base,obb_center_base=scene.center,R_base_obb=scene.rotation,
            obb_size_xyz_m=scene.size,support_normal_base=scene.normal,support_offset_m=scene.offset,
            pregrasp_T_base_tool=stages[0],approach_T_base_tool=stages[1],grasp_T_base_tool=stages[2],lift_T_base_tool=stages[3],
            opening_fit_clearance_each_side_m=e['opening_fit_clearance_each_side_m'],
            contact_height_bounds_m=bounds,bilateral_surface_evidence=bilateral,contact_height_axis_base_override=goal[:3,2],
            minimum_contact_patch_overlap_m=e['minimum_contact_overlap_m'])
        trial=dict(factor=factor,translation_base_m=(factor*displacement).tolist(),error_norm=float(np.linalg.norm(residual(z.x))),
            endpoint_clear=all(not x['overlap'] and x['support_clearance_m']>=g.support_clearance_m for x in checks),
            original_height_band_margin_ok=height.get('ok'),geometry=checks,nominal_q=z.x.tolist(),height=height,full_nominal_geometry=asdict(gate))
        trials.append(trial)
        print(json.dumps(dict(candidate=row['source_index'],factor=factor,translation_mm=(1000*factor*displacement).tolist(),clear=trial['endpoint_clear'],height=height.get('ok'),geometry_ok=gate.ok,geometry_reason=gate.failure_reason,ik_error=trial['error_norm'])),flush=True)
    rows.append(dict(source_index=row['source_index'],reference=ref,proposals=trials))
(D/'lateral_placement_probe.json').write_text(json.dumps(dict(diagnostic_only=True,execution_authorized=False,
    original_geometry_gate_rechecked=True,full_path_verified=False,elapsed_sec=time.monotonic()-started,rows=rows),indent=2)+'\n')
