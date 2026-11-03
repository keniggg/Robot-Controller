"""Reconstructed original-materializer family; finite IK endpoint proposals."""
from pathlib import Path
from dataclasses import dataclass,replace
from types import MappingProxyType
from typing import Mapping
import ast,gzip,hashlib,json,math,os,time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from probe_gen52_g51_full_collision_curves_20261018 import load
from review_gen51_g50_strict_records_20261018 import pose_matrix
from review_gen52_g51_contact_collisions_20261018 import sat
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1/reconstructed_endpoint_family_r3'
ROOT=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
    with p.open('x') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    assert json.loads(p.read_text())==json.loads(json.dumps(v,ensure_ascii=False,allow_nan=False))
def main():
    assert os.path.ismount('/media/zhuyupei/SanStand');D.mkdir(exist_ok=False)
    env=dict(__name__='__main__',np=np,math=math,time=time,hashlib=hashlib,ET=ET,dataclass=dataclass,ObservationPathError=RuntimeError)
    geom=ROOT/'grasp/gripper_geometry.py';mat=ROOT/'grasp/tabletop_geometry_candidates.py';guard=ROOT/'robot/observation_path_guard.py'
    for n in ast.parse(geom.read_text()).body:
        if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id.startswith('ANALYTICAL_') for t in n.targets):
            exec(compile(ast.Module(body=[n],type_ignores=[]),str(geom),'exec'),env)
    load(geom,{'_readonly_vector','_finite_number','GripperGeometry','parse_tool_axis','_validated_rotation','gripper_box_centers','_stage_boxes','_box_corners','_obb_overlap','_cross_2d','_signed_polygon_margin','finger_contact_patch_height_intervals_m','finger_contact_patch_overlap_m'},env)
    load(guard,{'_array','_axis_rotation','SerialUrdfFk'},env)
    table=dict(env,MappingProxyType=MappingProxyType,Mapping=Mapping,replace=replace,_MAX_MATERIALIZED_APPROACH_TILTS=8)
    load(mat,{'_InputInvalid','TabletopCandidateContractError','TabletopProposal','TabletopCandidate','_frozen_array','materialize_tabletop_candidates','_approach_insertion_variants','semantic_axes_to_tool_rotation','solve_tool0_translation_for_support_clearance','_candidate_unit_axis','_open_finger_corners','_validated_approach_tilt_degrees','_finite_vector','_unit_vector'},table)
    origin=V/'gen51_actual_handoff_monitor_r1/terminal_and_completed_diagnostics.json';ref=json.loads(origin.read_text())['unique_failure_replay_refs'][0]['reference'];fp=Path(ref['store_root'])/ref['relative_path'];raw=gzip.decompress(fp.read_bytes())
    assert sha(fp)==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256'];d=json.loads(raw)
    original=json.loads((V/'gen51_actual_handoff_monitor_r1/first_strict_contact_evidence_original_decoded.json').read_text())['decoded_original_contact_geometry_evidence']
    box=original['box_model_evidence'];normal=np.array(box['source_support_normal_base']);support=-box['source_support_offset_m']*normal;registration=np.array(box['registration_transform'])
    original_t=pose_matrix(d['plan_data']['poses_xyz_xyzw'][2]);source_t=np.linalg.inv(registration)@original_t;jaw=source_t[:3,1];tilt=math.degrees(math.acos(float(np.clip(-source_t[:3,2]@normal,-1,1))))
    g=env['GripperGeometry'](**{k:np.array(v) if isinstance(v,list) else v for k,v in d['config']['evaluated_gripper'].items()})
    proposal=table['TabletopProposal'](source_index=0,contact_center_base=np.array(box['source_center_base']),insertion_axis_base=-normal,jaw_axis_base=jaw,
        required_open_width_m=original['required_open_width_m'],aperture_margin_m=d['opening_m']-original['required_open_width_m'],negative_contact_count=0,positive_contact_count=0,contact_symmetry=0.,source_score=0.,angle_deg=0.,audit={})
    candidates=table['materialize_tabletop_candidates'](proposal,support,normal,g,approach_tilt_degrees=(tilt,))
    fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names']);sample=d['following_sample'];bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    qoriginal=np.array(d['checked_curve']['points'][-1]['positions'])-bias
    low=np.maximum.reduce([fk.limits[:,0],fk.limits[:,0]-bias,np.full(6,-math.pi)-bias]);high=np.minimum.reduce([fk.limits[:,1],fk.limits[:,1]-bias,np.full(6,math.pi-2*math.pi/4096)-bias])
    seeds=[qoriginal]
    for sign in (-1,1):
        q=qoriginal.copy();q[5]+=sign*math.pi;seeds.append(np.clip(q,low+1e-8,high-1e-8))
    def state(t):
        outputs=[];physical_normal=np.array(d['scene']['normal'])
        for body,center,size in env['_stage_boxes'](t,g,d['opening_m'],'y','z'):
            sep=sat(center,t[:3,:3],size,np.array(d['scene']['center']),np.array(d['scene']['rotation']),np.array(d['scene']['size']))
            clearance=float(np.min(env['_box_corners'](center,t[:3,:3],size)@physical_normal+d['scene']['offset']))
            outputs.append(dict(body=body,separation_m=sep['separation_m'],support_clearance_m=clearance,collision=bool(env['_obb_overlap'](center,t[:3,:3],size,np.array(d['scene']['center']),np.array(d['scene']['rotation']),np.array(d['scene']['size'])))))
        return outputs
    rows=[]
    for i,c in enumerate(candidates):
        target=registration@np.array(c.T_base_tool0);overlap=env['finger_contact_patch_overlap_m'](np.array(original['contact_center_base']),target[:3,3],target[:3,:3],np.array(original['contact_height_axis_base']),*original['contact_height_bounds_m'])
        row=dict(index=i,audit=dict(c.audit),goal_T_base_tool0=target.tolist(),contact_overlap_m=overlap,trials=[],reconstructed_not_original_materialized=True)
        if overlap+1e-9<original['minimum_contact_overlap_m']:
            row.update(eligible=False,reason='original minimum contact overlap rejects this reconstructed placement');rows.append(row);continue
        row['eligible']=True
        for si,seed in enumerate(seeds):
            start=time.monotonic()
            def residual(q):
                t=fk(q);return np.r_[(t[:3,3]-target[:3,3])/.0005,((target[:3,:3].T@t[:3,:3])-np.eye(3)).ravel()/.005]
            r=least_squares(residual,np.clip(seed,low+1e-8,high-1e-8),bounds=(low,high),max_nfev=120,ftol=1e-10,xtol=1e-10,gtol=1e-10)
            t=fk(r.x);pe=float(np.linalg.norm(t[:3,3]-target[:3,3]));ae=math.acos(float(np.clip((np.trace(target[:3,:3].T@t[:3,:3])-1)/2,-1,1)))
            trial=dict(seed_index=si,elapsed_sec=time.monotonic()-start,nfev=r.nfev,solver_success=bool(r.success),position_error_m=pe,orientation_error_rad=ae,q_actual=r.x.tolist(),q_command=(r.x+bias).tolist())
            trial['goal_qualified']=pe<=.0005 and ae<=.005
            if trial['goal_qualified']:
                trial.update(actual=state(t),command=state(fk(r.x+bias)))
                trial['dual_endpoint_safe']=all(not z['collision'] and z['support_clearance_m']>=.003-1e-9 for z in trial['actual']+trial['command'])
            row['trials'].append(trial)
        row['dual_endpoint_safe_found']=any(t.get('dual_endpoint_safe',False) for t in row['trials']);rows.append(row)
        write(D/('candidate_%02d.json'%i),row)
        print(json.dumps(dict(index=i,tilt=c.audit['approach_tilt_deg'],polarity=c.audit['approach_tilt_polarity'],jaw=c.variant_index,overlap_mm=overlap*1000,safe=row['dual_endpoint_safe_found'],elapsed=sum(t['elapsed_sec'] for t in row['trials']))),flush=True)
    result=dict(kind='GEN52_G51_reconstructed_original_materializer_six_branch_endpoint_probe',rows=rows,
      same_original_model=True,original_tilt_deg=tilt,source_sha256={str(p):sha(p) for p in [geom,mat,guard,origin,fp,Path(__file__)]},
      dual_endpoint_safe_found=sum(r.get('dual_endpoint_safe_found',False) for r in rows),full_path_checked=False,all_ik_branches_exhausted=False,
      limits=['The model/bias/physical scene/known tilt/registration are original. Inverse-register original goal to source frame, run unchanged source-frame materializer, apply original registration to each full pose, then check fixed original physical plane. No jaw projection is used. Six candidates are reconstructed, not claimed as original R22 list; original row IDs and final recenter placement audit are unavailable.',
      'Only original scalar minimum contact overlap is screened here; complete registered geometry/source validators and all 4 stage planning remain required.',
      'Three bounded numerical IK starts per eligible endpoint; solver flags grant no qualification, exact FK .5mm/.005rad and original SAT/support decide endpoints.',
      'No MoveIt, future feedback, complete trajectory, actual servo response or motion authority is asserted.'],
      production_or_tests_modified=False,ROS_or_hardware_called=False,temperature_read=False)
    write(D/'result.json',result);print(json.dumps(dict(result=str(D/'result.json'),sha256=sha(D/'result.json'),safe=result['dual_endpoint_safe_found'])))
if __name__=='__main__':main()
