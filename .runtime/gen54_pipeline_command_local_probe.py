"""Frozen GEN53 materialization and candidate open-phase semantic comparison."""
import ast, gzip, hashlib, json, os, platform, signal, struct, sys, time
from pathlib import Path
from dataclasses import asdict, dataclass
from types import SimpleNamespace
from copy import deepcopy
import extract_gen50_retry_r30_rgb_20261018 as boundary

W=Path(__file__).resolve().parents[1]
B=W/'.runtime/gen54_integrated_candidate'
OUT=boundary.N/'verification/gen54_geometry_pipeline_r1/command_local_materializer_r3'

def main():
    assert boundary.E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',TMPDIR=str(OUT))
    sys.dont_write_bytecode=True
    n=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(n)+(n.machine,)))
    platform.architecture=lambda *a,**kw:(str(8*struct.calcsize('P'))+'bit','ELF')
    boundary.OUT=OUT;sys.addaudithook(boundary.audit)
    signal.signal(signal.SIGALRM,lambda *a:(_ for _ in ()).throw(TimeoutError('90 second frozen materializer budget')))
    signal.alarm(90)
    import numpy as np
    sys.path.insert(0,str(B/'src/alicia_flexible_grasp_supervisor/src'))
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface,SurfaceView
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from gen54_pipeline_materializer import create_frozen_node,generate_local
    receipt=json.loads((OUT.parent/'exact_inputs/result.json').read_text())
    ap=Path(receipt['audit']['path']);assert boundary.sha(ap)==receipt['audit']['sha256']
    a=json.loads(ap.read_text());g=a['replay_geometry']
    geometry=GeometryEstimate(True,'','',np.array(g['obb_center_base_m']),np.array(g['R_base_obb']),
        np.array(g['obb_size_xyz_m']),np.array(g['support_normal_base']),g['support_offset_m'],
        g['support_inlier_ratio'],np.array(g['object_points_base_m']),g['source_mode'])
    ref=a['multiview_surface']['fused_scene_geometry']['surface_replay']['references']['snapshot']
    zipped=(Path(ref['store_root'])/ref['relative_path']).read_bytes();raw=gzip.decompress(zipped)
    assert hashlib.sha256(zipped).hexdigest()==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256']
    s=json.loads(raw);identity=TargetTrackIdentity(**s['identity'])
    surface=FusedTargetSurface(identity,np.array(s['points_base']),np.array(s['view_indices']),
        tuple(s['view_stamps_ns']),tuple(tuple(v)for v in s['view_sources']))
    reference=SurfaceView(identity=identity,**{k:v for k,v in s['reference'].items()if k!='identity'})
    node,env=create_frozen_node(B,a,geometry,surface,reference)
    seqfile=B/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_sequence.py'
    tree=ast.parse(seqfile.read_text());names={'make_grasp_sequence_from_grasp_pose','_validated_approach_direction','_offset_along_vector'}
    body=[x for x in tree.body if(isinstance(x,ast.FunctionDef)and x.name in names)or(isinstance(x,ast.ClassDef)and x.name=='Grasp6DPlan')]
    seqenv=dict(np=np,deepcopy=deepcopy,dataclass=dataclass)
    exec(compile(ast.Module(body=body,type_ignores=[]),str(seqfile),'exec'),seqenv)
    ggfile=Path(gg.__file__);gtree=ast.parse(ggfile.read_text())
    function=next(x for x in gtree.body if isinstance(x,ast.FunctionDef)and x.name=='_evaluate_physical_candidate')
    changed=[]
    for statement in function.body:
        if isinstance(statement,ast.Assign):
            target=next((t.id for t in statement.targets if isinstance(t,ast.Name)),None)
            if target in ('endpoint_contact','segment_contact'):
                old=ast.literal_eval(statement.value)
                new=(False,False,False,True) if target=='endpoint_contact' else (False,False,True)
                statement.value=ast.Tuple(elts=[ast.Constant(v)for v in new],ctx=ast.Load())
                changed.append(dict(variable=target,before=old,after=new))
    assert len(changed)==2
    ast.fix_missing_locations(function);gateenv=dict(gg.__dict__)
    exec(compile(ast.Module(body=[function],type_ignores=[]),str(ggfile),'exec'),gateenv)
    candidate_physical=gateenv['_evaluate_physical_candidate'];original_physical=gg._evaluate_physical_candidate
    remote=B/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
    capture_line=next(i+1 for i,l in enumerate(remote.read_text().splitlines())if l.strip()=='max_candidates = int(self.tabletop_geometry_config.max_candidates)')
    captures=[]
    def trace(frame,event,arg):
        if event=='line'and frame.f_code.co_name=='_generate_tabletop_candidates'and frame.f_lineno==capture_line:
            captures.extend(frame.f_locals['materialized'])
        return trace
    outputs=[];began=time.monotonic()
    for label,angles in [('diagnostic_bounded_lower_local_midpoints',[67.1875,67.34375])]:
        captures.clear();sys.settrace(trace)
        bounded,diagnostics=generate_local(node,env,geometry,angles)
        sys.settrace(None)
        all_items=list(captures);assert len(all_items)==diagnostics.get('materialized_total_count',0)
        if label=='original':
            print(json.dumps(_plain(dict(baseline_materialized_count=diagnostics['materialized_total_count'],bounded=len(bounded),
                profiles=diagnostics['adaptive_stage_profiles'],rejections=diagnostics['rejection_counts'])),ensure_ascii=False),flush=True)
            original_counts_verified=(diagnostics['materialized_total_count']==44 and len(bounded)==32
                and diagnostics['rejection_counts']==a['remote_diagnostics']['tabletop_geometry']['rejection_counts'])
        results=[]
        for item in all_items:
            T=np.array(item.T_base_tool0);p=dict(item.audit['adaptive_stage_profile'])
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=T[0,3],y=T[1,3],z=T[2,3])))
            sequence=seqenv['make_grasp_sequence_from_grasp_pose'](pose,
                pregrasp_distance_m=p['pregrasp_distance_m'],approach_offset_m=p['approach_offset_m'],
                lift_height_m=p['lift_height_m'],approach_direction_base=item.insertion_axis_base,
                pregrasp_direction_base=-geometry.support_normal_base,lift_direction_base=geometry.support_normal_base)
            stages={}
            for name in ('pregrasp','approach','grasp','lift'):
                pos=getattr(sequence,name).pose.position;matrix=T.copy();matrix[:3,3]=[pos.x,pos.y,pos.z];stages[name]=matrix
            _,evidence,bounds,axis=node._current_bilateral_surface_measurement(
                item.contact_center_base,T[:3,:3],surface_snapshot=(surface,reference))
            kwargs=dict(gripper=node.gripper_geometry,candidate_center_base=item.contact_center_base,
                candidate_tool0_base=T[:3,3],R_base_tool=T[:3,:3],required_open_width_m=item.required_open_width_m,
                target_points_base=surface.points_base,obb_center_base=geometry.center_base,
                R_base_obb=geometry.axes_base,obb_size_xyz_m=geometry.size_xyz_m,
                support_normal_base=geometry.support_normal_base,support_offset_m=geometry.support_offset_m,
                pregrasp_T_base_tool=stages['pregrasp'],approach_T_base_tool=stages['approach'],
                grasp_T_base_tool=stages['grasp'],lift_T_base_tool=stages['lift'],motion_cost=0.,
                tool_jaw_axis='y',tool_finger_length_axis='z',opening_fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m,
                width_projection_trim_fraction=node.tabletop_geometry_config.width_projection_trim_fraction,
                contact_band_fraction=node.tabletop_geometry_config.contact_band_fraction,
                bilateral_surface_evidence=evidence,contact_height_bounds_m=bounds,
                contact_height_axis_base_override=axis,minimum_contact_patch_overlap_m=p['contact_overlap_requirement_m'])
            original=gg.evaluate_explicit_candidate(**kwargs)
            gg._evaluate_physical_candidate=candidate_physical
            try:changed_result=gg.evaluate_explicit_candidate(**kwargs)
            finally:gg._evaluate_physical_candidate=original_physical
            results.append(dict(source_index=item.source_index,variant_index=item.variant_index,
                angle_deg=item.audit['sampled_angle_deg'],tilt_deg=item.audit['approach_tilt_deg'],
                tilt_polarity=item.audit['approach_tilt_polarity'],required_open_width_m=item.required_open_width_m,
                contact_bounds_m=bounds,overlap_m=item.audit['contact_patch_overlap_m'],
                original=asdict(original),open_phase_candidate=asdict(changed_result),
                T_base_tool0=T.tolist(),stage_matrices={k:v.tolist()for k,v in stages.items()}))
        outputs.append(dict(label=label,requested_local_angles_deg=angles,
            materialized_count=len(all_items),bounded_count=len(bounded),
            bounded_identities=[dict(source_index=i.source_index,variant_index=i.variant_index,
                angle_deg=i.audit['sampled_angle_deg'],tilt_deg=i.audit['approach_tilt_deg'],T_base_tool0=i.T_base_tool0) for i in bounded],
            candidate_analytic_pass_count=sum(r['original']['ok']for r in results),
            candidate_repeat_analytic_pass_count=sum(r['open_phase_candidate']['ok']for r in results),
            diagnostics=diagnostics,rows=results))
    result=dict(passed=True,diagnostic_only=True,execution_authorized=False,real_grasp_success=False,
        baseline=str(B),source_audit_sha256=boundary.sha(ap),source_surface_reference=ref,
        exact_production_materialization_replayed=True, baseline_original_44_32_verified_separately=True,original_counts_verified=False,
        semantic_counterfactual=changed,physical_gap_m=node.gripper_geometry.max_inner_gap_m,
        strict_accepted_opening_replayed=False,final_registered_height_recenter_replayed=False,
        full_robot_path_or_IK_replayed=False,production_or_record_modified=False,
        boundary_attempts=boundary.BOUNDARY,ROS_initialized=False,hardware_contacted=False,
        elapsed_sec=time.monotonic()-began,outputs=outputs,
        helper_sha256=boundary.sha(__file__),factory_sha256=boundary.sha(W/'.runtime/gen54_pipeline_materializer.py'),
        gripper_source_sha256=boundary.sha(ggfile),remote_source_sha256=boundary.sha(remote))
    assert not boundary.BOUNDARY
    (OUT/'result.json').write_text(json.dumps(_plain(result),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(result=str(OUT/'result.json'),summary=[{k:r[k]for k in ('label','materialized_count','bounded_count','candidate_analytic_pass_count','candidate_repeat_analytic_pass_count')}for r in outputs]),ensure_ascii=False))
    signal.alarm(0)

if __name__=='__main__':main()
