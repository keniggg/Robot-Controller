"""Original geometric collision callback on six frozen poses; no IK claim."""
import ast
from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
from types import SimpleNamespace

def run(baseline, geometry, snapshot_surface, model, artifact, observation_goal):
    import numpy as np
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry
    sequence_source=baseline/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/grasp6d_sequence.py'
    source_tree=ast.parse(sequence_source.read_text())
    # Execute the exact production sequence builder using data-only position
    # carriers and its explicit direction branch. No ROS or quaternion/IK
    # substitute is installed; all stage rotations remain the supplied matrix.
    selected={'make_grasp_sequence_from_grasp_pose','_validated_approach_direction','_offset_along_vector'}
    body=[x for x in source_tree.body if(isinstance(x,ast.FunctionDef)and x.name in selected)
        or(isinstance(x,ast.ClassDef)and x.name=='Grasp6DPlan')]
    env=dict(np=np,deepcopy=deepcopy,dataclass=dataclass)
    exec(compile(ast.Module(body=body,type_ignores=[]),str(sequence_source),'exec'),env)
    planning,enclosure=enclose_inferred_box_geometry(geometry,model)
    np.testing.assert_allclose(planning.size_xyz_m,artifact['planning_size_xyz_m'],rtol=0,atol=1e-15)
    gd=dict(artifact['finger_geometry'])
    gd['finger_size_xyz_m']=np.asarray(gd['finger_size_xyz_m']);gd['palm_size_xyz_m']=np.asarray(gd['palm_size_xyz_m'])
    gripper=gg.GripperGeometry(**gd)
    normal=np.asarray(planning.support_normal_base)
    results=[]
    for i,row in enumerate(artifact['materialized_contact_candidates']):
        T=np.asarray(row['T_base_tool0']);tilt=row['placement_audit']['approach_tilt_deg']
        profile=next(p for p in artifact['adaptive_profiles']if abs(p['tilt_deg']-tilt)<1e-9)
        pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=T[0,3],y=T[1,3],z=T[2,3])))
        seq=env['make_grasp_sequence_from_grasp_pose'](pose,
            pregrasp_distance_m=profile['pregrasp_distance_m'],approach_offset_m=profile['approach_offset_m'],
            lift_height_m=profile['lift_height_m'],approach_direction_base=row['insertion_axis_base'],
            pregrasp_direction_base=-normal,lift_direction_base=normal)
        stages={}
        for name in ('pregrasp','approach','grasp','lift'):
            p=getattr(seq,name).pose.position;stage=T.copy();stage[:3,3]=[p.x,p.y,p.z];stages[name]=stage
        evidence=gg.evaluate_inferred_box_contact(model,row['contact_center_base'],row['jaw_axis_base'],
            row['insertion_axis_base'],gripper,support_normal_base=model.support_normal_base,
            opening_fit_clearance_each_side_m=artifact['opening_fit_clearance_each_side_m'])
        assert evidence.ok
        gate=gg.evaluate_explicit_candidate(gripper=gripper,candidate_center_base=row['contact_center_base'],
            candidate_tool0_base=T[:3,3],R_base_tool=T[:3,:3],required_open_width_m=row['required_open_width_m'],
            target_points_base=snapshot_surface.points_base,obb_center_base=planning.center_base,
            R_base_obb=planning.axes_base,obb_size_xyz_m=planning.size_xyz_m,
            support_normal_base=normal,support_offset_m=planning.support_offset_m,
            pregrasp_T_base_tool=stages['pregrasp'],approach_T_base_tool=stages['approach'],
            grasp_T_base_tool=stages['grasp'],lift_T_base_tool=stages['lift'],
            tool_jaw_axis='y',tool_finger_length_axis='z',motion_cost=0.,
            opening_fit_clearance_each_side_m=artifact['opening_fit_clearance_each_side_m'],
            width_projection_trim_fraction=artifact['tabletop_config']['width_projection_trim_fraction'],
            contact_band_fraction=artifact['tabletop_config']['contact_band_fraction'],
            bilateral_surface_evidence=evidence,contact_height_bounds_m=row['contact_height_bounds_m'],
            contact_height_axis_base_override=row['contact_height_axis_base'],
            minimum_contact_patch_overlap_m=row['required_contact_overlap_m'])
        clearance={}
        for stage_name,stage in stages.items():
            boxes={}
            for name,center,size in gg._stage_boxes(stage,gripper,gripper.max_inner_gap_m,'y','z'):
                corners=gg._box_corners(center,stage[:3,:3],size)
                boxes[name]=float(np.min(corners@normal+planning.support_offset_m))
            clearance[stage_name]=boxes
        delta=np.asarray(observation_goal['fixed_goal_pose'][:3])-stages['pregrasp'][:3,3]
        normal_delta=float(delta@normal);tangent=delta-normal_delta*normal
        results.append(dict(artifact_index=i,proposal_index=row['proposal_index'],source_index=row['source_index'],
            variant_index=row['variant_index'],tilt_deg=tilt,pose_sha256=hashlib.sha256(T.astype('<f8').tobytes()).hexdigest(),
            stage_matrices={k:v.tolist()for k,v in stages.items()},stage_support_clearance_m=clearance,
            analytical_gate=asdict(gate),observation_planned_goal_minus_new_pregrasp=dict(
                delta_base_m=delta.tolist(),normal_component_m=normal_delta,
                tangent_norm_m=float(np.linalg.norm(tangent)),measured_TCP_comparison=False),
            required_open_width_m=row['required_open_width_m'],downstream_motion_authorized=False))
    return dict(rows=results,analytical_pass_count=sum(r['analytical_gate']['ok']for r in results),
        analytical_failure_count=sum(not r['analytical_gate']['ok']for r in results),
        checks='Real evaluate_explicit_candidate including original endpoint and interpolated gripper/support/target envelope checks on the generated ideal Cartesian stages.',
        complete_robot_or_controller_path_checked=False,IK_called=False,IK_mocked=False,
        command_or_predicted_actual_joint_curve_available=False,
        geometry_sha256=model.source['geometry_sha256'],model_sha256=model.model_sha256,
        sequence_source_sha256=hashlib.sha256(sequence_source.read_bytes()).hexdigest(),
        gripper_source_sha256=hashlib.sha256((baseline/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py').read_bytes()).hexdigest(),
        missing_for_full_path=['No R18 CONTACT plan was accepted; historical strict service deliveries are zero.',
            'No new-candidate actual IK solution or time-parameterized command trajectory exists in the original record.',
            'No future joint tracking/residual envelope or current full MoveIt collision world is proven by this diagnostic.',
            'Historical observation fixed_goal_pose is a planned target, not an archived measured TCP sample.'],
        preserved_inputs='All six candidate transforms, source model and complete geometric families remain unchanged.')
