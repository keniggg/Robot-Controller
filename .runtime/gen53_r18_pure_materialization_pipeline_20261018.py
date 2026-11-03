"""Frozen R18 pure production candidate/materialization callbacks, no services."""
import ast
from collections import Counter
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import math
from pathlib import Path
from types import MappingProxyType

def materialize(w, baseline, original_audit, geometry, snapshot_surface,
                reference_view, model):
    import numpy as np
    import yaml
    from alicia_flexible_grasp.vision.regular_box_inference import enclose_inferred_box_geometry
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
    from alicia_flexible_grasp.grasp.hybrid_grasp_candidates import MergeConfig
    from alicia_flexible_grasp.grasp.adaptive_stage_profiles import AdaptiveStageLimits, derive_adaptive_stage_profiles
    from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
    source=baseline/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
    tree=ast.parse(source.read_text())
    candidate_source=baseline/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/remote_grasp6d_client.py'
    # Extract only the original structured exception; importing this module
    # would unnecessarily import ROS TF for unrelated wire decoding functions.
    exception=next(n for n in ast.parse(candidate_source.read_text()).body
                   if isinstance(n,ast.ClassDef)and n.name=='CandidateContractError')
    env=dict(np=np,math=math,replace=replace,MappingProxyType=MappingProxyType,
        GripperGeometry=gg.GripperGeometry,TabletopGeometryConfig=tg.TabletopGeometryConfig,
        TabletopCandidateContractError=tg.TabletopCandidateContractError,MergeConfig=MergeConfig,
        AdaptiveStageLimits=AdaptiveStageLimits,FusedTargetSurface=FusedTargetSurface,SurfaceView=SurfaceView,
        materialize_tabletop_candidates=tg.materialize_tabletop_candidates,
        parse_tool_axis=gg.parse_tool_axis,contact_height_axis_base=gg.contact_height_axis_base,
        finger_contact_patch_overlap_m=gg.finger_contact_patch_overlap_m,
        evaluate_bilateral_surface_evidence_and_bounds=gg.evaluate_bilateral_surface_evidence_and_bounds,
        evaluate_inferred_box_contact=gg.evaluate_inferred_box_contact,
        BilateralSurfaceEvidence=gg.BilateralSurfaceEvidence,InferredBoxContactEvidence=gg.InferredBoxContactEvidence)
    exec(compile(ast.Module(body=[exception],type_ignores=[]),str(candidate_source),'exec'),env)
    functions={'_continuous_config_error','_strict_config_number','_strict_config_integer',
        '_strict_tabletop_approach_tilt_degrees','load_opening_fit_clearance_config',
        'load_tabletop_geometry_config','load_adaptive_stage_config'}
    constants={'TABLETOP_GEOMETRY_DEFAULTS','TABLETOP_GEOMETRY_MAX_CANDIDATES',
        'CONTINUOUS_CONFIG_INVALID','ADAPTIVE_STAGE_DEFAULTS'}
    body=[n for n in tree.body if (isinstance(n,ast.FunctionDef)and n.name in functions)
        or(isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id in constants for t in n.targets))]
    exec(compile(ast.Module(body=body,type_ignores=[]),str(source),'exec'),env)
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='RemoteGrasp6DNode')
    methods={'_tabletop_placement_gripper','_materialize_tabletop_union','_proposal_contact_height_bounds',
        '_tabletop_candidate_contact_overlap_m','_current_bilateral_surface_measurement',
        '_contact_evidence_valid','_phase_contact_overlap_requirement_m','_geometry_object_height_m'}
    body=[n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name in methods]
    assert len(body)==len(methods)
    frozen=ast.ClassDef(name='FrozenProducerCallbacks',bases=[],keywords=[],body=body,decorator_list=[])
    ast.fix_missing_locations(frozen)
    exec(compile(ast.Module(body=[frozen],type_ignores=[]),str(source),'exec'),env)
    node=env['FrozenProducerCallbacks']()
    config_path=baseline/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
    remote_cfg=deepcopy(yaml.safe_load(config_path.read_text())['grasp_6d']['remote'])
    env.update(self=node,remote_cfg=remote_cfg,gripper_geometry_cfg=remote_cfg['gripper_geometry'])
    init=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='__init__')
    assignments={'gripper_geometry','opening_fit_clearance_each_side_m','tabletop_geometry_enabled',
        'tabletop_geometry_config','hybrid_merge_config','adaptive_stage_limits',
        'gripper_tool_jaw_axis','gripper_tool_finger_length_axis'}
    for statement in init.body:
        if isinstance(statement,ast.Assign):
            attrs=[n.attr for target in statement.targets for n in ast.walk(target)
                if isinstance(n,ast.Attribute)and isinstance(n.value,ast.Name)and n.value.id=='self']
            if attrs and all(a in assignments for a in attrs):
                exec(compile(ast.Module(body=[statement],type_ignores=[]),str(source),'exec'),env)
    assert all(hasattr(node,n)for n in assignments)
    # Frozen source model is already formally rebound to this exact full
    # fused GeometryEstimate. No live source/owner or execution is fabricated.
    node._current_inferred_box=lambda surface_frame='snapshot':model
    planning,enclosure=enclose_inferred_box_geometry(geometry,model)
    n=planning.support_normal_base; support=-planning.support_offset_m*n
    generation=tg.generate_tabletop_proposals(planning.object_points_base,planning.center_base,
        planning.axes_base,planning.size_xyz_m,support,n,node.tabletop_geometry_config,
        fused_surface=snapshot_surface,inferred_box=model,finger_geometry=node.gripper_geometry,
        refine_obb_neighbor_angles=True)
    archived=original_audit['remote_diagnostics']['tabletop_geometry']
    execution_errors={float(p['execution_position_error_m'])for p in archived['adaptive_stage_profiles']}
    assert len(execution_errors)==1
    execution_error=next(iter(execution_errors))
    profiles=derive_adaptive_stage_profiles(object_height_m=node._geometry_object_height_m(planning),
        depth_repeatability_m=archived['depth_repeatability_m'],
        finger_length_m=float(node.gripper_geometry.finger_size_xyz_m[2]),
        support_clearance_m=node.gripper_geometry.support_clearance_m,
        limits=node.adaptive_stage_limits,execution_position_error_m=execution_error)
    tilts=tuple(p.tilt_deg for p in profiles if p.tilt_deg>1e-9)
    required=max(node._phase_contact_overlap_requirement_m(True,p)for p in profiles)
    rejected=Counter(); rows=[]
    for proposal in generation.proposals:
        # Same complete CAD materializer and original merge helper. This probe
        # covers the original adaptive grid, not additional boundary search.
        try:
            initial=tg.materialize_tabletop_candidates(proposal,support,n,node._tabletop_placement_gripper(True),
                node.gripper_tool_jaw_axis,node.gripper_tool_finger_length_axis,tilts)
            variants=node._materialize_tabletop_union(proposal,support,n,initial,tilts,tilts,True)
        except tg.TabletopCandidateContractError as exc:
            rejected[exc.code]+=1;continue
        for item in variants:
            _, evidence, bounds, axis=node._current_bilateral_surface_measurement(
                item.contact_center_base,np.asarray(item.T_base_tool0)[:3,:3],
                surface_snapshot=(snapshot_surface,reference_view))
            if not node._contact_evidence_valid(evidence) or bounds is None:
                rejected['CONTACT_EVIDENCE_INVALID']+=1;continue
            overlap=node._tabletop_candidate_contact_overlap_m(proposal,item,n,
                contact_height_bounds_m=bounds,contact_height_axis_base_override=axis)
            if overlap+1e-9<required:
                rejected['GRIPPER_CONTACT_PATCH_MISS']+=1;continue
            interval=gg._obb_line_interval(item.contact_center_base,item.jaw_axis_base,
                planning.center_base,planning.axes_base,planning.size_xyz_m)
            maximum_reach=.5*min(node.gripper_geometry.max_inner_gap_m,gg.ANALYTICAL_MAX_INNER_GAP_M)
            fit=node.opening_fit_clearance_each_side_m
            if interval is not None and interval[0]<0.<interval[1]and max(-interval[0]+fit,interval[1]+fit)>maximum_reach+1.001e-9:
                rejected['GRIPPER_SWEEP_COLLISION_FINGER_REACH']+=1;continue
            rows.append(dict(proposal_index=int(proposal.source_index),source_index=int(item.source_index),
                variant_index=int(item.variant_index),angle_deg=float(proposal.angle_deg),
                T_base_tool0=np.asarray(item.T_base_tool0).tolist(),contact_center_base=np.asarray(item.contact_center_base).tolist(),
                insertion_axis_base=np.asarray(item.insertion_axis_base).tolist(),jaw_axis_base=np.asarray(item.jaw_axis_base).tolist(),
                required_open_width_m=float(item.required_open_width_m),
                physical_aperture_margin_m=node.gripper_geometry.max_inner_gap_m-float(item.required_open_width_m),
                contact_height_bounds_m=list(bounds),contact_height_axis_base=np.asarray(axis).tolist(),
                contact_evidence_type=type(evidence).__name__,contact_overlap_m=float(overlap),
                required_contact_overlap_m=required,placement_audit=dict(item.audit),
                model_sha256=model.model_sha256,downstream_motion_authorized=False))
    return dict(proposal_ok=generation.ok,proposal_count=len(generation.proposals),
        generation_failure_code=generation.failure_code,generation_failure_reason=generation.failure_reason,
        proposal_rejections=[dict(r)for r in generation.surface_evidence_rejections],
        materialized_contact_candidates=rows,materialized_contact_candidate_count=len(rows),
        rejection_counts=dict(rejected),source_model_sha256=model.model_sha256,
        physical_max_inner_gap_m=node.gripper_geometry.max_inner_gap_m,
        opening_fit_clearance_each_side_m=node.opening_fit_clearance_each_side_m,
        finger_geometry=asdict(node.gripper_geometry),tabletop_config=asdict(node.tabletop_geometry_config),
        adaptive_limits=asdict(node.adaptive_stage_limits),adaptive_profiles=[asdict(p)for p in profiles],
        archived_execution_error_m=execution_error,archived_depth_repeatability_m=archived['depth_repeatability_m'],
        inferred_planning_enclosure=enclosure,planning_size_xyz_m=planning.size_xyz_m.tolist(),
        production_callbacks=sorted(methods),production_callback_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        frozen_yaml_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        configuration_scope='Frozen deployed-source YAML and original recorded dynamic depth/endpoint residual; no complete historical ROS parameter reconstruction.',
        coverage_scope='Adaptive tilt grid; CAD placement, contact evidence/overlap, aperture and early finger-reach gates. Boundary search, full analytic swept collision, IK, MoveIt and execution not run.',
        full_candidate_selection_or_motion_authorized=False)
