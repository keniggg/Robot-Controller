"""Read-only production AST callbacks with frozen GEN53 sources; no ROS imports.

The caller must install its own external I/O boundary before calling this module.
No result grants motion authority. Optional local yaw angles are diagnostic inputs.
"""
import ast
from pathlib import Path
from dataclasses import replace


def create_frozen_node(w, audit, geometry, surface, reference):
    import numpy as np
    from collections import Counter
    from types import SimpleNamespace
    import sys
    import gen53_r18_pure_materialization_pipeline_20261018 as original
    from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from alicia_flexible_grasp.grasp.adaptive_stage_profiles import (
        derive_adaptive_stage_profiles, adaptive_stage_profile_for_tilt)
    source=Path(original.__file__)
    tree=ast.parse(source.read_text())
    factory=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='materialize')
    body=[]
    for statement in factory.body:
        if (isinstance(statement,ast.Assign) and any(isinstance(t,ast.Attribute)
                and t.attr=='_current_inferred_box' for t in statement.targets)):
            break
        if (isinstance(statement,ast.Assign) and any(isinstance(t,ast.Name)
                and t.id=='methods' for t in statement.targets)):
            extra=['_generate_tabletop_candidates','_contact_boundary_tilts',
                '_stage_profile_audit','_tabletop_stratum_priority','_bilateral_surface_audit',
                '_inferred_candidate_reorientation']
            statement.value.elts.extend(ast.Constant(value=s) for s in extra)
        body.append(statement)
    body.append(ast.Return(value=ast.Tuple(elts=[ast.Name(id='node',ctx=ast.Load()),
        ast.Name(id='env',ctx=ast.Load())],ctx=ast.Load())))
    factory.body=body
    ast.fix_missing_locations(factory)
    namespace=dict(original.__dict__)
    exec(compile(ast.Module(body=[factory],type_ignores=[]),str(source),'exec'),namespace)
    node,env=namespace['materialize'](w,w,audit,geometry,surface,reference,None)
    env.update(Counter=Counter,generate_tabletop_proposals=tg.generate_tabletop_proposals,
        adaptive_stage_profile_for_tilt=adaptive_stage_profile_for_tilt,
        ANALYTICAL_MAX_INNER_GAP_M=gg.ANALYTICAL_MAX_INNER_GAP_M,
        _obb_line_interval=gg._obb_line_interval,
        CONTACT_EXECUTION_PLAN='CONTACT_EXECUTION_PLAN',
        FAR_FIELD_OBSERVATION_PLAN='FAR_FIELD_OBSERVATION_PLAN')
    sys.path.insert(0,str(w/'src/alicia_grasp_modes/src'))
    modepath=w/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
    mode=next(n for n in ast.parse(modepath.read_text()).body if isinstance(n,ast.ClassDef)
              and any(isinstance(m,ast.FunctionDef)and m.name=='_contact_boundary_tilts' for m in n.body))
    mode_methods={'_tabletop_placement_gripper','_tabletop_candidate_contact_overlap_m',
        '_contact_boundary_tilts','_tabletop_stratum_priority'}
    modeclass=ast.ClassDef(name='FrozenModeCallbacks',bases=[ast.Name(id='FrozenProducerCallbacks',ctx=ast.Load())],
        keywords=[],body=[n for n in mode.body if isinstance(n,ast.FunctionDef)and n.name in mode_methods],decorator_list=[])
    env['original']=SimpleNamespace(np=np,contact_height_axis_base=gg.contact_height_axis_base,
        parse_tool_axis=gg.parse_tool_axis)
    ast.fix_missing_locations(modeclass)
    exec(compile(ast.Module(body=[modeclass],type_ignores=[]),str(modepath),'exec'),env)
    node.__class__=env['FrozenModeCallbacks']
    node._mode_selection={'mode':'unknown','strategy':'observe_return'}
    node.near_field_planning_active=True
    node._direct_strategy_selected=lambda:True
    archived=audit['remote_diagnostics']['tabletop_geometry']
    errors={float(p['execution_position_error_m'])for p in archived['adaptive_stage_profiles']}
    assert len(errors)==1
    error=next(iter(errors))
    repeat=float(archived['depth_repeatability_m'])
    profiles=derive_adaptive_stage_profiles(object_height_m=node._geometry_object_height_m(geometry),
        depth_repeatability_m=repeat,finger_length_m=float(node.gripper_geometry.finger_size_xyz_m[2]),
        support_clearance_m=node.gripper_geometry.support_clearance_m,
        limits=node.adaptive_stage_limits,execution_position_error_m=error)
    node._current_inferred_box=lambda *args:None
    node._regular_box_snapshot=None
    node._snapshot_multiview_surface=lambda:(surface,reference)
    node._adaptive_stage_profiles=lambda *args,**kwargs:profiles
    node._snapshot_depth_mad_m=lambda *args:float(archived['depth_spatial_mad_m'])
    node._snapshot_depth_repeatability_m=lambda *args:repeat
    node._effective_adaptive_stage_limits=lambda:node.adaptive_stage_limits
    node._runtime_execution_position_error_m=lambda:error
    node._gripper_finger_length_m=lambda:float(node.gripper_geometry.finger_size_xyz_m[2])
    return node,env


def generate_local(node,env,geometry,angles_deg=None):
    """Execute original generation/materialization; optional bounded diagnostic axes."""
    from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
    import numpy as np
    original=tg._sample_angles
    if angles_deg is not None:
        def local(rotation,normal,config,refine_neighbors=False):
            x=tg._plane_basis_x(normal);y=np.cross(normal,x)
            return tuple((float(angle),x*np.cos(np.deg2rad(angle))+y*np.sin(np.deg2rad(angle)))
                         for angle in angles_deg)
        tg._sample_angles=local
    try:
        return node._generate_tabletop_candidates(geometry,snapshot=None,
            contact_execution_phase=True,refine_obb_neighbor_angles=True)
    finally:
        tg._sample_angles=original
