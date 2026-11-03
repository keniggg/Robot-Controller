"""Run the original reached-view branch without executing any motion preamble.

Only the surrounding caller frame is extracted. Its CAD/range/owner calls are
the actual node methods; all physical/RPC paths retain throwing sentinels.
"""
import ast
from copy import deepcopy
import hashlib
from pathlib import Path


def install_continuation(node, task_module, saved_grasp_config, lookup_transform, clock):
    from types import SimpleNamespace
    base=task_module.original
    source=Path(base.__file__)
    tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='GraspTaskNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_execute_grasp6d_plan')
    candidates=[n for n in method.body if isinstance(n,ast.If) and isinstance(n.test,ast.Name)
        and n.test.id=='defer_contact_gate' and any(isinstance(x,ast.Attribute)
        and x.attr=='_wait_for_fresh_observation_camera_target_range' for x in ast.walk(n))]
    assert len(candidates)==1
    branch=candidates[0]
    args=ast.arguments(posonlyargs=[],args=[ast.arg(arg=n)for n in
        ('self','plan','gcfg','far_field_reference_plan','move_pose','strict_execute_pose')],
        kwonlyargs=[],kw_defaults=[],defaults=[])
    function=ast.FunctionDef(name='original_reached_continuation',args=args,
        body=deepcopy(branch.body)+[ast.Return(value=ast.Name(id='station_phase',ctx=ast.Load()))],
        decorator_list=[])
    namespace=dict(vars(base))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),
                 str(source)+' [unchanged reached branch]','exec'),namespace)
    node.tf_buffer=SimpleNamespace(lookup_transform=lookup_transform)
    config=node._strategy_config(deepcopy(saved_grasp_config))
    calls=[]
    def forbid(*args,**kwargs):
        raise AssertionError('offline reached continuation requested physical correction or RPC')
    node._maybe_execute_observation_camera_retreat=forbid
    for name in ('_ensure_qualified_station_phase','_validate_measured_observation_envelope',
                 '_wait_for_fresh_observation_camera_target_range'):
        original=getattr(node,name)
        def traced(*args,_name=name,_original=original,**kwargs):
            row=dict(method=_name,entry_ns=clock());calls.append(row)
            try:
                value=_original(*args,**kwargs)
                if _name=='_ensure_qualified_station_phase' and value is not None:
                    row['phase_owner']=dict(phase_id=value.phase_id,header_ns=value.header.stamp.to_nsec(),
                        deadline_ns=value.deadline.to_nsec(),source_ns=value.reference_source_stamp.to_nsec(),
                        target_track_id=value.reference_target_track_id,active=value.active)
                    row['cache']=dict(started_ns=getattr(node,'_near_field_phase_started_ns',None),
                        deadline_ns=getattr(node,'_near_field_phase_deadline_ns',None),
                        started_sec=node._near_field_phase_started_sec,deadline_sec=node._near_field_phase_deadline_sec)
                elif isinstance(value,bool):row['ok']=value
                elif hasattr(value,'ok'):row.update(ok=value.ok,code=value.code,reason=value.reason)
                return value
            except Exception as exc:
                row['error']=type(exc).__name__+': '+str(exc);raise
            finally:row['exit_ns']=clock()
        setattr(node,name,traced)
    def run(plan):
        return namespace['original_reached_continuation'](node,plan,config,plan,forbid,forbid)
    audit=dict(calls=calls,caller_source=str(source),caller_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        caller_ast_line=branch.lineno,caller_ast_end_line=branch.end_lineno,unchanged_ast_branch=True,
        configuration_adapter='actual ModeAwareGraspTaskNode._strategy_config',
        observation_acceptance_policy=config.get('observation_acceptance_policy'),
        no_success_stubs=True,physical_correction_sentinel=True,
        historical_boundary='Original observation motion and settle are already recorded. No motion is re-executed.',
        remaining_uncovered=['Full task motion preamble','Remote candidate planning RPC/worker',
                             'Contact plan audit and execution','Physical grasp result'])
    return run,config,audit
