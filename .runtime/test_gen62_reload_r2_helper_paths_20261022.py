"""Offline finite path-boundary and immutable-byte checks for GEN62 reload r2."""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import sys
import time
import traceback
from types import SimpleNamespace

sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1]
F=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
D=F/'reload_r2_helpers_adaptation_r1'
OUT=D/'offline_contracts.json'
denied=[]
u=os.uname()
platform._uname_cache=platform.uname_result(u.sysname,u.nodename,u.release,u.version,u.machine,'')


def guard(event,args):
    forbidden=event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.posix_spawn','os.fork','os.exec')
    if event=='import' and str(args[0]).split('.')[0] in ('rospy','rosgraph','rosnode','rosmaster','actionlib'):
        forbidden=True
    if forbidden:
        denied.append(event);raise RuntimeError('offline boundary '+event)


sys.addaudithook(guard)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value)
    return value


class ResolvedFixturePath:
    """The actual equality predicate is tested without fabricating a future file."""
    def __init__(self,path):self.path=Path(path)
    def resolve(self,strict=True):return self.path


def extracted_prefix(module,path,function,stop):
    fn=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name==function)
    fn=deepcopy(fn)
    kept=[]
    for node in fn.body:
        if stop(node):break
        kept.append(node)
    else:raise AssertionError('prefix stop absent')
    fn.body=kept+[ast.Return(value=ast.Constant(value=True))]
    tree=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]))
    scope=dict(vars(module))
    exec(compile(tree,str(path)+'::actual_ast_path_prefix','exec'),scope)
    return scope[function]


def is_assignment(node,name):
    return isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in node.targets)


def reject(result,name,fn):
    try:fn()
    except (RuntimeError,AssertionError,KeyError,ValueError,TypeError) as exc:
        result['negative_cases'].append(dict(name=name,rejected=True,reason=str(exc)))
    else:raise AssertionError('bad path accepted '+name)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    start=time.monotonic()
    result=dict(kind='GEN62_reload_r2_finite_path_contracts',passed=False,positive_cases=[],negative_cases=[],
        boundary_attempts=denied,ROS_initialized=False,live_nodes_contacted=False,controls_written=False,
        deployment_run=False,future_release_or_acceptance_created=False,
        full_future_acceptance_closure_checked=False,
        scope='Actual AST path predicates; immutable byte reconstruction proves other existing gates are unchanged.')
    try:
        plan=json.loads((D/'plan.json').read_bytes())
        rows=plan['rows'];byname={Path(row['path']).name:row for row in rows}
        adapter=load(W/'.runtime/adapt_gen62_reload_r2_helpers_20261022.py','offline_r2_adapter')
        baseline={Path(row['path']).name:Path(row['backup']).read_bytes() for row in rows}
        expected=adapter.translated(baseline)
        for row in rows:
            name=Path(row['path']).name
            assert sha(row['backup'])==row['old_sha256']
            assert sha(row['path'])==row['new_sha256']
            assert Path(row['path']).read_bytes()==expected[name]
            assert sha(row['diff']['path'])==row['diff']['sha256']
        assert all(sha(p)==h for p,h in plan['protected_sha256'].items())
        result['positive_cases'].append('five_exact_transformations_old_backup_bytes_and_protected_inputs_unchanged')
        common_path=W/'.runtime/gen62_compute_attempt_common_20261022.py'
        post_path=W/'.runtime/read_gen62_post_attempt_20261022.py'
        manual_path=W/'.runtime/restore_gen62_after_handoff_20261022.py'
        sampler_path=W/'.runtime/sample_gen62_actual_task_resources_20261022.py'
        writer_path=W/'.runtime/write_gen62_accepted_release_20261022.py'
        common=load(common_path,'offline_r2_common')
        post=load(post_path,'offline_r2_post')
        manual=load(manual_path,'offline_r2_manual')
        sampler=load(sampler_path,'offline_r2_sampler')
        writer=load(writer_path,'offline_r2_writer')
        assert post.COMMON_SHA==sha(common_path) and manual.POST_SHA==sha(post_path)
        assert post.RUNTIME_SHA=='871b1b6b2ae048940beedd9d1a7c214fe2c9ff4201a981e1fd1f71efab6829de'
        assert all(sha(p)==h for p,h in post.PINS.items())
        result['positive_cases'].append('all_new_common_post_manual_cross_pins_and_fixed_runtime_match')
        bundle=json.loads((F/'runtime_prepare_r1/bundle.json').read_bytes())
        changed={row['path'] for row in rows}
        assert not changed.intersection(bundle['pins'])
        assert all(path not in json.dumps(bundle) for path in changed)
        assert len(bundle['pins'])==432 and len(bundle['source_sha256'])==1103
        result['positive_cases'].append('original_prepare_r1_bundle432pins_has_no_changed_helper_dependency')
        result['bundle_binding']=dict(path=str(F/'runtime_prepare_r1/bundle.json'),sha256=sha(F/'runtime_prepare_r1/bundle.json'),pins=432,source_count=1103)

        # Extract only the exact initial equality predicates; no synthetic future
        # file is created, read, or passed off as an accepted runtime closure.
        cgate=extracted_prefix(common,common_path,'load_context',lambda n:is_assignment(n,'pins'))
        pgate=extracted_prefix(post,post_path,'load_bindings',lambda n:is_assignment(n,'supplied'))
        sgate=extracted_prefix(sampler,sampler_path,'deployment_targets',lambda n:isinstance(n,ast.FunctionDef))
        accepted=F/'runtime_reload_r2/acceptance/result.json'
        release=post.RELEASE
        def args_for(path=accepted,release_path=release):
            return SimpleNamespace(release=ResolvedFixturePath(release_path),deployment=ResolvedFixturePath(path),
                helper_sha256=sha(post_path),common_sha256=sha(common_path),runtime_common_sha256=post.RUNTIME_SHA)
        assert cgate(args_for(),post_path) is True and pgate(args_for()) is True
        assert sgate({},accepted,'fixture-acceptance',release,'fixture-release') is True
        result['positive_cases'].extend(['common_exact_r2_path_predicate','post_exact_r2_path_predicate','sampler_exact_r2_path_predicate'])
        for label,path in [('r1',F/'runtime_reload_r1/acceptance/result.json'),('r3',F/'runtime_reload_r3/acceptance/result.json'),
                ('foreign',F.parent/'runtime_reload_r2/acceptance/result.json'),('near_miss',F/'runtime_reload_r2/acceptance/other.json')]:
            reject(result,'common_'+label,lambda p=path:cgate(args_for(p),post_path))
            reject(result,'post_'+label,lambda p=path:pgate(args_for(p)))
            reject(result,'sampler_'+label,lambda p=path:sgate({},p,'x',release,'y'))
        bad_release=release.parent/'gen61_release_manifest.json'
        reject(result,'common_old_release',lambda:cgate(args_for(release_path=bad_release),post_path))
        reject(result,'post_old_release',lambda:pgate(args_for(release_path=bad_release)))
        reject(result,'sampler_old_release',lambda:sgate({},accepted,'x',bad_release,'y'))
        old_pin_args=args_for();old_pin_args.common_sha256=byname[common_path.name]['old_sha256']
        reject(result,'post_old_common_pin',lambda:pgate(old_pin_args))
        reject(result,'common_old_common_pin',lambda:cgate(old_pin_args,post_path))

        wmain=next(n for n in ast.parse(writer_path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='main')
        predicate=next(n for n in wmain.body if isinstance(n,ast.Assert))
        tree=ast.fix_missing_locations(ast.Expression(body=deepcopy(predicate.test)))
        code=compile(tree,str(writer_path)+'::exact_acceptance_assert','eval')
        assert eval(code,dict(vars(writer),a=accepted)) is True
        result['positive_cases'].append('accepted_release_writer_exact_r2_path_predicate')
        for label,path in [('r1',F/'runtime_reload_r1/acceptance/result.json'),('r3',F/'runtime_reload_r3/acceptance/result.json'),
                           ('foreign',F.parent/'runtime_reload_r2/acceptance/result.json')]:
            assert eval(code,dict(vars(writer),a=path)) is False
            result['negative_cases'].append(dict(name='writer_'+label,rejected=True))

        # Also check the sampler's separate transition binding: prepare r1 stays
        # fixed while both acceptance and transition must refer to reload r2.
        sfn=next(n for n in ast.parse(sampler_path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='deployment_targets')
        start_index=next(i for i,n in enumerate(sfn.body) if is_assignment(n,'bundle_path'))
        body=deepcopy(sfn.body[start_index:start_index+3])
        assert is_assignment(body[1],'transition_path') and isinstance(body[2],ast.Expr)
        bpath=F/'runtime_prepare_r1/bundle.json';tpath=F/'runtime_reload_r2/result.json'
        fixture=dict(bundle=str(bpath),reload=str(tpath),bundle_sha256=sha(bpath),reload_sha256='fixture-only-transition-pin')
        fixture_release=dict(evidence_sha256={str(bpath):sha(bpath),str(tpath):'fixture-only-transition-pin'})
        def transition_gate(value):
            scope=dict(vars(sampler),session=F,value=value,release=fixture_release)
            exec(compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),str(sampler_path)+'::transition_predicate','exec'),scope)
        transition_gate(fixture)
        result['positive_cases'].append('sampler_original_prepare_r1_and_exact_reload_r2_transition')
        for label,field,value in [('transition_r1','reload',str(F/'runtime_reload_r1/result.json')),
                                  ('transition_r3','reload',str(F/'runtime_reload_r3/result.json')),
                                  ('prepare_r2','bundle',str(F/'runtime_prepare_r2/bundle.json')),
                                  ('wrong_transition_pin','reload_sha256','other')]:
            bad=dict(fixture);bad[field]=value
            reject(result,label,lambda v=bad:transition_gate(v))

        optimize_probe=load(W/'.runtime/test_gen62_manual_optimize_guard_20261022.py','offline_r2_optimize_probe')
        for compile_level,runtime_level in ((1,1),(2,2),(2,1)):
            assert optimize_probe.entry_probe(manual_path.read_text(),compile_level,runtime_level)['explicitly_rejected'] is True
        result['positive_cases'].append('manual_explicit_optimization_guard_preserved_three_rejections')
        sdk=lambda s:s[s.index("        driver = '/alicia_d_driver_node'"):s.index('        after = identities(nodes)')]
        assert sdk(post_path.read_text())==sdk(baseline[post_path.name].decode())
        result['positive_cases'].append('post_full_sdk20_idle_action_block_unchanged')
        assert not denied and all(sha(row['path'])==row['new_sha256'] for row in rows)
        result.update(passed=True,positive_count=len(result['positive_cases']),negative_count=len(result['negative_cases']),
            rows=rows,protected_inputs_unchanged=True,backup_verified=True,
            test=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    result['elapsed_seconds']=time.monotonic()-start
    raw=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode()
    with OUT.open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert OUT.read_bytes()==raw
    print(json.dumps(dict(path=str(OUT),sha256=sha(OUT),passed=result['passed'],positive=result.get('positive_count'),
        negative=result.get('negative_count'),error=result.get('error')),ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
