"""Finite offline checks for the exact new epoch/readiness/loader binding."""
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
G=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
F=G/'contact_feedback_fix_r1';D=F/'runtime_epoch_r2_implementation_r1'
OUT=D/'offline_contracts.json'
denied=[]
u=os.uname();platform._uname_cache=platform.uname_result(u.sysname,u.nodename,u.release,u.version,u.machine,'')


def guard(event,args):
    forbidden=event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.posix_spawn','os.fork','os.exec')
    if event=='import' and str(args[0]).split('.')[0] in ('rospy','rosgraph','rosnode','rosmaster','actionlib'):
        forbidden=True
    if forbidden:denied.append(event);raise RuntimeError('offline boundary '+event)


sys.addaudithook(guard)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_bytes())
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
def rejected(result,name,fn):
    try:fn()
    except (RuntimeError,AssertionError,ValueError,KeyError,TypeError) as exc:
        result['negative_cases'].append(dict(name=name,rejected=True,reason=str(exc)))
    else:raise AssertionError('negative accepted '+name)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    result=dict(kind='GEN62_new_epoch_r2_finite_contracts',passed=False,positive_cases=[],negative_cases=[],
        boundary_attempts=denied,ROS_initialized=False,live_nodes_contacted=False,controls_written=False,
        runtime_prepare_run=False,runtime_deploy_run=False,full_regression_rerun=False,
        future_bundle_or_acceptance_created=False)
    start=time.monotonic()
    try:
        plan=read(D/'plan.json');adapter=load(W/'.runtime/adapt_gen62_new_epoch_runtime_r2_20261022.py','offline_epoch_adapter')
        before={name:(adapter.BACKUP/name).read_bytes() for name in adapter.NAMES}
        expected=adapter.transformed(before)
        for row in plan['rows']:
            assert sha(row['backup'])==row['old_sha256'] and sha(row['path'])==row['new_sha256']
            assert Path(row['path']).read_bytes()==expected[Path(row['path']).name]
        assert all(sha(p)==h for p,h in plan['protected_sha256'].items())
        result['positive_cases'].append('three_constants_one_fixed_dependency_pin_and_four_exact_cascades_only')
        runtime_path=W/'.runtime/gen62_compute_runtime_r2_20261022.py'
        runtime=load(runtime_path,'offline_new_epoch_runtime')
        common=load(W/'.runtime/gen62_compute_attempt_common_20261022.py','offline_new_epoch_common')
        post=load(W/'.runtime/read_gen62_post_attempt_20261022.py','offline_new_epoch_post')
        manual=load(W/'.runtime/restore_gen62_after_handoff_20261022.py','offline_new_epoch_manual')
        assert runtime.EPOCH==1792668378941525117 and runtime.DESIGN==G/'user_resume_r1/readiness_r2/result.json'
        assert sha(runtime.DESIGN)==runtime.DESIGN_SHA==adapter.DESIGN_SHA
        assert common.RUNTIME==post.RUNTIME==runtime_path and post.RUNTIME_SHA==sha(runtime_path)
        assert post.COMMON_SHA==sha(common.__file__) and manual.POST_SHA==sha(post.__file__)
        assert all(sha(p)==h for p,h in post.PINS.items())
        result['positive_cases'].append('actual_readiness_and_all_cross_pins_match')
        oldtree=ast.parse(before[adapter.BASE_RUNTIME]);tree=ast.parse(runtime_path.read_text())
        oldfunctions={n.name:ast.dump(n,include_attributes=False) for n in oldtree.body if isinstance(n,ast.FunctionDef)}
        newfunctions={n.name:ast.dump(n,include_attributes=False) for n in tree.body if isinstance(n,ast.FunctionDef)}
        assert set(oldfunctions)==set(newfunctions)
        assert all(oldfunctions[n]==newfunctions[n] for n in oldfunctions if n!='prepare')
        prepare=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prepare')
        pins_update=next(n.value for n in prepare.body if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
            and isinstance(n.value.func,ast.Attribute) and isinstance(n.value.func.value,ast.Name)
            and n.value.func.value.id=='pins' and n.value.func.attr=='update')
        scope=dict(vars(runtime),pins={})
        eval(compile(ast.fix_missing_locations(ast.Expression(body=deepcopy(pins_update))),str(runtime_path)+'::literal_pins','eval'),scope)
        pins=scope['pins'];oldpath=W/'.runtime/gen62_compute_runtime_20261022.py'
        assert pins[str(oldpath)]==adapter.BEFORE[0] and pins[str(runtime_path)]==sha(runtime_path)
        runtime.pins_check(pins)
        starter_ast=ast.parse(runtime.STARTER.read_text())
        assert any(isinstance(n,ast.Import) and any(a.name=='gen62_compute_runtime_20261022' and a.asname=='adapter' for a in n.names)
            for n in ast.walk(starter_ast))
        result['positive_cases'].append('unchanged_starter_old_runtime_import_in_explicit_new_pins')
        for label,path in [('old_runtime',oldpath),('new_runtime',runtime_path),('new_readiness',runtime.DESIGN)]:
            altered=dict(pins);altered[str(path)]='0'*64
            rejected(result,label+'_pin_mismatch',lambda p=altered:runtime.pins_check(p))
        # Evaluate the unchanged original readiness predicate on the real sealed
        # new receipt and original accepted17/source map, then mutate one field.
        readiness=read(runtime.DESIGN);baseline=read(runtime.BASE);oldbundle=read(F/'runtime_prepare_r1/bundle.json')
        gate=next(n.value for n in prepare.body if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
            and any(isinstance(a,ast.Constant) and a.value=='resumed current reference differs' for a in n.value.args))
        gatecode=compile(ast.fix_missing_locations(ast.Expression(body=deepcopy(gate))),str(runtime_path)+'::readiness_gate','eval')
        def check_readiness(value):
            eval(gatecode,dict(vars(runtime),readiness=value,baseline=baseline,ctx=dict(nodes=oldbundle['nodes'])))
        check_readiness(readiness)
        result['positive_cases'].append('real_new_epoch_readiness_original_selection17nodes_source_gate')
        for label,key,value in [('old_epoch','epoch_ns',1792661888995982113),('future_epoch','epoch_ns',1792668378941525118),
            ('manual_false','manual_mode',False),('task_active','task_active',True),('actions_active','actions_active',True),
            ('unconfirmed','actuation_status','UNCONFIRMED:FEEDBACK_STALE')]:
            changed=deepcopy(readiness);changed[key]=value
            rejected(result,label,lambda v=changed:check_readiness(v))
        changed=deepcopy(readiness);changed['selection']['generation']=6
        rejected(result,'selection_changed',lambda:check_readiness(changed))
        changed=deepcopy(readiness);changed['before']['node_identities']['/grasp_task_node']['pid']+=1
        rejected(result,'instance_changed',lambda:check_readiness(changed))
        # Actual unchanged sample() with a pure injected validator proves it
        # forwards the new runtime-local EPOCH; no live sampler is invoked.
        observed=[]
        def idle(*args):
            observed.append(args[-2:]);assert args[-1]==runtime.EPOCH and args[-2]==runtime.SELECTION;return True
        saved=runtime.idle_function
        runtime.idle_function=lambda c,absent:(idle,None)
        try:assert runtime.sample(None,{},None,None,{'/grasp_task_node':{}}) is True
        finally:runtime.idle_function=saved
        assert observed==[(runtime.SELECTION,runtime.EPOCH)]
        result['positive_cases'].append('unchanged_sample_passes_new_epoch_to_original_validator_interface')
        sampler=(W/'.runtime/sample_gen62_actual_task_resources_20261022.py').read_text()
        assert sampler.count('runtime_prepare_r2/bundle.json')==1 and 'runtime_prepare_r1/bundle.json' not in sampler
        assert sampler.count('runtime_reload_r2/')==2
        result['positive_cases'].append('sampler_fixed_prepare_r2_reload_r2_paths')
        # Reuse the audited AST-only path-prefix extractor. No whole future
        # acceptance/context or original runtime prepare is called.
        probes=load(W/'.runtime/test_gen62_reload_r2_helper_paths_20261022.py','offline_epoch_path_probe')
        pgate=probes.extracted_prefix(post,Path(post.__file__),'load_bindings',lambda n:probes.is_assignment(n,'supplied'))
        args=SimpleNamespace(release=probes.ResolvedFixturePath(post.RELEASE),deployment=probes.ResolvedFixturePath(post.ACCEPTED),
            runtime_common_sha256=post.RUNTIME_SHA,common_sha256=post.COMMON_SHA)
        assert pgate(args) is True
        result['positive_cases'].append('post_exact_current_runtime_common_and_r2_path_gate')
        stale=SimpleNamespace(**vars(args));stale.runtime_common_sha256=adapter.BEFORE[0]
        rejected(result,'post_old_runtime_pin',lambda:pgate(stale))
        stale=SimpleNamespace(**vars(args));stale.common_sha256=adapter.BEFORE[1]
        rejected(result,'post_old_common_pin',lambda:pgate(stale))
        assert not denied and all(sha(row['path'])==row['new_sha256'] for row in plan['rows'])
        result.update(passed=True,positive_count=len(result['positive_cases']),negative_count=len(result['negative_cases']),
            protected_inputs_unchanged=True,helper_sha256={row['path']:row['new_sha256'] for row in plan['rows']},
            readonly_real_readiness=dict(path=str(runtime.DESIGN),sha256=runtime.DESIGN_SHA),
            test=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
    except BaseException as exc:result.update(error=repr(exc),traceback=traceback.format_exc())
    result['elapsed_seconds']=time.monotonic()-start
    raw=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode()
    with OUT.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert OUT.read_bytes()==raw
    print(json.dumps(dict(path=str(OUT),sha256=sha(OUT),passed=result['passed'],positive=result.get('positive_count'),
        negative=result.get('negative_count'),error=result.get('error')),ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
