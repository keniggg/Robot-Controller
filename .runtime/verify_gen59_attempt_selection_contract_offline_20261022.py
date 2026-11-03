"""Offline adapter review: inert line-boundary cases, source pins and AST guards.

This never imports ROS, runs a subprocess, captures a scene, or dispatches an
attempt. A passing result is a local implementation check, not independent
authorization or runtime readiness.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(output):
    output=output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and N/'gen58_resume_r1' in output.parents
    assert not output.exists() and output.parent.exists()
    output.mkdir();sys.dont_write_bytecode=True
    blocked=[]
    def audit(event,args):
        if event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.fork','os.forkpty','os.posix_spawn','os.posix_spawnp','os.exec'):
            blocked.append(event);raise RuntimeError('offline boundary: '+event)
        if event=='open' and len(args)>2 and isinstance(args[0],(str,bytes,os.PathLike)):
            mode,flags=args[1:3]
            if isinstance(mode,str) and any(c in mode for c in 'wax+') or isinstance(flags,int) and flags & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND):
                Path(os.fsdecode(args[0])).resolve().relative_to(output)
    sys.addaudithook(audit)
    (output/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    sys.path.insert(0,str(W/'.runtime'))
    from gen59_remote_attempt_entry_20261022 import run_with_line_gate,original_boundary
    import gen59_remote_attempt_entry_20261022 as entry
    import gen59_remote_attempt_common_20261022 as common
    checks=[]
    repair=N/'gen58_resume_r1/observation_pose_fix/attempt_selection_contract_fix_r1'
    class CanonicalComparison(ast.NodeTransformer):
        def visit_Call(self,node):
            node=self.generic_visit(node)
            if isinstance(node.func,ast.Name) and node.func.id=='same_selection':
                assert len(node.args)>=2 and not node.keywords
                return ast.Compare(left=node.args[0],ops=[ast.Eq() for _ in node.args[1:]],comparators=node.args[1:])
            return node
    def extract(path,name):
        tree=ast.parse(path.read_text())
        nodes=[node for node in tree.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name==name]
        assert len(nodes)==1
        return ast.dump(CanonicalComparison().visit(nodes[0]),include_attributes=False)
    unchanged=[('entry','original_boundary'),('entry','run_with_line_gate'),
        ('gate','RecordedMaster'),('gate','CurrentFeedback'),('gate','final_gate'),
        ('common','live_identity'),('common','recheck')]
    for role,name in unchanged:
        assert extract(repair/'before'/('gen59_remote_attempt_'+role+'_20261022.py'),name)==extract(W/('.runtime/gen59_remote_attempt_'+role+'_20261022.py'),name)
        checks.append('original_guard_AST_unchanged_except_selection_canonical_comparison:'+role+'.'+name)
    assert original_boundary()==121;checks.append('original_runner_unique_mode_boundary_line_121')
    dep_names=('ORIGINAL','CONTINUATION','PREWARM','RECORD_WRAPPER','RECORDER','ARCHIVE','COLLECTOR','WRITER')
    for name in dep_names:
        assert sha(getattr(entry,name))==getattr(entry,name+'_SHA')
        checks.append('dependency_sha:'+name)
    legacy=json.loads((N/'verification/gen57_cold_attempt_entry_independent_r1/result.json').read_text())
    for path,digest in legacy['dependency_sha256'].items():
        if str(W) in path and ('gen57_cold_attempt_' in path or path in [str(getattr(entry,k)) for k in dep_names[:6]]):
            assert sha(path)==digest
    checks.append('old_helpers_and_original_runner_guard_recorder_bytes_unchanged')
    class GateRejected(RuntimeError):pass
    normal="def run():\n    events.append('before')\n    events.append('sentinel')\n    events.append('after')\n    return 17\n"
    def case(name,raw=normal):
        path=output/(name+'.py');path.write_text(raw)
        events=[];space=dict(events=events,GateRejected=GateRejected)
        exec(compile(raw,str(path),'exec'),space)
        line=next((i for i,s in enumerate(raw.splitlines(),1) if "events.append('sentinel')" in s),100)
        return path,events,space,line
    def gate(events):
        return lambda _frame:events.append('gate')
    def rejected(call,reason):
        try:call()
        except RuntimeError:pass
        else:raise AssertionError(reason)
        assert sys.gettrace() is None
        checks.append(reason)
    path,events,space,line=case('success_before_bound_line')
    assert run_with_line_gate(path,sha(path),line,gate(events),space['run'])==17
    assert events==['before','gate','sentinel','after'] and sys.gettrace() is None
    checks.append('successful_gate_precedes_original_bound_line')
    path,events,space,line=case('gate_failure_blocks_bound_line')
    def reject(_frame):events.append('gate');raise GateRejected('rejected')
    rejected(lambda:run_with_line_gate(path,sha(path),line,reject,space['run']),'gate_failure_blocks_bound_line')
    assert events==['before','gate']
    path,events,space,line=case('duplicate_hit_blocks_second_bound_line',"def run():\n    for index in range(2):\n        events.append('sentinel')\n    return 0\n")
    rejected(lambda:run_with_line_gate(path,sha(path),line,gate(events),space['run']),'duplicate_boundary_rejected')
    assert events==['gate','sentinel']
    path,events,space,line=case('missing_boundary_returns_failure','def run():\n    return 0\n')
    rejected(lambda:run_with_line_gate(path,sha(path),line,gate(events),space['run']),'missing_boundary_rejected')
    assert events==[]
    path,events,space,line=case('wrong_source_sha_prevents_run')
    rejected(lambda:run_with_line_gate(path,'0'*64,line,gate(events),space['run']),'wrong_source_sha_prevents_run')
    assert events==[]
    path,events,space,line=case('source_change_before_boundary_blocks_line',"def run():\n    events.append('before')\n    mutate()\n    events.append('sentinel')\n    return 0\n")
    space['mutate']=lambda:path.write_text('changed inert fixture\n')
    rejected(lambda:run_with_line_gate(path,sha(path),line,gate(events),space['run']),'source_change_at_boundary_rejected')
    assert events==['before']
    path,events,space,line=case('original_failure_before_boundary_restores_tracer',"def run():\n    raise GateRejected('original early failure')\n    events.append('sentinel')\n")
    rejected(lambda:run_with_line_gate(path,sha(path),line,gate(events),space['run']),'original_early_failure_restores_tracer')
    assert events==[]
    path,events,space,line=case('original_failure_after_boundary_restores_tracer',"def run():\n    events.append('sentinel')\n    raise GateRejected('original later failure')\n")
    rejected(lambda:run_with_line_gate(path,sha(path),line,gate(events),space['run']),'original_later_failure_restores_tracer')
    assert events==['gate','sentinel']
    path,events,space,line=case('existing_tracer')
    existing=lambda *_:None
    sys.settrace(existing)
    try:
        try:run_with_line_gate(path,sha(path),line,gate(events),space['run'])
        except RuntimeError:pass
        else:raise AssertionError('existing tracer not rejected')
        assert sys.gettrace() is existing and events==[]
    finally:sys.settrace(None)
    checks.append('existing_tracer_preserved_and_run_rejected')
    import copy
    sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
    pre_path=N/'gen58_resume_r1/observation_pose_fix/actual_attempt_r1/scene_start_r1/preflight.json'
    dep_path=N/'gen58_resume_r1/observation_pose_fix/runtime_reload_r1/acceptance/result.json'
    assert sha(pre_path)=='18d5de3f11972bcb72dea97f1bf802c2f37280b93fe3a92b240d67a3f4e2365d'
    assert sha(dep_path)=='b5b53dcf13f51d507187a4c71e7ac20977e3bbc1ed6b51025f30ed5aabfb1082'
    pre,dep=json.loads(pre_path.read_bytes()),json.loads(dep_path.read_bytes())
    raw,canonical=pre['selection'],dep['selection'];unchanged_raw=copy.deepcopy(raw)
    assert raw!=canonical and pre['epoch_ns']==dep['epoch_ns']
    assert type(raw['stamp_ns']) is str and type(canonical['stamp_ns']) is int
    assert common.same_selection(raw,canonical) and common.same_selection(canonical,raw,canonical)
    assert common.selection_identity(raw)==canonical and raw==unchanged_raw and type(raw['stamp_ns']) is str
    selection_checks=['actual_r1_raw_vs_int_failure_reproduced','actual_r1_canonical_equivalence','raw_input_unchanged','three_way_equivalence']
    for field,value in [('mode','carton'),('strategy','direct'),('generation',3),('stamp_ns',canonical['stamp_ns']+1)]:
        different=dict(canonical,**{field:value})
        try:answer=common.same_selection(raw,different)
        except (ValueError,RuntimeError):answer=False
        assert answer is False
        selection_checks.append('changed_field_rejected:'+field)
    invalid=[]
    for key in raw:
        value=dict(raw);value.pop(key);invalid.append(('missing:'+key,value))
    invalid.extend([('extra_key',dict(raw,extra=1)),('empty',{}),('json_container',json.dumps(raw)),('list_container',list(raw.items())),('none_container',None)])
    for field,values in [('mode',[None,True,3,[],'invalid']),('strategy',[None,True,3,[],'invalid']),
            ('generation',[None,True,False,2.,'2',0,-1]),
            ('stamp_ns',[None,True,False,float(canonical['stamp_ns']),0,-1,'','-1','+1','1.0','1e18',' 1','1 ',float('nan'),float('inf')])]:
        for i,value in enumerate(values):invalid.append((field+':'+str(i),dict(raw,**{field:value})))
    for name,value in invalid:
        try:common.same_selection(raw,value)
        except (ValueError,RuntimeError,TypeError):pass
        else:raise AssertionError('invalid selection accepted:'+name)
        selection_checks.append('schema_rejected:'+name)
    # Raw/canonical compare is never allowed to round a nanosecond through float.
    assert canonical['stamp_ns']>2**53 and not common.same_selection(raw,dict(raw,stamp_ns=str(canonical['stamp_ns']+1)))
    selection_checks.append('one_nanosecond_above_float_precision_rejected')
    # All seven adapted comparison sites use the shared contract; the readiness
    # payload retains the unmodified raw preflight selection for the old runner.
    expected_calls={'capture':2,'receipt':1,'entry':2,'gate':2}
    for role,count in expected_calls.items():
        tree=ast.parse((W/('.runtime/gen59_remote_attempt_'+role+'_20261022.py')).read_bytes())
        assert sum(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='same_selection' for n in ast.walk(tree))==count
        selection_checks.append('comparison_sites:'+role+':'+str(count))
    capture=ast.parse((W/'.runtime/gen59_remote_attempt_capture_20261022.py').read_bytes())
    payload=[kw.value for n in ast.walk(capture) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='update' for kw in n.keywords if kw.arg=='selection']
    assert len(payload)==1 and ast.dump(payload[0],include_attributes=False)==ast.dump(ast.parse("pre['selection']",mode='eval').body,include_attributes=False)
    selection_checks.append('readiness_preserves_raw_preflight_selection')
    from alicia_grasp_modes import selection
    parser_path=Path(selection.__file__).resolve()
    release_path=N/'verification/gen59_release_manifest.json';release=json.loads(release_path.read_bytes())
    assert parser_path==W/'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py'
    assert sha(parser_path)==release['source_sha256'][str(parser_path.relative_to(W))]
    selection_checks.append('unchanged_installed_production_parser_reused')
    (output/'actual_r1_selection_fixture.json').write_text(json.dumps(dict(preflight_path=str(pre_path),preflight_sha256=sha(pre_path),
        acceptance_path=str(dep_path),acceptance_sha256=sha(dep_path),epoch_ns=pre['epoch_ns'],raw=raw,canonical=canonical),indent=2)+'\n')
    checks.extend(selection_checks)
    helpers=[W/('.runtime/gen59_remote_attempt_'+role+'_20261022.py') for role in ('common','capture','receipt','gate','entry')]
    for path in helpers:ast.parse(path.read_text())
    assert not blocked and 'rospy' not in sys.modules and 'rosgraph' not in sys.modules
    result=dict(passed=True,attempt_entry_reviewed=False,
        scope='Strict raw/canonical selection schema tests and original inert line-boundary cases only; root independent review and fresh current capture still required',
        checks=checks,line_boundary_cases=9,selection_contract_checks=len(selection_checks),dependency_sha256={str(p):sha(p) for p in helpers},
        original_dependency_sha256={str(getattr(entry,k)):getattr(entry,k+'_SHA') for k in dep_names[:6]},
        blocked_boundary_attempts=blocked,ROS_contacted=False,motion_commands_sent=False,services_started=False,
        temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
    raw=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode();(output/'result.json').write_bytes(raw)
    assert(output/'result.json').read_bytes()==raw
    print(json.dumps(dict(passed=True,checks=len(checks),line_boundary_cases=9,result=str(output/'result.json'),sha256=sha(output/'result.json'))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    main(parser.parse_args().output)
