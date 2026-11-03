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
    def extract(path,name):
        tree=ast.parse(path.read_text())
        nodes=[node for node in tree.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name==name]
        assert len(nodes)==1
        return ast.dump(nodes[0],include_attributes=False).replace('runtime epoch/selection changed','cold epoch/selection changed').replace('runtime17/GUI/publishers changed','cold17/GUI/publishers changed').replace('pinned runtime input changed','pinned cold input changed')
    unchanged=[('entry','original_boundary'),('entry','run_with_line_gate'),
        ('gate','CurrentFeedback'),('gate','final_gate'),('common','recheck')]
    for role,name in unchanged:
        assert extract(W/('.runtime/gen58_cold_attempt_'+role+'_20261021.py'),name)==extract(W/('.runtime/gen59_remote_attempt_'+role+'_20261022.py'),name)
        checks.append('original_guard_AST_unchanged_except_message:'+role+'.'+name)
    old_gate=(W/'.runtime/gen58_cold_attempt_gate_20261021.py').read_text()
    expected=old_gate.replace("extra = names - set(c['nodes']) - {self.rospy.get_name()}",
        "extra = names - set(c['nodes']) - set(c['deployment']['auxiliary_nodes']) - {self.rospy.get_name()}")
    expected_path=output/'expected_recorded_master.py';expected_path.write_text(expected)
    assert extract(expected_path,'RecordedMaster')==extract(W/'.runtime/gen59_remote_attempt_gate_20261022.py','RecordedMaster')
    checks.append('recorder_only_change_exact_accepted_auxiliary_set')
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
    sampler_path=W/'.runtime/sample_gen59_actual_task_resources_20261022.py'
    spec=importlib.util.spec_from_file_location('gen59_resource_contract_offline',sampler_path)
    sampler=importlib.util.module_from_spec(spec);spec.loader.exec_module(sampler)
    previous=json.loads((N/'gen58_resume_r1/full_restart_r1/acceptance/result.json').read_bytes())
    nodes=copy.deepcopy(previous['all_registered_nodes']);remote='/remote_grasp6d_node'
    nodes[remote]=dict(pid=999999,start_ticks='99999999',parent_pid=999998)
    preserved={n:r for n,r in nodes.items() if n!=remote}
    fixture=dict(deployment_passed=True,whole_patch_deployed=True,generation='GEN59',remote_reloaded=True,
        full_runtime_nodes=nodes,all_registered_nodes=nodes,final_all_registered_nodes=nodes,
        preserved_before=preserved,preserved_after=preserved,replaced_after={remote:nodes[remote]},
        after={n:nodes[n] for n in previous['after']})
    assert len(sampler.deployment_targets(fixture))==17
    checks.append('resource_accepts_explicit_synthetic_remote1_preserved16_core11_view')
    mutations=[('failed_deployment',lambda d:d.update(deployment_passed=False)),
        ('wrong_generation',lambda d:d.update(generation='GEN58')),
        ('remote_not_reloaded',lambda d:d.update(remote_reloaded=False)),
        ('missing_node',lambda d:d['full_runtime_nodes'].pop('/rosout')),
        ('nonremote_replaced',lambda d:d['replaced_after'].update({'/grasp_task_node':nodes['/grasp_task_node']})),
        ('changed_preserved',lambda d:d.update(preserved_before={})),
        ('wrong_compatibility_view',lambda d:d.update(after={})),
        ('pid_alias',lambda d:d['full_runtime_nodes'][remote].update(pid=nodes['/grasp_task_node']['pid']))]
    for name,mutate in mutations:
        d=copy.deepcopy(fixture);mutate(d)
        try:sampler.deployment_targets(d)
        except (RuntimeError,KeyError):pass
        else:raise AssertionError('resource accepted '+name)
        checks.append('resource_rejects:'+name)
    for name in ('record_root_identity','option_values','snapshot','interval','storage_check','script_and_args'):
        assert extract(W/'.runtime/sample_gen53_actual_task_resources_20261018.py',name)==extract(sampler_path,name)
        checks.append('original_resource_guard_AST_unchanged:'+name)
    from types import SimpleNamespace
    calls=[]; fixture_nodes={'/remote_grasp6d_node':{'pid':1}}
    graph=dict(node_identities=fixture_nodes,configured_nodes={},publishers={},core_service={},master_process={},gui_windows=[],auxiliary_nodes={})
    context=dict(deployment={'remote_unit':'unit','remote_service':{},'auxiliary_nodes':{}},
        verifier=SimpleNamespace(properties=lambda unit:{}),nodes=fixture_nodes,runtime_context={},
        runtime=SimpleNamespace(REMOTE='/remote_grasp6d_node',graph=lambda *args,**kwargs:(calls.append(kwargs) or graph)))
    for wait in (0.,3.):
        assert common.live_identity(context,None,None,graph_wait=wait)['nodes']==fixture_nodes
        assert calls[-1]['graph_wait']==wait
        checks.append('graph_wait_forwarded:'+str(wait))
    helpers=[W/('.runtime/gen59_remote_attempt_'+role+'_20261022.py') for role in ('common','capture','receipt','gate','entry')]
    helpers.append(sampler_path)
    for path in helpers:ast.parse(path.read_text())
    assert not blocked and 'rospy' not in sys.modules and 'rosgraph' not in sys.modules
    result=dict(passed=True,attempt_entry_reviewed=False,
        scope='Independent inert adapter/source review only; actual accepted GEN59 runtime binding and live pre-motion checks still required',
        checks=checks,line_boundary_cases=9,resource_contract_cases=9,dependency_sha256={str(p):sha(p) for p in helpers},
        original_dependency_sha256={str(getattr(entry,k)):getattr(entry,k+'_SHA') for k in dep_names[:6]},
        blocked_boundary_attempts=blocked,ROS_contacted=False,motion_commands_sent=False,services_started=False,
        temperature_queried=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
    raw=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode();(output/'result.json').write_bytes(raw)
    assert(output/'result.json').read_bytes()==raw
    print(json.dumps(dict(passed=True,checks=len(checks),line_boundary_cases=9,result=str(output/'result.json'),sha256=sha(output/'result.json'))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    main(parser.parse_args().output)
