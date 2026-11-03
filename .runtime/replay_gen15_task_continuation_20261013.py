#!/usr/bin/env python3
"""Deterministic R3 task continuation proof, not a real-time performance replay.

Original recorded motion is historical input. No hardware/planning RPC exists.
The real MRO measurement exchange, first phase creation, measured CAD envelope,
source-time RGBD working volume, and original caller's second ensure execute.
Original Unix timestamps are preserved, including the failing one-ULP stamp.
"""
import argparse
import ast
from contextlib import ExitStack
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import traceback
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261013T083836.150489Z_f4adfdd7ede44edeb725b4bfd7ac4f32')
D=N/'verification/gen15_qualified_owner_diagnosis'
I=N/'verification/gen15_continuation_inputs_r1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module);return module

def plain(value):
    if hasattr(value,'to_nsec'):return value.to_nsec()
    if hasattr(value,'__slots__'):return {k:plain(getattr(value,k)) for k in value.__slots__}
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    return value

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',default=str(W));parser.add_argument('--output',required=True)
    args=parser.parse_args();C=Path(args.candidate);O=Path(args.output)
    assert N in O.parents;O.mkdir(exist_ok=False)
    sys.path[:0]=[str(C/'src/alicia_grasp_modes/src'),str(C/'src/alicia_flexible_grasp_supervisor/src'),str(W/'.runtime')]
    import rospy,roslib.message,tf2_py,time
    from std_msgs.msg import String
    from alicia_grasp_modes import observe_return_task
    import gen14_three_owner_task_fixture_v6_20261013 as fixture
    from gen14_remote_fusion_fixture_20261013 import load_recorded_parameters
    base_path=C/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py'
    if not base_path.exists():base_path=W/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py'
    base=load_module('_alicia_original_grasp_task',base_path)
    task_path=C/'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py'
    if not task_path.exists():task_path=W/'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py'
    module=load_module('actual_gen15_continuation_task',task_path)
    wire=json.loads((D/'wire.json').read_text());manifest=json.loads((I/'manifest.json').read_text())
    def message(row):
        raw=Path(row['file']).read_bytes();assert hashlib.sha256(raw).hexdigest()==row['sha256']
        return roslib.message.get_message_class(row['datatype'])().deserialize(raw)
    executing=next(r for r in wire if r['topic'].endswith('measurement_request') and r['message']['state']=='executing')
    requal=next(r for r in wire if r['topic'].endswith('measurement_request') and r['message']['state']=='requalifying')
    done=next(r for r in wire if r['topic'].endswith('measurement_request') and r['message']['state']=='qualified')
    phase_row=next(r for r in wire if r['topic']=='/grasp/near_field_phase' and r['message']['active'])
    recorded_phase=message(phase_row);start_ns=recorded_phase.header.stamp.to_nsec()
    deadline_ns=recorded_phase.deadline.to_nsec();gate_ns=deadline_ns-90_000_000_000
    plan_row=next(r for r in wire if r['topic']=='/grasp_6d/plan_enriched' and r['message']['plan_id']==executing['message']['view_plan_id'])
    plan=message(plan_row);assert plan_row['sha256']=='7b2a10b47eae1daf9d6ab218ee5c610dfa7af4f5aac9e5ff38e578535030ed11'
    session=next(r['message'] for r in wire if r['topic']=='/grasp_mode/observe_return_session')
    saved=json.loads((R/'perception_execution_configuration.json').read_text())['parameters']
    camera=json.loads((N/'verification/gen14_arrival_independent/fixture.json').read_text())['camera']
    params,param_audit=load_recorded_parameters(camera,saved['/unknown_perception'],executing['message']['selection'])
    def assign(key,value):
        params[key]=deepcopy(value)
        if isinstance(value,dict):
            for name,item in value.items():assign(key+'/'+name,item)
    for key,value in saved.items():assign(key,value)
    param_audit['r3_runtime_overrides']={str(R/'perception_execution_configuration.json'):sha(R/'perception_execution_configuration.json')}
    stamp=lambda ns:rospy.Time(ns//10**9,ns%10**9)
    now=[requal['message']['arrival_stamp_ns']];node_box=[None];index=[0]
    publications=[];ingress=[];calls=[];first_phase=[];error=None;result=None
    buffer=tf2_py.BufferCore(rospy.Duration(600))
    events=[]
    for row in manifest['rows']:
        events.append((row['received_ns'],row['topic'],message(row),row))
    for row in wire:
        if row['topic']=='/perception/object' and requal['message']['arrival_stamp_ns']<=row['received_ns']<=1791880764200000000:
            events.append((row['received_ns'],row['topic'],message(row),row))
    events.sort(key=lambda r:r[0])
    def feed(until):
        assert until>=now[0],(until,now[0])
        while index[0]<len(events) and events[index[0]][0]<=until:
            received,topic,msg,row=events[index[0]];index[0]+=1
            now[0]=max(now[0],received);node=node_box[0]
            if topic in ('/tf','/tf_static'):
                setter=buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform
                for transform in msg.transforms:setter(transform,'original_R3')
            elif topic=='/alicia_d/accepted_joint_states':node.accepted_joint_cb(msg)
            elif topic=='/alicia_d/sdk_command':node.following_sdk_cb(msg)
            elif topic=='/joint_states':node.joint_cb(msg)
            elif topic=='/perception/object':node.obj_cb(msg)
            else:raise AssertionError(topic)
            ingress.append(dict(original_received_ns=received,callback_ns=now[0],topic=topic,sha256=row['sha256']))
        now[0]=until
    def no_rpc(*a,**kw):raise AssertionError('offline continuation attempted ROS/RPC/motion')
    def publish_request(msg):
        request=json.loads(msg.data);state=request['state'];expected={'requalifying':requal,'qualified':done}[state]
        assert request==expected['message'],('generated request differs from original R3',state)
        publications.append(dict(kind='request',now_ns=now[0],message=request))
        for row in wire:
            if row['topic'].endswith('measurement_ack') and row['message']['request_id']==request['request_id']:
                feed(row['received_ns']);node_box[0]._observation_measurement_ack_cb(message(row))
                ingress.append(dict(topic=row['topic'],original_received_ns=row['received_ns'],sha256=row['sha256']))
        feed(done['message']['stamp_ns'] if state=='requalifying' else gate_ns)
    def publish(kind):
        return lambda msg:publications.append(dict(kind=kind,now_ns=now[0],message=plain(msg)))
    base_tree=ast.parse(base_path.read_text())
    cls=next(n for n in base_tree.body if isinstance(n,ast.ClassDef) and n.name=='GraspTaskNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_execute_grasp6d_plan')
    candidates=[n for n in method.body if isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='defer_contact_gate'
                and any(isinstance(x,ast.Attribute) and x.attr=='_wait_for_fresh_observation_camera_target_range' for x in ast.walk(n))]
    assert len(candidates)==1
    branch=candidates[0]
    body=deepcopy(branch.body)+[ast.Return(value=ast.Name(id='station_phase',ctx=ast.Load()))]
    function=ast.FunctionDef(name='original_reached_continuation',args=ast.arguments(posonlyargs=[],args=[ast.arg(arg=n) for n in ('self','plan','gcfg','far_field_reference_plan','move_pose','strict_execute_pose')],kwonlyargs=[],kw_defaults=[],defaults=[]),body=body,decorator_list=[])
    namespace=dict(vars(base));exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),str(base_path)+' [unchanged reached branch]','exec'),namespace)
    with ExitStack() as stack:
        for key,value in [('get_param',lambda k,d=None:params.get(k,d)),('get_param_cached',lambda k,d=None:params.get(k,d)),('get_master',lambda:NS(getParam=lambda k:(1,'sealed original configuration',params.get(k)))),('is_shutdown',lambda:False),('sleep',lambda sec:feed(now[0]+round(sec*1e9))),('ServiceProxy',no_rpc),('wait_for_service',no_rpc),('init_node',no_rpc),('Publisher',no_rpc),('Subscriber',no_rpc)]:
            stack.enter_context(patch.object(rospy,key,value))
        for key in ('loginfo','logwarn','logwarn_throttle','logerr','logdebug'):
            stack.enter_context(patch.object(rospy,key,Mock()))
        stack.enter_context(patch.object(rospy.Time,'now',staticmethod(lambda:stamp(now[0]))))
        stack.enter_context(patch.object(time,'monotonic',lambda:now[0]*1e-9))
        stack.enter_context(patch.object(fixture,'CONFIGURATION',R/'perception_execution_configuration.json'))
        node,gcfg,task_audit=fixture.build_task(module,plan=plan,session=session,executing_request=executing['message'],settle=requal['message']['settle'],publish_request=publish_request,publish_state=publish('state'),publish_phase=publish('phase'),publish_near_field=publish('bool_near_field'))
        node_box[0]=node
        # The real ModeAware _execute_grasp6d_plan calls this strategy adapter
        # before the base method. The old handoff-only fixture never needed
        # its working-volume override and cannot stand in for this boundary.
        gcfg=node._strategy_config(deepcopy(saved['/grasp']))
        task_audit['real_strategy_config_applied']=True
        task_audit['observation_acceptance_policy']=gcfg.get('observation_acceptance_policy')
        # The actual recorded plan was frozen before motion. Re-run the exact
        # integrity/freeze function, including its raw-wire digest, not the
        # separate committed-plan digest used by the measurement protocol.
        node._freeze_execution_plan(plan)
        assert node._bound_execution_plan_digest==plan_row['sha256']
        node.tf_buffer=NS(lookup_transform=lambda a,b,s,t:buffer.lookup_transform_core(a,b,s))
        setter=node._set_near_field_active
        def set_phase(active,*a,**kw):
            if active and not first_phase:
                assert now[0]==gate_ns;feed(start_ns)
            value=setter(active,*a,**kw)
            if active:first_phase.append(deepcopy(node._published_near_field_phase))
            return value
        node._set_near_field_active=set_phase
        for name in ('_validate_measured_observation_envelope','_wait_for_fresh_observation_camera_target_range','_ensure_qualified_station_phase'):
            original=getattr(node,name)
            def traced(*a,_name=name,_original=original,**kw):
                item=dict(method=_name,entry_ns=now[0]);calls.append(item)
                try:
                    value=_original(*a,**kw);item['result']=plain(value) if isinstance(value,(bool,type(None))) or hasattr(value,'__slots__') else str(value);return value
                except Exception as exc:item['error']=type(exc).__name__+': '+str(exc);raise
                finally:item['exit_ns']=now[0]
            setattr(node,name,traced)
        node._maybe_execute_observation_camera_retreat=no_rpc
        try:
            feed(now[0]);node._observation_measurement_finish(plan,gcfg)
            assert len(first_phase)==1
            created=first_phase[0]
            assert created.header.stamp==recorded_phase.header.stamp and created.deadline==recorded_phase.deadline
            a,b=plain(created),plain(recorded_phase);a['header'].pop('seq');b['header'].pop('seq');assert a==b
            feed(1791880763662761000) # Original measured-envelope log event.
            result=namespace['original_reached_continuation'](node,plan,gcfg,plan,no_rpc,no_rpc)
        except Exception as exc:
            error=type(exc).__name__+': '+str(exc);traceback.print_exc()
        sources={str(p):sha(p) for p in (base_path,task_path,Path(observe_return_task.__file__),Path(fixture.__file__),Path(__file__))}
        output=dict(error=error,continuation_completed=result is not None and result is not False,
            first_phase_created=len(first_phase)==1,first_phase=plain(first_phase[0]) if first_phase else None,
            final_phase=plain(getattr(node,'_published_near_field_phase',None)),calls=calls,publications=publications,
            ingress=ingress,task_initialization=task_audit,parameter_audit=param_audit,
            envelope=getattr(node,'_last_measured_observation_envelope_sample',None),
            range=getattr(node,'_last_observation_camera_range_evidence',None),
            expected_original_header_ns=start_ns,expected_original_deadline_ns=deadline_ns,
            final_clock_ns=now[0],remaining_original_budget_sec=(deadline_ns-now[0])*1e-9,
            source_sha256=sources,inputs={str(D/'wire.json'):sha(D/'wire.json'),str(I/'manifest.json'):sha(I/'manifest.json')},
            caller_ast=dict(source=str(base_path),sha256=sha(base_path),line=branch.lineno,end_line=branch.end_lineno,
                            unchanged_ast_branch=True),no_live_ros=True,motion_commands_sent=False,
            scope='Original completed motion historical boundary; actual task finish/ACK parse/first phase/CAD/RGBD range/original second ensure. No planner, grasp, motion, CPU-budget or wall-time throughput claim.',
            clock_model='Deterministic original receive/event nanoseconds with unchanged production limits; not a real-time replay.',
            limitations=['Full _execute_grasp6d_plan motion preamble and planning/execution continuation are not invoked.',
              'Remote planning RPC/worker, contact plan generation, observed-contact audit and physical result remain separate.',
              'Unrecorded parameters use SHA-bound launch plus previously saved unchanged camera/handeye context.'])
        (O/'result.json').write_text(json.dumps(output,ensure_ascii=False,indent=2,default=str)+'\n')
    print(json.dumps(dict(output=str(O/'result.json'),sha256=sha(O/'result.json'),error=error,continuation_completed=output['continuation_completed'],calls=[(c['method'],c.get('error'))for c in calls]),ensure_ascii=False))

if __name__=='__main__':main()
