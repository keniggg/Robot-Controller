"""Pure offline contracts for GEN60 adapters; never imports ROS or executes main."""
import argparse
import ast
from copy import deepcopy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
from types import ModuleType,SimpleNamespace
import unittest
from unittest.mock import patch
sys.dont_write_bytecode=True
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'.runtime'))
sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
import gen60_gateway_reload_common_20261022 as c
import install_reload_gen60_gateway_20261022 as installer


class OfflineGuard:
    def __init__(self,out):self.out=out.resolve();self.blocked=[]
    def __call__(self,event,args):
        if event.startswith(('socket.','subprocess.','os.system','os.fork','os.exec','os.posix_spawn')):
            self.blocked.append(event);raise RuntimeError('offline operation blocked: '+event)
        if event=='open' and isinstance(args[0],(str,bytes,os.PathLike)):
            path=Path(os.fsdecode(args[0])).resolve();mode=args[1];flags=args[2]
            writing=(isinstance(mode,str) and any(x in mode for x in 'wax+')) or (isinstance(flags,int) and flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC))
            if writing and self.out not in path.parents:
                self.blocked.append('write:'+str(path));raise RuntimeError('offline write outside evidence directory')
        if event in ('os.remove','os.rmdir','os.mkdir','os.rename','os.chmod'):
            for index,item in enumerate(args[:2] if event=='os.rename' else args[:1]):
                if not isinstance(item,(str,bytes,os.PathLike)):continue
                path=Path(os.fsdecode(item))
                position=index+2 if event=='os.rename' else (2 if event in ('os.mkdir','os.chmod') else 1)
                descriptor=args[position] if len(args)>position else -1
                if not path.is_absolute() and isinstance(descriptor,int) and descriptor>=0:
                    path=Path(os.readlink('/proc/self/fd/'+str(descriptor)))/path
                path=path.resolve()
                if path!=self.out and self.out not in path.parents:
                    self.blocked.append(event+':'+str(path));raise RuntimeError('offline mutation outside evidence directory')


def function(path,name):
    return next(n for n in ast.parse(Path(path).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)


def fixture():
    ns=c.EPOCH+1_000_000_000_000
    stamp=lambda n:SimpleNamespace(to_nsec=lambda:n)
    header=lambda n:SimpleNamespace(stamp=stamp(n),frame_id='sdk_measured')
    owner=lambda n,latch='1':dict(callerid=n,latching=latch)
    messages={
      '/alicia_d/control_reference_epoch':SimpleNamespace(stamp=stamp(c.EPOCH),_connection_header=owner('/alicia_d_driver_node')),
      '/grasp/state':SimpleNamespace(active=False,state='FAILED',_connection_header=owner('/grasp_task_node')),
      '/grasp_mode/status':SimpleNamespace(data=json.dumps(dict(active=False,pending=False,start_reserved=False)),_connection_header=owner('/grasp_mode_router')),
      '/gui/joint_direct_mode':SimpleNamespace(data=True,_connection_header=owner('/alicia_supervisor_gui')),
      '/alicia_d/actuation_status':SimpleNamespace(data='CONFIRMED:offline-synthetic',_connection_header=owner('/alicia_d_driver_node')),
      '/alicia_d/motion_enabled':SimpleNamespace(data=True,_connection_header=owner('/alicia_d_driver_node')),
      '/alicia_d/feedback_ready':SimpleNamespace(data=True,_connection_header=owner('/alicia_d_driver_node'))}
    for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
        messages[topic]=SimpleNamespace(header=header(ns-1_000_000),status_list=[],_connection_header=owner('/bessica_d_hw_interface'))
    joints=[SimpleNamespace(header=header(ns-(20-i)*5_000_000),name=['Joint%d'%j for j in range(1,7)],position=[0.]*6,
        _connection_header=owner('/alicia_d_driver_node')) for i in range(20)]
    params={'/gui/joint_direct_mode':True,'/grasp/require_actuation_confirmation':True,
        '/alicia_d_driver_node/software_temperature_monitoring_enabled':False,
        '/alicia_d_driver_node/temperature_poll_rate_hz':0.,'/grasp/unknown_lift_after_close_enabled':False,
        '/grasp/unknown_observe_return_max_observations':1,'/grasp_mode/selection':dict(c.SELECTION)}
    def subscribe(topic,kind,callback,**kwargs):
        for msg in joints if topic=='/alicia_d/accepted_joint_states' else [messages[topic]]:callback(msg)
        return SimpleNamespace(unregister=lambda:None)
    ros=SimpleNamespace(Time=SimpleNamespace(now=lambda:stamp(ns)),Subscriber=subscribe,
        wait_for_message=lambda topic,kind,timeout:messages[topic],is_shutdown=lambda:False,
        wait_for_service=lambda *a,**k:None,ServiceProxy=lambda *a:lambda:SimpleNamespace(controller=[
            SimpleNamespace(name='alicia_controller',state='running'),SimpleNamespace(name='hand_controller',state='running')]))
    action_publishers=[('/alicia_controller/follow_joint_trajectory/status',['/bessica_d_hw_interface']),
        ('/hand_controller/follow_joint_trajectory/status',['/bessica_d_hw_interface'])]
    master=SimpleNamespace(getSystemState=lambda:[action_publishers,[],[('/controller_manager/list_controllers',['/bessica_d_hw_interface'])]],getParam=lambda k:params[k])
    ctx={'previous':{'epoch_ns':c.EPOCH},'old':SimpleNamespace(CONTROL=Path('/offline/unused-control'))}
    return ns,stamp,messages,joints,params,ros,master,ctx


def fake_message_modules():
    result={}
    for name,kinds in {'sensor_msgs.msg':['JointState'],'std_msgs.msg':['Header','String','Bool'],
        'actionlib_msgs.msg':['GoalStatusArray'],'alicia_flexible_grasp_supervisor.msg':['GraspState'],
        'controller_manager_msgs.srv':['ListControllers']}.items():
        parent=name.split('.')[0];result.setdefault(parent,ModuleType(parent));module=ModuleType(name)
        for kind in kinds:setattr(module,kind,type(kind,(),{}))
        result[name]=module
    return result


class Contracts(unittest.TestCase):
    def simulation_fixture(self,changed=None):
        # Real sealed GEN59 schema, synthetic observer methods: no ROS/process
        # calls. This reproduces the compact-vs-raw schema failure in graph().
        dep=c.read(c.PREVIOUS);nodes=dep['full_runtime_nodes']
        sim=deepcopy(dep['simulation_service'])
        if changed:sim.update(changed)
        units={dep['unit']:dep['core_service'],dep['simulation_unit']:sim,dep['remote_unit']:dep['remote_service']}
        v=SimpleNamespace(properties=lambda unit:deepcopy(units[unit]),process=lambda pid:dep['master_process'],
            configured_nodes=lambda rows,config,core:dict(count=len(rows)),in_group=lambda *args:True,gui_windows=lambda pid:['synthetic'])
        pubs=[('/alicia_controller/follow_joint_trajectory/status',['/bessica_d_hw_interface']),
              ('/hand_controller/follow_joint_trajectory/status',['/bessica_d_hw_interface'])]
        master=SimpleNamespace(getPid=lambda:dep['master_process']['pid'],
            getSystemState=lambda:[pubs,[],[(name+'/fixture_service',[name]) for name in nodes]])
        ctx=dict(v=v,previous=dep,nodes=nodes,config={},old=SimpleNamespace(PUBLISHERS={}),
            legacy=SimpleNamespace(remote_processes=lambda:[nodes[c.REMOTE]]))
        ros=SimpleNamespace(get_name=lambda:'/synthetic_offline_observer')
        with patch.object(c,'identity',side_effect=lambda ctx,master,caller,name:deepcopy(nodes[name])):
            return c.graph(ctx,ros,master,graph_wait=0.)

    def test_real_gen59_raw_simulation_schema_graph_passes(self):
        dep=c.read(c.PREVIOUS)
        self.assertEqual(set(dep['simulation_service']),{'MainPID','ControlGroup','ActiveState','SubState','InvocationID'})
        self.assertEqual(self.simulation_fixture()['simulation_service'],dep['simulation_service'])

    def test_real_gen59_simulation_each_identity_state_drift_rejected(self):
        for key,value in {'MainPID':'999999','ControlGroup':'/changed','InvocationID':'0'*32,
                          'ActiveState':'inactive','SubState':'dead'}.items():
            with self.subTest(key=key):
                with self.assertRaises(RuntimeError):self.simulation_fixture({key:value})

    def test_simulation_old_compact_schema_not_silently_accepted(self):
        with self.assertRaises(RuntimeError):self.simulation_fixture({'pid':139460})

    def test_helper_import_has_no_ros_or_execution(self):
        for path in c.HELPERS:
            tree=ast.parse(path.read_bytes())
            for node in tree.body:
                if isinstance(node,(ast.Import,ast.ImportFrom)):
                    names=[x.name for x in node.names] if isinstance(node,ast.Import) else [node.module]
                    self.assertFalse(any(x.startswith(('rospy','rosgraph')) for x in names))
            self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in
                ('publish','set_param','setParam','deleteParam','start','Popen','run','check_output')
                for n in ast.walk(tree)))
        tree=ast.parse(Path(installer.__file__).read_bytes())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='shutdown']
        self.assertEqual(len(calls),1)
        common=ast.parse(Path(c.__file__).read_bytes())
        proxy=[n for n in ast.walk(common) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='ServiceProxy']
        self.assertEqual(len(proxy),1);self.assertEqual(ast.literal_eval(proxy[0].args[0]),'/controller_manager/list_controllers')

    def test_modern_sdk_gate_ast_only_explicit_stricter_changes(self):
        original=function(c.RUNTIME59,'sample_idle');current=function(c.__file__,'sample_idle')
        # Normalize only generation/selection3, strict parser and explicit GUI
        # latching. Every original SDK/control/timeout/owner branch stays exact.
        old=ast.get_source_segment(c.RUNTIME59.read_text(),original)
        new=ast.get_source_segment(Path(c.__file__).read_text(),current)
        old=old.replace("states['gui'].data is True and master.getParam", "states['gui'].data is True and states['gui']._connection_header.get('latching')=='1' and master.getParam")
        old=old.replace("selection=parse_selection(master.getParam('/grasp_mode/selection'))", "selection=selection_identity(master.getParam('/grasp_mode/selection'))")
        old=old.replace("selection['generation']==2", "selection['generation']==3")
        old=old.replace("require(expected_selection is None or selection==expected_selection,", "require(selection==SELECTION and (expected_selection is None or selection==expected_selection),")
        old=old.replace("for k in states if k not in ('arm','hand')", "for k in states")
        # Two additional statements require unique action-status publisher owners.
        tree=ast.parse(new);body=tree.body[0].body
        protected=next(n for n in body if isinstance(n,ast.Try)).body
        added=[n for n in protected if (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='publishers' for t in n.targets)) or
            (isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and any(isinstance(a,ast.Constant) and a.value=='action status publisher ownership changed' for a in n.value.args))]
        self.assertEqual(len(added),2)
        for n in added:protected.remove(n)
        self.assertEqual(ast.dump(ast.parse(old)),ast.dump(tree))

    def test_positive20_sdk_real_gate_with_synthetic_messages(self):
        data=fixture();result=self.sample(data)
        self.assertEqual(result['sample_count'],20);self.assertEqual(result['selection'],c.SELECTION)
        self.assertEqual(len(result['samples']),20)
        self.assertTrue(all(x['stamp_ns']<=x['received_ns'] for x in result['samples']))

    def sample(self,data):
        with patch.dict(sys.modules,fake_message_modules()),patch.object(c,'recheck',return_value={'revision':67}),patch.object(c,'sha',return_value='synthetic'):
            return c.sample_idle(data[7],data[5],data[6])

    def test_complete_selection_exact_integer_text(self):
        self.assertEqual(c.selection_identity(c.SELECTION),c.selection_identity(dict(c.SELECTION,stamp_ns=str(c.SELECTION['stamp_ns']))))
        for key in c.SELECTION:
            value=dict(c.SELECTION);value.pop(key)
            with self.assertRaises(Exception):c.selection_identity(value)

    def test_original_launch_relative_path_and_default_cwd(self):
        launch=c.W/'.runtime/full_latest_gen36_single_observation_20261016.launch'
        for argument in (str(launch),'.runtime/full_latest_gen36_single_observation_20261016.launch'):
            self.assertEqual(c.check_launch_argument(['/opt/ros/noetic/bin/roslaunch','--skip-log-check',argument],c.W,launch),str(launch))
        for argv,cwd in ((['roslaunch','/tmp/wrong.launch'],c.W),(['roslaunch',str(launch),'/tmp/extra.launch'],c.W),
                         (['roslaunch',str(launch)],Path('/tmp'))):
            with self.assertRaises(RuntimeError):c.check_launch_argument(argv,cwd,launch)
        self.assertEqual(c.default_gateway_cwd({b'HOME':b'/home/example'}),Path('/home/example/.ros'))
        self.assertEqual(c.default_gateway_cwd({b'HOME':b'/home/example',b'ROS_HOME':b'/tmp/ros-custom'}),Path('/tmp/ros-custom'))
        with self.assertRaises(RuntimeError):c.default_gateway_cwd({b'HOME':b'relative'})

    def test_new_gateway_cwd_and_parent_load_configuration_preserved(self):
        before=dict(processes=dict(parent={'pid':1,'cwd':str(c.W)},gateway=dict(cwd='/home/example/.ros',executable='/usr/bin/python3.8',pythonpath=['devel'],environment_sha256='env')))
        c.same_gateway_load_contract(before,deepcopy(before))
        for key,value in (('cwd','/tmp'),('executable','/wrong/python'),('pythonpath',['wrong']),('environment_sha256','other')):
            after=deepcopy(before);after['processes']['gateway'][key]=value
            with self.assertRaises(RuntimeError):c.same_gateway_load_contract(before,after)
        after=deepcopy(before);after['processes']['parent']['pid']=2
        with self.assertRaises(RuntimeError):c.same_gateway_load_contract(before,after)
        for value in (dict(c.SELECTION,extra=1),dict(c.SELECTION,generation=True),dict(c.SELECTION,stamp_ns=float(c.EPOCH)),dict(c.SELECTION,stamp_ns='1e18')):
            with self.assertRaises(Exception):c.selection_identity(value)

    def test_rgb_ledger_and_all_parameters_exact(self):
        master=SimpleNamespace(getParam=lambda _: {'rgb_acquisition':{'ledger':{'phase':'finished'}},'robot':{'a':1}})
        before=c.parameters(master);c.same_parameters(before,deepcopy(before))
        altered=deepcopy(before);altered['parameter_values']['robot']['a']=2
        with self.assertRaises(RuntimeError):c.same_parameters(before,altered)
        for phase in ('reserved','executing'):
            with self.assertRaises(RuntimeError):c.parameters(SimpleNamespace(getParam=lambda _,p=phase:{'rgb_acquisition':{'ledger':{'phase':p}}}))
        with self.assertRaises(RuntimeError):c.parameters(SimpleNamespace(getParam=lambda _:{'rgb_acquisition':{'ledger':'malformed'}}))

    def test_cache_timestamp_hash_and_magic_classification(self):
        with tempfile.TemporaryDirectory(dir=OUTPUT) as folder:
            path=Path(folder)/'held.pyc';raw=b'abc';stat=SimpleNamespace(st_mtime=1234)
            def put(header):path.write_bytes(header+b'code');return c.cache_status(path,raw,stat,importlib.util.MAGIC_NUMBER)
            self.assertFalse(c.cache_status(path,raw,stat,importlib.util.MAGIC_NUMBER)['usable'])
            self.assertTrue(put(importlib.util.MAGIC_NUMBER+struct.pack('<III',0,1234,3))['usable'])
            self.assertFalse(put(importlib.util.MAGIC_NUMBER+struct.pack('<III',0,1233,3))['usable'])
            self.assertFalse(put(importlib.util.MAGIC_NUMBER+struct.pack('<III',0,1234,4))['usable'])
            self.assertTrue(put(importlib.util.MAGIC_NUMBER+struct.pack('<I',1)+b'0'*8)['usable'])
            self.assertTrue(put(importlib.util.MAGIC_NUMBER+struct.pack('<I',3)+b'0'*8)['usable'])
            self.assertFalse(put(b'fail'+struct.pack('<III',0,1234,3))['usable'])

    def test_atomic_install_exact_closure_fixture_only(self):
        with tempfile.TemporaryDirectory(dir=OUTPUT) as folder:
            root=Path(folder);workspace=root/'workspace';candidate=root/'candidate';backup=root/'backup';rows=[]
            relative=c.contract.SELECTOR;target=workspace/relative;source=candidate/relative
            target.parent.mkdir(parents=True);source.parent.mkdir(parents=True)
            target.write_bytes(b'old');source.write_bytes(b'new')
            with patch.object(c,'W',workspace),patch.object(c.contract,'C',candidate):
                row=installer.atomic_install_file(relative,c.sha(target),c.sha(source),backup,lambda a,b:rows.append((a,b)))
                self.assertEqual(target.read_bytes(),b'new');self.assertEqual((backup/relative).read_bytes(),b'old')
                self.assertEqual([a for a,b in rows],['install_intent','install_complete'])
                self.assertEqual(row['path'],relative)
                with self.assertRaises(RuntimeError):installer.atomic_install_file('../escape',None,'bad',backup,lambda *a:None)
                with self.assertRaises(RuntimeError):installer.atomic_install_file(relative,'wrong',c.sha(source),backup,lambda *a:None)

    def test_service_owners_all31_and_instance_required(self):
        names=c.read(c.DESIGN)['constructor_review']['registered_service_names'];ctx={'expected_gateway':{'pid':123}}
        providers=[(n,[c.GATEWAY]) for n in names];master=SimpleNamespace(getSystemState=lambda:[[],[],providers])
        with patch.object(c,'bind'),patch.object(c,'identity',return_value={'pid':123}):
            self.assertEqual(len(c.service_owners(ctx,master)),31)
            providers[0]=(names[0],[c.GATEWAY,'/duplicate'])
            with self.assertRaises(RuntimeError):c.service_owners(ctx,master)
        providers[0]=(names[0],[c.GATEWAY])
        with patch.object(c,'bind'),patch.object(c,'identity',return_value={'pid':124}):
            with self.assertRaises(RuntimeError):c.service_owners(ctx,master)

    def test_release_scope_matches_actual_closed_manifest_without_claiming_full_pass(self):
        manifest=c.read(c.O/'verification/freeze_r2/manifest_closed.json');base=c.read(c.BASE)
        self.assertEqual(len(manifest['source_sha256']),1097)
        self.assertEqual(set(manifest['source_sha256'])-set(base['source_sha256']),c.contract.ADDED)
        self.assertEqual({p for p,h in base['source_sha256'].items() if manifest['source_sha256'][p]!=h},c.contract.PRODUCTION|c.contract.CONTRACTS)
        c.contract.verify_contract_body(c.BASE,c.contract.C/c.contract.CONTRACT)
        self.assertEqual(manifest['source_sha256'][c.contract.SELECTOR],c.contract.SELECTOR_SHA)


def reject_case(name,mutate):
    def test(self):
        data=fixture();mutate(data)
        with self.assertRaises(Exception):self.sample(data)
    test.__name__='test_sdk_reject_'+name
    setattr(Contracts,test.__name__,test)


reject_case('duplicate_arrival',lambda d:setattr(d[3][10].header,'stamp',d[3][9].header.stamp))
reject_case('out_of_order',lambda d:d[3].__setitem__(slice(4,6),list(reversed(d[3][4:6]))))
reject_case('future',lambda d:setattr(d[3][-1].header,'stamp',d[1](d[0]+1)))
reject_case('old_per_message',lambda d:setattr(d[3][0].header,'stamp',d[1](d[0]-200_000_000)))
reject_case('wrong_sdk_owner',lambda d:d[3][0]._connection_header.update(callerid='/other'))
reject_case('wrong_frame',lambda d:setattr(d[3][0].header,'frame_id','predicted'))
reject_case('missing_joint',lambda d:setattr(d[3][-1],'name',['Joint%d'%j for j in range(1,6)]))
reject_case('nonfinite_joint',lambda d:d[3][0].position.__setitem__(0,float('nan')))
reject_case('position_span',lambda d:d[3][0].position.__setitem__(0,.005))
reject_case('gui_unlatched',lambda d:d[2]['/gui/joint_direct_mode']._connection_header.update(latching='0'))
reject_case('gui_owner',lambda d:d[2]['/gui/joint_direct_mode']._connection_header.update(callerid='/fake'))
reject_case('gui_not_manual',lambda d:setattr(d[2]['/gui/joint_direct_mode'],'data',False))
reject_case('task_active',lambda d:setattr(d[2]['/grasp/state'],'active',True))
reject_case('router_pending',lambda d:setattr(d[2]['/grasp_mode/status'],'data','{"pending":true}'))
reject_case('action_active',lambda d:setattr(d[2]['/alicia_controller/follow_joint_trajectory/status'],'status_list',[SimpleNamespace(status=1)]))
reject_case('action_stale',lambda d:setattr(d[2]['/alicia_controller/follow_joint_trajectory/status'].header,'stamp',d[1](d[0]-2_000_000_001)))
reject_case('action_foreign_caller',lambda d:d[2]['/alicia_controller/follow_joint_trajectory/status']._connection_header.update(callerid='/fake'))
reject_case('action_duplicate_owner',lambda d:d[6].getSystemState()[0][0][1].append('/fake'))
reject_case('action_foreign_owner',lambda d:d[6].getSystemState()[0][0][1].__setitem__(0,'/fake'))
reject_case('actuation_not_confirmed',lambda d:setattr(d[2]['/alicia_d/actuation_status'],'data','NOT_READY'))
reject_case('epoch_changed',lambda d:setattr(d[2]['/alicia_d/control_reference_epoch'],'stamp',d[1](c.EPOCH+1)))
reject_case('selection_stamp_changed',lambda d:d[4]['/grasp_mode/selection'].update(stamp_ns=c.SELECTION['stamp_ns']+1))
reject_case('selection_generation_changed',lambda d:d[4]['/grasp_mode/selection'].update(generation=4))
reject_case('reference_pose_changed',lambda d:d[7].update(idle_reference={'names':['Joint%d'%j for j in range(1,7)],'positions':[.005]*6}))


def main():
    global OUTPUT
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    OUTPUT=args.output.resolve();c.storage(OUTPUT);c.require(not OUTPUT.exists(),'new output required');OUTPUT.mkdir()
    guard=OfflineGuard(OUTPUT);sys.addaudithook(guard)
    stream=io.StringIO();started=__import__('time').monotonic()
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(Contracts)
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (OUTPUT/'tests.log').write_text(stream.getvalue())
    summary=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        skipped=len(result.skipped),elapsed_sec=__import__('time').monotonic()-started,helper_sha256=c.helper_map(),
        test_helper_sha256=c.sha(__file__),blocked_operations=guard.blocked,
        scope='pure schema/SDK/parameter/cache/service/atomic fixture contracts plus static effect allowlist',
        production_source_modified=False,ros_calls_performed=False,network_calls_performed=False,motion_commands_sent=False,
        runtime_deployment_verified=False,full_regression_claimed=False)
    c.save(OUTPUT/'result.json',summary);print(json.dumps(summary,ensure_ascii=False))
    if not result.wasSuccessful():raise SystemExit(1)


if __name__=='__main__':main()
