"""Offline GEN60 attempt-adapter contracts; never imports ROS or invokes a runner."""
import argparse
import ast
from copy import deepcopy
import hashlib
import importlib
import importlib.abc
import json
import os
from pathlib import Path
import platform
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
E60 = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1')
OLD_SHA = {
    'capture':'ee63f76649c65e53cff3a93fb1bcce5f7e1c5751ee08a90cd2716e38dfe2e8f4',
    'common':'01fdf3f476d400c2fcd71d6823b9f1f22d817697fc90a062b2417bbfa94b1c31',
    'entry':'8830e63c4f10ef403b0de43fdf31a671eb130ce1910d050e1e0e1dbfc72a9865',
    'gate':'5b3906092580de8a1c42678c9a3726ff497169548c1069e0fc01dd280637ae67',
    'receipt':'c2d242fb385a53f4acf0c2e46ede1365abfff316116fee2606262986d82121ec',
}
ATTEMPT_IMPORTS = {}
BOUNDARY_ATTEMPTS = []


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def blocked(*args, **kwargs):
    BOUNDARY_ATTEMPTS.append('network_or_subprocess')
    raise AssertionError('offline adapter test attempted an external boundary')


class NoRos(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('rospy','rosgraph','rosservice','moveit_commander'):
            BOUNDARY_ATTEMPTS.append('import:'+fullname)
            raise AssertionError('ROS import forbidden in offline adapter tests')


def fixture():
    """Synthetic schema values, not a deployment or future-motion receipt."""
    names = ['/motion_gateway','/remote_grasp6d_node','/alicia_supervisor_gui',
             '/alicia_d_driver_node'] + ['/retained_fixture_'+str(i) for i in range(13)]
    before = {n:dict(pid=100+i,parent_pid=42,start_ticks=1000+i) for i,n in enumerate(names)}
    previous = dict(full_runtime_nodes=deepcopy(before),epoch_ns=1792639534222874592,
        remote_unit='synthetic-retained-gen59-remote',remote_service={'MainPID':'101'},
        startup_configuration='synthetic-retained-construction',startup_configuration_sha256='c'*64,
        runtime_import_attestation='synthetic-retained-import',runtime_import_attestation_sha256='a'*64,
        remote_dependency=dict(startup_binding='synthetic-retained-startup',startup_binding_sha256='b'*64))
    post = dict(full_runtime_nodes_before=deepcopy(before),full_runtime_nodes_after=deepcopy(before),
        epoch_ns=previous['epoch_ns'],selection=dict(mode='unknown',strategy='observe_return',
        generation=3,stamp_ns=1792644272392787456))
    sources = {'synthetic/gateway_consumer.py':'d'*64,'synthetic/retained_remote.py':'e'*64}
    release = dict(generation='GEN60',passed=True,reviewed=True,installed=True,full_regression_passed=True,
        source_count=len(sources),source_sha256=sources,
        test_summary=dict(tests=37,failures=0,errors=0,skipped=0))
    nodes = deepcopy(before); nodes['/motion_gateway'].update(pid=300,start_ticks=3000)
    preserved = {n:deepcopy(r) for n,r in before.items() if n != '/motion_gateway'}
    dep = dict(generation='GEN60',deployment_passed=True,whole_patch_deployed=True,
        other_sixteen_processes_preserved=True,epoch_preserved=True,gateway_original_parent_preserved=True,
        full_restart=False,all_processes_replaced=False,
        deployment_kind='original_parent_gateway_respawn_sixteen_preserved',
        source_sha256=deepcopy(sources),test_summary=deepcopy(release['test_summary']),
        full_runtime_nodes=nodes,all_registered_nodes=deepcopy(nodes),final_all_registered_nodes=deepcopy(nodes),
        before=deepcopy(before),preserved_before=preserved,preserved_after=deepcopy(preserved),
        replaced_before={'/motion_gateway':deepcopy(before['/motion_gateway'])},
        replaced_after={'/motion_gateway':deepcopy(nodes['/motion_gateway'])},
        new_gateway_identity=deepcopy(nodes['/motion_gateway']),replaced_count=1,preserved_count=16,
        epoch_ns=previous['epoch_ns'],selection=deepcopy(post['selection']),manual_mode=True,
        task_active=False,actions_active=False,actuation_status='CONFIRMED: synthetic offline fixture',
        gui_owning_manual_ack=dict(callerid='/alicia_supervisor_gui',manual=True),
        composite_runtime=dict(gateway_generation='GEN60',remote_generation='GEN59',remote_reloaded=False,
            remote_preload_attestation_retained=True,changed_source_consumers=['/motion_gateway']),
        gateway_source_load_proof=dict(same_process_module_attestation=False),
        automatic_attempt_authorized=False,fresh_scene_and_dependency_gate_required_before_automatic_attempt=True)
    for name in ('remote_unit','remote_service','startup_configuration','startup_configuration_sha256',
                 'runtime_import_attestation','runtime_import_attestation_sha256','remote_dependency'):
        dep[name] = deepcopy(previous[name])
    return release, dep, previous, post, before


class Contracts(unittest.TestCase):
    def test_synthetic_positive_preserves_remote_and_complete_partition(self):
        values = fixture(); before = deepcopy(values)
        result = common.validate_acceptance_values(*values)
        self.assertEqual(result, values[1]['full_runtime_nodes'])
        self.assertEqual(result['/remote_grasp6d_node'],values[4]['/remote_grasp6d_node'])
        self.assertEqual(values,before)

    def test_selection_exact_integer_and_decimal_text_agree(self):
        a = fixture()[3]['selection']; b = dict(a,stamp_ns=str(a['stamp_ns']))
        self.assertTrue(common.same_selection(a,b))
        self.assertEqual(common.selection_identity(a),common.selection_identity(b))
        self.assertFalse(common.same_selection(a,dict(a,generation=4)))
        self.assertFalse(common.same_selection(a,dict(a,stamp_ns=a['stamp_ns']+1)))

    def test_selection_missing_extra_bool_and_lossy_stamp_rejected(self):
        a = fixture()[3]['selection']
        bad = [dict(a,extra=0),{k:v for k,v in a.items() if k!='strategy'},dict(a,generation=True),
               dict(a,stamp_ns=float(a['stamp_ns'])),dict(a,stamp_ns=-1),dict(a,stamp_ns='1.0'),
               dict(a,stamp_ns=None),json.dumps(a)]
        for value in bad:
            with self.subTest(value=value):
                with self.assertRaises((ValueError,RuntimeError)):
                    common.selection_identity(value)

    def test_four_flow_helpers_have_only_mechanical_and_two_action_owner_changes(self):
        for part in ('capture','receipt','gate','entry'):
            old = W/'.runtime'/('gen59_remote_attempt_'+part+'_20261022.py')
            new = W/'.runtime'/('gen60_gateway_attempt_'+part+'_20261022.py')
            self.assertEqual(sha(old),OLD_SHA[part])
            expected = old.read_text().replace('gen59_remote_attempt','gen60_gateway_attempt').replace(
                'GEN59_','GEN60_').replace('gen59_original_unchanged_runner','gen60_original_unchanged_runner')
            if part=='capture':
                expected = expected.replace('current remote-reloaded-stack readiness','current gateway-reloaded-stack readiness')
                expected = expected.replace('    CAPTURE, CAPTURE_SHA, DRIVER, GUI, S, W,',
                    '    ACTION_OWNER, CAPTURE, CAPTURE_SHA, DRIVER, GUI, S, W,')
            if part=='gate':
                expected = expected.replace('    A, CHECKER, CHECKER_SHA, CONTROL, CONTROL_SHA, DRIVER, E, GUI, Transport, W,',
                    '    A, ACTION_OWNER, CHECKER, CHECKER_SHA, CONTROL, CONTROL_SHA, DRIVER, E, GUI, Transport, W,')
            if part in ('capture','gate'):
                for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
                    expected = expected.replace("'"+topic+"', GoalStatusArray, None)",
                                                "'"+topic+"', GoalStatusArray, ACTION_OWNER)")
            if part=='entry':
                expected = expected.replace('One recorded remote-reload-session attempt','One recorded gateway-reload-session attempt')
                expected = expected.replace(OLD_SHA['capture'],sha(W/'.runtime/gen60_gateway_attempt_capture_20261022.py'))
                expected = expected.replace(OLD_SHA['receipt'],sha(W/'.runtime/gen60_gateway_attempt_receipt_20261022.py'))
            self.assertEqual(new.read_text(),expected,part)

    def test_original_common_safety_helpers_remain_exact(self):
        path = W/'.runtime/gen59_remote_attempt_common_20261022.py'
        self.assertEqual(sha(path),OLD_SHA['common'])
        old = {x.name:ast.dump(x,include_attributes=False) for x in ast.parse(path.read_text()).body
               if isinstance(x,(ast.FunctionDef,ast.ClassDef))}
        new = {x.name:ast.dump(x,include_attributes=False) for x in ast.parse(Path(common.__file__).read_text()).body
               if isinstance(x,(ast.FunctionDef,ast.ClassDef))}
        for name in ('require','selection_identity','same_selection','sha','read','ref','new_json','parser',
                     'checked_module','recheck','Transport','input_receipts'):
            self.assertEqual(old[name],new[name],name)
        self.assertEqual(common.S,E60/'actual_attempt_r1')

    def test_runtime_calls_use_retained_remote_and_gateway_graph_api(self):
        # API structure is checked without importing or running deployment helpers.
        tree = ast.parse(Path(common.__file__).read_text())
        load = next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='load_context')
        methods = [x.func.attr for x in ast.walk(load) if isinstance(x,ast.Call) and
                   isinstance(x.func,ast.Attribute) and isinstance(x.func.value,ast.Name) and x.func.value.id=='runtime']
        self.assertEqual(methods,['accepted_context'])
        live = next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='live_identity')
        calls = [x for x in ast.walk(live) if isinstance(x,ast.Call) and isinstance(x.func,ast.Attribute) and x.func.attr=='graph']
        self.assertEqual(len(calls),1)
        self.assertEqual([x.arg for x in calls[0].keywords],['gateway','auxiliaries','graph_wait'])

    def test_original_runner_and_dependency_helpers_are_byte_exact(self):
        entry = ATTEMPT_IMPORTS['entry']
        for name in ('ORIGINAL','CONTINUATION','PREWARM','RECORD_WRAPPER','RECORDER','ARCHIVE'):
            self.assertEqual(sha(getattr(entry,name)),getattr(entry,name+'_SHA'))
        self.assertEqual(sha(common.CAPTURE),common.CAPTURE_SHA)
        self.assertEqual(sha(common.CHECKER),common.CHECKER_SHA)
        self.assertIsInstance(entry.original_boundary(),int)

    def test_line_gate_failure_prevents_synthetic_boundary_and_restores_trace(self):
        path = TEMP/'synthetic_line_gate.py'; text='def run():\n    sink.append("boundary")\n'
        path.write_text(text); namespace = {'sink':[]}; exec(compile(text,str(path),'exec'),namespace)
        class Rejected(Exception): pass
        def reject(frame):raise Rejected('synthetic rejection')
        before = sys.gettrace()
        with self.assertRaises(Rejected):
            ATTEMPT_IMPORTS['entry'].run_with_line_gate(path,sha(path),2,reject,namespace['run'])
        self.assertEqual(namespace['sink'],[])
        self.assertIs(sys.gettrace(),before)

    def test_line_gate_visits_exactly_once_for_synthetic_file(self):
        path = TEMP/'synthetic_line_gate_pass.py'; text='def run():\n    sink.append("boundary")\n'
        path.write_text(text); namespace={'sink':[]};exec(compile(text,str(path),'exec'),namespace)
        calls=[]
        ATTEMPT_IMPORTS['entry'].run_with_line_gate(path,sha(path),2,lambda frame:calls.append('gate'),namespace['run'])
        self.assertEqual(calls,['gate']);self.assertEqual(namespace['sink'],['boundary'])

    def test_action_status_publishers_from_accepted_owner_pass(self):
        publishers={topic:[common.ACTION_OWNER] for topic in common.ACTION_STATUS_TOPICS}
        common.require_action_status_publishers(publishers,{common.ACTION_OWNER:{'pid':101}})

    def test_action_status_owner_must_be_an_accepted_process(self):
        publishers={topic:[common.ACTION_OWNER] for topic in common.ACTION_STATUS_TOPICS}
        with self.assertRaises(RuntimeError):common.require_action_status_publishers(publishers,{})


def reject_case(name, mutation):
    def test(self):
        values=fixture();mutation(*values)
        with self.assertRaises((RuntimeError,ValueError,KeyError)):
            common.validate_acceptance_values(*values)
    test.__name__='test_reject_'+name
    setattr(Contracts,test.__name__,test)


MUTATIONS = {
 'release_generation':lambda r,d,p,s,b:r.update(generation='GEN59'),
 'release_unfinished_full':lambda r,d,p,s,b:r.update(full_regression_passed=False),
 'release_count':lambda r,d,p,s,b:r.update(source_count=1094),
 'release_skipped_case':lambda r,d,p,s,b:r['test_summary'].update(skipped=1),
 'release_bool_case_count':lambda r,d,p,s,b:r['test_summary'].update(tests=True),
 'deployment_generation':lambda r,d,p,s,b:d.update(generation='GEN59'),
 'deployment_kind':lambda r,d,p,s,b:d.update(deployment_kind='GEN59_remote_only_sixteen_processes_preserved'),
 'full_restart_claim':lambda r,d,p,s,b:d.update(full_restart=True),
 'all_replaced_claim':lambda r,d,p,s,b:d.update(all_processes_replaced=True),
 'unaccepted_deployment':lambda r,d,p,s,b:d.update(deployment_passed=False),
 'incomplete_patch':lambda r,d,p,s,b:d.update(whole_patch_deployed=False),
 'preserved_flag':lambda r,d,p,s,b:d.update(other_sixteen_processes_preserved=False),
 'gateway_parent_flag':lambda r,d,p,s,b:d.update(gateway_original_parent_preserved=False),
 'source_mismatch':lambda r,d,p,s,b:d['source_sha256'].update({'synthetic/gateway_consumer.py':'f'*64}),
 'summary_mismatch':lambda r,d,p,s,b:d['test_summary'].update(tests=36),
 'missing_owner':lambda r,d,p,s,b:d['full_runtime_nodes'].pop('/alicia_d_driver_node'),
 'extra_owner':lambda r,d,p,s,b:d['full_runtime_nodes'].update({'/extra':{'pid':999}}),
 'final_inventory_changed':lambda r,d,p,s,b:d['final_all_registered_nodes']['/remote_grasp6d_node'].update(pid=999),
 'preserved_owner_changed':lambda r,d,p,s,b:d['preserved_after']['/alicia_d_driver_node'].update(pid=999),
 'gateway_parent_changed':lambda r,d,p,s,b:d['full_runtime_nodes']['/motion_gateway'].update(parent_pid=43),
 'replacement_count':lambda r,d,p,s,b:d.update(replaced_count=2),
 'preserved_count':lambda r,d,p,s,b:d.update(preserved_count=15),
 'wrong_gateway_identity':lambda r,d,p,s,b:d['new_gateway_identity'].update(pid=301),
 'remote_unit_changed':lambda r,d,p,s,b:d.update(remote_unit='synthetic-new-remote'),
 'remote_service_changed':lambda r,d,p,s,b:d['remote_service'].update(MainPID='999'),
 'epoch_changed':lambda r,d,p,s,b:d.update(epoch_ns=d['epoch_ns']+1),
 'selection_generation_changed':lambda r,d,p,s,b:d['selection'].update(generation=4),
 'selection_stamp_changed':lambda r,d,p,s,b:d['selection'].update(stamp_ns=d['selection']['stamp_ns']+1),
 'selection_incomplete':lambda r,d,p,s,b:d['selection'].pop('strategy'),
 'manual_false':lambda r,d,p,s,b:d.update(manual_mode=False),
 'task_active':lambda r,d,p,s,b:d.update(task_active=True),
 'action_active':lambda r,d,p,s,b:d.update(actions_active=True),
 'driver_pending':lambda r,d,p,s,b:d.update(actuation_status='PENDING: synthetic'),
 'gui_ack_owner':lambda r,d,p,s,b:d['gui_owning_manual_ack'].update(callerid='/other'),
 'remote_construction_relabelled':lambda r,d,p,s,b:d.update(startup_configuration='synthetic-new-gen60-remote'),
 'remote_import_relabelled':lambda r,d,p,s,b:d.update(runtime_import_attestation_sha256='f'*64),
 'remote_binding_relabelled':lambda r,d,p,s,b:d['remote_dependency'].update(startup_binding_sha256='f'*64),
 'remote_generation_relabelled':lambda r,d,p,s,b:d['composite_runtime'].update(remote_generation='GEN60'),
 'remote_reloaded_claim':lambda r,d,p,s,b:d['composite_runtime'].update(remote_reloaded=True),
 'unproven_memory_attestation_claim':lambda r,d,p,s,b:d['gateway_source_load_proof'].update(same_process_module_attestation=True),
 'reload_grants_future_authority_claim':lambda r,d,p,s,b:d.update(automatic_attempt_authorized=True),
 'fresh_scene_gate_removed_claim':lambda r,d,p,s,b:d.update(fresh_scene_and_dependency_gate_required_before_automatic_attempt=False),
}
for name,mutation in MUTATIONS.items():reject_case(name,mutation)


def callback_owner_loop(part):
    """Use the actual topic declaration and callback-validation loop, without ROS imports."""
    path=W/'.runtime'/('gen60_gateway_attempt_'+part+'_20261022.py')
    tree=ast.parse(path.read_text())
    if part=='capture':
        declaration=check=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    else:
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CurrentFeedback')
        declaration=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
        check=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='check')
    assignment=next(n for n in ast.walk(declaration) if isinstance(n,ast.Assign) and any(
        isinstance(t,ast.Name) and t.id=='topics' or isinstance(t,ast.Attribute) and t.attr=='topics'
        for t in n.targets))
    namespace={n:object for n in ('JointState','Header','GraspState','String','Bool','GoalStatusArray')}
    namespace.update(DRIVER=common.DRIVER,GUI=common.GUI,ACTION_OWNER=common.ACTION_OWNER)
    topics=eval(compile(ast.Expression(assignment.value),str(path),'eval'),namespace)
    loop=next(n for n in ast.walk(check) if isinstance(n,ast.For) and isinstance(n.target,ast.Tuple)
              and any(isinstance(x,ast.Name) and x.id=='owner' for x in n.target.elts))
    code=compile(ast.Module(body=[loop],type_ignores=[]),str(path),'exec')
    return topics,code


def action_callback_case(part, key, foreign):
    def test(self):
        topics,code=callback_owner_loop(part)
        owners={k:owner for k,_,_,owner in topics}
        self.assertEqual(owners['arm'],common.ACTION_OWNER)
        self.assertEqual(owners['hand'],common.ACTION_OWNER)
        state={k:(SimpleNamespace(_connection_header={'callerid':owner}),1_000_000_000)
               for k,_,_,owner in topics}
        if foreign is not None:state[key][0]._connection_header={'callerid':foreign}
        values=dict(require=common.require,state=state,topics=topics,self=SimpleNamespace(topics=topics),now=1_000_000_000)
        if foreign is None:exec(code,values)
        else:
            with self.assertRaises(RuntimeError):exec(code,values)
    suffix='positive' if foreign is None else key+'_'+('missing' if foreign=='' else 'foreign')
    setattr(Contracts,'test_'+part+'_actual_action_callback_loop_'+suffix,test)


for part in ('capture','gate'):
    action_callback_case(part,'arm',None)
    for key in ('arm','hand'):
        for foreign in ('/foreign_action_server',''):
            action_callback_case(part,key,foreign)


def action_publisher_case(index, owners, label):
    def test(self):
        topics=common.ACTION_STATUS_TOPICS
        publishers={topic:[common.ACTION_OWNER] for topic in topics}
        if owners is None:publishers.pop(topics[index])
        else:publishers[topics[index]]=owners
        # Execute the actual live_identity glue with a pure fake graph; this
        # proves that the unique-publisher check is connected to both callers.
        nodes={common.ACTION_OWNER:{'pid':101},'/motion_gateway':{'pid':102}}
        graph=dict(node_identities=nodes,publishers=publishers)
        runtime=SimpleNamespace(GATEWAY='/motion_gateway',graph=lambda *a,**k:graph)
        ctx=dict(deployment=dict(remote_unit='synthetic-unit',remote_service={'id':1},auxiliary_nodes={}),
                 runtime=runtime,runtime_context={},nodes=nodes,
                 verifier=SimpleNamespace(properties=lambda unit:{'id':1}))
        with self.assertRaises(RuntimeError):common.live_identity(ctx,None,None)
    setattr(Contracts,'test_live_identity_action_publisher_'+str(index)+'_'+label,test)


for index in (0,1):
    for owners,label in ((None,'missing'),([], 'empty'),(['/foreign_action_server'],'foreign'),
                         (['/bessica_d_hw_interface','/foreign_action_server'],'duplicate_foreign'),
                         (['/bessica_d_hw_interface','/bessica_d_hw_interface'],'duplicate_same')):
        action_publisher_case(index,owners,label)


def main():
    global common,TEMP
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and E60 in out.parents and not out.exists()
    out.mkdir();TEMP=out/'tmp';TEMP.mkdir()
    sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
    guard=NoRos();sys.meta_path.insert(0,guard)
    before_sources={str(W/'.runtime'/('gen60_gateway_attempt_'+p+'_20261022.py')):
        sha(W/'.runtime'/('gen60_gateway_attempt_'+p+'_20261022.py')) for p in OLD_SHA}
    started=time.monotonic()
    # Python 3.8 uuid asks platform.system(), whose uncached implementation
    # launches uname -p. Supply the same kernel value through os.uname so that
    # even this standard-library import needs no child process.
    with mock.patch.object(platform,'system',lambda:os.uname().sysname),mock.patch.object(socket,'socket',blocked),mock.patch.object(socket,'create_connection',blocked),mock.patch.object(subprocess,'Popen',blocked):
        for part in ('common','capture','receipt','gate','entry'):
            ATTEMPT_IMPORTS[part]=importlib.import_module('gen60_gateway_attempt_'+part+'_20261022')
            assert Path(ATTEMPT_IMPORTS[part].__file__).resolve()==W/'.runtime'/('gen60_gateway_attempt_'+part+'_20261022.py')
        common=ATTEMPT_IMPORTS['common']
        suite=unittest.defaultTestLoader.loadTestsFromTestCase(Contracts)
        with (out/'unittest.log').open('x') as log:
            result=unittest.TextTestRunner(stream=log,verbosity=2).run(suite)
    sys.meta_path.remove(guard)
    unchanged=all(sha(p)==h for p,h in before_sources.items())
    selection=importlib.import_module('alicia_grasp_modes.selection')
    selection_path=Path(selection.__file__).resolve()
    assert selection_path==W/'src/alicia_grasp_modes/src/alicia_grasp_modes/selection.py'
    value=dict(kind='GEN60_attempt_adapter_offline_value_and_unchanged_flow_contracts',
        passed=result.wasSuccessful() and unchanged and not BOUNDARY_ATTEMPTS,
        tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),
        elapsed_sec=time.monotonic()-started,helper_path=str(Path(__file__).resolve()),helper_sha256=sha(__file__),
        source_sha256=before_sources,source_unchanged=unchanged,
        selection_parser_path=str(selection_path),selection_parser_sha256=sha(selection_path),
        historical_helper_sha256=OLD_SHA,synthetic_binding_values_not_deployment_receipts=True,
        scope='pure acceptance values, complete selection parser, exact mechanical plus two action-owner diff, actual callback-owner loops and live_identity glue with fake graph, unchanged original gates, synthetic line tracing',
        live_acceptance_not_run=True,full_runtime_release_validation_not_run=True,
        external_boundary_attempts=BOUNDARY_ATTEMPTS,ROS_imported=False,remote_health_contacted=False,
        hardware_contacted=False,motion_commands_sent=False,new_authority_created=False,
        failure_details=[{'case':str(t),'traceback':s} for t,s in result.failures+result.errors])
    raw=(json.dumps(value,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
    with (out/'result.json').open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert (out/'result.json').read_bytes()==raw
    print(json.dumps({'path':str(out/'result.json'),'sha256':sha(out/'result.json'),'tests':result.testsRun,'passed':value['passed']}))
    return 0 if value['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
