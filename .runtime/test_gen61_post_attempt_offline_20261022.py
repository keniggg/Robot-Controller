"""Offline GEN61 post-idle adapter contracts; never run the live sampler."""
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

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'src/alicia_grasp_modes/src'))
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
OUT = E/'post_idle_adapter_offline_r1/contract_r2'
HELPER = W/'.runtime/read_gen61_post_attempt_20261022.py'
OLD = W/'.runtime/read_gen60_post_attempt_20261022.py'
denied = []

# Python 3.8 uuid imports platform.system(), which otherwise starts `uname -p`.
# Supply only the standard-library platform cache from the same read-only
# kernel syscall, before the all-process guard; no subprocess is launched.
kernel_uname = os.uname()
platform._uname_cache = platform.uname_result(kernel_uname.sysname,kernel_uname.nodename,
    kernel_uname.release,kernel_uname.version,kernel_uname.machine,'')


def guard(event, args):
    forbidden = event.startswith('socket.') or event in (
        'subprocess.Popen','os.system','os.posix_spawn','os.fork','os.exec')
    if event == 'import' and str(args[0]).split('.')[0] in (
            'rospy','rosgraph','rosnode','rosmaster','actionlib'):
        forbidden = True
    if forbidden:
        denied.append(dict(event=event, detail=str(args[0]) if args else ''))
        raise RuntimeError('offline check forbids '+event)


sys.addaudithook(guard)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def set_path(value, keys, replacement):
    for key in keys[:-1]:
        value = value[key]
    value[keys[-1]] = replacement


def rejection(result, name, fn):
    try:
        fn()
    except (RuntimeError, KeyError, ValueError, TypeError, AssertionError) as exc:
        result['negative_cases'].append(dict(name=name, rejected=True, reason=str(exc)))
    else:
        raise AssertionError('negative case accepted: '+name)


def main():
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and E.stat().st_dev == mount.stat().st_dev
    assert not os.statvfs(mount).f_flag & os.ST_RDONLY
    OUT.mkdir(exist_ok=False)
    start = time.monotonic()
    result = dict(kind='GEN61_post_idle_adapter_offline', passed=False,
        actual_attempt_tested=False, live_post_idle_sample_taken=False,
        ROS_initialized=False, live_nodes_contacted=False, motion_commands_sent=False,
        parameters_changed=False, temperature_queried=False, torque_disable_commands_sent=False,
        bootstrap=dict(platform_cache_from_os_uname=True,processor_unused_empty=True,
            subprocess_launched=False,reason='Python3.8 uuid platform import without uname subprocess'),
        fixture_scope='Actual frozen acceptance closure; synthetic return/selection and graph mutations only.',
        positive_cases=[], negative_cases=[], boundary_attempts=denied)
    pins = {str(p):sha(p) for p in (HELPER, OLD, Path(__file__).resolve())}
    try:
        h = load(HELPER, 'gen61_offline_post_idle')
        for path, digest in h.PINS.items():
            assert sha(path) == digest
            pins[str(path)] = digest
        common = load(h.COMMON, 'gen61_offline_post_idle_common')
        args = SimpleNamespace(release=h.RELEASE, release_sha256=h.RELEASE_SHA,
            deployment=h.ACCEPTED, deployment_sha256=h.ACCEPTED_SHA,
            helper_sha256=sha(HELPER), common_sha256=h.COMMON_SHA)
        # Real pinned filesystem closure only. No graph or ROS sampler is called.
        ctx = common.load_context(args, HELPER)
        deployment = ctx['deployment']
        assert len(ctx['nodes']) == 17 and len(ctx['transition']['units']) == 3
        assert len(deployment['preserved']) == 14 and ctx['release']['source_count'] == 1101
        pins.update({str(p):digest for p,digest in ctx['pins'].items()})
        result['positive_cases'].append('actual_release_acceptance_bundle_transition_17_3_14_1101_closure')
        result['accepted_nodes'] = ctx['nodes']
        result['accepted_compute_units'] = ctx['transition']['units']
        result['accepted_epoch_ns'] = deployment['epoch_ns']

        prior = deepcopy(deployment['selection'])
        selected = dict(prior, generation=prior['generation']+1, stamp_ns=int(prior['stamp_ns'])+1)
        entry = dict(kind='GEN61_remote_one_recorded_attempt_recorded',
            original_runner_returned=True, gate_returned_to_original_mode_line=True,
            automatic_retry=False, new_motion_authority_created=False, original_guards_replaced=False,
            temperature_queried=False, physical_success='unverified', original_mode_line=121,
            helper=dict(path=str(h.ENTRY), sha256=h.ENTRY_SHA), runner_returncode=6)
        runner_exit = dict(runner_returncode=6, selection=deepcopy(selected), physical_success='unverified')
        authorization = dict(selection=deepcopy(selected), previous_selection=deepcopy(prior),
            selection_response=dict(success=True),
            selection_commit=dict(latched_selection_matched=True, router_status=dict(selected, active=False)),
            continuation_release_commit=dict(selection=deepcopy(selected), control_revision=67,
                policy='final_checked_authorization_commit_before_pipe_release'),
            operator_authorized_autonomous_grasp=True, recording_scope='local_only', old_plan_reused=False,
            physical_lift_after_close=False, physical_success='unverified', temperature_queried=False,
            torque_disable_authorized=False)
        baseline = (entry, runner_exit, authorization, deployment, 5)
        assert h.returned_selection(*baseline) == selected
        result['positive_cases'].append('returned_failed_runner_still_exact_successor_without_physical_success_claim')
        rc_zero = deepcopy(baseline); rc_zero[0]['runner_returncode'] = rc_zero[1]['runner_returncode'] = 0
        assert h.returned_selection(*rc_zero) == selected
        result['positive_cases'].append('returned_zero_exit_same_exact_successor_without_physical_success_claim')
        decimal = deepcopy(baseline)
        for identity in (decimal[1]['selection'], decimal[2]['selection'], decimal[2]['previous_selection'],
                decimal[2]['selection_commit']['router_status'], decimal[2]['continuation_release_commit']['selection'],
                decimal[3]['selection']):
            identity['stamp_ns'] = str(identity['stamp_ns'])
        assert h.returned_selection(*decimal) == selected
        result['positive_cases'].append('exact_xmlrpc_decimal_stamp_representation')
        mutations = [
            ('not_original_returned',0,('original_runner_returned',),False),
            ('boundary_not_returned',0,('gate_returned_to_original_mode_line',),False),
            ('automatic_retry',0,('automatic_retry',),True),
            ('new_authority',0,('new_motion_authority_created',),True),
            ('original_guards_replaced',0,('original_guards_replaced',),True),
            ('wrong_entry_helper',0,('helper','sha256'),'0'*64),
            ('wrong_original_boundary',0,('original_mode_line',),122),
            ('wrong_entry_kind',0,('kind',),'GEN60_remote_one_recorded_attempt_recorded'),
            ('runner_return_mismatch',1,('runner_returncode',),0),
            ('bool_runner_return',1,('runner_returncode',),True),
            ('exit_claims_physical_success',1,('physical_success',),'verified'),
            ('generation_jump',1,('selection','generation'),6),
            ('same_generation',1,('selection','generation'),4),
            ('stamp_not_later',1,('selection','stamp_ns'),prior['stamp_ns']),
            ('wrong_strategy',1,('selection','strategy'),'lift'),
            ('authorization_selection_missing',2,('selection',),None),
            ('authorization_prior_changed',2,('previous_selection','stamp_ns'),1),
            ('selection_response_failed',2,('selection_response','success'),False),
            ('selection_latch_unacknowledged',2,('selection_commit','latched_selection_matched'),False),
            ('router_commit_stamp_changed',2,('selection_commit','router_status','stamp_ns'),1),
            ('release_commit_selection_changed',2,('continuation_release_commit','selection','generation'),6),
            ('release_commit_control_changed',2,('continuation_release_commit','control_revision'),68),
            ('old_plan_reused',2,('old_plan_reused',),True),
            ('temperature_queried',2,('temperature_queried',),True),
            ('torque_disable_authorized',2,('torque_disable_authorized',),True),
        ]
        for name,index,keys,value in mutations:
            values = deepcopy(baseline);set_path(values[index],keys,value)
            rejection(result,name,lambda v=values:h.returned_selection(*v))
        rejection(result,'wrong_expected_generation',lambda:h.returned_selection(*baseline[:-1],6))

        # This path is a value fixture only: no scene or recording is created.
        scene = h.ATTEMPTS/'scene_start_offline_value_fixture_never_created'
        preflight_path = scene/'preflight.json'
        synthetic_preflight_sha = 'a'*64
        scene_auth = dict(authorization, session_dir=str(scene),
            continuing_authorization=dict(path=str(common.A),sha256=common.AUTH_SHA),
            operator_observation=dict(runtime_deployment=dict(path=str(h.ACCEPTED),sha256=h.ACCEPTED_SHA),
                release=dict(path=str(h.RELEASE),sha256=h.RELEASE_SHA),
                preflight=dict(path=str(preflight_path),sha256=synthetic_preflight_sha),
                control_revision=67,control_sha256=common.CONTROL_SHA,valid_until_ns=1))
        preflight = dict(epoch_ns=deployment['epoch_ns'], selection=deepcopy(prior), receipt_ns=1)
        scene_base = (scene_auth,preflight,deployment,scene,{str(preflight_path):synthetic_preflight_sha},common)
        h.scene_predecessor(*scene_base)
        result['positive_cases'].append('historical_preflight_same_epoch_no_scene_deadline_refresh_or_freshness_claim')
        for name,index,keys,value in (
                ('preflight_epoch_changed',1,('epoch_ns',),deployment['epoch_ns']+1),
                ('preflight_selection_changed',1,('selection','generation'),5),
                ('scene_session_changed',0,('session_dir',),str(scene.parent/'scene_other')),
                ('scene_other_release',0,('operator_observation','release','sha256'),'b'*64),
                ('scene_other_deployment',0,('operator_observation','runtime_deployment','sha256'),'b'*64),
                ('scene_other_preflight',0,('operator_observation','preflight','sha256'),'b'*64),
                ('scene_other_control',0,('operator_observation','control_sha256'),'b'*64),
                ('scene_other_authorization',0,('continuing_authorization','sha256'),'b'*64)):
            values = list(scene_base);values[index] = deepcopy(values[index]);set_path(values[index],keys,value)
            rejection(result,name,lambda v=values:h.scene_predecessor(*v))

        graph_calls = []
        graph = deepcopy(deployment['after'])
        def fake_graph(*args, **kwargs):
            graph_calls.append((args,kwargs))
            return graph
        fake_ctx = dict(ctx,runtime=SimpleNamespace(graph=fake_graph))
        assert h.accepted_graph(common,fake_ctx,None,None) == graph
        assert graph_calls[-1][0][4:] == (deployment['targets'],ctx['transition']['units'],deployment['auxiliary_nodes'])
        result['positive_cases'].append('full_three_unit_graph_adapter_exact_arguments_and_action_owners')
        graph_mutations = (
            ('changed_retained_driver',('node_identities','/alicia_d_driver_node','pid'),1),
            ('changed_compute_process',('node_identities','/grasp_task_node','start_ticks'),'1'),
            ('changed_compute_service',('services','/grasp_task_node','InvocationID'),'changed'),
            ('foreign_action_publisher',('publishers','/alicia_controller/follow_joint_trajectory/status'),['/alien']),
            ('duplicate_action_publisher',('publishers','/hand_controller/follow_joint_trajectory/status'),['/bessica_d_hw_interface','/alien']),
        )
        for name,keys,value in graph_mutations:
            graph = deepcopy(deployment['after']);set_path(graph,keys,value)
            rejection(result,name,lambda:h.accepted_graph(common,fake_ctx,None,None))

        policy = {'/alicia_d_driver_node/software_temperature_monitoring_enabled':False,
            '/alicia_d_driver_node/temperature_poll_rate_hz':0.,'/grasp/require_actuation_confirmation':True,
            '/grasp/unknown_lift_after_close_enabled':False,'/grasp/unknown_observe_return_max_observations':1}
        requested = []
        def get_param(key):
            requested.append(key)
            return policy[key]
        proxy = SimpleNamespace(get_param=get_param)
        assert h.readonly_policy(proxy) == policy and set(requested) == set(policy)
        result['positive_cases'].append('configuration_only_temperature_disabled_reads')
        for key,value in tuple(policy.items()):
            replacement = not value if type(value) is bool else 2
            policy[key] = replacement
            rejection(result,'changed_policy_'+key,lambda:h.readonly_policy(proxy))
            policy[key] = value

        old, new = OLD.read_text(), HELPER.read_text()
        start_marker = "        selection_before = selection_identity(rospy.get_param('/grasp_mode/selection'))"
        end_marker = '        after = identities(nodes)'
        old_block = old[old.index(start_marker):old.index(end_marker)]
        new_block = new[new.index(start_marker):new.index(end_marker)]
        assert old_block == new_block
        result['original_sampling_block'] = dict(byte_identical=True,bytes=len(old_block.encode()),
            sha256=hashlib.sha256(old_block.encode()).hexdigest(),
            covers='exact selection, 20 SDK source samples and ages/spans, same epoch, GUI-owned manual, task/router idle, action status/freshness, CONFIRMED and enabled/ready')
        old_tree,new_tree = ast.parse(old),ast.parse(new)
        old_id = next(n for n in old_tree.body if isinstance(n,ast.FunctionDef) and n.name=='identities')
        new_id = next(n for n in new_tree.body if isinstance(n,ast.FunctionDef) and n.name=='identities')
        assert ast.dump(old_id,include_attributes=False) == ast.dump(new_id,include_attributes=False)
        forbidden_calls = {'set_param','delete_param','Publisher','publish','ServiceProxy','send_goal','deploy','accept','sample','idle_function'}
        assert not [(n.lineno,n.func.attr) for n in ast.walk(new_tree) if isinstance(n,ast.Call)
            and isinstance(n.func,ast.Attribute) and n.func.attr in forbidden_calls]
        result['positive_cases'].append('original_sdk_manual_idle_action_block_byte_exact_and_proc_identity_ast_exact')
        result['positive_cases'].append('no_parameter_write_publish_service_control_or_generation4_sampler_calls')
        assert not denied
        assert not set(sys.modules).intersection({'rospy','rosgraph','rosnode','rosmaster','actionlib'})
        assert all(sha(path)==digest for path,digest in pins.items())
        common.recheck(ctx)
        result.update(passed=True,positive_count=len(result['positive_cases']),negative_count=len(result['negative_cases']),
            actual_source_files_verified=ctx['release']['source_count'])
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        result.update(input_sha256=pins,elapsed_sec=time.monotonic()-start,recorded_ns=time.time_ns())
        for src,name in ((HELPER,'helper_source.py'),(OLD,'original_source.py'),(Path(__file__),'test_source.py')):
            with (OUT/name).open('xb') as stream:
                stream.write(src.read_bytes());stream.flush();os.fsync(stream.fileno())
        raw=(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
        with (OUT/'result.json').open('xb') as stream:
            stream.write(raw);stream.flush();os.fsync(stream.fileno())
        assert (OUT/'result.json').read_bytes()==raw
    print(json.dumps({k:result.get(k) for k in ('passed','positive_count','negative_count','elapsed_sec','error')},ensure_ascii=False))
    print(json.dumps(dict(result=str(OUT/'result.json'),sha256=sha(OUT/'result.json')),ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
