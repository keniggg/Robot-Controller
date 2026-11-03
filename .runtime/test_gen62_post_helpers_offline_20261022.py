"""Pure offline contract checks; no live helper main, ROS or future evidence."""
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
F = E/'contact_feedback_fix_r1'
OUT = F/'post_helpers_offline_r1'
POST = W/'.runtime/read_gen62_post_attempt_20261022.py'
MANUAL = W/'.runtime/restore_gen62_after_handoff_20261022.py'
EXPECTED = {POST:'fb65dde02e80636774ec677a21cbc595c82b41d396a27fb9548f36743456f7c0',
    MANUAL:'80bdd39487363353f4dadf45b149843c425a1e38bc030ff0a6da29405873a281'}
denied = []
u = os.uname()
platform._uname_cache = platform.uname_result(u.sysname,u.nodename,u.release,u.version,u.machine,'')


def guard(event, args):
    forbidden = event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.posix_spawn','os.fork','os.exec')
    if event == 'import' and str(args[0]).split('.')[0] in ('rospy','rosgraph','rosnode','rosmaster','actionlib'):
        forbidden = True
    if forbidden:
        denied.append(event)
        raise RuntimeError('offline boundary: '+event)


sys.addaudithook(guard)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path, name):
    spec = importlib.util.spec_from_file_location(name,path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def reject(result, name, fn):
    try:
        fn()
    except (AssertionError, RuntimeError, KeyError, ValueError, TypeError) as exc:
        result['negative_cases'].append(dict(name=name,rejected=True,reason=str(exc)))
    else:
        raise AssertionError('negative accepted: '+name)


def put(value, keys, replacement):
    for key in keys[:-1]:
        value = value[key]
    value[keys[-1]] = replacement


class Stamp:
    def __init__(self, ns): self.ns = ns
    def to_nsec(self): return self.ns


def snapshot_fixture(manual):
    epoch, now = 1_000_000_000, 5_000_000_000
    names = ['Joint%d'%i for i in range(1,7)]
    samples = [dict(stamp_ns=now-1_950_000_000+i*100_000_000,
        received_ns=now-1_950_000_000+i*100_000_000+5_000_000,
        caller=manual.DRIVER,frame_id='sdk_measured',names=list(names),positions=[0.1]*6) for i in range(20)]
    states = {}
    for key,owner in manual.OWNERS.items():
        msg = SimpleNamespace(_connection_header=dict(callerid=owner,latching='1'),data=True,
            stamp=Stamp(epoch),header=SimpleNamespace(stamp=Stamp(now-10_000_000)),status_list=[],active=False)
        if key == 'mode': msg.data = json.dumps(dict(active=False,pending=False,start_reserved=False))
        if key == 'actuation': msg.data = 'CONFIRMED: fixture'
        states[key] = (msg,now-10_000_000)
    return samples,states,now,epoch


def main():
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and F.stat().st_dev == mount.stat().st_dev
    OUT.mkdir(exist_ok=False)
    start = time.monotonic()
    result = dict(kind='GEN62_post_helpers_offline_contracts',passed=False,
        ROS_initialized=False,live_nodes_contacted=False,controls_written=False,
        temperature_queried=False,automatic_attempt_authorized=False,
        actual_GEN62_release_acceptance_checked=False,
        future_evidence_scope='Explicit CLI pins mandatory; future files not fabricated or self-hashed.',
        positive_cases=[],negative_cases=[],boundary_attempts=denied,input_sha256={str(p):h for p,h in EXPECTED.items()})
    try:
        assert all(sha(p)==h for p,h in EXPECTED.items())
        h,m = load(POST,'offline_gen62_post'),load(MANUAL,'offline_gen62_manual')
        assert m.POST_SHA == EXPECTED[POST]
        for p,digest in h.PINS.items(): assert sha(p)==digest
        assert sha(m.BASELINE)==m.BASELINE_SHA
        old = (W/'.runtime/read_gen61_post_attempt_20261022.py').read_text()
        current = POST.read_text()
        sdk = lambda s:s[s.index("        driver = '/alicia_d_driver_node'"):s.index('        after = identities(nodes)')]
        assert sdk(old)==sdk(current)
        result['positive_cases'].append('readonly_complete_sdk20_manual_idle_actions_confirmed_block_byte_identical')
        assert 'common.load_context(args, Path(__file__).resolve())' in current
        assert '== 1103' in current and 'RELEASE_SHA =' not in current and 'ACCEPTED_SHA =' not in current
        assert "release_commit['control_revision'] == 69" in current
        for source in (current,MANUAL.read_text()): compile(source,'offline_helper','exec')
        tree = ast.parse(MANUAL.read_text())
        writes = [n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                  and n.func.attr=='set_param']
        publishes = [n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
                     and n.func.attr=='publish']
        assert len(writes)==len(publishes)==1
        assert ast.literal_eval(writes[0].args[0])=='/gui/joint_direct_mode' and ast.literal_eval(writes[0].args[1]) is True
        lines = MANUAL.read_text().splitlines()
        final_line = next(i for i,line in enumerate(lines,1) if 'final_samples, final_states, final_sampled = checked_snapshot()' in line)
        assert final_line < writes[0].lineno < publishes[0].lineno
        assert any('common.recheck(ctx)' in line for line in lines[:final_line])
        assert any('get_num_connections()>0' in line for line in lines[:final_line])
        result['positive_cases'].append('one_manual_true_and_publish_only_after_final_snapshot')

        prior = dict(mode='unknown',strategy='observe_return',generation=5,stamp_ns=1000000001)
        selected = dict(prior,generation=6,stamp_ns=2000000001)
        deployment = dict(selection=deepcopy(prior),epoch_ns=100)
        entry = dict(kind='GEN62_remote_one_recorded_attempt_recorded',original_runner_returned=True,
            gate_returned_to_original_mode_line=True,automatic_retry=False,new_motion_authority_created=False,
            original_guards_replaced=False,temperature_queried=False,physical_success='unverified',
            original_mode_line=121,helper=dict(path=str(h.ENTRY),sha256=h.ENTRY_SHA),runner_returncode=6)
        exit_row = dict(runner_returncode=6,selection=deepcopy(selected),physical_success='unverified')
        auth = dict(selection=deepcopy(selected),previous_selection=deepcopy(prior),selection_response=dict(success=True),
            selection_commit=dict(latched_selection_matched=True,router_status=deepcopy(selected)),
            continuation_release_commit=dict(selection=deepcopy(selected),control_revision=69,
                policy='final_checked_authorization_commit_before_pipe_release'),
            operator_authorized_autonomous_grasp=True,recording_scope='local_only',old_plan_reused=False,
            physical_lift_after_close=False,physical_success='unverified',temperature_queried=False,torque_disable_authorized=False)
        base = [entry,exit_row,auth,deployment,6]
        assert h.returned_selection(*base)==selected
        result['positive_cases'].append('synthetic_exact_generation5_to6_original_runner_return_and_authorization')
        for name,index,keys,value in [
            ('not_returned',0,['original_runner_returned'],False),
            ('not_original_gate_return',0,['gate_returned_to_original_mode_line'],False),
            ('retry',0,['automatic_retry'],True),('new_authority',0,['new_motion_authority_created'],True),
            ('original_guards_changed',0,['original_guards_replaced'],True),
            ('entry_pin_changed',0,['helper','sha256'],'0'*64),('entry_kind_changed',0,['kind'],'GEN61_remote_one_recorded_attempt_recorded'),
            ('original_line_changed',0,['original_mode_line'],122),('runner_rc_changed',1,['runner_returncode'],0),
            ('runner_rc_bool',1,['runner_returncode'],True),('physical_success_claim',1,['physical_success'],'verified'),
            ('jump_generation',1,['selection','generation'],7),('no_successor',1,['selection','generation'],5),
            ('nonincreasing_selection_stamp',1,['selection','stamp_ns'],prior['stamp_ns']),
            ('strategy_changed',1,['selection','strategy'],'lift'),('prior_changed',2,['previous_selection','generation'],4),
            ('selection_service_failed',2,['selection_response','success'],False),
            ('selection_unlatched',2,['selection_commit','latched_selection_matched'],False),
            ('router_mismatch',2,['selection_commit','router_status','generation'],5),
            ('release_control_changed',2,['continuation_release_commit','control_revision'],68),
            ('release_selection_changed',2,['continuation_release_commit','selection','generation'],5),
            ('unauthorized',2,['operator_authorized_autonomous_grasp'],False),('scope_changed',2,['recording_scope'],'remote'),
            ('old_plan_reused',2,['old_plan_reused'],True),('lift_enabled',2,['physical_lift_after_close'],True),
            ('temperature_query',2,['temperature_queried'],True),('disable_authorized',2,['torque_disable_authorized'],True)]:
            value_rows=deepcopy(base);put(value_rows[index],keys,value)
            reject(result,name,lambda rows=value_rows:h.returned_selection(*rows))
        reject(result,'wrong_expected_generation',lambda:h.returned_selection(*base[:4],5))
        h.add_pin(h.RELEASE,'1'*64);h.add_pin(h.ACCEPTED,'2'*64)
        reject(result,'conflicting_future_pin',lambda:h.add_pin(h.RELEASE,'3'*64))
        reject(result,'missing_explicit_sha',lambda:h.add_pin(Path('/offline-missing'),''))
        reject(result,'nonhex_explicit_sha',lambda:h.add_pin(Path('/offline-missing'),'z'*64))

        scene=h.ATTEMPTS/'scene_start_offline'
        common=SimpleNamespace(A=Path('/offline/continuous_authorization.json'),AUTH_SHA='4'*64,CONTROL_SHA='5'*64)
        auth2=deepcopy(auth)
        auth2.update(session_dir=str(scene),operator_observation=dict(
            runtime_deployment=dict(path=str(h.ACCEPTED),sha256='2'*64),release=dict(path=str(h.RELEASE),sha256='1'*64),
            preflight=dict(path=str(scene/'preflight.json'),sha256='6'*64),control_revision=69,control_sha256=common.CONTROL_SHA),
            continuing_authorization=dict(path=str(common.A),sha256=common.AUTH_SHA))
        preflight=dict(epoch_ns=deployment['epoch_ns'],selection=deepcopy(prior))
        entry_inputs={str(scene/'preflight.json'):'6'*64}
        h.scene_predecessor(auth2,preflight,deployment,scene,entry_inputs,common)
        result['positive_cases'].append('synthetic_scene_predecessor_exact_pins_same_epoch_no_deadline_refresh')
        for name,keys,value in [
            ('scene_changed',['session_dir'],str(scene.parent/'other')),
            ('release_pin_changed',['operator_observation','release','sha256'],'7'*64),
            ('acceptance_pin_changed',['operator_observation','runtime_deployment','sha256'],'7'*64),
            ('preflight_pin_changed',['operator_observation','preflight','sha256'],'7'*64),
            ('control_revision_changed',['operator_observation','control_revision'],68),
            ('control_hash_changed',['operator_observation','control_sha256'],'7'*64),
            ('continuation_changed',['continuing_authorization','sha256'],'7'*64)]:
            altered=deepcopy(auth2);put(altered,keys,value)
            reject(result,name,lambda a=altered:h.scene_predecessor(a,preflight,deployment,scene,entry_inputs,common))
        for name,key,value in [('epoch_changed','epoch_ns',101),('scene_prior_changed','selection',selected)]:
            altered=deepcopy(preflight);altered[key]=value
            reject(result,name,lambda p=altered:h.scene_predecessor(auth2,p,deployment,scene,entry_inputs,common))

        # Validate the real sealed GEN61 schema with its real wrapper rc=2 and
        # original runner rc=6. Temporarily substitute only historical constants;
        # no GEN62 release or future result is created or accepted here.
        R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T092802.096230Z_567d85bbe8f54a56af71145548b078f8')
        old_post=load(W/'.runtime/read_gen61_post_attempt_20261022.py','offline_old_post')
        historical_entry=E/'actual_attempt_r1/scene_start_r9/recorded_entry_result.json'
        historical=SimpleNamespace(record=R,attempt_result=historical_entry,attempt_result_sha256=sha(historical_entry),
            runner_exit=R/'user_request_runner_exit.json',runner_exit_sha256=sha(R/'user_request_runner_exit.json'),
            authorization=R/'authorization.json',authorization_sha256=sha(R/'authorization.json'),
            record_manifest=R/'manifest.json',record_manifest_sha256=sha(R/'manifest.json'),
            release_sha256=old_post.RELEASE_SHA,deployment_sha256=old_post.ACCEPTED_SHA)
        originals={name:getattr(h,name) for name in ('ENTRY','ENTRY_SHA','RELEASE','ACCEPTED','COMMON_SHA')}
        for name in originals:setattr(h,name,getattr(old_post,name))
        real_entry=h.read(historical_entry);real_exit=h.read(historical.runner_exit);real_manifest=h.read(historical.record_manifest)
        assert real_manifest['result_evidence']['runner_returncode']==2 and real_exit['runner_returncode']==real_entry['runner_returncode']==6
        assert h.closed_record(historical,real_entry,real_exit)==R
        result['positive_cases'].append('real_GEN61_sealed_record_schema_wrapper_rc2_distinct_from_original_rc6')
        saved_read=h.read
        for name,keys,value in [('record_not_closed',['recorder_closed'],False),('record_not_complete',['state'],'recording'),
                ('wrong_record_id',['record_id'],'wrong'),('missing_completion',['completed_at'],''),
                ('wrong_wrapper',['metadata','launcher'],'other'),('auth_manifest_pin',['files',0,'sha256'],'0'*64)]:
            altered=deepcopy(real_manifest);put(altered,keys,value)
            h.read=lambda p,altered=altered:altered if Path(p)==historical.record_manifest else saved_read(p)
            reject(result,name,lambda:h.closed_record(historical,real_entry,real_exit))
        h.read=saved_read
        for name in originals:setattr(h,name,originals[name])

        samples,states,now,epoch=snapshot_fixture(m)
        m.validate_idle_snapshot(samples,states,now,epoch)
        result['positive_cases'].append('current20sdk_same_epoch_stable_idle_confirmed')
        for name,change in [
            ('sdk_200ms',lambda s,v:s[-1].update(stamp_ns=now-200_000_000)),
            ('sdk_800ms',lambda s,v:s[-1].update(stamp_ns=now-800_000_000)),
            ('sdk_future',lambda s,v:s[-1].update(stamp_ns=now+1)),
            ('sdk_foreign',lambda s,v:s[-1].update(caller='/foreign')),
            ('sdk_older_than_epoch',lambda s,v:s[0].update(stamp_ns=epoch-1,received_ns=epoch)),
            ('sdk_wrong_frame',lambda s,v:s[-1].update(frame_id='commanded')),
            ('sdk_drift',lambda s,v:s[-1]['positions'].__setitem__(0,.105)),
            ('sdk_nan',lambda s,v:s[-1]['positions'].__setitem__(0,float('nan'))),
            ('sdk_duplicate',lambda s,v:s[-1].update(stamp_ns=s[-2]['stamp_ns'])),
            ('epoch_changed_before_request',lambda s,v:setattr(v['epoch'][0],'stamp',Stamp(epoch+1))),
            ('task_active_before_request',lambda s,v:setattr(v['task'][0],'active',True)),
            ('router_active_before_request',lambda s,v:setattr(v['mode'][0],'data','{"active":true}')),
            ('router_pending_before_request',lambda s,v:setattr(v['mode'][0],'data','{"pending":true}')),
            ('router_reserved_before_request',lambda s,v:setattr(v['mode'][0],'data','{"start_reserved":true}')),
            ('arm_busy_before_request',lambda s,v:setattr(v['arm'][0],'status_list',[SimpleNamespace(status=1)])),
            ('hand_busy_before_request',lambda s,v:setattr(v['hand'][0],'status_list',[SimpleNamespace(status=7)])),
            ('action_stale',lambda s,v:setattr(v['arm'][0].header,'stamp',Stamp(now-2_000_000_001))),
            ('not_confirmed',lambda s,v:setattr(v['actuation'][0],'data','PENDING: fixture')),
            ('not_ready',lambda s,v:setattr(v['ready'][0],'data',False)),
            ('not_enabled',lambda s,v:setattr(v['enabled'][0],'data',False)),
            ('foreign_state',lambda s,v:v['task'][0]._connection_header.update(callerid='/foreign')),
            ('stale_state_delivery',lambda s,v:v.__setitem__('task',(v['task'][0],now-1_000_000_000)))]:
            s,v=deepcopy(samples),deepcopy(states);change(s,v)
            reject(result,name,lambda s=s,v=v:m.validate_idle_snapshot(s,v,now,epoch))
        # The same final gate rejects a frozen SDK after publisher waiting.
        reject(result,'publisher_wait_stale_snapshot',lambda:m.validate_idle_snapshot(samples,states,now+5_000_000_000,epoch))
        ack=(SimpleNamespace(data=True,_connection_header=dict(callerid=m.GUI,latching='1')),now)
        assert m.own_ack(ack,now)
        result['positive_cases'].append('own_gui_true_latched_ack_at_or_after_request')
        for name,ackrow,request in [('foreign_ack',(SimpleNamespace(data=True,_connection_header=dict(callerid='/foreign',latching='1')),now),now),
            ('early_ack',ack,now+1),('false_ack',(SimpleNamespace(data=False,_connection_header=dict(callerid=m.GUI,latching='1')),now),now),
            ('nonlatched_ack',(SimpleNamespace(data=True,_connection_header=dict(callerid=m.GUI,latching='0')),now),now),
            ('missing_ack',None,now)]:
            assert not m.own_ack(ackrow,request)
            result['negative_cases'].append(dict(name=name,rejected=True))
        assert not denied
        assert all(sha(p)==digest for p,digest in EXPECTED.items())
        result.update(passed=True,positive_count=len(result['positive_cases']),negative_count=len(result['negative_cases']),
            note='Pure contracts and real historical schema only; no future GEN62 deployment or actual attempt was run.')
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    result['elapsed_seconds']=time.monotonic()-start
    raw=(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with (OUT/'result.json').open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert (OUT/'result.json').read_bytes()==raw
    print(json.dumps(dict(path=str(OUT/'result.json'),sha256=sha(OUT/'result.json'),passed=result['passed'],
        positive_count=result.get('positive_count'),negative_count=result.get('negative_count'),error=result.get('error')),ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
