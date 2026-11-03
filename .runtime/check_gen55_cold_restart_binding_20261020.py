"""Offline checks for the cold-restart binding; no main deployment entry called."""
import ast
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import types

sys.dont_write_bytecode = True
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
A=V/'gen55_cold_restart_binding_r1'
O=A/'offline_checks'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def definition(text,name):
    return next(n for n in ast.parse(text).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and A.is_dir()
    O.mkdir(exist_ok=False)
    denied=[]
    def boundary(event,args):
        bad=event.startswith(('socket.','subprocess.')) or event in ('os.system','os.exec','os.posix_spawn','os.fork')
        if event=='open' and args and isinstance(args[0],(str,bytes,os.PathLike)):
            path=Path(os.fsdecode(args[0])).resolve()
            flags=args[2] if len(args)>2 and isinstance(args[2],int) else 0
            mode=args[1] if len(args)>1 and isinstance(args[1],str) else ''
            if flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND) or any(c in mode for c in 'wax+'):
                bad=O not in path.parents
            if str(path).startswith('/dev/') and str(path) not in ('/dev/null','/dev/random','/dev/urandom'):
                bad=True
        if bad:
            denied.append(dict(event=event,args=repr(args)[:150]))
            raise RuntimeError('offline boundary denied '+event)
    sys.addaudithook(boundary)
    sys.path.insert(0,str(W/'.runtime'))
    import gen55_idle_reload_common_20261019 as common
    import install_and_seal_gen55_20261019 as installer
    binding=read(A/'result.json')
    assert binding['passed'] and all(sha(p)==d['after_sha256'] for p,d in binding['helper_delta'].items())
    checks=[]
    def case(name,fn,accept=True):
        try:
            fn()
        except Exception as exc:
            got=False;detail=repr(exc)
        else:
            got=True;detail=None
        checks.append(dict(name=name,expected_accept=accept,accepted=got,passed=got==accept,detail=detail))
    previous,baseline=common.validate_previous(common.PREVIOUS)
    cold=read(previous['cold_restart_acceptance'])
    case('actual_accepted_cold_adapter',lambda:common.validate_previous(common.PREVIOUS))
    def adapter_partition():
        assert sha(previous['cold_restart_acceptance'])==previous['cold_restart_acceptance_sha256']
        assert set(previous['after'])==set(common.TARGETS) and set(previous['preserved_after'])==set(common.PRESERVE)
        assert not set(previous['after'])&set(previous['preserved_after'])
        assert baseline==previous['cold_restart_all_nodes']==cold['after']
        assert cold['preserved_after']=={} and cold['all_processes_replaced'] is True
        assert previous['selection']==cold['selection'] and previous['epoch_ns']==cold['epoch_ns']
        assert all(row['parent_pid']==8000 for row in baseline.values())
        assert 'NEXT' in previous['preserved_after_semantics'] and previous['dedicated_compute_reloader_compatible'] is True
    case('eleven_new_cold_nodes_partitioned_only_for_next_reload',adapter_partition)
    prior=read(installer.PRIOR_INSTALLED_MANIFEST)
    source=copy.deepcopy(prior['source_sha256'])
    for name in ('test_contact_sequence_precheck.py','test_gen38_contact_replay_staging.py'):
        source['src/alicia_flexible_grasp_supervisor/tests/'+name]='1'*64
    manifest={'source_sha256':source}
    _,changed,added=installer.check_delta(manifest)
    def delta_scope():
        assert len(changed)==7 and len(added)==7 and len(source)==1082
    case('seven_changed_seven_retained_scope',delta_scope)
    case('exact_prior_installed_seven_additions',lambda:installer.verify_retained_additions(manifest,added))
    wrong=copy.deepcopy(manifest);wrong['source_sha256'][next(iter(added))]='2'*64
    case('candidate_retained_addition_mismatch_rejected',lambda:installer.verify_retained_additions(wrong,added),False)
    wrong=copy.deepcopy(manifest)
    wrong['source_sha256']['src/alicia_flexible_grasp_supervisor/tests/test_contact_sequence_precheck.py']=read(installer.BASE_RELEASE)['source_sha256']['src/alicia_flexible_grasp_supervisor/tests/test_contact_sequence_precheck.py']
    case('missing_one_reviewed_old_test_change_rejected',lambda:installer.check_delta(wrong),False)
    real_sha=installer.sha
    changed_path=installer.W/next(iter(added))
    installer.sha=lambda p:'0'*64 if Path(p)==changed_path else real_sha(p)
    case('existing_retained_file_mismatch_rejected',lambda:installer.verify_retained_additions(manifest,added),False)
    installer.sha=real_sha
    installer.sha=lambda p:'0'*64 if Path(p)==installer.PRIOR_INSTALLED_MANIFEST else real_sha(p)
    case('prior_installed_manifest_digest_mismatch_rejected',lambda:installer.verify_retained_additions(manifest,added),False)
    installer.sha=real_sha
    original=(A/'backup_helpers/install_and_seal_gen55_20261019.py').read_text()
    current=(W/'.runtime/install_and_seal_gen55_20261019.py').read_text()
    def retained_review_gate():
        assert ast.dump(definition(original,'verify_review'))==ast.dump(definition(current,'verify_review'))
        install=definition(current,'install')
        loops=[n for n in ast.walk(install) if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='name']
        assert len(loops)==2 and all(ast.dump(n.iter)==ast.dump(ast.parse('sorted(changed)',mode='eval').body) for n in loops)
        assert 'retained_additions_rewritten=False' in current
    case('independent_review_gate_unchanged_only_seven_existing_sources_written',retained_review_gate)
    finalizer=(W/'.runtime/finalize_gen55_idle_deployment_20261019.py').read_text()
    finalizer_before=(A/'backup_helpers/finalize_gen55_idle_deployment_20261019.py').read_text()
    def helper_pins_and_chain_code():
        tree=ast.parse(finalizer)
        pin=next(n for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='HELPER_SHA' for t in n.targets))
        assert ast.literal_eval(pin.value)=={str(p):sha(p) for p in common.HELPERS}
        assert ast.dump(definition(finalizer,'deployment_chain'))==ast.dump(definition(finalizer_before,'deployment_chain'))
        assert ast.dump(definition(finalizer,'validate_ack'))==ast.dump(definition(finalizer_before,'validate_ack'))
    case('finalizer_new_helper_hashes_existing_chain_and_ack_retained',helper_pins_and_chain_code)
    ns=dict(require=common.require,TARGETS=common.TARGETS,PRESERVE=common.PRESERVE,
        COMPUTE=common.COMPUTE,GATEWAY_SERVICES=common.GATEWAY_SERVICES)
    code=ast.Module(body=[definition(finalizer,n) for n in ('identities_valid','deployment_chain')],type_ignores=[])
    exec(compile(code,'actual_finalizer_functions','exec'),ns)
    parent={'pid':8000,'parent_pid':1,'start_ticks':'1'}
    after={n:dict(pid=90001+i,parent_pid=90000 if n in common.COMPUTE else 8000,start_ticks=str(2+i)) for i,n in enumerate(common.TARGETS)}
    preflight=dict(node_identities=baseline,gateway_original_parent=parent)
    compute=dict(before={n:baseline[n] for n in common.TARGETS},after=after,
        preserved_before={n:baseline[n] for n in common.PRESERVE},preserved_after={n:baseline[n] for n in common.PRESERVE},
        replaced_count=4,preserved_count=7,gateway_respawn_parent=parent,gateway_after_respawn=after['/motion_gateway'],
        gateway_started_by_compute_launch=False,gateway_original_parent_preserved=True,
        gateway_service_owners={n:['/motion_gateway'] for n in common.GATEWAY_SERVICES})
    case('four_node_reload_from_single_old_parent_accepted',lambda:ns['deployment_chain'](previous,preflight,compute))
    wrong=copy.deepcopy(compute);wrong['preserved_after']['/supervisor_camera']['pid']+=100000
    case('one_of_seven_preserved_processes_changed_rejected',lambda:ns['deployment_chain'](previous,preflight,wrong),False)
    wrong=copy.deepcopy(compute);wrong['gateway_original_parent_preserved']=False
    case('original_core_parent_not_preserved_rejected',lambda:ns['deployment_chain'](previous,preflight,wrong),False)
    readiness=(W/'.runtime/read_gen55_preinstall_readiness_20261019.py').read_text()
    main_node=definition(readiness,'main')
    target_try=next(n for n in main_node.body if isinstance(n,ast.Try))
    statements=target_try.body
    first=next(i for i,n in enumerate(statements) if isinstance(n,ast.Assert) and "state['epoch'].stamp.to_nsec()" in ast.get_source_segment(readiness,n))
    guard=ast.Module(body=statements[first:first+5],type_ignores=[])
    guardcode=compile(guard,'actual_readiness_epoch_unique_stability_guards','exec')
    def sample_guard(values,duplicate=False,epoch=None):
        epoch=cold['epoch_ns'] if epoch is None else epoch
        def stamp(value):return types.SimpleNamespace(to_nsec=lambda:value)
        history=[types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp(epoch+1 if duplicate else epoch+i+1)),position=[v]) for i,v in enumerate(values)]
        env=dict(state={'epoch':types.SimpleNamespace(stamp=stamp(epoch))},deployment=previous,
            history=history,joint=types.SimpleNamespace(name=['Joint1']))
        exec(guardcode,env)
    case('fresh_stable_samples_can_differ_from_cold_start_pose',lambda:sample_guard([1.0]*20))
    case('sample_drift_rejected',lambda:sample_guard([1.0]*19+[1.005]),False)
    case('duplicate_measurement_stamps_rejected',lambda:sample_guard([1.0]*20,duplicate=True),False)
    case('old_epoch_rejected',lambda:sample_guard([1.0]*20,epoch=1792373681250409915),False)
    result=dict(passed=all(row['passed'] for row in checks) and not denied,checks=checks,boundary_attempts=denied,
        live_ros_contacted=False,hardware_contacted=False,task_sources_installed=False,runtime_reloaded=False,
        real_candidate_regression_reexecuted=False,synthetic_cases_not_real_regression=True,
        dependency_sha256={str(Path(__file__).resolve()):sha(__file__),str(A/'result.json'):sha(A/'result.json'),
            str(common.PREVIOUS):sha(common.PREVIOUS),**{p:d['after_sha256'] for p,d in binding['helper_delta'].items()}},
        recorded_ns=time.time_ns())
    with (O/'result.json').open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
    print(json.dumps(dict(passed=result['passed'],checks=len(checks),result=str(O/'result.json'),sha256=sha(O/'result.json')),ensure_ascii=False))
    if not result['passed']:
        raise RuntimeError('one or more offline binding checks failed')


if __name__=='__main__':
    main()
