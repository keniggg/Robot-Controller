"""Offline acceptance corruption checks; no ROS, process or network authority."""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

W = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(W/'src/alicia_grasp_modes/src'))
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
denied = []
def guard(event, args):
    if event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.posix_spawn'):
        denied.append(event)
        raise RuntimeError('offline contract check forbids '+event)
sys.addaudithook(guard)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    out = E/'attempt_adapter_offline_r2'
    assert Path('/media/zhuyupei/SanStand').is_mount() and not out.exists()
    out.mkdir()
    path = W/'.runtime/gen61_compute_attempt_common_20261022.py'
    spec = importlib.util.spec_from_file_location('gen61_offline_common',path)
    common = importlib.util.module_from_spec(spec);spec.loader.exec_module(common)
    design_path = E/'runtime_adapter_draft_r1/bundle.json'
    bundle = json.loads(design_path.read_text())
    assert not bundle['deployment_ready']
    bundle = deepcopy(bundle);bundle['deployment_ready'] = True
    # Only an explicitly synthetic value fixture is promoted. No file is
    # changed and no final deployment artifact is constructed by this test.
    names = ('/unknown_tabletop_perception','/remote_grasp6d_node','/grasp_task_node')
    nodes = deepcopy(bundle['nodes'])
    for index,name in enumerate(names):
        nodes[name] = dict(pid=900000+index,parent_pid=800000,start_ticks=str(100+index))
    targets = {n:nodes[n] for n in names}
    services = {n:dict(MainPID=str(nodes[n]['pid'])) for n in names}
    transition = dict(passed=True,kind='GEN61_three_compute_reload',targets=targets,
        units={n:'synthetic-unit-'+str(i) for i,n in enumerate(names)},
        after=dict(services=services,auxiliary_nodes={}))
    deployment = dict(kind='GEN61_three_compute_acceptance',generation='GEN61',deployment_passed=True,
        full_restart=False,replaced_count=3,preserved_count=14,full_runtime_nodes=nodes,targets=targets,
        preserved=deepcopy(bundle['preserved']),before=dict(node_identities=nodes),
        after=dict(node_identities=nodes,services=services,auxiliary_nodes={}),auxiliary_nodes={},
        epoch_ns=bundle['epoch_ns'],selection=deepcopy(bundle['selection']),manual_mode=True,
        task_active=False,actions_active=False,actuation_status='CONFIRMED: synthetic offline fixture',
        gui_owning_manual_ack=True,automatic_attempt_authorized=False,
        fresh_scene_and_dependency_gate_required_before_automatic_attempt=True,
        source_sha256=deepcopy(bundle['source_sha256']),
        source_process_proof={n:dict(identity=nodes[n],source_cache_process_proof=True,memory_attestation=False) for n in names})
    release = dict(generation='GEN61',passed=True,reviewed=True,installed=True,full_regression_passed=True,
        source_count=len(bundle['source_sha256']),source_sha256=deepcopy(bundle['source_sha256']),
        test_summary=dict(tests=10045,failures=0,errors=0,skipped=0))
    baseline = (release,deployment,bundle,transition)
    result = dict(kind='GEN61_attempt_acceptance_offline_value_contracts',passed=False,
        actual_deployment_proven=False,fixture_is_synthetic=True,cases=[],flow_checks=[],
        ROS_initialized=False,live_nodes_contacted=False,motion_commands_sent=False)
    assert common.validate_acceptance_values(*deepcopy(baseline)) == nodes
    changes = [
        ('wrong_generation',0,('generation',),'GEN60'),
        ('not_reviewed',0,('reviewed',),False),
        ('not_installed',0,('installed',),False),
        ('incomplete_regression',0,('full_regression_passed',),False),
        ('regression_failure',0,('test_summary','failures'),1),
        ('regression_skip',0,('test_summary','skipped'),1),
        ('old_case_count',0,('test_summary','tests'),10044),
        ('wrong_source_count',0,('source_count',),1),
        ('draft_bundle',2,('deployment_ready',),False),
        ('wrong_acceptance_kind',1,('kind',),'GEN60_gateway_acceptance'),
        ('not_accepted',1,('deployment_passed',),False),
        ('overstated_full_restart',1,('full_restart',),True),
        ('wrong_replace_count',1,('replaced_count',),1),
        ('wrong_preserved_count',1,('preserved_count',),16),
        ('missing_target',1,('targets',),{}),
        ('wrong_retained_map',1,('preserved',),{}),
        ('wrong_acceptance_graph',1,('after','node_identities'),{}),
        ('unclosed_transition',3,('passed',),False),
        ('wrong_transition_kind',3,('kind',),'GEN60_gateway_reload'),
        ('missing_unit',3,('units',),{}),
        ('different_unit_owner',3,('after','services'),{}),
        ('different_auxiliary_owner',1,('auxiliary_nodes',),{'/alien':nodes['/motion_gateway']}),
        ('epoch_changed',1,('epoch_ns',),1),
        ('selection_changed',1,('selection','generation'),5),
        ('not_manual',1,('manual_mode',),False),
        ('task_running',1,('task_active',),True),
        ('action_running',1,('actions_active',),True),
        ('actuation_not_confirmed',1,('actuation_status',),'READY'),
        ('missing_manual_ack',1,('gui_owning_manual_ack',),False),
        ('acceptance_grants_attempt',1,('automatic_attempt_authorized',),True),
        ('no_fresh_scene_required',1,('fresh_scene_and_dependency_gate_required_before_automatic_attempt',),False),
        ('source_proof_missing',1,('source_process_proof',),{}),
        ('overstated_memory_attestation',1,('source_process_proof',names[0],'memory_attestation'),True),
        ('missing_source_process_proof',1,('source_process_proof',names[0],'source_cache_process_proof'),False),
    ]
    for name,index,keys,value in changes:
        values=deepcopy(baseline);target=values[index]
        for key in keys[:-1]:target=target[key]
        target[keys[-1]]=value
        try:common.validate_acceptance_values(*values)
        except (RuntimeError,KeyError,ValueError,TypeError):rejected=True
        else:rejected=False
        result['cases'].append(dict(name=name,rejected=rejected))
        assert rejected,name
    pins = {str(path):sha(path),str(design_path):sha(design_path),str(Path(__file__).resolve()):sha(__file__)}
    for name in ('capture','receipt','entry','gate'):
        old=W/('.runtime/gen60_gateway_attempt_'+name+'_20261022.py')
        new=W/('.runtime/gen61_compute_attempt_'+name+'_20261022.py')
        adapted=old.read_text().replace('gen60_gateway_attempt','gen61_compute_attempt').replace('GEN60','GEN61').replace('gen60_gateway','gen61_compute')
        assert adapted==new.read_text();ast.parse(adapted)
        pins[str(old)]=sha(old);pins[str(new)]=sha(new)
        result['flow_checks'].append(dict(name=name,unchanged_except_identifier_mapping=True))
    assert not denied
    result.update(passed=True,positive_cases=1,negative_cases=len(changes),input_sha256=pins,
        boundary_attempts=denied,recorded_ns=time.time_ns())
    (out/'test_source.py').write_bytes(Path(__file__).read_bytes())
    with (out/'result.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(passed=True,negative_cases=len(changes),result=str(out/'result.json'),sha256=sha(out/'result.json'))))

if __name__=='__main__':main()
