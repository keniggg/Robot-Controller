#!/usr/bin/env python3
"""Read saved GEN59 evidence and source bytes only; never contact ROS/network."""
from pathlib import Path
import hashlib
import json
import math
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
O = N / 'gen58_resume_r1/observation_pose_fix'
C = W / '.runtime/gen59_observation_pose_candidate_r3'
OUT = O / 'runtime_acceptance_independent_r1'
PINS = {
    N / 'verification/gen59_release_manifest.json': '5fed4cbed7ea1e824eda3bb750657970dcc1ec2d711cd041eff1a3d482ef2c6c',
    O / 'runtime_reload_r1/acceptance/result.json': 'b5b53dcf13f51d507187a4c71e7ac20977e3bbc1ed6b51025f30ed5aabfb1082',
    O / 'verification/full_r3/result.json': 'f30873a4a4b36e9e0f0e5fc5e897d2e693fa2133dec82e54326cab87f677dbc8',
    O / 'verification/freeze_r3/manifest_closed.json': '89ac6a0f87f28c43f49dcc4c84682e2b539394b16038fdc276960efe4af0f5b9',
    O / 'install_plan_r3/plan.json': '98b73cea1b23313f5315e3ab28a603c3da9f79053df15832a043f9c1f3801760',
    O / 'install_helper_review_r3/result.json': '42cc1b5019a9a5204be4c02ee64efd97997d331dc159ddc3c890b8dbaff6f27a',
    O / 'verification/full_r3/independent_verification.json': 'dc509e451e5ccee60d298265ec1e1150f38e0a5cdfbd2103d34cf6473fc43859',
    O / 'verification/full_r3/final_process_receipt.json': 'f94e04099d3da1ed37e977dbff41433bd36e37bf115e4cce4076aa919312fd8c',
}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def read(path):
    return json.loads(Path(path).read_text())

def verify_map(values):
    for path, expected in values.items():
        assert sha(path) == expected, str(path)

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir(exist_ok=False)
    verify_map(PINS)
    release = read(N / 'verification/gen59_release_manifest.json')
    accepted = read(O / 'runtime_reload_r1/acceptance/result.json')
    full = read(O / 'verification/full_r3/result.json')
    freeze = read(O / 'verification/freeze_r3/manifest_closed.json')
    plan = read(O / 'install_plan_r3/plan.json')
    construction = read(accepted['startup_configuration'])
    preload = read(accepted['runtime_import_attestation'])
    old = read(accepted['previous_deployment'])
    reload = read(accepted['reload_evidence'])
    stopped = read(accepted['stop_receipt'])
    dependency = accepted['remote_dependency']
    binding = read(dependency['startup_binding'])
    health = read(dependency['readiness'])
    verify_map(release['evidence_sha256'])
    verify_map(accepted['input_sha256'])
    verify_map(construction['helper_sha256'])
    verify_map({row['path']: row['sha256'] for row in binding['evidence']})
    verify_map({accepted['runtime_import_attestation']: accepted['runtime_import_attestation_sha256'],
                accepted['startup_configuration']: accepted['startup_configuration_sha256'],
                dependency['startup_binding']: dependency['startup_binding_sha256'],
                dependency['readiness']: dependency['readiness_sha256'],
                health['probe']['body_path']: health['probe']['body_sha256']})
    source = release['source_sha256']
    assert len(source) == release['source_count'] == 1094
    assert source == accepted['source_sha256'] == construction['source_sha256'] == freeze['source_sha256'] == full['source_sha256'] == plan['validation']['source_sha256']
    verify_map({W / p: h for p, h in source.items()})
    verify_map({C / p: h for p, h in source.items()})
    assert release['test_summary'] == accepted['test_summary'] == full['summary'] == dict(tests=9993, failures=0, errors=0, skipped=0)
    assert full['passed'] and full['full_regression_passed'] and full['source_unchanged'] and full['scope_complete']
    assert release['passed'] and release['installed'] and release['runtime_deployed'] is False
    assert accepted['deployment_passed'] and accepted['whole_patch_deployed'] and not accepted['full_restart']
    assert accepted['deployment_kind'] == 'GEN59_remote_only_sixteen_processes_preserved'
    remote = accepted['new_remote_identity']
    assert len(preload['loaded_sources']) == 3
    for path, row in preload['loaded_sources'].items():
        assert row['path'] == str(W / path) and row['sha256'] == source[path]
        assert row['compiled_from_verified_source_bytes'] is True
    for identity in [preload['node_identity'], binding['node_identity'], health['before']['node_identity'], health['after']['node_identity']]:
        assert all(identity[k] == remote[k] for k in ('pid', 'parent_pid', 'start_ticks'))
    assert preload['node_identity'] == binding['node_identity'] == health['before']['node_identity'] == health['after']['node_identity']
    old_nodes = old['final_all_registered_nodes']
    nodes = accepted['full_runtime_nodes']
    preserved = {n: p for n, p in old_nodes.items() if n != '/remote_grasp6d_node'}
    assert len(old_nodes) == len(nodes) == 17 and len(preserved) == 16
    assert preserved == accepted['preserved_before'] == accepted['preserved_after'] == reload['preserved_before']
    assert nodes == accepted['all_registered_nodes'] == accepted['final_all_registered_nodes'] == dict(preserved, **{'/remote_grasp6d_node': remote})
    assert accepted['replaced_before'] == {'/remote_grasp6d_node': old_nodes['/remote_grasp6d_node']}
    assert accepted['replaced_after'] == {'/remote_grasp6d_node': remote} and remote != old_nodes['/remote_grasp6d_node']
    assert accepted['core_service'] == old['core_service'] and accepted['unit'] == old['unit']
    assert accepted['master_process'] == old['master_process'] and accepted['simulation_unit'] == old['simulation_unit']
    assert int(accepted['simulation_service']['MainPID']) == old['simulation_service']['pid']
    assert accepted['simulation_service']['InvocationID'] == old['simulation_service']['invocation_id']
    assert accepted['simulation_service']['ControlGroup'] == old['simulation_service']['control_group']
    assert int(accepted['remote_service']['MainPID']) == remote['pid'] and accepted['remote_unit'] == construction['unit']
    assert accepted['epoch_ns'] == old['epoch_ns'] == stopped['epoch_ns'] == construction['epoch_ns'] == 1792639534222874592
    assert accepted['selection'] == construction['selection']
    assert accepted['reset_ack']['ready'] and all(accepted['reset_ack'][k] == v for k, v in accepted['selection'].items())
    samples = accepted['samples']
    assert len(samples) == accepted['sample_count'] == 20
    stamps = [s['stamp_ns'] for s in samples]
    assert stamps == sorted(set(stamps)) and stamps[0] > accepted['epoch_ns']
    assert len(accepted['names']) == 7 and len(set(accepted['names'])) == 7
    for s in samples:
        assert s['frame_id'] == 'sdk_measured' and s['callerid'] == '/alicia_d_driver_node'
        assert s['names'] == accepted['names'] and s['positions'] == samples[0]['positions']
        assert all(math.isfinite(x) for x in s['positions'])
    assert accepted['maximum_joint_delta_rad'] == 0 and accepted['manual_mode'] and accepted['gui_owning_manual_ack']
    assert accepted['task_active'] is False and accepted['actions_active'] is False
    assert accepted['router']['active'] is False and accepted['router']['start_reserved'] is False
    assert accepted['actuation_status'].startswith('CONFIRMED:') and accepted['feedback_ready'] and accepted['motion_enabled']
    assert accepted['control_revision'] == accepted['control_snapshot']['revision'] == 67 and accepted['control_unchanged']
    assert binding['parameters'] == construction['parameters'] == health['before']['parameters'] == health['after']['parameters']
    assert health['before'] == health['after'] and health['passed'] and health['dependency_ready']
    assert health['probe']['http_status'] == 200 and health['probe']['body_complete']
    assert health['probe']['url'] == binding['parameters']['/grasp_6d/remote/server_url'] + '/health'
    assert health['capabilities']['protocol_version'] == 3 and health['capabilities']['require_candidate_depth'] is True
    assert health['capabilities']['backend'] in binding['allowed_backends']
    assert set(['score', 'width_m', 'height_m', 'depth_m', 'translation_m', 'rotation_matrix']).issubset(health['capabilities']['candidate_fields'])
    assert construction['recorded_ns'] < preload['recorded_ns'] < binding['created_ns'] < health['started_ns'] < health['finished_ns'] < accepted['recorded_ns']
    assert health['prediction_requests'] == 0 and health['ros_write_operations'] == []
    for obj in (accepted, health):
        for key in ('motion_commands_sent', 'temperature_queried', 'torque_disable_commands_sent'):
            assert obj[key] is False
    result = dict(kind='GEN59_saved_runtime_acceptance_independent_audit', passed=True, recorded_ns=time.time_ns(),
                  auditor_sha256=sha(__file__), pinned_inputs={str(p): h for p, h in PINS.items()},
                  source_count=1094, source_matches_active_and_tested_candidate=True, test_summary=full['summary'],
                  release_evidence_verified=324, acceptance_inputs_verified=370, preloaded_modules=preload['loaded_sources'],
                  runtime_node_count=17, preserved_process_count=16, remote_identity=remote, epoch_ns=accepted['epoch_ns'],
                  source_release_runtime_deployed_flag=False, separate_runtime_acceptance_deployment_passed=True,
                  full_restart=False, acceptance_sample_count=20, acceptance_joint_span_rad=0.0,
                  remote_health_recorded_interval_ns=[health['started_ns'], health['finished_ns']],
                  health_bound_to_same_process_and_startup_parameters=True,
                  scope='Saved acceptance evidence plus current file hashes; no live state or health request, no tests, no ROS/network/hardware calls.',
                  limitations=['Acceptance idle/health statements describe their recorded time, not current state.',
                               'Source-only release and pre-acceptance reload receipts intentionally retain runtime_deployed/acceptance_passed false; final acceptance is separate.',
                               'Health proves advertised availability/protocol only, not prediction, scene clearance, path safety, or physical grasp.'],
                  motion_commands_sent=False, temperature_queried=False, physical_grasp_verified=False)
    target = OUT / 'result.json'
    with target.open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2); f.write('\n')
    print(json.dumps(dict(passed=True, path=str(target), sha256=sha(target)), ensure_ascii=False))

if __name__ == '__main__':
    main()
