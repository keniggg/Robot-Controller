"""Bind the existing camera-only A/B/A experiment to verified current instances."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1')
R = F / 'actual_gen63_baseline_validation_20261023T041149Z'
OUT = R / 'current_depth_preset_aba_r1'


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir()
    os.environ['ROS_LOG_DIR'] = str(OUT / 'controller_prepare_ros')
    helper = W / '.runtime/depth_stage_camera_session_20261022.py'
    spec = importlib.util.spec_from_file_location('existing_camera_session', helper)
    s = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(s)
    s.OUT = OUT
    previous_path = F / 'depth_baseline_deployment_r2/plan.json'
    old = s.read_ref(dict(path=str(previous_path), sha256='5df34e6ff4c9e3cc177a1e6ffc47ff77b66b005c8f3a912c788c17d5207b724e'))
    for path, sha in old['sources'].items():
        s.require(s.digest(path) == sha, 'camera/probe source changed: ' + path)
    camera = s.read_ref(dict(path=str(F / 'depth_baseline_deployment_r2/result.json'),
        sha256='17430a8174be79b5811e9f91e9f67abaeed970210d82efca2e5109618a001bbc'))['production']['new_camera']
    compute_ref = dict(path=str(R / 'repair_deployment_r1/result.json'),
        sha256='95ed6d28f249b96cf0232b965c4c34fe4747369a4bdf6f207194ff472507b69c')
    compute = s.read_ref(compute_ref)
    s.require(compute['passed'] is True, 'compute deployment did not pass')
    monitor = s.Monitor()
    try:
        s.require(monitor.node('/supervisor_camera') == camera, 'camera instance changed')
        intent = json.loads((F / 'depth_baseline_deployment_r2/restored/launch_intent.json').read_text())
        s.require(s.unit_pid(intent['unit']) == camera, 'camera service changed')
        plan = copy.deepcopy(old)
        for name, row in compute['nodes_after'].items():
            s.require(name in ('/unknown_tabletop_perception', '/remote_grasp6d_node'), 'unexpected compute replacement')
            plan['preserved'][name] = s.process_identity(row)
        s.preserved(plan, monitor)
        s.require_device_owner(plan, camera['pid'])
        cfg = monitor.master.getParam('/camera')
        s.require(s.configuration_contract(cfg) == old['configuration_contract'], 'current camera config changed')
        before = monitor.sample()
        s.require(before['selection']['mode'] == 'unknown', 'unknown scope required')
        for key in ('persistent_profile', 'revision_inputs', 'previous_configuration', 'desired_configuration',
                    'configuration_before', 'depth_quality_or_grasp_success_claimed'):
            plan.pop(key, None)
        plan.update(schema=4, kind='same_pair_online_camera_only_probe', output=str(OUT),
            old_camera=camera, old_camera_uri=monitor.master.lookupNode('/supervisor_camera'),
            old_camera_unit=intent['unit'], before=before, camera_configuration=cfg,
            diagnostic_trials=[dict(label='A1', preset='high_accuracy'),
                dict(label='B', preset='saved_baseline'), dict(label='A2', preset='high_accuracy')],
            preset_changed=True, configuration_revision=False, mode_changes_sent=False,
            current_compute_deployment=compute_ref, prepared_ns=time.time_ns(),
            physical_scene_stationarity='No arm motion requested. SDK pose/epoch monitored; RGB scene drift and temporal qualification measured per capture.')
        plan['sources'][str(Path(__file__).resolve())] = s.digest(__file__)
        sha = s.save(OUT / 'plan.json', plan)
        print(json.dumps(dict(prepared=True, output=str(OUT), sha256=sha,
            preserved_nodes=len(plan['preserved']), motion_authorized=False)), flush=True)
    finally:
        monitor.close()


if __name__ == '__main__':
    main()
