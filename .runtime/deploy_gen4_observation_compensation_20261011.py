"""Load reviewed observation changes in idle gateway and three compute nodes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', type=Path, required=True)
    ap.add_argument('--previous-deployment', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    w = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in a.output.resolve().parents
    manifest = json.loads(a.manifest.read_text())
    previous = json.loads(a.previous_deployment.read_text())
    assert manifest['passed'] is True and manifest['reviewed'] is True
    assert previous['deployment_passed'] is True
    for p, sha in manifest['source_sha256'].items():
        assert hashlib.sha256((w/p).read_bytes()).hexdigest() == sha, p
    a.output.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(a.output/'observer_ros')
    import rospy
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from std_msgs.msg import String
    rospy.init_node('deploy_observation_compensation', anonymous=True, disable_rosout=True)
    result = dict(deployment_passed=False, source_manifest=str(a.manifest),
        previous_deployment=str(a.previous_deployment), motion_commands_sent=False,
        torque_commands_sent=False, temperature_queried=False)

    def save():
        (a.output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')

    try:
        assert not rospy.wait_for_message('/grasp/state', GraspState, timeout=5).active
        mode = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=5).data)
        assert not any(mode.get(k) for k in ('active', 'start_reserved', 'pending'))
        params = {'/grasp_6d/remote/observation_camera_visibility_margin_px': 24.0,
                  '/grasp/observation_endpoint_compensation_enabled': True,
                  '/grasp/unknown_pregrasp_position_preference_m': 0.006}
        result['parameter_changes'] = {k: dict(previous=rospy.get_param(k, None), requested=v)
                                       for k, v in params.items()}
        result['generic_visibility_margin_before'] = rospy.get_param('/grasp_6d/remote/camera_visibility_margin_px')
        save()
        for name, value in params.items():
            rospy.set_param(name, value)
        assert all(rospy.get_param(k) == v for k, v in params.items())
        gateway_output = a.output/'gateway'
        subprocess.run(['/usr/bin/python3', str(w/'.runtime/reload_idle_contact_gateway_gen14_20261010.py'),
            '--manifest', str(a.manifest), '--previous-baseline', str(a.previous_deployment),
            '--output', str(gateway_output)], cwd=w, check=True, timeout=150.)
        gateway = json.loads((gateway_output/'result.json').read_text())
        assert gateway['deployment_passed'] is True
        rospy.wait_for_service('/supervisor/compensate_observation_endpoint', timeout=10.)
        baseline = dict(previous)
        baseline['preserved_after'] = dict(previous['preserved_after'], **{'/motion_gateway': gateway['after']})
        baseline['gateway_reload_evidence'] = str(gateway_output/'result.json')
        assert all(gateway['preserved_after'][n] == p for n,p in previous['after'].items())
        baseline_path = a.output/'runtime_after_gateway.json'
        baseline_path.write_text(json.dumps(baseline, ensure_ascii=False, indent=2)+'\n')
        compute_output = a.output/'compute'
        subprocess.run(['/usr/bin/python3', str(w/'.runtime/reload_dedicated_observation_consumers_20261009.py'),
            '--manifest', str(a.manifest), '--previous-deployment', str(baseline_path),
            '--output', str(compute_output)], cwd=w, check=True, timeout=180.)
        compute = json.loads((compute_output/'result.json').read_text())
        assert compute['deployment_passed'] is True
        assert all(rospy.get_param(k) == v for k, v in params.items())
        assert rospy.get_param('/grasp_6d/remote/camera_visibility_margin_px') == result['generic_visibility_margin_before']
        result.update(compute)
        result.update(deployment_passed=True, completed_ns=time.time_ns(),
            gateway_reload_evidence=str(gateway_output/'result.json'),
            compute_reload_evidence=str(compute_output/'result.json'),
            integrated_previous_deployment=str(a.previous_deployment),
            parameters_after=params, physical_grasp_success=False)
    except Exception as exc:
        result.update(deployment_passed=False, error=repr(exc))
        raise
    finally:
        save()
        print(json.dumps(dict(deployment_passed=result['deployment_passed'],
            output=str(a.output), error=result.get('error')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
