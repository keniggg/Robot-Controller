"""Bind a freshly reviewed scene and actuation evidence to the existing launcher."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
G = N/'gen56_runtime'
CONTROL = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/control.json'
REPLY = G/'user_scene_clear_reply_r3.json'
COLLECTOR_BASE = W/'.runtime/capture_gen56_actuation_start_r2_20261020.py'
PINS = {
    W/'.runtime/read_gen35_readiness_20261016.py': '5930828ffdd1773bafae2ab3be6f3aeb07e678e2a773b2800885ec0178c9e64a',
    W/'.runtime/prepare_current_session_continuation_receipt_20261015.py': 'aa1a2e2313c0f64c3cdb4c5b86544fcdfa90bffbc638c6be1fe48c862d71c0ac',
    W/'.runtime/start_gen56_recorded_attempt_ready_20261020.py': '17413b9c738a5791fb00b1ca639a4ea4ec654119c2780c9354098f770f9261b5',
    N/'verification/gen56_release_manifest.json': '784af71745650269b1d114a5f9cbba32eaa792f8cf2029825eb5a56d52a8f255',
    G/'gen56_runtime_deployment.json': 'd02a3dde8f8b7d52b93ed57ce958a64526a68d11a912dd518435ffcd74995f85',
    CONTROL: '2e0a1f2c696cb433ecc5e4bb98ec70e47ffa563736144dc98476bd78d4605ba7',
    REPLY: 'b3639ad86ff29529c6bc6c8ba5f5fa97ebb096c6528bbbc41490e7a1a3fa47bb',
    N/'gen57_resume_r1/user_resume_request.json': '0dc895a7ba8ae43ea7ae9825d4a1080e892721915c241ad09e9315fc195bbbe4',
    W/'tools/run_recorded_grasp.py': '4274c27330bb45eeaacc989226162e42cc2a1a796693928f1cf3a20369dffd9e',
    COLLECTOR_BASE: '9a0ee83928a299d6c2c3f8668f4fff703883b2ee0a819a00aa104946e1213aa9',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def ref(path):
    return dict(path=str(path), sha256=sha(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-index', type=int, required=True)
    parser.add_argument('--assessment', required=True)
    args = parser.parse_args()
    assert args.scene_index >= 5 and args.assessment.strip()
    assert E.parents[1].is_mount()
    assert all(sha(p) == h for p, h in PINS.items())
    control, reply = read(CONTROL), read(REPLY)
    assert control['state'] == 'active' and control['revision'] == 64
    assert reply['message_id'] == 'call_41f049f5ac72464d9b53f90b983531bc:0'
    assert reply['answer'] == '双手和前臂已移开，可以重新采图'
    suffix = 'r%d' % args.scene_index
    scene = G/('scene_start_'+suffix)
    ready_path = G/('readiness_start_'+suffix)/'result.json'
    act_path = G/('actuation_start_'+suffix)/'result.json'
    pre, ready, act = read(scene/'preflight.json'), read(ready_path), read(act_path)
    assert ready['passed'] is True and act['passed'] is True
    assert ready['deployment_processes_unchanged'] and ready['source_matches_operational_release']
    assert act['motion_enabled'] and act['feedback_ready']
    assert act['actuation_status'].startswith('CONFIRMED:')
    assert ready['epoch_ns'] == act['epoch_ns'] == pre['epoch_ns']
    assert ready['selection'] == act['selection'] == pre['selection']
    q = pre['topics']['/alicia_d/accepted_joint_states']
    assert q['names'] == ready['names']
    assert len(q['positions']) == len(ready['positions'])
    assert max(abs(a-b) for a, b in zip(q['positions'], ready['positions'])) <= .004
    now = time.time_ns()
    assert pre['receipt_ns'] <= ready['recorded_ns'] <= now
    assert pre['receipt_ns'] <= act['recorded_ns'] <= now
    assert 0 <= now-pre['receipt_ns'] < 90_000_000_000
    release = read(N/'verification/gen56_release_manifest.json')
    assert all(sha(W/p) == h for p, h in release['source_sha256'].items())
    deployment = read(G/'gen56_runtime_deployment.json')
    nodes = dict(deployment['after'], **deployment['preserved_after'])
    collector = W/('.runtime/capture_gen56_actuation_start_%s_20261020.py' % suffix)
    base_text = COLLECTOR_BASE.read_text()
    original_output = "OUTPUT = N/'gen56_runtime/actuation_start_r2'"
    assert base_text.count(original_output) == 1
    expected_collector = base_text.replace(original_output,
        "OUTPUT = N/'gen56_runtime/actuation_start_%s'" % suffix).encode()
    expected_collector_sha = hashlib.sha256(expected_collector).hexdigest()
    assert collector.read_bytes() == expected_collector
    assert act['helper_path'] == str(collector) and act['helper_sha256'] == expected_collector_sha
    assert act['input_sha256'] == {str(p): PINS[p] for p in (
        N/'verification/gen56_release_manifest.json', G/'gen56_runtime_deployment.json', CONTROL)}
    assert act['source_sha256'] == release['source_sha256']
    assert act['node_identities_before'] == act['node_identities_after'] == act['node_identities'] == nodes
    assert act['unique_sample_count'] >= 20
    for item in nodes.values():
        stat = Path('/proc/%d/stat' % item['pid']).read_text().rsplit(')', 1)[1].split()
        assert stat[0] != 'Z' and stat[19] == item['start_ticks']
        assert int(stat[1]) == item['parent_pid']
    os.environ['ROS_LOG_DIR'] = str(scene/'prelaunch_ros')
    import rospy
    from std_msgs.msg import String, Bool, Header
    rospy.init_node('gen56_prelaunch_actuation_readonly', anonymous=True, disable_rosout=True)
    received_ns = {}
    def observe(topic, kind):
        value = rospy.wait_for_message(topic, kind, timeout=3)
        received_ns[topic] = time.time_ns()
        return value
    status = observe('/alicia_d/actuation_status', String).data
    motion = observe('/alicia_d/motion_enabled', Bool).data
    feedback = observe('/alicia_d/feedback_ready', Bool).data
    epoch = observe('/alicia_d/control_reference_epoch', Header).stamp.to_nsec()
    required = rospy.get_param('/grasp/require_actuation_confirmation', None)
    assert required is True and status.startswith('CONFIRMED:')
    assert motion is True and feedback is True and epoch == pre['epoch_ns']
    assert rospy.get_param('/gui/joint_direct_mode') is True
    assert rospy.get_param('/grasp_mode/selection') == pre['selection']
    checked_ns = time.time_ns()
    assert all(0 <= checked_ns-stamp < 1_000_000_000 for stamp in received_ns.values())
    proof = dict(recorded_ns=time.time_ns(), actuation_status=status, motion_enabled=motion,
        feedback_ready=feedback, require_actuation_confirmation=required, epoch_ns=epoch,
        actuation_evidence=ref(act_path), readiness=ref(ready_path),
        image=ref(scene/'current_scene.png'), visual_assessment=args.assessment,
        current_image_review_no_conflicting_hand_in_motion_area=True,
        current_user_reply=ref(REPLY), state_received_ns=received_ns,
        motion_commands_sent=False, helper=ref(Path(__file__)))
    with (scene/'prelaunch_actuation_and_image_review.json').open('x') as stream:
        json.dump(proof, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    rospy.signal_shutdown('read-only evidence complete')
    assert all(sha(p) == h for p, h in PINS.items())
    assert sha(collector) == expected_collector_sha
    subprocess.run([sys.executable, '-B',
        str(W/'.runtime/prepare_current_session_continuation_receipt_20261015.py'),
        '--session', str(G), '--scene', str(scene), '--readiness', str(ready_path),
        '--visual-assessment', args.assessment, '--control-revision', '64',
        '--user-request', '继续处理',
        '--clearance-reply-id', 'call_41f049f5ac72464d9b53f90b983531bc:0',
        '--clearance-text', '双手和前臂已移开，可以重新采图'], check=True)
    os.execv(sys.executable, [sys.executable, '-B',
        str(W/'.runtime/start_gen56_recorded_attempt_ready_20261020.py'),
        '--scene', str(scene), '--startup-binding', str(G/'remote_startup_binding_r1/startup_binding.json'),
        '--startup-binding-sha256', 'cd9f737f65a7772bf3e81dadabd19ad745c6716341ec9c2b74deb0819dda5ae8',
        '--checker-sha256', 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'])


if __name__ == '__main__':
    main()
