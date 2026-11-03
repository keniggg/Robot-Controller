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
REPLY = G/'user_scene_clear_reply_r4.json'
COLLECTOR_BASE = W/'.runtime/capture_gen56_actuation_start_r6_20261020.py'
PINS = {
    W/'.runtime/read_gen35_readiness_20261016.py': '5930828ffdd1773bafae2ab3be6f3aeb07e678e2a773b2800885ec0178c9e64a',
    W/'.runtime/prepare_current_session_continuation_receipt_20261015.py': 'aa1a2e2313c0f64c3cdb4c5b86544fcdfa90bffbc638c6be1fe48c862d71c0ac',
    W/'.runtime/start_gen56_recorded_attempt_ready_20261020.py': '17413b9c738a5791fb00b1ca639a4ea4ec654119c2780c9354098f770f9261b5',
    N/'verification/gen56_release_manifest.json': '784af71745650269b1d114a5f9cbba32eaa792f8cf2029825eb5a56d52a8f255',
    G/'gen56_runtime_deployment.json': 'd02a3dde8f8b7d52b93ed57ce958a64526a68d11a912dd518435ffcd74995f85',
    CONTROL: 'b2177bee366a8b5ef90ebd9e9fc4373a6bb8305599ab8cb68235c49e36cea662',
    REPLY: 'abc21065c36c583550eedcee90b5c07a3007f3c96553bf77206f80bd82373434',
    N/'gen57_resume_r2/user_resume_request.json': 'cf5609f8cf146561583f8645f372c3f58d5b80a3e00a423135689702719c08ed',
    W/'tools/run_recorded_grasp.py': '4274c27330bb45eeaacc989226162e42cc2a1a796693928f1cf3a20369dffd9e',
    COLLECTOR_BASE: '6141605e7221bf010ee7e9b4eeb1d4b372b2d56dfd196c66e0f102f15c71747e',
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
    assert args.scene_index >= 6 and args.assessment.strip()
    assert E.parents[1].is_mount()
    assert all(sha(p) == h for p, h in PINS.items())
    control, reply = read(CONTROL), read(REPLY)
    assert control['state'] == 'active' and control['revision'] == 66
    assert reply['message_id'] == 'call_8c098ed1801e4190bb8d118b207c8769:0'
    assert reply['answer'] == '双手和前臂已移开，可以重新检查'
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
    original_output = "OUTPUT = N/'gen56_runtime/actuation_start_r6'"
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
        '--visual-assessment', args.assessment, '--control-revision', '66',
        '--user-request', '你继续处理，并说明max档位下运行的3个子代理分别指派了什么任务',
        '--clearance-reply-id', 'call_8c098ed1801e4190bb8d118b207c8769:0',
        '--clearance-text', '双手和前臂已移开，可以重新检查'], check=True)
    os.execv(sys.executable, [sys.executable, '-B',
        str(W/'.runtime/start_gen56_recorded_attempt_ready_20261020.py'),
        '--scene', str(scene), '--startup-binding', str(G/'remote_startup_binding_r1/startup_binding.json'),
        '--startup-binding-sha256', 'cd9f737f65a7772bf3e81dadabd19ad745c6716341ec9c2b74deb0819dda5ae8',
        '--checker-sha256', 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'])


if __name__ == '__main__':
    main()
