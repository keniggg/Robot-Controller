"""One recorded cold-session attempt; original runner and checks stay byte exact."""
import ast
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

from gen58_cold_attempt_common_20261021 import (
    A, AUTH_SHA, CHECKER, CHECKER_SHA, CONTROL_SHA, E, S, W, checked_module, input_receipts, load_context,
    CONTINUE_REQUEST, CONTINUE_REQUEST_SHA,
    new_json, parser, read, recheck, ref, require, sha, storage)

ORIGINAL = W / '.runtime/run_reboot_realigned_grasp_20261010.py'
ORIGINAL_SHA = 'cb7996419e32954b683fc4b148f54c9e467f2a8b6f259664fa2a5bb1d0d5ed11'
CONTINUATION = W / '.runtime/continuous_grasp_authorization_20261012.py'
CONTINUATION_SHA = '916f5b03585ce26402c50515d88592b27354650c85aab3c9ea8c2538e76f8454'
PREWARM = W / '.runtime/prewarm_mode_grasp_runner_20261012.py'
PREWARM_SHA = '3e5a0798bccaf49ea15cb8eac977d2621cfe48a78e8ae141bf324f4154e2fa49'
RECORD_WRAPPER = W / 'tools/run_recorded_grasp.py'
RECORD_WRAPPER_SHA = '4274c27330bb45eeaacc989226162e42cc2a1a796693928f1cf3a20369dffd9e'
RECORDER = W / 'tools/record_grasp_keyframes_readonly.py'
RECORDER_SHA = 'b8b3eae0138aaa7f4e42e68da0f7b8a8d76f1f8c4ac266c2e888e5a191b81748'
ARCHIVE = W / 'tools/grasp_archive.py'
ARCHIVE_SHA = '9aaed6157e8ba761994d168a326c50ed6ef75ebf7cba93f43d8f4c7b57f9f136'
COLLECTOR = W / '.runtime/gen58_cold_attempt_capture_20261021.py'
COLLECTOR_SHA = '0361ceeaafdaec09cde6cf193ada05af1c27d5c58a9e80036b914d116061dd7a'
WRITER = W / '.runtime/gen58_cold_attempt_receipt_20261021.py'
WRITER_SHA = '55ff111ef2b9cf1c77f42bd188e3dce41208297c29770d2c7e501310e8e7878f'
GATE = W / '.runtime/gen58_cold_attempt_gate_20261021.py'
REQUEST_TEXT = '你一直进行处理，直到成功抓取'


def original_boundary():
    require(sha(ORIGINAL) == ORIGINAL_SHA, 'original runner bytes changed')
    raw = ORIGINAL.read_text()
    calls = [node for node in ast.walk(ast.parse(raw)) if isinstance(node, ast.Call) and
             isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name) and
             node.func.value.id == 'rospy' and node.func.attr == 'set_param' and
             len(node.args) == 2 and isinstance(node.args[0], ast.Constant) and
             node.args[0].value == '/gui/joint_direct_mode' and
             isinstance(node.args[1], ast.Constant) and node.args[1].value is False and not node.keywords]
    require(len(calls) == 1 and raw.splitlines()[calls[0].lineno - 1].strip() ==
            "rospy.set_param('/gui/joint_direct_mode', False)", 'unique original mode boundary differs')
    return calls[0].lineno


def run_with_line_gate(path, expected_sha, target_line, gate, run_callable):
    """Pure tracing adapter; an exception prevents execution of the bound line."""
    path = Path(path).resolve(strict=True)
    require(sha(path) == expected_sha and type(target_line) is int and target_line > 0,
            'trace source binding invalid')
    previous = sys.gettrace()
    require(previous is None, 'unexpected existing tracer; do not silently replace it')
    hits = 0

    def trace(frame, event, _arg):
        nonlocal hits
        if frame.f_code.co_filename != str(path):
            return None
        if event == 'line' and frame.f_lineno == target_line:
            hits += 1
            require(hits == 1 and sha(path) == expected_sha, 'mode gate repeated or source changed')
            gate(frame)
        return trace

    try:
        sys.settrace(trace)
        result = run_callable()
        require(hits == 1, 'original runner never reached the required mode gate')
        return result
    finally:
        sys.settrace(previous)


def scene_context(a):
    context = load_context(a, __file__)
    scene = a.scene.resolve(strict=True)
    require(scene.parent == S and scene.name.startswith('scene_start_'), 'fresh cold scene path required')
    observation_path, preparation_path = scene / 'operator_observation.json', scene / 'receipt_preparation.json'
    require(sha(observation_path) == a.observation_sha256 and sha(preparation_path) == a.receipt_sha256,
            'reviewed scene observation/receipt changed')
    observation, preparation = read(observation_path), read(preparation_path)
    require(preparation['passed'] is True and preparation['helper_path'] == str(WRITER) and
            preparation['helper_sha256'] == WRITER_SHA and preparation['observation'] == ref(observation_path) and
            preparation['control_revision'] == 67, 'honest observation writer acceptance missing')
    require(observation['preparation_script'] == ref(WRITER) and observation['control_revision'] == 67 and
            observation['control_sha256'] == CONTROL_SHA and observation['new_user_confirmation_claimed'] is False and
            observation['cold_deployment'] == ref(context['deployment_path']) and
            observation['release'] == ref(context['release_path']) and
            observation['latest_continue_request'] == dict(path=str(CONTINUE_REQUEST),
                sha256=CONTINUE_REQUEST_SHA,source='actual_user_message',new_scene_confirmation_claimed=False) and
            observation['continuing_authorization'] == dict(path=str(A), sha256=AUTH_SHA),
            'current reviewed observation binding differs')
    ready_path = Path(observation['readiness']['path']).resolve(strict=True)
    review_path = Path(observation['hands_clear_basis']['current_image_review']['path']).resolve(strict=True)
    require(preparation['readiness'] == observation['readiness'] == ref(ready_path) and
            preparation['review'] == observation['hands_clear_basis']['current_image_review'] == ref(review_path),
            'readiness/review provenance changed')
    ready, review, pre = read(ready_path), read(review_path), read(scene / 'preflight.json')
    require(ready['passed'] is True and ready['helper_path'] == str(COLLECTOR) and
            ready['helper_sha256'] == COLLECTOR_SHA and ready['scene'] == str(scene) and
            ready['source_sha256'] == context['release']['source_sha256'] and
            ready['node_identities_before'] == ready['node_identities_after'] == context['nodes'] and
            ready['release'] == ref(context['release_path']) and ready['deployment'] == ref(context['deployment_path']) and
            ready['epoch_ns'] == pre['epoch_ns'] == context['deployment']['epoch_ns'] and
            ready['selection'] == pre['selection'] == context['deployment']['selection'],
            'collector is not the current complete cold release')
    # Compare path/digest separately: observation additionally records image source stamp.
    require(review['source'] == 'actual_current_image_review' and review['new_user_confirmation_claimed'] is False and
            review['same_target_observed'] is True and review['target_visible'] is True and
            review['current_image_review_no_conflicting_hand_in_motion_area'] is True and
            review['image'] == ref(scene / 'current_scene.png') and review['preflight'] == ref(scene / 'preflight.json') and
            review['readiness'] == ref(ready_path) and review['assessed_ns'] == observation['assessed_ns'],
            'actual current review evidence differs')
    pins = {ORIGINAL: ORIGINAL_SHA, CONTINUATION: CONTINUATION_SHA, PREWARM: PREWARM_SHA,
            RECORD_WRAPPER: RECORD_WRAPPER_SHA, RECORDER: RECORDER_SHA, ARCHIVE: ARCHIVE_SHA,
            COLLECTOR: COLLECTOR_SHA, WRITER: WRITER_SHA, GATE: a.gate_sha256,
            observation_path: a.observation_sha256, preparation_path: a.receipt_sha256,
            ready_path: observation['readiness']['sha256'], review_path: preparation['review']['sha256'],
            scene / 'preflight.json': observation['preflight']['sha256'],
            scene / 'current_scene.png': observation['image']['sha256']}
    context['pins'].update(pins)
    recheck(context)
    continuation = checked_module(CONTINUATION, CONTINUATION_SHA, 'continuous_grasp_authorization_20261012')
    # The constructor is validation only; the original runner alone claims the observation.
    check = continuation.ContinuationAttempt(A, E / 'grasp_records', scene, REQUEST_TEXT, time.time_ns())
    require(0 <= time.time_ns() - pre['receipt_ns'] < 90_000_000_000, 'scene is over 90 seconds old')
    return context, scene, check


def main():
    p = parser(__doc__)
    p.add_argument('--phase', choices=('launch', 'recorded'), required=True)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--observation-sha256', required=True)
    p.add_argument('--receipt-sha256', required=True)
    p.add_argument('--gate-sha256', required=True)
    a = p.parse_args()
    scene = a.scene.resolve(strict=True)
    output = storage(scene / ('recording_launch_result.json' if a.phase == 'launch' else 'recorded_entry_result.json'))
    require(not output.exists(), 'one invocation only; existing result must remain')
    result = dict(kind='GEN58_cold_one_recorded_attempt_' + a.phase, passed=False,
                  started_ns=time.time_ns(), helper=ref(Path(__file__)), new_motion_authority_created=False,
                  original_guards_replaced=False, automatic_retry=False, temperature_queried=False,
                  physical_success='unverified', gate_returned_to_original_mode_line=False)
    try:
        context, scene, validated = scene_context(a)
        target_line = original_boundary()
        result.update(input_sha256=input_receipts(context), original_mode_line=target_line)
        if a.phase == 'launch':
            require('GRASP_RECORD_DIR' not in os.environ, 'outer launcher cannot attach to an existing record')
            remote_output = scene / 'remote_dependency_before_recording'
            require(not remote_output.exists(), 'one pre-record dependency check only')
            probe = subprocess.run([sys.executable, '-B', str(CHECKER), '--output', str(remote_output),
                '--startup-binding', str(context['binding_path']), '--startup-binding-sha256',
                context['pins'][context['binding_path']]], cwd=W, timeout=20.,
                env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
            remote = read(remote_output / 'result.json')
            result['remote_dependency_before_recording'] = ref(remote_output / 'result.json')
            require(probe.returncode == 0 and remote['passed'] is True and remote['dependency_ready'] is True and
                    remote['helper_sha256'] == CHECKER_SHA and
                    remote['startup_binding_sha256'] == context['pins'][context['binding_path']] and
                    remote['after']['selection'] == validated.preflight['selection'],
                    'current remote dependency not ready; no recording started')
            validated.check_active(time.time_ns())
            recheck(context)
            require(0 <= time.time_ns() - remote['finished_ns'] <= 5_000_000_000 and
                    time.time_ns() - validated.preflight['receipt_ns'] < 90_000_000_000,
                    'remote/scene freshness expired before recording launch')
            arguments = list(sys.argv[1:])
            require(arguments.count('--phase') == 1, 'one explicit phase argument required')
            arguments[arguments.index('--phase') + 1] = 'recorded'
            inner = [sys.executable, '-B', str(Path(__file__).resolve())] + arguments
            command = [sys.executable, '-B', str(RECORD_WRAPPER), '--root', str(E / 'grasp_records'),
                       '--duration', '600', '--'] + inner
            environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1',
                PYTHONUNBUFFERED='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                ROS_LOG_DIR=str(scene / 'recorded_attempt_ros'))
            launch = dict(owner=context['verifier'].process(os.getpid()), created_ns=time.time_ns(),
                source=ref(Path(__file__)), gate=ref(GATE), inner_command=inner, record_command=command,
                observation=ref(scene / 'operator_observation.json'), release=ref(context['release_path']),
                deployment=ref(context['deployment_path']), original_runner=ref(ORIGINAL),
                no_new_motion_authority=True, one_recording_only=True)
            new_json(scene / 'recording_launch_claim.json', launch)
            result.update(passed=True, exec_requested=True, recording_launch=ref(scene / 'recording_launch_claim.json'))
            new_json(output, result)
            os.chdir(W)
            os.execve(sys.executable, command, environment)
        record = Path(os.environ['GRASP_RECORD_DIR']).resolve(strict=True)
        require(record.parent == E / 'grasp_records', 'record is not a current external recording')
        launch = read(scene / 'recording_launch_claim.json')
        require(launch['source'] == ref(Path(__file__)) and launch['gate'] == ref(GATE) and
                launch['inner_command'] == [sys.executable, '-B', str(Path(__file__).resolve())] + sys.argv[1:] and
                launch['owner']['pid'] == os.getppid() and
                launch['observation'] == ref(scene / 'operator_observation.json') and
                launch['release'] == ref(context['release_path']) and launch['deployment'] == ref(context['deployment_path']),
                'inner command is not this exact one-record launch')
        gate_module = checked_module(GATE, a.gate_sha256, 'gen58_cold_attempt_final_gate')
        original = checked_module(ORIGINAL, ORIGINAL_SHA, 'gen58_original_unchanged_runner')
        original_argv = sys.argv
        sys.argv = [str(ORIGINAL), '--session-dir', str(scene), '--user-message', REQUEST_TEXT,
                    '--continuation-authorization', str(A)]

        def at_mode_line(frame):
            require(frame.f_code.co_name == 'main', 'mode boundary outside original main')
            gate_result = dict(kind='GEN58_additional_mode_boundary_gate', passed=False,
                started_ns=time.time_ns(), entry=ref(Path(__file__)), gate=ref(GATE),
                original=ref(ORIGINAL), original_line=target_line, record_dir=str(record),
                snapshot_only=True, automatic_mode_write_confirmed=False,
                new_motion_authority_created=False, original_guards_replaced=False)
            try:
                gate_module.final_gate(context, scene, record, launch, frame, gate_result)
                result['gate_returned_to_original_mode_line'] = True
            except BaseException:
                gate_result.update(passed=False, error=traceback.format_exc(), failed_ns=time.time_ns())
                new_json(scene / 'mode_switch_gate_failure.json', gate_result)
                raise

        try:
            code = run_with_line_gate(ORIGINAL, ORIGINAL_SHA, target_line, at_mode_line, original.main)
        finally:
            sys.argv = original_argv
        result.update(passed=code == 0, runner_returncode=code, record_dir=str(record),
                      original_runner_returned=True, finished_ns=time.time_ns())
    except BaseException as exc:
        result.update(passed=False, error=repr(exc), traceback=traceback.format_exc(), finished_ns=time.time_ns())
    if not output.exists():
        new_json(output, result)
    elif not result['passed']:
        new_json(scene / 'recording_exec_failure.json', result)
    print(result)
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
