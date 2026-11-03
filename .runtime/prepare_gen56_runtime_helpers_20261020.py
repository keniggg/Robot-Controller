"""Prepare new GEN56 operational helpers; never modify GEN55 or contact ROS."""
import ast
import hashlib
import json
from pathlib import Path
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V = N/'verification'
O = V/'gen56_runtime_helper_preparation_r1'
PREVIOUS = N/'gen55_runtime/gen55_runtime_deployment.json'
PREVIOUS_SHA = '7e9ee66527515dfc22e919ddc754bb6742b3f5067c714aa3bceb64da2c8732b8'
BASE_SHA = '0f93bc7eb5561f11407fb7cc986344e214e9f0877125bf759068a3007cfd6937'
BASELINE_SHA = '75c02bc24f59759ff62dd563350c9a1c198ded6ea0fba669e057c25460cfea2e'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replace(text, old, new):
    assert text.count(old) == 1, old[:150]
    return text.replace(old, new)


def name(old):
    return old.replace('gen55', 'gen56').replace('20261019', '20261020')


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
    assert sha(PREVIOUS) == PREVIOUS_SHA
    assert sha(V/'gen55_release_manifest.json') == BASE_SHA
    assert sha(W/'.runtime/gen56_baseline_source/runner_manifest.json') == BASELINE_SHA
    files = ['gen55_idle_reload_common_20261019.py',
        'read_gen55_deployment_preflight_20261019.py',
        'read_gen55_preinstall_readiness_20261019.py',
        'load_gen55_idle_compute_gateway_20261019.py',
        'gen55_observation_measurement_consumers_20261019.launch',
        'start_gen55_observation_measurement_consumers_20261019.sh',
        'finalize_gen55_idle_deployment_20261019.py',
        'start_gen55_recorded_attempt_20261019.sh',
        'start_gen55_recorded_attempt_ready_20261020.py']
    original = {f:(W/'.runtime'/f).read_text() for f in files}
    assert all(not (W/'.runtime'/name(f)).exists() for f in files)
    result = {}
    for filename, text in original.items():
        # Only copied operational helper names/generation labels change here.
        # Python body changes below are explicit and preserved in a diff.
        result[filename] = text.replace('gen55', 'gen56').replace('GEN55', 'GEN56')
        for old in files + ['verify_gen55_parallel_offline_20261019.py']:
            interim = old.replace('gen55', 'gen56')
            result[filename] = result[filename].replace(interim, name(old))
    key = files[0]; text = result[key]
    text = replace(text, "BASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'", 'BASE_SHA = '+repr(BASE_SHA))
    text = replace(text, "PREVIOUS = E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'", "PREVIOUS = N/'gen55_runtime/gen55_runtime_deployment.json'")
    text = replace(text, "PREVIOUS_SHA = '7e256ee6f5cb64756d40e4c2b07468a791d10704c81d4154144b141a11f23bd3'", 'PREVIOUS_SHA = '+repr(PREVIOUS_SHA))
    text = replace(text, "BASELINE = W/'.runtime/gen54_baseline_source/runner_manifest.json'", "BASELINE = W/'.runtime/gen56_baseline_source/runner_manifest.json'")
    text = replace(text, "BASELINE_SHA = 'e0ea9529ff6afdff8a6794578f1e64d996cdb124ee1adf3e6732e38e44984748'", 'BASELINE_SHA = '+repr(BASELINE_SHA))
    text = replace(text, "HISTORICAL_XML = N/'verification/gen53_full_parallel_r1/pytest.xml'", "HISTORICAL_XML = N/'verification/gen55_full_parallel_r2/pytest.xml'")
    tree = ast.parse(text)
    assignment = next(n for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'HISTORICAL_XML_SHA' for t in n.targets))
    text = replace(text, ast.get_source_segment(text, assignment), 'HISTORICAL_XML_SHA = '+repr(sha(V/'gen55_full_parallel_r2/pytest.xml')))
    text = text.replace('previous GEN54', 'previous GEN55')
    result[key] = text
    key = files[2]; text = result[key]
    text = text.replace('GEN54', 'GEN55').replace('gen54_release_manifest', 'gen55_release_manifest')
    text = replace(text, "RELEASE_SHA = 'ec0d1c1d2ebdb88fc9c77174790190dfff71024e58fea6958b356cb9ebf5d3f8'", 'RELEASE_SHA = '+repr(BASE_SHA))
    text = replace(text, "DEPLOYMENT = E/'ros_log/full_validated_gen54_restart_20261020T012700Z/gen54_current_runtime_identity.json'", "DEPLOYMENT = N/'gen55_runtime/gen55_runtime_deployment.json'")
    text = replace(text, "DEPLOYMENT_SHA = '7e256ee6f5cb64756d40e4c2b07468a791d10704c81d4154144b141a11f23bd3'", 'DEPLOYMENT_SHA = '+repr(PREVIOUS_SHA))
    text = replace(text, '== 1075', '== 1082')
    text = replace(text, "    args = parser.parse_args()", "    parser.add_argument('--expected-generation', type=int, required=True)\n    args = parser.parse_args()")
    text = replace(text, "type(selection['generation']) is int and selection == deployment['selection']", "type(selection['generation']) is int and selection['generation'] == args.expected_generation\n        assert selection['generation'] >= deployment['selection']['generation']\n        assert int(selection['stamp_ns']) >= int(deployment['selection']['stamp_ns'])")
    result[key] = text
    key = files[1]; text = result[key]
    text = replace(text, '    args = parser.parse_args()', "    parser.add_argument('--expected-generation', type=int, required=True)\n    args = parser.parse_args()")
    text = replace(text, "type(selection.get('generation')) is int and selection == previous['selection']", "type(selection.get('generation')) is int and selection['generation'] == args.expected_generation\n            and selection['generation'] >= previous['selection']['generation']\n            and int(selection['stamp_ns']) >= int(previous['selection']['stamp_ns'])")
    text = text.replace('GEN54', 'GEN55').replace('expected unchanged accepted cold-start selection with exact source stamp', 'expected current selection generation and monotonic source stamp')
    result[key] = text
    result[files[3]] = result[files[3]].replace('GEN54', 'GEN55')
    key = files[6]; text = result[key].replace('GEN54', 'GEN55')
    # The five reviewed reload helpers are immutable inputs to finalization.
    helper_sha = {str(W/'.runtime'/name(old)):hashlib.sha256(result[old].encode()).hexdigest() for old in files[:2]+files[3:6]}
    tree = ast.parse(text)
    assignment = next(n for n in tree.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'HELPER_SHA' for t in n.targets))
    text = replace(text, ast.get_source_segment(text, assignment), 'HELPER_SHA = '+repr(helper_sha))
    result[key] = text
    # New recorded entry retains the old ownership/recording checks and uses
    # the already reviewed remote checker before any automatic-mode selection.
    key = files[8]; text = result[key]
    new_launcher_sha = hashlib.sha256(result[files[7]].encode()).hexdigest()
    text = replace(text, "LAUNCHER_SHA = '310fcefccf41c3fe79757df4831dba24e245fde9b9c10851ac7eabf57765f2a5'", 'LAUNCHER_SHA = '+repr(new_launcher_sha))
    result[key] = text
    import difflib
    O.mkdir()
    for filename, text in result.items():
        target = W/'.runtime'/name(filename)
        if target.suffix == '.py': ast.parse(text)
        with target.open('x') as stream: stream.write(text)
        assert target.read_text() == text
        (O/(target.name+'.diff')).write_text(''.join(difflib.unified_diff(original[filename].splitlines(True), text.splitlines(True), fromfile=filename, tofile=target.name)))
    receipt = dict(passed=True, stage='draft_requires_independent_review_before_use',
        original_helper_sha256={str(W/'.runtime'/p):sha(W/'.runtime'/p) for p in original},
        new_helper_sha256={str(W/'.runtime'/name(p)):sha(W/'.runtime'/name(p)) for p in original},
        source_installed=False, runtime_reloaded=False, motion_commands_sent=False,
        previous_deployment_sha256=PREVIOUS_SHA, baseline_release_sha256=BASE_SHA,
        recorded_ns=time.time_ns())
    (O/'result.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(O),helpers=len(result),source_modified=False)))


if __name__ == '__main__':
    main()
