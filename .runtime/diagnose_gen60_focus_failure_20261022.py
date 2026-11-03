"""Finite diagnostic of one failed contract using its immutable worker guard."""
from pathlib import Path
import hashlib
import json
import os
import sys
from copy import deepcopy

W = Path(__file__).resolve().parents[1]
C = W/'.runtime/gen60_held_lattice_candidate_r1'
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1')
O = E/'focus_failure_review_r2'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
M = E/'verification/freeze_r1/manifest_closed.json'
assert sha(M) == 'ba935ed33ebe24a3e8815bb32a05a50e951683adedb5b089e34a8b304d9729df'
manifest = json.loads(M.read_bytes())
assert all(sha(C/p) == h for p,h in manifest['copied_input_sha256'].items())
assert 'sitecustomize' in sys.modules and os.environ['OFFLINE_BOUNDARY_LOG'] == str(O/'boundary_attempts.jsonl')
assert O.exists() and not (O/'result.json').exists()
from _pytest.monkeypatch import MonkeyPatch
import test_gen55_held_pregrasp_reuse as test
data = test.data.__wrapped__()
patch = MonkeyPatch()
try:
    test.test_unsuccessful_solver_retains_exact_failure_and_evidence(data,patch,'unqualified')
except AssertionError as error:
    tb = error.__traceback__
    while tb and tb.tb_frame.f_code.co_name != 'test_unsuccessful_solver_retains_exact_failure_and_evidence': tb = tb.tb_next
    assert tb is not None
    pairs = deepcopy(tb.tb_frame.f_locals['failures'])
else:
    raise AssertionError('expected historical assertion did not reproduce')
finally:
    patch.undo()
old,new = [json.loads(p[1]) for p in pairs]
audit = new['pregrasp_endpoint_selection']['quantized_neighborhood']
remaining = deepcopy(new)
del remaining['pregrasp_endpoint_selection']['quantized_neighborhood']
assert pairs[0][0] == pairs[1][0]
assert json.dumps(remaining,sort_keys=True) == pairs[0][1]
assert audit['success'] is False and not any(r['qualified'] for r in audit['checked'])
assert all(t['rejection'] == pairs[0][0] for t in new['pregrasp_endpoint_selection']['trials'])
assert all(sha(C/p) == h for p,h in manifest['copied_input_sha256'].items())
result = dict(kind='GEN60_one_failed_test_exact_evidence_diagnostic',passed=True,
    helper_sha256=sha(__file__),manifest_sha256=sha(M),test_sha256=sha(test.__file__),
    baseline_code=pairs[0][0],candidate_code=pairs[1][0],baseline_evidence=old,candidate_evidence=new,
    only_added_semantic_path='pregrasp_endpoint_selection.quantized_neighborhood',
    remaining_stable_JSON_bytes_exact=True,elapsed_rule='Existing stable() removes elapsed_sec recursively; unchanged.',
    original_three_trial_rejections=[r['rejection'] for r in new['pregrasp_endpoint_selection']['trials']],
    quantized_neighborhood=audit,full_regression_passed=False,source_unchanged=True,
    live_ROS_contacted=False,hardware_contacted=False,motion_commands_sent=False)
with (O/'result.json').open('x') as f: json.dump(result,f,indent=2,allow_nan=False); f.write('\n')
print(json.dumps(dict(passed=True,original_code=pairs[0][0],audit=audit,output=str(O/'result.json'))))
