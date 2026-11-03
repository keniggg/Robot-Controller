"""Seal verified GEN7 installation for the existing idle-only consumer loader."""
import hashlib
import json
from pathlib import Path
import time

w = Path(__file__).resolve().parents[1]
s = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
names = ['verification/gen7_yield_install/result.json',
         'verification/gen7_yield_integrated/result.json',
         'verification/gen7_yield_independent_review.json']
installed, tested, review = [json.loads((s / p).read_text()) for p in names]
assert all(r['passed'] is True for r in (installed, tested, review))
assert tested['source_unchanged'] and tested['source_sha256'] == installed['source_sha256']
assert sha(Path(installed['review'])) == installed['review_sha256']
assert sha(Path(installed['delivery'])) == installed['delivery_sha256'] == review['delivery_sha256']
source = installed['source_sha256']
assert all(sha(w / p) == h for p, h in source.items())
names += ['verification/gen7_yield_delivery/delivery_manifest.json',
          'verification/gen6_prewarm_release_manifest.json',
          'gen7_analysis/precheck_timing_independent_review.json']
result = dict(passed=True, reviewed=True, installed=True, recorded_ns=time.time_ns(),
    kind='completed_original_family_search_budget', source_sha256=source,
    evidence_sha256={str(s / p): sha(s / p) for p in names}, test_summary=tested['summary'],
    changed_files_sha256=installed['changed_files_sha256'],
    previous_robot_deployment=str(s / 'gen5_handoff_runtime_deployment.json'),
    previous_robot_deployment_sha256=sha(s / 'gen5_handoff_runtime_deployment.json'),
    source_unchanged=True, physical_grasp_verified=False,
    limits_unchanged=['30-second child ROS/monotonic budget', 'original near-field phase deadline',
        'exactly two distinct completed-family certificate', 'bound real collision receipt',
        'full collision curves and sampling', 'maximum two observations', 'automatic lift disabled'],
    limitations=['Offline synthetic receipts beyond the recorded GEN7 prefix are not live replay',
        'Does not speed up each precheck or guarantee a second completed family within the original deadline',
        'New single-round confirmation is required for the next live trial'],
    motion_commands_sent=False, temperature_queried=False)
target = s / 'verification/gen7_yield_release_manifest.json'
with target.open('x') as f:
    json.dump(result, f, indent=2)
    f.write('\n')
print(json.dumps(dict(passed=True, source_count=len(source), test_summary=tested['summary'],
                     manifest=str(target), sha256=sha(target))), flush=True)
