"""Install only the independently reviewed, immutable GEN7 patch files."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

ap = argparse.ArgumentParser()
ap.add_argument('--delivery', type=Path, required=True)
ap.add_argument('--review', type=Path, required=True)
args = ap.parse_args()
w = Path(__file__).resolve().parents[1]
s = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
delivery = json.loads(args.delivery.read_text())
review = json.loads(args.review.read_text())
assert review['passed'] is True
assert delivery['all_base_production_hashes_unchanged'] is True
assert delivery['semantic_review']['ast_equal_after_reverting_one_count_subscript'] is True
assert delivery['tests'][-1]['expected'] == 'pass'
assert delivery['tests'][-1]['counts'] == dict(tests=302, failures=0, errors=0, skipped=0)
assert review['delivery_sha256'] == sha(args.delivery)
assert all(sha(w / p) == h for p, h in review['operational_files_sha256'].items())
old = json.loads((s / 'verification/gen6_prewarm_release_manifest.json').read_text())
assert all(sha(w / p) == h for p, h in old['source_sha256'].items())
stage = Path(delivery['stage'])
candidate = {r['path']: r['candidate_sha256'] for r in delivery['files']}
baseline = {r['path']: r['base_sha256'] for r in delivery['files']}
assert candidate and set(candidate) == set(baseline)
assert len([p for p in candidate if '/tests/' not in p]) == 1
for relative, digest in candidate.items():
    path = Path(relative)
    assert not path.is_absolute() and '..' not in path.parts
    assert sha(stage / relative) == digest
    assert (sha(w / relative) if (w / relative).exists() else None) == baseline[relative]
out = s / 'verification/gen7_yield_install'
out.mkdir(exist_ok=False)
for relative in candidate:
    target = w / relative
    if target.exists():
        backup = out / 'before' / relative
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(target, backup)
        assert sha(backup) == baseline[relative]
for relative, digest in candidate.items():
    target = w / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(stage / relative, target)
    assert sha(target) == digest
source = dict(old['source_sha256'], **candidate)
for relative in (
    '.runtime/run_gen7_yield_integrated_20261012.py',
    '.runtime/read_gen7_yield_readiness_20261012.py',
    '.runtime/restore_manual_gen7_yield_deployment_20261012.py',
    '.runtime/install_gen7_yield_20261012.py',
    '.runtime/analyze_gen7_precheck_timing_20261012.py',
):
    source[relative] = sha(w / relative)
assert all(sha(w / p) == h for p, h in source.items())
result = dict(passed=True, installed_ns=time.time_ns(), delivery=str(args.delivery),
              delivery_sha256=sha(args.delivery), review=str(args.review), review_sha256=sha(args.review),
              changed_files_sha256=candidate, changed_files_baseline_sha256=baseline,
              source_sha256=source, robot_nodes_restarted=False,
              motion_commands_sent=False, temperature_queried=False)
(out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(dict(passed=True, changed_files=len(candidate), source_count=len(source),
                     result=str(out / 'result.json'))), flush=True)
