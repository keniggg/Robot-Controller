"""Install the tested staged patch and save the exact reviewed deployment inputs."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import time

work=Path(__file__).resolve().parents[1]
stage=work/'.runtime/initial_precommit_staging_20261009'
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=root/'verification/initial_preflight_combined_final'
tests=json.loads((out/'result.json').read_text())
review_path=root/'gen3_handoff_analysis/independent_preflight_review/review.json'
review=json.loads(review_path.read_text())
base_path=root/'verification/station_registration_combined/deployment_sources.json'
base=json.loads(base_path.read_text())
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
assert tests['passed'] and tests['source_unchanged']
assert tests['source_sha256']==review['source_sha256']
for p,d in base['source_sha256'].items():
    assert digest(work/p)==d, p
for p,d in tests['source_sha256'].items():
    assert digest(stage/p)==d, p
    ast.parse((stage/p).read_text())
for p in tests['source_sha256']:
    dest=work/p
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(str(stage/p),str(dest))
sources=dict(base['source_sha256'],**tests['source_sha256'])
for p in ['tools/record_grasp_keyframes_readonly.py', '.runtime/watch_station_compute_20261009.py',
          '.runtime/replay_initial_preflight_remote_gen3_20261009.py']:
    ast.parse((work/p).read_text()); sources[p]=digest(work/p)
for p,d in sources.items(): assert digest(work/p)==d,p
manifest=dict(passed=True,reviewed=True,source_sha256=sources,integrated_ns=time.time_ns(),
    base_manifest=str(base_path),base_manifest_sha256=digest(base_path),
    tests=str(out/'result.json'),tests_sha256=digest(out/'result.json'),
    unique_combined_tests=tests['unique_tests'],review=str(review_path),review_sha256=digest(review_path),
    recorded_replay=str(root/'gen3_handoff_analysis/initial_preflight_remote_task_recorded2.json'),
    physical_success_verified=False,
    limitations=review['accepted_limits'])
(out/'deployment_sources.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(integrated=True,sources=len(sources),tests=tests['unique_tests'],manifest=str(out/'deployment_sources.json')),ensure_ascii=False))
