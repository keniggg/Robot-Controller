"""Install exactly the reviewed and combined-tested contact changes while idle."""
from pathlib import Path
import ast, hashlib, json, shutil, tarfile, time
work=Path(__file__).resolve().parents[1]
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=root/'verification/gen5_contact_combined'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
test_path=out/'result.json'; tests=read(test_path)
assert tests['passed'] is True and tests['source_unchanged'] is True
stage=Path(tests['stage']); assert work/'.runtime' in stage.parents
base_path=root/'verification/initial_preflight_combined_final/deployment_sources.json'
base=read(base_path)
for p,h in base['source_sha256'].items():assert digest(work/p)==h,p
sources=tests['source_sha256']; assert len(sources)==8
changes=read(out/'deployment_changes.json')
assert len(changes['files'])==8
for row in changes['files']:
    current=work/row['path']
    actual=digest(current) if current.exists() else None
    assert actual==row['original_live_sha256'],row['path']
    assert sources[row['path']]==row['staged_sha256'],row['path']
for p,h in sources.items():
    assert digest(stage/p)==h,p
    if p.endswith('.py'):ast.parse((stage/p).read_text())
sdk_review_path=root/'verification/contact_sdk_wait_independent_review.json'
yaw_review_path=root/'verification/gen5_contact_yaw_independent_review_v2.json'
sdk_review=read(sdk_review_path); yaw_review=read(yaw_review_path)
# Explicit reviewed source identities, including the recorded endpoint fixture.
yaw=read(work/'.runtime/gen5_contact_yaw_staging_20261010/delivery.json')
for row in yaw['files']:assert sources[row['path']]==row['sha256'],row['path']
assert sources['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py']=='ce8982f83946f14b7e3c26fc915c2896aee056f1b138a10dae42f59a0b0b1073'
assert sdk_review['disposition']=='no_blocking_findings_in_final_staged_patch'
assert yaw_review['disposition']=='no_blocking_findings_in_frozen_patch'
for row in yaw_review['files']:assert sources[row['path']]==row['sha256']
backup=work/'.runtime/source_before_gen5_contact_repairs_20261010.tar'
assert not backup.exists()
with tarfile.open(str(backup),'w') as archive:
    for p in sorted(sources):
        if (work/p).exists():archive.add(str(work/p),arcname=p)
for p in sources:
    (work/p).parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(str(stage/p),str(work/p))
merged=dict(base['source_sha256'],**sources)
for p,h in merged.items():assert digest(work/p)==h,p
manifest=dict(passed=True,reviewed=True,source_sha256=merged,integrated_ns=time.time_ns(),
    base_manifest=str(base_path),base_manifest_sha256=digest(base_path),
    tests=str(test_path),tests_sha256=digest(test_path),unique_combined_tests=tests['unique_tests'],
    sdk_review=str(sdk_review_path),sdk_review_sha256=digest(sdk_review_path),
    yaw_review=str(yaw_review_path),yaw_review_sha256=digest(yaw_review_path),
    source_backup=str(backup),source_backup_sha256=digest(backup),physical_success_verified=False,
    limitations=['New yaw endpoints and evidence passed offline; IK and full contact path remain original live checks.',
      'SDK retry acceptance deadline is bounded; synchronous ROS ownership RPC is not forcibly interrupted.',
      'Physical user-measured 40mm width is independent reference; runtime geometry is not overridden.'])
(out/'deployment_sources.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(integrated=True,changed_sources=len(sources),manifest_sources=len(merged),tests=tests['unique_tests'],manifest=str(out/'deployment_sources.json')),ensure_ascii=False))
