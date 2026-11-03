"""Install only the immutable, reviewed station snapshot admission repair."""
from pathlib import Path
import ast,hashlib,json,shutil,tarfile,time
work=Path(__file__).resolve().parents[1]
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=root/'verification/gen8_station_admission_combined'
def read(p):return json.loads(p.read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
test_path=out/'result.json';test=read(test_path)
review_path=root/'verification/gen8_station_admission_independent_review.json';review=read(review_path)
base_path=root/'verification/gen7_station_handoff_combined/deployment_sources.json';base=read(base_path)
assert test['passed'] is True and test['source_unchanged'] is True
assert review['disposition']=='no_blocking_findings_in_frozen_patch'
stage=Path(test['stage']);assert work/'.runtime' in stage.parents
sources=test['source_sha256'];assert sources
reviewed={row['path']:row['sha256'] for row in review['files']}
assert sources==reviewed
for p,h in base['source_sha256'].items():assert digest(work/p)==h,p
changes=read(out/'deployment_changes.json')
assert {row['path'] for row in changes['files']}==set(sources)
for row in changes['files']:
    p=row['path'];current=work/p
    assert (digest(current) if current.exists() else None)==row['original_live_sha256'],p
    assert digest(stage/p)==row['staged_sha256']==sources[p],p
    if p.endswith('.py'):ast.parse((stage/p).read_text())
backup=work/'.runtime/source_before_gen8_station_admission_20261010.tar';assert not backup.exists()
with tarfile.open(str(backup),'w') as archive:
    for p in sorted(sources):
        if (work/p).exists():archive.add(str(work/p),arcname=p)
for p in sources:
    (work/p).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(str(stage/p),str(work/p))
merged=dict(base['source_sha256'],**sources)
for p,h in merged.items():assert digest(work/p)==h,p
result=dict(passed=True,reviewed=True,source_sha256=merged,integrated_ns=time.time_ns(),
    base_manifest=str(base_path),base_manifest_sha256=digest(base_path),
    tests=str(test_path),tests_sha256=digest(test_path),unique_combined_tests=test['unique_tests'],
    review=str(review_path),review_sha256=digest(review_path),
    source_backup=str(backup),source_backup_sha256=digest(backup),physical_success_verified=False,
    limitations=['Audited station history may inform a new snapshot; withdrawn old tickets never regain authority.',
      'Full physical arrival, contact path and object holding remain unverified until live execution.'])
(out/'deployment_sources.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(integrated=True,changed_sources=len(sources),manifest_sources=len(merged),tests=test['unique_tests'],manifest=str(out/'deployment_sources.json')),ensure_ascii=False))
