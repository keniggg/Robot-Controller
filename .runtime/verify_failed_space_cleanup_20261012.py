"""Verify exact deletion receipts, retained metadata, and protected recordings."""
import argparse
import hashlib
import json
from pathlib import Path
import time

root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
ap=argparse.ArgumentParser()
ap.add_argument('--receipt',type=Path,required=True)
a=ap.parse_args()
assert Path('/media/zhuyupei/SanStand').is_mount() and root in a.receipt.resolve().parents
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
receipt=json.loads(a.receipt.read_text())
execution=json.loads((a.receipt.parent/'execution_source_binding.json').read_text())
assert sha(execution['path'])==execution['sha256']==receipt['source_sha256']
assert receipt['state'] in ('complete','eligible_exhausted')
assert all(i['state']=='deleted' and i['retained_hashes_unchanged'] for i in receipt['files'])
assert receipt['skipped']==[]
retained={}
for item in receipt['files']:
    assert not (root/item['path']).exists(),item['path']
    assert not (root/item['quarantine']).exists(),item['quarantine']
    for name,value in item['retained_sha256'].items():
        assert sha(root/name)==value,name
        retained[name]=value
    for proof in item['evidence']:
        assert sha(root/proof['path'])==proof['sha256'],proof['path']
before=json.loads((root/'maintenance/failed_record_space_guard_protected_before_20261012.json').read_text())
for name,identity in before['protected_raw_identity'].items():
    stat=(root/name).stat()
    current=dict(device=stat.st_dev,inode=stat.st_ino,size_bytes=stat.st_size,mtime_ns=stat.st_mtime_ns)
    assert current==identity,name
for path,value in before['storage_config_sha256'].items():assert sha(path)==value,path
work=Path(__file__).resolve().parents[1]
deployment=json.loads((root/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen5_handoff_runtime_deployment.json').read_text())
for path,value in deployment['source_sha256'].items():assert sha(work/path)==value,path
result=dict(passed=True,verified_ns=time.time_ns(),receipt=str(a.receipt),receipt_sha256=sha(a.receipt),
    state=receipt['state'],deleted_file_count=len(receipt['files']),deleted_bytes=receipt['deleted_bytes'],
    deleted_allocated_bytes=receipt['deleted_allocated_bytes'],final_free_bytes=receipt['final_free_bytes'],
    retained_metadata_count=len(retained),all_retained_metadata_sha256_unchanged=True,
    protected_raw_count=len(before['protected_raw_identity']),all_protected_raw_identities_unchanged=True,
    storage_config_sha256_unchanged=True,robot_deployment_source_count=len(deployment['source_sha256']),
    robot_deployment_sources_unchanged=True,new_grasp_started=False,temperature_queried=False)
(a.receipt.parent/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
