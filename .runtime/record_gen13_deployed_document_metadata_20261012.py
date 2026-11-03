"""Record sealed deployment in docs/metadata; no ROS or control operations."""
from pathlib import Path
import hashlib
import json
import os
import time

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=EXT/'ros_log/full_latest_contact_admission_20261013T011103Z'
OUT=N/'verification/gen13_deployed_document_metadata_update'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_text())
assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
deployment=N/'gen13_deployment.json'; reload=N/'gen13_compute_reload/result.json'
release=N/'verification/gen13_station_identity_release_manifest.json'
tests=N/'verification/gen13_actual_integrated/result.json'
installed=N/'verification/gen13_identity_integrated_install/result.json'
manual=N/'gen13_before_deployment_manual/result.json'
data=read(deployment); test=read(tests); installation=read(installed)
assert sha(deployment)=='c5b5d66eb0e3c439fa1ea491baf227f4c99e6daa68b20e9352ce0a0a09178f9a'
assert sha(release)==data['source_manifest_sha256']=='eb3b44a65fd997896097a173688af29a69df012f3097de8d7e0329b4ac3730d6'
assert sha(reload)==data['reload_sha256']
assert all(data[k] for k in ('deployment_passed','whole_patch_deployed','compute_reloaded','hardware_processes_preserved'))
assert data['preserved_before']==data['preserved_after'] and len(data['preserved_after'])==8
assert len(data['after'])==3 and all(data['before'][k]!=v for k,v in data['after'].items())
assert test['passed'] and test['source_unchanged']
assert test['summary']==dict(tests=3621,failures=0,errors=0,skipped=0)
sources=data['source_sha256']
assert sources==read(release)['source_sha256']==test['source_sha256']==installation['source_sha256']
assert len(sources)==890 and len(installation['test_scope'])==137
assert all(sha(W/p)==h for p,h in sources.items())
assert read(manual)['manual_restored'] and read(manual)['verified_inactive']
relative='docs/operations/2026-10-11-frozen-grasp-acceptance.md'
paths=[W/relative,EXT/'文档'/Path(relative).name,N/Path(relative).name]
before=paths[0].read_bytes()
assert sha(paths[0])=='34f663e208cfc9d81b6cc505ff9d5fcdc139c0333108980b4cdf24c7f738b3fa'
assert all(p.read_bytes()==before for p in paths)
fragment=W/'.runtime/gen13_deployed_doc_fragment.txt'
assert fragment.read_text().splitlines()[0] not in before.decode()
progress_paths=[N/'current_progress.json',W/'.runtime/latest_full_ros_start.json']
assert all(str(p.relative_to(W)) not in sources for p in paths if W in p.parents)
assert '.runtime/latest_full_ros_start.json' not in sources
snapshot={str(p):p.read_bytes() for p in progress_paths}
OUT.mkdir()
(OUT/'before.md').write_bytes(before)
for p in progress_paths: (OUT/(p.stem+'_before.json')).write_bytes(snapshot[str(p)])

def atomic(p,payload):
    temporary=p.with_name(p.name+'.gen13_deployed_tmp')
    with temporary.open('xb') as f:
        f.write(payload);f.flush();os.fsync(f.fileno())
    os.replace(temporary,p)

after=before+b'\n\n'+fragment.read_bytes()
for p in paths: atomic(p,after)
now=time.time_ns()
updates=dict(recorded_ns=now,updated_ns=now,
    active_work='gen13_deployed_followup_grasp_managed_by_root',
    source_manifest=str(release),source_manifest_sha256=sha(release),
    installed_repair=str(installed),current_regression=str(tests.parent),
    current_test_result=str(tests),current_test_result_sha256=sha(tests),
    current_regression_summary=test['summary'],current_test_scope_files=137,
    current_source_count=890,source_count=890,disk_source_count=890,
    deployment=str(deployment),current_deployment=str(deployment),latest_deployment=str(deployment),
    deployment_sha256=sha(deployment),deployment_passed=True,
    runtime_receipt=str(deployment),runtime_reload_pending=False,
    installed_but_not_loaded_patch=None,pending_combined_test_scope_files=0,
    whole_patch_deployed=True,compute_reloaded=True,hardware_processes_preserved=True,
    operational_source_sha256=sources,
    deployed_node_identities={**data['preserved_after'],**data['after']},
    compute_unit=data['unit'],reload_evidence=str(reload),reload_evidence_sha256=sha(reload),
    verified_manual_state_evidence=str(manual),manual_evidence=str(manual),
    manual_state_is_live_snapshot=False,
    metadata_scope='Sealed deployment update only; historical manual and grasp fields are not new live observations. Root owns attempt 2 and subsequent grasp state.',
    physical_grasp_verified=False,
    next_action='Root continues the next recorded grasp under current selection, scene and actual feedback; record its outcome separately without inferring success from regression or deployment.',
    metadata_update_audit=str(OUT/'receipt.json'))
for p in progress_paths:
    assert p.read_bytes()==snapshot[str(p)], 'concurrent metadata update: '+str(p)
    current=json.loads(snapshot[str(p)])
    current.update(updates)
    atomic(p,(json.dumps(current,ensure_ascii=False,indent=2)+'\n').encode())
assert all(p.read_bytes()==after for p in paths)
assert all(sha(W/p)==h for p,h in sources.items())
evidence=[deployment,reload,release,tests,installed,manual]
receipt=dict(passed=True,recorded_ns=now,history_preserved=True,source_count=890,test_scope_count=137,
    summary=test['summary'],document_sha256=sha(paths[0]),document_paths=list(map(str,paths)),
    before_document_sha256=hashlib.sha256(before).hexdigest(),fragment_sha256=sha(fragment),
    metadata_before_sha256={p:hashlib.sha256(v).hexdigest() for p,v in snapshot.items()},
    metadata_after_sha256={str(p):sha(p) for p in progress_paths},
    evidence_sha256={str(p):sha(p) for p in evidence},
    production_sources_unchanged=True,live_actions_performed=False,
    physical_grasp_verified=False,motion_commands_sent=False,temperature_queried=False,
    script_sha256=sha(__file__))
(OUT/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:receipt[k] for k in ('passed','document_sha256','metadata_after_sha256')},ensure_ascii=False))
