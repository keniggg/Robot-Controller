"""Append installed GEN13 independent verification while actual union runs."""
from pathlib import Path
import hashlib
import json
import os
import time

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=EXT/'ros_log/full_latest_contact_admission_20261013T011103Z'
OUT=N/'verification/gen13_identity_progress_document_update'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_text())
assert Path('/media/zhuyupei/SanStand').is_mount()
installed=N/'verification/gen13_identity_integrated_install/result.json'
installation=read(installed)
assert installation['passed'] and not installation['robot_nodes_restarted']
assert len(installation['source_sha256'])==890 and len(installation['test_scope'])==137
assert all(sha(W/p)==h for p,h in installation['source_sha256'].items())
relative='docs/operations/2026-10-11-frozen-grasp-acceptance.md'
assert relative not in installation['source_sha256']
paths=[W/relative,EXT/'文档'/Path(relative).name,N/Path(relative).name]
before=paths[0].read_bytes()
assert sha(paths[0])=='de6ec7cb69f5e0ea795da07d2fa989c81df4fb9f91838e5e2b0936188b349f1f'
assert all(p.read_bytes()==before for p in paths)
fragment=W/'.runtime/gen13_identity_progress_doc_fragment.txt'
assert fragment.read_text().splitlines()[0] not in before.decode()
assert not (N/'verification/gen13_actual_integrated/result.json').exists(), 'actual union finished: document status needs update'
final=N/'verification/gen13_station_appearance_focused_final_r5/result.json'
assert read(final)['counts']==dict(tests=487,errors=0,failures=0,skipped=0) and read(final)['passed']
replay=N/'verification/gen13_full_qualified_station_process_v5/validation_receipt.json'
assert read(replay)['passed'] and read(replay)['fresh_current_geometry_count']==4 and read(replay)['natural_fifo_expired_pending_count']==1
OUT.mkdir(exist_ok=False)
(OUT/'before.md').write_bytes(before)
after=before+b'\n\n'+fragment.read_bytes()
for p in paths:
 temporary=p.with_name(p.name+'.gen13_progress_tmp')
 with temporary.open('xb') as f:
  f.write(after);f.flush();os.fsync(f.fileno())
 os.replace(temporary,p)
assert all(p.read_bytes()==after for p in paths)
assert all(sha(W/p)==h for p,h in installation['source_sha256'].items())
evidence=[installed,final,replay,N/'verification/gen13_station_identity_delivery_seal/result.json',
 N/'verification/gen13_station_identity_delivery_seal/independent_review.json',
 N/'verification/gen13_station_identity_and_typed_independent_r9/result.json',
 N/'verification/gen13_support_guard_baseline/result.json',N/'verification/gen13_support_guard_expectation/result.json',
 N/'verification/gen13_control_flow_baseline/result.json',N/'verification/gen13_control_flow_candidate/result.json']
result=dict(passed=True,recorded_ns=time.time_ns(),document_sha256=sha(paths[0]),
 document_paths=list(map(str,paths)),before_sha256=hashlib.sha256(before).hexdigest(),
 fragment_sha256=sha(fragment),evidence_sha256={str(p):sha(p) for p in evidence},
 source_count=890,test_scope_count=137,production_sources_unchanged=True,history_preserved=True,
 gen12_runtime_reloaded=False,gen13_runtime_reloaded=False,gen13_sources_installed=True,actual_union_running=True,physical_grasp_verified=False,
 motion_commands_sent=False,temperature_queried=False,script_sha256=sha(__file__))
(OUT/'receipt.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ('passed','document_sha256','source_count','test_scope_count')},ensure_ascii=False))
