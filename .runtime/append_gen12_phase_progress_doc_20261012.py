"""Append installed-but-not-reloaded GEN12 and measured GEN13 diagnosis."""
from pathlib import Path
import hashlib
import json
import os
import time

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=EXT/'ros_log/full_latest_contact_admission_20261013T011103Z'
OUT=N/'verification/gen12_phase_progress_document_update'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_text())
assert Path('/media/zhuyupei/SanStand').is_mount()
installed=N/'verification/gen12_phase_integrated_install/result.json'
installation=read(installed)
assert installation['passed'] and not installation['robot_nodes_restarted']
assert len(installation['source_sha256'])==885 and len(installation['test_scope'])==121
assert all(sha(W/p)==h for p,h in installation['source_sha256'].items())
relative='docs/operations/2026-10-11-frozen-grasp-acceptance.md'
assert relative not in installation['source_sha256']
paths=[W/relative,EXT/'文档'/Path(relative).name,N/Path(relative).name]
before=paths[0].read_bytes()
assert sha(paths[0])=='7397727711ce95605bfc3aaf695deae55ffd96f4fe7c57072204e8186f0d9f0b'
assert all(p.read_bytes()==before for p in paths)
fragment=W/'.runtime/gen12_phase_and_identity_progress_doc_fragment.txt'
assert fragment.read_text().splitlines()[0] not in before.decode()
sidecar=N/'full_frame_sidecar_readonly_check'
assert read(sidecar/'validation.json')['complete_original_rgbd_pairs']==41
assert read(sidecar/'validation.json')['passed']
assert sha(sidecar/'full_frames.bag')==read(sidecar/'result.json')['sha256']
diagnosis=N/'verification/attempt1_qualified_and_origin_identity_candidates/result.json'
rows=read(diagnosis)['rows']
for stamp in (1791854972438854694,1791854972907250881):
 row=next(r for r in rows if r['source_ns']==stamp)
 fixed=next(r for r in row['narrow_origin_preserving_and_frozen_qualified_diagnostics'] if r['name']=='all_frozen_qualified')
 assert fixed['passed'] and fixed['station_reference_ns']==1791854971572257757
 assert all(item['iou']>=.85 for item in fixed['overlap_rows'])
OUT.mkdir(exist_ok=False)
(OUT/'before.md').write_bytes(before)
after=before+b'\n\n'+fragment.read_bytes()
for p in paths:
 temporary=p.with_name(p.name+'.gen12_progress_tmp')
 with temporary.open('xb') as f:
  f.write(after);f.flush();os.fsync(f.fileno())
 os.replace(temporary,p)
assert all(p.read_bytes()==after for p in paths)
assert all(sha(W/p)==h for p,h in installation['source_sha256'].items())
evidence=[installed,N/'verification/gen12_phase_delivery_seal/result.json',
 N/'verification/gen12_phase_owner_focused_r5/result.json',
 N/'verification/gen12_registered_bad_negative/validation_receipt.json',
 diagnosis,N/'verification/attempt1_first_phase_with_qualified_source/fixture.json',
 sidecar/'result.json',sidecar/'validation.json']
result=dict(passed=True,recorded_ns=time.time_ns(),document_sha256=sha(paths[0]),
 document_paths=list(map(str,paths)),before_sha256=hashlib.sha256(before).hexdigest(),
 fragment_sha256=sha(fragment),evidence_sha256={str(p):sha(p) for p in evidence},
 source_count=885,test_scope_count=121,production_sources_unchanged=True,history_preserved=True,
 gen12_runtime_reloaded=False,gen13_complete=False,physical_grasp_verified=False,
 motion_commands_sent=False,temperature_queried=False,script_sha256=sha(__file__))
(OUT/'receipt.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:result[k] for k in ('passed','document_sha256','source_count','test_scope_count')},ensure_ascii=False))
