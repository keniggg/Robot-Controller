"""Verify closed experiment, protect diagnostics, update audit/docs/metadata."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import time

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=EXT/'ros_log/full_latest_contact_admission_20261013T011103Z'
R=EXT/'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
OUT=N/'attempt_2_seal'
def sha(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as f:
        while True:
            chunk=f.read(8*1024*1024)
            if not chunk:break
            value.update(chunk)
    return value.hexdigest()
read=lambda p:json.loads(Path(p).read_text())
assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
manifest=read(R/'manifest.json');outcome=read(R/'recorder_outcome.json')
reason='OBSERVATION_MEASUREMENT_FAILED: OBSERVATION_MEASUREMENT_ACK_TIMEOUT'
assert manifest['state']=='complete' and manifest['result']=='failure' and manifest['failure_reason']==reason
assert manifest['recorder_closed'] and outcome['bag_closed']
assert manifest['result_evidence']['runner_returncode']==6
assert len(manifest['result_evidence']['observations'])>=2
assert all(x['result']=='failure' for x in manifest['result_evidence']['observations'])
assert 'GRASP_RESULT success=False message='+reason in (R/'runner.log').read_text()
rows=[]
for item in manifest['files']:
    p=(R/item['path']).resolve()
    assert R in p.parents and p.is_file() and not (R/item['path']).is_symlink()
    size=p.stat().st_size;hash_value=sha(p)
    assert size==item['size_bytes'] and hash_value==item['sha256'],item['path']
    rows.append(dict(path=item['path'],size_bytes=size,sha256=hash_value,passed=True))
assert len(rows)==10
sidecar_path=N/'attempt_2_full_frames/result.json';sidecar=read(sidecar_path)
bag=Path(sidecar['path'])
assert sidecar['recording_closed'] and sidecar['returncode']==0
assert bag==N/'attempt_2_full_frames/full_frames.bag'
assert bag.stat().st_size==sidecar['size_bytes'] and sha(bag)==sidecar['sha256']
bags=[R/'keyframes.bag',bag]
assert not list(R.glob('*.bag.active')) and not list(bag.parent.glob('*.bag.active'))
handles=subprocess.run(['lsof','-F','pftan','--',*map(str,bags)],capture_output=True,text=True)
assert handles.returncode in (0,1),handles.stderr
assert handles.returncode==1 and not handles.stdout.strip(), 'sealed bags still open: '+handles.stdout
manual_path=N/'attempt_2_after_manual/result.json';manual=read(manual_path)
assert manual['manual_restored'] and manual['gui_own_publisher_acknowledged'] and manual['verified_inactive']
assert manual['generation']==3
selection=read(R/'authorization.json')['selection']
assert selection['generation']==3 and selection['stamp_ns']==1791860166743916511
deployment_path=N/'gen13_deployment.json';deployment=read(deployment_path)
sources=deployment['source_sha256'];assert len(sources)==890
assert all(sha(W/p)==h for p,h in sources.items())
protected_path=EXT/'maintenance/failed_record_space_guard_protected.json'
protected_before=protected_path.read_bytes();protected=json.loads(protected_before)
entries=protected['protected_paths'];assert len(entries)==105
new_paths=[str(R.relative_to(EXT)),str(bag.parent.relative_to(EXT))]
assert not any(row['path'] in new_paths for row in entries)
docs=[W/'docs/operations/2026-10-11-frozen-grasp-acceptance.md',
      EXT/'文档/2026-10-11-frozen-grasp-acceptance.md',N/'2026-10-11-frozen-grasp-acceptance.md']
doc_before=docs[0].read_bytes()
assert sha(docs[0])=='a5e6230d57ae09636809ed50a059eda93d72c133567f2b2c89f57dd2d997764a'
assert all(p.read_bytes()==doc_before for p in docs)
metadata_paths=[N/'current_progress.json',W/'.runtime/latest_full_ros_start.json']
metadata_before={str(p):p.read_bytes() for p in metadata_paths}
OUT.mkdir()
(OUT/'document_before.md').write_bytes(doc_before)
(OUT/'protected_before.json').write_bytes(protected_before)
for p in metadata_paths:(OUT/(p.stem+'_before.json')).write_bytes(metadata_before[str(p)])
def atomic(p,payload):
    temporary=p.with_name(p.name+'.attempt2_seal_tmp')
    with temporary.open('xb') as f:f.write(payload);f.flush();os.fsync(f.fileno())
    os.replace(temporary,p)
def write_json(p,data):atomic(p,(json.dumps(data,ensure_ascii=False,indent=2)+'\n').encode())
now=time.time_ns()
verification=dict(passed=True,recorded_ns=now,record=str(R),manifest_sha256=sha(R/'manifest.json'),
    files=rows,file_count=len(rows),recorder_closed=True,bag_closed=True,
    open_bag_check=dict(command=handles.args,returncode=handles.returncode,stdout=handles.stdout,
        stderr=handles.stderr,no_open_descriptors=True,no_active_bags=True),
    sidecar=dict(result_path=str(sidecar_path),result_sha256=sha(sidecar_path),path=str(bag),
        size_bytes=sidecar['size_bytes'],sha256=sidecar['sha256'],recording_closed=True),
    manual=dict(path=str(manual_path),sha256=sha(manual_path),generation=3,
        manual_restored=True,gui_own_publisher_acknowledged=True,verified_inactive=True),
    selection=selection,result='failure',failure_reason=reason,root_cause_confirmed=False,
    production_sources_unchanged=True,motion_commands_sent=False,temperature_queried=False)
write_json(OUT/'manifest_verification.json',verification)
assert protected_path.read_bytes()==protected_before,'protection concurrently changed'
protected['protected_paths']=entries+[
    dict(path=new_paths[0],reason='Current GEN14 observation measurement ACK-timeout investigation; sealed attempt 2 failure confirmed by task state and StartGrasp. Preserve all original recording, geometry, request/ACK and audit evidence.'),
    dict(path=new_paths[1],reason='Current GEN14 ACK-timeout investigation original full-frame RGB-D/TF/SDK sidecar for attempt 2; closed and hash-verified, preserve complete raw evidence.')]
assert protected['protected_paths'][:105]==entries
write_json(protected_path,protected)
fragment=f'''### 2026-10-12 部署后第2轮：观测测量ACK等待超时，失败已封口并继续定位

GEN13部署后，主代理以selection generation3／stamp1791860166743916511发起本次会话第2轮，记录为 `grasp_records/{R.name}`。任务终态与StartGrasp响应均报告 `{reason}`，`GRASP_RESULT success=False`；runner按普通失败以6退出，没有launcher错误。本段只记录已确认的ACK等待超时，不据通用错误推断目标丢失、硬件保护或具体哪一端的根因；后续GEN14继续调查原请求／确认与时序证据。此前GEN13离线回归和部署通过的结论保持，但本轮没有实物抓取成功证据。

`manifest.json` 于UTC2026-10-13T02:56:59.260501封口，state=complete、result=failure、recorder_closed=true；`recorder_outcome.json` 记录bag_closed=true、81组关键帧、149对观察RGB-D，stop_reason为`observed_task_became_inactive`。逐项实读校验manifest全部 **10份文件** 的大小与SHA256均匹配；keyframes.bag共241,679,070字节，SHA256 `6fe9df1bc047f86b57faad2ff97822ff1f93955f41401f819f7698070580274e`。核验回执为 `attempt_2_seal/manifest_verification.json`，同时保留manifest摘要及封口结果依据，不以单一退出码推断失败。

独立完整原始帧sidecar亦已由主代理停止并封口：`attempt_2_full_frames/result.json` 记录returncode=0、recording_closed=true；`full_frames.bag` 共4,216,897,925字节，实读SHA256 `46086ebd97453de78052ee41e35541c70069209f8687663d894ef278609559b9` 与回执一致。两份bag均无`.active`文件，封口审计时lsof未发现打开的文件描述符。此sidecar包含原RGB-D、TF、非温度SDK及任务／请求／ACK等证据，不用抽样图替代完整录制。

GUI自身手动回传及任务／动作空闲核验通过，证据 `attempt_2_after_manual/result.json` 绑定generation3；这是收尾时点的核验，不是后续自动启动授权或现场快照。890份已部署来源与GEN13统一部署逐项一致。`maintenance/failed_record_space_guard_protected.json` 由105项增至107项，新增本轮完整实验及配套sidecar两条保护，全部105条历史逐项保留；更新前文件、文档及进度元数据备份均保存在 `attempt_2_seal/`。没有删除录制、上传文件或更改local_only和温度约定。当前记录为sealed failure、active_record=null、automatic_grasp_running=false，继续定位GEN14；根因尚未确认，实物抓取仍未成功。
'''
fragment_path=OUT/'document_fragment.txt';fragment_path.write_text(fragment)
doc_after=doc_before+b'\n\n'+fragment.encode()
for p in docs:atomic(p,doc_after)
updates=dict(recorded_ns=now,updated_ns=now,active_work='diagnosing_gen14_observation_measurement_ack_timeout',
    automatic_grasp_started=True,automatic_grasp_running=False,start_reserved=False,
    active_record=None,last_record=str(R),current_record=str(R),last_result='failure',
    last_record_sealed=True,last_failure_reason=reason,current_generation=3,expected_generation=3,
    next_generation=4,current_selection=selection,selection=selection,
    attempt_scene=str(N/'attempt_2_scene'),manual_mode=True,gui_manual=True,gui_manual_restored=True,
    manual_evidence=str(manual_path),manual_restore_receipt=str(manual_path),
    verified_manual_state_evidence=str(manual_path),manual_state_is_live_snapshot=False,
    physical_grasp_verified=False,physical_grasp_success=False,
    root_cause_confirmed=False,closed_record_verification=str(OUT/'manifest_verification.json'),
    full_frame_sidecar=str(sidecar_path),full_frame_sidecar_bag=str(bag),full_frame_sidecar_sha256=sidecar['sha256'],
    next_action='Investigate GEN14 observation measurement ACK timeout from sealed original requests, ACKs and full-frame evidence; no physical success claim.',
    metadata_scope='Attempt 2 sealed failure and verified manual cleanup; root continues diagnosis. Live state must be sampled again before future motion.',
    metadata_update_audit=str(OUT/'result.json'))
for p in metadata_paths:
    assert p.read_bytes()==metadata_before[str(p)],'metadata concurrently changed: '+str(p)
    current=json.loads(metadata_before[str(p)]);current.update(updates);write_json(p,current)
assert all(p.read_bytes()==doc_after for p in docs)
assert read(protected_path)['protected_paths'][:105]==entries
assert all(sha(W/p)==h for p,h in sources.items())
receipt=dict(passed=True,recorded_ns=now,record=str(R),result='failure',failure_reason=reason,
    root_cause_confirmed=False,manifest_verification=dict(path=str(OUT/'manifest_verification.json'),sha256=sha(OUT/'manifest_verification.json')),
    protected_before_count=105,protected_after_count=107,protected_added=new_paths,
    protected_before_sha256=hashlib.sha256(protected_before).hexdigest(),protected_after_sha256=sha(protected_path),
    document_sha256=sha(docs[0]),document_paths=list(map(str,docs)),history_preserved=True,
    metadata_before_sha256={p:hashlib.sha256(b).hexdigest() for p,b in metadata_before.items()},
    metadata_after_sha256={str(p):sha(p) for p in metadata_paths},
    evidence_sha256={str(p):sha(p) for p in (R/'manifest.json',R/'recorder_outcome.json',R/'authorization.json',sidecar_path,manual_path,deployment_path)},
    source_count=890,production_sources_unchanged=True,physical_grasp_verified=False,
    motion_commands_sent=False,temperature_queried=False,no_recording_deleted=True,
    script_sha256=sha(__file__))
write_json(OUT/'result.json',receipt)
print(json.dumps({k:receipt[k] for k in ('passed','record','document_sha256','protected_after_count')},ensure_ascii=False))
