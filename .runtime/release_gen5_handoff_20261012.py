"""Bind required passing regression, broad baseline comparison, and review."""
import hashlib,json,re,time
from pathlib import Path

work=Path(__file__).resolve().parents[1]
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
v=root/'verification'
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=dict(required=v/'gen5_handoff_integrated_required_r2/result.json',
    broad=v/'gen5_handoff_integrated/result.json',
    baseline=v/'gen5_handoff_baseline_comparison/result.json',
    review=v/'gen5_handoff_independent_review/result.json',
    task=v/'gen5_handoff_task_stage/result.json',
    producer=v/'gen5_handoff_producer_stage/producer_stage_receipt.json',
    remote=v/'gen5_handoff_remote_stage/result.json',
    install=v/'gen5_handoff_integration_install/install_receipt.json')
d={k:read(p) for k,p in paths.items()}
assert d['required']['passed'] is True and d['required']['source_unchanged'] is True
assert all(d['required']['summary'][k]==0 for k in ('failures','errors','skipped'))
assert d['broad']['source_unchanged'] is True
assert d['baseline']['source_unchanged'] is True
assert d['baseline']['matching_failure_ids'] is True and d['baseline']['new_failure_ids']==[]
normalize=lambda value:re.sub(r'0x[0-9a-fA-F]+','0xADDRESS',value)
assert all(normalize(row['message'])==normalize(d['baseline']['baseline_failures'][name]['message'])
           for name,row in d['baseline']['actual_failures'].items())
assert all(d[k]['summary'][n]==0 for k in ('baseline','broad') for n in ('errors','skipped'))
assert d['review']['passed'] is d['review']['reviewed'] is True
assert all(d[k]['passed'] is True for k in ('task','producer','remote','install'))
snapshot=d['required']['source_sha256']
assert snapshot==d['broad']['source_sha256']==d['install']['source_sha256']
assert d['review']['source_sha256']==d['install']['staged_changed_sha256']
for owner in ('review','task','producer'):
    assert all(snapshot[p]==h for p,h in d[owner]['source_sha256'].items())
for p,h in snapshot.items():assert sha(work/p)==h,p
operational={}
for filename in ('integrate_gen5_supplement_handoff_20261012.py',
    'run_gen5_supplement_handoff_regression_20261012.py',
    'read_idle_handoff_deployment_20261012.py','combine_gen5_handoff_deployment_20261012.py',
    'check_gen5_handoff_baseline_regression_20261012.py','release_gen5_handoff_20261012.py'):
    p=work/'.runtime'/filename
    compile(p.read_text(),str(p),'exec')
    operational[str(p.relative_to(work))]=sha(p)
manifest=dict(passed=True,reviewed=True,recorded_ns=time.time_ns(),
    validation_kind='required_scope_passed_and_no_new_broad_suite_failures',
    source_sha256=dict(snapshot,**operational),tested_source_sha256=snapshot,
    operational_source_sha256=operational,operational_validation='syntax_and_read_only_review',
    changed_source_sha256=d['install']['staged_changed_sha256'],
    evidence={k:dict(path=str(p),sha256=sha(p)) for k,p in paths.items()},
    required_summary=d['required']['summary'],broad_summary=d['broad']['summary'],
    baseline_summary=d['baseline']['summary'],broad_regression_all_passed=False,
    baseline_failure_messages_match_ignoring_process_addresses=True,
    known_baseline_failure_ids=sorted(d['baseline']['baseline_failures']),
    new_broad_failures=[],runtime_deployed=False,physical_grasp_verified=False,
    motion_commands_sent=False,temperature_queried=False,
    limits_unchanged=['source freshness 0.8 seconds','original frame completion deadline',
                      'contact compute deadline 60 seconds','solver iterations','collision sampling'],
    limitations=['Component and combined offline evidence are not a full live multi-node grasp.',
                 'The broad suite retains existing failures reproduced on the exact pre-patch tree.',
                 'Previous single-grasp authorization was consumed by GEN5; no new motion authorized.'])
out=v/'gen5_handoff_release_manifest.json'
with out.open('x') as f:f.write(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(passed=True,reviewed=True,manifest=str(out),
    source_count=len(manifest['source_sha256']),required_summary=manifest['required_summary'],
    broad_summary=manifest['broad_summary'],new_broad_failures=[])),flush=True)
