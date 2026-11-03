"""Join exact reload identities, distinguishing the intermediate mixed runtime."""
import argparse
import hashlib
import json
from pathlib import Path
import time

ap = argparse.ArgumentParser()
ap.add_argument('--kind',choices=('gateway','compute'),required=True)
for key in ('previous','reload','manifest','output'):
    ap.add_argument('--'+key,type=Path,required=True)
a=ap.parse_args()
work=Path(__file__).resolve().parents[1]
mount=Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and mount in a.output.resolve().parents
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous,reload,manifest=map(read,(a.previous,a.reload,a.manifest))
assert previous['deployment_passed'] is reload['deployment_passed'] is True
assert manifest['passed'] is manifest['reviewed'] is True
assert reload['source_sha256']==manifest['source_sha256']
assert set(previous['source_sha256'])<=set(manifest['source_sha256'])
for p,h in manifest['source_sha256'].items():
    assert sha(work/p)==h,p
result=dict(previous)
for key in ('before','preserved_before','command','reset_ack','old_parent_pid',
            'stopped_unit','operational_change'):
    result.pop(key,None)
result.update(source_sha256=manifest['source_sha256'],recorded_ns=time.time_ns(),
    previous_deployment=str(a.previous),previous_deployment_sha256=sha(a.previous),
    source_manifest=str(a.manifest),source_manifest_sha256=sha(a.manifest),
    motion_commands_sent=False,temperature_queried=False,torque_commands_sent=False,
    physical_grasp_verified=False,new_selection_required_before_automatic_grasp=True)
if a.kind=='gateway':
    assert reload['previous_runtime_baseline']==str(a.previous)
    assert reload['before']==previous['preserved_after']['/motion_gateway']
    prior=dict(previous['preserved_after'],**previous['after'])
    assert set(reload['preserved_after'])==set(prior)-{'/motion_gateway'}
    assert all(reload['preserved_before'][n]==row==prior[n]
               for n,row in reload['preserved_after'].items())
    result.update(deployment_kind='gateway_loaded_compute_reload_pending',
        whole_patch_deployed=False,compute_reloaded=False,
        gateway_reload_evidence=str(a.reload),gateway_reload_sha256=sha(a.reload),
        preserved_after=dict(previous['preserved_after'],**{'/motion_gateway':reload['after']}))
else:
    assert previous['whole_patch_deployed'] is False
    assert reload['previous_deployment']==str(a.previous)
    assert reload['before']==previous['after']
    assert reload['preserved_before']==reload['preserved_after']==previous['preserved_after']
    assert set(reload['after'])==set(previous['after'])
    result.update(deployment_kind='gateway_and_dedicated_consumers',whole_patch_deployed=True,
        compute_reloaded=True,compute_reload_evidence=str(a.reload),compute_reload_sha256=sha(a.reload),
        unit=reload['unit'],after=reload['after'],preserved_after=reload['preserved_after'])
    for key in ('before','preserved_before','command','reset_ack','old_parent_pid','stopped_unit'):
        if key in reload:
            result[key]=reload[key]
with a.output.open('x') as stream:
    stream.write(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(deployment_passed=True,whole_patch_deployed=result['whole_patch_deployed'],
    result=str(a.output),sha256=sha(a.output))),flush=True)
