"""Bind a gateway-only reload to the unchanged compute/hardware baseline."""
import argparse
import hashlib
import json
from pathlib import Path
import time

ap = argparse.ArgumentParser()
for name in ('previous', 'gateway', 'manifest', 'output'):
    ap.add_argument('--'+name, type=Path, required=True)
a = ap.parse_args()
w = Path(__file__).resolve().parents[1]
mount = Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and mount in a.output.resolve().parents
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
previous, gateway, manifest = map(read, (a.previous, a.gateway, a.manifest))
assert previous['deployment_passed'] is gateway['deployment_passed'] is True
assert manifest['passed'] is manifest['reviewed'] is True
assert gateway['previous_runtime_baseline'] == str(a.previous)
assert gateway['source_sha256'] == manifest['source_sha256']
assert gateway['before'] == previous['preserved_after']['/motion_gateway']
prior_processes = dict(previous['preserved_after'], **previous['after'])
assert set(gateway['preserved_after']) == set(prior_processes)-{'/motion_gateway'}
assert all(gateway['preserved_before'][n] == row == prior_processes[n]
           for n, row in gateway['preserved_after'].items())
changed = {p for p, h in manifest['source_sha256'].items()
           if previous['source_sha256'].get(p) != h}
source = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_compensation_gateway.py'
assert source in changed
assert all(p == source or p.startswith('src/alicia_flexible_grasp_supervisor/tests/')
           for p in changed)
assert set(previous['source_sha256']) <= set(manifest['source_sha256'])
for p, h in manifest['source_sha256'].items():
    assert sha(w/p) == h, p
result = dict(previous)
result.update(source_sha256=manifest['source_sha256'], recorded_ns=time.time_ns(),
              deployment_kind='gateway_only', compute_reloaded=False,
              previous_deployment=str(a.previous),
              previous_deployment_sha256=sha(a.previous),
              source_manifest=str(a.manifest), source_manifest_sha256=sha(a.manifest),
              gateway_reload_evidence=str(a.gateway), gateway_reload_sha256=sha(a.gateway),
              preserved_after=dict(previous['preserved_after'], **{'/motion_gateway':gateway['after']}),
              motion_commands_sent=False, torque_commands_sent=False,
              temperature_queried=False, physical_grasp_verified=False)
a.output.parent.mkdir(parents=True, exist_ok=True)
with a.output.open('x') as stream:
    stream.write(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(deployment_passed=True, result=str(a.output),
                     sha256=sha(a.output), compute_unit_unchanged=result['unit'])))
