"""Install an explicitly reviewed staged patch after checking both baselines."""
import hashlib
import json
import shutil
import time
from pathlib import Path

work = Path(__file__).resolve().parents[1]
stage = work / '.runtime/gen5_supplement_handoff_20261012'
root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z')
out = root / 'verification/gen5_handoff_integration_install'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount()
base = json.loads((stage / 'base_sha256.json').read_text())
previous = json.loads((root / 'gen4_contact_runtime_deployment.json').read_text())
assert previous['deployment_passed'] is True
for p, h in dict(base, **previous['source_sha256']).items():
    assert sha(work / p) == h, ('actual baseline changed', p)
for p in base:
    assert (stage / p).is_file(), ('stage deleted source', p)
current = {str(p.relative_to(stage)): sha(p)
           for package in ('alicia_grasp_modes', 'alicia_flexible_grasp_supervisor')
           for p in (stage / 'src' / package).rglob('*')
           if p.is_file() and '__pycache__' not in p.parts
           and '.pytest_cache' not in p.parts and p.suffix != '.pyc'}
changed = {p: h for p, h in current.items() if base.get(p) != h}
production = {
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
    'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/supplement_measurement_admission.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observe_return_gateway.py',
}
assert production <= set(changed)
assert all(p in production or '/tests/test_' in p for p in changed)
assert all((work/p).exists() == (p in base) for p in changed)
out.mkdir(exist_ok=False)
receipt = dict(passed=False, recorded_ns=time.time_ns(), staged_changed_sha256=changed,
               before_sha256={p: base.get(p) for p in changed},
               previous_deployment=str(root/'gen4_contact_runtime_deployment.json'),
               motion_commands_sent=False, temperature_queried=False)
(out/'install_receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
for p in changed:
    if p in base:
        backup = out/'before'/p
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(work/p, backup)
        assert sha(backup) == base[p]
for p, h in changed.items():
    assert sha(stage/p) == h, ('stage changed during installation', p)
    target = work/p
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(stage/p, target)
    assert sha(target) == h
snapshot = dict(previous['source_sha256'], **current)
for p, h in snapshot.items():
    assert sha(work/p) == h, p
receipt.update(passed=True, source_sha256=snapshot,
               production_changed=sorted(production), runtime_reloaded=False)
(out/'install_receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
print(json.dumps(dict(passed=True, changed_paths=sorted(changed),
    source_count=len(snapshot), receipt=str(out/'install_receipt.json'))), flush=True)
