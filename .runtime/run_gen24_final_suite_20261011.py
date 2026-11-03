"""Run the integrated fragment regression suite; evidence stays on SanStand."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


ap = argparse.ArgumentParser()
ap.add_argument('--delivery', type=Path, required=True)
ap.add_argument('--scope', nargs='+', required=True)
ap.add_argument('--output-prefix', type=Path, required=True)
ap.add_argument('--previous-deployment', type=Path)
a = ap.parse_args()
work = Path(__file__).resolve().parents[1]
root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
assert root in a.output_prefix.resolve().parents
delivery = json.loads(a.delivery.read_text())
baseline = json.loads((a.previous_deployment or root/'full_stack_deployment.json').read_text())
names = set(baseline['source_sha256']) | {p.split('::')[0] for p in a.scope}
for key in ('source_sha256', 'files', 'test_files'):
    names.update(delivery.get(key, {}))
def sources():
    return {p: hashlib.sha256((work/p).read_bytes()).hexdigest() for p in sorted(names)}
before = sources()
xml = a.output_prefix.with_suffix('.xml')
log = a.output_prefix.with_suffix('.log')
manifest_path = a.output_prefix.with_name(a.output_prefix.name+'_manifest.json')
assert not any(p.exists() for p in (xml, log, manifest_path))
a.output_prefix.parent.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
env['PYTHONPATH'] = ':'.join([str(work/'src/alicia_grasp_modes/src'), str(work/'src/alicia_flexible_grasp_supervisor/tests'), env.get('PYTHONPATH', '')])
cmd = ['/usr/bin/python3', '-m', 'pytest', '-q', *a.scope, '--junitxml='+str(xml), '-p', 'no:cacheprovider']
start = time.monotonic()
with log.open('x') as handle:
    result = subprocess.run(cmd, cwd=work, env=env, stdout=handle, stderr=subprocess.STDOUT)
after = sources()
manifest = dict(command=cmd, test_scope=a.scope, source_sha256=before,
    source_after_sha256=after, source_unchanged=before == after,
    returncode=result.returncode, elapsed_sec=time.monotonic()-start)
manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
print(json.dumps(dict(manifest=str(manifest_path), returncode=result.returncode,
    source_unchanged=before == after, elapsed_sec=manifest['elapsed_sec'])), flush=True)
print('\n'.join(log.read_text().splitlines()[-8:]), flush=True)
raise SystemExit(result.returncode if before == after else 2)
