"""Bind completed compute and gateway reload receipts; never commands ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser()
    for name in ('compute', 'gateway', 'manifest', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    work = Path(__file__).resolve().parents[1]
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    read = lambda path: json.loads(path.read_text())
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    compute, gateway, manifest = map(read, (args.compute, args.gateway, args.manifest))
    assert compute['deployment_passed'] is gateway['deployment_passed'] is True
    assert manifest['passed'] is manifest['reviewed'] is True
    assert gateway['previous_runtime_baseline'] == str(args.compute)
    assert compute['source_sha256'] == gateway['source_sha256'] == manifest['source_sha256']
    assert gateway['before'] == compute['preserved_after']['/motion_gateway']
    old = dict(compute['preserved_after'], **compute['after'])
    assert set(gateway['preserved_after']) == set(old) - {'/motion_gateway'}
    for name, row in gateway['preserved_after'].items():
        assert row == old[name] == gateway['preserved_before'][name], name
    for name, digest in manifest['source_sha256'].items():
        assert sha(work/name) == digest, name
    result = dict(compute)
    result['preserved_after'] = dict(compute['preserved_after'])
    result['preserved_after']['/motion_gateway'] = gateway['after']
    result.update(combined_deployment=True, recorded_ns=time.time_ns(),
                  final_manifest=str(args.manifest), final_manifest_sha256=sha(args.manifest),
                  compute_deployment=dict(path=str(args.compute), sha256=sha(args.compute)),
                  gateway_deployment=dict(path=str(args.gateway), sha256=sha(args.gateway)),
                  motion_commands_sent=False, temperature_queried=False,
                  torque_disable_commands_sent=False,
                  physical_grasp_verified=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as out:
        out.write(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(deployment_passed=True, result=str(args.output),
                         sha256=sha(args.output), unit=result['unit']), ensure_ascii=False))


if __name__ == '__main__':
    main()
