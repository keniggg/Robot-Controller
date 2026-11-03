"""Bind a compute-only reload to the prior complete deployment, without ROS writes."""
import argparse
import hashlib
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser()
    for key in ('previous', 'reload', 'manifest', 'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    work = Path(__file__).resolve().parents[1]
    assert mount.is_mount() and mount in args.output.resolve().parents
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    prior, reload, manifest = (json.loads(p.read_text())
                               for p in (args.previous, args.reload, args.manifest))
    assert prior['deployment_passed'] and prior['whole_patch_deployed']
    assert reload['deployment_passed'] and manifest['passed'] and manifest['reviewed']
    assert manifest['installed']
    compute = {'/grasp_task_node', '/remote_grasp6d_node', '/unknown_tabletop_perception'}
    hardware = {'/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
                '/motion_gateway', '/supervisor_camera', '/compliant_gripper_controller',
                '/grasp_mode_router', '/alicia_supervisor_gui'}
    assert set(prior['after']).isdisjoint(prior['preserved_after'])
    old = dict(prior['after'], **prior['preserved_after'])
    assert set(old) == compute | hardware
    assert set(reload['before']) == set(reload['after']) == compute
    assert reload['before'] == {key: old[key] for key in compute}
    assert reload['preserved_before'] == reload['preserved_after'] == {
        key: old[key] for key in hardware}
    process_key = lambda row: (row['pid'], row['start_ticks'])
    assert all(process_key(reload['before'][key]) != process_key(reload['after'][key])
               for key in compute)
    assert len({process_key(row) for row in
                dict(reload['after'], **reload['preserved_after']).values()}) == 11
    assert reload['source_sha256'] == manifest['source_sha256']
    for relative, digest in manifest['source_sha256'].items():
        assert sha(work / relative) == digest, relative
    for name, identity in dict(reload['after'], **reload['preserved_after']).items():
        stat = Path('/proc/%s/stat' % identity['pid']).read_text().rsplit(')', 1)[1].split()
        assert stat[0] != 'Z' and stat[19] == identity['start_ticks'], name
    result = dict(reload, whole_patch_deployed=True, compute_reloaded=True,
        hardware_processes_preserved=True, source_manifest=str(args.manifest),
        source_manifest_sha256=sha(args.manifest), reload_evidence=str(args.reload),
        reload_sha256=sha(args.reload), previous_deployment=str(args.previous),
        previous_deployment_sha256=sha(args.previous), recorded_ns=time.time_ns(),
        physical_grasp_verified=False,
        deployment_kind='compute_only_reload_after_complete_cold_start')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(deployment_passed=True, result=str(args.output),
                         sha256=sha(args.output))), flush=True)


if __name__ == '__main__':
    main()
