"""File/identity contract shared by GEN29 passive capture and offline replay."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import xmlrpc.client

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arguments(description, *, replay=False):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--output', type=Path, required=True)
    for field in ('release', 'deployment'):
        parser.add_argument('--' + field, type=Path, required=True)
        parser.add_argument('--' + field + '-sha256', required=True)
    parser.add_argument('--expected-generation', type=int, required=True)
    parser.add_argument('--expected-selection-stamp-ns', type=int, required=True)
    parser.add_argument('--expected-epoch', type=int, required=True)
    if replay:
        parser.add_argument('--capture', type=Path, required=True)
        parser.add_argument('--capture-sha256', required=True)
    args = parser.parse_args()
    assert all(getattr(args, key) > 0 for key in (
        'expected_generation', 'expected_selection_stamp_ns', 'expected_epoch'))
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    args.output = args.output.resolve()
    assert E / 'ros_log' in args.output.parents and not args.output.exists()
    for field in ('release', 'deployment'):
        path = getattr(args, field).resolve(strict=True)
        assert E in path.parents and path.is_file()
        setattr(args, field, path)
        assert sha(path) == getattr(args, field + '_sha256')
    release = json.loads(args.release.read_text())
    deployment = json.loads(args.deployment.read_text())
    assert release['kind'] == 'gen29_supplemental_preview_station_handoff'
    assert release['passed'] is True and release['reviewed'] is True
    assert release['source_count'] == len(release['source_sha256']) == 970
    assert release['test_summary'] == dict(tests=7125, failures=0, errors=0, skipped=0)
    assert deployment['deployment_passed'] is True and deployment['whole_patch_deployed'] is True
    assert Path(deployment['source_manifest']).resolve() == args.release
    assert deployment['source_manifest_sha256'] == args.release_sha256
    assert deployment['source_sha256'] == release['source_sha256']
    expected_processes = dict(deployment['preserved_after'], **deployment['after'])
    assert len(expected_processes) == 11
    binding = dict(release=str(args.release), release_sha256=args.release_sha256,
        deployment=str(args.deployment), deployment_sha256=args.deployment_sha256,
        expected_generation=args.expected_generation,
        expected_selection_stamp_ns=args.expected_selection_stamp_ns,
        expected_epoch_ns=args.expected_epoch)
    return args, release, deployment, expected_processes, binding


def source_check(args, release, helpers):
    assert sha(args.release) == args.release_sha256
    assert sha(args.deployment) == args.deployment_sha256
    for relative, expected in release['source_sha256'].items():
        path = Path(relative)
        assert not path.is_absolute() and '..' not in path.parts
        assert sha(W / path) == expected, 'Released source changed: ' + relative
    for relative, expected in helpers.items():
        assert sha(W / relative) == expected, 'Helper changed: ' + relative


def process_check(rospy, expected_processes):
    class TimeoutTransport(xmlrpc.client.Transport):
        def make_connection(self, host):
            connection = super().make_connection(host)
            connection.timeout = 2.
            return connection
    observed = {}
    for name, expected in expected_processes.items():
        code, _message, uri = rospy.get_master().lookupNode(name)
        assert code == 1
        proxy = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())
        code, _message, pid = proxy.getPid(rospy.get_name())
        assert code == 1 and pid == expected['pid'], name
        fields = Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()
        actual = dict(pid=pid, start_ticks=fields[19], parent_pid=int(fields[1]))
        assert actual == expected, name
        observed[name] = actual
    return observed
