"""Reverify a completed owned-unit stop; never repeat stop or start operations."""
import argparse
import copy
import json
import time
from pathlib import Path
import subprocess
import gen58_cold_install_guard_20261021 as guard
import gen58_full_cold_restart_common_20261021 as common


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--original-sha256', required=True)
    p.add_argument('--guard-sha256', required=True)
    a = p.parse_args()
    original = common.N/'gen58_resume_r1/stop_r1/result.json'
    output = common.N/'gen58_resume_r1/stop_recovery_r1'
    guard_path = common.W/'.runtime/gen58_cold_install_guard_20261021.py'
    require, sha, read = common.require, common.sha, common.read
    require(common.M.is_mount() and not output.exists(), 'Fresh mounted output required')
    require(sha(original) == a.original_sha256 and sha(guard_path) == a.guard_sha256, 'Recovery inputs changed')
    old = read(original)
    require(old['passed'] is False and old['stop_requested'] is True and old['stop_performed'] is False
        and old['error'] == "RuntimeError('unreadable same-user descriptor table: 34171')", 'Not the exact recorded post-stop inspection failure')
    before = old['before']
    common.control_and_request()
    require(before['control_snapshot'] == read(common.CONTROL) and before['control_sha256'] == common.CONTROL_SHA, 'Control changed')
    require(all(sha(path) == digest for path, digest in old['input_sha256'].items()), 'Original stop inputs changed')
    require(old['release'] == dict(path=str(common.RELEASE), sha256=sha(common.RELEASE)), 'Release changed')
    source = read(common.RELEASE)['source_sha256']
    common.release_scope(common.RELEASE, read(common.RELEASE))
    require(source == before['source_sha256'] and all(sha(common.W/path) == digest for path, digest in source.items()), 'Tested source changed')
    expected = [['systemctl','--user','stop',before['units'][key]['unit']] for key in ('core','simulation')]
    require(len(old['stop_commands']) == 2 and all(row['command'] == command and row['returncode'] == 0
        for row, command in zip(old['stop_commands'], expected)), 'Both owned stops did not complete')
    units = {}
    for key in ('core','simulation'):
        unit = before['units'][key]['unit']
        raw = subprocess.check_output(['systemctl','--user','show',unit,'--property=Id','--property=MainPID',
            '--property=ActiveState','--property=SubState'], text=True, timeout=3.)
        props = dict(line.split('=',1) for line in raw.splitlines() if '=' in line)
        require(props['Id'] == unit+'.service' and props['MainPID'] == '0' and props['ActiveState'] == 'inactive'
            and props['SubState'] == 'dead', 'Old unit not fully stopped')
        units[key] = props
    for row in before['node_identities'].values():
        proc = Path('/proc')/str(row['pid'])/'stat'
        require(not proc.exists() or proc.read_text().rsplit(')',1)[1].split()[19] != row['start_ticks'], 'An original node still exists')
    cold = guard.cold_pair()
    require(cold['passed'], 'Cold software/visible-writer verification failed')
    output.mkdir()
    result = copy.deepcopy(old)
    result.pop('error')
    result.update(passed=True, stop_performed=True, cold_after=cold,
        kind='completed_owned_stop_with_separate_readonly_cold_reverification',
        original_failed_receipt=dict(path=str(original), sha256=a.original_sha256),
        original_failure_preserved=True, stop_commands_origin='original stop_r1 invocation; not reissued',
        stop_or_start_commands_sent_this_invocation=[], inactive_owned_units=units,
        guard_scope='Known software and visible serial writers absent; identified non-dumpable desktop secret-service FDs and executable symlink remain unverified',
        recovery_helper=dict(path=str(Path(__file__).resolve()), sha256=sha(__file__)), recorded_ns=time.time_ns())
    result['input_sha256'].update({str(original):a.original_sha256,str(guard_path):a.guard_sha256,
        str(Path(__file__).resolve()):sha(__file__)})
    common.new_json(output/'result.json',result)
    print(json.dumps(dict(passed=True,path=str(output/'result.json'),sha256=sha(output/'result.json'),new_stop_or_start_commands=0)))


if __name__ == '__main__':
    main()
