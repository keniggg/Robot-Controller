"""Start the frozen full stack once, after sealed actual-worktree regression."""
import hashlib
import json
from pathlib import Path
import shutil
import socket
import subprocess
import time

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT / '实验记录/Robot-Controller'
N = EXT / 'ros_log/full_latest_contact_admission_20261013T011103Z'
S = EXT / 'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
RELEASE = N / 'verification/gen11_contact_release_manifest.json'
STARTUP_DEPENDENCIES = {
    '.runtime/full_latest_no_skin_20261010.launch': '28ec4d2ba5138b48fb4f44b9824c586448e718d31a567701c395b1c0ae3645ed',
    '.runtime/full_startup_candidate/candidate_pair_and_simulation.launch': '04e4d86d82afbd4272f0e1a21fcf3e03208d0f5e8a3b9186d826ff494989e6e0',
    '.runtime/candidate_models/expanded_metric_20260929/candidate_parameters.launch': '4fe1fdf65ff3ea5b1f2b66f0a019feaf8cfba15769844a180ee2d96f74ca38cf',
    '.runtime/full_startup_candidate/candidate_simulation_startup.py': 'dab922722b43e85ab7723ba4c44b88f53d31498e4ef206d6f8f7c43a07542dfd',
    '.runtime/verify_full_contact_restart_20261012.py': '297cefe0b3c5e57cc0acdb6d0b86be1b345a68bacf73ba7d88f3589d8a680e2e',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def main():
    assert MOUNT.is_mount() and N.stat().st_dev == MOUNT.stat().st_dev
    release = json.loads(RELEASE.read_text())
    assert all(release[k] is True for k in ('passed', 'reviewed', 'installed'))
    assert release['test_summary']['tests'] > 0
    assert all(release['test_summary'][k] == 0 for k in ('failures', 'errors', 'skipped'))
    for relative, expected in release['source_sha256'].items():
        assert sha(W / relative) == expected, relative
    for path, expected in release['evidence_sha256'].items():
        assert sha(path) == expected, path
    for relative, expected in STARTUP_DEPENDENCIES.items():
        assert sha(W / relative) == expected, relative
    config = json.loads((EXT / 'grasp_records/archive_config.json').read_text())
    assert config['upload_enabled'] is False and config['delete_enabled'] is False
    assert shutil.disk_usage(EXT).free >= config['min_free_bytes']
    control = json.loads((EXT / 'maintenance/storage_control.json').read_text())
    assert control['archive_policy'] == 'local_only'
    for port in (11311, 8002):
        with socket.socket() as client:
            client.settimeout(.3)
            assert client.connect_ex(('127.0.0.1', port)) != 0, ('existing listener', port)
    old = json.loads((S / 'gen10_ready_for_gen11_runtime_deployment.json').read_text())
    names = set(old['after']) | set(old['preserved_after'])
    existing = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc / 'cmdline').read_bytes().split(b'\0')
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
        for name in names:
            if ('__name:=' + name[1:]).encode() in argv or ('__name:=' + name).encode() in argv:
                existing.append(dict(node=name, pid=int(proc.name)))
    assert not existing, existing
    current = json.loads((N / 'startup.json').read_text())
    assert not current.get('core') and not current.get('simulation'), 'already attempted startup'
    current.update(source_manifest=str(RELEASE), source_manifest_sha256=sha(RELEASE),
                   startup_dependencies_sha256=STARTUP_DEPENDENCIES,
                   startup_script_sha256=sha(__file__),
                   pre_start_existing_nodes=existing, pre_start_listener_ports=[],
                   pre_start_checked_ns=time.time_ns(), full_restart=True,
                   active_work='starting_reviewed_full_stack')
    old_start = json.loads((S / 'startup.json').read_text())
    save(N / 'startup.json', current)
    for key in ('simulation', 'core'):
        prior = old_start[key]
        unit = prior['unit'].replace('20261012t021050z', '20261013t011103z')
        command = [str(part).replace(str(S), str(N)).replace(prior['unit'], unit)
                   for part in prior['command']]
        assert '--property=Restart=no' in command
        completed = subprocess.run(command, capture_output=True, text=True, timeout=30.)
        current[key] = dict(unit=unit, command=command, returncode=completed.returncode,
                            stdout=completed.stdout, stderr=completed.stderr, requested_ns=time.time_ns())
        save(N / 'startup.json', current)
        print(json.dumps(dict(component=key, **current[key]), ensure_ascii=False), flush=True)
        assert completed.returncode == 0, completed.stderr
    current['active_work'] = 'waiting_for_new_stack_acceptance'
    save(N / 'startup.json', current)


if __name__ == '__main__':
    main()
