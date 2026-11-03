#!/usr/bin/env python3
"""Resume policy-permitted storage services after SanStand appears; never restart them.

The timer invokes this bounded storage-maintenance check. No arm/ROS services,
archive payloads, upload manifests, or running worker processes are modified.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import tempfile

MOUNT = Path('/media/zhuyupei/SanStand')
BASE = MOUNT / '实验记录/Robot-Controller'
SERVICES = ('alicia-grasp-archive.service', 'alicia-grasp-archive-unknown.service',
            'alicia-active-log-mirror.service', 'alicia-archive-finish-20260927.service')
FINISH_SERVICE = SERVICES[-1]
FINISH_STATUS = BASE / 'maintenance/archive_finish_20260927/status.json'
PAUSE_STATUS = BASE / 'maintenance/storage_control.json'
LOCAL_SERVICES = {'alicia-active-log-mirror.service', 'alicia-log-spool-cleanup.service'}
POLICIES = ('local_only', 'github_verified')


def control_status(path=None):
    """Missing or unreadable control state fails closed until explicit resume."""
    path = Path(path or PAUSE_STATUS)
    try:
        value = json.loads(path.read_text())
        if value.get('schema') != 1 or value.get('state') not in ('paused', 'resumed'):
            raise ValueError('invalid storage control record')
        policy = value.get('archive_policy', 'local_only')
        if policy not in POLICIES:
            raise ValueError('invalid archive policy')
        return {'paused': value['state'] == 'paused', 'reason': value['state'], 'archive_policy': policy}
    except FileNotFoundError:
        return {'paused': True, 'reason': 'storage_control_missing'}
    except (OSError, ValueError, AttributeError) as error:
        return {'paused': True, 'reason': 'storage_control_unreadable: ' + str(error)}


def set_control(state, reason, mount=MOUNT, path=None, mount_check=None, archive_policy=None):
    """Persist a user-directed pause/resume without changing any service."""
    if state not in ('paused', 'resumed') or not reason.strip():
        raise ValueError('a paused/resumed state and explicit reason are required')
    mounted = mount_check or (lambda: Path(mount).is_mount())
    if not mounted():
        raise RuntimeError('EXPERIMENT_STORAGE_UNAVAILABLE: mount SanStand before setting control state')
    path = Path(path or PAUSE_STATUS)
    policy = archive_policy or control_status(path).get('archive_policy', 'local_only')
    if policy not in POLICIES:
        raise ValueError('invalid archive policy')
    path.parent.mkdir(parents=True, exist_ok=True)
    value = {'schema': 1, 'state': state, 'reason': reason, 'archive_policy': policy,
             'updated_utc': datetime.now(timezone.utc).isoformat()}
    temporary = None
    try:
        if not mounted():
            raise RuntimeError('EXPERIMENT_STORAGE_UNAVAILABLE: SanStand disappeared')
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.storage-control-', delete=False) as output:
            temporary = Path(output.name)
            json.dump(value, output, ensure_ascii=False, indent=2)
            output.write('\n')
            output.flush()
            os.fsync(output.fileno())
        if not mounted():
            raise RuntimeError('EXPERIMENT_STORAGE_UNAVAILABLE: SanStand disappeared')
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return value


def service_allowed(unit, control):
    if control['paused']:
        return False
    if control.get('archive_policy', 'local_only') == 'local_only':
        return unit in LOCAL_SERVICES
    return unit in SERVICES or unit in LOCAL_SERVICES


def invoke(arguments):
    return subprocess.run(['systemctl', '--user', *arguments], check=True,
                          capture_output=True, text=True, timeout=15)


def resume_services(mount=MOUNT, finish_status=FINISH_STATUS, invoke_command=invoke,
                    mount_check=None, pause_status=None):
    mount = Path(mount)
    mounted = mount_check or (lambda: mount.is_mount())
    report = {'mounted': bool(mounted()), 'started': [], 'unchanged': [], 'failed': []}
    if not report['mounted']:
        # In particular do not create log directories beneath an unmounted path.
        report['reason'] = 'EXPERIMENT_STORAGE_UNAVAILABLE: waiting for SanStand'
        return report
    control = control_status(pause_status)
    if control['paused']:
        report.update(control)
        return report
    finish_complete = False
    if Path(finish_status).exists():
        try:
            status = json.loads(Path(finish_status).read_text())
            finish_complete = (status.get('schema') == 1 and status.get('complete') is True and
                               status.get('phase') == 'complete' and
                               status.get('scope') == 'frozen-experiments-through-20260927')
        except (OSError, ValueError):
            # An absent/unreadable completion receipt cannot assert completion.
            pass
    for unit in SERVICES:
        if not service_allowed(unit, control):
            report['unchanged'].append({'unit': unit, 'reason': 'archive_policy_local_only'})
            continue
        if unit == FINISH_SERVICE and finish_complete:
            report['unchanged'].append({'unit': unit, 'reason': 'archive_scope_complete'})
            continue
        if not mounted():
            report['mounted'] = False
            report['reason'] = 'EXPERIMENT_STORAGE_UNAVAILABLE: SanStand disappeared'
            break
        try:
            response = invoke_command(['show', unit, '--property=LoadState', '--property=ActiveState'])
            state = dict(line.split('=', 1) for line in response.stdout.splitlines() if '=' in line)
            if state.get('LoadState') != 'loaded':
                report['unchanged'].append({'unit': unit, 'reason': 'unit_not_loaded'})
                continue
            active = state.get('ActiveState')
            if active not in ('inactive', 'failed'):
                report['unchanged'].append({'unit': unit, 'reason': active or 'unknown_state'})
                continue
            if not mounted():
                report['mounted'] = False
                report['reason'] = 'EXPERIMENT_STORAGE_UNAVAILABLE: SanStand disappeared'
                break
            # The marker is re-read immediately before each start. ExecCondition
            # also checks it when a queued start eventually executes.
            control = control_status(pause_status)
            if control['paused']:
                report.update(control)
                break
            if not service_allowed(unit, control):
                report['unchanged'].append({'unit': unit, 'reason': 'archive_policy_local_only'})
                continue
            if active == 'failed':
                invoke_command(['reset-failed', unit])
            # start on an already-active unit is idempotent even if its state
            # changed after show. No stop/restart/kill operation is available.
            invoke_command(['start', '--no-block', unit])
            report['started'].append(unit)
        except (OSError, subprocess.SubprocessError) as error:
            report['failed'].append({'unit': unit, 'error': str(error)})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--set-state', choices=('paused', 'resumed'))
    mode.add_argument('--check-enabled', action='store_true', help='ExecCondition: require mount and no user pause')
    parser.add_argument('--reason', default='', help='Required for an explicit persistent state change')
    parser.add_argument('--archive-policy', choices=POLICIES, help='Explicit policy change; pause/resume otherwise preserves policy')
    parser.add_argument('--unit', default='', help='Service name for --check-enabled (systemd %%n)')
    args = parser.parse_args()
    if args.archive_policy and not args.set_state:
        parser.error('--archive-policy requires --set-state and an explicit reason')
    if args.set_state:
        try:
            report = set_control(args.set_state, args.reason, archive_policy=args.archive_policy)
        except (OSError, ValueError, RuntimeError) as error:
            parser.exit(1, str(error) + '\n')
        print(json.dumps(report, ensure_ascii=False), flush=True)
        return 0
    if args.check_enabled:
        return 0 if MOUNT.is_mount() and service_allowed(args.unit, control_status()) else 1
    report = resume_services()
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return 1 if report['failed'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
