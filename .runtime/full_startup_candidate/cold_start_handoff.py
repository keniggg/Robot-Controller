"""Retire named temporary software owners only inside a future cold root start.

Default is review-only. --prepare is restricted to the cold root entry.
No ROS APIs, joint enable/disable commands or hardware service names are used.
"""
import argparse,json,os,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT_UNIT='alicia-ros-live-20260928.service'
ROOT_ENTRY='/home/zhuyupei/alicia_wa_full/.ros_log/ros_live_20260922_session/start_ros.sh'
OWNERS=HERE/'legacy_owner_commands.json'


def properties(unit):
    r=subprocess.run(['systemctl','--user','show',unit,'--property=MainPID',
        '--property=ActiveState','--property=LoadState','--property=ExecStart',
        '--property=ConsistsOf','--property=Job','--property=Type'],check=True,text=True,capture_output=True,timeout=3.)
    return dict(line.split('=',1) for line in r.stdout.splitlines() if '=' in line)


def validate_cold_parent(root,parent_pid,parent_argv):
    if (root.get('Type') not in ('simple','exec')
            or root.get('ActiveState') not in ('active','activating')
            or int(root.get('MainPID','0'))!=parent_pid
            or parent_argv not in [['/bin/bash',ROOT_ENTRY],['/usr/bin/bash',ROOT_ENTRY]]):
        raise RuntimeError('handoff is allowed only from the cold root entry before roslaunch')


def validate_owner(unit,props,expected):
    if props.get('LoadState')=='not-found':
        return False
    raw=props.get('ExecStart','')
    command=raw.split('argv[]=',1)[1].split(' ; ignore_errors=',1)[0] if 'argv[]=' in raw else ''
    if command!=expected:
        raise RuntimeError('temporary software owner changed: '+unit)
    return True


def cold_parent():
    parent=os.getppid()
    argv=(Path('/proc')/str(parent)/'cmdline').read_bytes().decode().strip('\0').split('\0')
    root=properties(ROOT_UNIT);validate_cold_parent(root,parent,argv)
    return root


def prepare():
    expected=json.loads(OWNERS.read_text())
    root=cold_parent()
    unexpected=set(root.get('ConsistsOf','').split())-set(expected)-{'alicia-candidate-simulation.service'}
    if unexpected:raise RuntimeError('unlisted root child owner: '+','.join(sorted(unexpected)))
    selected=[unit for unit,command in expected.items() if validate_owner(unit,properties(unit),command)]
    if selected:
        # Include inactive loaded units: an old PartOf start job may still be
        # queued. A single transaction cancels those jobs as well as live copies.
        subprocess.run(['systemctl','--user','stop',*selected],check=True,timeout=40.)
    cold_parent()
    for unit in selected:
        current=properties(unit)
        if (current.get('LoadState')!='not-found'
                and (current.get('ActiveState') not in ('inactive','failed')
                     or current.get('Job','') not in ('','0','0 /'))):
            raise RuntimeError('temporary owner still active or queued: '+unit)
    print(json.dumps(dict(status='temporary_owners_retired',units=selected,hardware_commands_sent=0)))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare',action='store_true');args=parser.parse_args(argv)
    if args.prepare:prepare()
    else:print(json.dumps(dict(status='review_only',owners=json.loads(OWNERS.read_text()),services_touched=False),indent=2))


if __name__=='__main__':main()
