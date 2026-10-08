#!/usr/bin/env python3
"""Read-only final state for the bounded receive diagnostic round."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import rospy
import rostopic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path)
    parser.add_argument('--experiments', nargs='+', default=('experiment1','experiment2'))
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    logs = mount/'实验记录/Robot-Controller/ros_log'
    case = (args.case or logs/'unknown_receive_round_20261008T042515Z').resolve()
    if logs not in case.parents or not all(Path(n).name == n and n not in ('.','..') for n in args.experiments):
        raise RuntimeError('Canonical external case and experiment names required')
    output = case/'final_runtime_state.json'
    if not mount.is_mount() or output.exists():
        raise RuntimeError('Mounted external directory and new result required')
    result = dict(start_wall_ns=time.time_ns(), motion_commands_sent=False,
                  temperature_queried=False, sources={}, parameters={}, topics={}, processes={})
    result['sources'][str(Path(__file__).resolve())] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    rospy.init_node('unknown_receive_round_final_readonly', anonymous=True, disable_rosout=True)
    for name in ('/grasp_mode/selection', '/camera/fps', '/camera/publish_fps',
                 '/grasp/unknown_lift_after_close_enabled'):
        try:
            result['parameters'][name] = rospy.get_param(name)
        except Exception as exc:
            result['parameters'][name] = dict(error=repr(exc))
    for name in ('/perception/unknown/detector_status', '/grasp/state',
                 '/alicia_d/accepted_joint_states', '/alicia_d/control_reference_epoch'):
        try:
            cls, _, _ = rostopic.get_topic_class(name, blocking=False)
            if cls is None:
                raise RuntimeError('Topic class unavailable')
            msg = rospy.wait_for_message(name, cls, timeout=3.)
            entry = dict(original_text=str(msg), observed_wall_ns=time.time_ns())
            if name.endswith('detector_status'):
                entry['original_json'] = msg.data
                entry['parsed'] = json.loads(msg.data)
            elif name == '/grasp/state':
                entry['active'] = bool(msg.active)
            elif name.endswith('accepted_joint_states'):
                entry.update(stamp_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id,
                             names=list(msg.name), positions=list(msg.position))
            else:
                entry.update(stamp_ns=msg.stamp.to_nsec(), frame_id=msg.frame_id)
            result['topics'][name] = entry
        except Exception as exc:
            result['topics'][name] = dict(error=repr(exc))
    for name, pid, expected in (('unknown',118128,2489668),('camera',413207,17664295),
                                ('driver',309913,11723315)):
        try:
            base = Path('/proc')/str(pid)
            fields = base.joinpath('stat').read_text().rsplit(')',1)[1].split()
            result['processes'][name] = dict(pid=pid,start_ticks=int(fields[19]),state=fields[0],
                identity_preserved=int(fields[19])==expected and fields[0]!='Z')
        except OSError as exc:
            result['processes'][name] = dict(pid=pid,error=repr(exc),identity_preserved=False)
    # Inspect the two ended, owned experiments only; do not signal any process.
    result['diagnostic_children'] = {}
    for experiment in args.experiments:
        data=json.loads((case/experiment/'experiment.json').read_text())
        statuses={}
        for name, child in data['children'].items():
            ident=child['identity']; path=Path('/proc')/str(ident['pid'])/'stat'
            try:
                fields=path.read_text().rsplit(')',1)[1].split()
                still_owned=int(fields[19])==ident['start_ticks'] and fields[0]!='Z'
            except FileNotFoundError:
                still_owned=False
            statuses[name]=dict(pid=ident['pid'],same_owned_process_still_running=still_owned,
                                recorded_returncode=child.get('returncode'))
        result['diagnostic_children'][experiment]=statuses
    result['finish_wall_ns']=time.time_ns()
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    output.with_suffix('.sha256').write_text(hashlib.sha256(output.read_bytes()).hexdigest()+'  '+output.name+'\n')
    print(json.dumps(dict(output=str(output),processes=result['processes'],
        active=result['topics'].get('/grasp/state',{}).get('active'),
        selection=result['parameters']['/grasp_mode/selection'],
        diagnostic_children=result['diagnostic_children']),ensure_ascii=False))


if __name__=='__main__':
    main()
