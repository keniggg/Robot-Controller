"""Read sealed GEN61 failure evidence only. No node, network or control I/O."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import platform
import struct
import sys
import time

sys.dont_write_bytecode = True
# Offline import bootstrap: eliminate Python3.8 uuid/Cryptodome platform probes.
_u=os.uname()
platform._uname_cache=platform.uname_result(_u.sysname,_u.nodename,_u.release,_u.version,_u.machine,'')
platform.architecture=lambda *a,**kw:(str(struct.calcsize('P')*8)+'bit','ELF')
denied=[]
def guard(event,args):
    if event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.posix_spawn','os.fork','os.exec'):
        denied.append(event);raise RuntimeError('offline analysis forbids '+event)
sys.addaudithook(guard)
import rosbag

W=Path(__file__).resolve().parents[1]
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T092802.096230Z_567d85bbe8f54a56af71145548b078f8')
OUT=E/'contact_handoff_diagnosis_r1'
BAG=R/'keyframes.bag'
TOPICS=(
 '/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference_epoch',
 '/alicia_d/actuation_status','/alicia_d/motion_enabled',
 '/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status',
 '/alicia_controller/follow_joint_trajectory/goal','/alicia_controller/follow_joint_trajectory/result',
 '/grasp/state','/grasp/task_event','/grasp/near_field_phase',
 '/grasp_mode/contact_station_retirement_request','/grasp_mode/contact_station_retirement_ack',
 '/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack',
 '/grasp_mode/initial_preflight_attempt','/grasp_6d/frozen_contact_plan',
 '/grasp_6d/preview_plan_enriched','/grasp_6d/plan_enriched','/grasp_6d/status',
 '/perception/unknown/detector_status','/experiment/observation_rgbd/metadata',
 '/experiment/runtime_execution_error_snapshot','/rosout_agg')
LOG_OWNERS={'/grasp_task_node','/remote_grasp6d_node','/unknown_tabletop_perception','/motion_gateway'}


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def save(path,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with path.open('xb') as f:f.write(raw);f.flush();os.fsync(f.fileno())
    assert path.read_bytes()==raw


def data(value):
    if hasattr(value,'to_nsec'):
        return dict(ns=value.to_nsec())
    if hasattr(value,'__slots__'):
        return {k:data(getattr(value,k)) for k in value.__slots__}
    if isinstance(value,(list,tuple)):
        return [data(v) for v in value]
    if isinstance(value,bytes):
        raise RuntimeError('unexpected raw bytes in selected non-image topic')
    return value


def main():
    started=time.monotonic();mount=Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and E.stat().st_dev==mount.stat().st_dev
    assert not os.statvfs(mount).f_flag & os.ST_RDONLY
    manifest=json.loads((R/'manifest.json').read_bytes())
    assert manifest['recorder_closed'] is True and manifest['result']=='failure'
    row=next(x for x in manifest['files'] if x['path']=='keyframes.bag')
    assert BAG.stat().st_size==row['size_bytes'] and sha(BAG)==row['sha256']
    OUT.mkdir(exist_ok=False)
    counts=Counter();joints=[];events=[];contact=[];plans=[]
    with rosbag.Bag(str(BAG),'r') as bag, (OUT/'selected_wire.jsonl').open('x') as wire:
        inventory={topic:dict(type=info.msg_type,count=info.message_count)
                   for topic,info in bag.get_type_and_topic_info().topics.items()}
        for topic,msg,received in bag.read_messages(topics=TOPICS):
            if topic=='/rosout_agg' and msg.name not in LOG_OWNERS:
                continue
            payload=data(msg)
            if hasattr(msg,'data') and isinstance(msg.data,str):
                try:payload['parsed']=json.loads(msg.data)
                except ValueError:pass
            record=dict(topic=topic,bag_receipt_ns=received.to_nsec(),message=payload)
            wire.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
            counts[topic]+=1
            if topic=='/alicia_d/accepted_joint_states':
                joints.append(dict(source_ns=msg.header.stamp.to_nsec(),bag_receipt_ns=received.to_nsec(),
                    frame_id=msg.header.frame_id,names=list(msg.name),positions=list(msg.position)))
            if topic in ('/grasp/state','/grasp/task_event','/grasp/near_field_phase'):
                events.append(record)
            if 'contact_station_retirement' in topic:contact.append(record)
            if topic in ('/grasp_6d/preview_plan_enriched','/grasp_6d/plan_enriched'):
                plans.append(dict(received_ns=received.to_nsec(),source_ns=msg.header.stamp.to_nsec(),
                    valid=msg.valid,plan_id=msg.plan_id,diagnostic=msg.diagnostic))
        wire.flush();os.fsync(wire.fileno())
    failures=[r for r in events if r['topic']=='/grasp/state' and
              'CONTACT_STATION_FRESH_FEEDBACK_REQUIRED' in r['message'].get('message','')]
    assert failures
    failure_ns=failures[0]['message']['header']['stamp']['ns']
    window=[r for r in joints if failure_ns-5_000_000_000<=r['bag_receipt_ns']<=failure_ns+3_000_000_000]
    gaps=[b['source_ns']-a['source_ns'] for a,b in zip(window,window[1:])]
    ages=[r['bag_receipt_ns']-r['source_ns'] for r in window]
    latest=max((r for r in joints if r['bag_receipt_ns']<=failure_ns),key=lambda r:r['bag_receipt_ns'])
    save(OUT/'task_events.json',events)
    save(OUT/'contact_requests.json',contact)
    save(OUT/'accepted_feedback_failure_window.json',window)
    save(OUT/'plans_timing.json',plans)
    save(OUT/'topic_inventory.json',inventory)
    source_paths=[W/'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_task.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py',
        W/'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
        W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py',
        W/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py']
    result=dict(kind='GEN61_contact_handoff_frozen_wire_diagnosis',passed=True,
        recorded_failure_reproduced_as_evidence=True,production_failure_mechanism_reproduced=False,
        source_record=str(R),bag_sha256=row['sha256'],selected_topics=list(TOPICS),decoded_counts=dict(counts),
        decoder_did_not_initialize_ROS=True,network_or_process_attempts=denied,temperature_topics_not_decoded=True,
        failure_ns=failure_ns,contact_requests=contact,
        sdk_failure_window=dict(samples=len(window),maximum_source_gap_ns=max(gaps),
            minimum_receipt_age_ns=min(ages),maximum_receipt_age_ns=max(ages),
            latest_before_failure=latest,source_age_at_failure_ns=failure_ns-latest['source_ns']),
        scope='Bag receipt proves recorder arrival, not task-local callback/cache time.',
        source_sha256={str(p):sha(p) for p in source_paths},
        input_sha256={str(p):sha(p) for p in (R/'manifest.json',R/'recorder_outcome.json',R/'result_summary.json',Path(__file__))},
        artifact_sha256={str(p):sha(p) for p in OUT.iterdir() if p.is_file()},
        elapsed_sec=time.monotonic()-started,recorded_ns=time.time_ns())
    assert not denied
    assert sha(BAG)==row['sha256']
    save(OUT/'wire_result.json',result)
    print(json.dumps({k:result[k] for k in ('failure_ns','sdk_failure_window','contact_requests','elapsed_sec')},ensure_ascii=False))
    print(json.dumps(dict(result=str(OUT/'wire_result.json'),sha256=sha(OUT/'wire_result.json')),ensure_ascii=False))


if __name__=='__main__':main()
