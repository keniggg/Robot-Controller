"""Bounded file-only wrapper for the existing actual handoff snapshot.

--prepare stores inode/offset metadata only. A later --record binds a fresh
MODE_ATTEMPT parsed by the existing observer; no future identity is invented.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
import monitor_gen45_actual_handoff_20261018 as original

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def metadata(path):
    if not path.exists():return dict(path=str(path.resolve()),exists=False)
    s=path.stat()
    return dict(path=str(path.resolve()),exists=True,device=s.st_dev,inode=s.st_ino,
                size=s.st_size,mtime_ns=s.st_mtime_ns)
def put(path,value):
    with path.open('x')as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    assert json.loads(path.read_text())==value

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepare',action='store_true')
    ap.add_argument('--record',type=Path)
    ap.add_argument('--consumers',type=Path,required=True)
    ap.add_argument('--gateway-log',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    assert original.MOUNT.is_mount()
    assert all(original.inside(p,original.N)for p in(args.consumers,args.gateway_log))
    assert original.inside(args.output,original.V)and args.output.resolve()!=original.V.resolve()
    args.output.mkdir(exist_ok=True)
    current=dict(consumers=metadata(args.consumers),gateway=metadata(args.gateway_log))
    if args.prepare:
        assert args.record is None
        value=dict(kind='actual_handoff_monitor_preparation_only',prepared_ns=time.time_ns(),
                   source_log_baselines=current,selection=None,record=None,active_monitor_started=False,
                   helper_sha256=sha(__file__),original_helper_sha256=sha(original.__file__),
                   no_ROS_or_device_access=True,no_temperature_read=True,no_commands_sent=True,
                   no_grasp_or_release_result_claimed=True)
        path=args.output/('preparation_'+str(value['prepared_ns'])+'.json');put(path,value)
        print(json.dumps(dict(preparation=str(path),sha256=sha(path)),ensure_ascii=False));return
    assert args.record is not None and original.inside(args.record,original.E/'grasp_records')
    before=set(args.output.glob('snapshot_*.json'))
    argv=sys.argv
    try:
        sys.argv=[original.__file__,'--record',str(args.record),'--consumers',str(args.consumers),
                  '--gateway-log',str(args.gateway_log),'--output',str(args.output)]
        original.main()
    finally:sys.argv=argv
    added=set(args.output.glob('snapshot_*.json'))-before
    assert len(added)==1
    snapshot_path=added.pop();snapshot=json.loads(snapshot_path.read_text())
    start=snapshot['selection']['stamp_ns']/1e9
    diagnostics=[];rows,consumer_range=original.read_rows(args.consumers,4*1024*1024)
    for text,ref in rows:
        stamp=original.numeric_time(text)
        if stamp is None or stamp<start:continue
        data=original.json_after(text,'Station task diagnostic: ')
        if data is None:continue
        old=(data.get('before_retire')or{}).get('current_reference')or data.get('current_reference')or{}
        generation=old.get('selection_generation')
        diagnostics.append(dict(stamp_sec=stamp,evidence=ref,diagnostic=data,
            selection_generation=generation,
            association=('same_generation'if generation==snapshot['mode_generation']else
                         'foreign_generation'if generation is not None else'unresolved_do_not_assume_current_owner')))
    preparations=sorted(args.output.glob('preparation_*.json'))
    prep=json.loads(preparations[-1].read_text())if preparations else None
    baseline=(prep or{}).get('source_log_baselines',{})
    continuity={}
    for key,value in current.items():
        prior=baseline.get(key,{})
        continuity[key]=dict(before=prior,at_snapshot=value,
            same_inode=(bool(prior.get('exists'))and bool(value.get('exists'))and
                        (prior.get('device'),prior.get('inode'))==(value.get('device'),value.get('inode'))),
            truncated=(prior.get('exists')is True and value.get('exists')is True and value['size']<prior['size']))
    result=dict(kind='actual_handoff_with_station_diagnostic_snapshot',captured_ns=time.time_ns(),
        original_snapshot=dict(path=str(snapshot_path),sha256=sha(snapshot_path)),
        record=str(args.record.resolve()),selection=snapshot['selection'],
        source_log_continuity=continuity,station_task_diagnostics=diagnostics,
        consumer_range=consumer_range,strict_deliveries=snapshot['candidate_local_rejections'],
        first_Task_failure=snapshot['first_Task_failure'],runner_result=snapshot['runner_result'],
        endpoint_first_tested_sample_is_not_first_collision_along_curve=True,
        candidate_rejection_or_perception_retirement_alone_is_not_Task_terminal=True,
        no_physical_success_claimed=True,no_ROS_or_device_access=True,
        no_temperature_read=True,no_commands_sent=True,source_or_record_modified=False,
        helper_sha256=sha(__file__),original_helper_sha256=sha(original.__file__))
    path=args.output/('station_snapshot_'+str(result['captured_ns'])+'.json');put(path,result)
    print(json.dumps(dict(snapshot=str(path),sha256=sha(path),selection=result['selection'],
        station_diagnostics=len(diagnostics),strict_deliveries=len(result['strict_deliveries']),
        first_failure=None if result['first_Task_failure']is None else result['first_Task_failure']['message'][:300]),ensure_ascii=False))

if __name__=='__main__':main()
