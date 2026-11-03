"""Summarize a completed read-only transport observation, without ROS imports."""
import argparse
import collections
import hashlib
import json
from pathlib import Path

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')

def numeric(values):
    values = sorted(values)
    return dict(count=len(values), median=values[len(values)//2] if values else None,
                p95=values[min(len(values)-1, int(len(values)*.95))] if values else None,
                maximum=values[-1] if values else None)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--load-interval', type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    if EXT not in directory.parents or not Path('/media/zhuyupei/SanStand').is_mount():
        parser.error('existing mounted SanStand observation required')
    manifest = json.loads((directory/'manifest.json').read_text())
    events = [json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
    status = [row for row in events if row['kind'] == 'unknown_status']
    report = dict(schema=1, manifest=manifest, state_counts=[dict(generation=k[0], state=k[1], detail=k[2], count=v)
        for k,v in collections.Counter((r.get('generation'), r.get('state'),r.get('detail')) for r in status).items()], streams={})
    frames = {r.get('frame_timing', {}).get('source_stamp_ns'):r.get('frame_timing', {}) for r in status}
    for stream in ('color','depth'):
        rows = sorted((r for r in events if r['kind']=='image_header' and r['stream']==stream),key=lambda r:r['receive_monotonic'])
        source_map = {r['source_stamp_ns']:r for r in rows}
        arrivals = [f['raw_arrivals'][stream] for f in frames.values() if stream in f.get('raw_arrivals',{})]
        matched = [r for r in arrivals if r['source_stamp_ns'] in source_map]
        report['streams'][stream] = dict(independent_age_sec=numeric([r['source_age_sec'] for r in rows]),
            independent_receive_gap_sec=numeric([b['receive_monotonic']-a['receive_monotonic'] for a,b in zip(rows,rows[1:])]),
            unknown_matching_arrival_age_sec=numeric([r['source_age_at_arrival_sec'] for r in matched]),
            unknown_minus_independent_receive_sec=numeric([(r['arrival_ros_ns']-source_map[r['source_stamp_ns']]['receive_wall_ns'])*1e-9 for r in matched]))
    report['processing_wall_sec'] = numeric([f['processing_wall_sec']for f in frames.values() if 'processing_wall_sec' in f])
    report['stages'] = {}
    for stage in ('segmentation','rgb_refinement','tf_lookup'):
        fs=[f for f in frames.values() if stage in f.get('stage_wall_sec',{})]
        report['stages'][stage] = dict(wall_sec=numeric([f['stage_wall_sec'][stage]for f in fs]),
            thread_cpu_sec=numeric([f['stage_thread_cpu_sec'][stage]for f in fs if stage in f.get('stage_thread_cpu_sec',{})]))
    report['non_lost_timeline'] = []
    for row in status:
        if row.get('detail') == 'target_lost_requires_new_generation':
            continue
        f=row.get('frame_timing',{})
        report['non_lost_timeline'].append(dict(receive_wall_ns=row['receive_wall_ns'],generation=row.get('generation'),
            state=row.get('state'),detail=row.get('detail'),source_stamp_ns=row.get('source_stamp_ns'),
            source_age_sec=f.get('status_source_age_sec'),processing_sec=f.get('processing_wall_sec'),
            raw_arrivals=f.get('raw_arrivals',{})))
    if args.load_interval:
        load = json.loads(args.load_interval.read_text())
        report['external_load_interval'] = load
        start=manifest['started_wall_ns'];end=start+round(manifest['elapsed_sec']*1e9)
        report['load_observation_overlap_sec']=max(0,min(end,load['end_wall_ns'])-max(start,load['start_wall_ns']))*1e-9
        report['ready_statuses_during_load'] = [dict(receive_wall_ns=r['receive_wall_ns'],source_stamp_ns=r.get('source_stamp_ns'),generation=r.get('generation'))
            for r in status if r.get('state')=='ready' and load['start_wall_ns']<=r['receive_wall_ns']<=load['end_wall_ns']]
    report['limitations'] = ['Only numeric observer headers retained; no image or geometric quality replay.',
        'Unknown timing available only for status-sampled processed frames, not every raw callback.',
        'Healthy reception in this window does not establish resolution of an earlier intermittent stall.',
        'Status arrival indicates observation, not duration of authority or permission to move.',
        'Extra full RGB-D subscriptions add serialization/transport load.']
    report['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output=directory/'analysis.json'
    if output.exists():
        raise RuntimeError('analysis exists; preserve prior evidence')
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(output),'state_counts':report['state_counts'],'streams':report['streams'],
        'load_observation_overlap_sec':report.get('load_observation_overlap_sec')},ensure_ascii=False),flush=True)

if __name__=='__main__':
    main()
