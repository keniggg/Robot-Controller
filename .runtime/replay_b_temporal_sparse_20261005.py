#!/usr/bin/env python3
"""One original-parameter SDK temporal counterfactual on eight saved B frames."""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('ab_analysis',HERE/'analyze_camera_preset_ab_20261004.py')
ab=importlib.util.module_from_spec(spec);spec.loader.exec_module(ab)
rs=ab.rs


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--capture',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();folder=args.capture/'stages'
    summary=json.loads((folder/'summary.json').read_text())
    for item in summary['files']:
        p=folder/item['name'];assert p.stat().st_size==item['bytes'] and ab.sha(p)==item['sha256']
    infos=[json.loads((folder/f'sample_{i:02}.json').read_text()) for i in range(1,25)]
    revised=args.capture/'offline_comparison_revised_20261005.json'
    cfg=ab.tabletop.Config(**json.loads(revised.read_text())['config'])
    report=dict(schema=1,source=str(args.capture),source_summary_sha256=ab.sha(folder/'summary.json'),
        revised_report_sha256=ab.sha(revised),script_sha256=ab.sha(__file__),sdk_version=rs.__version__,
        source_manifest_verified=True,production_modified=False,physical_devices_opened=False,
        geometry_authority=False,selection_or_motion_authority=False,
        scope='Sparse 8-frame SDK mathematical counterfactual, not live stationary filtering or repaired evidence.',
        limitations=['Saved frames are approximately 0.5 s apart, not the complete 30 FPS input. Missing frames are not generated.',
          'Filter history is deliberately retained across these sparse samples only to test the mathematical effect of the original settings. Production continuous RGB/encoder gates would not authorize this replay.',
          'No reconstruction of missing actual TF, per-frame exposure, visual-stationarity or source evidence. Old experiment remains invalid.',
          'Numerical original initial support check is not a measured target, full pipeline replay or permission for grasp.',
          'SDK software_device constructor initializes a udev monitor internally; no physical device enumeration, pipeline or device opening is performed.'],
        temporal_parameters=dict(alpha=.4,delta_depth_units=100,persistence=0),
        recorded_status=[],phase_status={},counterfactual_rows=[])
    for info in infos:
        temporal=info['runtime_profile'].get('stationary_temporal',{})
        visual=info['production_read_diagnostics'].get('stationary_rgb_evidence')
        report['recorded_status'].append(dict(index=info['sample_index'],phase=info['phase'],complete=info['complete'],
            temporal_active=temporal.get('active'),visual_reason=visual.get('reason') if visual else None))
    for phase in ('A1','B','A2'):
        rows=[r for r in report['recorded_status'] if r['phase']==phase and r['complete']]
        report['phase_status'][phase]=dict(complete_frames=len(rows),temporal_active=sum(r['temporal_active'] for r in rows),
                                          reasons=dict(Counter(r['visual_reason'] for r in rows)))
    rows=[i for i in infos if i['phase']=='B' and i['complete']];assert len(rows)==8
    first=rows[0];scale=first['depth_scale_m'];model=first['aligned_depth_intrinsics']
    mapper=ab.PairedPinholeRectification(first['runtime_profile']['color_intrinsics'],sdk=rs)
    K=[mapper.output_intrinsics[k] for k in ('fx','fy','cx','cy')]
    # This is an isolated software-only sensor. Never construct rs.context or a pipeline.
    device=rs.software_device();sensor=device.add_sensor('offline_saved_arrays_only')
    stream=rs.video_stream();stream.type=rs.stream.depth;stream.index=0;stream.uid=1
    stream.width=model['width'];stream.height=model['height'];stream.fps=30;stream.bpp=2;stream.fmt=rs.format.z16
    stream.intrinsics=ab.stage.intrinsics(model)
    profile=sensor.add_video_stream(stream).as_video_stream_profile()
    sensor.add_read_only_option(rs.option.depth_units,scale)
    queue=rs.frame_queue(1);sensor.open(profile);sensor.start(queue)
    temporal=rs.temporal_filter()
    for option,value in [(rs.option.filter_smooth_alpha,.4),(rs.option.filter_smooth_delta,100.),(rs.option.holes_fill,0.)]:
        temporal.set_option(option,value)
    report['actual_filter_option_readback']={str(o):temporal.get_option(o) for o in
        (rs.option.filter_smooth_alpha,rs.option.filter_smooth_delta,rs.option.holes_fill)}
    try:
        for info in rows:
            assert info['aligned_depth_intrinsics']==model and info['depth_scale_m']==scale
            index=info['sample_index'];meta=info['sdk_frames']['depth']
            with np.load(folder/f'sample_{index:02}.npz',allow_pickle=False) as z:
                source=z['pre_stationary_depth'].copy();rgb=z['raw_rgb'].copy();actual=z['post_stationary_depth'].copy()
            assert ab.stage.array_sha(source)==info['arrays']['pre_stationary_depth']['sha256']
            assert np.array_equal(source,actual),'B recorded temporal unexpectedly changed input'
            frame=rs.software_video_frame();frame.pixels=source;frame.bpp=2;frame.stride=source.shape[1]*2
            frame.timestamp=meta['timestamp_ms'];frame.domain=getattr(rs.timestamp_domain,meta['timestamp_domain'].split('.')[-1])
            frame.frame_number=meta['frame_number'];frame.profile=profile;frame.depth_units=scale
            sensor.on_video_frame(frame);input_frame=queue.wait_for_frame(1000)
            assert np.array_equal(np.asanyarray(input_frame.get_data()),source)
            output=np.asanyarray(temporal.process(input_frame).get_data()).copy()
            assert not np.any(output[source==0]),'persistence zero filled an input hole'
            rect_in=mapper.apply(rgb,source)[1];rect_out=mapper.apply(rgb,output)[1]
            if index==rows[0]['sample_index']:assert np.array_equal(source,output),'first frame must have no prior history'
            report['counterfactual_rows'].append(dict(index=index,source_frame_number=meta['frame_number'],
                source_timestamp_ms=meta['timestamp_ms'],source_array_sha256=ab.stage.array_sha(source),
                hypothetical_output_sha256=ab.stage.array_sha(output),current_zeros_preserved=True,
                changed_positive=int(np.count_nonzero(source!=output)),
                input_original_initial_support=ab.original_initial_support(rect_in,scale,K,cfg),
                hypothetical_original_initial_support=ab.original_initial_support(rect_out,scale,K,cfg)))
    finally:sensor.stop();sensor.close()
    report['summary']=dict(original_support_passes=sum(r['input_original_initial_support']['passed'] for r in report['counterfactual_rows']),
        hypothetical_support_passes=sum(r['hypothetical_original_initial_support']['passed'] for r in report['counterfactual_rows']),
        qualified_continuous_replays=0,changed_filter_parameters=False)
    assert Path('/media/zhuyupei/SanStand').is_mount(),'no storage fallback'
    with args.output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps(dict(output=str(args.output),sha256=ab.sha(args.output),phase_status=report['phase_status'],summary=report['summary'])))


if __name__=='__main__':main()
