#!/usr/bin/env python3
"""Offline fixed-version SDK metadata/source comparison; no device access."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
ROOT = Path(__file__).resolve().parents[1]
WINDOWS = ['regional_production_20261006T105318.899385Z',
           'regional_production_20261006T112923.272799Z',
           'regional_production_20261006T123748.120315Z']


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def analyze(directory):
    path = directory/'read_diagnostics.jsonl'
    summary = json.loads((directory/'read_diagnostics.summary.json').read_text())
    assert summary['original_read_restored'] and not summary['queued_unwritten']
    reads = [json.loads(line)['existing_read_diagnostics'] for line in path.read_text().splitlines()]
    complete = [r for r in reads if r['result']=='complete']
    previous, low, mismatches = {}, [], []
    for index,row in enumerate(reads):
        raw=row.get('raw_frames',{})
        if row['result']=='complete':
            fps={s:raw[s]['metadata_raw']['actual_fps'] for s in ('color','depth')}
            if fps['color']<29000 and fps['depth']>=29000:
                mismatches.append(dict(read_index=index, color_raw=fps['color'],depth_raw=fps['depth'],
                    local_window=[dict(read_index=i,result=reads[i]['result'],
                        streams={s:{key:reads[i]['raw_frames'][s][key]
                            for key in ('frame_number','device_timestamp_ms','stream_fps')}
                            for s in ('color','depth')})
                        for i in range(max(0,index-1),min(len(reads),index+3))]))
        for stream in ('color','depth'):
            current=raw.get(stream,{})
            if not current.get('present'):
                continue
            old=previous.get(stream)
            key=(current['frame_number'],current['device_timestamp_ms'])
            if old is not None and key==(old['frame_number'],old['device_timestamp_ms']):
                continue
            fps=current['metadata_raw']['actual_fps']
            if row['result']=='complete' and fps<29000:
                dt=current['device_timestamp_ms']-old['device_timestamp_ms'] if old else None
                count=current['frame_number']-old['frame_number'] if old else None
                predicted=1000000*count/dt if dt and count is not None else None
                low.append(dict(read_index=index,stream=stream,actual_fps_raw=fps,
                    observed_unique_source_count_increment=count,observed_unique_source_interval_ms=dt,
                    predicted_raw_fps_from_observed_predecessor=predicted,
                    raw_quantization_remainder=(predicted-fps if predicted is not None else None),
                    paired_raw_fps={s:raw[s]['metadata_raw']['actual_fps'] for s in ('color','depth')}))
            previous[stream]=current
    stats={}
    for stream in ('color','depth'):
        values=[r['raw_frames'][stream]['metadata_raw']['actual_fps'] for r in complete]
        run=maximum=0
        for value in values:
            run=run+1 if value<29000 else 0
            maximum=max(maximum,run)
        stats[stream]=dict(raw_fps_distribution=dict(sorted(Counter(values).items())),
            complete_reads=len(values),below_29fps_count=sum(v<29000 for v in values),
            maximum_consecutive_low_fps_complete_reads=maximum)
    return dict(directory=str(directory),read_log_sha256=sha(path.read_bytes()),
        complete_reads=len(complete),per_stream=stats,low_fps_observations=low,
        color_only_low_fps=mismatches)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output=BASE/'actual_fps_metadata_review_20261006'
    output.mkdir(exist_ok=False)
    sources={
        'src/metadata-parser.h':dict(lines=[445,458],fact='ACTUAL_FPS is integer(calc_actual_fps()*1000).'),
        'src/frame.cpp':dict(lines=[157,166],fact='calc_actual_fps = 1000 * frame-number delta / timestamp delta, in milliseconds; zero when invalid.'),
        'src/uvc-sensor.cpp':dict(lines=[119,180],fact='Each UVC profile callback carries previous timestamp/frame number, then updates them for subsequent frame.'),
        'src/ds/d400/d400-color.cpp':dict(lines=[63,117,182,190],fact='D405 color shares the depth endpoint, _separate_color=false; color AUTO_EXPOSURE_PRIORITY is registered only when separate color is true.'),
        'src/ds/ds-timestamp.cpp':dict(lines=[76,99,312,320],fact='Metadata frame counter has a backup reader; its fallback increments independent Z16/non-Z16 counters.'),
        'src/ds/ds-color-common.cpp':dict(lines=[67,84],fact='Color ACTUAL_FPS uses ds_md_attribute_actual_fps, distinct from hardware exposure and frame-counter metadata.')}
    for name,entry in sources.items():
        entry['url']='https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.5/'+name
        raw=urlopen(entry['url'],timeout=12).read()
        entry['download_sha256']=sha(raw)
        entry['download_bytes']=len(raw)
    configs=[ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
        ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/depth_preset.py',
        ROOT/'.runtime/d405_carton_depth_baseline.json']
    results=[analyze(BASE/window) for window in WINDOWS if (BASE/window/'read_diagnostics.summary.json').exists()]
    report=dict(schema=1,diagnostic_only=True,hardware_access=False,quality_thresholds_changed=False,
        script_sha256=sha(Path(__file__).read_bytes()),sdk_version_examined='2.56.5',official_sources=sources,
        windows=results,source_sha256={str(p):sha(p.read_bytes()) for p in configs},
        configuration_findings=[
            'Manager requests 30 Hz for both streams via config.enable_stream, and reads profile FPS. No explicit exposure or auto_exposure_priority set_option call exists in this manager.',
            'ModeDepthPreset restores the saved advanced baseline before applying unknown HighAccuracy visual_preset=3. Saved baseline has controls-autoexposure-auto=True, controls-autoexposure-manual=33000, controls-depth-gain=16; these are stored settings, not an exposure measurement.',
            'No current sensor option query was performed. The separate-color AUTO_EXPOSURE_PRIORITY branch is not the D405 branch in this SDK version.'
        ], conclusions=[
            'ACTUAL_FPS is a derivative of frame counters and timestamps, not an independent exposure-priority flag or direct confirmation of a configured lower frame rate.',
            'Every low-FPS complete observation in the first two 30-second windows is isolated, followed by ordinary ~30 Hz metadata. This does not show sustained 15 Hz operation.',
            'At old window index89, count+1 over133.365ms predicts7498.206 raw units; at new window index127, color+1 over66.680ms predicts14996.961 while depth advanced+2 overthe same pair span and reports29993. The next pair has repeated old color and newer depth, followed by a fresh normal interval.',
            'Using actual_fps to widen a continuity window would use the same observed gap to justify itself and could reclassify a missing frame as valid lower-rate motion evidence. These records do not justify that change.',
            'frame_counter metadata is reported unsupported. SDK has software backup counters, but the public metadata absence alone does not fully establish which internal path every frame used.',
            'The observations are compatible with a missing/delayed color stream sample or backend/SDK delivery history, but cannot locate the loss among camera, USB/virtualization, backend or scheduling.',
            'The final old-camera stderr sample at1791289377.321969 shows prior completed color14996/depth29993; its repeated color in the rejected pair is the same source, not another independent15Hz observation.',
            'These static metadata windows are not generation20 support or grasp validation. No motion, image capture, exposure or preset change is performed.'
        ])
    path=output/'report.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(report=str(path),sha256=sha(path.read_bytes()),windows=[dict(name=Path(r['directory']).name,complete=r['complete_reads'],low={s:r['per_stream'][s]['below_29fps_count'] for s in ('color','depth')}) for r in results])))


if __name__=='__main__':
    main()
