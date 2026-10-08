#!/usr/bin/env python3
"""SDK 2.56.5 SOFTWARE frames only; no context, pipeline or physical device."""
import json
from pathlib import Path
import resource
import shutil
import sys
import time

import cv2
import numpy as np

WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
sys.path.insert(0,str(WORK/'.runtime/realsense-2.56.5'))
from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth
import pyrealsense2 as rs
import check_saved_rgb_hold_overlap_20261005 as base


def forbidden(*args,**kwargs):
    raise AssertionError('physical SDK context or pipeline is forbidden in this software benchmark')


assert rs.__version__=='2.56.5'
rs.context=forbidden
rs.pipeline=forbidden
if not base.MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
SOURCE=base.EXT/'ros_log/matched_shadow_current_20261005T024652.714830Z/stages/sample_11'
OUT=base.EXT/'ros_log/regional_sdk_software_benchmark_20261005T0450Z'
info=json.loads(SOURCE.with_suffix('.json').read_text())
with np.load(SOURCE.with_suffix('.npz'),allow_pickle=False) as data:
    color=data['raw_rgb']
if color.shape!=(480,640,3) or color.dtype!=np.uint8 or base.array_sha(color)!=info['arrays']['raw_rgb']['sha256']:
    raise ValueError('retained native RGB source differs')
color.setflags(write=False)
cv2.setNumThreads(1)
rss_before=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
device=rs.software_device()
sensor=device.add_sensor('regional_five_temporal_software_only')
stream=rs.video_stream();stream.type=rs.stream.depth;stream.index=0;stream.uid=91
stream.width=640;stream.height=480;stream.fps=30;stream.bpp=2;stream.fmt=rs.format.z16
k=rs.intrinsics();k.width=640;k.height=480;k.fx=k.fy=400.;k.ppx=320.;k.ppy=240.
k.model=rs.distortion.none;k.coeffs=[0.]*5;stream.intrinsics=k
profile=sensor.add_video_stream(stream).as_video_stream_profile()
sensor.add_read_only_option(rs.option.depth_units,.0001)
queue=rs.frame_queue(1);sensor.open(profile);sensor.start(queue)
filters=[];filter_durations=[]


def factory(*,alpha,delta,persistence):
    assert (alpha,delta,persistence)==(.4,100,0)
    native=rs.temporal_filter()
    native.set_option(rs.option.filter_smooth_alpha,alpha)
    native.set_option(rs.option.filter_smooth_delta,float(delta))
    native.set_option(rs.option.holes_fill,float(persistence))
    class Timed:
        def __init__(self):
            self.calls=0
        def process(self,frame):
            began=time.perf_counter()
            result=native.process(frame)
            filter_durations.append(time.perf_counter()-began)
            self.calls+=1
            return result
    wrapped=Timed();filters.append(wrapped)
    return wrapped


engine=RegionalStationaryDepth(factory)
rows=[]
yy,xx=np.indices((480,640))
fixed_holes=(yy%13==0)&(xx%17==0)
try:
    for index in range(120):
        pixels=np.full((480,640),2000+50*(index%2),np.uint16)
        holes=fixed_holes.copy();holes[:,index*5%640]=True
        pixels[holes]=0
        original=pixels.copy()
        incoming=rs.software_video_frame();incoming.pixels=pixels
        incoming.bpp=2;incoming.stride=1280;incoming.timestamp=1000.+index*1000./30.
        incoming.domain=rs.timestamp_domain.hardware_clock;incoming.frame_number=100+index
        incoming.profile=profile;incoming.depth_units=.0001
        sensor.on_video_frame(incoming)
        frame=queue.wait_for_frame(1000)
        source=dict(frame_number=frame.get_frame_number(),device_timestamp_ms=frame.get_timestamp(),
            timestamp_domain=str(frame.get_frame_timestamp_domain()),stream_fps=30.)
        before=time.perf_counter()
        # This explicit alignment flag belongs only to the synthetic software
        # test's same-grid inputs, never proof about the physical camera.
        output,report=engine.process(frame,color,source,1000.+index/30.,True,aligned_to_color=True)
        elapsed=time.perf_counter()-before
        unchanged=bool(np.array_equal(np.asanyarray(frame.get_data()),original)
            and np.array_equal(pixels,original))
        zero_preserved=bool(np.all(output[holes]==0))
        detached=not np.shares_memory(output,np.asanyarray(frame.get_data()))
        assert unchanged and zero_preserved and detached
        if index<11:
            assert not report['active'] and np.array_equal(output,original)
        rows.append(dict(index=index,virtual_source_timestamp_ms=source['device_timestamp_ms'],
            virtual_host_seconds=1000.+index/30.,process_seconds=elapsed,
            input_unchanged=unchanged,current_zero_preserved=zero_preserved,result_detached=detached,
            current_zero_pixels=int(holes.sum()),active=report['active'],fully_active=report['fully_active'],
            filtered_fraction=report['filtered_fraction'],changed_pixel_count=report['changed_pixel_count'],
            valid_output_min=int(output[~holes].min()),valid_output_max=int(output[~holes].max()),
            regions=[dict(name=r['name'],qualified=r['qualified'],warming=r['warming'],
                history_reset=r['history_reset'],filter_applied=r['filter_applied'],
                visual_reason=r['visual']['reason'],motion_p90_px=r['visual'].get('motion_p90_px'))
                for r in report['regions']]))
        del frame,incoming
finally:
    engine.reset()
    sensor.stop();sensor.close()


def timing(values):
    values=np.asarray(values)*1000.
    return dict(count=len(values),mean_ms=float(values.mean()),p50_ms=float(np.median(values)),
        p95_ms=float(np.percentile(values,95)),max_ms=float(values.max()))


result=dict(schema=1,software_only=True,sdk_version=rs.__version__,
    physical_context_forbidden=True,physical_pipeline_forbidden=True,robot_commands_sent=False,
    real_camera_alignment_claimed=False,source_RGB=str(SOURCE.with_suffix('.npz')),
    source_RGB_array_sha256=base.array_sha(color),source_RGB_shape=list(color.shape),
    source_metadata_sha256=base.sha(SOURCE.with_suffix('.json')),
    regional_module_sha256=base.sha(WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py'),
    script_sha256=base.sha(__file__),software_stream=dict(width=640,height=480,fps=30,format='z16',depth_units=.0001),
    limits=dict(alpha=.4,delta_depth_units=100,persistence=0,frames=120,window_seconds=.35,motion_px=.25),
    process_timing=timing([r['process_seconds'] for r in rows]),
    warmed_process_timing=timing([r['process_seconds'] for r in rows[11:]]),
    SDK_temporal_per_call_timing=timing(filter_durations),
    filter_instances=len(filters),filter_call_counts=[f.calls for f in filters],
    first_active_index=next((r['index'] for r in rows if r['active']),None),
    active_frames=sum(r['active'] for r in rows),fully_active_frames=sum(r['fully_active'] for r in rows),
    all_input_unchanged=all(r['input_unchanged'] for r in rows),all_current_zero_preserved=all(r['current_zero_preserved'] for r in rows),
    RSS_peak_before_kib=rss_before,RSS_peak_after_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    limitations=['Reuses one retained actual native RGB image for all frames; synthetic uniform alternating depth and moving zero masks. This validates SDK/software history behavior and bounded processing, not real-scene depth quality.',
        'SDK source timestamps and host time advance together virtually at30FPS so faster-than-real-time offline execution does not falsify the source gate. Wall processing time is measured separately.',
        'Mean/P95/max exclude software frame submission and queue wait; include full Regional module RGB tracking, five SDK filters, compositing and input/zero checks.',
        'No hardware context/pipeline is created. The SDK software device may initialize its own udev monitor; it is not a physical camera connection.'],rows=rows)
assert result['filter_instances']==5 and result['filter_call_counts']==[120]*5
assert result['first_active_index']==11 and result['fully_active_frames']==109
OUT.mkdir(exist_ok=False)
shutil.copy2(__file__,OUT/Path(__file__).name)
(OUT/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(report=str(OUT/'report.json'),sha256=base.sha(OUT/'report.json'),
    process_timing=result['process_timing'],warmed=result['warmed_process_timing'],
    SDK_per_call=result['SDK_temporal_per_call_timing'],filter_instances=result['filter_instances'],
    first_active_index=result['first_active_index'],fully_active_frames=result['fully_active_frames'],
    RSS_peak_before_kib=result['RSS_peak_before_kib'],RSS_peak_after_kib=result['RSS_peak_after_kib'])))
