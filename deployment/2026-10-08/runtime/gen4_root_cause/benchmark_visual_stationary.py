"""Offline CPU and pixel replay only; no ROS node, camera, or hardware connection."""
import collections
import hashlib
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rosbag
from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
assert Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand unavailable'
OUT = ROOT / 'temporal_review'
OUT.mkdir(exist_ok=True)
image = cv2.imread(str(ROOT / 'j5_negative6_20261003T072610Z/1791012372778900146_rgb.png'))
assert image is not None
result = dict(opencv=cv2.__version__, opencv_threads=cv2.getNumThreads(), input_shape=list(image.shape),
              source_file_sha256=hashlib.sha256(Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py').read_bytes()).hexdigest(),
              no_hardware_or_runtime_changes=True, benchmarks={}, pixel_replays={})
for name in ('static_repeated_real_image', 'synthetic_7_5hz_one_pixel_vibration'):
    window=VisualStationaryWindow(); wall=[]; cpu=[]; process_cpu=[]; reports=[]
    for i in range(450):
        current=image
        if name.startswith('synthetic'):
            current=cv2.warpAffine(image,np.float32([[1,0,0],[0,1,np.sin(np.pi*i/2)]]),
                (image.shape[1],image.shape[0]),borderMode=cv2.BORDER_REFLECT)
        source=dict(frame_number=i,device_timestamp_ms=1000.+i*1000./30.,timestamp_domain='hardware_clock',stream_fps=30.)
        t=time.perf_counter(); c=time.thread_time(); pc=time.process_time()
        report=window.observe(current,source,10.+i/30.)
        process_cpu.append((time.process_time()-pc)*1000.); cpu.append((time.thread_time()-c)*1000.); wall.append((time.perf_counter()-t)*1000.)
        reports.append(report)
    result['benchmarks'][name]=dict(count=len(wall),qualified=sum(r['qualified'] for r in reports),
        wall_ms=dict(p50=float(np.percentile(wall,50)),p95=float(np.percentile(wall,95)),max=max(wall)),
        caller_thread_cpu_ms=dict(p50=float(np.percentile(cpu,50)),p95=float(np.percentile(cpu,95)),max=max(cpu)),
        process_cpu_ms=dict(p50=float(np.percentile(process_cpu,50)),p95=float(np.percentile(process_cpu,95)),max=max(process_cpu)),
        reasons=dict(collections.Counter(r['reason'] for r in reports)))
for session in ('single_axis_20261003T070709Z','j5_negative6_20261003T072610Z'):
    rows=[];window=VisualStationaryWindow()
    with rosbag.Bag(str(ROOT/session/'samples.bag')) as bag:
        for _,msg,receipt in bag.read_messages(topics=['/supervisor/camera/color/image_raw']):
            assert msg.encoding=='bgr8'
            current=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            i=len(rows)
            # Deliberately synthetic nominal 20 Hz identity, only to exercise
            # pixels. Bag headers are publication time, NOT SDK acquisition id.
            info=dict(frame_number=i,device_timestamp_ms=1000.+i*50.,timestamp_domain='hardware_clock',stream_fps=20.)
            report=window.observe(current,info,10.+i*.05)
            rows.append(dict(publication_stamp_ns=msg.header.stamp.to_nsec(),**report))
    result['pixel_replays'][session]=dict(count=len(rows),qualified=sum(r['qualified'] for r in rows),
        reasons=dict(collections.Counter(r['reason'] for r in rows)),
        caveat='Pixel-only test using synthetic consecutive identity/nominal 20Hz. NOT SDK source continuity, NOT valid deployed filter activation evidence.',rows=rows)
(OUT/'visual_stationarity_benchmark_and_pixel_replay.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='pixel_replays'},indent=2))
print(json.dumps({k:{j:v for j,v in value.items() if j!='rows'} for k,value in result['pixel_replays'].items()},indent=2))
