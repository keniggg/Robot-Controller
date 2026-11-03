#!/usr/bin/env python3
"""Offline OpenCV resource A/B, using frozen images and unchanged visual gates.

Never creates a ROS node or SDK pipeline. Synthetic source clocks deliberately
isolate algorithm performance; reports confer no camera/motion authority.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import resource
import sys
import time
import zipfile
import zlib

import cv2
import numpy as np
from sensor_msgs.msg import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth


def digest(b):return hashlib.sha256(b).hexdigest()
def sha(p):return digest(p.read_bytes())
def stats(a):
    return dict(count=len(a),total=float(sum(a)),mean=float(np.mean(a)),
                p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),
                p99=float(np.percentile(a,99)),maximum=float(max(a)))
def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def load_inputs(external):
    capture=external/'ros_log/camera_stage_capture_20261004T124933Z'
    summary=json.loads((capture/'stages/summary.json').read_text())
    result=json.loads((capture/'result.json').read_text())
    assert result['capture_complete'] and result['stage_summary']['counts']['complete']==12
    sdk=[];provenance=[]
    for n in range(1,13):
        stem=capture/'stages'/('sample_%02d'%n)
        m=json.loads(stem.with_suffix('.json').read_text())
        with np.load(stem.with_suffix('.npz'),allow_pickle=False) as z:
            a=np.ascontiguousarray(z['raw_rgb'])
        assert a.dtype==np.uint8 and a.shape==(480,640,3)
        assert digest(a.tobytes())==m['arrays']['raw_rgb']['sha256']
        assert m['array_coordinate_spaces']['raw_rgb']=='native SDK color intrinsics'
        sdk.extend([a]*12)
        provenance.append(dict(path=str(stem.with_suffix('.npz')),file_sha256=sha(stem.with_suffix('.npz')),
            array_sha256=digest(a.tobytes()),original_sdk_color=m['sdk_frames']['color'],
            coordinate_space=m['array_coordinate_spaces']['raw_rgb']))
    gen17=external/'ros_log/gen17_support_measurement_20261006/raw_support_v2'
    ss=json.loads((gen17/'summary.json').read_text())
    assert ss['archive_closed'] and ss['complete_without_writer_error']
    images=[];inputs=[]
    with zipfile.ZipFile(gen17/'diagnostic.zip') as z:
        ns=sorted(n for n in z.namelist() if n.startswith('pairs/') and n.endswith('_color.rosmsg.zlib'))
        for name in ns:
            b=zlib.decompress(z.read(name));m=Image().deserialize(b)
            stamp=int(name.split('/')[1].split('_')[0]);assert stamp==m.header.stamp.to_nsec()
            assert m.encoding=='bgr8'
            a=np.ascontiguousarray(np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3))
            images.append(a)
            inputs.append(dict(archive_entry=name,ros_message_sha256=digest(b),pixel_sha256=digest(a.tobytes()),source_stamp_ns=stamp,shape=list(a.shape)))
    assert len(images)==74
    return [dict(name='sdk_original_12_images_each_repeated12',images=sdk,provenance=provenance,
                 caveat='Original 640x480 pre-rectification SDK RGB; each sparse captured image repeated 12 times to form a fixed bounded workload. Synthetic source clock, not original SDK timing replay.'),
            dict(name='gen17_published_rgb_sequence_repeated2',images=images*2,provenance=inputs,
                 caveat='Original 653x489 rectified published RGB, not pre-rectification SDK camera input; performance analogy only. Real image sequence under synthetic continuous source clocks, not a failed-frame exact replay.')]


def run(images,threads):
    cv2.setNumThreads(threads)
    assert cv2.getNumThreads()==threads
    regional=RegionalStationaryDepth(lambda **kw:None)
    regional._geometry(images[0].shape[:2])
    outputs=[];wall=[];cpu=[];thread_cpu=[]
    before=resource.getrusage(resource.RUSAGE_SELF)
    for i,image in enumerate(images):
        assert image.shape[:2]==regional._shape
        source=dict(frame_number=i,device_timestamp_ms=1000.+i*1000./30.,
                    timestamp_domain='hardware_clock',stream_fps=30.)
        t=time.perf_counter(); c=time.process_time(); ct=time.thread_time()
        reports=[]
        for region in regional._regions:
            x0,y0,x1,y1=region['bounds']
            crop=np.ascontiguousarray(image[y0:y1,x0:x1])
            reports.append(dict(name=region['name'],report=region['visual'].observe(crop,source,10.+i/30.)))
        thread_cpu.append((time.thread_time()-ct)*1000.)
        cpu.append((time.process_time()-c)*1000.)
        wall.append((time.perf_counter()-t)*1000.)
        outputs.append(reports)
    after=resource.getrusage(resource.RUSAGE_SELF)
    flattened=[r['report'] for frame in outputs for r in frame]
    return dict(opencv_threads=cv2.getNumThreads(),frames=len(images),regional_windows_per_frame=5,
        wall_ms=stats(wall),process_cpu_ms=stats(cpu),caller_thread_cpu_ms=stats(thread_cpu),
        cpu_per_wall=sum(cpu)/sum(wall),involuntary_context_switches=after.ru_nivcsw-before.ru_nivcsw,
        voluntary_context_switches=after.ru_nvcsw-before.ru_nvcsw,
        qualified_region_observations=sum(bool(r['qualified']) for r in flattened),
        reason_counts=dict(Counter(r['reason'] for r in flattened)),
        report_sha256=digest(canonical(outputs)),
        sample_wall_ms=wall,sample_process_cpu_ms=cpu,outputs=outputs)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    ext=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and ext in a.output.resolve().parents
    assert not a.output.exists();a.output.mkdir(parents=True)
    default_threads=cv2.getNumThreads()
    data=load_inputs(ext)
    # Warm backend execution equally; measurements do not include disk I/O.
    for t in (12,1):run([data[0]['images'][0]]*16,t)
    report=dict(schema=1,scope='offline_fixed_visual_gate_resource_AB',production_files_changed=False,
        ROS_node_started=False,hardware_opened=False,opencv_version=cv2.__version__,opencv_module=cv2.__file__,
        fresh_process_default_threads=default_threads,opencv_cpus=cv2.getNumberOfCPUs(),
        python=sys.executable,environment={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__).resolve(),
            ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py',
            ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py']},
        order=[12,1,1,12],datasets=[],limitations=[
            'No SDK alignment/base filters/temporal depth filters/publisher or live transport included.',
            'Synthetic monotonic/device source clock isolates numeric workload; elapsed benchmark wall time is never used as a camera qualification.',
            'Offline A/B cannot establish the cause of gen18 source deadline or prove transport recovery.'])
    for dataset in data:
        entry={k:v for k,v in dataset.items() if k!='images'};entry['blocks']=[]
        for block,threads in enumerate(report['order']):
            r=run(dataset['images'],threads)
            out=a.output/('%s_block%d_threads%d.json'%(dataset['name'],block,threads))
            out.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
            entry['blocks'].append({**{k:v for k,v in r.items() if k not in ('outputs','sample_wall_ms','sample_process_cpu_ms')},'artifact':out.name,'artifact_sha256':sha(out)})
            print(json.dumps(dict(dataset=dataset['name'],block=block,threads=threads,
                wall_p50_ms=r['wall_ms']['p50'],wall_p95_ms=r['wall_ms']['p95'],cpu_p50_ms=r['process_cpu_ms']['p50'],
                report_sha256=r['report_sha256']),ensure_ascii=False),flush=True)
        entry['all_numeric_reports_bitwise_equal']=len({b['report_sha256'] for b in entry['blocks']})==1
        report['datasets'].append(entry)
    report['numeric_equivalence_passed']=all(x['all_numeric_reports_bitwise_equal'] for x in report['datasets'])
    report['complete']=True
    (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(complete=True,numeric_equivalence_passed=report['numeric_equivalence_passed'],report=str(a.output/'report.json'))),flush=True)

if __name__=='__main__':main()
