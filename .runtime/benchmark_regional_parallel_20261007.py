#!/usr/bin/env python3
"""Bounded serial/three-thread software-SDK benchmark, no physical device/ROS."""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import resource
import sys
import time

import cv2
import numpy as np

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))
import pyrealsense2 as rs
from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth
from alicia_flexible_grasp.vision import regional_stationary_depth, visual_stationary
from regional_parallel_prototype_20261007 import ParallelRegionalPrototype, ORIGINAL_PROCESS_SOURCE


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def blocked(*args, **kwargs):
    raise AssertionError('physical_context_pipeline_or_enumeration_forbidden')


def stats(values):
    a = np.asarray(values)*1000
    return dict(count=len(a), mean_ms=float(np.mean(a)),
                **dict(zip(('min_ms','p50_ms','p95_ms','max_ms'),
                           np.percentile(a,[0,50,95,100]).tolist())))


class MemoryDepthSource:
    def __init__(self):
        self.device = rs.software_device()
        self.sensor = self.device.add_sensor('offline_parallel_regions_memory_only')
        stream = rs.video_stream()
        stream.type = rs.stream.depth; stream.index=0; stream.uid=9107
        stream.width=640; stream.height=480; stream.fps=30
        stream.bpp=2; stream.fmt=rs.format.z16
        k=rs.intrinsics(); k.width=640; k.height=480
        k.fx=k.fy=400.; k.ppx=320.; k.ppy=240.; k.coeffs=[0.]*5
        k.model=rs.distortion.none; stream.intrinsics=k
        self.profile=self.sensor.add_video_stream(stream).as_video_stream_profile()
        self.sensor.add_read_only_option(rs.option.depth_units,.0001)
        self.queue=rs.frame_queue(1)
        self.sensor.open(self.profile); self.sensor.start(self.queue)

    def frame(self, pixels, source):
        incoming=rs.software_video_frame(); incoming.pixels=pixels
        incoming.bpp=2; incoming.stride=pixels.shape[1]*2
        incoming.timestamp=source['device_timestamp_ms']
        incoming.domain=getattr(rs.timestamp_domain,source['timestamp_domain'].rsplit('.',1)[-1])
        incoming.frame_number=source['frame_number']; incoming.profile=self.profile
        incoming.depth_units=.0001
        self.sensor.on_video_frame(incoming)
        return self.queue.wait_for_frame(1000), incoming

    def close(self):
        self.sensor.stop(); self.sensor.close()


class Factory:
    def __init__(self):
        self.created_count=0

    def __call__(self, *, alpha, delta, persistence):
        assert (alpha,delta,persistence)==(.4,100,0)
        f=rs.temporal_filter()
        f.set_option(rs.option.filter_smooth_alpha,alpha)
        f.set_option(rs.option.filter_smooth_delta,float(delta))
        f.set_option(rs.option.holes_fill,float(persistence))
        # Do not retain historical filters: production releases each old
        # regional history on reset. Retention changes memory pressure/timing.
        self.created_count+=1
        return f


def load_rgb(folder):
    summary=json.loads((folder/'summary.json').read_text())
    assert summary['sealed'] and not summary['errors'] and summary['diagnostic_only']
    assert not summary['pixels_changed'] and not summary['motion_authorized']
    by_source={}; files=[]
    for entry in summary['files']:
        path=folder/entry['name']; assert sha(path)==entry['sha256']
        with np.load(path,allow_pickle=False) as archive:
            meta=json.loads(archive['metadata_json'].tobytes())
            assert meta['coordinate_domain']=='raw_sdk_color_before_pair_rectification'
            for role,name,source,host in (
                ('current','raw_color',meta['source'],meta['filter_host_monotonic']),
                ('anchor','anchor_raw_color',meta['pre_observe']['anchor_source'],
                 meta['pre_observe']['anchor_filter_host_monotonic'])):
                color=archive[name].copy(); assert color.shape==(480,640,3) and color.dtype==np.uint8
                key=(source['frame_number'],source['device_timestamp_ms'])
                value=dict(color=color,source=source,host=host,sha256=array_sha(color),
                           file=entry['name'],role=role)
                if key in by_source:
                    assert value['sha256']==by_source[key]['sha256']
                    assert value['source']==by_source[key]['source'] and host==by_source[key]['host']
                else:
                    color.setflags(write=False); by_source[key]=value
        files.append(dict(name=entry['name'],sha256=entry['sha256'],category=entry['category']))
    return sorted(by_source.values(),key=lambda x:x['source']['device_timestamp_ms']),files


def sequences(records):
    exact=[dict(color=r['color'],source=deepcopy(r['source']),host=r['host'],
                rgb_sha256=r['sha256'],provenance='exact_saved_raw_source',
                encoder=True,aligned=True) for r in records]
    derived=[]
    # Actual RGB pixels are held for sixteen virtual frames per saved source.
    # This explicitly synthetic source schedule exercises genuine SDK filter
    # warmup/qualification. It is NOT reconstruction of unsaved camera frames.
    index=0
    for record in records:
        for repeat in range(16):
            derived.append(dict(color=record['color'],source=dict(frame_number=1000+index,
                device_timestamp_ms=1000+index*1000/30, timestamp_domain='hardware_clock',stream_fps=30.),
                host=1000+index/30, rgb_sha256=record['sha256'],
                provenance='real_saved_rgb_virtual_source_and_synthetic_depth',encoder=True,aligned=True))
            index+=1
    # Match unchanged original source, ownership and zero permission barriers.
    faults=[]
    for i in range(36):
        source=dict(frame_number=2000+i,device_timestamp_ms=5000+i*1000/30,
                    timestamp_domain='hardware_clock',stream_fps=30.)
        if i==14: source['frame_number']+=1
        if i==19: source['device_timestamp_ms']+=100
        faults.append(dict(color=records[0]['color'],source=source,host=2000+i/30,
                           rgb_sha256=records[0]['sha256'],provenance='explicit_synthetic_gate_fault',
                           encoder=i!=24,aligned=i!=29))
    return dict(exact_saved_sources=exact,derived_continuous_warmup=derived,derived_gate_faults=faults)


def run_variant(name, sequence, depth_source):
    factory=Factory()
    engine=RegionalStationaryDepth(factory) if name=='serial' else ParallelRegionalPrototype(factory,3)
    yy,xx=np.indices((480,640))
    fixed=(yy%13==0)&(xx%17==0)
    rows=[]
    try:
        for i,item in enumerate(sequence):
            pixels=np.full((480,640),2000+50*(i%2),np.uint16)
            pixels[fixed]=0; pixels[:,(i*7)%640]=0
            original=pixels.copy()
            frame,ref=depth_source.frame(pixels,item['source'])
            before=(time.perf_counter(),time.process_time(),time.thread_time())
            output,report=engine.process(frame,item['color'],item['source'],item['host'],
                                         item['encoder'],aligned_to_color=item['aligned'])
            elapsed=(time.perf_counter()-before[0],time.process_time()-before[1],time.thread_time()-before[2])
            assert np.array_equal(np.asanyarray(frame.get_data()),original)
            assert np.array_equal(pixels,original)
            assert np.all(output[original==0]==0)
            assert not np.shares_memory(output,np.asanyarray(frame.get_data()))
            assert array_sha(item['color'])==item['rgb_sha256']
            rows.append(dict(index=i,wall_sec=elapsed[0],process_cpu_sec=elapsed[1],
                caller_thread_cpu_sec=elapsed[2],output_sha256=array_sha(output),report=report,
                jobs=[] if name=='serial' else deepcopy(engine.last_jobs)))
            del frame,ref,output
    finally:
        if name=='serial':engine.reset()
        else:engine.close()
    return rows


def summarize(rows):
    return dict(frames=len(rows),wall=stats([r['wall_sec']for r in rows]),
        process_cpu=stats([r['process_cpu_sec']for r in rows]),
        caller_thread_cpu=stats([r['caller_thread_cpu_sec']for r in rows]),
        active_frames=sum(r['report']['active']for r in rows),
        center_reasons=dict(Counter(r['report']['regions'][0]['visual']['reason']
            if r['report']['regions'] else r['report']['reason']for r in rows)))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--capture',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--rounds',type=int,default=4)
    args=ap.parse_args()
    assert 1<=args.rounds<=4
    assert rs.__version__=='2.56.5'
    assert Path(rs.__file__).resolve()==(W/'.runtime/realsense-2.56.5/pyrealsense2.cpython-38-x86_64-linux-gnu.so').resolve()
    mount=Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    args.output.mkdir(parents=True,exist_ok=False)
    rs.context=blocked; rs.pipeline=blocked
    cv2.setNumThreads(1)
    records,files=load_rgb(args.capture/'visual_snapshots')
    inputs=sequences(records)
    source_files=[Path(__file__),W/'.runtime/regional_parallel_prototype_20261007.py',
                  Path(regional_stationary_depth.__file__),Path(visual_stationary.__file__),
                  Path(rs.__file__),W/'.runtime/realsense-2.56.5/lib/librealsense2.so.2.56.5']
    frozen={str(p):sha(p)for p in source_files}
    report=dict(schema=1,scope='offline_parallel_regions_equivalence_and_timing',
        sdk_version=rs.__version__,sdk_path=rs.__file__,python=sys.executable,
        hardware_context_and_pipeline_blocked=True,ros_io=False,robot_commands_sent=False,
        real_depth=False,physical_camera_alignment_claimed=False,
        opencv_version=cv2.__version__,opencv_threads=cv2.getNumThreads(),workers=3,
        source_sha256=frozen,original_process_source_sha256=hashlib.sha256(ORIGINAL_PROCESS_SOURCE.encode()).hexdigest(),
        input_summary_sha256=sha(args.capture/'visual_snapshots/summary.json'),input_files=files,
        unique_rgb_sources=[{k:v for k,v in r.items()if k!='color'}for r in records],
        rounds=[],all_pixels_and_reports_exact=True,complete=False,
        limitations=[
            'Software SDK depth pixels are synthetic alternating surfaces with persistent and moving zero masks, not measured target depth.',
            'Exact-source sequence contains only retained anchor/current RGB frames and real gaps. No missing camera frame is interpolated or fabricated.',
            'Continuous warmup schedule explicitly reuses saved RGB with virtual 30Hz source and host clocks; it tests equivalent algorithm state, not real sensor stability.',
            'Timer excludes source file I/O, software-frame injection, output hashing and assertion comparison; includes complete regional processing.',
            'Process CPU includes all process threads; worker-local CPU is separately measured. This does not prove real-time ROS scheduling, USB recovery or all source gaps solved.',
            'Parallel failure jobs are all joined before outer reset; factory identity check is serialized. This remains an offline prototype, not deployed code.'])
    src=MemoryDepthSource()
    started=time.monotonic();rss_before=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    try:
        for round_id in range(args.rounds):
            ordering=('serial','parallel')if round_id%2==0 else('parallel','serial')
            for sequence_name,sequence in inputs.items():
                variants={name:run_variant(name,sequence,src)for name in ordering}
                mismatches=[]
                for a,b in zip(variants['serial'],variants['parallel']):
                    if a['output_sha256']!=b['output_sha256']or a['report']!=b['report']:
                        mismatches.append(dict(index=a['index'],serial=a,parallel=b))
                report['all_pixels_and_reports_exact'] &= not mismatches
                report['rounds'].append(dict(round=round_id,sequence=sequence_name,ordering=list(ordering),
                    summary={name:summarize(rows)for name,rows in variants.items()},mismatches=mismatches,
                    variants=variants))
                print(json.dumps(dict(round=round_id,sequence=sequence_name,
                    serial=report['rounds'][-1]['summary']['serial']['wall'],
                    parallel=report['rounds'][-1]['summary']['parallel']['wall'],mismatches=len(mismatches))),flush=True)
        assert all(sha(Path(p))==digest for p,digest in frozen.items())
        report['complete']=True
    except BaseException as exc:
        report['error']=dict(kind=type(exc).__name__,message=str(exc))
        raise
    finally:
        src.close()
        report['elapsed_sec']=time.monotonic()-started
        report['peak_rss_kib_before']=rss_before
        report['peak_rss_kib_after']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        with (args.output/'report.json').open('x')as stream:
            json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(dict(report=str(args.output/'report.json'),sha256=sha(args.output/'report.json'),
                          complete=report['complete'],exact=report['all_pixels_and_reports_exact'])))


if __name__=='__main__':main()
