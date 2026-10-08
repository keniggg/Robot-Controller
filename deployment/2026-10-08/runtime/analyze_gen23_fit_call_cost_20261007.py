#!/usr/bin/env python3
"""Offline call counts on the original gen23 pipeline; no ROS/device imports."""
import argparse
from dataclasses import fields
import importlib.util
import json
from pathlib import Path
import time
from types import SimpleNamespace
import zipfile
import numpy as np

W=Path(__file__).resolve().parents[1]
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_20261007T112309.952794Z/recorded_retry_20261007T120118Z/raw_support')
spec=importlib.util.spec_from_file_location('offline_depth_decoder',W/'.runtime/analyze_current_support_rgbd_20261007.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a);t=a.table


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.output)
    if Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller') not in out.resolve().parents or out.exists():raise ValueError('new_external_output_required')
    report=json.loads((C/'analysis/report.json').read_text())
    if report['production_tabletop_sha256']!=a.sha(Path(t.__file__).read_bytes()):raise ValueError('production_changed')
    result=dict(diagnostic_only=True,production_modified=False,motion_authorized=False,
                source_sha256=a.sha(Path(__file__).read_bytes()),production_tabletop_sha256=report['production_tabletop_sha256'],
                replay_report_sha256=a.sha((C/'analysis/report.json').read_bytes()),rows=[],
                limitations=['Offline calls reconstruct immutable anchor and last position from the exact recorded reference source status.',
                    'No tracker state commit or RGB recovery replay; returned candidates are not authorized target geometry.',
                    'Nested timings are inclusive; do not sum parent and child timings.',
                    'Timing uses current host load, not original live execution.'])
    with zipfile.ZipFile(C/'diagnostic.zip') as z:
        es={v['name']:v for v in json.loads(z.read('entry_manifest.json'))}
        def read(n):
            b=z.read(n)
            if len(b)!=es[n]['bytes'] or a.sha(b)!=es[n]['sha256']:raise ValueError('archive_hash:'+n)
            return b
        meta=json.loads(read(next(n for n in es if n.startswith('metadata/'))))['parameters'];cam=meta['/camera'];rawcfg=meta['/unknown_perception'];K=[cam[k] for k in ('fx','fy','cx','cy')]
        cfg=t.Config(**{f.name:rawcfg[f.name] for f in fields(t.Config) if f.name in rawcfg})
        statuses={}
        for n in es:
            if n.startswith('status/'):
                s=json.loads(json.loads(read(n))['original_status_data'])
                if s.get('generation')==23 and s.get('source_stamp_ns') is not None:statuses[s['source_stamp_ns']]=s
        names=['_fit_plane','_fit_support_ring_sequence','_audit_current_support','segment','_project','_project_pixels','_depth_components']
        originals={name:getattr(t,name) for name in names};events=[]
        def wrap(name):
            def traced(*args,**kwargs):
                before=time.perf_counter();cpu=time.thread_time();event={'name':name}
                if name=='_fit_plane':event.update(point_count=len(args[0]),points_sha256=a.sha(np.asarray(args[0],dtype=float).tobytes()))
                if name=='segment':event['fit_current_support']=kwargs.get('fit_current_support',True)
                try:return originals[name](*args,**kwargs)
                except Exception as exc:event['exception']=str(exc);raise
                finally:
                    event.update(wall_sec=time.perf_counter()-before,thread_cpu_sec=time.thread_time()-cpu);events.append(event)
            return traced
        for name in names:setattr(t,name,wrap(name))
        try:
            for r in report['pending_replay']:
                ns=r['source_stamp_ns'];s=statuses[ns];refsource=s['source_qualification']['reference_source_stamp_ns'];laststatus=statuses[refsource]
                if laststatus['state']!='ready' or laststatus['target_lost']:raise ValueError('last_good_reference_not_ready')
                item=json.loads(read('pairs/%s.json'%ns))['streams']['depth'];raw=a.bounded_original_message(read('pairs/%s_depth.rosmsg.zlib'%ns),item['serialized_bytes'])
                if a.sha(raw)!=item['serialized_sha256']:raise ValueError('depth_hash')
                header,pixels=a.decode_image(raw)
                if header['source_stamp_ns']!=ns:raise ValueError('source_mismatch')
                depth=pixels.astype(np.float32)*float(cam['depth_scale']);T=np.asarray(r['tf_matrix'])
                anchor=SimpleNamespace(position_base=np.asarray(s['anchor_position_base']),extent_m=np.asarray(s['anchor_extent_m']),support_footprint_base=np.asarray(s['reference_support_footprint_base']))
                last=SimpleNamespace(position_base=np.asarray(laststatus['position_base']))
                row=dict(source_stamp_ns=ns,status_entry=r['status_entry'],reference_source_stamp_ns=refsource,repeats=[]);result['rows'].append(row)
                for _ in range(3):
                    events.clear();w=time.perf_counter();cpu=time.thread_time()
                    seg=t.segment_with_support_fallback(depth,K,T,cfg,anchor,last,np.asarray(s['reference_support_plane_base']))
                    row['repeats'].append(dict(wall_sec=time.perf_counter()-w,thread_cpu_sec=time.thread_time()-cpu,
                        candidates=len(seg.candidates),eligible_candidates=len(t._matching_candidates(seg.candidates,anchor,last,cfg)),
                        metrics=seg.metrics,events=list(events)))
                print(json.dumps({'source':ns,'fit_calls':sum(e['name']=='_fit_plane' for e in row['repeats'][0]['events']),
                                  'segment_calls':sum(e['name']=='segment' for e in row['repeats'][0]['events']),
                                  'eligible':row['repeats'][0]['eligible_candidates']}),flush=True)
        finally:
            for name,original in originals.items():setattr(t,name,original)
    out.mkdir(parents=True,exist_ok=False);b=a.encoded(result);(out/'report.json').write_bytes(b);(out/'SHA256SUMS').write_text(a.sha(b)+'  report.json\n');print(out,a.sha(b))


if __name__=='__main__':main()
