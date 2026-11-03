#!/usr/bin/env python3
"""Pure array regional compositing benchmark; no ROS/SDK/device initialization.

Profiles only unchanged output copy/compositing/counting after original SDK
filters have returned. It cannot prove end-to-end camera throughput or fixes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import tracemalloc

import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision.regional_stationary_depth import RegionalStationaryDepth
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SOURCE=EXT/'ros_log/matched_shadow_current_20261005T024652.714830Z/stages/sample_11'
VARIANTS=('original','cached_counts','cached_counts_copyto')
SCENARIOS={'none_active':(False,)*5,'center_only':(True,False,False,False,False),
           'two_quadrants':(False,True,False,False,True),'all_active':(True,)*5}

def sha(raw):return hashlib.sha256(raw).hexdigest()
def array_sha(array):return sha(np.ascontiguousarray(array).tobytes())


def regions(shape):
    engine=RegionalStationaryDepth(lambda **kwargs:None)
    engine._geometry(shape)
    rows=[]
    for region in engine._regions:
        rows.append(dict(name=region['name'],bounds=region['bounds'],owned=region['owned'],
                         owned_pixels=int(np.count_nonzero(region['owned']))))
    return rows


def composite(current,filtered_arrays,rows,active,variant):
    if variant not in VARIANTS:raise ValueError('unsupported_variant')
    output=current.copy(); qualified=0; counts=[]
    for region,filtered,enabled in zip(rows,filtered_arrays,active):
        x0,y0,x1,y1=region['bounds']
        if enabled:
            patch=output[y0:y1,x0:x1]; source=filtered[y0:y1,x0:x1]
            if variant=='cached_counts_copyto':np.copyto(patch,source,where=region['owned'])
            else:patch[region['owned']]=source[region['owned']]
            qualified+=(int(np.count_nonzero(region['owned'])) if variant=='original' else region['owned_pixels'])
        counts.append(int(np.count_nonzero(region['owned'])) if variant=='original' else region['owned_pixels'])
    report=dict(region_owned_pixels=counts,filtered_pixel_count=qualified,
                fully_active=qualified==current.size,active=bool(qualified),
                changed_pixel_count=int(np.count_nonzero(output!=current)),
                filtered_fraction=qualified/float(current.size))
    return output,report


def stats(values):
    a=np.asarray(values)*1000
    return dict(count=len(values),mean_ms=float(a.mean()),p50_ms=float(np.median(a)),
                p95_ms=float(np.percentile(a,95)),minimum_ms=float(a.min()),maximum_ms=float(a.max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output.resolve()
    if not Path('/media/zhuyupei/SanStand').is_mount() or EXT.resolve() not in output.parents:
        raise ValueError('new SanStand output required; no fallback')
    if output.exists():raise ValueError('new output only')
    metadata=json.loads(SOURCE.with_suffix('.json').read_text())
    with np.load(SOURCE.with_suffix('.npz'),allow_pickle=False) as data:
        current=data['pre_stationary_depth'];filtered=data['matched_shadow_depth']
    for name,array in (('pre_stationary_depth',current),('matched_shadow_depth',filtered)):
        if (array.shape!=(480,640) or array.dtype!=np.uint16
                or array_sha(array)!=metadata['arrays'][name]['sha256']):raise ValueError('saved_array_source_mismatch')
    current.setflags(write=False);filtered.setflags(write=False)
    rows=regions(current.shape)
    # These are saved measured input and saved software-shadow filtered arrays.
    # Alternating two real arrays exercises distinct owner outputs without
    # claiming five archived independent histories for this same frame.
    arrays=[filtered,current,filtered,current,filtered]
    report=dict(schema=1,scope='pure NumPy composition/counting microbenchmark only',
        no_ros_or_hardware_or_sdk_initialization=True,production_modified=False,
        source_metadata=str(SOURCE.with_suffix('.json')),source_metadata_sha256=sha(SOURCE.with_suffix('.json').read_bytes()),
        arrays_sha256={name:metadata['arrays'][name]['sha256'] for name in ('pre_stationary_depth','matched_shadow_depth')},
        source_sha256={str(p):sha(p.read_bytes()) for p in (Path(__file__),ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py')},
        python=sys.version,numpy=np.__version__,shape=list(current.shape),
        geometry=[dict(name=r['name'],bounds=list(r['bounds']),owned_pixels=r['owned_pixels']) for r in rows],
        rounds=6,iterations_per_variant_per_round=80,timing_includes='output current.copy, owner compositing, counts, and final changed-pixel count',
        timing_excludes='source I/O, SDK temporal filtering, RGB/LK, mutation/hole checks, report-copy wrappers',
        scenarios={},limitations=[
            'Magnitude is only the isolated compositing subset, not full 13-16ms regional processing.',
            'Two actual saved arrays alternate between owners; this is not an independent 5-history SDK replay.',
            'tracemalloc reports tracked Python/NumPy peak allocations, not SDK/native allocator/RSS.',
            'No source/history/visual/support threshold, ownership geometry, filter call count or SDK pixel rule is changed by the candidate expressions.',
            'Actual production equivalence still requires full original-report/pixel regression before any deployment.'])
    for name,active in SCENARIOS.items():
        expected,original_report=composite(current,arrays,rows,active,'original')
        series={variant:[] for variant in VARIANTS};cpus={variant:[] for variant in VARIANTS}
        peaks={}
        for variant in VARIANTS:
            actual,counts=composite(current,arrays,rows,active,variant)
            assert np.array_equal(actual,expected) and counts==original_report
            assert not np.shares_memory(actual,current)
            for _ in range(5):composite(current,arrays,rows,active,variant)
            allocations=[]
            for _ in range(5):
                tracemalloc.start();out,counts=composite(current,arrays,rows,active,variant)
                allocations.append(tracemalloc.get_traced_memory()[1]);tracemalloc.stop();del out
            peaks[variant]=dict(min_peak_bytes=min(allocations),median_peak_bytes=int(np.median(allocations)),maximum_peak_bytes=max(allocations))
        for round_id in range(6):
            order=VARIANTS[round_id%3:]+VARIANTS[:round_id%3]
            if round_id%2:order=tuple(reversed(order))
            for variant in order:
                start=time.perf_counter();cpu=time.process_time()
                for _ in range(80):out,counts=composite(current,arrays,rows,active,variant)
                cpus[variant].append((time.process_time()-cpu)/80)
                series[variant].append((time.perf_counter()-start)/80)
                assert np.array_equal(out,expected) and counts==original_report
        report['scenarios'][name]=dict(active=list(active),original_report=original_report,
            all_outputs_and_report_counts_exact=True,output_sha256=array_sha(expected),
            wall_batch_per_call={variant:stats(values) for variant,values in series.items()},
            process_cpu_batch_per_call={variant:stats(values) for variant,values in cpus.items()},
            tracked_allocation_peaks=peaks)
    assert array_sha(current)==metadata['arrays']['pre_stationary_depth']['sha256']
    assert array_sha(filtered)==metadata['arrays']['matched_shadow_depth']['sha256']
    report['source_arrays_unchanged']=True
    output.mkdir(parents=True,exist_ok=False)
    raw=(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    (output/'report.json').write_bytes(raw);(output/'SHA256SUMS').write_text(sha(raw)+'  report.json\n')
    print(json.dumps(dict(output=str(output),sha256=sha(raw),scenarios={name:row['wall_batch_per_call']for name,row in report['scenarios'].items()})))


if __name__=='__main__':main()
