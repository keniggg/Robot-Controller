#!/usr/bin/env python3
"""Six sequential short isolated copy replays, no ROS nodes or live connections."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

HERE=Path(__file__).resolve().parent
MOUNT=Path('/media/zhuyupei/SanStand')
EXT=MOUNT/'实验记录/Robot-Controller/ros_log'
CANDIDATE_SHA='c461601a31e4120f03e01305450f4df9740a2c467ea134717a2c6e87fbc3a0c3'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();out=a.output.resolve()
    if not MOUNT.is_mount() or EXT not in out.parents or out.exists():raise ValueError('New mounted external output required')
    candidate=HERE/'unknown_pruned_copy_candidate_20261008.py';replay=HERE/'replay_unknown_receive_copy_20261008.py'
    if sha(candidate)!=CANDIDATE_SHA:raise ValueError('Frozen candidate changed')
    sources={str(p):sha(p) for p in (candidate,replay,Path(__file__))}
    out.mkdir();(out/'sources').mkdir()
    for p,d in sources.items():
        dst=out/'sources'/Path(p).name;shutil.copyfile(p,dst)
        if sha(dst)!=d:raise ValueError('Source archive mismatch')
    cases=[dict(name='normal_color_one',wire=958007,count=1,prefix=0,tail=0),
           dict(name='depth_nine_actual_fragment',wire=638691,count=9,prefix=425176,tail=0),
           dict(name='color_six_actual_partial',wire=958007,count=6,prefix=261932,tail=458381)]
    results=[];started=time.monotonic();error=None
    try:
        for case in cases:
            outputs={}
            for variant in ('original','candidate'):
                for p,d in sources.items():
                    if sha(p)!=d:raise ValueError('Source changed before benchmark')
                output=out/(case['name']+'_'+variant+'.json')
                cmd=['/usr/bin/python3',str(replay),'--wire-bytes',str(case['wire']),
                     '--complete-messages',str(case['count']),'--prefix-bytes',str(case['prefix']),
                     '--tail-bytes',str(case['tail']),'--iterations','5','--deadline-sec','3','--output',str(output)]
                if variant=='candidate':cmd+=['--candidate',str(candidate)]
                before=time.monotonic()
                proc=subprocess.run(cmd,capture_output=True,text=True,timeout=8,
                    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
                (out/(output.stem+'.log')).write_text(proc.stdout+proc.stderr)
                if proc.returncode:raise RuntimeError(variant+' returned '+str(proc.returncode)+': '+proc.stderr)
                data=json.loads(output.read_text())
                outputs[variant]=dict(file=output.name,sha256=sha(output),elapsed_monotonic_sec=time.monotonic()-before,data=data)
            left=outputs['original']['data'];right=outputs['candidate']['data']
            proofs={r['output_sha256'] for d in (left,right) for r in d['rows']}
            equivalent=(len(proofs)==1 and left['actual_iterations']==right['actual_iterations']==5)
            if not equivalent:raise RuntimeError('Original/candidate output difference or partial iteration count')
            stats=right['timed_candidate_statistics']
            expected_skip=max(0,case['count']-2)*(case['wire']-4)*5
            if stats['skipped_payload_bytes']!=expected_skip or stats['calls_failed'] or stats['fallback_calls'] or stats['optimized_calls']!=5:
                raise RuntimeError('Candidate deterministic copy counters differ')
            results.append(dict(case=case,output_equivalent=True,expected_skipped_payload_bytes_per_timed_call=expected_skip//5,variants=outputs))
    except BaseException as exc:error=repr(exc)
    report=dict(schema=1,kind='bounded_synthetic_copy_comparison',source_sha256=sources,
        source_unchanged=all(sha(p)==d for p,d in sources.items()),started_monotonic=started,finished_monotonic=time.monotonic(),
        valid=error is None and len(results)==3,error=error,cases=results,
        input_payload_cap_bytes=16*1024*1024,largest_input_batch_bytes=max(c['wire']*c['count']+c['tail'] for c in cases),
        actual_original_recv_capacity=16*1024*1024,
        no_original_socket_allocation_or_live_pressure=True,
        limitations=['Each variant runs in a separate Python process, sequentially; first sample and following four are retained separately.',
            'Input batch is capped at16MiB, not total process RSS. Current and high-water process RSS and separate allocation probe are reported by each child.',
            'Fixed original-then-candidate order and five samples are a bounded microbenchmark, not randomized performance causality proof.',
            'Original and candidate retain the same final messages/partial bytes. Only deterministic skipped body-copy bytes are exact savings evidence.',
            'The candidate changes deserialization materialization only; it does not optimize preceding BytesIO.write or prove the multi-second problem resolved.'],
        problem_resolved=False)
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    paths=[p for p in out.iterdir() if p.is_file()]
    (out/'SHA256SUMS').write_text(''.join(sha(p)+'  '+p.name+'\n' for p in sorted(paths)))
    print(json.dumps(dict(output=str(out),valid=report['valid'],summary_sha256=sha(out/'summary.json'),error=error)))
    return 0 if report['valid'] else 2


if __name__=='__main__':raise SystemExit(main())
