"""Selected exact gen32 sources through final production multi-match arrival."""
from pathlib import Path
import argparse,json,time,hashlib,sys
import numpy as np,cv2
p=Path(__file__).with_name('replay_gen32_arrival_matching.py');g={};exec(compile(p.read_text().split('files=')[0],str(p),'exec'),g)
table=g['table'];cfg=g['cfg'];frame=g['frame'];helper=g['helper'];meta=g['meta'];anchor=g['anchor'];last=g['last'];plane=g['plane']
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path);args=parser.parse_args()
OUT=args.output if args.output is not None else g['D']/'matching_fragments_production_replay'
OUT.mkdir(exist_ok=True);assert not (OUT/'result.json').exists(),'Preserve prior replay evidence'
files=[Path(table.__file__),Path(g['rgbmod'].__file__),Path(g['groupmod'].__file__),Path(sys.modules[type(helper).__module__].__file__)];before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};rows=[]
for row in meta['sources'][2:]:
    a,rgb,dep,T,K,valid=frame(row);ns=row['source_stamp_ns'];start=time.perf_counter();r=dict(source_stamp_ns=ns,exact_online_rejected_source=True)
    try:
        seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
        r['original_matching_count']=len(table._matching_candidates(seg.candidates,anchor,last,cfg))
        confirmed,candidate,staged,evidence=helper.observe(seg,rgb,dep,K,T,ns,rgb_valid=valid)
        r.update(passed=True,confirmed=confirmed,metrics=candidate.metrics,evidence=evidence,raw_points=int(np.count_nonzero(staged.last.mask)),raw_valid_points=staged.last.metrics['valid_points'])
        assert staged.anchor is anchor and np.array_equal(staged.plane_base,plane)
        assert np.count_nonzero(staged.last.mask)==staged.last.metrics['valid_points']
        cv2.imwrite(str(OUT/(str(ns)+'_mask.png')),candidate.mask)
    except ValueError as exc:r.update(passed=False,reason=str(exc))
    r['offline_wall_sec']=time.perf_counter()-start;rows.append(r)
    print(json.dumps({k:r.get(k) for k in ['source_stamp_ns','original_matching_count','passed','confirmed','reason','offline_wall_sec']}),flush=True)
after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};assert before==after,'Production changed during replay'
report=dict(scope='ACTUAL_PRODUCTION_EXACT_RECORDED_SOURCES_OFFLINE_NO_AUTHORITY',record=meta['record'],source_sha256=before,reference_checks=g['checks'],rows=rows,all_passed=all(r['passed'] for r in rows),third_qualified=bool(rows[-1].get('confirmed')),limitations=['Three exact sources selected from four saved arrival RGB-D pairs matching eighteen online statuses; no absent frames reconstructed.','Original full support/RGB/geometry and three-frame confirmation functions; no live timing, consumer ACK, or execution authority proof.'])
(OUT/'result.json').write_text(json.dumps(report,indent=2)+'\n');print('RESULT',OUT/'result.json')
