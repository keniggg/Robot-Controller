"""Later frozen-source measurement replay with unchanged last-good state; no runtime admission claim."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
import argparse
import ast
from types import SimpleNamespace as NS
from dataclasses import asdict, fields
import hashlib
import inspect
import json
from pathlib import Path
import resource
import sys
import time
import traceback

LIVE = Path(__file__).resolve().parents[1]
W = LIVE/'.runtime/gen61_seed_wait_candidate_r1'
S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/actual_attempt_r1')
I = S/'failed_seed_inputs_r2'
T = S/'terminal_evidence_r1'
ANCHOR, FROZEN, QUALIFIED, LAST, FAILED = (1792649183970164060, 1792649207904424190,
    1792649241371229887, 1792649243437337398, 1792649243704485416)
sys.dont_write_bytecode = True
sys.path[:0] = [str(LIVE/'.runtime/realsense-2.56.5'), str(W/'src/alicia_grasp_modes/src'), str(W/'src/alicia_flexible_grasp_supervisor/src')]
denied = []
def guard(event, args):
    if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.posix_spawn'):
        denied.append(event)
        raise RuntimeError('offline owner replay forbids '+event)
sys.addaudithook(guard)
import cv2
import numpy as np
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes import measured_mask_seeds as seeds
from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.rgb_instance import refine_candidate, freeze_observation_spatial_reference
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance, match_observation_appearance
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def binary_sha(x): return hashlib.sha256(np.ascontiguousarray(x>0,dtype=np.uint8).tobytes()).hexdigest()
def plain(x):
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,np.generic): return x.item()
    if isinstance(x,dict): return {str(k):plain(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)): return [plain(v) for v in x]
    return x
def compact(c):
    return dict(position_base=c.position_base,extent_m=c.extent_m,quality=c.quality,
                mask_pixels=int(np.count_nonzero(c.mask)),mask_sha256=binary_sha(c.mask),
                metrics=c.metrics,support_footprint_base=c.support_footprint_base)

def main():
    global FAILED
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--source',type=int,choices=[1792649244237583160,1792649244372374773],required=True);args=ap.parse_args();FAILED=args.source
    out=args.output.resolve();assert S.parent/'seed_wait_fix_r1' in out.parents and Path('/media/zhuyupei/SanStand').is_mount()
    out.mkdir(exist_ok=False);cv2.setNumThreads(1)
    assert np.__version__ == '1.17.4' and cv2.__version__ == '4.2.0'
    assert os.environ.get('PYTHONNOUSERSITE') == '1'
    started=time.monotonic();report=dict(passed=False,scope='Actual recorded later RGBD: exact producer numeric recovery method; clock and ownership boundaries are inert offline inputs, no runtime admission claim',checkpoints=[],seed_calls=[],owner_stages=[])
    def save(name,value):
        p=out/name
        with p.open('x') as f:json.dump(plain(value),f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
        return dict(path=str(p),sha256=sha(p))
    try:
        assert sha(I/'manifest.json')=='874f55c7c11e618aa69e718890649ebfa44b67ec9b2fc288ce890a8a5424f91d'
        manifest=json.loads((I/'manifest.json').read_text());rows={r['source_stamp_ns']:r for r in manifest['rows']}
        statusrows=[json.loads(line) for line in (T/'state_messages.jsonl').read_text().splitlines()]
        statuses={r['data']['source_stamp_ns']:r['data'] for r in statusrows if r['topic']=='/perception/unknown/detector_status' and isinstance(r['data'],dict) and 'source_stamp_ns' in r['data'] and r['data'].get('generation')==4}
        cfgdict=json.loads((I/'unknown_perception_configuration.json').read_text())
        cfg=table.Config(**{f.name:cfgdict[f.name] for f in fields(table.Config) if f.name in cfgdict})
        minimum_quality=float(cfgdict.get('minimum_geometry_quality',.5))
        pins={str(I/'manifest.json'):sha(I/'manifest.json'),str(T/'state_messages.jsonl'):sha(T/'state_messages.jsonl'),str(I/'camera_metadata.json'):sha(I/'camera_metadata.json'),str(I/'unknown_perception_configuration.json'):sha(I/'unknown_perception_configuration.json')}
        def load(stamp):
            row=rows[stamp];assert sha(row['path'])==row['npz_sha256'];pins[row['path']]=row['npz_sha256']
            with np.load(row['path']) as z:d={k:z[k].copy() for k in z.files}
            d['depth']=d['depth_raw'].astype(np.float32)*float(d['depth_scale'])
            assert int(d['source_stamp_ns'])==stamp
            return d
        a=load(ANCHOR);seg=table.segment_with_support_fallback(a['depth'],a['K'],a['T_base_camera'],cfg)
        tracker=table.TargetTracker(cfg);anchor,reason=tracker.choose(seg,a['depth'].shape);assert anchor is not None,reason
        plane=seg.plane_base.copy();recorded=statuses[1792649243704485416]
        comparisons={}
        for key,actual,expected in [('position',anchor.position_base,recorded['anchor_position_base']),('extent',anchor.extent_m,recorded['anchor_extent_m']),('plane',plane,recorded['reference_support_plane_base']),('footprint',anchor.support_footprint_base,recorded['reference_support_footprint_base']),('height',anchor.metrics['median_height_m'],recorded['reference_object_height_m'])]:
            comparisons[key]=dict(max_abs_delta=float(np.max(np.abs(np.asarray(actual)-expected))),equal_at_1e_10=bool(np.allclose(actual,expected,atol=1e-10,rtol=0)))
        report['anchor']=dict(comparisons=comparisons,candidate=compact(anchor),config=asdict(cfg));assert all(x['equal_at_1e_10'] for x in comparisons.values()),'anchor reconstruction mismatch'
        spatial=freeze_observation_spatial_reference(anchor,plane);last=anchor
        for stamp in (FROZEN,QUALIFIED,LAST):
            d=load(stamp)
            seg=table.segment_with_support_fallback(d['depth'],d['K'],d['T_base_camera'],cfg,anchor,last,plane,prefer_current_support=True)
            matches=table._matching_candidates(seg.candidates,anchor,last,cfg);assert len(matches)==1,(stamp,len(matches))
            raw=matches[0][1]
            kw=dict(rgb_valid=d['rgb_valid'])
            if stamp!=FROZEN:kw.update(observation_arrival_retry=True,observation_spatial_reference=spatial)
            refined=refine_candidate(d['color'],d['depth'],d['K'],d['T_base_camera'],raw,cfg,**kw)
            bit_equal=bool(np.array_equal(refined.mask,d['recorded_refined_mask']))
            record=dict(source_stamp_ns=stamp,raw=compact(raw),refined=compact(refined),recorded_mask_bit_equal=bit_equal,recorded_mask_sha256=binary_sha(d['recorded_refined_mask']))
            report['checkpoints'].append(record);print(json.dumps(dict(source_stamp_ns=stamp,recorded_mask_bit_equal=bit_equal)),flush=True)
            assert bit_equal,('recorded mask not reproduced',stamp)
            if stamp==QUALIFIED:
                hist,contour=_appearance(d['color'],_mask(d['recorded_refined_mask'],d['depth'].shape))
                frozen=FrozenObservationAppearance(d['color'],d['recorded_refined_mask'],d['rgb_valid'],stamp,hist,contour)
            if stamp==LAST:last_mask=refined.mask.copy()
            last=raw
        d=load(FAILED)
        seg=table.segment_with_support_fallback(d['depth'],d['K'],d['T_base_camera'],cfg,anchor,last,plane,prefer_current_support=True)
        matches=table._matching_candidates(seg.candidates,anchor,last,cfg);assert len(matches)==1,len(matches)
        raw=matches[0][1]
        refined=refine_candidate(d['color'],d['depth'],d['K'],d['T_base_camera'],raw,cfg,rgb_valid=d['rgb_valid'],observation_arrival_retry=True,observation_spatial_reference=spatial)
        input_array_hashes={k:hashlib.sha256(v.tobytes()).hexdigest() for k,v in d.items() if isinstance(v,np.ndarray)}
        def check_current():
            assert all(hashlib.sha256(d[k].tobytes()).hexdigest()==v for k,v in input_array_hashes.items())
        appearance=match_observation_appearance(d['color'],refined.mask,hist,contour,rgb_valid=d['rgb_valid'],appearance_context=dict(reference=frozen,source_ns=FAILED,check_current=check_current,same_station_mask_iou_minimum=.85,reference_role='qualified_station_identity_template'))
        report['failed_primary']=dict(raw=compact(raw),refined=compact(refined),appearance=appearance,online_comparisons=None,native_candidates=[compact(c) for c in seg.candidates],segmentation_metrics=seg.metrics)
        report['source_stamp_ns']=FAILED
        report['state_policy']='Exact original anchor/current last-good raw geometry and third-qualified appearance are frozen; no failed/later frame updates any authority.'
        report['original_failed_frame_deadlines']=recorded['frame_timing'].get('arrival_frame_completion')
        report['original_observation_request']=recorded['observation_measurement']['request']
        report['temporal_scope']='Only source timestamps are compared. Historical live terminal had already occurred; callback custody/deadline admission is not reconstructed.'
        camera_rows=json.loads((I/'camera_metadata.json').read_text())
        selected=max((r for r in camera_rows if r['bag_receipt_ns']<=FAILED),key=lambda r:r['bag_receipt_ns'])
        camera=selected['data']['parameters']
        assert camera['runtime_profile']['geometry_intrinsics']==rows[FAILED]['camera_geometry']['runtime_profile_geometry']['geometry_intrinsics']
        production=W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py'
        production_sha=sha(production)
        syntax=ast.parse(production.read_text())
        method=next(fn for cls in syntax.body if isinstance(cls,ast.ClassDef) for fn in cls.body
                    if isinstance(fn,ast.FunctionDef) and fn.name=='_station_compute_measure_recovery')
        method_ast=ast.dump(method,include_attributes=False)
        from alicia_grasp_modes.rgb_observation_source import require_fresh_source
        namespace=dict(__name__='alicia_grasp_modes.station_compute_wait_producer',
            __package__='alicia_grasp_modes',__file__=str(production),
            np=np,_matching_candidates=table._matching_candidates,
            _validate_support_plane_update=table._validate_support_plane_update,
            require_fresh_source=require_fresh_source,
            rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:FAILED))))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(production),'exec'),namespace)
        reference=dict(mask=last_mask,reference_plane=plane,reference_footprint=anchor.support_footprint_base)
        marker=object();registration=dict(reference=reference,marker=marker)
        frame=dict(registration=registration)
        scope_calls=[]
        def inert_scope(offered,offered_camera):
            assert offered is registration and offered_camera is camera
            check_current();scope_calls.append(True)
            return reference
        def inert_marker(offered):
            assert offered is marker
            return hist,contour,None
        source=(NS(header=NS(stamp=NS(to_nsec=lambda:FAILED))),)
        def appearance_context(offered_source,offered_marker,check):
            assert offered_source is source and offered_marker is marker
            return dict(reference=frozen,source_ns=FAILED,check_current=check,
                same_station_mask_iou_minimum=.85,reference_role='qualified_station_identity_template')
        model=NS(_station_compute_frame=frame,_station_compute_scope=inert_scope,
            tracker=NS(anchor=anchor,last=last),config=cfg,max_age=.8,
            bridge=NS(imgmsg_to_cv2=lambda msg,desired_encoding:d['color']),
            _observation_marker_appearance=inert_marker,
            _observation_marker_appearance_context=appearance_context)
        try:
            namespace['_station_compute_measure_recovery'](model,source,seg,refined,camera)
            report['numeric_recovery_pass']=True
            report['numeric_recovery_evidence']=frame['recovery_evidence']
        except ValueError as exc:
            report['numeric_recovery_pass']=False
            report['numeric_recovery_error']=str(exc)
        assert sha(production)==production_sha
        assert scope_calls and np.array_equal(last_mask,reference['mask'])
        check_current()
        report['producer_numeric_method']=dict(path=str(production),sha256=production_sha,
            ast_sha256=hashlib.sha256(method_ast.encode()).hexdigest(),
            exact_method_body_used=True,inert_scope_call_count=len(scope_calls))
        report['numeric_source_contract']=dict(anchor_source_ns=ANCHOR,last_good_source_ns=LAST,
            fixed_qualified_template_source_ns=QUALIFIED,
            last_good_mask_iou=float(np.count_nonzero((last_mask>0)&(refined.mask>0))/np.count_nonzero((last_mask>0)|(refined.mask>0))),
            support_metrics={k:v for k,v in seg.metrics.items() if k.startswith('support_')})
        report.update(passed=True,input_sha256=pins)
    except BaseException as exc:
        report.update(error=repr(exc),traceback=traceback.format_exc())
    modules=[table,seeds,owner,sys.modules['alicia_grasp_modes.rgb_instance'],sys.modules['alicia_grasp_modes.observation_appearance']]
    report.update(elapsed_sec=time.monotonic()-started,helper=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)),
        source_sha256={m.__file__:sha(m.__file__) for m in modules},denied_operations=denied,
        ROS_initialized=False,live_custody_or_deadlines_replayed=False,production_modified=False,
        thresholds_changed=False,depth_filled=False,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        numerical_runtime=dict(numpy=np.__version__,opencv=cv2.__version__,PYTHONNOUSERSITE=os.environ.get('PYTHONNOUSERSITE')))
    report['limits']=['Only numeric recovery gates execute: exact candidate uniqueness, support audit and plane/footprint consistency, last-good IoU, fixed-template appearance.',
        'Scope ownership, RGB bridge and marker lookup are inert bound adapters; historical source ns is an explicit simulated clock, so no runtime deadline or live custody proof is claimed.',
        'Each later frame uses the same original anchor/last-good/template. No failed or later geometry updates an authority.',
        'Missing first-qualified image is not synthesized; exact recorded third-qualified image remains the original template.']
    (out/'probe_source.py').write_bytes(Path(__file__).read_bytes())
    save('result.json',report)
    print(json.dumps(dict(passed=report['passed'],error=report.get('error'),numeric_recovery_pass=report.get('numeric_recovery_pass'),
        numeric_recovery_error=report.get('numeric_recovery_error'),output=str(out))),flush=True)
    return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
