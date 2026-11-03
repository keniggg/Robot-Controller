"""Exact frozen-image owner replay; observational tracing only, no ROS clock claim."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
import argparse
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
W = LIVE/'.runtime/gen60_held_lattice_candidate_r2'
SDK = LIVE/'.runtime/realsense-2.56.5/pyrealsense2.cpython-38-x86_64-linux-gnu.so'
assert hashlib.sha256(SDK.read_bytes()).hexdigest() == '0bc7b6cad9ba2e602889b4771ff011781fe3cecd9bbc03b02713412c2678617f'
S = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/actual_attempt_r1')
I = S/'failed_seed_inputs_r2'
T = S/'terminal_evidence_r1'
E = S.parent/'seed_wait_fix_r1'
ANCHOR, FROZEN, QUALIFIED, LAST, FAILED = (1792649183970164060, 1792649207904424190,
    1792649241371229887, 1792649243437337398, 1792649243704485416)
sys.dont_write_bytecode = True
sys.path[:0] = [str(W/'src/alicia_grasp_modes/src'), str(W/'src/alicia_flexible_grasp_supervisor/src'),str(SDK.parent)]
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
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    out=args.output.resolve();assert E in out.parents and Path('/media/zhuyupei/SanStand').is_mount()
    out.mkdir(exist_ok=False);cv2.setNumThreads(1)
    assert np.__version__ == '1.17.4' and cv2.__version__ == '4.2.0'
    assert os.environ.get('PYTHONNOUSERSITE') == '1'
    started=time.monotonic();report=dict(passed=False,scope='unchanged pure segmentation/refinement/source-owner replay; no live station custody or deadlines replayed',checkpoints=[],seed_calls=[],owner_stages=[])
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
        plane=seg.plane_base.copy();recorded=statuses[FAILED]
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
        online=statuses[FAILED]['frame_timing']['observation_appearance_identity'][0]
        compare={k:dict(replay=appearance[k],recorded=online[k],equal=appearance[k]==online[k]) for k in ('code','current_mask_sha256','reference_mask_sha256','same_station_mask_iou','colour_distance','contour_distance')}
        report['failed_primary']=dict(raw=compact(raw),refined=compact(refined),appearance=appearance,online_comparisons=compare,native_candidates=[compact(c) for c in seg.candidates],segmentation_metrics=seg.metrics)
        assert all(v['equal'] for v in compare.values()),'online primary appearance not exactly reproduced'
        assert appearance['code']=='SAME_STATION_COMPLETE_OUTLINE_MISMATCH'
        camera_rows=json.loads((I/'camera_metadata.json').read_text())
        selected=max((r for r in camera_rows if r['bag_receipt_ns']<=FAILED),key=lambda r:r['bag_receipt_ns'])
        camera=selected['data']['parameters']
        assert camera['runtime_profile']['geometry_intrinsics']==rows[FAILED]['camera_geometry']['runtime_profile_geometry']['geometry_intrinsics']
        topology=SourceSamplingTopology(camera)
        report['camera_metadata_source']=dict(bag_receipt_ns=selected['bag_receipt_ns'],parameter_read_interval=selected['data']['parameter_read_interval'],source_time_association=selected['data']['image_time_association'],topology_digest=topology.contract_digest,source_map_sha256=topology.map_sha256)
        active={};fitdata={};stage=['not_started']
        def record_stage(name):
            stage[0]=name;report['owner_stages'].append(name)
        def finish_seed(frame,error=None):
            key=id(frame);record=active.pop(key);loc=frame.f_locals
            arrays={k:np.array(loc[k],copy=True) for k in ('seed','points','depth','heights','valid','measured','core','normals','reliable','cosine','top','side','foreground','background') if k in loc}
            arrays.update(fitdata.pop(key,{}))
            name='seed_%02d.npz'%record['call_index'];np.savez_compressed(out/name,**arrays)
            record.update(error=error,arrays=dict(path=str(out/name),sha256=sha(out/name)),counts={k:int(np.count_nonzero(v)) for k,v in arrays.items() if k in ('seed','valid','measured','core','reliable','top','side','foreground','background')},scalars={k:plain(loc[k]) for k in ('upper_height','original_top_pixels','upper_surface_band_used','separated','colour_evidence') if k in loc})
            core=arrays.get('core')
            if core is not None and 'fit_count' in arrays:
                conditions={'count_ge21':arrays['fit_count']>=21,'variance_le_0005sq':arrays['fit_variance']<=seeds.LOCAL_SURFACE_MAX_RMS_M**2,'variance_ratio_le_004':arrays['fit_variance']/np.maximum(arrays['fit_values'].sum(-1),1e-15)<=.04,'depth_span_le_008':arrays['fit_local_max']-arrays['fit_local_min']<=.008,'rank2_eigen_ge_1e8':arrays['fit_values'][...,1]>=1e-8}
                record['local_fit_pass_counts_among_eroded_core']={k:int(np.count_nonzero(core & v)) for k,v in conditions.items()}
                record['all_local_fit_gates_among_eroded_core']=int(np.count_nonzero(core & np.logical_and.reduce(list(conditions.values()))))
            report['seed_calls'].append(record)
        def trace(frame,event,arg):
            code=frame.f_code
            if code is seeds._confidence_labels.__code__:
                if event=='call':
                    parent=frame.f_back
                    while parent and parent.f_code is not owner._recover_source_connected_owner.__code__:parent=parent.f_back
                    loc={} if parent is None else parent.f_locals
                    sl=loc.get('sl');active[id(frame)]=dict(call_index=len(report['seed_calls'])+1,owner_stage=stage[0],caller_line=None if parent is None else parent.f_lineno,candidate_index=loc.get('index') if stage[0]=='current_projection_and_native_cores' else None,crop_yx=None if sl is None else [[s.start,s.stop] for s in sl],exception=None)
                elif event=='exception':active[id(frame)]['exception']=type(arg[1]).__name__+': '+str(arg[1])
                elif event=='return':finish_seed(frame,active[id(frame)]['exception'])
                return trace
            if code is seeds.local_surface_normals.__code__:
                if event=='return':
                    p=frame.f_back
                    while p and p.f_code is not seeds._confidence_labels.__code__:p=p.f_back
                    if p is not None:
                        fitdata[id(p)]={'fit_'+k:np.array(frame.f_locals[k],copy=True) for k in ('count','values','variance','local_max','local_min')}
                return trace
            return None
        assert sys.gettrace() is None
        sys.settrace(trace)
        try:
            recovered=owner.recover_source_connected_fragment_owner(seg,d['color'],d['depth'],d['K'],d['T_base_camera'],cfg,anchor,last,spatial,refined,minimum_quality,source_topology=topology,camera=camera,source_stamp_ns=FAILED,rgb_valid=d['rgb_valid'],check_current=check_current,record_stage=record_stage)
            report['owner_returned']=None if recovered is None else compact(recovered.owner)
            report['owner_error']=None
        except ValueError as exc:
            report['owner_error']=type(exc).__name__+': '+str(exc)
        finally:sys.settrace(None)
        check_current()
        assert report['owner_error']=='ValueError: rgb_seed_core_too_small',report['owner_error']
        report.update(passed=True,reproduced_failure=True,input_sha256=pins)
    except BaseException as exc:
        report.update(error=repr(exc),traceback=traceback.format_exc())
    modules=[table,seeds,owner,sys.modules['alicia_grasp_modes.rgb_instance'],sys.modules['alicia_grasp_modes.source_sampling_topology']]
    report.update(elapsed_sec=time.monotonic()-started,helper=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)),source_sha256={m.__file__:sha(m.__file__) for m in modules},denied_operations=denied,ROS_initialized=False,live_custody_or_deadlines_replayed=False,production_modified=False,thresholds_changed=False,depth_filled=False,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    import pyrealsense2 as rs
    assert Path(rs.__file__).resolve() == SDK and rs.__version__ == '2.56.5'
    report['numerical_runtime'] = dict(numpy_version=np.__version__,numpy_path=np.__file__,opencv_version=cv2.__version__,opencv_path=cv2.__file__,sdk_path=str(SDK),sdk_sha256=sha(SDK),sdk_version=rs.__version__,PYTHONNOUSERSITE=os.environ['PYTHONNOUSERSITE'])
    report['limits']=['Pure geometry computation begins after recorded station/feedback qualification; no ROS callback custody or wall-clock deadline admission is recreated.','Missing first qualified image is not synthesized. Recorded third qualified mask is used only as the original fixed appearance template.','Tracing observes original locals/returns/exceptions; no core pixels or rejection threshold are altered.','Local reliability gate pass counts are independent marginal counts, not disjoint failure counts.']
    save('result.json',report)
    print(json.dumps(dict(passed=report['passed'],error=report.get('error'),owner_error=report.get('owner_error'),seed_calls=len(report['seed_calls']),output=str(out))),flush=True)
    return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
