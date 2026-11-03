"""Frozen image/source-topology diagnosis only; no ROS node or device I/O."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
import argparse
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path
import platform
import sys
import time
import traceback
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path[:0] = [str(W/'.runtime/gen64_image_arbitration_candidate_r1/src/alicia_grasp_modes/src'),
    str(W/'src/alicia_flexible_grasp_supervisor/src'), str(W/'.runtime/realsense-2.56.5'),
    '/opt/ros/noetic/lib/python3/dist-packages']
# Python 3.8's first platform query invokes read-only uname; warm that cache
# before the I/O guard, then forbid all subprocesses during the replay.
platform.uname()
denied = []
def guard(event, args):
    if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.posix_spawn'):
        denied.append(event)
        raise RuntimeError('offline diagnosis forbids '+event)
sys.addaudithook(guard)
import cv2
import numpy as np
import genpy
import tf2_py
from geometry_msgs.msg import TransformStamped
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes import measured_mask_seeds as seeds
from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.rgb_instance import freeze_observation_spatial_reference
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance
from alicia_grasp_modes.arrival_fragment_group import recover_arrival_matching_fragments
from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

def plain(x):
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, np.generic): return x.item()
    if isinstance(x, dict): return {str(k):plain(v) for k,v in x.items()}
    if isinstance(x, (list,tuple)): return [plain(v) for v in x]
    return x
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def masksha(x): return hashlib.sha256(np.ascontiguousarray(x>0,dtype=np.uint8).tobytes()).hexdigest()
def bounds(x):
    yy,xx=np.nonzero(x)
    return None if len(xx)==0 else [int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)]
def compact(c):
    return dict(position_base=c.position_base,extent_m=c.extent_m,quality=c.quality,
        mask_pixels=int(np.count_nonzero(c.mask)),mask_sha256=masksha(c.mask),bbox=c.bbox,
        metrics=c.metrics)
def matrix(message):
    t,q=message.transform.translation,message.transform.rotation
    quaternion=np.array([q.x,q.y,q.z,q.w],dtype=float)
    norm=float(quaternion@quaternion)
    if not np.isfinite(norm) or abs(norm-1.)>.01: raise ValueError('invalid exact quaternion')
    x,y,z,w=quaternion/np.sqrt(norm)
    m=np.eye(4)
    m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    m[:3,3]=[t.x,t.y,t.z]
    return m

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--inputs',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--sources',type=int,nargs='+',default=[1792730242218747854,1792730242621507406,1792730242954490184])
    args=ap.parse_args(); D=args.inputs.resolve(); out=args.output.resolve()
    root=Path('/media/zhuyupei/SanStand')
    assert root.is_mount() and root in out.parents and root in D.parents
    out.mkdir(exist_ok=False)
    assert os.environ.get('PYTHONNOUSERSITE')=='1'
    assert np.__version__=='1.17.4' and cv2.__version__=='4.2.0'
    cv2.setNumThreads(1)
    start=time.monotonic()
    report=dict(scope='isolated image repair on frozen geometry/TF; full source checks and original independent proof; no live custody or deadline replay',
        rows=[],denied_operations=denied,ROS_initialized=False,production_modified=False,
        thresholds_changed=False,depth_filled=False,passed=False)
    loaded=[table,seeds,owner,sys.modules['alicia_grasp_modes.rgb_instance'],
        sys.modules['alicia_grasp_modes.arrival_fragment_group'],
        sys.modules['alicia_grasp_modes.source_sampling_topology']]
    report['source_sha256']={m.__file__:sha(m.__file__) for m in loaded}
    try:
        statuspath=D/'detector_status_window.json'; tfpath=D/'tf_frozen_window.json'
        campath=D.parent/'scene_start_r2/camera_configuration.json'
        meta=json.loads((D/'representative_frames.json').read_text())
        cfgpath=Path(meta['bag']).parent/'perception_execution_configuration.json'
        report['input_sha256']={str(p):sha(p) for p in (statuspath,tfpath,campath,cfgpath,D/'representative_frames.json')}
        statuses={x['status']['source_stamp_ns']:x['status'] for x in json.loads(statuspath.read_text()) if 'source_stamp_ns' in x['status']}
        reference=next(s for s in statuses.values() if s.get('station_seed_insufficiency'))
        cfgvalues=json.loads(cfgpath.read_text())['parameters']['/unknown_perception']
        cfg=table.Config(**{f.name:cfgvalues[f.name] for f in fields(table.Config) if f.name in cfgvalues})
        report['config']=asdict(cfg)
        camera=json.loads(campath.read_text()); K=np.array([camera[k] for k in ('fx','fy','cx','cy')])
        topology=SourceSamplingTopology(camera)
        rgb_valid=rectification_rgb_valid_mask(camera,topology.output_shape)
        report['topology']=dict(contract_digest=topology.contract_digest,map_sha256=topology.map_sha256)
        tfdata=json.loads(tfpath.read_text()); buffer=tf2_py.BufferCore()
        for row in tfdata['static']+tfdata['rows']:
            m=TransformStamped();m.header.frame_id=row['parent'];m.child_frame_id=row['child']
            m.header.stamp=genpy.Time(row['source_ns']//10**9,row['source_ns']%10**9)
            m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=row['translation']
            m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=row['rotation_xyzw']
            (buffer.set_transform_static if row['topic']=='/tf_static' else buffer.set_transform)(m,'frozen_record')
        anchor=SimpleNamespace(position_base=np.array(reference['anchor_position_base']),
            extent_m=np.array(reference['anchor_extent_m']),
            support_footprint_base=np.array(reference['reference_support_footprint_base']),
            metrics={'median_height_m':reference['reference_object_height_m']})
        plane=np.array(reference['reference_support_plane_base'])
        spatial=freeze_observation_spatial_reference(anchor,plane)
        qualified=1792730241754907846
        qrgb=cv2.imread(str(D/('%d_color.png'%qualified)),cv2.IMREAD_COLOR)
        qmask=cv2.imread(str(D/('%d_mask.png'%qualified)),cv2.IMREAD_GRAYSCALE)
        histogram,contour=_appearance(qrgb,_mask(qmask,qmask.shape))
        frozen=FrozenObservationAppearance(qrgb,qmask,rgb_valid,qualified,histogram,contour)
        last=anchor
        for stamp in args.sources:
            rgbp=D/('%d_color.png'%stamp);depthp=D/('%d_depth.npy'%stamp);maskp=D/('%d_mask.png'%stamp)
            for p in (rgbp,depthp,maskp):report['input_sha256'][str(p)]=sha(p)
            rgb=cv2.imread(str(rgbp),cv2.IMREAD_COLOR)
            rawdepth=np.load(depthp);depth=rawdepth.astype(np.float32)*float(camera['depth_scale'])
            recorded_mask=cv2.imread(str(maskp),cv2.IMREAD_GRAYSCALE)
            T=matrix(buffer.lookup_transform_core('base_link','camera_link',genpy.Time(stamp//10**9,stamp%10**9)))
            row=dict(source_ns=stamp,seed_calls=[],owner_stages=[],T_base_camera=T)
            report['rows'].append(row)
            seg=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,plane,prefer_current_support=True)
            row['native_candidates']=[compact(c) for c in seg.candidates]
            matches=table._matching_candidates(seg.candidates,anchor,last,cfg)
            row['metric_matches']=len(matches);row['segmentation_metrics']=seg.metrics
            row['match_indices']=[next(i for i,c in enumerate(seg.candidates) if c is matched) for _,matched in matches]
            active={};fitdata={};stage=['not_started']
            def record_stage(name):stage[0]=name;row['owner_stages'].append(name)
            def trace(frame,event,arg):
                if frame.f_code is owner._recover_source_connected_owner.__code__:
                    if event=='return':
                        loc=frame.f_locals
                        keys=['group','raw_owner','raw_only','output_region','valid','heights','plane_camera','source_group']
                        arrays={k:np.array(loc[k],copy=True) for k in keys if k in loc}
                        if arrays:
                            p=out/('%d_owner_arrays.npz'%stamp);np.savez_compressed(p,**arrays)
                            row['owner_arrays']=dict(path=str(p),sha256=sha(p))
                            row['owner_counts']={k:int(np.count_nonzero(v)) for k,v in arrays.items() if k in ('group','raw_owner','raw_only','output_region','valid','source_group')}
                            row['owner_bounds']={k:bounds(arrays[k]) for k in ('group','raw_owner','raw_only','output_region') if k in arrays}
                            if 'extra_regions' in loc:row['extra_regions_pixels']=[int(np.count_nonzero(x)) for x in loc['extra_regions']]
                    return trace
                if frame.f_code is seeds._confidence_labels.__code__:
                    if event=='call':
                        parent=frame.f_back
                        while parent and parent.f_code is not owner._recover_source_connected_owner.__code__:parent=parent.f_back
                        if parent is None:return None
                        loc=parent.f_locals;sl=loc.get('sl')
                        active[id(frame)]=dict(call_index=len(row['seed_calls'])+1,stage=stage[0],
                            crop_yx=None if sl is None else [[s.start,s.stop] for s in sl],error=None)
                    elif event=='exception' and id(frame) in active:
                        active[id(frame)]['error']=type(arg[1]).__name__+': '+str(arg[1])
                    elif event=='return' and id(frame) in active:
                        rec=active.pop(id(frame));loc=frame.f_locals
                        arrays={k:np.array(loc[k],copy=True) for k in ('seed','points','depth','heights','valid','measured','core','normals','reliable','cosine','top','side','foreground','background') if k in loc}
                        arrays.update(fitdata.pop(id(frame),{}))
                        p=out/('%d_seed_%02d.npz'%(stamp,rec['call_index']));np.savez_compressed(p,**arrays)
                        rec.update(arrays=dict(path=str(p),sha256=sha(p)),counts={k:int(np.count_nonzero(v)) for k,v in arrays.items() if k in ('seed','valid','measured','core','reliable','top','side','foreground','background')},
                            scalars={k:plain(loc[k]) for k in ('upper_height','original_top_pixels','upper_surface_band_used','separated','colour_evidence') if k in loc})
                        if 'core' in arrays and 'fit_count' in arrays:
                            gates={'count_ge21':arrays['fit_count']>=21,'variance_le_0005sq':arrays['fit_variance']<=seeds.LOCAL_SURFACE_MAX_RMS_M**2,'variance_ratio_le_004':arrays['fit_variance']/np.maximum(arrays['fit_values'].sum(-1),1e-15)<=.04,'depth_span_le_008':arrays['fit_local_max']-arrays['fit_local_min']<=.008,'rank2_eigen_ge_1e8':arrays['fit_values'][...,1]>=1e-8}
                            rec['fit_gate_pass_counts_among_core']={k:int(np.count_nonzero(arrays['core']&v)) for k,v in gates.items()}
                            rec['all_fit_gates_among_core']=int(np.count_nonzero(arrays['core']&np.logical_and.reduce(list(gates.values()))))
                        row['seed_calls'].append(rec)
                    return trace
                if frame.f_code is seeds.local_surface_normals.__code__:
                    if event=='return':
                        p=frame.f_back
                        while p and p.f_code is not seeds._confidence_labels.__code__:p=p.f_back
                        if p is not None and id(p) in active:
                            fitdata[id(p)]={'fit_'+k:np.array(frame.f_locals[k],copy=True) for k in ('count','values','variance','local_max','local_min')}
                    return trace
                return None
            before=[hashlib.sha256(x.tobytes()).hexdigest() for x in (rgb,depth,K,T)]
            def check_current():
                assert [hashlib.sha256(x.tobytes()).hexdigest() for x in (rgb,depth,K,T)]==before
            def independent_measurement():
                check_current()
                return recover_arrival_matching_fragments(seg,rgb,depth,K,T,cfg,anchor,last,spatial,histogram,contour,.5,
                    rgb_valid=rgb_valid,appearance_context=dict(reference=frozen,source_ns=stamp,check_current=check_current,
                        same_station_mask_iou_minimum=.85,reference_role='qualified_station_identity_template'))
            sys.settrace(trace)
            try:
                recovered=owner.recover_source_connected_measurement(seg,rgb,depth,K,T,cfg,anchor,last,
                    spatial,None,.5,source_topology=topology,camera=camera,source_stamp_ns=stamp,
                    rgb_valid=rgb_valid,check_current=check_current,record_stage=record_stage,
                    independent_measurement=independent_measurement)
                row['owner_error']=None
                if recovered is not None:
                    row['recovered_audit']=recovered.audit
                    row['candidate_after']=compact(recovered.refined)
                    row['owner_evidence']=recovered.audit['raw_only_unqualified_regions'][0]
                    path=out/('%d_candidate_mask.npy'%stamp);np.save(path,recovered.refined.mask)
                    row['candidate_mask']=dict(path=str(path),sha256=sha(path))
            except ValueError as exc:
                recovered=None;row['owner_error']=type(exc).__name__+': '+str(exc)
                row['owner_evidence']=getattr(exc,'evidence',None)
            finally:sys.settrace(None)
            check_current()
            online=statuses[stamp].get('station_seed_insufficiency')
            if online:
                row['online_evidence']=online
                actual=row.get('owner_evidence',{})
                row['online_evidence_exact']={k:actual.get(k)==v for k,v in online.items() if k not in ('version','parent_source_ns')}
                assert all(row['online_evidence_exact'].values()),'replayed raw-only evidence differs from online'
            if not online:
                if recovered is None:
                    newseg,pair=recover_arrival_matching_fragments(seg,rgb,depth,K,T,cfg,anchor,last,spatial,histogram,contour,.5,
                        rgb_valid=rgb_valid,appearance_context=dict(reference=frozen,source_ns=stamp,check_current=check_current,
                            same_station_mask_iou_minimum=.85,reference_role='qualified_station_identity_template'))
                    assert pair is not None,'recorded good-frame recovery did not return pair'
                    last,refined=pair
                else:last,refined=recovered.raw,recovered.refined
                row['good_mask_bit_equal']=bool(np.array_equal(refined.mask,recorded_mask))
                row['good_reconstructed']=compact(refined)
                assert row['good_mask_bit_equal'],'recorded good mask not reproduced'
            elif online:
                assert recovered is not None, row.get('owner_error')
                before=D/'raw_only_replay_r2'/('%d_independent_mask.npy'%stamp)
                row['unchanged_original_proof_mask_bit_equal']=bool(np.array_equal(recovered.refined.mask,np.load(before)))
                assert row['unchanged_original_proof_mask_bit_equal']
                assert 'source_four_neighbor_articulation' in row['owner_stages']
                assert 'complete_current_ownership_and_geometry' in row['owner_stages']
                assert recovered.audit['articulation_guard_passed']
                assert not recovered.audit['native_union_used_as_rgb_seed']
                assert recovered.audit['independent_current_native_proof_used']
                recovered.require_unchanged()
            print(json.dumps(plain(dict(source_ns=stamp,matches=len(matches),owner_error=row['owner_error'],
                online_exact=all(row.get('online_evidence_exact',{}).values()),good_mask_bit_equal=row.get('good_mask_bit_equal'),
                owner_counts=row.get('owner_counts'),bounds=row.get('owner_bounds')))),flush=True)
        assert all(sha(p)==h for p,h in report['source_sha256'].items())
        report['passed']=True
    except BaseException as exc:
        report.update(error=repr(exc),traceback=traceback.format_exc())
    report.update(elapsed_sec=time.monotonic()-start,numerical_runtime=dict(numpy=np.__version__,opencv=cv2.__version__,threads=1),helper=dict(path=str(Path(__file__).resolve()),sha256=sha(__file__)))
    (out/'result.json').write_text(json.dumps(plain(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'],error=report.get('error'),output=str(out))),flush=True)
    return 0 if report['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
