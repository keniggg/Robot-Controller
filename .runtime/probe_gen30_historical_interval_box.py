"""One finite offline comparison of three original initial-view RGB-D sources.

Isolated function clones change rectangle estimation, never production code or
gate thresholds. The two predeclared alternatives use endpoint least squares,
and optionally exact fractional pixel-cell area for the same 5x5 corner gate.
"""
import ast
import difflib
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
O = M/'实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z/verification/gen30_historical_interval_box_r1'
IDS = (1792062749691942453, 1792062749893290758, 1792062750092055082)
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
assert not O.exists()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', PYTHONDONTWRITEBYTECODE='1',
                  OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
for relative in ('src/alicia_flexible_grasp_supervisor/src', 'devel/lib/python3/dist-packages'):
    sys.path.insert(0, str(W/relative))

import numpy as np
from alicia_flexible_grasp.vision import regular_box_inference as box
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def arrsha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


inputs = {}
def read(path):
    inputs[str(path)] = sha(path)
    return json.loads(Path(path).read_text())


prior = read(N/'verification/gen30_initial_view_box_replay_r1/result.json')
assert inputs[str(N/'verification/gen30_initial_view_box_replay_r1/result.json')] == '8334b360c1af652b735c4bf51b89e55a5a5b26ddd0ebc5ba9ce1d783c81f56ce'
axes_diagnosis = read(N/'verification/gen30_initial_visual_axes_r1/result.json')
assert tuple(prior['selected_original_sources']) == IDS
assert tuple(row['source_stamp_ns'] for row in axes_diagnosis['rows']) == IDS
source_paths = [Path(inspect.getsourcefile(module)) for module in
    (box, sys.modules[fuse_stable_samples.__module__], sys.modules[estimate_object_geometry.__module__])]
sources = {str(path): sha(path) for path in source_paths}
assert sha(source_paths[0]) == '9006b9b3b416cb7b5a43b58e9ab420e8d7294442235eaf7a95673f6188729c0f'
identity = TargetTrackIdentity(**prior['source_identity'])
arrays, samples = [], []
for ns, binding in zip(IDS, prior['frame_bindings']):
    assert binding['source_stamp_ns'] == ns
    path = Path(binding['input_npz'])
    inputs[str(path)] = sha(path)
    assert inputs[str(path)] == binding['input_sha256']
    with np.load(path, allow_pickle=False) as archive:
        a = {key: archive[key].copy() for key in archive.files}
    assert all(arrsha(a[key]) == digest for key, digest in binding['array_sha256'].items())
    assert int(a['source_stamp_ns']) == ns
    bbox = tuple(binding['bbox_xywh'])
    # Geometry does not inspect the ROS object message. Keep its original
    # detected/bbox facts without importing ROS or opening the archived bag.
    message = NS(detected=True, bbox_x=bbox[0], bbox_y=bbox[1],
                 bbox_width=bbox[2], bbox_height=bbox[3])
    q = np.asarray(binding['accepted_joint_bracket']['before']['positions_rad'])
    samples.append(RgbdSample(a['color_bgr'], a['depth_raw'], a['object_mask'], bbox,
        message, ns/1e9, 'camera_link', q, ns, identity.epoch, identity))
    arrays.append(a)
scale = float(arrays[-1]['depth_scale_m'])
k = arrays[-1]['intrinsics']
K = NS(fx=k[0], fy=k[1], cx=k[2], cy=k[3],
       width=arrays[-1]['depth_raw'].shape[1], height=arrays[-1]['depth_raw'].shape[0])
optical_to_link = np.eye(4)
optical_to_link[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
T = arrays[-1]['T_base_camera_link'] @ optical_to_link
snapshot = fuse_stable_samples(samples, **prior['snapshot_arguments'])
assert snapshot.ok and tuple(snapshot.sample_stamp_ns) == IDS
geometry = estimate_object_geometry(snapshot.depth_raw, snapshot.target_depth_raw,
    snapshot.object_mask, snapshot.bbox, K, scale, T, snapshot.source_mode,
    **prior['geometry_arguments'])
assert geometry.ok
assert arrsha(geometry.object_points_base) == prior['geometry']['object_points_sha256']
for key in ('center_base', 'axes_base', 'size_xyz_m', 'support_normal_base'):
    np.testing.assert_array_equal(getattr(geometry, key), prior['geometry'][key])


# One declared, isolated partial-top contract. No threshold sweep and no
# source modification. Extract reusable pure LS and recorded original plane
# geometry, never execute historical replay scripts.
from itertools import product
import cv2

fit_path=W/'.runtime/replay_gen30_initial_visual_direction_full.py'
inputs[str(fit_path)]=sha(fit_path)
fit_text=fit_path.read_text(); fit_tree=ast.parse(fit_text)
fit_ast=next(node for node in fit_tree.body if isinstance(node,ast.FunctionDef) and node.name=='fitted_rectangle')
exec(compile(ast.get_source_segment(fit_text,fit_ast),'<isolated_endpoint_fit>','exec'),globals())
trace_path=N/'verification/gen30_initial_view_visual_trace_r1/result.json'
inputs[str(trace_path)]=sha(trace_path)
trace_report=json.loads(trace_path.read_text())
trace=dict(rows=trace_report['raw_frames_with_same_fused_geometry'])
ids=IDS
# Re-evaluate only the original function once per historical source, retaining
# local planes absent from the old read-only trace. Same original results are
# checked exactly; the new fixed contract below is identical to current r1.
for ns,a,row in zip(ids,arrays,trace['rows']):
    local={}
    def record_local(frame,event,arg):
        if frame.f_code is box.qualify_regular_box_top.__code__:
            if event=='return':
                for key in ('origin','tn','axes','lower','upper','pixel_size','top_height'):
                    value=frame.f_locals[key]
                    local[key]=value.tolist() if isinstance(value,np.ndarray) else value
            return record_local
        return None
    previous=sys.gettrace()
    try:
        sys.settrace(record_local)
        original=box.qualify_regular_box_top(a['depth_raw'],a['object_mask'],K,scale,
            a['T_base_camera_link']@optical_to_link,geometry,identity,ns,color_bgr=a['color_bgr'])
    finally:sys.settrace(previous)
    assert original.code==row['qualification']['code'] and dict(original.audit)==row['qualification']['audit']
    row['qualification']['return_local_diagnostics'].update(local)


def on_plane(pixels,T,K,origin,tn,shift=0.):
    ray=np.column_stack(((pixels[:,0]-K[2])/K[0],(pixels[:,1]-K[3])/K[1],np.ones(len(pixels))))@T[:3,:3].T
    denominator=ray@tn
    distance=(float((origin-T[:3,3])@tn)+shift)/denominator
    if np.any(~np.isfinite(distance)) or np.any(distance<=0.):raise ValueError('unbounded plane projection')
    return T[:3,3]+ray*distance[:,None]

def in_top(coords,axes,origin,tn):
    tangent=coords@axes[:,:2].T
    return tangent+np.outer(((origin-tangent)@tn)/(tn@axes[:,2]),axes[:,2])

def image_sample(depth,pixels,T,K,origin,tn):
    xy=np.rint(pixels).astype(int)
    if np.any(xy<0) or np.any(xy[:,0]>=depth.shape[1]) or np.any(xy[:,1]>=depth.shape[0]):
        raise ValueError('sampling outside image')
    z=depth[xy[:,1],xy[:,0]].astype(float)*scale
    good=np.isfinite(z)&(z>0.)
    points=np.column_stack(((xy[:,0]-K[2])*z/K[0],(xy[:,1]-K[3])*z/K[1],z))@T[:3,:3].T+T[:3,3]
    plane=(points-origin)@tn
    return xy,good,plane

def interval_candidate(a,q,selected,index):
    """Current line location intervals; unknown depth is never synthesized.

    Bounds combine the endpoint normal residual and every half-pixel cell
    corner projected through the existing empirical top-plane error slab.
    These remain empirical, not a calibrated confidence interval. All Hough
    hypotheses are evaluated before a common envelope is formed.
    """
    d=q['return_local_diagnostics'];audit=q['audit']
    depth=a['depth_raw'];mask=a['object_mask']>0
    T=a['T_base_camera_link']@optical_to_link;K=a['intrinsics']
    origin=np.array(d['origin']);tn=np.array(d['tn']);original_axes=np.array(d['axes'])
    pixel=d['pixel_size'];plane_budget=max(.001,2*audit['top_plane_rms_m']+scale)
    project=lambda pixels:on_plane(pixels,T,K,origin,tn)
    axes,lo,hi,slope,residual,fit=fitted_rectangle(selected,project,original_axes)
    row=dict(index=index,fit=fit,size_xy_m=(hi-lo).tolist(),center_xy_m=((hi+lo)/2.).tolist(),
        old_local_corner_coverage_is_not_a_gate=True,shape_evidence_pass=False,
        interval_evidence_pass=False,edges=[],rejections=[],plane_error_budget_m=plane_budget)
    if np.any(hi-lo<.005) or np.any(hi-lo>.6):
        row['rejections'].append('invalid_metric_extent');return row,None
    corners=np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]])
    projected=box._project(in_top(corners,axes,origin,tn),T,K)
    row['projected_corners_px']=projected.tolist()
    if min(float(projected.min()),float((depth.shape[1]-1-projected[:,0]).min()),float((depth.shape[0]-1-projected[:,1]).min()))<4:
        row['rejections'].append('image_border');return row,None
    corner_cov=[]
    for p in projected:
        x,y=np.rint(p).astype(int);corner_cov.append(float(min(1,np.mean(mask[y-2:y+3,x-2:x+3])/.25)))
    row['local_corner_coverage_audit_only']=corner_cov
    edges=cv2.Canny(cv2.cvtColor(a['color_bgr'],cv2.COLOR_BGR2GRAY),50.,100.)
    edges[cv2.dilate(mask.astype(np.uint8),np.ones((5,5),np.uint8))==0]=0
    edge_pixels=cv2.dilate(edges,np.ones((5,5),np.uint8))>0
    ys,xs=np.nonzero(mask);silhouette=project(np.column_stack((xs,ys)))@axes[:,:2]
    slo,shi=silhouette.min(axis=0),silhouette.max(axis=0)
    intervals=[];mask_boundaries=0;coverages=[];continuations=[];outside_valids=[];extensions=[]
    for i in range(4):
        across=1 if i%2==0 else 0
        nominal=(lo[1],hi[0],hi[1],lo[0])[i]
        endpoint_cells=np.array([p+[dx,dy] for p in selected[i][2] for dx,dy in product((-.5,.5),repeat=2)])
        slab=np.vstack([on_plane(endpoint_cells,T,K,origin,tn,s)@axes[:,:2] for s in (-plane_budget,plane_budget)])
        interval=float(np.max(np.abs(slab[:,across]-nominal)))
        intervals.append(interval)
        start,end=corners[i],corners[(i+1)%4];v=end-start
        outward=np.array([v[1],-v[0]])/np.linalg.norm(v)
        if outward@((start+end)/2.-corners.mean(axis=0))<0:outward*=-1
        center_samples=start+np.outer(np.linspace(.08,.92,31),v)
        cp=np.rint(box._project(in_top(center_samples,axes,origin,tn),T,K)).astype(int)
        visual_support=float(np.mean(edge_pixels[cp[:,1],cp[:,0]]));coverages.append(visual_support)
        fractions=np.linspace(.12,.88,21)
        # One pixel beyond the complete interval avoids calling a pixel whose
        # footprint touches the uncertain boundary an external contradiction.
        distance=interval+pixel
        samples=[start+np.outer(fractions,v)+direction*distance*outward for direction in (-1.,1.)]
        inside,outside=[image_sample(depth,box._project(in_top(s,axes,origin,tn),T,K),T,K,origin,tn) for s in samples]
        xy,valid,plane=outside
        mask_border=float(np.mean(~mask[xy[:,1],xy[:,0]]));mask_boundaries+=int(mask_border>=.70)
        valid_inside=float(np.mean(inside[1]));supported=float(np.mean(inside[1]&(np.abs(inside[2])<.0015)))
        valid_outside=float(np.mean(valid));continued=float(np.count_nonzero(valid&(np.abs(plane)<.0015)))/max(1,int(np.count_nonzero(valid)))
        higher=float(np.mean(valid&(plane>.0025)))
        continuations.append(continued);outside_valids.append(valid_outside)
        extension=0.
        if valid_inside<.60 or valid_outside<.60:
            extension=max(0.,(lo[1]-slo[1],shi[0]-hi[0],shi[1]-hi[1],lo[0]-slo[0])[i])
        extensions.append(float(extension))
        er=dict(edge=i,normal_interval_m=interval,outside_sample_distance_m=distance,visual_support=visual_support,
            mask_border_fraction=mask_border,valid_inside=valid_inside,inside_top_fraction=supported/max(valid_inside,1e-12),
            valid_outside=valid_outside,outside_top_continuation=continued,occluder_fraction=higher,
            original_mask_unobserved_extension_m=float(extension),outside_sample_pixels=xy.tolist())
        row['edges'].append(er)
        if visual_support<.70:row['rejections'].append('visible_line_support_edge_%d'%i)
        if valid_inside>=.60 and supported/max(valid_inside,1e-12)<.70:row['rejections'].append('measured_inner_not_top_edge_%d'%i)
        if valid_outside>=.20 and continued>.30:row['rejections'].append('top_continues_beyond_localization_interval_edge_%d'%i)
        if higher>.30:row['rejections'].append('measured_occluder_edge_%d'%i)
    row['shape_evidence_pass']=all(c>=.70 for c in coverages)
    row['mask_boundary_count']=mask_boundaries
    if mask_boundaries<3:row['rejections'].append('fewer_than_three_instance_boundaries')
    row['interval_evidence_pass']=not row['rejections']
    row['line_position_intervals_m']=intervals
    row['unobserved_extension_m']=max(extensions)
    row['positional_budget_m']=max(plane_budget+.5*pixel+max(extensions),max(intervals))
    # Endpoint direction interval includes both endpoint cell/slab error. It
    # intentionally does not reuse a favorable average orientation alone.
    angles=[math.atan2(2*e,float(np.linalg.norm(project(s[2])[1]-project(s[2])[0]))) for e,s in zip(intervals,selected)]
    row['yaw_budget_rad']=max(math.radians(.25),math.atan(slope),*angles)
    payload=dict(axes=axes,lo=lo,hi=hi,corner_cov=corner_cov,coverages=coverages,slope=slope,
        extension=max(extensions),continuation=continuations,outside_valid=outside_valids,
        positional_budget=row['positional_budget_m'],yaw_budget=row['yaw_budget_rad'])
    return row,payload

# Clone the downstream qualifier solely to assess this explicitly different
# visual contract. The old serialized model validator is NOT bypassed into a
# production object: a diagnostic dict is returned before serialization.
def change(s,old,new):
    assert s.count(old)==1,old
    return s.replace(old,new)
original_qualifier=inspect.getsource(box.qualify_regular_box_top)
qualifier=change(original_qualifier,'            lower,upper=visual[1],visual[2]',
    '            lower,upper=visual[1],visual[2]\n            axes=chosen["axes"]\n            coord=top@axes[:,:2]')
qualifier=change(qualifier,'        if (visual is None and fill < .88) or min(corner_coverage) < .5 or min(edge_coverage) < .7:',
    '        if (visual is None and (fill < .88 or min(corner_coverage) < .5)) or min(edge_coverage) < .7:')
start=qualifier.index('        continuation, outside_valid = [], []')
end=qualifier.index('        edge_rms = ',start)
qualifier=qualifier[:start]+'''        continuation=chosen['continuation']
        outside_valid=chosen['outside_valid']
        audit.update(outside_top_continuation_fraction=continuation,outside_valid_depth_fraction=outside_valid,
            outside_sampling='beyond_measured_line_position_interval_plus_one_pixel')
'''+qualifier[end:]
qualifier=change(qualifier,'        height_uncertainty = max(',
    '        edge_uncertainty=max(edge_uncertainty,chosen["positional_budget"])\n        height_uncertainty = max(')
qualifier=change(qualifier,'        measured = np.asarray(geometry.object_points_base)',
    '        yaw_uncertainty=max(yaw_uncertainty,chosen["yaw_budget"])\n        measured = np.asarray(geometry.object_points_base)')
qualifier=change(qualifier,"        model['model_sha256'] = _hash(model)",
    "        model['model_sha256'] = _hash(model)\n        return dict(ok=True,code='ISOLATED_PARTIAL_TOP_MODEL_BUDGET_QUALIFIED',audit=dict(audit),model=model,production_schema_valid=False)")

def downstream(a,ns,chosen):
    def visual(*args):
        return (float(np.prod(chosen['hi']-chosen['lo'])),chosen['lo'],chosen['hi'],chosen['corner_cov'],
            chosen['coverages'],chosen['slope'],chosen['extension'])
    namespace=dict(vars(box),chosen=chosen,_visual_rectangle=visual)
    exec(compile(qualifier,'<isolated_partial_top_qualifier>','exec'),namespace)
    result=namespace['qualify_regular_box_top'](a['depth_raw'],a['object_mask'],K,scale,
        a['T_base_camera_link']@optical_to_link,geometry,identity,ns,color_bgr=a['color_bgr'])
    if isinstance(result,dict):return result
    return dict(ok=result.ok,code=result.code,audit=dict(result.audit),model=None)

rows=[]
for ns,a,tr in zip(ids,arrays,trace['rows']):
    assert ns==tr['source_stamp_ns']
    q=tr['qualification'];choices=q['return_local_diagnostics']['visual_rectangle_trace']['choices_before_top3']
    choices=[[(r['offset_m'],r['overlap_m'],np.array(r['pixels']),r['slope']) for r in group] for group in choices]
    candidates=[];qualified=[];precision=[]
    for index,selected in enumerate(product(*choices),1):
        row,payload=interval_candidate(a,q,selected,index)
        if row['interval_evidence_pass']:
            result=downstream(a,ns,payload);row['downstream']=result
            if result['ok']:qualified.append(result['model'])
            elif result['code']=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED':precision.append(index)
        candidates.append(row)
    output=dict(source_stamp_ns=ns,original_code=q['code'],hypotheses_examined=len(candidates),
        line_supported_shape_hypotheses=sum(r['shape_evidence_pass'] for r in candidates),
        interval_evidence_hypotheses=sum(r['interval_evidence_pass'] for r in candidates),
        individually_budget_qualified=len(qualified),candidates=candidates,
        unresolved_precision_hypotheses=precision)
    # All remaining physical explanations are united. A failed precision gate
    # cannot be discarded merely because another hypothesis fits a gripper.
    if not qualified:
        output['outcome']='PRECISION_REQUIRED' if output['line_supported_shape_hypotheses'] else 'NO_SUPPORTED_VISUAL_SHAPE'
        output['reason']='visible regular-top line hypotheses exist but current boundary evidence does not resolve a metric model' if output['line_supported_shape_hypotheses'] else 'no supported four-edge visual shape'
    else:
        # Deterministic reference independent of grasp convenience; envelope
        # contains every surviving model with its own positional/yaw budgets.
        reference=min(qualified,key=lambda m:tuple(m['center_base']))
        A=np.array(reference['axes_base']);cloud=[]
        for m in qualified:
            ax=np.array(m['axes_base']);sz=np.array(m['size_xyz_m']);yaw=m['empirical_yaw_uncertainty_rad'];u=m['empirical_edge_uncertainty_m']
            spread=[x*math.cos(min(yaw,math.atan2(y,x)))+y*math.sin(min(yaw,math.atan2(y,x)))+2*u for x,y in ((sz[0],sz[1]),(sz[1],sz[0]))]
            center=np.array(m['center_base']);half=np.array([spread[0],spread[1],sz[2]])/2
            cloud.extend(center+ax@(np.array(sign)*half) for sign in product((-1.,1.),repeat=3))
        cloud=np.array(cloud);local=cloud@A;extent=local.max(axis=0)-local.min(axis=0)
        output.update(ensemble_enclosure_size_m=extent.tolist(),ensemble_enclosure_center_base=(A@((local.max(axis=0)+local.min(axis=0))/2)).tolist(),
            jaw_gap_limit_m=.05,minimum_geometric_gap_m=float(min(extent[:2])),contact_clearance_not_yet_added=True)
        output['outcome']='PRECISION_REQUIRED' if precision or min(extent[:2])>.05 else 'ISOLATED_ENVELOPE_WITHIN_NOMINAL_JAW_GAP'
        output['reason']='all surviving interpretations are included; actual contact clearance/path/closure gates are not executed'
    rows.append(output)
assert all(sha(path)==digest for path,digest in sources.items())
assert all(sha(path)==digest for path,digest in inputs.items())
O.mkdir()
report=dict(classification='same_fixed_partial_top_interval_contract_historical_three_source_proof',rows=rows,
    input_sha256=inputs,source_sha256=sources,script_sha256=sha(__file__),production_modified=False,
    no_ros_network_calls=True,no_hardware_access=True,execution_authority=False,production_schema_model_created=False,
    method=dict(corner_coverage='audit only for a supported four-line visual hypothesis',
        plane_interval='existing max(1mm,2*plane_rms+raw_depth_scale), empirical not calibrated confidence',
        line_interval='all endpoint half-pixel cells projected through plane interval, plus orthogonal-fit discrepancy',
        outside_sampling='one metric pixel outside complete line interval; original continuation/occlusion ratios unchanged',
        hypotheses='all original distinct Hough groups, no top-three pruning; no best-only selection',
        retained_gates='original metric size, top plane, target fill, top occupancy, internal holes, measured model conflict, height, 10mm positional and 12degree yaw limits',
        uncertainty='all unknown-mask extension and surviving line/center/angle/extent interpretations retained'),
    limitations=['Not a production model or motion authorization.','Candidate Hough enumeration remains finite and is not a completeness proof for all possible unseen shapes.',
        'Raw edge depth blending is not assigned an invented fixed-width allowance. Outside continuation beyond the independent line interval remains contradictory.',
        'No calibration confidence model for top plane exists; budgets remain explicitly empirical.',
        'Actual contact clearance, plan collision, closure and execution are not evaluated; nominal 50mm opening is only a necessary test.'])
(O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=serial,allow_nan=False)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),rows=[{k:v for k,v in row.items() if k!='candidates'} for row in rows]),ensure_ascii=False))

