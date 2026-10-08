#!/usr/bin/env python3
"""Fixed-source offline LK boundary and coherent-motion diagnostics; no ROS."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np

W = Path(__file__).resolve().parents[1]
ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/regional_production_20261007T011805.778547Z')
REPLAYER = W/'.runtime/replay_raw_visual_snapshots_20261006.py'
spec = importlib.util.spec_from_file_location('frozen_context_replayer', REPLAYER)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentiles(values):
    values = np.asarray(values)
    return dict(count=len(values), median=float(np.median(values)), p90=float(np.percentile(values,90)),
                maximum=float(np.max(values)))


def context_gray(rgb, meta):
    if meta['region'] == 'center':
        # The source-grid offset is 120/48 pixels: both multiples of six.
        # 5/6 scale gives integer offset100/40, also multiples of4 so pyramid
        # phase matches at all original three LK levels. No artificial pixels.
        assert rgb.shape == (480,640,3)
        crop = np.ascontiguousarray(rgb[:480,8:638])
        full = cv2.cvtColor(cv2.resize(crop,(525,400),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2GRAY)
        offset = np.float32([100,40])
        bounds = [8,0,638,480]
    else:
        assert meta['region'] == 'nested_center'
        full = cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
        offset = np.float32(meta['bounds_xyxy'][:2])
        bounds = [0,0,rgb.shape[1],rgb.shape[0]]
    return full,offset,bounds


def predict(params, model, p):
    design = np.column_stack([p,np.ones(len(p))])
    if model == 'translation':
        return p+params
    if model == 'affine':
        return design@params
    h = np.r_[params,1.].reshape(3,3)
    result = design@h.T
    if np.any(np.abs(result[:,2])<1e-9):
        raise ValueError('singular_projective_prediction')
    return result[:,:2]/result[:,2:]


def fit_model(model, p, q):
    if model == 'translation':
        return np.mean(q-p,axis=0)
    if model == 'affine':
        design = np.column_stack([p,np.ones(len(p))])
        coef,_,rank,_ = np.linalg.lstsq(design,q,rcond=None)
        assert rank == 3
        return coef
    x,y = p.T
    u,v = q.T
    ones,zero = np.ones(len(p)),np.zeros(len(p))
    a = np.empty((2*len(p),8))
    a[::2] = np.column_stack([x,y,ones,zero,zero,zero,-u*x,-u*y])
    a[1::2] = np.column_stack([zero,zero,zero,x,y,ones,-v*x,-v*y])
    _,_,rank,_ = np.linalg.lstsq(a,q.ravel(),rcond=None)
    assert rank == 8
    # method=0 uses all points, then refines geometric error by LM; no RANSAC
    # or residual-based point selection. Assert its mask retains every point.
    h,mask = cv2.findHomography(p,q,method=0)
    assert h is not None and np.isfinite(h).all() and abs(h[2,2])>1e-12
    assert mask is not None and np.all(mask==1)
    return (h/h[2,2]).ravel()[:8]


def coherent_models(points, vectors, quadrants):
    origin = np.mean(points,axis=0)
    normalizer = float(max(np.ptp(points,axis=0)))
    assert normalizer>0
    p = (points-origin)/normalizer
    q = p+vectors/normalizer
    # Predetermined per-quadrant splits use geometry only, never flow/residual.
    fold = np.empty(len(p),int)
    rng = np.random.RandomState(2707)
    for cell in range(4):
        ids = np.flatnonzero(quadrants==cell)
        rng.shuffle(ids)
        fold[ids] = np.arange(len(ids))%5
    results = {}
    for model in ('translation','affine','homography'):
        params = fit_model(model,p,q)
        fitted = predict(params,model,p)
        residual = (q-fitted)*normalizer
        cv = np.zeros_like(residual)
        for k in range(5):
            test = fold==k
            assert test.any() and (~test).sum()>=8
            partial = fit_model(model,p[~test],q[~test])
            cv[test] = (q[test]-predict(partial,model,p[test]))*normalizer
        results[model] = dict(parameter_count={'translation':2,'affine':6,'homography':8}[model],
            normalized_fit_parameters=params.tolist(), training_residual_full_px=percentiles(np.linalg.norm(residual,axis=1)),
            cross_validated_residual_full_px=percentiles(np.linalg.norm(cv,axis=1)),
            fitted_motion_full_px=percentiles(np.linalg.norm((fitted-p)*normalizer,axis=1)),
            cv_residual_fraction_above_original_025=float(np.mean(np.linalg.norm(cv,axis=1)>.25)))
    return dict(original_valid_points=len(p), full_pixel_origin=origin.tolist(), normalization_scale=normalizer,
        fold_assignment=fold.tolist(), methods=results,
        semantics='Least squares on every original valid track; five geometry-stratified holdout folds. Residuals do not authorize stationarity.')


def self_check():
    rng = np.random.RandomState(3)
    p = rng.uniform(-1,1,(64,2))
    affine = np.array([[1.002,.003],[-.004,.998],[.002,-.003]])
    q = predict(affine,'affine',p)
    assert np.max(np.abs(predict(fit_model('affine',p,q),'affine',p)-q))<1e-12
    h = np.array([1.002,-.004,.002,.003,.998,-.003,.002,-.003])
    q = predict(h,'homography',p)
    assert np.max(np.abs(predict(fit_model('homography',p,q),'homography',p)-q))<1e-6
    cells = (p[:,0]>=0).astype(int)+2*(p[:,1]>=0)
    models = coherent_models(p*100,(q-p)*100,cells)
    assert models['methods']['homography']['cross_validated_residual_full_px']['maximum']<1e-4


def review(arrays,meta):
    crop,current,scale = m.validate_snapshot(arrays,meta)
    anchor,points = arrays['anchor_gray'],arrays['seed_points']
    ca,offset,bounds = context_gray(arrays['anchor_raw_color'],meta)
    cb,other,_ = context_gray(arrays['raw_color'],meta)
    assert np.array_equal(offset,other) and np.all(offset%4==0)
    x,y = map(int,offset)
    inside_a = ca[y:y+anchor.shape[0],x:x+anchor.shape[1]]
    inside_b = cb[y:y+current.shape[0],x:x+current.shape[1]]
    assert np.array_equal(inside_a,anchor) and np.array_equal(inside_b,current)
    before = m.lk(anchor,current,points,scale)
    after = m.lk(ca,cb,points+offset,scale)
    original_xy = (after['forward']-offset).reshape(-1,2)
    after['valid'] &= ((original_xy[:,0]>=0)&(original_xy[:,0]<current.shape[1])
                       &(original_xy[:,1]>=0)&(original_xy[:,1]<current.shape[0]))
    keep = before['valid']
    seed = points.reshape(-1,2)
    raw = (seed+.5)*scale-.5+np.asarray(meta['bounds_xyxy'][:2])
    cells = (seed[:,0]>=anchor.shape[1]/2).astype(int)+2*(seed[:,1]>=anchor.shape[0]/2)
    intersection = keep&after['valid']
    covered = m.V._coverage((after['forward']-offset)[after['valid']],current.shape)
    original_coverage_eligible = int(after['valid'].sum())>=max(m.V.MIN_POINTS,int(np.ceil(.8*len(points)))) and covered>=3
    original_coverage_metrics = dict(tracked_points=int(after['valid'].sum()), covered_original_roi_quadrants=covered,
        tracks_eligible=original_coverage_eligible,
        motion_p90_px=float(np.percentile(after['motion'][after['valid']],90)) if original_coverage_eligible else None)
    original_coverage_metrics['motion_gate_passed'] = bool(original_coverage_eligible and original_coverage_metrics['motion_p90_px']<=.25)
    edge = np.minimum.reduce([seed[:,0],seed[:,1],anchor.shape[1]-1-seed[:,0],anchor.shape[0]-1-seed[:,1]])*min(scale)
    row = dict(region=meta['region'],category=meta['category'],source=meta['source'],
        original_return=meta['original_return'], source_anchor=meta['pre_observe']['anchor_source'],
        original_lk=before['metrics'],
        expanded_context=dict(raw_bounds_xyxy=bounds,gray_shape=list(ca.shape), seed_offset_gray=offset.tolist(),
            scale_full_input=scale.tolist(), original_roi_gray_both_images_bitwise_equal=True,
            pyramid_phase_matches_through_level_2=True, seeds_unchanged_except_translation=True,
            original_roi_coverage_metrics=original_coverage_metrics,
            original_valid_count=int(keep.sum()), original_valid_still_valid=int(intersection.sum()),
            all_original_valid_retained=bool(np.all(after['valid'][keep])),
            motion_p90_on_original_valid=float(np.percentile(after['motion'][keep],90)) if np.all(after['valid'][keep]) else None,
            vector_change_full_px=percentiles(np.linalg.norm(after['vectors'][intersection]-before['vectors'][intersection],axis=1)),
            vectors_full_px_all_original_seeds=after['vectors'].tolist(), validity_all_original_seeds=after['valid'].tolist()),
        boundary_diagnostic=[dict(minimum_original_crop_edge_distance_full_px=float(threshold),
            count=int(np.count_nonzero(keep&(edge<threshold))),
            original_above_025=int(np.count_nonzero(keep&(edge<threshold)&(before['motion']>.25)))) for threshold in (10,20,40)],
        coherent_motion=coherent_models(raw[keep],before['vectors'][keep],cells[keep]),
        original_valid_vectors_full_px=before['vectors'][keep].tolist(),original_valid_seed_raw_uv=raw[keep].tolist())
    return row


def visualize(output,name,arrays,meta,row):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    pts = np.asarray(row['original_valid_seed_raw_uv'])
    vectors = np.asarray(row['original_valid_vectors_full_px'])
    magnitude = np.linalg.norm(vectors,axis=1)
    fig,axes = plt.subplots(1,2,figsize=(12.5,5.0),dpi=150)
    for ax,rgb,label in zip(axes,[arrays['anchor_raw_color'],arrays['raw_color']],['Actual anchor raw RGB','Current raw RGB + original valid LK vectors']):
        ax.imshow(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB))
        x0,y0,x1,y1=meta['bounds_xyxy'];ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,color='cyan',linewidth=1.2))
        ax.set_title(label);ax.set_xlabel('Full raw image x / px');ax.set_ylabel('y / px')
    axes[1].quiver(pts[:,0],pts[:,1],vectors[:,0]*35,vectors[:,1]*35,magnitude,
        angles='xy',scale_units='xy',scale=1,cmap='inferno',clim=(0,.6),width=.0035)
    axes[1].scatter(pts[:,0],pts[:,1],s=7,c=magnitude,cmap='inferno',vmin=0,vmax=.6)
    fig.suptitle('%s | %s | arrows magnified 35x; no motion compensation applied\nOriginal P90 %.3f px; context P90 %.3f px; affine holdout residual P90 %.3f px' % (
        name,meta['region'],row['original_lk']['motion_p90_px'],row['expanded_context']['original_roi_coverage_metrics']['motion_p90_px'],
        row['coherent_motion']['methods']['affine']['cross_validated_residual_full_px']['p90']),fontsize=10)
    fig.tight_layout();fig.savefig(output/(name+'.png'));plt.close(fig)


def main():
    cv2.setNumThreads(1)
    self_check()
    capture = ROOT/'visual_snapshots'
    summary,entries = m.sealed_entries(capture)
    inputs = {str(capture/'summary.json'):digest(capture/'summary.json'), str(REPLAYER):digest(REPLAYER),
              str(m.VISUAL_PATH):digest(m.VISUAL_PATH),str(Path(__file__).resolve()):digest(Path(__file__).resolve())}
    rows=[];pictures={}
    for entry in entries:
        path = capture/entry['name']
        assert digest(path)==entry['sha256']
        inputs[str(path)]=entry['sha256']
        with np.load(path,allow_pickle=False) as a:
            arrays={k:a[k].copy() for k in a.files}
        meta=json.loads(arrays['metadata_json'].tobytes())
        assert meta['source']==entry['source'] and meta['source_implementation']==summary['source_implementation']
        row=review(arrays,meta);row['file']=entry['name'];rows.append(row)
        if meta['category']=='visual_motion' and meta['region'] not in pictures:
            pictures[meta['region']]=(entry['name'],arrays,meta,row)
    assert all(digest(Path(p))==expected for p,expected in inputs.items())
    report=dict(schema=1,diagnostic_only=True,motion_authorized=False,production_modified=False,inputs_sha256=inputs,
        opencv_version=cv2.__version__,opencv_threads=cv2.getNumThreads(),self_check_passed=True,rows=rows,
        region_counts=dict(Counter(x['region']+'/'+x['category'] for x in rows)),
        methods=dict(context='Actual source RGB outside original crop; original ROI grayscale required bitwise identical in both images; original seeds translated only; pyramid offset divisible by4.',
            model='Ordinary least squares affine; normalized coordinates with OpenCV findHomography(method=0), all-point solution and LM geometric refinement. No RANSAC, no flow-dependent point removal. Five deterministic geometry-stratified held-out folds.',
            context_coverage='Coverage evaluated in ORIGINAL crop quadrants; expanded-image quadrant metric is not used as original permission.'),
        limitations=['No diagnostic residual grants stationarity; original motion remains recorded and no image is altered for production.',
            'Only selected visual examples are replayed; held-out geometric residuals do not establish physical vibration or image noise.',
            'A good coherent model may reflect real motion or image formation; a poor one does not prove sensor noise.',
            'This diagnostic contains no support-plane or grasp validation; pose/light differ from earlier experiments.'])
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output=ROOT/'context_motion_diagnostic_v1';output.mkdir(exist_ok=False)
    for region,(name,arrays,meta,row) in pictures.items():
        visualize(output,name.replace('.npz',''),arrays,meta,row)
    with (output/'report.json').open('x') as f:
        json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=digest(output/'report.json'),rows=len(rows),figures=len(pictures))))


if __name__=='__main__':
    main()
