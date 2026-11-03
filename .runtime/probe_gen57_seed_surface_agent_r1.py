"""Finite offline geometry exploration; no admission rule or deployment."""
from pathlib import Path
import hashlib
import json
import os
import resource
import sys
import time

os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
I = O / 'seed_geometry_diagnosis_r1'
D = O / 'seed_surface_exploration_agent_r1'
sys.path[:0] = [str(W/'src/alicia_grasp_modes/src')]
def audit(event, args):
    if event in ('socket.connect', 'socket.bind', 'socket.sendto', 'subprocess.Popen', 'os.system'):
        raise RuntimeError('offline probe rejects ' + event)
sys.addaudithook(audit)
import cv2
import numpy as np
from alicia_grasp_modes import measured_mask_seeds as seeds
from alicia_grasp_modes.tabletop import Config, _project, _depth_components

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def explore(d):
    cfg = Config()
    cutoff = seeds.LOCAL_SURFACE_MAX_RMS_M
    separated = cfg.foreground_min_m + 2 * cfg.support_threshold_m
    eligible = d['interior'] & d['reliable'] & (d['cosine'] >= np.sqrt(.5))
    upper = float(np.percentile(d['heights'][d['seed']], 90.))
    anchors = d['foreground'] & (d['heights'] >= upper)
    n, ids = cv2.connectedComponents(eligible.astype(np.uint8), connectivity=8)
    strict, bridge = np.zeros_like(eligible), np.zeros_like(eligible)
    rows = []
    for i in range(1, n):
        region = ids == i
        points = d['points'][region]
        center = points.mean(0)
        centered = points-center
        eigen, vectors = np.linalg.eigh(centered.T@centered/len(points))
        normal = vectors[:, 0]
        rms = float(np.sqrt(max(0., eigen[0])))
        count = int(np.count_nonzero(region & anchors))
        report = dict(pixels=int(region.sum()), original_upper_anchors=count,
            eigenvalues_m2=eigen.tolist(), rms_m=rms,
            minimum_height_m=float(d['heights'][region].min()))
        reasons = []
        if count < 25: reasons.append('fewer_than_25_original_upper_anchors')
        if eigen[1] < 1e-8: reasons.append('surface_rank_unproven')
        if rms > cutoff: reasons.append('global_plane_residual_exceeds_existing_local_scale')
        if d['heights'][region].min() < separated: reasons.append('not_fully_separated_from_support')
        if abs(float(normal@d['normal']/np.linalg.norm(d['normal']))) < np.sqrt(.5):
            reasons.append('surface_not_near_parallel_to_support')
        report['rejection_reasons'] = reasons
        if not reasons:
            residual = np.abs((d['points']-center)@normal)
            strict_i = region & (residual <= cutoff)
            strict |= strict_i
            # Only current measured seed points connect a fixed plane. Edge
            # points never become hard seeds, and the model is never refit.
            path = (d['seed'] & (residual <= cutoff)
                & (d['heights'] >= separated))
            reachable = np.zeros_like(path)
            for component in _depth_components(path.astype(np.uint8), d['depth'], 1,
                                               cfg.depth_edge_m, connectivity=8):
                if np.count_nonzero(component & anchors & region) >= 25:
                    reachable |= component
            bridge_i = reachable & eligible
            bridge |= bridge_i
            report.update(strict_inlier_pixels=int(strict_i.sum()),
                measured_path_reachable_pixels=int(reachable.sum()),
                reliable_reachable_pixels=int(bridge_i.sum()),
                reachable_outside_original_component=int(np.count_nonzero(bridge_i & ~region)),
                fitted_plane_camera=[*normal.tolist(), -float(center@normal)])
        rows.append(report)
    yy, xx = np.indices(eligible.shape)
    midpoint = float(np.median(xx[d['interior']]))
    output = {}
    masks = {}
    for name, addition in [('strict_reliable_component', strict),
                           ('fixed_plane_current_measured_bridge', bridge)]:
        proposed = d['foreground'] | addition
        demoted, evidence = seeds._side_seed_colour_evidence(d['bgr'], proposed,
            d['background'], d['heights'], upper, d['cosine'])
        final = d['foreground'] | demoted
        new = final & ~d['foreground']
        output[name] = dict(proposed_total=int(proposed.sum()), total=int(final.sum()),
            new_pixels=int(new.sum()), left_half_new_pixels=int(np.count_nonzero(new&(xx<midpoint))),
            new_pixels_without_valid_depth=int(np.count_nonzero(new & ~np.isfinite(d['depth']))),
            new_pixels_outside_measured_interior=int(np.count_nonzero(new&~d['interior'])),
            colour_demoted_proposal_pixels=int(np.count_nonzero(proposed&~final)),
            colour_evidence=evidence)
        masks[name] = final
    return dict(regions=rows, variants=output, original_foreground_pixels=int(d['foreground'].sum()),
        reliable_interior_pixels=int(eligible.sum()), left_half_boundary_x=midpoint), masks

def fixture_data(f, index):
    depth = f['depth'][index].astype(np.float32)*float(f['depth_scale'])
    points = _project(depth, f['intrinsics'])
    plane = f['T'][index].T @ f['plane'][index]
    heights = points@plane[:3]+plane[3]
    cfg = Config()
    valid = (np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
             &np.all(np.isfinite(points),axis=-1))
    seed = (f['mask'][index]>0)&valid&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)
    fg,bg,_ = seeds.confidence_labels(seed,points,depth,heights,plane[:3],cfg,bgr=f['bgr'][index])
    normals,reliable = seeds.local_surface_normals(points,depth,valid)
    return dict(seed=seed,points=points,depth=depth,heights=heights,normal=plane[:3],
        bgr=f['bgr'][index],foreground=fg,background=bg,
        interior=cv2.erode(seed.astype(np.uint8),np.ones((3,3),np.uint8))>0,
        reliable=reliable,cosine=np.abs(normals@plane[:3]/np.linalg.norm(plane[:3])))

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    D.mkdir(exist_ok=False)
    cv2.setNumThreads(1)
    start=time.monotonic();usage=resource.getrusage(resource.RUSAGE_SELF)
    inputs={};rows=[]
    for name in ('seed_01','seed_03'):
        path=I/(name+'.npz');inputs[str(path)]=sha(path)
        with np.load(path,allow_pickle=False) as f: d=dict(f)
        row,masks=explore(d);row['name']=name;rows.append(row)
        np.savez_compressed(D/(name+'_proposals.npz'),**masks)
    negative_rows=[]
    fixtures=W/'src/alicia_grasp_modes/tests/fixtures'
    for filename in ('shadow_jump_20260923.npz','short_side_rgbd_20260925.npz'):
        path=fixtures/filename;inputs[str(path)]=sha(path)
        with np.load(path,allow_pickle=False) as f:
            for index in range(3):
                d=fixture_data(f,index);row,masks=explore(d)
                forbidden=np.zeros(d['seed'].shape,bool)
                if filename.startswith('shadow'):
                    forbidden[299:309,321:362]=True;forbidden[235:280,374:383]=True
                else:
                    forbidden[33:65,20:34]=True;forbidden[33:65,69:81]=True
                row.update(name=filename,index=index,known_support_evaluation={k:dict(
                    new_hard_seed_pixels=int(np.count_nonzero(v&~d['foreground']&forbidden)),
                    total_hard_seed_pixels=int(np.count_nonzero(v&forbidden))) for k,v in masks.items()})
                negative_rows.append(row)
    usage2=resource.getrusage(resource.RUSAGE_SELF)
    result=dict(scope='Finite geometric proposal exploration only; no mask replay or runtime qualification',
        rows=rows,negative_fixture_rows=negative_rows,input_sha256=inputs,
        helper_sha256=sha(__file__),cutoffs=dict(plane_residual_m=seeds.LOCAL_SURFACE_MAX_RMS_M,
            min_original_upper_anchors=25,min_rank_second_eigenvalue_m2=1e-8,
            min_support_height_m=Config().foreground_min_m+2*Config().support_threshold_m),
        limitations=['No threshold sweep, colour selection, historical-pixel selection, or hole filling.',
            'Hard seeds remain a subset of reliable eroded current measured seed; missing-depth RGB regions cannot gain hard foreground.',
            'Connectivity is evidence only: it does not prove same-object identity or final mask acceptance.',
            'Known-support patch counts evaluate seed geometry only, not full negative mask or downstream checks.',
            'Original appearance, geometry, station freshness and motion are not exercised.'],
        wall_seconds=time.monotonic()-start,user_cpu_seconds=usage2.ru_utime-usage.ru_utime,
        system_cpu_seconds=usage2.ru_stime-usage.ru_stime,max_rss_kib=usage2.ru_maxrss,
        all_inputs_unchanged=all(sha(p)==digest for p,digest in inputs.items()))
    with (D/'result.json').open('x') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    print(json.dumps(dict(path=str(D/'result.json'),rows=[dict(name=r['name'],variants=r['variants']) for r in rows],
        negatives=[dict(name=r['name'],index=r['index'],known_support_evaluation=r['known_support_evaluation']) for r in negative_rows],
        wall_seconds=result['wall_seconds']),ensure_ascii=False))

if __name__=='__main__': main()
