"""Independent offline replay of unchanged measured support-plane quality gates.

Only saved arrays are read. No ROS, SDK, motion, or filter write is imported.
"""
import dataclasses
import hashlib
import json
import sys
from pathlib import Path

import cv2
import numpy as np

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
OUT = ROOT / 'source_plane_ring_after_async_20261003T121054Z'
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop as tt
CONFIG = tt.Config()
HASHES = {}


def use(path):
    path = Path(path).resolve()
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if str(path) in HASHES and HASHES[str(path)] != sha:
        raise RuntimeError('input changed: ' + str(path))
    HASHES[str(path)] = sha
    return path


def fit_diagnostic(points):
    # Identical production ordering, deterministic seed17, 128 RANSAC trials,
    # two SVD refinements, and original 2mm/70% thresholds.
    ids = np.linspace(0, len(points)-1,
                      min(len(points), CONFIG.support_max_samples), dtype=int)
    sampled = points[ids]
    rng = np.random.RandomState(17)
    best = np.zeros(len(sampled), bool)
    for _ in range(128):
        ids = rng.randint(len(sampled), size=3)
        if len(set(ids.tolist())) < 3:
            continue
        a, b, c = sampled[ids]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1e-9:
            continue
        n /= length
        inside = abs(np.einsum('ij,j->i', sampled-a, n)) <= CONFIG.support_threshold_m
        if inside.sum() > best.sum():
            best = inside
    if best.sum() < CONFIG.support_min_points:
        raise ValueError('support_plane_not_found')
    before = float(best.mean())
    for _ in range(2):
        center = sampled[best].mean(0)
        _, _, vh = np.linalg.svd(sampled[best]-center, full_matrices=False)
        n = vh[-1]
        d = -float(n@center)
        best = abs(np.einsum('ij,j->i', sampled, n)+d) <= CONFIG.support_threshold_m
    if d < 0:
        n, d = -n, -d
    sampled_residual = np.einsum('ij,j->i', sampled, n)+d
    full_residual = np.einsum('ij,j->i', points, n)+d
    ratio = float(best.mean())
    return dict(seed=17, sample_count=len(sampled),
        ransac_best_ratio_before_SVD=before, final_ratio=ratio,
        original_ratio_gate_pass=ratio >= CONFIG.support_min_ratio,
        original_2mm_sample_inlier_rmse_mm=float(np.sqrt(
            np.mean(sampled_residual[best]**2))*1000),
        full_ring_ratio=float(np.mean(abs(full_residual)<=CONFIG.support_threshold_m)),
        full_signed_residual_percentiles_mm=np.percentile(
            full_residual*1000,[0,5,25,50,75,95,100]).tolist(),
        full_residual_MAD_mm=float(np.median(abs(full_residual-np.median(full_residual)))*1000),
        plane_camera=np.r_[n,d].tolist())


def load_frame(name, path):
    with np.load(use(path), allow_pickle=False) as z:
        saved = {k:z[k].copy() for k in z.files}
    depth = (saved['depth_m'] if 'depth_m' in saved else
             (saved['depth_raw']*float(saved['depth_scale'])).astype(np.float32))
    K = saved['K']
    intrinsics = ([K[0,0],K[1,1],K[0,2],K[1,2]] if K.shape==(3,3) else K.tolist())
    return dict(name=name, input_path=str(path), depth=depth, K=intrinsics,
        rgb=saved['rgb'] if 'rgb' in saved else saved['image_bgr'],
        source_stamp_ns=int(saved['source_stamp_ns']) if 'source_stamp_ns' in saved else None,
        T_base_optical=saved.get('T_base_optical'))


def frame_review(frame):
    depth = frame['depth']
    projected = tt._project(depth, frame['K'])
    v,u = np.indices(depth.shape)
    rho = np.hypot(u-320,v-240)
    valid = np.isfinite(depth)&(depth>=CONFIG.depth_min_m)&(depth<=CONFIG.depth_max_m)
    rings = []
    for outer in (170,140,110):
        area = (rho>=70)&(rho<=outer)
        ring = valid&area
        points = projected[ring]
        review = fit_diagnostic(points)
        try:
            production = tt._fit_plane(points, CONFIG)
        except ValueError as exc:
            review['unmodified_production_result'] = str(exc)
            assert str(exc)=='support_plane_inlier_ratio_low'
            assert not review['original_ratio_gate_pass']
        else:
            review['unmodified_production_result'] = 'passed'
            assert abs(production[1]-review['final_ratio'])<1e-12
            assert np.max(abs(production[0]-review['plane_camera']))<1e-12
            assert review['original_ratio_gate_pass']
        review.update(outer_radius_px=outer, inner_radius_px=70,
            valid_ring_points=int(ring.sum()), geometric_ring_pixels=int(area.sum()),
            valid_depth_fraction=float(ring.sum()/area.sum()),
            depth_percentiles_5_50_95_m=np.percentile(depth[ring],[5,50,95]).tolist())
        rings.append(review)
    return dict(name=frame['name'], input_path=frame['input_path'],
        source_stamp_ns=frame['source_stamp_ns'], K=frame['K'], rings=rings)


def sample_patch(frame, uv):
    depth = frame['depth']
    values, errors = [], []
    for x,y in uv:
        ix,iy = int(round(x)),int(round(y))
        x0,y0 = max(ix-2,0),max(iy-2,0)
        patch = depth[y0:iy+3,x0:ix+3]
        py,px = np.indices(patch.shape)
        valid = (patch>=CONFIG.depth_min_m)&(patch<=CONFIG.depth_max_m)
        if valid.sum()<18:
            values.append(np.nan); errors.append(np.inf); continue
        design = np.column_stack([px[valid]+x0-x,py[valid]+y0-y,np.ones(valid.sum())])
        coeff = np.linalg.lstsq(design,1/patch[valid],rcond=None)[0]
        if coeff[2]<=0:
            values.append(np.nan); errors.append(np.inf); continue
        residual = 1/(design@coeff)-patch[valid]
        values.append(1/coeff[2])
        errors.append(np.sqrt(np.mean(residual**2)))
    return np.array(values),np.array(errors)


def temporal_review(a,b):
    v,u = np.indices(a['depth'].shape)
    rho = np.hypot(u-320,v-240)
    mask = ((rho>=70)&(rho<=170)).astype(np.uint8)*255
    g0 = cv2.cvtColor(a['rgb'],cv2.COLOR_BGR2GRAY)
    g1 = cv2.cvtColor(b['rgb'],cv2.COLOR_BGR2GRAY)
    p = cv2.goodFeaturesToTrack(g0,500,.01,10,mask=mask)
    q,status,_ = cv2.calcOpticalFlowPyrLK(g0,g1,p,None,winSize=(21,21),maxLevel=3)
    back,reverse,_ = cv2.calcOpticalFlowPyrLK(g1,g0,q,None,winSize=(21,21),maxLevel=3)
    selected = (status[:,0].astype(bool)&reverse[:,0].astype(bool)
                &(np.linalg.norm((p-back).reshape(-1,2),axis=1)<.3))
    uv0,uv1 = p[:,0][selected],q[:,0][selected]
    z0,e0 = sample_patch(a,uv0)
    z1,e1 = sample_patch(b,uv1)
    good = np.isfinite(z0)&np.isfinite(z1)&(e0<.0015)&(e1<.0015)
    assert good.any()
    flow = np.linalg.norm(uv1[good]-uv0[good],axis=1)
    dz = (z1[good]-z0[good])*1000
    return dict(source=a['name'], destination=b['name'],
        tracked_reciprocal_points=len(uv0), patch_quality_points=int(good.sum()),
        RGB_flow_percentiles_5_50_95_px=np.percentile(flow,[5,50,95]).tolist(),
        signed_depth_difference_percentiles_5_25_50_75_95_mm=np.percentile(dz,[5,25,50,75,95]).tolist(),
        depth_difference_MAD_mm=float(np.median(abs(dz-np.median(dz)))),
        depth_difference_abs_95_mm=float(np.percentile(abs(dz),95)),
        within_patch_RMSE_percentiles_5_50_95_mm=np.percentile(
            np.r_[e0[good],e1[good]]*1000,[5,50,95]).tolist())


def main():
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('canonical external drive missing; no fallback')
    cv2.setNumThreads(1)
    old_review = json.loads(use(ROOT/'source_plane_ring_review_20261003T110831Z/result.json').read_text())
    use(Path(tt.__file__)); use(Path(__file__))
    sequences = {}
    contexts = {}
    specs = [
        ('old','static_hold_20261003T092104Z',[1791019264907726287,1791019274976715326,1791019284912037849]),
        ('new','static_hold_20261003T121054Z',[1791029455729485750,1791029465829997539,1791029475832506179])]
    for sequence,dirname,stamps in specs:
        folder = ROOT/dirname
        result = json.loads(use(folder/'result.json').read_text())
        parameters = json.loads(use(folder/'camera_parameters.json').read_text())
        counts = json.loads(use(folder/'counts.json').read_text())
        use(folder/'events.jsonl')
        sequences[sequence] = [load_frame(sequence+'_'+name,
            folder/(str(stamp)+'_rgbd_tf.npz'))
            for name,stamp in zip(('hold_start','hold_10sec','hold_20sec'),stamps)]
        contexts[sequence] = dict(input_directory=str(folder),
            actual_change_deg=result['actual_change_deg'], initial_q_rad=result['initial_q_rad'],
            final_q_rad=result['final_q_rad'],sdk_q_rad=result.get('sdk_q_rad'),
            stationary_temporal_parameter_snapshot=parameters.get('runtime_profile',{}).get('stationary_temporal',{}),
            runtime_profile_parameter_snapshot=parameters.get('runtime_profile',{}),
            RGB_pairs_recorded=counts['counts'].get('/supervisor/camera/color/image_raw'),
            archive_writer=counts['archive_writer'])
    frames = [frame_review(frame) for seq in sequences.values() for frame in seq]
    old_expected = {f['name']:f for f in old_review['frames']}
    for frame in frames[:3]:
        expected = old_expected[frame['name'][4:]]
        for ring,prior in zip(frame['rings'],expected['rings']):
            assert abs(ring['final_ratio']-prior['final_ratio'])<1e-12
            assert ring['unmodified_production_result']==prior['unmodified_production_result']
    temporal = [temporal_review(seq[0],seq[i])
        for seq in sequences.values() for i in (1,2)]
    old_q = np.array(contexts['old']['initial_q_rad'])[:6]
    new_q = np.array(contexts['new']['initial_q_rad'])[:6]
    result = dict(method='Offline saved-array replay only; identical seed17/128RANSAC/twoSVD/2mm/70% production plane gate. Fixed pixel center(320,240),inner70,outer170/140/110. No rejected plane admitted; no gate or depth altered.',
        production_config=dataclasses.asdict(CONFIG), environment=dict(
            python=sys.version,numpy=np.__version__,opencv=cv2.__version__),
        frames=frames, hold_temporal_comparison=temporal,
        prior_review_summary=[dict(name=f['name'],ratios=[r['final_ratio'] for r in f['rings']],
            production_results=[r['unmodified_production_result'] for r in f['rings']])
            for f in old_review['frames']],
        sequence_context=contexts,
        between_sequences_encoder_pose_delta_deg=((new_q-old_q)*180/np.pi).tolist(),
        validations=dict(old_three_frames_exact_ratio_replay_matches=True,
            unchanged_production_crosscheck_for_all_18_ring_fits=True,
            no_ROS_SDK_device_commands=True),
        limitations=[
            'Each runtime active flag is a parameter snapshot,not a source-bound audit proving every saved depth frame had active temporal history.',
            'Only three saved pairs per20s hold; statistics are selected samples,not an exhaustive every-frame noise evaluation.',
            'Each sequence has unchanged encoder feedback; between old/new sequences first two encoder values differ by onequantum(~0.08789deg). This is not a controlled same-physical-pose causal experiment.',
            'KLT/patch qualification matches historical diagnostic. The point sets can differ between sequences; measurements characterize these saved samples and do not uniquely identify filter versus USB/optical/physical causes.',
            'A support-plane ratio in camera coordinates precedes TF; improved frame noise does not by itself settle the separate cross-view base-frame discrepancy.'
        ])
    for path in list(HASHES): use(path)
    result['input_sha256_by_path'] = dict(sorted(HASHES.items()))
    OUT.mkdir(exist_ok=False)
    (OUT/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
    (OUT/'sha256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()
        for p in OUT.iterdir() if p.is_file() and p.name!='sha256.json'},indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT/'result.json'), frames=[dict(name=f['name'],
        ratios=[r['final_ratio'] for r in f['rings']],
        full_ratios=[r['full_ring_ratio'] for r in f['rings']],
        MAD_mm=[r['full_residual_MAD_mm'] for r in f['rings']]) for f in frames],
        temporal=temporal, input_files_hashed=len(HASHES)),indent=2),flush=True)


if __name__=='__main__': main()
