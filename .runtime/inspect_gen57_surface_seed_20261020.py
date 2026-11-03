"""Read-only seed geometry observer on exact recorded inputs; no ROS/network."""
from pathlib import Path
import hashlib
import importlib.util
import os
import sys

sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = os.environ['OMP_NUM_THREADS'] = '1'
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
BASE = O / 'replay_masks_r2.py'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == '76ac1bd14f91c6b2c31fa4d7a1aea1a16d2a92a3eed09b1b7f65247713984482'
spec = importlib.util.spec_from_file_location('seed_geometry_base', BASE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
base.D = O / 'seed_geometry_diagnosis_r1'
import alicia_grasp_modes.measured_mask_seeds as seeds
import alicia_grasp_modes.rgb_instance as rgb_instance
original = seeds._confidence_labels
calls = []


def observe(seed, points, depth, heights, plane_normal, config, *, bgr, local_fit):
    result = original(seed, points, depth, heights, plane_normal, config, bgr=bgr, local_fit=local_fit)
    np, cv2 = base.np, base.cv2
    valid = (np.isfinite(depth) & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
             & np.all(np.isfinite(points), axis=-1))
    normals, reliable = local_fit(points, depth, valid)
    normal = plane_normal / np.linalg.norm(plane_normal)
    cosine = np.abs(normals @ normal)
    measured = seed & valid & (heights >= config.foreground_min_m) & (heights <= config.foreground_max_m)
    interior = cv2.erode(measured.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    fg, bg, report = result
    n = len(calls)
    np.savez_compressed(base.D / ('seed_%02d.npz' % n), seed=seed, points=points, depth=depth,
        heights=heights, normal=plane_normal, bgr=bgr, normals=normals, reliable=reliable,
        cosine=cosine, interior=interior, foreground=fg, background=bg)
    rows=[]
    for subset in np.array_split(np.arange(seed.shape[1]), 4):
        area=np.zeros(seed.shape,bool); area[:,subset]=True
        m=interior & area
        rows.append(dict(interior_pixels=int(m.sum()),fg_pixels=int((fg & area).sum()),
            reliable_pixels=int((m & reliable).sum()),height_quantiles=np.percentile(heights[m],[0,10,50,90,100]).tolist() if m.any() else [],
            cosine_quantiles=np.percentile(cosine[m & reliable],[0,10,50,90,100]).tolist() if (m & reliable).any() else []))
    calls.append(dict(index=n,report=report,quarters=rows))
    base.write('seed_%02d.json' % n, calls[-1])
    return result


seeds._confidence_labels = rgb_instance._confidence_labels = observe
base.main()
