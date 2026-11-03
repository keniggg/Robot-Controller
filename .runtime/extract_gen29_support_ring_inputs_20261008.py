#!/usr/bin/env python3
"""Offline exact ring extraction; lower ratio is reporting-only, never admission."""
import ast
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge
from alicia_grasp_modes import tabletop as module

ROOT = Path(__file__).resolve().parents[1]
RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261008T110323.888418Z_6dd99ae4ff4340a6a35d509899eb6b85')
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z/gen29_support_diagnosis/baseline_ring_inputs')
FINAL = 1791457418721812009


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not OUT.exists()
    manifest = json.loads((RECORD/'manifest.json').read_text())
    assert manifest['recorder_closed'] and manifest['state'] == 'complete'
    statuses, depths, cameras, dynamic, static = [], {}, [], [], []
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for t, m, _ in bag.read_messages(topics=['/perception/unknown/detector_status',
            '/supervisor/camera/depth/image_raw', '/grasp_record/camera_runtime_metadata', '/tf', '/tf_static']):
            if t.endswith('detector_status'):
                d = json.loads(m.data)
                if d.get('generation') == 29 and 0 < d.get('source_stamp_ns', 0) <= FINAL:
                    statuses.append(d)
            elif t.endswith('depth/image_raw'):
                ns = m.header.stamp.to_nsec()
                if ns <= FINAL:
                    depths[ns] = m
            elif t.endswith('camera_runtime_metadata'):
                d = json.loads(m.data)
                if d.get('status') == 'available': cameras.append(d['parameters'])
            elif t == '/tf_static': static.extend(m.transforms)
            else: dynamic.extend(m.transforms)
    keys = ('fx', 'fy', 'cx', 'cy', 'width', 'height', 'depth_scale', 'frame_id')
    camera = {k: cameras[0][k] for k in keys}
    assert all({k: c[k] for k in keys} == camera for c in cameras)
    pending = [d for d in statuses if d['detail'] == 'support_measurement_pending_revalidation']
    selected_ns = {d['source_stamp_ns'] for d in pending}
    selected_ns.update(d['source_qualification']['reference_source_stamp_ns'] for d in pending)
    selected_ns.add(FINAL)
    acquired = next(d for d in statuses if d['detail'] == 'acquired')
    selected_ns.add(acquired['source_stamp_ns'])
    anchor = np.array(acquired['anchor_position_base'])
    reference = np.array(acquired['reference_support_plane_base'])
    buffer = tf2_py.BufferCore(rospy.Duration(300.))
    for t in static: buffer.set_transform_static(t, 'sealed_gen29')
    for t in sorted(dynamic, key=lambda t:t.header.stamp.to_nsec()): buffer.set_transform(t, 'sealed_gen29')
    producer = ROOT/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
    tree = ast.parse(producer.read_text()); function = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'transform_matrix')
    namespace = {'np': np}
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(producer), 'exec'), namespace)
    cfg = module.Config(); diagnostic_cfg = replace(cfg, support_min_ratio=0.)
    intrinsics = [camera[k] for k in ('fx', 'fy', 'cx', 'cy')]
    bridge = CvBridge(); rows = []
    OUT.mkdir(parents=True, exist_ok=False)
    for status in statuses:
        ns = status['source_stamp_ns']
        if ns not in selected_ns: continue
        assert ns in depths
        msg = depths[ns]
        raw = np.asarray(bridge.imgmsg_to_cv2(msg, 'passthrough'))
        assert raw.dtype == np.uint16
        depth = np.asarray(raw, np.float32) * float(camera['depth_scale'])
        transform = namespace['transform_matrix'](buffer.lookup_transform_core('base_link', 'camera_link', msg.header.stamp))
        center = module.project_base_to_pixel(anchor, transform, intrinsics)
        v, u = np.indices(depth.shape)
        scale = min(depth.shape[1]/640., depth.shape[0]/480.)
        radius = np.hypot(u-center[0], v-center[1])
        valid = np.isfinite(depth) & (depth >= cfg.depth_min_m) & (depth <= cfg.depth_max_m)
        arrays = dict(raw_depth=raw, depth_m_float32=depth, transform_base_camera=transform,
                      center_px=center, anchor_base=anchor, reference_plane_base=reference)
        row = dict(source_stamp_ns=ns, detail=status['detail'], center_px=center.tolist(),
                   raw_depth_sha256=hashlib.sha256(np.ascontiguousarray(raw).tobytes()).hexdigest(),
                   recorded_current_ratio=(status.get('geometry_metrics') or {}).get('support_current_audit_inlier_ratio'),
                   recorded_current_outer=(status.get('geometry_metrics') or {}).get('support_current_audit_ring_outer_radius_px'), rings=[])
        for outer in (170, 140, 110):
            ring = valid & (radius >= 70*scale) & (radius <= outer*scale)
            ys, xs = np.nonzero(ring)
            points = module._project_pixels(depth[ring], xs, ys, intrinsics)
            indices = np.linspace(0, len(points)-1, min(len(points), cfg.support_max_samples), dtype=int)
            samples = points[indices]
            arrays.update({f'r{outer}_points':points, f'r{outer}_sample_points':samples,
                           f'r{outer}_uv':np.column_stack((xs, ys)),f'r{outer}_sample_indices':indices})
            plane, ratio, residual, count = module._fit_plane(points, diagnostic_cfg)
            distance = points@plane[:3]+plane[3]
            fullin = abs(distance) <= cfg.support_threshold_m
            quadrants = {}
            for left in (True,False):
                for top in (True,False):
                    selected = (xs<center[0] if left else xs>=center[0]) & (ys<center[1] if top else ys>=center[1])
                    quadrants[str((left,top))] = dict(points=int(selected.sum()), inliers=int((selected&fullin).sum()))
            row['rings'].append(dict(outer_radius_px=outer*scale,nominal_outer=outer,
                all_points=len(points),sample_count=count,best_count=int(round(ratio*count)),
                original_fit_ratio=ratio,original_fit_residual_m=residual,original_fit_plane_camera=plane.tolist(),
                original_ratio_gate_passes=bool(ratio>=cfg.support_min_ratio),
                same_plane_full_points_ratio=float(fullin.mean()),quadrants=quadrants,
                signed_distance_percentiles_m=np.percentile(distance,[0,5,10,25,50,75,90,95,100]).tolist()))
        np.savez_compressed(OUT/(str(ns)+'.npz'),**arrays)
        row['npz_sha256']=sha(OUT/(str(ns)+'.npz'));rows.append(row)
        print(json.dumps(dict(ns=ns,detail=row['detail'],ratios=[round(r['original_fit_ratio'],5) for r in row['rings']],full=[round(r['same_plane_full_points_ratio'],5) for r in row['rings']])),flush=True)
    report=dict(scope='read_only_original_ring_inputs_and_rejected_fit_diagnostics',execution_authorized=False,
        original_config=asdict(cfg),reporting_only_ratio_override=0.,original_gate_applied_to_report=True,
        note='Ratio override only exposes rejected model numeric values. No frame is admitted by this diagnostic; original ratio .70 and distance .002 gates are unchanged.',
        source_bag=str(RECORD/'keyframes.bag'),bag_manifest=next(f for f in manifest['files'] if f['path']=='keyframes.bag'),
        camera=camera,camera_metadata_projection_consistent_count=len(cameras),
        producer_source_sha256=sha(producer),tabletop_source_sha256=sha(ROOT/'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'),
        script_sha256=sha(__file__),anchor=acquired,rows=rows)
    (OUT/'meta.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(output=str(OUT),rows=len(rows),meta_sha256=sha(OUT/'meta.json'))))


if __name__ == '__main__': main()
