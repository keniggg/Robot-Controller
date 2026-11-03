#!/usr/bin/env python3
"""Read-only exact-source segmentation profiling. No ROS or control authority."""
from pathlib import Path
from types import SimpleNamespace
import cProfile
import hashlib
import json
import pstats
import time
import resource

import cv2
import numpy as np
from alicia_grasp_modes import tabletop as table

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen33_handoff_analysis')
OUT = ROOT / 'segmentation_resource_timing'
OUT.mkdir(exist_ok=False)
meta = json.loads((ROOT / 'exact_source_images/manifest.json').read_text())
rows = {r['role']: r for r in meta['sources']}
cv2.setNumThreads(1)
cfg = table.Config()
status = rows['original_anchor']['detector_status']['message']
anchor = SimpleNamespace(
    position_base=np.asarray(status['anchor_position_base']),
    extent_m=np.asarray(status['anchor_extent_m']),
    support_footprint_base=np.asarray(status['reference_support_footprint_base']),
    metrics={'median_height_m': status['reference_object_height_m']})
plane = np.asarray(status['reference_support_plane_base'])
last = SimpleNamespace(position_base=np.asarray(rows['last_good']['detector_status']['message']['position_base']))

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def digest(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()

def summary(result):
    return dict(plane=result.plane_base.tolist(), metrics=result.metrics,
                candidates=[dict(points=int(np.count_nonzero(c.mask)), bbox=c.bbox,
                                 position=c.position_base.tolist(), extent=c.extent_m.tolist(),
                                 mask_sha256=digest(c.mask), footprint_sha256=digest(c.support_footprint_base),
                                 metrics=c.metrics) for c in result.candidates])

events = []
stack = []
saved = {}

def wrap(owner, name):
    old = getattr(owner, name)
    saved[(owner, name)] = old
    def measured(*args, **kwargs):
        label = name
        if name == 'segment':
            label += ':current' if kwargs.get('fit_current_support', True) else ':locked'
        event = dict(name=label, parent=stack[-1] if stack else None)
        if name in ('_fit_plane', '_measured_support_footprint', 'svd'):
            event['input_shape'] = list(np.shape(args[0]))
        if name == '_depth_components':
            event['foreground_pixels'] = int(np.count_nonzero(args[0]))
        if name == 'floodFill':
            event['patch_shape'] = list(np.shape(args[0]))
        if name == '_fit_support_ring_sequence':
            event['full_frame_projection_provided'] = kwargs.get('points_camera') is not None
        index = len(events)
        events.append(event)
        stack.append(index)
        usage0 = resource.getrusage(resource.RUSAGE_THREAD)
        wall, cpu, process = time.perf_counter(), time.thread_time(), time.process_time()
        try:
            result = old(*args, **kwargs)
            if name == '_depth_components':
                event['output_component_pixels'] = [int(np.count_nonzero(v)) for v in result]
            if name == '_fit_plane':
                event['inlier_ratio'] = result[1]
            if name == '_fit_support_ring_sequence':
                event['ring_outer_px'] = result[-1]
            if name == 'segment':
                event['candidate_count'] = len(result.candidates)
            return result
        except Exception as exc:
            event['error'] = str(exc)
            raise
        finally:
            usage1 = resource.getrusage(resource.RUSAGE_THREAD)
            event['resource_delta'] = {k:getattr(usage1,k)-getattr(usage0,k) for k in ('ru_utime','ru_stime','ru_minflt','ru_majflt','ru_nvcsw','ru_nivcsw')}
            event.update(wall_sec=time.perf_counter()-wall, cpu_sec=time.thread_time()-cpu, process_cpu_sec=time.process_time()-process)
            stack.pop()
    setattr(owner, name, measured)

for name in ('_audit_current_support', '_fit_support_ring_sequence', '_fit_plane', '_project',
             '_depth_components', '_measured_support_footprint', 'segment'):
    wrap(table, name)
for name in ('floodFill', 'connectedComponentsWithStats'):
    wrap(cv2, name)
wrap(np.linalg, 'svd')

source_before = {table.__file__: sha(table.__file__)}
report = dict(scope='OFFLINE_EXACT_RGBD_PURE_SEGMENTATION_NO_AUTHORITY',
              source_before=source_before, config=vars(cfg), rows=[],
              limitations=[
                  'Exact recorded RGB-D, calibration and TF; configuration uses deployed source defaults, not a newly read runtime parameter server.',
                  'First-loss prior tracker position uses the exact last-good status; no fragment recovery occurred at this initial station.',
                  'Last-good prior tracker position is not exported separately; uses its own final position solely for matching routing. Locked/current segment geometry does not depend on last.',
                  'Offline timing has no online concurrent planning load and is not a latency guarantee.',
                  'Recorded failure mask is an unavailable notification, not an algorithm output.'])
for role in ('last_good', 'first_loss'):
    row = rows[role]
    a = np.load(row['npz'])
    camera = row['camera_metadata']['message']['parameters']
    depth = a['depth_raw'].astype(np.float32) * camera['depth_scale']
    K = [camera[k] for k in ('fx', 'fy', 'cx', 'cy')]
    transform = a['T_base_camera']
    entry = dict(role=role, source_stamp_ns=row['source_stamp_ns'], npz_sha256=sha(row['npz']), runs=[])
    for iteration in range(6):
        events.clear()
        wall, cpu, process = time.perf_counter(), time.thread_time(), time.process_time()
        try:
            result = table.segment_with_support_fallback(depth, K, transform, cfg, anchor, last, plane,
                                                         prefer_current_support=True)
            output = summary(result)
        except Exception as exc:
            output = dict(error=str(exc))
        run = dict(iteration=iteration, wall_sec=time.perf_counter()-wall,
                   cpu_sec=time.thread_time()-cpu, process_cpu_sec=time.process_time()-process, events=list(events), output=output)
        entry['runs'].append(run)
        print(json.dumps(dict(role=role, iteration=iteration, wall_sec=run['wall_sec'], cpu_sec=run['cpu_sec'],
                              worst=sorted(events,key=lambda e:e['wall_sec'],reverse=True)[:3],
                              result=output.get('error', len(output.get('candidates', []))))), flush=True)
    report['rows'].append(entry)
for (owner, name), old in saved.items():
    setattr(owner, name, old)

profile = cProfile.Profile()
profile.enable()
result = table.segment_with_support_fallback(depth, K, transform, cfg, anchor, last, plane, prefer_current_support=True)
profile.disable()
profile.dump_stats(str(OUT / 'first_loss.prof'))
with (OUT / 'first_loss_profile.txt').open('w') as f:
    pstats.Stats(profile, stream=f).sort_stats('cumulative').print_stats(50)
report['source_after'] = {p: sha(p) for p in source_before}
assert report['source_after'] == source_before
report['script_sha256'] = sha(__file__)
(OUT / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
print('RESULT', OUT / 'result.json', flush=True)
