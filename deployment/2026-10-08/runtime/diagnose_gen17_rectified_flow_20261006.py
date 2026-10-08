#!/usr/bin/env python3
"""Bounded saved-RGB flow diagnostic. Not a replay of raw SDK stationarity."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import zlib

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.runtime/realsense-2.56.5'))
sys.path.insert(0, str(ROOT / 'src/alicia_flexible_grasp_supervisor/src'))
import pyrealsense2 as sdk
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow
from sensor_msgs.msg import Image

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/gen17_support_measurement_20261006')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stats(values):
    if not len(values):
        return None
    return dict(zip(('min', 'p50', 'p90', 'max'), map(float, np.percentile(values, [0, 50, 90, 100]))))


def flow(reference, current, bounds):
    x0, y0, x1, y1 = bounds
    colors = [a[y0:y1, x0:x1] for a in (reference, current)]
    h, w = colors[0].shape[:2]
    ratio = min(1., VisualStationaryWindow.MAX_SIDE / float(max(h, w)))
    grays = [cv2.cvtColor(cv2.resize(a, (round(w*ratio), round(h*ratio)),
                                   interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY) for a in colors]
    scale = np.float32([w / float(grays[0].shape[1]), h / float(grays[0].shape[0])])
    window = VisualStationaryWindow()
    # Only reuse spatial seeding. No invented source clock, frame number or
    # continuous-stationary qualification is passed to observe().
    assert window._seed(grays[0], None, None)
    points = window._points
    kwargs = dict(winSize=(15, 15), maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, .01))
    forward, status, error = cv2.calcOpticalFlowPyrLK(grays[0], grays[1], points, None, **kwargs)
    backward, reverse, _ = cv2.calcOpticalFlowPyrLK(grays[1], grays[0], forward, None, **kwargs)
    fb = np.linalg.norm((backward-points).reshape(-1, 2)*scale, axis=1)
    xy = forward.reshape(-1, 2)
    valid = ((status.ravel() == 1) & (reverse.ravel() == 1)
        & (fb <= window.MAX_FORWARD_BACKWARD_PX)
        & (xy[:, 0] >= 0) & (xy[:, 0] < grays[0].shape[1])
        & (xy[:, 1] >= 0) & (xy[:, 1] < grays[0].shape[0]))
    motion = np.linalg.norm((forward-points).reshape(-1, 2)*scale, axis=1)
    rows = []
    for i, (before, after) in enumerate(zip(points.reshape(-1, 2), forward.reshape(-1, 2))):
        p0 = cv2.getRectSubPix(grays[0], (9, 9), tuple(map(float, before))).astype(float)
        p1 = cv2.getRectSubPix(grays[1], (9, 9), tuple(map(float, after))).astype(float)
        position = before*scale+np.array([x0, y0])
        end = after*scale+np.array([x0, y0])
        rows.append(dict(index=i, valid=bool(valid[i]),
            above_original_motion_threshold=bool(valid[i] and motion[i] > window.MAX_MOTION_PX),
            origin_uv=position.tolist(), current_uv=end.tolist(),
            motion_px=float(motion[i]), forward_backward_px=float(fb[i]),
            lk_l1_error=float(error.ravel()[i]),
            shifted_patch_mean_change=float(np.mean(p1-p0)),
            shifted_patch_absolute_change=float(np.mean(abs(p1-p0))),
            reference_patch_std=float(p0.std()),
            center_roi_quadrant=int(before[0] >= grays[0].shape[1]/2)+2*int(before[1] >= grays[0].shape[0]/2)))
    bad = [r for r in rows if r['above_original_motion_threshold']]
    good = [r for r in rows if r['valid'] and not r['above_original_motion_threshold']]
    return dict(seed_points=len(points), valid_tracks=int(valid.sum()),
        covered_quadrants=window._coverage(forward[valid], grays[0].shape),
        motion_p90_px=float(np.percentile(motion[valid], 90)),
        motion_median_px=float(np.median(motion[valid])),
        above_threshold_count=len(bad), above_threshold_quadrants=dict(Counter(r['center_roi_quadrant'] for r in bad)),
        all_valid_quadrants=dict(Counter(r['center_roi_quadrant'] for r in rows if r['valid'])),
        gray_whole_crop_mean_change=float(np.mean(grays[1].astype(float)-grays[0].astype(float))),
        gray_whole_crop_absolute_change=float(np.mean(abs(grays[1].astype(float)-grays[0].astype(float)))),
        point_group_metrics={label:{key:stats([r[key] for r in subset]) for key in
            ('forward_backward_px', 'lk_l1_error', 'shifted_patch_mean_change',
             'shifted_patch_absolute_change', 'reference_patch_std')}
            for label,subset in [('above_threshold',bad),('below_threshold',good)]}, points=rows)


def main():
    cv2.setNumThreads(1)
    assert Path('/media/zhuyupei/SanStand').is_mount()
    summary = json.loads((BASE / 'raw_support_v2/summary.json').read_text())
    assert summary['archive_closed'] and summary['complete_without_writer_error']
    stamps = summary['saved_pair_source_stamps_ns'][:24]
    assert len(stamps) == 24
    images, sources = {}, []
    with zipfile.ZipFile(BASE / 'raw_support_v2/diagnostic.zip') as archive:
        name = next(n for n in archive.namelist() if n.startswith('metadata/'))
        camera = json.loads(archive.read(name))['parameters']['/camera']
        contract = camera['runtime_profile']['rectification']
        rectifier = PairedPinholeRectification(contract['source_intrinsics'], sdk)
        assert rectifier.contract == contract
        for stamp in stamps:
            raw = zlib.decompress(archive.read('pairs/%d_color.rosmsg.zlib' % stamp))
            info = json.loads(archive.read('pairs/%d.json' % stamp))
            assert sha(raw) == info['streams']['color']['serialized_sha256']
            msg = Image().deserialize(raw)
            assert msg.header.stamp.to_nsec() == stamp and msg.encoding == 'bgr8'
            image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            images[stamp] = image.copy()
            sources.append(dict(source_stamp_ns=stamp, serialized_sha256=sha(raw),
                image_shape=list(image.shape), header_sequence=msg.header.seq))
    # A conservative inner rectangular crop in the rectified raster whose
    # inverse-map coordinates all lie inside the production raw-center ROI.
    valid = ((rectifier.map_x >= 128) & (rectifier.map_x < 512)
             & (rectifier.map_y >= 48) & (rectifier.map_y < 432))
    v, u = np.nonzero(valid)
    bounds = [int(u.min()), int(v.min()), int(u.max())+1, int(v.max())+1]
    while True:
        x0,y0,x1,y1 = bounds
        if valid[y0:y1, x0:x1].all():
            break
        bounds = [x0+1,y0+1,x1-1,y1-1]
    pairs = []
    for before, after in zip(stamps[:-1], stamps[1:]):
        row = flow(images[before], images[after], bounds)
        row.update(kind='adjacent_saved_publications', reference_stamp_ns=before, current_stamp_ns=after,
            saved_source_interval_sec=(after-before)/1e9)
        pairs.append(row)
    lastgood = 1791282095498089551
    pending = (1791282095700153589, 1791282095865248441, 1791282096096692800)
    for after in pending:
        row = flow(images[lastgood], images[after], bounds)
        row.update(kind='fixed_lastgood_to_pending', reference_stamp_ns=lastgood, current_stamp_ns=after,
            saved_source_interval_sec=(after-lastgood)/1e9)
        pairs.append(row)
    output = BASE / 'rectified_flow_diagnostic_v1'
    output.mkdir(exist_ok=False)
    report = dict(schema=1, diagnostic_only=True, ros_or_device_commands_sent=False,
        production_files_modified=False, source_sha256=sha(Path(__file__).read_bytes()),
        production_visual_source_sha256=sha((ROOT/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py').read_bytes()),
        sdk_version=sdk.__version__, cv_version=cv2.__version__, cv_threads=cv2.getNumThreads(),
        decoded_image_count=len(images), sources=sources,
        mapped_raw_center_xyxy=[128,48,512,432], rectified_inner_crop_xyxy=bounds,
        rectification_contract_digest=rectifier.contract['digest'], rows=pairs,
        limitations=[
            'These are actual saved ROS-published rectified RGB images, not raw SDK RGB inputs used by VisualStationaryWindow.',
            'Only the first 24 saved sources are decoded. Source intervals are original and approximately 0.2 seconds, not the SDK 30 Hz acquisition cadence.',
            'Spatial seeding, LK parameters, forward-backward validation and motion summary reuse production definitions; observe() is never called and no SDK frame identity or continuity is invented.',
            'The mapped inner rectangular crop, rectification resampling and sparse sources differ from the production raw-center crop and continuous immutable visual anchor. This is a location/photometric diagnostic, not a reproduction of gate failure.',
            'Large estimated motion and photometric differences alone do not identify actual scene motion, camera motion, illumination changes or tracking noise. No threshold, qualification or grasp authority is changed.'
        ])
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    worst = max((r for r in pairs if r['kind']=='adjacent_saved_publications'), key=lambda r:r['motion_p90_px'])
    chosen = [next(r for r in pairs if r['kind']=='fixed_lastgood_to_pending'), worst]
    fig,axes = plt.subplots(2,3,figsize=(12,7.5),constrained_layout=True)
    for axes_row,row in zip(axes,chosen):
        before=images[row['reference_stamp_ns']][:,:,::-1];after=images[row['current_stamp_ns']][:,:,::-1]
        axes_row[0].imshow(before);axes_row[0].set_title('Saved reference RGB')
        axes_row[1].imshow(after);axes_row[1].set_title('Saved current RGB (%.3f s later)'%row['saved_source_interval_sec'])
        axes_row[2].imshow(before)
        for point in row['points']:
            x,y=point['origin_uv'];color='red' if point['above_original_motion_threshold'] else ('lime' if point['valid'] else 'gray')
            axes_row[2].plot(x,y,'o',markersize=3,color=color)
            if point['above_original_motion_threshold']:
                axes_row[2].text(x+3,y-2,str(point['index']),fontsize=6,color='yellow')
        from matplotlib.patches import Rectangle
        x0,y0,x1,y1=bounds
        axes_row[2].add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor='cyan',linewidth=1))
        axes_row[2].set_title('Flow P90 %.3f px; >0.25: %d/%d'%(row['motion_p90_px'],row['above_threshold_count'],row['valid_tracks']))
        for ax in axes_row:ax.set_axis_off()
    fig.suptitle('Saved rectified RGB diagnostic only — not raw SDK gate replay\nRed: motion >0.25 px; green: other valid tracks; cyan: mapped inner center crop',fontsize=11)
    fig.savefig(output/'tracked_points.png',dpi=140)
    plt.close(fig)
    print(json.dumps(dict(output=str(output),report_sha256=sha((output/'report.json').read_bytes()),
        pairs=len(pairs),worst_p90=worst['motion_p90_px'],pending_p90=[r['motion_p90_px'] for r in pairs if r['kind']=='fixed_lastgood_to_pending'])))


if __name__=='__main__':
    main()
