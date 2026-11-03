#!/usr/bin/env python3
"""Verify and analyse finite same-read depth taps, without ROS or device access.

The native depth grid is deliberately excluded from pixelwise comparisons with
the aligned color grid.  Rectified ROIs are pulled back through the exact,
hash-verified injective depth map, including its fixed empty canvas cells.
"""
import argparse
import hashlib
import importlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import re
import resource
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import numpy as np

WORKTREE = Path(__file__).resolve().parents[2]
MOUNT = Path('/media/zhuyupei/SanStand')
STORE = MOUNT / '实验记录/Robot-Controller'
RECTIFIER = WORKTREE / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py'


def require(value, reason):
    if not value:
        raise ValueError(reason)


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def read_file(path, inputs, expected=None):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), 'regular file required: ' + str(path))
    before = path.stat()
    raw = path.read_bytes()
    after = path.stat()
    fields = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns')
    require(all(getattr(before, k) == getattr(after, k) for k in fields), 'file changed while reading')
    digest = sha256(raw)
    if expected is not None:
        require(len(raw) == expected['bytes'] and digest == expected['sha256'], 'hash/size mismatch: ' + str(path))
    inputs[str(path.resolve())] = dict(bytes=len(raw), sha256=digest)
    return raw


def load_rois(folder, inputs):
    folder = Path(folder).resolve()
    manifest = json.loads(read_file(folder / 'manifest.json', inputs))
    require(manifest['status'] == 'closed_observation_bundle', 'ROI source is not sealed')
    summary = json.loads(read_file(folder / 'summary.json', inputs, manifest['files']['summary.json']))
    records = summary['rois_fixed_before_capture']
    require(all(k in records for k in ('table', 'object')), 'sealed table/object ROIs required')
    for name in ('table', 'object'):
        require(records[name].get('selected_before_next_capture') is True, 'ROI was not fixed before capture')
    return {name: records[name] for name in ('table', 'object')}


def load_roi_spec(path, inputs):
    """Current RGB-selected rectangles; no historical scene or invented timing."""
    path = Path(path).resolve()
    record = json.loads(read_file(path, inputs))
    source = record['source_rgb']
    raw = read_file(Path(source['path']), inputs)
    require(sha256(raw) == source['sha256'], 'ROI reference image hash differs')
    import cv2
    color = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    require(color is not None and color.ndim == 3 and color.shape[2] == 3, 'ROI reference RGB layout differs')
    camera = json.loads(read_file(path.parent/'camera_configuration.json', inputs))
    contract = camera['runtime_profile']['rectification']
    require(list(color.shape[:2]) == [contract['output_intrinsics']['height'],
                                     contract['output_intrinsics']['width']], 'ROI reference geometry differs')
    records = {}
    for name, key in (('table', 'table'), ('object', 'object_interior')):
        bounds = record['rois'][key]
        require(len(bounds) == 4 and all(type(v) is int for v in bounds), 'ROI integer bounds required')
        x0,y0,x1,y1 = bounds
        require(0 <= x0 < x1 <= color.shape[1] and 0 <= y0 < y1 <= color.shape[0], 'ROI outside reference image')
        records[name] = dict(bounds_xyxy=bounds, image_shape=list(color.shape[:2]),
            declaration_key=key, declaration_path=str(path), declaration_recorded_ns=record.get('recorded_ns'),
            selection_method=record.get('selection_basis'),
            meaning='RGB-selected interior rectangle, not a complete object or depth truth mask; timestamp retained without a before-capture claim')
    return records, color, dict(kind='current_scene_RGB_declaration', declaration_path=str(path),
        source_rgb=source, selection=record.get('selection_basis')), contract


def load_historical_color(folder, inputs, rectification):
    folder = Path(folder).resolve()
    manifest = json.loads(read_file(folder/'manifest.json',inputs))
    frames = manifest['frame_manifests_sha256']
    name = sorted(frames)[-1]
    relative = Path(name)
    require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe historical frame path')
    raw = read_file(folder/relative,inputs)
    require(sha256(raw) == frames[name], 'historical frame manifest hash differs')
    frame_manifest = json.loads(raw)
    parent = (folder/relative).parent
    meta = json.loads(read_file(parent/'metadata.json',inputs,frame_manifest['files']['metadata.json']))
    require(meta['geometry']['rectification'] == rectification, 'historical/current rectification differs')
    raw = read_file(parent/'rgb.npy',inputs,frame_manifest['files']['rgb.npy'])
    color = np.load(io.BytesIO(raw),allow_pickle=False)
    require(color.dtype == np.uint8 and color.ndim == 3 and color.shape[2] == 3, 'historical RGB layout differs')
    return color,dict(frame_manifest=str((folder/relative).resolve()),metadata=meta.get('rgb',{}),
                      selection='last frame of the explicitly supplied sealed burst')


def color_comparison(before, after, mask):
    require(before.shape == after.shape and before.shape[:2] == mask.shape and mask.any(), 'color comparison grid differs')
    a,b = before[mask].astype(float),after[mask].astype(float)
    delta = np.abs(b-a)
    peak = delta.max(axis=1)
    corr = float(np.corrcoef(a.ravel(),b.ravel())[0,1]) if a.std() > 0 and b.std() > 0 else None
    return dict(observed_pixels=int(mask.sum()),mean_absolute_channel_difference_0_255=float(delta.mean()),
        median_max_channel_difference_0_255=float(np.median(peak)),
        p95_max_channel_difference_0_255=float(np.percentile(peak,95)),
        fraction_with_any_channel_difference_at_least_20=float((peak >= 20).mean()),
        flattened_channel_correlation=corr)


def compare_scene(historical, rows, rectifier, rois):
    first=rows[0]['arrays']['after_rectification_color']
    last=rows[-1]['arrays']['after_rectification_color']
    scopes={}
    masks={'full_observed_canvas':rectifier.rgb_valid.copy()}
    for name,record in rois.items():
        x0,y0,x1,y1=record['bounds_xyxy']
        mask=np.zeros(rectifier.rgb_valid.shape,dtype=bool);mask[y0:y1,x0:x1]=True
        masks[name]=mask & rectifier.rgb_valid
    for name,mask in masks.items():
        scopes[name]=dict(historical_last_to_current_first=color_comparison(historical,first,mask),
            historical_last_to_current_last=color_comparison(historical,last,mask),
            current_first_to_current_last=color_comparison(first,last,mask))
    return dict(scopes=scopes,same_scene_proved=False,object_roi_current_occupancy_proved=False,
        meaning='Direct fixed-grid pixel differences include illumination, texture and motion; threshold 20 is descriptive only and does not authorize scene acceptance.')


def verify_capture(folder, inputs, expected_summary_sha=None):
    folder = Path(folder).resolve()
    raw = read_file(folder / 'summary.json', inputs)
    if expected_summary_sha:
        require(sha256(raw) == expected_summary_sha, 'summary differs from supplied receipt')
    summary = json.loads(raw)
    require(summary.get('closed') is True and summary.get('active_pair') is False, 'capture is not closed')
    require(summary.get('error') is None, 'capture writer reported an error')
    require(summary.get('diagnostic_only') is True, 'unexpected capture kind')
    entries = summary['files']
    require(len({r['name'] for r in entries}) == len(entries), 'duplicate capture files')
    payloads = {}
    for entry in entries:
        name = entry['name']
        require(re.fullmatch(r'pair_[0-9]{2}\.(?:json|npz)', name), 'unexpected capture file name')
        payloads[name] = read_file(folder / name, inputs, entry)
    stems = sorted({name.rsplit('.', 1)[0] for name in payloads})
    require(len(stems) == summary['counts']['written'] > 0, 'written pair count differs')
    require(sum(len(v) for v in payloads.values()) == summary['written_bytes'], 'capture byte count differs')
    rows = []
    for stem in stems:
        require(stem + '.json' in payloads and stem + '.npz' in payloads, 'incomplete stored pair')
        meta = json.loads(payloads[stem + '.json'])
        require(meta.get('complete') is True and meta['original_read']['result'] == 'complete', 'interrupted pair')
        require(meta['original_read']['pair_timing']['accepted'] is True, 'source pair was rejected')
        require(meta['source'] == meta['original_read']['raw_frames'], 'source identities differ within pair')
        with np.load(io.BytesIO(payloads[stem + '.npz']), allow_pickle=False) as saved:
            require(set(saved.files) == set(meta['arrays']) == set(meta['stage_order']), 'stage inventory differs')
            require(len(meta['stage_order']) == len(saved.files), 'duplicate stage order')
            arrays = {name: np.array(saved[name], copy=True) for name in saved.files}
        for name, value in arrays.items():
            expected = meta['arrays'][name]
            require(list(value.shape) == expected['shape'] and str(value.dtype) == expected['dtype'], 'array layout mismatch')
            require(sha256(value.tobytes(order='C')) == expected['sha256'], 'array pixel hash mismatch')
            color = name.endswith('_color')
            require(value.dtype == (np.uint8 if color else np.uint16) and value.ndim == (3 if color else 2), 'unsupported pixel layout')
            require(not color or value.shape[2] == 3, 'unsupported color layout')
        rows.append(dict(name=stem, metadata=meta, arrays=arrays))
    require(len(rows) <= 12, 'capture exceeds finite pair budget')
    for a, b in zip(rows, rows[1:]):
        for stream in ('color', 'depth'):
            require(b['metadata']['source'][stream]['frame_number'] > a['metadata']['source'][stream]['frame_number'], 'source frame order/reuse')
    return summary, rows


class SDKMathOnly:
    """Expose numerical SDK calls only; no context, device, pipeline or sensor."""
    def __init__(self, sdk):
        self.__version__ = sdk.__version__
        for name in ('intrinsics', 'distortion', 'rs2_deproject_pixel_to_point', 'rs2_project_point_to_pixel'):
            setattr(self, name, getattr(sdk, name))


def make_rectifier(rows, sdk_root, inputs):
    contract = rows[0]['metadata']['context']['runtime_profile']['rectification']
    require(contract.get('enabled') is True, 'this analyser requires the recorded injective rectification')
    for row in rows:
        meta = row['metadata']
        require(meta['context']['runtime_profile']['rectification'] == contract, 'geometry changed within capture')
        require(meta['context']['alignment_requested'] is True and meta['original_read']['sdk_color_alignment_applied'] is True, 'color alignment is not proved')
    source_bytes = read_file(RECTIFIER, inputs)
    expected_source = rows[0]['metadata']['owner']['sources'].get(str(RECTIFIER))
    require(expected_source == sha256(source_bytes), 'rectifier source differs from captured owner')
    spec = importlib.util.spec_from_file_location('verified_probe_rectification', str(RECTIFIER))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sdk_root = Path(sdk_root).resolve()
    sys.path.insert(0, str(sdk_root))
    sdk = importlib.import_module('pyrealsense2')
    require(Path(sdk.__file__).resolve().parent == sdk_root, 'unexpected SDK module source')
    require(sdk.__version__ == contract['sdk_version'] == '2.56.5', 'unverified SDK numerical version')
    for path, digest in rows[0]['metadata']['owner']['sources'].items():
        if 'realsense' in path and '.so' in Path(path).name:
            require(sha256(read_file(Path(path), inputs)) == digest, 'SDK binary differs from capture')
    rectifier = module.PairedPinholeRectification(contract['source_intrinsics'], sdk=SDKMathOnly(sdk))
    require(rectifier.contract == contract, 'reconstructed rectification contract/map hashes differ')
    return rectifier


def transition(before, after, scale):
    require(before.shape == after.shape and before.size > 0, 'transition grid differs/empty')
    positive_before, positive_after = before > 0, after > 0
    both = positive_before & positive_after
    delta = np.abs(after[both].astype(float) - before[both].astype(float)) * scale
    return dict(pixels=int(before.size), before_zero=int((~positive_before).sum()),
        after_zero=int((~positive_after).sum()), positive_to_zero=int((positive_before & ~positive_after).sum()),
        zero_to_positive=int((~positive_before & positive_after).sum()),
        changed_positive_values=int((both & (before != after)).sum()),
        unchanged_values=int((before == after).sum()), positive_in_both=int(both.sum()),
        median_abs_change_m=float(np.median(delta)) if delta.size else None,
        p95_abs_change_m=float(np.percentile(delta, 95)) if delta.size else None,
        maximum_abs_change_m=float(delta.max()) if delta.size else None)


def sequence(values, scale, names):
    require(values.ndim == 2 and values.shape[0] == len(names) and values.shape[1] > 0, 'sequence layout')
    valid = values > 0
    count = valid.sum(axis=0)
    return dict(pixels=int(values.shape[1]), zero_counts=(~valid).sum(axis=1).tolist(),
        zero_fractions=(~valid).mean(axis=1).tolist(),
        always_valid_pixels=int((count == len(values)).sum()), always_zero_pixels=int((count == 0).sum()),
        intermittent_pixels=int(((count > 0) & (count < len(values))).sum()),
        adjacent_pairs=[dict(previous=names[i-1], current=names[i], **transition(values[i-1], values[i], scale))
                        for i in range(1, len(values))])


def source_roi(rectifier, bounds):
    x0, y0, x1, y1 = bounds
    require(all(type(v) is int for v in bounds), 'ROI integer coordinates required')
    require(0 <= x0 < x1 <= rectifier.width and 0 <= y0 < y1 <= rectifier.height, 'ROI outside rectified canvas')
    target = rectifier.target_indices
    tx, ty = target % rectifier.width, target // rectifier.width
    selected = (tx >= x0) & (tx < x1) & (ty >= y0) & (ty < y1)
    require(selected.any(), 'ROI has no mapped source pixels')
    return rectifier.source_indices[selected], target[selected]


def zero_origins(stages, arrays):
    labels = np.zeros(arrays[0].shape, dtype=np.int16)
    for index in range(1, len(arrays)):
        labels[(arrays[index-1] > 0) & (arrays[index] == 0)] = index
    final_zero = arrays[-1] == 0
    return {stage: int((final_zero & (labels == i)).sum()) for i, stage in enumerate(stages)}


def analyse_rows(rows, rectifier, rois):
    names = [r['name'] for r in rows]
    first = rows[0]['metadata']
    scale = first['context']['depth_scale_m']
    require(math.isfinite(scale) and scale > 0, 'invalid measured depth scale')
    stages = [s for s in first['stage_order'] if s.startswith('after_') and not s.endswith('_color')]
    require(stages[0] == 'after_align_depth' and stages[-2:] == ['after_clip_depth', 'after_rectification_depth'], 'unsupported stage order')
    require('after_stationary_depth' in stages, 'stationary tap missing')
    source_shape = (rectifier.source_height, rectifier.source_width)
    target_shape = (rectifier.height, rectifier.width)
    traces, identities, native = [], [], []
    for row in rows:
        meta, arrays = row['metadata'], row['arrays']
        require(meta['stage_order'] == first['stage_order'], 'stage order changed')
        require(meta['context']['depth_scale_m'] == scale, 'scale changed within capture')
        for s in stages[:-1]:
            require(arrays[s].shape == source_shape, 'aligned stage shape differs: ' + s)
        require(arrays[stages[-1]].shape == target_shape, 'rectified shape differs')
        require(np.array_equal(arrays['after_clip_depth'].ravel()[rectifier.source_indices],
                               arrays['after_rectification_depth'].ravel()[rectifier.target_indices]), 'rectification changed source depth values')
        padding = np.ones(rectifier.width * rectifier.height, dtype=bool)
        padding[rectifier.target_indices] = False
        require(np.all(arrays['after_rectification_depth'].ravel()[padding] == 0), 'unmapped canvas has nonzero depth')
        native.append(arrays['native_depth'].ravel())
        src = meta['source']
        identities.append(dict(pair=row['name'], raw_frames={stream: {k:src[stream].get(k) for k in
            ('frame_number','device_timestamp_ms','timestamp_domain','stream_fps','profile_signature')} for stream in ('color','depth')},
            selected_monotonic=meta['selected_monotonic'], completed_monotonic=meta['completed_monotonic'],
            copy_wall_sec=meta['copy_wall_sec'], copy_thread_cpu_sec=meta['copy_thread_cpu_sec'],
            original_read_total_sec=meta['original_read']['total_sec'],
            regional_stationary_evidence=meta['original_read'].get('regional_stationary_evidence'),
            pair_timing=meta['original_read']['pair_timing']))
    for stream in ('color','depth'):
        require(all(r['raw_frames'][stream]['profile_signature'] == identities[0]['raw_frames'][stream]['profile_signature'] for r in identities), 'native/color stream geometry changed')
    scopes = {}
    roi_bounds = {'full_aligned_grid': [0, 0, rectifier.width, rectifier.height]}
    for name, record in rois.items():
        require(record['image_shape'] == list(target_shape), 'historical ROI canvas differs')
        roi_bounds[name] = record['bounds_xyxy']
    for name, bounds in roi_bounds.items():
        source, target = source_roi(rectifier, bounds)
        all_values = {}
        for stage in stages:
            indices = target if stage == 'after_rectification_depth' else source
            all_values[stage] = np.stack([r['arrays'][stage].ravel()[indices] for r in rows])
        x0,y0,x1,y1 = bounds
        canvas = np.stack([r['arrays']['after_rectification_depth'][y0:y1,x0:x1].ravel() for r in rows])
        final_origins = []
        adjacency_origins = []
        for i, row in enumerate(rows):
            vals = [all_values[s][i] for s in stages]
            final_origins.append(dict(pair=row['name'], counts=zero_origins(stages, vals)))
            if i:
                selection = (all_values[stages[-1]][i-1] > 0) & (vals[-1] == 0)
                adjacency_origins.append(dict(previous=names[i-1], current=names[i],
                    final_positive_to_zero_pixels=int(selection.sum()),
                    current_frame_last_zero_origin_counts=zero_origins(stages, [v[selection] for v in vals])))
        scopes[name] = dict(rectified_bounds_xyxy=bounds, mapped_source_pixels=int(len(source)),
            canvas_pixels=int(canvas.shape[1]), fixed_unmapped_canvas_pixels=int(canvas.shape[1]-len(source)),
            aligned_source_indices_sha256=sha256(source.tobytes()), rectified_target_indices_sha256=sha256(target.tobytes()),
            stage_sequences={s:sequence(v,scale,names) for s,v in all_values.items()},
            same_frame_transitions=[dict(before=stages[j-1],after=stages[j],pairs=[
                dict(pair=names[i],**transition(all_values[stages[j-1]][i],all_values[stages[j]][i],scale))
                for i in range(len(rows))]) for j in range(1,len(stages))],
            rectified_canvas_sequence=sequence(canvas,scale,names), final_zero_origins=final_origins,
            adjacent_final_drop_origins=adjacency_origins)
    return dict(frame_count=len(rows),depth_scale_m_per_unit=scale,stage_order=stages,identities=identities,
        source_frame_gaps=[dict(previous=a['pair'],current=b['pair'],
            delta_frames={s:b['raw_frames'][s]['frame_number']-a['raw_frames'][s]['frame_number'] for s in ('color','depth')})
            for a,b in zip(identities,identities[1:])],
        native_depth_independent_grid=sequence(np.stack(native),scale,names),
        native_to_aligned_pixelwise_comparison_performed=False,rectification_source_values_preserved=True,
        zero_origin_meaning='Last positive-to-zero transition among recorded aligned stages; an already-zero after_align pixel cannot be attributed to native sensing versus alignment.',
        historical_roi_meaning='Explicit RGB-selected interior ROIs reused without selection by depth residual; current object occupancy requires scene review.',
        scopes=scopes)


def report_text(result):
    analysis = result['analysis']
    lines = ['# 同帧深度处理阶段核验', '', '逐文件大小与 SHA256、逐数组像素 SHA256、同源帧身份及校正映射均已核验。',
        '原生深度按自身网格独立统计；没有将 native 与 color 对齐网格直接按像素比较。',
        'ROI 来自明确提供的 RGB 参考图；内部矩形不代表完整物体真值掩膜，当前场景仍需核对。', '',
        '帧数：%d；深度单位：%.12g m/原单位。' % (analysis['frame_count'], analysis['depth_scale_m_per_unit']), '',
        '|范围|阶段|零值比例最小–最大|相邻帧正值转零总数|', '|---|---|---:|---:|']
    native = analysis['native_depth_independent_grid']
    lines.append('|原生深度全网格|native|%.3f%%–%.3f%%|%d|' %
        (100*min(native['zero_fractions']),100*max(native['zero_fractions']),sum(p['positive_to_zero'] for p in native['adjacent_pairs'])))
    for name,scope in analysis['scopes'].items():
        for stage,seq in scope['stage_sequences'].items():
            lines.append('|%s（映射点）|%s|%.3f%%–%.3f%%|%d|' % (name,stage,100*min(seq['zero_fractions']),100*max(seq['zero_fractions']),sum(p['positive_to_zero'] for p in seq['adjacent_pairs'])))
    lines += ['', '|范围|阶段变化|同帧新增零值总数|同帧零值恢复总数|', '|---|---|---:|---:|']
    for name,scope in analysis['scopes'].items():
        for pair in scope['same_frame_transitions']:
            lines.append('|%s|%s → %s|%d|%d|' % (name,pair['before'],pair['after'],
                sum(p['positive_to_zero'] for p in pair['pairs']),sum(p['zero_to_positive'] for p in pair['pairs'])))
    lines += ['', '同帧新增/恢复统计以映射后的同一源像素为单位；以上总数可在多个帧重复计数，不是唯一像素数。',
        'after_align 已为零，只能确认零值在空间/区域时域/范围裁剪之前已存在；本分析不能单独区分原生测深与对齐造成的零值。',
        '校正画布的固定空格单列在 JSON 中，不能当成原始深度丢失。此次核验不构成标定、场景清空或抓取验收。', '']
    lines += ['|范围|ROI参考图→当前首帧 RGB平均绝对差（0–255）|任一通道差≥20的像素比例|当前首帧→末帧平均绝对差|',
              '|---|---:|---:|---:|']
    for name,row in result['scene_comparison']['scopes'].items():
        previous=row['historical_last_to_current_first'];current=row['current_first_to_current_last']
        lines.append('|%s|%.3f|%.3f%%|%.3f|' % (name,previous['mean_absolute_channel_difference_0_255'],
            100*previous['fraction_with_any_channel_difference_at_least_20'],current['mean_absolute_channel_difference_0_255']))
    lines += ['', 'RGB差值同时受曝光、照明、纹理和场景移动影响；不能单凭该数值证明场景未改变。', '']
    return '\n'.join(lines)


def diagnostic_figure(historical, rows, rectifier, rois, output_parent):
    # Keep optional matplotlib font/cache writes on the same external storage.
    parent=Path(output_parent).resolve(strict=True)
    require(MOUNT.is_mount() and STORE.resolve() in parent.parents, 'external plotting cache required')
    cache=parent/'.depth_stage_matplotlib_cache'
    cache.mkdir(exist_ok=True)
    os.environ['MPLCONFIGDIR']=str(cache)
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.colors import ListedColormap,BoundaryNorm
    from matplotlib.patches import Rectangle,Patch
    stages=['native_depth']+[s for s in rows[0]['metadata']['stage_order']
        if s.startswith('after_') and not s.endswith('_color')]
    columns=len(stages)
    fig=plt.figure(figsize=(4*columns,11.7))
    grid=fig.add_gridspec(3,columns,height_ratios=[1.12,1,1])
    divisions=np.linspace(0,columns,4).astype(int)
    colors={'table':'#f4b400','object':'#00c8ff'}
    def outline(ax):
        for name,record in rois.items():
            x0,y0,x1,y1=record['bounds_xyxy']
            ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor=colors[name],linewidth=1.4))
    for i,(title,color) in enumerate((('ROI reference RGB',historical),
        ('Current first RGB: '+rows[0]['name'],rows[0]['arrays']['after_rectification_color']),
        ('Current last RGB: '+rows[-1]['name'],rows[-1]['arrays']['after_rectification_color']))):
        ax=fig.add_subplot(grid[0,divisions[i]:divisions[i+1]])
        ax.imshow(color[:,:,::-1]);outline(ax);ax.set_title(title);ax.set_axis_off()
    cmap=ListedColormap(['#f7f7f7','#c63838','#8c98aa']);norm=BoundaryNorm([-.5,.5,1.5,2.5],3)
    for line,row in enumerate((rows[0],rows[-1]),start=1):
        for j,stage in enumerate(stages):
            ax=fig.add_subplot(grid[line,j]);depth=row['arrays'][stage]
            if stage=='native_depth':
                mask=(depth==0).astype(np.uint8);subtitle='native grid; no ROI correspondence'
            elif stage=='after_rectification_depth':
                mask=(depth==0).astype(np.uint8)
                mapped=np.zeros(mask.size,dtype=bool);mapped[rectifier.target_indices]=True
                mask.ravel()[~mapped]=2;subtitle='recorded rectified grid';outline(ax)
            else:
                mask=np.full((rectifier.height,rectifier.width),2,dtype=np.uint8)
                mask.ravel()[rectifier.target_indices]=(depth.ravel()[rectifier.source_indices]==0).astype(np.uint8)
                subtitle='source pixels on verified rectified map';outline(ax)
            ax.imshow(mask,cmap=cmap,norm=norm,interpolation='nearest')
            ax.set_title('%s / %s\n%s\nsource zero: %.2f%%' %
                (row['name'],stage.replace('after_',''),subtitle,100*float((depth==0).mean())),fontsize=8)
            ax.set_axis_off()
    fig.legend(handles=[Patch(color='#f7f7f7',label='Measured positive depth'),Patch(color='#c63838',label='Zero depth'),
        Patch(color='#8c98aa',label='Fixed unmapped canvas'),Patch(fill=False,edgecolor=colors['table'],label='Declared table ROI'),
        Patch(fill=False,edgecolor=colors['object'],label='Declared object interior ROI')],loc='lower center',ncol=5)
    fig.suptitle('Finite same-read depth diagnostics | Native pixels are not compared directly with aligned pixels',fontsize=15)
    fig.subplots_adjust(top=.91,bottom=.06,left=.01,right=.99,hspace=.20,wspace=.06)
    buffer=io.BytesIO();fig.savefig(buffer,format='png',dpi=130);plt.close(fig)
    return buffer.getvalue()


def write_results(output, result, report=None, extra_files=None):
    output = Path(output).absolute()
    parent = output.parent.resolve(strict=True)
    require(MOUNT.is_mount() and (STORE.resolve() == parent or STORE.resolve() in parent.parents), 'mounted SanStand output required')
    device = MOUNT.stat().st_dev
    require(parent.stat().st_dev == device and device != Path('/').stat().st_dev, 'external output device required')
    output.mkdir(mode=0o700)
    fd = os.open(str(output), os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
    try:
        require(os.fstat(fd).st_dev == device, 'output device changed')
        files = {'result.json':(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()}
        if report is not None:
            files['report.md'] = report.encode()
        files.update(extra_files or {})
        for name,raw in files.items():
            require(Path(name).name == name, 'output basename required')
            require(MOUNT.is_mount() and MOUNT.stat().st_dev == device, 'SanStand became unavailable')
            file_fd = os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600,dir_fd=fd)
            with os.fdopen(file_fd,'wb') as stream:
                stream.write(raw);stream.flush();os.fsync(stream.fileno())
            require(read_file(output/name,{}) == raw, 'output readback differs')
        os.fsync(fd)
    finally:
        os.close(fd)
    return {name:sha256(raw) for name,raw in files.items()}


def self_test():
    checks = []
    a = np.array([0,1,2,3,4],dtype=np.uint16)
    b = np.array([5,0,2,4,0],dtype=np.uint16)
    v = transition(a,b,.001)
    require((v['positive_to_zero'],v['zero_to_positive'],v['changed_positive_values']) == (2,1,1), 'transition counting')
    checks.append('positive/zero transitions and changed valid values')
    v = sequence(np.stack([a,b]),.001,['a','b'])
    require(v['intermittent_pixels'] == 3 and v['always_valid_pixels'] == 2, 'sequence counting')
    checks.append('intermittent/always valid sequence counts')
    class Rect:
        width,height=4,3
        source_indices=np.arange(6,dtype=np.int64)
        target_indices=np.array([0,2,3,5,9,11],dtype=np.int64)
    src,dst = source_roi(Rect(),[1,0,4,2])
    require(src.tolist() == [1,2,3] and dst.tolist() == [2,3,5], 'ROI inverse map/padding')
    checks.append('rectified ROI exact source mapping and padding exclusion')
    stages=['aligned','spatial','temporal','clip']
    arrays=[np.array(v,dtype=np.uint16) for v in ([0,1,1,1],[0,0,2,1],[0,4,2,1],[0,0,2,0])]
    require(zero_origins(stages,arrays) == dict(aligned=1,spatial=0,temporal=0,clip=2), 'last zero origin')
    checks.append('drop/restore/drop attribution uses latest zero transition')
    try:
        source_roi(Rect(),[0,0,5,3])
    except ValueError:
        checks.append('invalid ROI rejected')
    else:
        raise ValueError('invalid ROI accepted')
    return dict(kind='depth_stage_analyzer_self_test',passed=True,checks=checks,
                SDK_imported=False,ROS_contacted=False,device_access=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture_positional',type=Path,nargs='?')
    parser.add_argument('--capture',type=Path)
    roi_args=parser.add_mutually_exclusive_group()
    roi_args.add_argument('--roi-burst',type=Path)
    roi_args.add_argument('--roi-spec',type=Path)
    parser.add_argument('--sdk-root',type=Path,default=WORKTREE/'.runtime/realsense-2.56.5')
    parser.add_argument('--summary-sha256')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args();started=time.monotonic();cpu=time.process_time()
    extra_files={}
    if args.self_test:
        result=self_test();report=None
    else:
        require(not (args.capture and args.capture_positional),'supply capture once')
        capture=args.capture or args.capture_positional
        require(capture is not None and (args.roi_burst or args.roi_spec),'capture and explicit ROI source required')
        inputs={}
        if args.roi_spec:
            rois,historical,historical_source,reference_contract=load_roi_spec(args.roi_spec,inputs)
        else:
            rois=load_rois(args.roi_burst,inputs)
        summary,rows=verify_capture(capture,inputs,args.summary_sha256)
        rectifier=make_rectifier(rows,args.sdk_root,inputs)
        if args.roi_spec:
            require(reference_contract == rectifier.contract, 'current ROI reference/capture rectification differs')
        else:
            historical,historical_source=load_historical_color(args.roi_burst,inputs,rectifier.contract)
        result=dict(schema=1,kind='same_read_depth_stage_analysis',passed=True,diagnostic_only=True,
            input_files=inputs,capture_summary=summary,rois=rois,rectification=rectifier.contract,
            analysis=analyse_rows(rows,rectifier,rois),
            historical_rgb_source=historical_source,scene_comparison=compare_scene(historical,rows,rectifier,rois),
            ROS_contacted=False,SDK_device_access=False,motion_commands_sent=False,
            parameters_changed=False,grasp_or_calibration_verified=False)
        report=report_text(result)
        extra_files['diagnostic.png']=diagnostic_figure(historical,rows,rectifier,rois,args.output.parent)
    result.update(source_sha256=sha256(Path(__file__).read_bytes()),elapsed_sec=time.monotonic()-started,
                  process_cpu_sec=time.process_time()-cpu,peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    hashes=write_results(args.output,result,report,extra_files)
    print(json.dumps(dict(output=str(args.output),passed=True,files=hashes,elapsed_sec=result['elapsed_sec']),ensure_ascii=False))


if __name__ == '__main__':
    main()
