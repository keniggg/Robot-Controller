#!/usr/bin/env python3
"""Offline examination of closed RGB-D bursts; no ROS or camera access."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import struct
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
import numpy as np


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def image_message(raw):
    """Decode the serialized sensor_msgs/Image directly, without ROS imports."""
    pos = 0
    def number(fmt):
        nonlocal pos
        result = struct.unpack_from(fmt, raw, pos)
        pos += struct.calcsize(fmt)
        return result
    def string():
        nonlocal pos
        size, = number('<I')
        value = raw[pos:pos+size].decode('utf8'); pos += size
        return value
    seq, secs, nsecs = number('<III')
    frame = string()
    height, width = number('<II')
    encoding = string()
    endian, step, size = number('<BII')
    payload = raw[pos:pos+size]; pos += size
    assert pos == len(raw) and len(payload) == height*step == size
    assert encoding == '16UC1' and step >= width*2
    array = np.ndarray((height, width), dtype=('>' if endian else '<')+'u2',
                       buffer=payload, strides=(step, 2)).copy()
    return array, dict(seq=seq, stamp_ns=secs*10**9+nsecs, frame_id=frame,
        encoding=encoding, width=width, height=height, step=step,
        is_bigendian=endian, data_sha256=digest(payload))


def verified(folder, manifest, name, inputs):
    path = folder/name
    assert not path.is_symlink() and path.resolve().parent == folder.resolve()
    raw = path.read_bytes(); expected = manifest['files'][name]
    assert len(raw) == expected['bytes'] and digest(raw) == expected['sha256'], str(path)
    inputs[str(path.resolve())] = digest(raw)
    return raw


def stats(arrays, bounds, scale):
    x0, y0, x1, y1 = bounds
    assert 0 <= x0 < x1 <= arrays.shape[2] and 0 <= y0 < y1 <= arrays.shape[1]
    data = arrays[:, y0:y1, x0:x1]
    valid = data > 0; n = valid.shape[1]*valid.shape[2]
    count = valid.sum(axis=0)
    pairs = []
    for i in range(1, len(data)):
        both = valid[i-1] & valid[i]
        delta = np.abs(data[i].astype(float)-data[i-1].astype(float))[both]*scale
        pairs.append(dict(previous=i-1, current=i,
            positive_to_zero=int((valid[i-1] & ~valid[i]).sum()),
            zero_to_positive=int((~valid[i-1] & valid[i]).sum()),
            positive_in_both=int(both.sum()),
            median_abs_change_m=float(np.median(delta)) if delta.size else None,
            p95_abs_change_m=float(np.percentile(delta, 95)) if delta.size else None))
    # Deterministic row-major examples; these do not select which frames count.
    ys, xs = np.nonzero((count > 0) & (count < len(data)))
    samples = [dict(x=int(x+x0), y=int(y+y0), raw_depth_units=data[:,y,x].tolist(),
                    depth_m=(data[:,y,x].astype(float)*scale).tolist())
               for y,x in list(zip(ys,xs))[::max(1,len(xs)//8)][:8]]
    return dict(bounds_xyxy=bounds, pixels=n,
        zero_counts=(~valid).sum(axis=(1,2)).tolist(),
        zero_fractions=((~valid).sum(axis=(1,2))/float(n)).tolist(),
        always_valid_pixels=int((count==len(data)).sum()),
        always_zero_pixels=int((count==0).sum()),
        intermittent_pixels=int(((count>0)&(count<len(data))).sum()),
        intermittent_fraction=float(((count>0)&(count<len(data))).sum()/float(n)),
        adjacent_pairs=pairs, example_pixel_sequences=samples)


def examine(folder):
    started=time.monotonic(); inputs={}
    raw=(folder/'manifest.json').read_bytes(); top=json.loads(raw)
    inputs[str((folder/'manifest.json').resolve())]=digest(raw)
    assert top['status']=='closed_observation_bundle'
    summary=json.loads(verified(folder,top,'summary.json',inputs))
    report=json.loads(verified(folder,top,'same_pose_report.json',inputs))
    parameters=json.loads(verified(folder,top,'parameters_before.json',inputs))
    after=json.loads(verified(folder,top,'parameters_after.json',inputs))
    arrays=[]; metadata=[]; rows=[]
    expected=summary['frame_manifests_sha256']
    assert expected==top['frame_manifests_sha256']
    for name,h in sorted(expected.items()):
        path=folder/name; raw=path.read_bytes(); assert digest(raw)==h
        inputs[str(path.resolve())]=h; manifest=json.loads(raw)
        meta=json.loads(verified(path.parent,manifest,'metadata.json',inputs))
        saved=np.load(io.BytesIO(verified(path.parent,manifest,'depth_processed.npy',inputs)),allow_pickle=False)
        original,header=image_message(verified(path.parent,manifest,'depth_processed.rosmsg',inputs))
        mask=np.load(io.BytesIO(verified(path.parent,manifest,'valid_depth.npy',inputs)),allow_pickle=False)
        assert saved.dtype==original.dtype and saved.shape==original.shape
        assert np.array_equal(original,saved) and np.array_equal(mask,original>0)
        assert all(meta['depth_processed'][k]==v for k,v in header.items())
        assert not meta['acquisition_issues']
        assert meta['depth_processed']['publisher_callerid']=='/supervisor_camera'
        exact=meta['synchronization'].get('exact_stamp_matched_device_diagnostic')
        row=dict(index=len(arrays), source_stamp_ns=header['stamp_ns'],
            received_data_equal_saved_pixels=True, publisher=meta['depth_processed']['publisher_callerid'],
            source_skew_ns=meta['synchronization']['source_skew_ns'],
            rectification_digest=meta['geometry']['rectification']['digest'],
            exact_device_diagnostic_available=exact is not None)
        if exact:
            assert int(exact['source_stamp_ns'])==header['stamp_ns']
            ev=exact['evidence']; reg=ev.get('regional_stationary_evidence',{})
            row.update(raw_frame_numbers={k:v['frame_number'] for k,v in ev['raw_frames'].items()},
                device_skew_ms=ev['pair_timing']['skew_ms'],
                regional_temporal={k:reg.get(k) for k in ('active','current_zero_pixels_preserved',
                    'persistence','filtered_fraction','global_history_reset_reason')},
                regions=[{k:r.get(k) for k in ('name','filter_applied','history_reset','bounds_xyxy')}
                         for r in reg.get('regions',[])])
        rows.append(row); arrays.append(original); metadata.append(meta)
    assert len(arrays)==summary['planned_frame_count']==summary['analysable_frame_count']
    assert all(b['source_stamp_ns']>a['source_stamp_ns'] for a,b in zip(rows,rows[1:]))
    assert len(set(r['rectification_digest'] for r in rows))==1
    scales={m['geometry']['depth_scale_m_per_unit'] for m in metadata}; assert len(scales)==1
    scale=scales.pop(); arrays=np.stack(arrays)
    scopes={name:stats(arrays,desc['bounds_xyxy'],scale)
            for name,desc in summary['rois_fixed_before_capture'].items()}
    scopes['full_image']=stats(arrays,[0,0,arrays.shape[2],arrays.shape[1]],scale)
    rect=metadata[0]['geometry']['rectification']
    joint_max=max(abs(v['absolute_change']) for r in report['actual_joint_changes_from_first']
                  for v in r['joints'].values())
    result=dict(folder=str(folder.resolve()),input_sha256=inputs,frame_count=len(arrays),
        all_serialized_pixels_match_saved_arrays=True,all_validity_masks_match_received_zeros=True,
        source_kind='processed_ROS_Image; not native/pre-alignment/pre-filter depth',
        depth_scale_m_per_unit=scale,original_diagnostic_status=summary['status'],
        original_diagnostic_alerts=report['alerts'],joint_max_change=joint_max,
        fixed_canvas_unmapped_pixels=int(arrays.shape[1]*arrays.shape[2]-rect['mapped_source_pixel_count']),
        fixed_canvas_total_pixels=int(arrays.shape[1]*arrays.shape[2]),
        camera_geometry_before_after_same=parameters['camera']['runtime_profile']['rectification']==after['camera']['runtime_profile']['rectification'],
        rows=rows,scopes=scopes,elapsed_sec=time.monotonic()-started,
        raw_depth_unavailable=True,root_cause_not_yet_proven=True)
    return result,arrays


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bursts',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args(); mount=Path('/media/zhuyupei/SanStand')
    output=args.output.resolve();assert mount.is_mount() and mount in output.parents
    output.mkdir(exist_ok=False); results=[];started=time.monotonic()
    for folder in args.bursts:
        result,arrays=examine(folder.resolve());results.append(result)
    final=dict(schema=1,kind='offline_depth_validity_investigation',passed=True,
        meaning='data integrity and phenomenon confirmed; not camera/grasp readiness',
        motion_commands_sent=False,camera_parameters_changed=False,bursts=results,
        source_sha256={str(Path(__file__).resolve()):digest(Path(__file__).read_bytes())},
        elapsed_sec=time.monotonic()-started)
    path=output/'result.json';raw=(json.dumps(final,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with path.open('xb') as out:out.write(raw);out.flush();os.fsync(out.fileno())
    assert path.read_bytes()==raw
    print(json.dumps(dict(result=str(path),sha256=digest(raw),elapsed_sec=final['elapsed_sec'],
        bursts=[dict(name=Path(r['folder']).name,scopes={k:dict(zero_fraction_range=[min(v['zero_fractions']),max(v['zero_fractions'])],
            intermittent_fraction=v['intermittent_fraction'],positive_to_zero=[p['positive_to_zero'] for p in v['adjacent_pairs']])
            for k,v in r['scopes'].items() if k!='full_image'}) for r in results]),ensure_ascii=False))


if __name__=='__main__':
    main()
