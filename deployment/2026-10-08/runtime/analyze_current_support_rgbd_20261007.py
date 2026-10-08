#!/usr/bin/env python3
"""Offline current-scene support quality; no target identity or ROS runtime."""
import argparse
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path
import struct
import sys
import zipfile
import zlib

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/alicia_grasp_modes/src'))
sys.path.insert(0, str(ROOT/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_grasp_modes import tabletop as table

EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
_RECTIFIERS = {}


def sha(raw): return hashlib.sha256(raw).hexdigest()
def encoded(value): return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()


def decode_image(raw):
    """Decode ROS 1 sensor_msgs/Image bytes without importing or starting ROS."""
    offset = 0
    def integer():
        nonlocal offset
        value = struct.unpack_from('<I',raw,offset)[0]; offset += 4
        return value
    def string():
        nonlocal offset
        length=integer(); value=raw[offset:offset+length].decode('utf8'); offset += length
        return value
    seq, secs, nsecs = integer(), integer(), integer()
    frame=string(); height,width=integer(),integer(); encoding=string()
    endian=raw[offset]; offset+=1
    step,length=integer(),integer()
    if (encoding not in ('bgr8','16UC1') or endian not in (0,1) or not 0 < height <= 1080
            or not 0 < width <= 1920 or length != step*height or offset+length != len(raw)):
        raise ValueError('original_image_serialization_invalid')
    channels = 3 if encoding == 'bgr8' else 2
    if step < channels*width:
        raise ValueError('original_image_step_invalid')
    packed=np.frombuffer(raw, dtype=np.uint8, count=length, offset=offset).reshape(height,step)
    pixels=packed[:,:channels*width].copy()
    if encoding == 'bgr8': pixels=pixels.reshape(height,width,3)
    else: pixels=pixels.view('>u2' if endian else '<u2').reshape(height,width)
    return dict(seq=seq,source_stamp_ns=secs*10**9+nsecs,frame_id=frame,encoding=encoding,
                width=width,height=height,step=step),pixels


def original_fit(points, cfg):
    """Observe rejected original fit locals without altering any threshold/model."""
    captured={}; previous=sys.gettrace(); target=table._fit_plane.__code__
    def trace(frame,event,arg):
        if frame.f_code is not target: return None
        if event in ('return','exception'):
            for name in ('best_plane','best_count','best_residual','ratio'):
                value=frame.f_locals.get(name)
                if value is not None: captured[name]=value.copy() if isinstance(value,np.ndarray) else value
        return trace
    plane=None
    try:
        sys.settrace(trace)
        plane,ratio,residual,count=table._fit_plane(points,cfg)
        result=dict(original_fit_qualified=True,ratio=ratio,residual_m=residual,sample_count=count)
    except ValueError as exc:
        result=dict(original_fit_qualified=False,error=str(exc))
    finally: sys.settrace(previous)
    result['rejected_model_is_not_accepted']=not result['original_fit_qualified']
    result['original_locals']={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in captured.items()}
    return result,plane


def diagnose_depth(depth, intrinsics, transform, cfg, owner=None, regions=None):
    height,width=depth.shape; v,u=np.indices(depth.shape); center=np.array([width/2.,height/2.])
    radius=np.hypot(u-center[0],v-center[1]); scale=min(width/640.,height/480.)
    valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
    result=dict(center_uv=center.tolist(),rings=[],support_available=False,
        current_frame_segment=None,target_identity=None,motion_authorized=False)
    sequence_active=True
    for outer in (170,140,110):
        spatial=(radius>=70*scale)&(radius<=outer*scale); ring=spatial&valid
        points=table._project_pixels(depth[ring],u[ring],v[ring],intrinsics)
        fit,plane=original_fit(points,cfg)
        row=dict(outer_base_px=outer,ring_pixels=int(spatial.sum()),valid_points=int(ring.sum()),
            valid_fraction=float(ring.sum()/max(1,spatial.sum())),fit=fit,
            consulted_by_original_sequence=sequence_active)
        if owner is not None:
            row['actual_region_ownership_valid_points']={region['name']:int(np.count_nonzero(ring&(owner==i)))
                for i,region in enumerate(regions)}
            row['region_filter_applied_valid_fraction']=float(sum(np.count_nonzero(ring&(owner==i))
                for i,region in enumerate(regions) if region.get('filter_applied') is True)/max(1,ring.sum()))
            row['unmapped_valid_points']=int(np.count_nonzero(ring&(owner<0)))
        if plane is not None:
            residual=np.einsum('ij,j->i',points,plane[:3])+plane[3]
            row['quadrants']=table._support_quadrants(abs(residual)<=cfg.support_threshold_m,u[ring],v[ring],center)
            row['all_valid_points_abs_residual_mm_p50_p90_p99']=np.percentile(abs(residual)*1000,[50,90,99]).tolist()
        # The rejected original model is diagnostic only; it never changes the
        # acceptance sequence or substitutes a new fit on a different point set.
        model=plane if plane is not None else fit['original_locals'].get('best_plane')
        if model is not None and len(points):
            model=np.asarray(model,dtype=float)
            if model.shape==(4,) and np.all(np.isfinite(model)):
                absolute=np.abs(np.einsum('ij,j->i',points,model[:3])+model[3])
                row['original_model_full_ring_diagnostic']=dict(
                    inlier_ratio=float(np.mean(absolute<=cfg.support_threshold_m)),
                    abs_residual_mm_p50_p90_p99=np.percentile(absolute*1000,[50,90,99]).tolist(),
                    within_original_threshold_quadrants=table._support_quadrants(
                        absolute<=cfg.support_threshold_m,u[ring],v[ring],center),
                    metric_geometry_authority=False,no_new_fit_or_changed_model=True)
        if sequence_active:
            if fit['original_fit_qualified']:
                sequence_active=False
                if fit['residual_m']>cfg.support_threshold_m: result['support_reason']='support_plane_residual_too_large'
                elif row['quadrants']<3: result['support_reason']='insufficient_support_spatial_coverage'
                else:
                    result.update(support_available=True,support_reason='original_current_ring_quality_pass',
                        plane_camera=plane.tolist(),selected_outer_base_px=outer)
                    if transform is not None:
                        result['plane_base']=table._normalized_plane(np.linalg.solve(transform.T,plane)).tolist()
            elif fit['error'] != 'support_plane_inlier_ratio_low' or outer == 110:
                sequence_active=False; result['support_reason']=fit['error']
        result['rings'].append(row)
    if transform is not None:
        try:
            segmentation=table.segment(depth,intrinsics,transform,cfg)
            result['current_frame_segment']=dict(passed=True,metrics=segmentation.metrics,
                candidates_count=len(segmentation.candidates),
                candidate_metrics=[dict(bbox=list(c.bbox),center_uv=c.center_uv.tolist(),metrics=c.metrics)
                    for c in segmentation.candidates],selection_performed=False)
        except ValueError as exc:
            result['current_frame_segment']=dict(passed=False,error=str(exc),selection_performed=False)
    else: result['current_frame_segment']=dict(unavailable='exact same-stamp TF unavailable')
    return result


def diagnostic_owner(camera, evidence, shape):
    """Reproduce injective source-to-published mapping, never assume matching ROIs."""
    report=evidence.get('regional_stationary_evidence')
    if not isinstance(report,dict) or not report.get('regions'):
        return None,None,dict(available=False,reason='regional_read_evidence_missing')
    regions=report['regions']
    contract=camera.get('runtime_profile',{}).get('rectification')
    if not isinstance(contract,dict):
        return None,None,dict(available=False,reason='actual_rectification_contract_missing')
    sys.path.insert(0,str(ROOT/'.runtime/realsense-2.56.5'))
    import pyrealsense2 as sdk
    from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
    key=sha(encoded(contract))
    if key not in _RECTIFIERS:
        _RECTIFIERS[key]=PairedPinholeRectification(contract['source_intrinsics'],sdk)
    rectifier=_RECTIFIERS[key]
    if rectifier.contract != contract or shape != (rectifier.height,rectifier.width):
        raise ValueError('actual_rectification_contract_not_reproduced')
    owner=np.full((rectifier.source_height,rectifier.source_width),-1,np.int16)
    for i,region in enumerate(regions):
        x0,y0,x1,y1=region['bounds_xyxy']; patch=owner[y0:y1,x0:x1]
        patch[patch<0]=i
        if np.count_nonzero(owner==i) != region['owned_pixels']:
            raise ValueError('recorded_regional_ownership_count_mismatch')
    if np.any(owner<0): raise ValueError('incomplete_regional_owner')
    mapped=np.full(shape,-1,np.int16); mapped.ravel()[rectifier.target_indices]=owner.ravel()
    return mapped,regions,dict(available=True,contract_reproduced_exactly=True,
        depth_forward_map_sha256=contract['depth_forward_map_sha256'],
        coordinate_domain='actual SDK color-aligned owner mapped into published rectified raster',
        filter_effect_causality_established=False)


def bounded_original_message(compressed, expected_bytes):
    if type(expected_bytes) is not int or not 0 < expected_bytes <= 4*1024*1024+65536:
        raise ValueError('original_message_size_out_of_bounds')
    decoder=zlib.decompressobj()
    raw=decoder.decompress(compressed,expected_bytes+1)
    if len(raw)!=expected_bytes or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('original_message_decompression_bound')
    return raw


def short_time_changes(rows):
    result=[]; previous=None
    for row in rows:
        if not row['analysis']['support_available']: continue
        if previous is not None:
            item=dict(previous_source_stamp_ns=previous['source_stamp_ns'],
                current_source_stamp_ns=row['source_stamp_ns'],
                source_interval_sec=(row['source_stamp_ns']-previous['source_stamp_ns'])/1e9,
                physical_cause_inferred=False, target_reference_created=False)
            for domain in ('camera','base'):
                key='plane_'+domain
                if key not in previous['analysis'] or key not in row['analysis']: continue
                if domain=='base':
                    epochs=[value['original_source_pose_evidence']['feedback']['epoch_stamp_ns']
                            for value in (previous,row)]
                    if epochs[0] is None or epochs[0]!=epochs[1]:
                        item['base_comparison_unavailable']='different_or_missing_control_reference_epoch'
                        continue
                a=np.asarray(previous['analysis'][key]); b=np.asarray(row['analysis'][key])
                if a[:3]@b[:3]<0: b=-b
                item[domain]=dict(normal_angle_deg=float(np.degrees(np.arccos(np.clip(a[:3]@b[:3],-1.,1.)))),
                    plane_equation_offset_delta_mm=float((b[3]-a[3])*1000),
                    offset_delta_is_not_target_height_error=True)
            result.append(item)
        previous=row
    return result


def config_invariant(parameter, value):
    if parameter != '/camera': return value
    result={key:value[key] for key in value if key!='runtime_profile'}
    profile=value.get('runtime_profile',{})
    result['runtime_geometry']={key:profile[key] for key in (
        'geometry_intrinsics','geometry_intrinsics_source','geometry_activation_stamp_ns','rectification') if key in profile}
    return result


def uniform_source_pairs(pairs, maximum):
    """Fixed source-time sampling before fitting; never quality-dependent."""
    stamps=[pair.get('source_stamp_ns') for pair in pairs]
    if any(type(stamp) is not int or stamp<=0 for stamp in stamps) or len(set(stamps))!=len(stamps):
        raise ValueError('invalid_or_duplicate_pair_source')
    ordered=sorted(pairs,key=lambda pair:pair['source_stamp_ns'])
    if maximum is None: return ordered
    if type(maximum) is not int or not 1<=maximum<=6:
        raise ValueError('max_pairs_requires_1_to_6')
    if len(ordered)<=maximum: return ordered
    if maximum==1: return [ordered[0]]
    lo,hi=ordered[0]['source_stamp_ns'],ordered[-1]['source_stamp_ns']
    # Integer arithmetic retains nanoseconds even for present-day ROS stamps.
    chosen={min(stamps,key=lambda stamp:(abs((stamp-lo)*(maximum-1)-i*(hi-lo)),stamp))
            for i in range(maximum)}
    return [pair for pair in ordered if pair['source_stamp_ns'] in chosen]


def exact_read_evidence(saved, read, camera):
    """Use only the recorder's hashed exact-source reference, never a neighbour."""
    refs=saved.get('exact_read_history_references',[])
    if saved.get('exact_read_history_unique') is not True or len(refs)!=1:
        return None,dict(available=False,reason='exact_source_history_missing_or_conflicting')
    original=json.loads(read(refs[0]['entry']))
    envelope=original.get('original_parameter_value',{})
    if (original.get('parameter')!='/camera_read_diagnostics' or original.get('original_rpc_code')!=1
            or envelope.get('encoding')!='json'):
        raise ValueError('exact_source_record_schema_invalid')
    payload=json.loads(envelope['payload_json'])
    matches=[source for source in payload['entries'] if source['source_stamp_ns']==saved['source_stamp_ns']]
    if len(matches)!=1: raise ValueError('exact_source_row_not_unique')
    entry=matches[0]
    exact_raw=(json.dumps(entry,ensure_ascii=False,sort_keys=True,indent=2)+'\n').encode()
    if sha(exact_raw)!=refs[0]['source_sha256']: raise ValueError('exact_source_evidence_hash_mismatch')
    activation=camera.get('runtime_profile',{}).get('geometry_activation_stamp_ns')
    if (entry.get('frame_id')!=saved['frame_id'] or entry.get('evidence',{}).get('result')!='complete'
            or activation is None or str(entry.get('geometry_activation_stamp_ns'))!=str(activation)
            or entry.get('geometry_generation')!=payload.get('geometry_generation')):
        raise ValueError('exact_source_camera_owner_conflict')
    return entry,dict(available=True,original_metadata_entry=refs[0]['entry'],
        source_sha256=refs[0]['source_sha256'],copy_errors=payload.get('copy_errors'),
        no_unsampled_history_completeness_claim=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture','--input',dest='capture',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-pairs',type=int,choices=range(1,7),default=None)
    parser.add_argument('--camera-plane-only',action='store_true',
        help='Skip TF/target segmentation and SDK rectification ownership replay; exact metadata is still retained.')
    args=parser.parse_args(); capture=args.capture.resolve(); output=args.output.resolve()
    if EXTERNAL.resolve() not in capture.parents or EXTERNAL.resolve() not in output.parents:
        raise ValueError('external experiment paths required')
    if not Path('/media/zhuyupei/SanStand').is_mount(): raise ValueError('SanStand unavailable; no fallback')
    summary_raw=(capture/'summary.json').read_bytes()
    summary=json.loads(summary_raw)
    checksums={name:checksum for checksum,name in (line.split(None,1)
        for line in (capture/'SHA256SUMS').read_text().splitlines() if line.strip())}
    if checksums.get('summary.json')!=sha(summary_raw) or checksums.get('diagnostic.zip')!=summary.get('archive_sha256'):
        raise ValueError('capture_summary_checksum_mismatch')
    archive_raw=(capture/'diagnostic.zip').read_bytes()
    if not summary['sealed'] or len(archive_raw)>38*1024*1024 or sha(archive_raw)!=summary['archive_sha256']:
        raise ValueError('capture_not_sealed_or_hash_mismatch')
    if summary.get('ready') is not True or summary.get('error') is not None or summary.get('rpc_thread_alive') is not False:
        raise ValueError('capture_failed_or_unfinished')
    del archive_raw
    selected_pairs=uniform_source_pairs(summary['pairs'],args.max_pairs)
    if not selected_pairs: raise ValueError('no_original_exact_pairs_saved')
    cv2.setNumThreads(1)
    report=dict(schema=1,diagnostic_only=True,target_identity=None,motion_authorized=False,
        capture=str(capture),capture_summary_sha256=sha((capture/'summary.json').read_bytes()),
        production_tabletop_sha256=sha(Path(table.__file__).read_bytes()),
        analysis_source_sha256=sha(Path(__file__).read_bytes()),rows=[],quality_thresholds_changed=False,
        camera_plane_only=args.camera_plane_only,total_saved_pairs=len(summary['pairs']),
        selected_sources=[pair['source_stamp_ns'] for pair in selected_pairs],
        sampling='Uniform original source time, at most requested count, fixed before quality fitting; integer ns, no interpolation.',
        limitations=['Current image-centered generic support only; no retired generation, target anchor or grasp acceptance.',
            'Parameter reads are interval evidence, not exposure-synchronous parameter snapshots.',
            'Quality-pass means original support fit/coverage only, not target or multi-view identity acceptance.',
            'Filter qualification/plane quality association does not establish temporal-filter causal benefit.',
            'Five to ten fixed-cadence samples cannot prove absence of intermittent source gaps between them.'])
    with zipfile.ZipFile(capture/'diagnostic.zip') as archive:
        entries={entry['name']:entry for entry in summary['archive_entries']}
        if (set(archive.namelist())!=set(entries) or len(archive.namelist())!=len(entries)
                or len(entries)!=len(summary['archive_entries'])): raise ValueError('archive_entry_list_mismatch')
        def read(name):
            raw=archive.read(name)
            if len(raw)!=entries[name]['bytes'] or sha(raw)!=entries[name]['sha256']:
                raise ValueError('archive_entry_hash_mismatch')
            return raw
        metadata=[(item['entry'],json.loads(read(item['entry']))) for item in summary['parameter_records']]
        parameters={}; optional_defaults={}
        required_parameters=('/camera','/unknown_perception') if args.camera_plane_only else (
            '/camera','/handeye','/unknown_perception','/robot_description')
        for key in required_parameters:
            rows=[(name,row) for name,row in metadata if row.get('parameter')==key and row.get('original_rpc_code')==1]
            if not rows:
                absent=[(name,row) for name,row in metadata if row.get('parameter')==key]
                if (key=='/unknown_perception' and absent and all(
                    row.get('original_rpc_code')==-1 and row.get('original_rpc_message')==
                    'Parameter [/unknown_perception] is not set' for name,row in absent)):
                    parameters[key]={}
                    optional_defaults[key]=dict(original_metadata_entries=[name for name,row in absent],
                        actual_producer_contract="rospy.get_param('/unknown_perception', {}) then production Config defaults",
                        source_path='src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
                        producer_source_sha256=sha((ROOT/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py').read_bytes()),
                        parameter_present=False,effective_config_overrides={})
                    continue
                raise ValueError('missing_current_parameter:'+key)
            first=config_invariant(key,rows[0][1]['original_parameter_value'])
            if any(config_invariant(key,row['original_parameter_value'])!=first for name,row in rows):
                raise ValueError('current_geometry_or_config_changed:'+key)
            parameters[key]=rows[0][1]['original_parameter_value']
        camera=parameters['/camera']; cp=parameters['/unknown_perception']
        cfg=table.Config(**{field.name:cp[field.name] for field in fields(table.Config) if field.name in cp})
        K=[float(camera[key]) for key in ('fx','fy','cx','cy')]
        if not np.all(np.isfinite(K)) or min(K[:2])<=0 or not np.isfinite(camera['depth_scale']) or camera['depth_scale']<=0:
            raise ValueError('invalid_recorded_camera_parameters')
        report.update(config=asdict(cfg),camera_parameters=camera,parameter_observed_invariants=True,
            camera_intrinsics=K,depth_scale_m=camera['depth_scale'],optional_defaults=optional_defaults)
        for saved in selected_pairs:
            pair=json.loads(read(saved['entry'])); stamp=pair['source_stamp_ns']; row=dict(source_stamp_ns=stamp)
            if stamp!=saved['source_stamp_ns'] or len(pair['images'])!=2 or {image['stream'] for image in pair['images']}!={'color','depth'}:
                raise ValueError('pair_index_or_streams_conflict')
            raw_images={}
            for image in pair['images']:
                raw=bounded_original_message(read(image['entry']),image['serialized_bytes'])
                if len(raw)!=image['serialized_bytes'] or sha(raw)!=image['serialized_sha256']:
                    raise ValueError('original_ros_image_hash_mismatch')
                header,pixels=decode_image(raw)
                if header['source_stamp_ns']!=stamp or header['frame_id']!=saved['frame_id']:
                    raise ValueError('source_image_header_conflict')
                if ((header['width'],header['height'])!=(camera['width'],camera['height'])
                        or header['frame_id']!=camera['frame_id'] or header['source_stamp_ns']!=image['source_stamp_ns']
                        or header['seq']!=image['source_seq']):
                    raise ValueError('original_image_camera_or_serialization_conflict')
                raw_images[image['stream']]=(header,pixels)
            if raw_images['color'][1].shape[:2]!=raw_images['depth'][1].shape:
                raise ValueError('paired_geometry_mismatch')
            depth=raw_images['depth'][1].astype(np.float32)*float(camera['depth_scale'])
            row['parameter_intervals']={key:[dict(entry=name,started=record['started'],finished=record['finished'])
                for name,record in metadata if record.get('parameter')==key and record.get('original_rpc_code')==1]
                for key in required_parameters}
            transform=None; at=pair['at_source']; row['original_source_pose_evidence']=at
            if (not args.camera_plane_only and at.get('tf',{} ) and at['tf']['exact_valid']
                    and at['feedback']['source_bracket_available']):
                tf=at['tf']; x,y,z,w=tf['quaternion_xyzw']; transform=np.eye(4)
                transform[:3,:3]=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                transform[:3,3]=tf['translation']
            exact,audit=exact_read_evidence(saved,read,camera)
            row['exact_read_history_unique']=exact is not None
            row['exact_read_history_audit']=audit
            evidence={}
            if exact is not None:
                row['exact_sdk_regional_read']=exact; evidence=exact['evidence']
            if args.camera_plane_only:
                owner,regions,mapping=None,None,dict(available=False,reason='camera_plane_only_no_sdk_owner_replay')
            else: owner,regions,mapping=diagnostic_owner(camera,evidence,depth.shape)
            row['region_mapping']=mapping
            row['analysis']=diagnose_depth(depth,K,transform,cfg,owner,regions)
            if args.camera_plane_only:
                row['analysis']['current_frame_segment']=dict(unavailable='camera_plane_only_requested_no_target_or_tf_authority')
            regional=evidence.get('regional_stationary_evidence',{})
            centers=[region for region in regional.get('regions',[]) if region.get('name')=='center']
            row['same_source_center']=centers[0] if len(centers)==1 else None
            row['same_source_global_history_reset_reason']=regional.get('global_history_reset_reason')
            row['whole_image_valid_depth_fraction']=float(np.mean(
                np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)))
            report['rows'].append(row)
        report['quality_pass_count']=sum(row['analysis']['support_available'] for row in report['rows'])
        report['saved_pair_count']=len(report['rows'])
        report['short_time_changes_current_diagnostic_only']=short_time_changes(report['rows'])
        report['exact_read_history_count']=sum(row['exact_read_history_unique'] for row in report['rows'])
        report['sdk_quality_association_complete']=report['exact_read_history_count']==len(report['rows'])
    output.mkdir(parents=True,exist_ok=False)
    raw=encoded(report); (output/'report.json').write_bytes(raw)
    snapshot=Path(__file__).read_bytes()
    (output/'script_at_analysis.py').write_bytes(snapshot)
    (output/'SHA256SUMS').write_text(sha(raw)+'  report.json\n'+sha(snapshot)+'  script_at_analysis.py\n')
    print(json.dumps(dict(output=str(output),sha256=sha(raw),pairs=len(report['rows']),quality_pass_count=report['quality_pass_count'])))


if __name__=='__main__': main()
