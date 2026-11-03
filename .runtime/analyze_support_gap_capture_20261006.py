#!/usr/bin/env python3
"""Offline sealed diagnostic analysis; no ROS node or online service calls."""
import argparse
from collections import Counter
from dataclasses import asdict, fields
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/alicia_grasp_modes/src'))
import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop as table


def digest(data):
    return hashlib.sha256(data).hexdigest()


def fit_diagnostic(points, cfg):
    """Inspect the original function's locals, including rejected consensus.

    No input, threshold, sampling, return value or exception is modified.
    A rejected fit remains rejected; its model is descriptive evidence only.
    """
    captured = {}
    target = table._fit_plane.__code__
    previous = sys.gettrace()

    def trace(frame, event, arg):
        if frame.f_code is not target:
            return None
        if event in ('return', 'exception'):
            for name in ('best_plane', 'best_count', 'best_residual', 'ratio'):
                if name in frame.f_locals:
                    value = frame.f_locals[name]
                    captured[name] = value.copy() if isinstance(value, np.ndarray) else value
            captured['sample_count'] = len(frame.f_locals['points'])
        return trace
    try:
        sys.settrace(trace)
        plane, ratio, residual, count = table._fit_plane(points, cfg)
        result = dict(qualified=True, ratio=ratio, residual_m=residual, sample_count=count)
    except ValueError as exc:
        result = dict(qualified=False, error=str(exc))
    finally:
        sys.settrace(previous)
    plane = captured.get('best_plane')
    result['original_function_locals'] = {k:(v.tolist() if isinstance(v, np.ndarray) else v)
        for k,v in captured.items()}
    if plane is not None:
        plane = table._normalized_plane(plane)
        if plane[3] < 0:
            plane = -plane
        residuals = np.einsum('ij,j->i', points, plane[:3])+plane[3]
        result['all_ring_points_inlier_ratio_same_model'] = float(np.mean(abs(residuals) <= cfg.support_threshold_m))
        result['all_ring_points_abs_residual_percentiles_mm'] = np.percentile(abs(residuals)*1000, [25,50,70,90,95,99]).tolist()
    return result, plane


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--generation', type=int, default=17)
    args = parser.parse_args()
    summary = json.loads((args.capture/'summary.json').read_text())
    assert summary['archive_closed'] and summary['complete_without_writer_error']
    assert summary['byte_budget_passed'] and summary['directory_budget_passed']
    ext = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and ext in args.output.resolve().parents
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    bridge = CvBridge()
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.), debug=False)
    with zipfile.ZipFile(args.capture/'diagnostic.zip') as archive:
        names = archive.namelist()
        metadata = [json.loads(archive.read(name)) for name in names if name.startswith('metadata/')]
        parameters = metadata[0]['parameters']
        invariant_keys = ('/unknown_perception','/handeye','/robot_description')
        for key in invariant_keys:
            assert all(row['parameters'][key] == parameters[key] for row in metadata), key+' changed'
        cfg_params = parameters['/unknown_perception']
        cfg = table.Config(**{f.name:cfg_params[f.name] for f in fields(table.Config) if f.name in cfg_params})
        camera = parameters['/camera']
        # Runtime stationary-filter counters describe changing measurements;
        # they are retained but are not camera projection/calibration inputs.
        invariant_camera = {key:value for key,value in camera.items() if key != 'runtime_profile'}
        assert all({key:value for key,value in row['parameters']['/camera'].items()
                    if key != 'runtime_profile'} == invariant_camera for row in metadata)
        intrinsics = [float(camera[key]) for key in ('fx','fy','cx','cy')]
        base_frame = str(cfg_params.get('base_frame','base_link'))
        camera_frame = str(cfg_params.get('camera_frame',camera.get('frame_id','camera_link')))
        dynamic = []
        for name in names:
            if not name.startswith('tf/') or not name.endswith('.rosmsg.zlib'):
                continue
            msg = TFMessage().deserialize(zlib.decompress(archive.read(name)))
            for transform in msg.transforms:
                if name.startswith('tf/tf_static_'):
                    buffer.set_transform_static(transform, 'sealed_original_messages')
                else:
                    dynamic.append(transform)
        for transform in sorted(dynamic, key=lambda tr:tr.header.stamp.to_nsec()):
            buffer.set_transform(transform,'sealed_original_messages')
        statuses = []
        for name in sorted(n for n in names if n.startswith('status/')):
            row = json.loads(archive.read(name)); value = json.loads(row['original_status_data'])
            if value.get('generation') == args.generation:
                statuses.append((name,row,value))
        pending = [row for row in statuses if row[2].get('detail') == 'support_measurement_pending_revalidation']
        ready = [row for row in statuses if row[2].get('state') == 'ready']
        failed = [row for row in statuses if row[2].get('first_loss')]
        selection = (ready[:1]+ready[-3:]+pending[:12]+failed[:1]) if pending else ready[::max(1,len(ready)//5)][:6]
        seen = set(); selected = []
        for item in selection:
            stamp = item[2].get('source_stamp_ns')
            if stamp not in seen and stamp is not None:
                selected.append(item); seen.add(stamp)
        report = dict(schema_version=1, generation=args.generation, capture=str(args.capture),
            capture_summary_sha256=digest((args.capture/'summary.json').read_bytes()),
            source_sha256={str(path.relative_to(ROOT)):digest(path.read_bytes()) for path in
                (Path(__file__).resolve(), ROOT/'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py')},
            config=asdict(cfg), parameter_snapshots_checked=len(metadata),
            parameter_provenance='Captured parameter namespace and production Config defaults; projection/config/handeye/URDF invariant across all snapshots; runtime_profile filter statistics vary and are retained in original metadata; reads are not exposure-synchronous',
            optional_defaults=metadata[0].get('absent_optional_parameter_defaults',{}),
            intrinsics=intrinsics, frames=dict(base=base_frame,camera=camera_frame),
            status_counts=dict(Counter(row[2].get('detail') for row in statuses)),
            pending_reproduced_in_this_generation=bool(pending),
            no_hardware_or_parameter_changes=True, no_quality_threshold_changes=True, rows=[])
        for name,event,status in selected:
            ns = status['source_stamp_ns']
            row = dict(source_stamp_ns=ns,status_entry=name,detail=status['detail'],
                recorded_context=event['context'],
                recorded_source_qualification=status.get('source_qualification'),
                recorded_geometry_metrics=status.get('geometry_metrics'))
            pairname='pairs/%d.json'%ns
            if pairname not in names:
                row['error']='missing_recorded_exact_pair';report['rows'].append(row);continue
            info=json.loads(archive.read(pairname))
            depth_msg=Image().deserialize(zlib.decompress(archive.read('pairs/%d_depth.rosmsg.zlib'%ns)))
            assert depth_msg.header.stamp.to_nsec()==ns==status['depth_stamp_ns']==status['color_stamp_ns']
            assert depth_msg.header.frame_id==camera_frame
            raw=bridge.imgmsg_to_cv2(depth_msg,'passthrough')
            if depth_msg.encoding in ('16UC1','mono16'):
                depth=np.asarray(raw,dtype=np.float32)*float(camera['depth_scale'])
            elif depth_msg.encoding=='32FC1':depth=np.asarray(raw,dtype=np.float32)
            else:raise ValueError('unsupported original depth encoding')
            transform_msg=buffer.lookup_transform_core(base_frame,camera_frame,depth_msg.header.stamp)
            q=transform_msg.transform.rotation;t=transform_msg.transform.translation
            transform=quaternion_matrix([q.x,q.y,q.z,q.w]);transform[:3,3]=[t.x,t.y,t.z]
            row['tf_lookup_requested_source_stamp_ns']=ns
            row['tf_library_returned_header_stamp_ns']=transform_msg.header.stamp.to_nsec()
            row['tf_library_returned_header_delta_ns']=transform_msg.header.stamp.to_nsec()-ns
            if 'anchor_position_base' not in status:
                row['error']='original_status_has_no_anchor';report['rows'].append(row);continue
            anchor=SimpleNamespace(position_base=np.asarray(status['anchor_position_base']),
                support_footprint_base=np.asarray(status['reference_support_footprint_base']))
            reference=np.asarray(status['reference_support_plane_base'])
            center=table.project_base_to_pixel(anchor.position_base,transform,intrinsics)
            v,u=np.indices(depth.shape);scale=min(depth.shape[1]/640.,depth.shape[0]/480.)
            radius=np.hypot(u-center[0],v-center[1])
            valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
            try:
                audit=table._audit_current_support(depth,intrinsics,transform,cfg,anchor,reference)
                row['original_audit']=audit
            except ValueError as exc:row['original_audit_error']=str(exc)
            row.update(center_uv=center.tolist(),T_base_camera=transform.tolist(),
                anchor_position_base=anchor.position_base.tolist(),reference_support_plane_base=reference.tolist(),
                reference_support_footprint_base=anchor.support_footprint_base.tolist(),rings=[])
            reference_camera=transform.T@reference
            sequence_active=True
            for outer in (170,140,110):
                region=(radius>=70*scale)&(radius<=outer*scale);ring=region&valid
                pu,pv=u[ring],v[ring];points=table._project_pixels(depth[ring],pu,pv,intrinsics)
                fit,plane=fit_diagnostic(points,cfg)
                entry=dict(outer_base_px=outer,outer_actual_px=outer*scale,
                    valid_points=len(points),valid_fraction=float(ring.sum()/region.sum()),
                    consulted_by_original_ring_sequence=sequence_active,fit=fit)
                if sequence_active and (fit['qualified'] or fit.get('error')!='support_plane_inlier_ratio_low'):
                    sequence_active=False
                if plane is not None:
                    residuals=np.einsum('ij,j->i',points,plane[:3])+plane[3]
                    support=abs(residuals)<=cfg.support_threshold_m
                    entry['quadrants']=table._support_quadrants(support,pu,pv,center)
                    current=table._normalized_plane(np.linalg.solve(transform.T,plane))
                    try:
                        _,agreement=table._validate_support_plane_update(reference,current,anchor.position_base,cfg,anchor.support_footprint_base)
                        entry.update(model_reference_consistent=True,model_reference_agreement=agreement)
                    except ValueError as exc:entry.update(model_reference_consistent=False,model_reference_error=str(exc))
                    residual=np.einsum('ij,j->i',points,reference_camera[:3])+reference_camera[3]
                    entry['frozen_reference_inlier_ratio']=float(np.mean(abs(residual)<=cfg.support_threshold_m))
                    entry['frozen_reference_signed_percentiles_mm']=np.percentile(residual*1000,[1,5,10,25,50,75,90,95,99]).tolist()
                    entry['diagnostic_model_does_not_override_failed_quality']=True
                row['rings'].append(entry)
            report['rows'].append(row)
            print(json.dumps(dict(source_stamp_ns=ns,detail=row['detail'],audit=row.get('original_audit'),
                rings=[dict(outer=x['outer_base_px'],qualified=x['fit']['qualified'],
                    ratio=x['fit']['original_function_locals'].get('ratio'),
                    full_ratio=x['fit'].get('all_ring_points_inlier_ratio_same_model'),
                    quadrants=x.get('quadrants'),agreement=x.get('model_reference_agreement')) for x in row['rings']])),flush=True)
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
