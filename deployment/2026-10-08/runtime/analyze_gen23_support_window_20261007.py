"""Bounded offline replay of archived gen23 sources, TF and support pending.

Uses an in-process tf2 buffer only. No ROS node, listener, master or devices.
"""
from collections import Counter
from dataclasses import fields
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile
import numpy as np

W=Path(__file__).resolve().parents[1]
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_20261007T112309.952794Z/recorded_retry_20261007T120118Z/raw_support')


def main():
    import rospy
    import tf2_py
    from tf2_msgs.msg import TFMessage
    if rospy.core.is_initialized():raise RuntimeError('offline_only')
    spec=importlib.util.spec_from_file_location('support_analysis',W/'.runtime/analyze_current_support_rgbd_20261007.py')
    a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
    summary=json.loads((C/'summary.json').read_text())
    if not summary['archive_closed'] or not summary['complete_without_writer_error']:
        raise ValueError('archive_not_closed')
    report=dict(diagnostic_only=True,generation=23,motion_authorized=False,target_recovered=False,
        new_selection_created=False,source_sha256=a.sha(Path(__file__).read_bytes()),
        production_tabletop_sha256=a.sha(Path(a.table.__file__).read_bytes()),
        capture_summary_sha256=a.sha((C/'summary.json').read_bytes()),
        archive_sha256=a.sha((C/'diagnostic.zip').read_bytes()),timeline=[],rows=[],
        accepted_feedback_independently_recorded=False,
        limitations=['Only this sealed 15-second source window; later first loss is outside capture.',
            'Original ROS published rectified RGB-D, not SDK raw images.',
            'TF reconstructed from this capture only with original integer timestamps; returned timestamp delta retained.',
            'Recorder has no accepted_joint_states stream. Production source qualification is not independent feedback-bracket proof.',
            'Original thresholds and sampling unchanged; full-ring ratios are diagnostic only.',
            'No frozen target is revived and no sample authorizes a grasp.'])
    with zipfile.ZipFile(C/'diagnostic.zip') as archive:
        entries={e['name']:e for e in json.loads(archive.read('entry_manifest.json'))}
        if set(archive.namelist())!={'entry_manifest.json'}|set(entries):raise ValueError('manifest_list_mismatch')
        def read(name):
            raw=archive.read(name)
            if len(raw)!=entries[name]['bytes'] or a.sha(raw)!=entries[name]['sha256']:raise ValueError('entry_mismatch:'+name)
            return raw
        metadata=[json.loads(read(n)) for n in entries if n.startswith('metadata/')]
        first=metadata[0]['parameters'];camera=first['/camera'];cfgraw=first['/unknown_perception']
        for meta in metadata:
            if meta['errors']:raise ValueError('metadata_errors')
            for key in ('/camera','/handeye','/unknown_perception','/robot_description'):
                if a.config_invariant(key,meta['parameters'][key])!=a.config_invariant(key,first[key]):
                    raise ValueError('changed_config:'+key)
        cfg=a.table.Config(**{f.name:cfgraw[f.name] for f in fields(a.table.Config) if f.name in cfgraw})
        K=[camera[k] for k in ('fx','fy','cx','cy')]
        histories={}
        for meta in metadata:
            envelope=meta['parameters'].get('/camera_read_diagnostics',{})
            for entry in json.loads(envelope.get('payload_json','{}')).get('entries',[]):
                ns=entry['source_stamp_ns']
                if ns in histories and histories[ns]!=entry:raise ValueError('history_conflict')
                histories[ns]=entry
        tfbuffer=tf2_py.BufferCore(rospy.Duration(60))
        tfmeta=[]
        for name in entries:
            if name.startswith('tf/') and name.endswith('.json'):
                d=json.loads(read(name));tfmeta.append((d['receipt_ros_ns'],name,d))
        for _,name,d in sorted(tfmeta):
            raw=a.bounded_original_message(read(name[:-5]+'.rosmsg.zlib'),d['serialized_bytes'])
            if a.sha(raw)!=d['serialized_sha256']:raise ValueError('tf_serialized_hash')
            msg=TFMessage().deserialize(raw)
            for transform in msg.transforms:
                setter=tfbuffer.set_transform_static if d['topic']=='/tf_static' else tfbuffer.set_transform
                setter(transform,d['original_connection_header'].get('callerid','archived'))
        statuses=[]
        for name in sorted(entries):
            if not name.startswith('status/') or not name.endswith('.json'):continue
            envelope=json.loads(read(name));raw=envelope['original_status_data'].encode()
            if a.sha(raw)!=envelope['original_status_utf8_sha256']:raise ValueError('status_original_hash')
            status=json.loads(raw)
            if status.get('generation')!=23:continue
            statuses.append((name,envelope,status))
            report['timeline'].append(dict(entry=name,receipt_ros_ns=envelope['receipt_ros_ns'],
                source_stamp_ns=status.get('source_stamp_ns'),state=status['state'],detail=status.get('detail'),
                target_lost=status.get('target_lost'),first_loss=status.get('first_loss'),
                source_qualification=status.get('source_qualification'),frame_timing=status.get('frame_timing')))
        report['status_counts']={state+':'+detail:count for (state,detail),count in Counter((s['state'],s.get('detail','')) for _,_,s in statuses).items()}
        report['first_loss_in_capture']=next((s['first_loss'] for _,_,s in statuses if s.get('first_loss')),None)
        for name,envelope,status in statuses:
            ns=status.get('source_stamp_ns');pairname='pairs/%s.json'%ns
            if pairname not in entries:continue
            pair=json.loads(read(pairname));item=pair['streams']['depth']
            raw=a.bounded_original_message(read('pairs/%s_depth.rosmsg.zlib'%ns),item['serialized_bytes'])
            if a.sha(raw)!=item['serialized_sha256']:raise ValueError('depth_original_hash')
            header,pixels=a.decode_image(raw)
            if header['source_stamp_ns']!=ns:raise ValueError('pair_source_conflict')
            depth=pixels.astype(np.float32)*float(camera['depth_scale'])
            row=dict(status_entry=name,source_stamp_ns=ns,original_state=status['state'],original_detail=status.get('detail'),
                original_qualification=status.get('source_qualification'),rings=[],source_history_available=ns in histories)
            report['rows'].append(row)
            reg=histories.get(ns,{}).get('evidence',{}).get('regional_stationary_evidence',{})
            row['center_filter']=next((r for r in reg.get('regions',[]) if r['name']=='center'),None)
            row['global_history_reset_reason']=reg.get('global_history_reset_reason')
            if 'anchor_position_base' not in status:
                row['initial_image_center_analysis']=a.diagnose_depth(depth,K,None,cfg);continue
            try:
                tf=tfbuffer.lookup_transform_core('base_link','camera_link',rospy.Time(*divmod(ns,10**9)))
            except Exception as exc:
                row['tf_error']=repr(exc);continue
            row['tf_return_delta_ns']=tf.header.stamp.to_nsec()-ns
            row['tf_exact_return']=row['tf_return_delta_ns']==0
            row['pose_qualification_not_asserted_without_independent_feedback']=True
            q=tf.transform.rotation;t=tf.transform.translation;x,y,z,w=q.x,q.y,q.z,q.w
            T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];T[:3,3]=[t.x,t.y,t.z]
            row['tf_matrix']=T.tolist()
            anchor=SimpleNamespace(position_base=np.asarray(status['anchor_position_base']),
                support_footprint_base=np.asarray(status['reference_support_footprint_base']))
            reference=np.asarray(status['reference_support_plane_base'])
            try:row['original_locked_audit']=a.table._audit_current_support(depth,K,T,cfg,anchor,reference)
            except ValueError as exc:row['original_locked_audit_error']=str(exc)
            center=a.table.project_base_to_pixel(anchor.position_base,T,K);row['projected_anchor_center_uv']=center.tolist()
            v,u=np.indices(depth.shape);rad=np.hypot(u-center[0],v-center[1]);scale=min(depth.shape[1]/640,depth.shape[0]/480)
            valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
            for outer in (170,140,110):
                ring=valid&(rad>=70*scale)&(rad<=outer*scale)
                pts=a.table._project_pixels(depth[ring],u[ring],v[ring],K)
                fit,plane=a.original_fit(pts,cfg);rr=dict(outer_base_px=outer,valid_points=int(ring.sum()),fit=fit)
                diagnostic=fit['original_locals'].get('best_plane')
                if diagnostic is not None:
                    residual=np.abs(pts@np.asarray(diagnostic)[:3]+diagnostic[3])
                    rr['all_actual_ring_inlier_ratio_diagnostic']=float(np.mean(residual<=cfg.support_threshold_m))
                if plane is not None:
                    rr['quadrants']=a.table._support_quadrants(abs(pts@plane[:3]+plane[3])<=cfg.support_threshold_m,u[ring],v[ring],center)
                row['rings'].append(rr)
        report['saved_new_generation_pair_count']=len(report['rows'])
        report['pending_replay']=[r for r in report['rows'] if r['original_detail']=='support_measurement_pending_revalidation']
    report['ros_node_initialized_after']=rospy.core.is_initialized()
    if report['ros_node_initialized_after']:raise RuntimeError('unexpected_ros_node')
    out=C/'analysis';out.mkdir(exist_ok=False)
    raw=a.encoded(report);(out/'report.json').write_bytes(raw)
    (out/'SHA256SUMS').write_text(a.sha(raw)+'  report.json\n')
    print(json.dumps(dict(output=str(out),sha256=a.sha(raw),status_counts=report['status_counts'],
        first_loss_in_capture=report['first_loss_in_capture'],paired_statuses=len(report['rows']),pending=len(report['pending_replay']))))


if __name__=='__main__':main()
