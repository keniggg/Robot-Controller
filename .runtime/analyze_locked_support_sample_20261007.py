"""Replay retired gen22 support audit offline, never restores target authority."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile
import numpy as np

W=Path(__file__).resolve().parents[1]
C=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_20261007T112309.952794Z/current_support_sample')


def main():
    spec=importlib.util.spec_from_file_location('support_diagnostic',W/'.runtime/analyze_current_support_rgbd_20261007.py')
    a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
    r=json.loads((C/'analysis/report.json').read_text());s=json.loads((C/'summary.json').read_text())
    fpath=C.parent/'detector_failure.json'; failure=json.loads(fpath.read_text())['status']
    anchor=SimpleNamespace(position_base=np.asarray(failure['anchor_position_base']),
        support_footprint_base=np.asarray(failure['reference_support_footprint_base']))
    reference=np.asarray(failure['reference_support_plane_base'])
    cfg=a.table.Config(**r['config']);K=r['camera_intrinsics']
    report=dict(diagnostic_only=True,target_recovered=False,motion_authorized=False,
        new_target_selection_created=False,reference_generation=failure['generation'],
        reference_file=str(fpath),reference_file_sha256=a.sha(fpath.read_bytes()),
        capture_archive_sha256=s['archive_sha256'],original_quality_thresholds_changed=False,
        source_sha256=a.sha(Path(__file__).read_bytes()),
        production_tabletop_sha256=a.sha(Path(a.table.__file__).read_bytes()),rows=[],
        limitations=['Retired reference used only for counterfactual offline audit, never to restore identity or geometry authority.',
            'Only original exact-valid TF with accepted measured feedback bracket is admitted; no stamp corrections.',
            'Later sparse capture cannot reproduce the earlier loss interval or prove continuous support availability.'])
    if a.sha((C/'diagnostic.zip').read_bytes())!=s['archive_sha256']:raise ValueError('archive_changed')
    with zipfile.ZipFile(C/'diagnostic.zip') as archive:
        entries={e['name']:e for e in s['archive_entries']}
        def read(name):
            raw=archive.read(name)
            if a.sha(raw)!=entries[name]['sha256']:raise ValueError('archive_entry_changed')
            return raw
        for i,(saved,prior) in enumerate(zip(s['pairs'],r['rows'])):
            at=prior['original_source_pose_evidence']
            row=dict(index=i,source_stamp_ns=prior['source_stamp_ns'])
            report['rows'].append(row)
            if not at['tf']['exact_valid'] or not at['feedback']['source_bracket_available']:
                row['excluded']='original exact TF or feedback bracket unavailable';continue
            tf=at['tf'];x,y,z,w=tf['quaternion_xyzw'];T=np.eye(4)
            T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
            T[:3,3]=tf['translation']
            pair=json.loads(read(saved['entry']));item=next(x for x in pair['images'] if x['stream']=='depth')
            raw=a.bounded_original_message(read(item['entry']),item['serialized_bytes'])
            if a.sha(raw)!=item['serialized_sha256']:raise ValueError('original_depth_changed')
            header,pixels=a.decode_image(raw)
            if header['source_stamp_ns']!=row['source_stamp_ns']:raise ValueError('source_mismatch')
            depth=pixels.astype(np.float32)*r['depth_scale_m']
            try: row['original_locked_audit']=a.table._audit_current_support(depth,K,T,cfg,anchor,reference)
            except ValueError as exc: row['original_locked_audit_error']=str(exc)
            center=a.table.project_base_to_pixel(anchor.position_base,T,K)
            v,u=np.indices(depth.shape);rad=np.hypot(u-center[0],v-center[1]);scale=min(depth.shape[1]/640,depth.shape[0]/480)
            valid=(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)&np.isfinite(depth)
            row.update(projected_anchor_center_uv=center.tolist(),image_center_uv=[depth.shape[1]/2,depth.shape[0]/2],rings=[])
            for outer in (170,140,110):
                ring=valid&(rad>=70*scale)&(rad<=outer*scale)
                pts=a.table._project_pixels(depth[ring],u[ring],v[ring],K)
                fit,plane=a.original_fit(pts,cfg);rr=dict(outer_base_px=outer,valid_points=int(ring.sum()),fit=fit)
                diagnostic_plane=np.asarray(fit['original_locals']['best_plane'])
                all_residual=np.abs(pts@diagnostic_plane[:3]+diagnostic_plane[3])
                rr['full_actual_ring_diagnostic']=dict(
                    inlier_ratio=float(np.mean(all_residual<=cfg.support_threshold_m)),
                    inlier_count=int(np.count_nonzero(all_residual<=cfg.support_threshold_m)),
                    abs_residual_mm_p50_p90_p99=np.percentile(all_residual*1000,[50,90,99]).tolist(),
                    diagnostic_only=True,does_not_replace_original_sample_gate=True)
                if plane is not None:
                    residual=pts@plane[:3]+plane[3]
                    rr['quadrants']=a.table._support_quadrants(abs(residual)<=cfg.support_threshold_m,u[ring],v[ring],center)
                    current=a.table._normalized_plane(np.linalg.solve(T.T,plane))
                    try:
                        _,agreement=a.table._validate_support_plane_update(reference,current,anchor.position_base,cfg,anchor.support_footprint_base)
                        rr['reference_comparison']=agreement
                    except ValueError as exc:rr['reference_comparison_error']=str(exc)
                row['rings'].append(rr)
    report['admitted_sample_count']=sum('excluded' not in row for row in report['rows'])
    report['locked_audit_qualified_count']=sum(row.get('original_locked_audit',{}).get('support_current_audit_available') is True for row in report['rows'])
    raw=a.encoded(report);out=C/'analysis/locked_reference_audit.json';out.write_bytes(raw)
    out.with_suffix('.sha256').write_text(a.sha(raw)+'  '+out.name+'\n')
    print(json.dumps(dict(output=str(out),sha256=a.sha(raw),admitted=report['admitted_sample_count'],qualified=report['locked_audit_qualified_count'])))


if __name__=='__main__':main()
