#!/usr/bin/env python3
"""Three fixed, exact saved generation-25 frames; offline original-fit replay."""
from dataclasses import asdict, fields
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
CASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_resume_20261008T023234Z/selection')
SOURCES = (1791426951605816602, 1791426954621882677, 1791426957410692930)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    helper_path = WORK/'.runtime/analyze_current_support_rgbd_20261007.py'
    spec = importlib.util.spec_from_file_location('original_support_analysis', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    helper.cv2.setNumThreads(1)
    raw = CASE/'raw_support'
    summary = json.loads((raw/'summary.json').read_text())
    assert summary['archive_closed'] and summary['complete_without_writer_error']
    assert summary['cooperative_total_deadline_passed']
    out = CASE/'analysis/three_fresh_support_frames'
    out.mkdir(exist_ok=False)
    sources = {str(helper_path): sha(helper_path.read_bytes()),
               str(Path(helper.table.__file__)): sha(Path(helper.table.__file__).read_bytes()),
               str(WORK/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'):
                   sha((WORK/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py').read_bytes()),
               str(Path(__file__).resolve()): sha(Path(__file__).read_bytes())}
    report = dict(schema=1, offline_only=True, sdk_imported=False, device_or_ros_calls=False,
        generation=25, camera_plane_only=True, transform_used=None, source_files=sources,
        input_summary_sha256=sha((raw/'summary.json').read_bytes()),
        chosen_sources=list(SOURCES), selection_reason='First saved fresh failed frame after the receive stall, middle and late fresh failed frames; fixed before fitting.',
        quality_thresholds_changed=False, image_pixels_modified=False, original_entries_verified={}, rows=[],
        limitations=['Recorded parameter intervals agree; they are not exposure-synchronous snapshots.',
            'Parameter absence was explicitly recorded and the producer uses Config defaults, rather than guessing a missing setting.',
            'Rejected best-plane residual diagnostics do not authorize a grasp or convert a rejected plane into accepted geometry.',
            'No base-frame transform, target selection, multiview registration or physical-success claim is inferred.',
            'Three frames diagnose lack of planar consensus; they do not alone distinguish random depth noise from systematic distortion or physical nonplanarity.'])
    with zipfile.ZipFile(raw/'diagnostic.zip') as archive:
        manifest = json.loads(archive.read('entry_manifest.json'))
        by_name = {entry['name']:entry for entry in manifest}
        report['entry_manifest_sha256'] = sha(archive.read('entry_manifest.json'))
        def read(name):
            value=archive.read(name); meta=by_name[name]
            assert len(value)==meta['bytes'] and sha(value)==meta['sha256']
            report['original_entries_verified'][name]=meta['sha256']
            return value
        metadata=[]; statuses={}; sdk={}
        for name in archive.namelist():
            if name.startswith('metadata/'):
                record=json.loads(read(name)); metadata.append((name,record))
                envelope=record['parameters']['/camera_read_diagnostics']
                assert envelope['encoding']=='json'
                for e in json.loads(envelope['payload_json'])['entries']:
                    ns=e['source_stamp_ns']; assert ns not in sdk or sdk[ns]==e; sdk[ns]=e
            if name.startswith('status/'):
                record=json.loads(read(name)); data=json.loads(record['original_status_data'])
                assert sha(record['original_status_data'].encode())==record['original_status_utf8_sha256']
                if data.get('generation')==25 and data.get('source_stamp_ns') in SOURCES:
                    assert data['source_stamp_ns'] not in statuses
                    statuses[data['source_stamp_ns']]=(name,record,data)
        camera=metadata[0][1]['parameters']['/camera']
        overrides=metadata[0][1]['parameters']['/unknown_perception']
        parameter_intervals=[]
        for name,record in metadata:
            assert helper.config_invariant('/camera',record['parameters']['/camera'])==helper.config_invariant('/camera',camera)
            assert record['parameters']['/unknown_perception']==overrides
            assert record['absent_optional_parameter_defaults']['/unknown_perception']=={
                'effective_default':{},'parameter_present':False,
                'semantics':"producer get_param('/unknown_perception', {}) followed by Config defaults"}
            parameter_intervals.append(dict(entry=name,read_start_ros_ns=record['read_start_ros_ns'],read_end_ros_ns=record['read_end_ros_ns']))
        assert overrides=={}
        cfg=helper.table.Config(**{field.name:overrides[field.name] for field in fields(helper.table.Config) if field.name in overrides})
        K=[float(camera[k]) for k in ('fx','fy','cx','cy')]
        scale=float(camera['depth_scale'])
        report.update(config=asdict(cfg),camera_intrinsics=K,depth_scale_m=scale,
            parameter_read_intervals=parameter_intervals,
            explicit_absent_parameter_default=True,producer_config_contract='unknown_tabletop_perception.py:160-162')
        for stamp in SOURCES:
            status_name, original, status=statuses[stamp]
            assert status['detail']=='perception_failure: support_plane_inlier_ratio_low'
            assert status.get('target_locked') is False and status.get('target_lost') is False
            assert 0<=status['frame_timing']['status_source_age_sec']<status['frame_timing']['max_frame_age_sec']
            pair=json.loads(read(f'pairs/{stamp}.json'))
            assert pair['exact_source_stamp_match'] is True and pair['source_stamp_ns']==stamp
            images={};headers={}
            for stream in ('color','depth'):
                image=pair['streams'][stream]
                packed=helper.bounded_original_message(read(f'pairs/{stamp}_{stream}.rosmsg.zlib'),image['serialized_bytes'])
                assert sha(packed)==image['serialized_sha256']
                header,pixels=helper.decode_image(packed)
                assert header['source_stamp_ns']==stamp and header['frame_id']==camera['frame_id']
                assert header['seq']==image['source_seq']==status['color_seq' if stream=='color' else 'depth_seq']
                assert (header['width'],header['height'])==(camera['width'],camera['height'])
                headers[stream]=header;images[stream]=pixels
            assert images['color'].shape[:2]==images['depth'].shape
            depth=images['depth'].astype(np.float32)*scale
            analysis=helper.diagnose_depth(depth,K,None,cfg)
            h,w=depth.shape;v,u=np.indices(depth.shape);center=np.array([w/2.,h/2.])
            radius=np.hypot(u-center[0],v-center[1]);rscale=min(w/640.,h/480.)
            valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
            for row in analysis['rings']:
                plane=np.asarray(row['fit']['original_locals']['best_plane'],dtype=float)
                ring=valid&(radius>=70*rscale)&(radius<=row['outer_base_px']*rscale)
                points=helper.table._project_pixels(depth[ring],u[ring],v[ring],K)
                absolute=np.abs(np.einsum('ij,j->i',points,plane[:3])+plane[3])
                row['original_rejected_model_all_points_diagnostic']={
                    'inlier_ratio':float(np.mean(absolute<=cfg.support_threshold_m)),
                    'abs_residual_mm_p50_p90_p99':np.percentile(absolute*1000,[50,90,99]).tolist(),
                    'within_original_2mm_quadrants':helper.table._support_quadrants(absolute<=cfg.support_threshold_m,u[ring],v[ring],center),
                    'metric_geometry_authority':False,'no_new_fit_or_changed_model':True}
            evidence=sdk[stamp]['evidence'];regional=evidence['regional_stationary_evidence']
            report['rows'].append(dict(source_stamp_ns=stamp, original_status_entry=status_name,
                original_status_sha256=original['original_status_utf8_sha256'],status_source_age_sec=status['frame_timing']['status_source_age_sec'],
                original_pair_headers=headers,whole_image_valid_depth_fraction=float(np.mean(valid)),
                original_support_replay=analysis,same_source_camera_center=next(r for r in regional['regions'] if r['name']=='center'),
                same_source_camera_global_reset_reason=regional.get('global_history_reset_reason')))
    assert 'pyrealsense2' not in sys.modules and 'rospy' not in sys.modules
    report['all_original_sequences_rejected_at_ratio_gate']=all(
        not r['original_support_replay']['support_available'] and
        r['original_support_replay'].get('support_reason')=='support_plane_inlier_ratio_low'
        for r in report['rows'])
    report['conclusion']=('All three original 170/140/110 ring sequences reject at the unchanged original inlier-ratio gate.'
        if report['all_original_sequences_rejected_at_ratio_gate'] else
        'Replay differs from at least one original producer rejection; inspect configuration/geometry/source provenance before attributing cause.')
    (out/'script_at_analysis.py').write_bytes(Path(__file__).read_bytes())
    path=out/'report.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'report':str(path),'sha256':sha(path.read_bytes()),'sources':sources,
        'rows':[{'source':r['source_stamp_ns'],'status_source_age_sec':r['status_source_age_sec'],'rings':[
            {'outer':x['outer_base_px'],'valid_points':x['valid_points'],'valid_fraction':x['valid_fraction'],
             'fit':x['fit'],'full_residual':x['original_rejected_model_all_points_diagnostic']}
            for x in r['original_support_replay']['rings']]} for r in report['rows']]},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
