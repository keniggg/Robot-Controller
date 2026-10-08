"""Offline bounded summary and original-depth residual visualization; no ROS."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

W = Path(__file__).resolve().parents[1]
C = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_20261007T112309.952794Z/current_support_sample')
O = C/'analysis'


def main():
    spec = importlib.util.spec_from_file_location('original_support_analysis', W/'.runtime/analyze_current_support_rgbd_20261007.py')
    source = importlib.util.module_from_spec(spec); spec.loader.exec_module(source)
    report = json.loads((O/'report.json').read_text())
    sealed = json.loads((C/'summary.json').read_text())
    if hashlib.sha256((C/'diagnostic.zip').read_bytes()).hexdigest() != sealed['archive_sha256']:
        raise ValueError('archive_changed')
    summary = dict(diagnostic_only=True, target_identity=None, motion_authorized=False,
        report_sha256=hashlib.sha256((O/'report.json').read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_quality_pass_count=report['quality_pass_count'], sample_count=len(report['rows']),
        original_selected_rings=dict(Counter(str(r['analysis']['selected_outer_base_px']) for r in report['rows'])),
        rings={}, centers=[], history={}, residual_diagnostics=[], limitations=report['limitations'])
    for k, outer in enumerate((170,140,110)):
        rings = [r['analysis']['rings'][k] for r in report['rows']]
        fits = [r['fit'] for r in rings]
        summary['rings'][outer] = dict(qualified_count=sum(f['original_fit_qualified'] for f in fits),
            ratio_min_max=[min(f['original_locals']['ratio'] for f in fits),max(f['original_locals']['ratio'] for f in fits)],
            original_inlier_residual_mm_min_max=[min(f['original_locals']['best_residual'] for f in fits)*1000,max(f['original_locals']['best_residual'] for f in fits)*1000],
            valid_fraction_min_max=[min(r['valid_fraction'] for r in rings),max(r['valid_fraction'] for r in rings)],
            valid_points_min_max=[min(r['valid_points'] for r in rings),max(r['valid_points'] for r in rings)])
    for i,row in enumerate(report['rows']):
        region = row.get('exact_sdk_regional_read',{}).get('evidence',{}).get('regional_stationary_evidence',{})
        center = next((r for r in region.get('regions',[]) if r['name']=='center'),None)
        summary['centers'].append(dict(index=i,source_stamp_ns=row['source_stamp_ns'],center=center,
            global_history_reset_reason=region.get('global_history_reset_reason'),
            same_source_exact_evidence=center is not None))
    with zipfile.ZipFile(C/'diagnostic.zip') as archive:
        expected = {e['name']:e for e in sealed['archive_entries']}
        def read(name):
            raw=archive.read(name)
            if hashlib.sha256(raw).hexdigest()!=expected[name]['sha256']:
                raise ValueError('archive_entry_changed')
            return raw
        rows={}
        for meta in sealed['parameter_records']:
            if meta['parameter']!='/camera_read_diagnostics': continue
            snapshot=json.loads(read(meta['entry']))
            value=snapshot.get('original_parameter_value',{})
            if snapshot.get('original_rpc_code')!=1 or not isinstance(value,dict): continue
            for entry in json.loads(value.get('payload_json','{}')).get('entries',[]):
                ns=entry['source_stamp_ns']; old=rows.get(ns)
                if old is not None and old!=entry: raise ValueError('conflicting_exact_history')
                rows[ns]=entry
        low=report['rows'][0]['source_stamp_ns']; high=report['rows'][-1]['source_stamp_ns']
        history=[r for ns,r in sorted(rows.items()) if low<=ns<=high]
        centers=[next((r for r in e.get('evidence',{}).get('regional_stationary_evidence',{}).get('regions',[]) if r['name']=='center'),{}) for e in history]
        summary['history']=dict(unique_source_entries=len(history),source_start_ns=low,source_end_ns=high,
            center_reasons=dict(Counter(c.get('visual',{}).get('reason','missing') for c in centers)),
            center_filter_applied=sum(c.get('filter_applied') is True for c in centers),
            global_reset_reasons=dict(Counter(e.get('evidence',{}).get('regional_stationary_evidence',{}).get('global_history_reset_reason') or 'none' for e in history)),
            source_history_is_read_evidence_not_complete_published_stream=True)
        fig, axes=plt.subplots(3,3,figsize=(13,10),constrained_layout=True)
        for rownum, index in enumerate((0,2,8)):
            saved=sealed['pairs'][index]; pair=json.loads(read(saved['entry'])); images={}
            for item in pair['images']:
                raw=source.bounded_original_message(read(item['entry']),item['serialized_bytes'])
                if hashlib.sha256(raw).hexdigest()!=item['serialized_sha256']: raise ValueError('image_changed')
                _,images[item['stream']]=source.decode_image(raw)
            depth=images['depth'].astype(np.float32)*report['depth_scale_m']
            h,w=depth.shape; v,u=np.indices(depth.shape); valid=(depth>=.03)&(depth<=2.)
            xyz=source.table._project_pixels(depth,u,v,report['camera_intrinsics'])
            plane=np.asarray(report['rows'][index]['analysis']['plane_camera'])
            residual=(np.einsum('ijk,k->ij',xyz,plane[:3])+plane[3])*1000
            residual[~valid]=np.nan
            scale=min(w/640,h/480); radius=np.hypot(u-w/2,v-h/2)
            ring=(radius>=70*scale)&(radius<=170*scale)&valid
            summary['residual_diagnostics'].append(dict(index=index,
                signed_ring_residual_mm_p01_p10_p50_p90_p99=np.percentile(residual[ring],[1,10,50,90,99]).tolist(),
                ring_outside_original_2mm_fraction=float(np.mean(abs(residual[ring])>2)),
                accepted_plane_outer_px=report['rows'][index]['analysis']['selected_outer_base_px'],
                residual_map_is_diagnostic_not_new_fit=True))
            axes[rownum,0].imshow(images['color'][...,::-1]); axes[rownum,0].set_title('Pair %02d: original published RGB'%index)
            z=depth.astype(float)*1000;z[~valid]=np.nan
            im=axes[rownum,1].imshow(z,cmap='viridis',vmin=200,vmax=700)
            axes[rownum,1].set_title('Original depth, mm (holes white)');fig.colorbar(im,ax=axes[rownum,1],shrink=.65)
            im=axes[rownum,2].imshow(residual,cmap='coolwarm',vmin=-8,vmax=8)
            axes[rownum,2].set_title('Signed residual to accepted plane, mm');fig.colorbar(im,ax=axes[rownum,2],shrink=.65)
            for ax in axes[rownum]:
                for rad in (70,110,140,170): ax.add_patch(Circle((w/2,h/2),rad*scale,fill=False,lw=.6,color='lime'))
                ax.set_xlim(100,w-100);ax.set_ylim(h-35,35);ax.set_xticks([]);ax.set_yticks([])
        fig.suptitle('Same original gates; ring diagnostics only, no target or grasp acceptance')
        fig.savefig(O/'rgb_depth_residual_diagnostic.png',dpi=130);plt.close(fig)
    summary['interpretation']=[
        'All ten original support ring sequences qualify; this bounded later sample does not reproduce the earlier retired generation loss.',
        'Five outer-ring fits and one middle-ring fit fail the unchanged inlier-ratio gate; fallback rings retain the original threshold and recover these samples.',
        'All mapped support pixels belong to center. Only one of nine exact-associated image pairs applied center filtering; seven were warming and one failed visual motion.',
        'The full exact-source history counts describe filter eligibility, not proof that current missing history is caused by exposure, SDK loss, optical motion, or image noise.',
        'Depth holes and spatially varying residuals are observable. Sparse frames do not identify physical noise versus surface shape or establish filter causality.']
    raw=(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    (O/'interpretation.json').write_bytes(raw)
    (O/'diagnostic_SHA256SUMS').write_text(hashlib.sha256(raw).hexdigest()+'  interpretation.json\n'+hashlib.sha256((O/'rgb_depth_residual_diagnostic.png').read_bytes()).hexdigest()+'  rgb_depth_residual_diagnostic.png\n')
    print(json.dumps(dict(interpretation_sha256=hashlib.sha256(raw).hexdigest(),history=summary['history'],rings=summary['rings'])))


if __name__=='__main__': main()
