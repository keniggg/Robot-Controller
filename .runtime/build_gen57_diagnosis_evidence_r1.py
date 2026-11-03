"""Read immutable finite probes into a compact diagnostic recovery index."""
from pathlib import Path
import hashlib
import json
import os
import numpy as np

W=Path(__file__).resolve().parents[1]
O=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
D=O/'missing_depth_prior_fix_r1'
TARGETS=(1792548361880352258,1792548362414543151)

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def digest_array(a): return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def ref(p): return dict(path=str(p),sha256=sha(p))
def read(p): return json.loads(Path(p).read_text())
def appearance(a):
    return {k:a.get(k) for k in ('code','appearance_match','colour_distance','contour_distance',
        'chroma_distance','max_colour_distance','max_contour_distance','reference_mask_sha256',
        'current_mask_sha256','fallback_attempted','reference_refreshed')}

PROBES=[
 ('mask_identity_replay_r2',None,'No intervention; unchanged pure geometry and original appearance observer.'),
 ('seed_label_diagnosis_r1','inspect_gen57_seed_labels_20261021.py','Read-only GrabCut input/output label observer.'),
 ('seed_geometry_diagnosis_r1',None,'Read-only confidence-label geometry observer.'),
 ('native_seed_counterfactual_r1','probe_gen57_native_seed_20261021.py','Retain full native measured depth for the same intersecting members instead of clipping each member to the frozen footprint.'),
 ('upper_band_counterfactual_r1','probe_gen57_upper_band_20261020.py','Apply the existing reliable near-horizontal +/-0.5 mm upper band at every seed size; retain old upper seeds.'),
 ('reliable_interior_counterfactual_r1','probe_gen57_reliable_interior_20261020.py','Diagnostic hard-foreground expansion to all reliable near-horizontal measured interior; no height band.'),
 ('lab_features_counterfactual_r1','probe_gen57_lab_mask_20261020.py','Replace only arrival GrabCut input BGR features by CIELAB; keep original labels and BGR appearance evaluation.'),
 ('unknown_depth_prior_counterfactual_r1','probe_gen57_unknown_depth_prior_20261020.py','Broad diagnostic: original probable background with nonfinite or out-of-range depth becomes probable foreground immediately before GrabCut; hard labels remain untouched.'),
]

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    baseline=read(O/'mask_identity_replay_r2/result.json')
    base_rows={r['source_stamp_ns']:r for r in baseline['rows']}
    table=[]
    for name,helper,change in PROBES:
        result=read(O/name/'result.json')
        entry=dict(name=name,result=ref(O/name/'result.json'),only_intended_change=change,
            config_equal_to_baseline=result['config']==baseline['config'],
            input_sha256_map_equal_to_baseline=result['input_sha256']==baseline['input_sha256'],
            declared_scope=result['scope'],rows=[])
        if helper: entry['helper']=ref(W/'.runtime'/helper)
        if 'counterfactual_function_source' in result:
            entry['counterfactual_function_source_sha256']=hashlib.sha256(result['counterfactual_function_source'].encode()).hexdigest()
        for row in result['rows']:
            stamp=row['source_stamp_ns'];cap=row['captured'][0];ap=cap['original_appearance']
            base=base_rows[stamp]['captured'][0]
            saved=O/name/f'{stamp}_masks.npz'
            with np.load(saved,allow_pickle=False) as f, np.load(O/'mask_identity_replay_r2'/f'{stamp}_masks.npz',allow_pickle=False) as g:
                same={k:bool(np.all((f[k]==g[k]) | (np.isnan(f[k])&np.isnan(g[k])))) for k in
                    ('rgb','depth','K','T_base_camera','spatial_domain','spatial_clipped_proposal','trusted_core')}
                core=dict(pixels=int(np.count_nonzero(f['trusted_core'])),sha256=digest_array(f['trusted_core']),
                    byte_equal_to_baseline=same['trusted_core'])
            entry['rows'].append(dict(source_stamp_ns=stamp,appearance=appearance(ap),
                downstream_reason=row['replayed_reason'],hard_core=core,arrays_unchanged_vs_baseline=same,
                masks_artifact=ref(saved),reference_unchanged_vs_baseline=ap['reference_mask_sha256']==base['original_appearance']['reference_mask_sha256'],
                threshold_equal_to_baseline=all(ap[k]==base['original_appearance'][k] for k in ('max_colour_distance','max_contour_distance'))))
        entry['original_appearance_passed_on_both_recorded_rejections']=all(r['appearance']['appearance_match'] for r in entry['rows'])
        entry['sufficient_for_original_two_frame_appearance_rejection']=name=='unknown_depth_prior_counterfactual_r1'
        entry['not_runtime_qualification']=True
        table.append(entry)

    chains=[]
    narrow_matches=[]
    for name in ('arrival_chain_candidate_r1','arrival_chain_candidate_reconstructed_r1'):
        folder=O/name;result=read(folder/'result.json');manifest=read(folder/'source_manifest.json')
        entry=dict(name=name,result=ref(folder/'result.json'),source_manifest=ref(folder/'source_manifest.json'),
            input_manifest=ref(folder/'input_manifest.json'),rows=[],
            reference_summary={k:result['reference'].get(k) for k in ('reference_mode',
                'reference_mask_frozen_from_original_recorded_bytes','reconstructed_reference_is_counterfactual',
                'current_source_recomputed_reference_bit_equal','recorded_mask','normalized_mask')},
            three_frame_geometry_confirmed=result['three_frame_geometry_confirmed'],
            full_runtime_handoff_verified=result['full_runtime_handoff_verified'],
            runtime_qualification_verified=result['runtime_qualification_verified'])
        module=read(folder/'module_provenance.json')
        entry['rgb_instance_module']=module['alicia_grasp_modes.rgb_instance']
        entry['measured_mask_seeds_module']=module['alicia_grasp_modes.measured_mask_seeds']
        for row in result['rows']:
            ap=row['appearance_identity'][0]
            entry['rows'].append(dict(source_stamp_ns=row['source_stamp_ns'],appearance=appearance(ap),
                reason=row['evidence']['reason'],confirmed=row['confirmed'],
                confirmation_source_stamps_ns=row['confirmation_source_stamps_ns']))
            if name=='arrival_chain_candidate_r1' and row['source_stamp_ns'] in TARGETS:
                broad=next(r for r in table[-1]['rows'] if r['source_stamp_ns']==row['source_stamp_ns'])['appearance']
                narrow_matches.append(dict(source_stamp_ns=row['source_stamp_ns'],
                    mask_sha_equal=ap['current_mask_sha256']==broad['current_mask_sha256'],
                    shape_equal=ap['contour_distance']==broad['contour_distance'],
                    colour_equal=ap['colour_distance']==broad['colour_distance']))
        s=result['station']
        entry['station']=dict(source_stamp_ns=s['source_stamp_ns'],appearance=[appearance(a) for a in s['appearance_identity']],
            pure_geometry_passed=s['pure_geometry_passed'],runtime_qualified=s['runtime_qualified'],
            ack_handoff_executed=s['ack_handoff_executed'],runtime_marker_created=s['runtime_marker_created'],
            template_is_offline_counterfactual=s['template_is_offline_counterfactual'],premise=s['premise'])
        chains.append(entry)

    recovery=[]
    observer=read(O/'seed_label_diagnosis_r1/result.json')
    for stamp,call_index in zip(TARGETS,(2,3)):
        call=observer['grabcut_calls'][call_index];x0,y0,x1,y1=call['box_xyxy']
        old_path=O/'mask_identity_replay_r2'/f'{stamp}_masks.npz'
        new_path=O/'unknown_depth_prior_counterfactual_r1'/f'{stamp}_masks.npz'
        with np.load(old_path) as old,np.load(new_path) as new,np.load(call['artifact']) as labels:
            recovered=((new['refined_mask']>0)&~(old['refined_mask']>0))[y0:y1,x0:x1]
            zero=labels['depth']==0
            prbg=labels['labels_before']==2
            fgafter=np.isin(labels['labels_after'],(1,3))
            recovery.append(dict(source_stamp_ns=stamp,label_artifact=ref(call['artifact']),
                baseline_final_attempt_box_xyxy=call['box_xyxy'],newly_included_pixels=int(recovered.sum()),
                recovered_zero_depth_pixels=int(np.count_nonzero(recovered&zero)),
                recovered_zero_depth_previously_probable_background=int(np.count_nonzero(recovered&zero&prbg)),
                recovered_zero_depth_previous_grabcut_background=int(np.count_nonzero(recovered&zero&~fgafter)),
                interpretation='Output-difference diagnostic, not a manually labelled ground-truth contour.'))

    seed_path=W/'src/alicia_grasp_modes/src/alicia_grasp_modes/measured_mask_seeds.py'
    seed_old=next(v for k,v in baseline['source_sha256'].items() if k.endswith('/measured_mask_seeds.py'))
    out=dict(schema=1,scope='Compact independent causal evidence index; no final runtime acceptance claim',
        problem='Two exact arrival frames fail original contour identity because part of the RGB object is segmented as background.',
        exact_recorded_sources=list(TARGETS),baseline_harness=ref(O/'replay_masks_r2.py'),
        frozen_reference_sha256='6c93eef7e7277329810d798d30b410c08ea0dd92a6e7fedff406afc02fb61b24',
        probes=table,production_candidate_chains=chains,recovered_pixel_evidence=recovery,
        hard_seed_contract=dict(current_measured_mask_seeds=ref(seed_path),baseline_sha256=seed_old,
            source_unchanged=(sha(seed_path)==seed_old),original_two_frame_hard_core_unchanged_in_sufficient_probe=all(
                r['hard_core']['byte_equal_to_baseline'] for r in table[-1]['rows']),
            production_source_review=ref(D/'review_r1.json')),
        broad_diagnostic_to_narrow_production=dict(
            diagnostic_rule='All nonfinite or configured out-of-range depth: PR_BGD -> PR_FGD only.',
            production_rule='geometry exists; NaN or exactly zero depth: original PR_BGD -> PR_FGD, then original hard core/support/border overwrite.',
            production_keeps_negative_infinite_and_positive_out_of_range_priors_unchanged=True,
            production_on_original_frozen_reference_matches_broad_diagnostic=narrow_matches),
        conclusion={'supported':'Changing the missing-depth probable initialization is sufficient for the two recorded appearance rejections while original hard seeds, spatial proposal, BGR input and appearance thresholds remain fixed.',
            'p90_status':'Spatially biased upper-tail hard seeds are a measured observation, but neither upper-band nor full reliable-interior interventions alone repairs the two rejected contours; changing P90 is not needed by the sufficient production fix.',
            'not_supported':'No claim that changed prior alone solves arbitrary ambiguous scenes, no runtime qualified ACK, no complete three-owner handoff, no real grasp or motion success.'},
        unresolved_validation=['Full regression outcome and current three-owner handoff evidence are owned by the ongoing parent task.',
            'Four-source candidate artifacts prove their stated pure computation only; clocks, capture deadlines and runtime ACK are not replayed.',
            'Reconstructed-reference chain is explicitly counterfactual; it does not replace the historical frozen reference.'],
        seed_surface_exploration=ref(O/'seed_surface_exploration_agent_r1/result.json'),
        writer=ref(__file__))
    assert out['hard_seed_contract']['source_unchanged']
    assert all(all(x[k] for k in ('mask_sha_equal','shape_equal','colour_equal')) for x in narrow_matches)
    p=D/'diagnosis_evidence_r1.json'
    with p.open('x') as f:json.dump(out,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    print(json.dumps(dict(output=ref(p),probes=len(table),chains=len(chains),recovered_pixel_evidence=recovery,
        sufficient_narrow_matches=narrow_matches),ensure_ascii=False))

if __name__=='__main__': main()
