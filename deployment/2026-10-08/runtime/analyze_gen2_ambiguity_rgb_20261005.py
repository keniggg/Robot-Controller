#!/usr/bin/env python3
"""Read only the extracted immutable pairs; write an RGB/feedback diagnostic."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np

WORK=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WORK/'src/alicia_grasp_modes/src'),str(WORK/'src/alicia_flexible_grasp_supervisor/src')]
from alicia_grasp_modes.rgb_observation_handoff import _appearance,HandoffPolicy
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy,propose_rgb_observation_regions
from alicia_grasp_modes import tabletop
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
          '20261005T013130.144638Z_f95c4f0498e54f8a922171b982c2df69/first_ambiguity_analysis')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    r=json.loads((ROOT/'report.json').read_text());files={p['path']:p for p in r['files']}
    for name,record in files.items():assert sha(ROOT/name)==record['sha256']
    good=r['first_loss']['last_good_source_stamp_ns'];loss=r['first_loss']['source_stamp_ns']
    previous=np.load(ROOT/(str(good)+'.npz'));reference_mask=previous['published_mask']>0
    reference=_appearance(previous['rgb'],reference_mask.astype(np.uint8))
    x,y,w,h=cv2.boundingRect(reference_mask.astype(np.uint8))
    policy=CenterSearchPolicy(center_xy=((x+w/2)/reference_mask.shape[1],(y+h/2)/reference_mask.shape[0]),radius_fraction=.125)
    gates=HandoffPolicy();scale=r['camera_metadata']['depth_scale_m_per_unit']
    loss_data=np.load(ROOT/(str(loss)+'.npz'));second_region=loss_data['candidate_1_mask']>0
    plane=np.array(r['first_loss']['reference_support_plane_base']);feedback=r['accepted_feedback']
    q_all=np.array([f['positions'][:6] for f in feedback]);rows=[]
    for frame in r['source_frames']:
        ns=frame['source_stamp_ns'];z=np.load(ROOT/(str(ns)+'.npz'));rgb=z['rgb']
        valid=rectification_rgb_valid_mask({'runtime_profile':r['camera_metadata']},reference_mask.shape)
        proposals=propose_rgb_observation_regions(rgb,policy=policy,rgb_valid=valid);alternatives=[]
        for index,candidate in enumerate(proposals.candidates):
            mask=candidate.mask>0;hist,contour=_appearance(rgb,mask.astype(np.uint8))
            color=float(np.sqrt(max(0.,1-np.sum(np.sqrt(hist*reference[0])))))
            shape=float(cv2.matchShapes(reference[1],contour,cv2.CONTOURS_MATCH_I1,0))
            iou=float(np.count_nonzero(reference_mask&mask)/np.count_nonzero(reference_mask|mask))
            alternatives.append(dict(index=index,bbox=list(candidate.bbox_xywh),color_distance=color,
                contour_distance=shape,reference_mask_iou=iou,
                passes_original_hold_rgb_gates=bool(iou>=.85 and color<=gates.max_colour_distance and shape<=gates.max_contour_distance)))
        before=max((q for q in feedback if q['stamp_ns']<=ns),key=lambda q:q['stamp_ns'])
        after=min((q for q in feedback if q['stamp_ns']>=ns),key=lambda q:q['stamp_ns'])
        depth=z['depth_raw'].astype(np.float32)*scale
        points=tabletop._project(depth,z['K']);T=z['T_base_camera'];base=points@T[:3,:3].T+T[:3,3]
        heights=base@plane[:3]+plane[3];selected=second_region&(depth>0)
        row=dict(source_stamp_ns=ns,source_offset_from_last_good_seconds=(ns-good)*1e-9,
            observed_age_since_last_good_seconds=(frame['status']['frame_timing']['status_build_ros_ns']-good)*1e-9,
            old_online_state=frame['status']['detail'],original_metric_eligible_count=frame['eligible_count'],
            feedback=dict(before=before,after=after,span_ns=after['stamp_ns']-before['stamp_ns'],
                span_qualified=after['stamp_ns']-before['stamp_ns']<=200000000),
            independent_rgb_proposal_reason=proposals.reason,rgb_candidates=alternatives,
            independently_proposed_full_outline_hold_pass_count=sum(a['passes_original_hold_rgb_gates'] for a in alternatives),
            second_component_frozen_pixels=dict(pixels=int(second_region.sum()),valid_pixels=int(selected.sum()),
                above_4mm_pixels=int(np.count_nonzero(selected&(heights>=.004))),
                measured_height_mm_percentiles=np.percentile(heights[selected]*1000,[10,50,90]).tolist()),
            metric_refined_rgb_diagnostics=[])
        for i,c in enumerate(frame['candidates']):
            key='candidate_'+str(i)+'_refined'
            if key not in z:continue
            mask=z[key]>0;refined=c['refined']
            iou=float(np.count_nonzero(reference_mask&mask)/np.count_nonzero(reference_mask|mask))
            row['metric_refined_rgb_diagnostics'].append(dict(candidate_index=i,reference_mask_iou=iou,
                reference_color_distance=refined['previous_color_distance'],
                reference_contour_distance=refined['previous_contour_distance'],
                original_refinement_passed=True,
                all_three_original_rgb_gates_pass=bool(iou>=.85 and refined['previous_color_distance']<=gates.max_colour_distance
                    and refined['previous_contour_distance']<=gates.max_contour_distance)))
        rows.append(row)
    result=dict(schema=1,source_report_sha256=sha(ROOT/'report.json'),script_sha256=sha(__file__),
        policy=asdict(policy),gates=asdict(gates),full_outline_iou_gate=.85,
        accepted_feedback_joint_span_rad=(q_all.max(axis=0)-q_all.min(axis=0)).tolist(),rows=rows,
        findings=[
            'The extra eligible native component is on the patterned mat below the object in the exact RGB frame; it has zero intersection with the previous published RGB silhouette. RGB evidence does not show a separate solid object at that location.',
            'Both native candidates meet the existing 0.5 extent-ratio floor, so the shrinking-fragment hold intentionally defers to ordinary metric tracking.',
            'Existing independent RGB proposals do not meet the original 0.85 full-outline IoU. Reusing the current hold unchanged would not accept this episode.',
            'Refining all metric candidates, without taking a nearest winner, produces only one candidate with original full-outline, colour and contour compatibility. This is a diagnostic, not an implemented ambiguity policy.',
            'The next two recorded frames have one eligible native metric candidate and their original refinement has compatible RGB silhouette within the last-good 0.8 second window.',
            'A conservative future design can publish false/empty during a qualified short ambiguity, preserve the immutable original last-good deadline, then require current unique metric plus RGB/source/support/pose evidence before restoring measurement. No geometry merge, deadline renewal or lost-generation revival.',
        ],limitations=[
            'The actual private last-good identity cache and complete source-context digest are not recorded, so this report cannot prove a hypothetical live hold would have had every required input.',
            'All refined candidates were evaluated offline; current independent proposal failure must remain explicit. Adding refinement-derived identity evidence requires a separately reviewed generic contract and negative tests.',
            'Earlier/later frames are descriptive, not a claim of online target recovery; the actual generation remained irreversibly lost.',
        ])
    output=ROOT/'rgb_identity_and_feedback.json';output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(path=str(output),sha256=sha(output),joint_span=result['accepted_feedback_joint_span_rad'])))


if __name__=='__main__':main()
