"""Fixed original observation-domain sensitivity; no production/ROS writes."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from alicia_grasp_modes import tabletop
from analyze_gen13_exact_support_20261004 import disagreement


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def coverage(vertices, observed, axes, transform, intrinsics, shape):
    hull = cv2.convexHull((observed@axes.T).astype(np.float32))
    inside = [cv2.pointPolygonTest(hull,tuple(map(float,xy)),False) >= 0
        for xy in vertices@axes.T]
    nearest = [float(np.min(np.linalg.norm(observed-v,axis=1))*1000) for v in vertices]
    pixels = np.asarray([tabletop.project_base_to_pixel(v,transform,intrinsics) for v in vertices])
    in_image = (pixels[:,0] >= 0)&(pixels[:,0] < shape[1])&(pixels[:,1] >= 0)&(pixels[:,1] < shape[0])
    radius = np.linalg.norm(pixels-[shape[1]/2,shape[0]/2],axis=1)
    return dict(point_count=len(vertices),within_current_measured_support_convex_hull=inside,
        within_current_measured_support_convex_hull_count=sum(inside),
        nearest_current_actual_support_inlier_mm=nearest,
        nearest_current_actual_support_inlier_mm_range=[min(nearest),max(nearest)],
        projection_pixels=pixels.tolist(),within_current_image=in_image.tolist(),
        within_current_image_count=int(np.count_nonzero(in_image)),
        projected_radius_px=radius.tolist(),
        coverage_meaning='Convex-hull containment is a spatial audit only, not a measured surface or filled depth hole.')


def main(support_report, output):
    report=json.loads(support_report.read_text())
    reference_path=Path(report['provenance']['reference'])
    reference=json.loads(reference_path.read_text())
    volumes=json.loads(reference['outcome']['episode']['reference_volumes_json'])
    original=np.asarray(report['reference']['plane_base'])
    ring_footprint=np.asarray(report['reference']['footprint_base_m'])
    axes=np.asarray(tabletop._plane_axes(original[:3]))
    members=[]
    for i,volume in enumerate(volumes):
        vertices=np.asarray(volume['vertices_base_m'])
        projected=vertices-(vertices@original[:3]+original[3])[:,None]*original[:3]
        assert vertices.shape==projected.shape==(8,3)
        assert np.max(abs(projected@original[:3]+original[3]))<1e-12
        members.append(dict(member_index=i,original_vertices_base_m=vertices.tolist(),
            original_support_plane_projected_all_eight_vertices_base_m=projected.tolist(),
            footprint_span_plane_mm=(np.ptp(projected@axes.T,axis=0)*1000).tolist(),
            semantics='Frozen observation uncertainty domain; constructed geometry is not support measurement.'))
    result=dict(scope='fixed_original_all_member_observation_volume_support_domain_comparison',
        production_changes=False,robot_commands_sent=False,motion_authorized=False,
        original_threshold_m=.004,original_angle_threshold_deg=4.,
        original_reference_id=reference['outcome']['episode']['reference_id'],
        provenance=dict(support_report=str(support_report),support_report_sha256=sha(support_report),
            reference=str(reference_path),reference_sha256=sha(reference_path),script_sha256=sha(__file__)),
        original_measured_support_ring=report['reference'],fixed_uncertainty_members=members,
        current_frame_used_to_shrink_domain=False,member_union_or_center_substitution=False,rows=[])
    for row in report['rows']:
        data=np.load(row['source_file'],allow_pickle=False)
        depth=data['depth_raw'].astype(np.float32)*float(data['depth_scale'])
        K,T=data['K'],data['T_base_camera'];points=tabletop._project(depth,K)
        vv,uu=np.indices(depth.shape);radius=np.hypot(uu-depth.shape[1]/2,vv-depth.shape[0]/2)
        valid=np.isfinite(depth)&(depth>=report['config']['depth_min_m'])&(depth<=report['config']['depth_max_m'])
        camera=np.asarray(row['production_plane_camera']);base=np.asarray(row['production_plane_base'])
        distances=np.einsum('ijk,k->ij',points,camera[:3])+camera[3]
        inliers=valid&(radius>=70)&(radius<=row['support_metrics']['support_ring_outer_radius_px'])&(abs(distances)<=.002)
        observed=points[inliers]@T[:3,:3].T+T[:3,3]
        cases=[]
        for member in members:
            domain=np.asarray(member['original_support_plane_projected_all_eight_vertices_base_m'])
            cases.append(dict(member_index=member['member_index'],
                support_disagreement=disagreement(original,base,domain),
                current_measured_support_coverage=coverage(domain,observed,axes,T,K,depth.shape)))
        result['rows'].append(dict(source_stamp_ns=row['source_stamp_ns'],recorded_reason=row['recorded_reason'],
            observation_index=row['observation_index'],current_actual_support_inlier_pixels=len(observed),
            original_ring_disagreement=row['production_disagreement'],
            original_ring_coverage=coverage(ring_footprint,observed,axes,T,K,depth.shape),
            fixed_uncertainty_members=cases,
            all_members_pass_same_4mm=all(c['support_disagreement']['passes_original_4mm'] for c in cases)))
    failures=[r for r in result['rows'] if r['recorded_reason'].startswith('support_inconsistent')]
    controls=[r for r in result['rows'] if not r['recorded_reason'].startswith('support_inconsistent')]
    result['summary']=dict(original_rejected_count=len(failures),
        original_rejected_with_uncertainty_domain_pass=sum(r['all_members_pass_same_4mm'] for r in failures),
        controls_count=len(controls),controls_with_uncertainty_domain_pass=sum(r['all_members_pass_same_4mm'] for r in controls),
        original_rejected_uncertainty_max_distance_mm=[max(m['support_disagreement']['maximum_distance_mm']
            for m in r['fixed_uncertainty_members']) for r in failures])
    result['limits']=[
        'This is a fixed-domain counterfactual audit, not a new production handoff or execution permission.',
        'Actual support must still pass unchanged native-depth sample, residual, ratio, quadrant and original 4 degree normal gates.',
        'An observation uncertainty footprint is derived from original all-eight volume vertices, never current RGB center or a shrunken current mask.',
        'The existing production reference intentionally stores measured support footprint; replacing that measurement with constructed vertices would violate its current contract.',
        'Any future separate consistency-domain field must retain original measured support evidence and all members, and must not alter whole-path current/retained scene collision proofs.',
        'Passing the counterfactual local domain does not resolve native target depth deficiency or independently verify physical calibration.',
    ]
    output.mkdir(parents=True,exist_ok=False)
    p=output/'report.json';p.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(p),sha256=sha(p),summary=result['summary'],members=members),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--support-report',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    main(args.support_report,args.output)
