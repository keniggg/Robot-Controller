#!/usr/bin/env python3
"""Append a model-layer clarification; preserve the sealed review unchanged."""
import hashlib
import json
from pathlib import Path

V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT = V / 'gen54_command_semantics_independent_review_r1'


def ref(p):
    b = p.read_bytes()
    return dict(path=str(p), sha256=hashlib.sha256(b).hexdigest(), bytes=len(b))


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    original = ref(OUT / 'result.json')
    assert original['sha256'] == '3844e284f7d711dce743f89ea97f0b3e09b21a7eefd0f39acb7eb4c36c9040d6'
    path = V / 'gen54_collision_geometry_r1/actual_api_residual_CAD_depth.json'
    evidence = ref(path)
    assert evidence['sha256'] == '141d82bac7786f2d4f1955724fff7975699100baf211c69bdc53130c7de40358'
    data = json.loads(path.read_text())
    source = ref(Path(data['source_path']))
    assert source['sha256'] == data['source_sha256'] == '10573477073c9c3a83660c36d0d508eb8710ce7417d45a7638818f56524870d3'
    rows = []
    for row in data['rows']:
        for group in row['probes']:
            failures = group['failures']
            rows.append(dict(source_index=row['source_index'], variant_index=row['variant_index'],
                angle_deg=row['angle_deg'], counts_each_axis=group['counts_each_axis'],
                guard_failed_corners=group['failed_corners'],
                any_CAD_AABB_overlap=group['any_CAD_AABB_overlap'],
                any_exact_mesh_surface_target_intersection=group['any_exact_mesh_surface_target_intersection'],
                any_observed_point_inside_exact_CAD_AABB=group['any_observed_point_inside_exact_CAD_AABB'],
                padded_sat_overlap_range_m=([min(f['padded_sat']['overlap_m'] for f in failures),
                                            max(f['padded_sat']['overlap_m'] for f in failures)] if failures else None),
                exact_CAD_AABB_sat_overlap_range_m=([min(f['exact_CAD_AABB_sat']['overlap_m'] for f in failures),
                                                   max(f['exact_CAD_AABB_sat']['overlap_m'] for f in failures)] if failures else None)))
    result = dict(kind='GEN54_command_semantics_model_layer_clarification',
        original_review=original, additional_evidence=evidence, source_evidence=source,
        rows=rows,
        clarifications=[
            'In the original review, current-height residual collision means rejection by the unchanged analytical padded collision guard. It does not assert real solid-mesh intersection or executed physical contact.',
            'For both 67.5-degree API height variants, the seven one-count failed corners overlap only the padded guard; CAD AABB has a separating axis and contains the mesh, so those sampled mesh states are disjoint from the target OBB.',
            'For the 68-degree variants, some one-count corners overlap the CAD AABB. An AABB overlap alone does not prove solid mesh intersection; no sampled mesh-surface triangle/target intersection or observed point inside the CAD AABB was reported.',
            'Finite clear states or absent observed points do not certify continuous uncertainty interiors, unobserved target occupancy, future feedback or physical safety.',
            'The exact counterexamples still invalidate a claim that dual-reference pass covers the original fixed padded guard over the full bias/residual set. They do not authorize shrinking padding, removing a curve, or treating this review as deployment approval.',
        ],
        original_review_modified=False, new_geometry_or_physical_test_run=False,
        source_or_helpers_modified=False, ROS_or_hardware_accessed=False,
        temperature_queried_or_values_diagnosed=False, helper=ref(Path(__file__)))
    target = OUT / 'model_layer_clarification.json'
    with target.open('x') as f:
        json.dump(result, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write('\n')
    print(json.dumps(ref(target), ensure_ascii=False))


if __name__ == '__main__':
    main()
