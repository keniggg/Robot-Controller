"""Independent read-only checks of the bounded RGB direction prototype receipt."""
import hashlib,json,math
from pathlib import Path
import numpy as np
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
P=V/'gen27_rgb_direction_prototype_r1';O=V/'gen27_rgb_direction_independent_review'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not O.exists()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
r=json.loads((P/'receipt.json').read_text())
source_matches={p:sha(p)==expected for p,expected in r['source_and_input_sha256'].items()}
assert all(source_matches.values())
rows=[]
for item in r['experiments']:
    assert sha(item['path'])==item['sha256']
    d=json.loads(Path(item['path']).read_text());seed=np.asarray(d['candidates'][0]['axes_base']);lines={x['line_id']:x for x in d['actual_hough_lines']}
    maximum_rotation_error=0.;maximum_normal_error=0.;maximum_reconstruction_error=0.
    for c in d['candidates']:
        axes=np.asarray(c['axes_base'])
        maximum_rotation_error=max(maximum_rotation_error,float(np.max(abs(axes.T@axes-np.eye(3)))),abs(float(np.linalg.det(axes))-1))
        maximum_normal_error=max(maximum_normal_error,float(np.max(abs(axes[:,2]-seed[:,2]))))
        if c['kind']!='original_depth_axis':
            line=lines[c['hough_line_id']];assert line['orientation_disposition']=='evaluated_actual_direction'
            angle=line['delta_rad'];ax0=math.cos(angle)*seed[:,0]+math.sin(angle)*seed[:,1]
            expected=np.column_stack((ax0,np.cross(seed[:,2],ax0),seed[:,2]))
            maximum_reconstruction_error=max(maximum_reconstruction_error,float(np.max(abs(expected-axes))))
            assert math.isclose(math.degrees(angle),c['relative_angle_deg'],abs_tol=1e-12)
        if c['ok']:
            model=c['model'];center=np.asarray(model['center_base']);normal=np.asarray(model['support_normal_base']);size=np.asarray(model['size_xyz_m'])
            assert abs(center@normal+model['support_offset_m']-size[2]/2)<1e-7
            assert np.allclose(model['axes_base'],axes,rtol=0,atol=1e-12)
    assert maximum_rotation_error<1e-7 and maximum_normal_error<1e-12 and maximum_reconstruction_error<1e-12
    assert d['candidate_count']<=13
    rows.append(dict(name=item['name'],candidate_count=d['candidate_count'],qualified_rgb_candidate_count=d['qualified_rgb_candidate_count'],
        maximum_orthonormal_or_determinant_error=maximum_rotation_error,maximum_support_normal_change=maximum_normal_error,
        maximum_actual_line_axis_reconstruction_error=maximum_reconstruction_error,
        empty_edge_candidates=[dict(candidate_index=c['candidate_index'],relative_angle_deg=c['relative_angle_deg'],empty_edges=c['edge_search']['empty_edge_indices']) for c in d['candidates']],
        selected_candidate_index=d['selected_candidate_index']))
report=dict(classification='independent_static_and_numeric_offline_review_not_production_or_motion_validation',
    prototype_receipt=str(P/'receipt.json'),prototype_receipt_sha256=sha(P/'receipt.json'),all_pinned_inputs_and_sources_match=True,
    checked_source_count=len(source_matches),numeric_checks=rows,
    static_findings=[
      'Axes override is placed before recomputing top coordinates and bounds. The support normal, measured plane, raw pixels and geometry evidence are retained.',
      'Original visual rectangle, four-edge/corner, contour occupancy/hole, outside-plane, measured point enclosure, uncertainty and model validation checks are rerun without threshold replacement.',
      'Directions originate only from existing 5x5-dilated-mask Canny/Hough domain: longest 48 measured image segments, actual directions deduplicated at 0.5 degrees, at most 12 plus original depth direction. This is not an arbitrary angle grid.',
      'The same mask search restriction remains. Meaningful initial three-frame directions still lack lower_y edge candidates; changing axes alone cannot recover a missing candidate line.',
      'Candidate lines include printed content and side boundaries. They are hypotheses only; origin from a Hough segment does not independently establish a physical top edge.',
      'The cap prefers longest retained directions, not independently established physical boundary directions. This is a bounded candidate recall limitation, not a qualification bypass.',
      'Only later initial-pose raw source 1792050907639824151 qualifies. Actual-trigger three-frame reconstruction and nearfield raw source do not; prototype is not a validated solution for the original failed route.'
    ],visual_review=dict(
      sources=[1792050906773036718,1792050906972096443,1792050907106353521],
      image_directory=str(V/'gen27_attempt3_initial_inference_review'),
      visible_lower_front_edge='White front side against red background is visibly distinguishable across all three frames; no hand occlusion and little apparent image displacement.',
      recorded_hough_lower_front_segments=[[[320,269],[357,274]],[[282,266],[319,269]]],
      limitations=['Rear lower edge is occluded by the box; narrow right side has few pixels.',
                   'Whole object mask includes visible side face and irregular lower corner protrusions; its silhouette is not a precise top rectangle.',
                   'Image inspection and detected segments support testing an edge-based prism hypothesis, not metric contact authorization or all-four-edge observability.']),
    production_files_modified=False,no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True)
O.mkdir();p=O/'findings.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=sha(p),source_count=len(source_matches),numeric_checks=rows),ensure_ascii=False))
