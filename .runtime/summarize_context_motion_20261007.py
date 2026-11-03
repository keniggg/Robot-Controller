"""Save a bounded interpretation of the frozen offline context/model report."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/regional_production_20261007T011805.778547Z/context_motion_diagnostic_v1')
path=ROOT/'report.json'
raw=path.read_bytes()
report=json.loads(raw)
motion=[x for x in report['rows'] if x['category']=='visual_motion']
still=[x for x in report['rows'] if x['category']=='stationary']
summary=dict(schema=1,diagnostic_only=True,production_modified=False,motion_authorized=False,
    report_sha256=hashlib.sha256(raw).hexdigest(),
    figures_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.glob('*.png'))},
    original_roi_gray_both_images_exact=all(x['expanded_context']['original_roi_gray_both_images_bitwise_equal'] for x in report['rows']),
    original_pyramid_phase_preserved=all(x['expanded_context']['pyramid_phase_matches_through_level_2'] for x in report['rows']),
    context=dict(motion_examples=len(motion),motion_examples_now_below_original_threshold=sum(x['expanded_context']['original_roi_coverage_metrics']['motion_gate_passed'] for x in motion),
        stationary_examples=len(still),stationary_examples_still_below_threshold=sum(x['expanded_context']['original_roi_coverage_metrics']['motion_gate_passed'] for x in still),
        changed_cases=[dict(file=x['file'],region=x['region'],original_p90=x['original_lk']['motion_p90_px'],
            context_p90=x['expanded_context']['original_roi_coverage_metrics']['motion_p90_px'],
            original_valid_count=x['expanded_context']['original_valid_count'],
            all_original_valid_retained=x['expanded_context']['all_original_valid_retained'])
            for x in motion if x['expanded_context']['original_roi_coverage_metrics']['motion_gate_passed']]),
    cross_validation={},
    reviewed_visual_evidence='The two plotted current raw images contain a larger forearm/hand-shaped region at the left edge than their anchors, with visible changes to tabletop shading. This is qualitative image inspection, not a calibrated illumination or physical-motion measurement.',
    conclusions=[
        'Genuine outside-crop context changes two near-threshold child examples while retaining all 64 original valid tracks in each; it does not resolve 14/16 selected motion examples.',
        'Parent context uses 5/6 INTER_AREA on [8,0,638,480], producing 525x400 with original ROI offset[100,40]. Both original ROI grays are bitwise equal and offsets divisible by4 preserve LK pyramid phase. Child uses full native gray with no resizing.',
        'Coverage and endpoint bounds remain defined by the ORIGINAL ROI; newly valid tracks are not selectively removed to improve P90. For example snapshot18 original-valid-only P90 falls, but the additional valid track leaves the complete context result above threshold.',
        'Affine/homography fits use every original valid track and geometry-stratified held-out points. Only snapshot16 has holdout P90<=0.25 for either model; neither model supports treating the other recorded motion as coherent motion that can simply be cancelled.',
        'Least squares is sensitive to large valid-track errors. Snapshot27 contains an original-valid displacement about23.43px, snapshot29 about5.54px, and original-stationary snapshot06 about12.48px; these can strongly contaminate global fits although P90 excludes a small tail. Poor fits therefore do not prove absence of real camera motion or presence of sensor noise.',
        'Visible changing illumination/occlusion is an additional explanation, not a proven sole cause. Geometry and photometry remain mixed in the real data.',
        'No compensation, context-padding change, model-residual gate or production algorithm is enabled by these diagnostics. This is not support-plane, target-tracking or grasp validation.'
    ])
for model in ('translation','affine','homography'):
    values=[x['coherent_motion']['methods'][model]['cross_validated_residual_full_px']['p90'] for x in motion]
    summary['cross_validation'][model]=dict(motion_examples=len(values),p90_residual_median=float(np.median(values)),
        p90_residual_minimum=min(values),p90_residual_maximum=max(values),examples_at_or_below_025=sum(v<=.25 for v in values))
assert Path('/media/zhuyupei/SanStand').is_mount() and path.read_bytes()==raw
out=ROOT/'interpretation.json'
with out.open('x') as f:
    json.dump(summary,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest())))
