"""Inspect unchanged MSER duplicate families for the three residual SDK RGB failures."""
from collections import OrderedDict
import json
from types import SimpleNamespace

import numpy as np

import compare_gen13_rgb_sdk_projection_20261004 as comparison
from alicia_grasp_modes import rgb_observation_region as regions
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy
from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy


def main():
    b = comparison.baseline
    report = json.loads(comparison.OUT.read_text())
    evidence = json.loads(b.ARCHIVE.read_text())
    raw_index = {r['source_stamp_ns']: r for r in evidence['raw_frames']}
    reference, _, policy = b.original_reference()
    original_same_region = regions._same_region
    records = []
    for row in report['records']:
        if row['baseline_reason'] != 'no_cross_view_rgb_match' or row['sdk_reason'] != 'no_cross_view_rgb_match':
            continue
        raw = np.load(raw_index[row['source_stamp_ns']]['file'])
        projected = np.asarray(row['projected_volume_pixels'])
        search = projected_rgb_search_policy(policy, projected, raw['depth_raw'].shape)
        captured = OrderedDict()

        def capture(a, c):
            for candidate in (a, c):
                key = (candidate.bbox_xywh, candidate.mask.tobytes())
                captured[key] = candidate
            return original_same_region(a, c)

        try:
            regions._same_region = capture
            proposed = regions.propose_rgb_observation_regions(raw['color_bgr'], policy=search)
        finally:
            regions._same_region = original_same_region
        output_candidates = comparison.candidate_audit(reference, raw['color_bgr'], proposed,
            projected, HandoffPolicy())
        selected = [c for c in output_candidates if c['appearance_passes']]
        assert len(selected) == 1
        representative = proposed.candidates[selected[0]['index']]
        variants = []
        for compact in captured.values():
            if not original_same_region(compact, representative):
                continue
            mask = np.zeros(raw['depth_raw'].shape, bool)
            x, y, w, h = compact.bbox_xywh
            mask[y:y+h, x:x+w] = compact.mask
            candidate = SimpleNamespace(mask=mask, bbox_xywh=compact.bbox_xywh)
            detail = comparison.candidate_audit(reference, raw['color_bgr'],
                SimpleNamespace(candidates=[candidate]), projected, HandoffPolicy())[0]
            detail.update(center_score=compact.score, center_distance_fraction=compact.center_distance_fraction,
                channels=list(compact.channels), chosen=detail['mask_sha256'] == selected[0]['mask_sha256'])
            variants.append(detail)
        variants.sort(key=lambda x: x['center_score'])
        records.append(dict(source_stamp_ns=row['source_stamp_ns'], selected_candidate=selected[0],
            unchanged_returned_masks=[c['mask_sha256'] for c in output_candidates],
            duplicate_family_variants=variants))
    result = dict(scope=__doc__, comparison_report=str(comparison.OUT), comparison_sha256=b.sha(comparison.OUT),
        script_sha256=b.sha(__file__), module_sha256=b.sha(regions.__file__), records=records,
        unchanged_production=True, authority='diagnostic only; no alternative mask was authorized or substituted',
        limit='Nested MSER outlines are one location family, not independently proven object boundaries; '
            'choosing only an outline that passes volume overlap would require a separate generic contract and negative tests.')
    output = comparison.OUT.parent/'rgb_remaining_duplicate_family_audit.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output), sha256=b.sha(output), rows=[dict(source=r['source_stamp_ns'],
        variants=len(r['duplicate_family_variants']), chosen=r['selected_candidate'],
        otherwise_passing_variants=[v for v in r['duplicate_family_variants'] if v['appearance_passes'] and v['projection_passes']])
        for r in records]), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
