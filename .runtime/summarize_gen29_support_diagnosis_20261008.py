#!/usr/bin/env python3
"""Summarize sealed Gen29 measurement evidence; never admits a frame."""
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = ARCHIVE/'ros_log/supplement_grasp_20261008T103117Z'
OUT = SESSION/'gen29_support_diagnosis'
INPUT = OUT/'baseline_ring_inputs'
STAMPS = (1791457418117137432, 1791457418370887756,
          1791457418581797122, 1791457418721812009)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    destination = OUT/'diagnosis.json'
    assert not destination.exists()
    meta = json.loads((INPUT/'meta.json').read_text())
    rows = {row['source_stamp_ns']: row for row in meta['rows']}
    arrays = {ns: np.load(INPUT/(str(ns)+'.npz')) for ns in STAMPS}
    reference = arrays[STAMPS[0]]
    quantiles = [0, 5, 10, 25, 50, 75, 90, 95, 100]
    frames = []
    for ns in STAMPS:
        data = arrays[ns]
        uv = data['r110_uv']
        mask = np.zeros(data['raw_depth'].shape, dtype=bool)
        mask[uv[:, 1], uv[:, 0]] = True
        mask &= reference['raw_depth'] > 0
        delta_mm = ((data['raw_depth'].astype(float)-reference['raw_depth'])
                    * meta['camera']['depth_scale'] * 1000.)[mask]
        frame = dict(source_stamp_ns=ns, detail=rows[ns]['detail'],
                     rings=rows[ns]['rings'],
                     same_pixel_depth_change_vs_last_good=dict(
                         domain='intersection of current valid r110 pixels and last-good nonzero depth',
                         pixel_count=int(mask.sum()), standard_deviation_mm=float(delta_mm.std()),
                         quantiles=quantiles,
                         quantile_values_mm=np.percentile(delta_mm, quantiles).tolist()),
                     transform_max_absolute_difference_vs_last_good=float(np.max(abs(
                         data['transform_base_camera']-reference['transform_base_camera']))))
        counterfactual = OUT/('fit_counterfactual_'+str(ns)+'.json')
        if counterfactual.exists():
            frame['same_sample_stronger_fit_evidence'] = dict(path=str(counterfactual),
                                                             sha256=sha(counterfactual))
        frames.append(frame)
    camera = []
    with rosbag.Bag(meta['source_bag']) as bag:
        for _, message, bag_stamp in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']):
            if not 1791457406000000000 <= bag_stamp.to_nsec() <= 1791457419500000000:
                continue
            record = json.loads(message.data)
            stationary = record.get('parameters', {}).get('runtime_profile', {}).get('stationary_temporal', {})
            camera.append(dict(bag_stamp_ns=bag_stamp.to_nsec(),
                image_time_association=record.get('image_time_association'),
                parameter_read_interval=record.get('parameter_read_interval'),
                filtered_fraction=stationary.get('filtered_fraction'),
                alpha=stationary.get('alpha'),
                persistence=stationary.get('persistence'),
                regions=[dict(name=region['name'], applied=region['filter_applied'],
                              visual=region.get('visual', {}))
                         for region in stationary.get('regions', [])]))
    report = dict(
        scope='offline measurement diagnosis, no production changes or execution authorization',
        input_meta=dict(path=str(INPUT/'meta.json'), sha256=sha(INPUT/'meta.json')),
        script_sha256=sha(__file__), frames=frames, camera_parameter_snapshots=camera,
        conclusions=[
            'The two final pending frames fail original 0.70 inlier ratio at every original ring; the same models also fail on complete unsampled point sets.',
            'Independent stronger RANSAC on the identical original samples did not find a model passing 0.70 at 2 mm; finite search is not a proof of global impossibility.',
            'The final current frame recovers support with r170 ratio 0.7116666667. Its terminal failure occurs after recovery processing when the previous accepted reference exceeds 0.8 seconds.',
            'Published depths change by millimetres at the same pixels with effectively identical recorded camera pose. Point count and quadrant coverage are ample; static surface shape alone cannot explain the temporal change.',
            'Camera parameter snapshots show repeated regional depth filtering transitions between 0 and 0.61 filtered fraction, with center warmup and visual-motion resets. This is a possible contributor, not a frame-bound causal proof.',
        ],
        limitations=[
            'depth/image_raw contains the published final depth image, potentially including SDK processing, regional temporal filtering and calibrated projection; raw_depth denotes its unchanged uint16 message array, not pre-filter SDK depth.',
            'Camera runtime snapshots explicitly declare image_time_association=none and occur roughly once per second. They cannot establish the exact filter state of any selected depth frame.',
            'The recording does not provide exact full-rate pre-rectification RGB/depth and per-frame regional-history evidence needed to reproduce depth-filter authorization.',
            'No evidence currently separates physical scene change, sensor measurement noise and processing-history transitions into causal contributions.',
        ],
        recommendation=dict(
            retain_original_gates=True,
            rejected_shortcuts=['raise inlier-distance threshold', 'lower inlier ratio',
                                'extend reference 0.8-second lifetime',
                                'filter current support by a previous plane',
                                'increase RANSAC work without evidence of a missed valid model'],
            next_bounded_diagnostic='If further source diagnosis is needed, capture source-bound regional filter evidence with the existing gates unchanged, and compare pre-filter and final current depth on exactly the same support ring.',
            production_patch_supported_by_this_evidence=False))
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(path=str(destination), sha256=sha(destination), frames=len(frames),
                         camera_parameter_snapshots=len(camera))))


if __name__ == '__main__':
    main()
