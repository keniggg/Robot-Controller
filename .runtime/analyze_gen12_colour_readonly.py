#!/usr/bin/env python3
"""Compare sealed same-source masks without changing RGB or production policy."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen12_arrival_analysis')
OLD = BASE / 'crop_counterfactual_48'
NEW = BASE / 'crop48_current_measured_probable_fg'
OUT = BASE / 'colour_diagnostics'
RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T042814.711798Z_02d7a2117e114e0ea476744c19fa64c4')


def hist(rgb, mask):
    x, y, w, h = cv2.boundingRect(mask.astype(np.uint8))
    lab = cv2.cvtColor(np.ascontiguousarray(rgb[y:y+h, x:x+w]), cv2.COLOR_BGR2LAB)
    value = cv2.calcHist([lab], [0, 1, 2], mask[y:y+h, x:x+w].astype(np.uint8),
                         [8, 8, 8], [0, 256] * 3).astype(float)
    for axis in range(3):
        padding = [(0, 0)] * 3
        padding[axis] = (1, 1)
        expanded = np.pad(value, padding, mode='edge')
        parts = []
        for offset in range(3):
            slices = [slice(None)] * 3
            slices[axis] = slice(offset, offset + 8)
            parts.append(expanded[tuple(slices)])
        value = .125 * parts[0] + .75 * parts[1] + .125 * parts[2]
    return value / value.sum()


def hellinger(a, b):
    return float(np.sqrt(max(0., 1. - np.sqrt(a * b).sum())))


def distances(reference, value):
    return dict(joint_Lab=hellinger(reference, value),
                marginal_L=hellinger(reference.sum((1, 2)), value.sum((1, 2))),
                marginal_ab=hellinger(reference.sum(0), value.sum(0)),
                marginal_a=hellinger(reference.sum((0, 2)), value.sum((0, 2))),
                marginal_b=hellinger(reference.sum((0, 1)), value.sum((0, 1))))


def stats(rgb, mask):
    selected = mask > 0
    if not selected.any():
        return dict(pixels=0)
    lab = cv2.cvtColor(rgb, cv2.COLOR_BGR2LAB)[selected]
    bgr = rgb[selected]
    result = dict(pixels=int(selected.sum()))
    for name, values in [('Lab_u8', lab), ('BGR_u8', bgr)]:
        result[name] = dict(mean=values.mean(0).tolist(), std=values.std(0).tolist(),
                           percentiles=np.percentile(values, [5, 25, 50, 75, 95], axis=0).tolist())
    # Descriptive pixel composition only; these thresholds never gate identity.
    result['descriptive_neutral_bright_fraction'] = float(np.mean(
        (lab[:, 0] >= 180) & (np.abs(lab[:, 1].astype(float)-128) <= 12)
        & (np.abs(lab[:, 2].astype(float)-128) <= 12)))
    result['descriptive_dark_fraction'] = float(np.mean(lab[:, 0] < 100))
    return result


def camera_metadata():
    rows = []
    keys = set()
    topic = '/grasp_record/camera_runtime_metadata'
    def selected_fields(value, path=''):
        found = {}
        if isinstance(value, dict):
            for key, item in value.items():
                lower = key.lower()
                if 'temperature' in lower:
                    continue
                current = path + '/' + key
                if any(term in lower for term in ('exposure', 'gain', 'white_balance')):
                    found[current] = item
                elif isinstance(item, dict):
                    found.update(selected_fields(item, current))
        return found
    with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
        for _, message, stamp in bag.read_messages(topics=[topic]):
            data = json.loads(message.data)
            row = {key: data.get(key) for key in ('status', 'availability_scope',
                   'image_time_association', 'time_semantics', 'unavailable_reason')}
            row['stamp_ns'] = stamp.to_nsec()
            row['exposure_gain_white_balance_fields'] = selected_fields(data.get('parameters', {}))
            keys.update(row['exposure_gain_white_balance_fields'])
            if not rows or {k:v for k,v in row.items() if k != 'stamp_ns'} != {
                    k:v for k,v in rows[-1].items() if k != 'stamp_ns'}:
                rows.append(row)
    return dict(topic=topic, distinct_consecutive_states=rows, field_paths=sorted(keys),
                scope='Only recorded camera runtime metadata read; no SDK diagnostics or live access.')


def main():
    ref = np.load(OLD / 'reference.npz')
    reference_hist = hist(ref['rgb'], ref['mask'])
    assert np.array_equal(reference_hist, ref['histogram'])
    rows = []
    for path in sorted(OLD.glob('*_source.npz')):
        newer = NEW / path.name
        if not newer.exists():
            continue
        old, new = np.load(path), np.load(newer)
        assert np.array_equal(old['rgb'], new['rgb'])
        assert np.all((old['depth'] == new['depth'])
                      | (np.isnan(old['depth']) & np.isnan(new['depth'])))
        assert np.array_equal(old['raw_mask'], new['raw_mask'])
        rgb, a, b = old['rgb'], old['refined_mask'] > 0, new['refined_mask'] > 0
        ha, hb = hist(rgb, a), hist(rgb, b)
        assert np.array_equal(ha, old['histogram'])
        assert np.array_equal(hb, new['histogram'])
        rows.append(dict(source_stamp_ns=int(path.stem.split('_')[0]),
                         same_rgb_depth_rawseed=True,
                         old_mask=dict(distances=distances(reference_hist, ha), stats=stats(rgb, a)),
                         new_mask=dict(distances=distances(reference_hist, hb), stats=stats(rgb, b)),
                         added_pixels=stats(rgb, b & ~a), removed_pixels=stats(rgb, a & ~b),
                         common_pixels=stats(rgb, a & b)))
    result = dict(scope='Offline same RGB/depth/raw-seed mask-composition diagnostic; no identity or motion authority.',
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  reference_ns=1791520119101696014,
                  reference_stats=stats(ref['rgb'], ref['mask']),
                  original_histograms_reproduced_bit_equal=True,
                  units='OpenCV Lab uint8: L scaled to 0..255, a and b offset by 128; BGR uint8.',
                  percentile_order=[5, 25, 50, 75, 95], rows=rows,
                  camera_metadata=camera_metadata(),
                  limitations=['Marginal Hellinger distances are not additive contributions to the joint distance.',
                               'Mask candidates are offline counterfactuals, not ground-truth instance annotations.',
                               'The same-source comparison holds RGB/exposure fixed; it diagnoses mask composition without proving cross-view photometric invariance.',
                               'Available 12 RGB-D pairs are not the complete set of online evaluated sources.',
                               'No production thresholds, images, masks, or depth were edited; no live ROS access.'])
    OUT.mkdir(exist_ok=True)
    (OUT / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(rows=len(rows), output=str(OUT / 'result.json'),
                         reference_mean=result['reference_stats']['Lab_u8']['mean'],
                         old_pass=sum(r['old_mask']['distances']['joint_Lab'] <= .4 for r in rows),
                         new_pass=sum(r['new_mask']['distances']['joint_Lab'] <= .4 for r in rows),
                         first=rows[0], camera_metadata=result['camera_metadata']), ensure_ascii=False))


if __name__ == '__main__':
    main()
