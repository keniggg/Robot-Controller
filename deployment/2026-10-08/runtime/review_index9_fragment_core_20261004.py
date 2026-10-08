"""Extract real full-producer index-9 masks and audit a proposed core rule only."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('raw_replay', HERE/'review_raw_fragment_hold_20261004.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
module = replay.module


def main():
    source = Path(sys.argv[1])
    output = Path(sys.argv[2]).resolve()
    mount = Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount/'实验记录/Robot-Controller' not in output.parents:
        raise RuntimeError('SanStand output required')
    rows, frames = replay.load_archive(source)
    target_stamp = rows[9]['source_stamp_ns']
    captured = {}
    original = module.UnknownTabletopPerception._locked_rgb_shrinking_fragment

    def capture(self, candidate, reference, messages, camera):
        if messages[0].header.stamp.to_nsec() == target_stamp:
            captured.update(candidate=deepcopy(candidate), reference=deepcopy(reference),
                            policy=deepcopy(self._rgb_interest_policy))
        return original(self, candidate, reference, messages, camera)

    module.UnknownTabletopPerception._locked_rgb_shrinking_fragment = capture
    try:
        run = replay.replay(rows, frames, 0)
    finally:
        module.UnknownTabletopPerception._locked_rgb_shrinking_fragment = original
    reference = captured['reference']
    mask = captured['candidate'].mask
    x, y, width, height = cv2.boundingRect(reference['mask'])
    from dataclasses import replace
    rgb, native, transform = frames[9]
    policy = replace(captured['policy'], center_xy=((x+.5*width)/rgb.shape[1], (y+.5*height)/rgb.shape[0]))
    proposals = module.propose_rgb_observation_regions(rgb, policy=policy)
    source_row = next(item for item in run['rows'] if item['index'] == 9)
    record = source_row['rgb_boundary_audit']
    eligible = [item for item in record['proposals'] if item['original_match_gates_pass']]
    assert len(eligible) == 1
    current = proposals.candidates[eligible[0]['index']].mask.astype(np.uint8)
    core = cv2.erode(mask, np.ones((3, 3), np.uint8))
    metrics = dict(fragment_pixels=int(np.count_nonzero(mask)), core_pixels=int(np.count_nonzero(core)))
    for name, outline in (('reference', reference['mask']), ('current', current)):
        outside = (mask != 0) & (outline == 0)
        one_ring = cv2.dilate(outline.astype(np.uint8), np.ones((3, 3), np.uint8))
        ys, xs = np.nonzero(outside)
        metrics[name] = dict(outside_pixels=int(np.count_nonzero(outside)),
            retained_fraction=float(1.-np.count_nonzero(outside)/np.count_nonzero(mask)),
            core_outside_pixels=int(np.count_nonzero((core != 0) & (outline == 0))),
            fragment_outside_one_pixel_dilation=int(np.count_nonzero((mask != 0) & (one_ring == 0))),
            outside_xy=np.column_stack((xs, ys)).tolist())
    npz_path = output.with_suffix('.npz')
    np.savez_compressed(npz_path, reference_rgb=reference['bgr'], reference_mask=reference['mask'],
        current_rgb=rgb, current_depth_raw=native, current_T_base_camera=transform,
        current_unique_rgb_mask=current, candidate_mask=mask, candidate_eroded_core=core)
    report = dict(source=str(source), source_sha256=replay.sha(source), script_sha256=replay.sha(__file__),
        producer_sha256=replay.sha(module.__file__), diagnostic_masks_file=str(npz_path),
        diagnostic_masks_sha256=replay.sha(npz_path), reference_stamp_ns=reference['source_stamp_ns'],
        source_stamp_ns=target_stamp, metrics=metrics,
        actual_producer_status=source_row['rgb_hold'], original_rgb_association=record,
        interpretation=['Masks extracted from actual producer sequence initialized at archived frame 0.',
            'Core/dilation computations are diagnostic only; no production rule was changed.',
            'This later archive is not exact generation 9 first-loss evidence.'],
        ros_io=False, robot_commands_sent=False)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output), metrics=metrics), indent=2))


if __name__ == '__main__':
    cv2.setNumThreads(1)
    main()
