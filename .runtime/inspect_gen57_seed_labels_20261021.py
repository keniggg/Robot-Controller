"""Observe exact existing GrabCut labels on the two frozen rejection frames."""
from pathlib import Path
import hashlib
import importlib.util
import inspect
import json
import os
import sys

sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = os.environ['OMP_NUM_THREADS'] = '1'
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
BASE = O / 'replay_masks_r2.py'
BASE_SHA = '76ac1bd14f91c6b2c31fa4d7a1aea1a16d2a92a3eed09b1b7f65247713984482'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and sha(BASE) == BASE_SHA
    spec = importlib.util.spec_from_file_location('gen57_seed_label_base', BASE)
    base = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base)
    base.D = O / 'seed_label_diagnosis_r1'
    cv2, np = base.cv2, base.np
    original, calls = cv2.grabCut, []

    def observe(image, labels, *args, **kwargs):
        caller = inspect.currentframe().f_back
        local = caller.f_locals
        before = labels.copy()
        result = original(image, labels, *args, **kwargs)
        if caller.f_code.co_name == 'attempt':
            assert len(calls) < 20
            box = [int(local[k]) for k in ('ax0', 'ay0', 'ax1', 'ay1')]
            x0, y0, x1, y1 = box
            prefix = 'grabcut_%02d' % len(calls)
            arrays = dict(rgb=image.copy(), labels_before=before, labels_after=labels.copy(),
                          measured=local['measured_now'].copy(), core=local['core_now'].copy(),
                          support=local['support_now'].copy(),
                          depth=local['depth'][y0:y1, x0:x1].copy(),
                          heights=local['heights'][y0:y1, x0:x1].copy())
            np.savez_compressed(base.D / (prefix + '.npz'), **arrays)
            palette = np.array([[0, 0, 255], [0, 255, 0], [100, 100, 100], [255, 255, 0]], np.uint8)
            panels = [base.tile(image, 'Recorded crop'),
                      base.tile(palette[before], 'Before: hard BG red / FG green'),
                      base.tile(palette[labels], 'After: probable BG grey / FG cyan')]
            cv2.imwrite(str(base.D / (prefix + '.png')), np.hstack(panels))
            calls.append(dict(index=len(calls), box_xyxy=box,
                              counts_before={str(i): int((before == i).sum()) for i in range(4)},
                              counts_after={str(i): int((labels == i).sum()) for i in range(4)},
                              artifact=str(base.D / (prefix + '.npz'))))
        return result

    original_write = base.write
    def write(name, value):
        if name == 'result.json':
            value.update(label_observer_helper=dict(path=str(Path(__file__).resolve()), sha256=sha(Path(__file__))),
                         grabcut_calls=calls, grabcut_inputs_and_outputs_not_modified=True,
                         observer_runtime_not_online_timing=True)
        return original_write(name, value)
    base.write = write
    cv2.grabCut = observe
    try:
        base.main()
    finally:
        cv2.grabCut = original


if __name__ == '__main__':
    main()
