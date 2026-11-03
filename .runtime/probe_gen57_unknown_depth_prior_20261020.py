"""Offline diagnostic: unknown depth does not supply a probable background prior."""
from pathlib import Path
import hashlib
import importlib.util
import os
import inspect
import sys
sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = os.environ['OMP_NUM_THREADS'] = '1'
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
BASE = O / 'replay_masks_r2.py'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == '76ac1bd14f91c6b2c31fa4d7a1aea1a16d2a92a3eed09b1b7f65247713984482'
spec = importlib.util.spec_from_file_location('lab_mask_base', BASE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
base.D = O / 'unknown_depth_prior_counterfactual_r1'
original = base.cv2.grabCut


def grabcut(rgb, *args, **kwargs):
    if (base.D / 'reference_reconstruction.json').exists():
        loc = inspect.currentframe().f_back.f_locals
        crop = loc['depth'][loc['ay0']:loc['ay1'],loc['ax0']:loc['ax1']]
        config = loc['config']
        unknown = ~base.np.isfinite(crop) | (crop < config.depth_min_m) | (crop > config.depth_max_m)
        args[0][unknown & (args[0] == base.cv2.GC_PR_BGD)] = base.cv2.GC_PR_FGD
    return original(rgb, *args, **kwargs)


base.cv2.grabCut = grabcut
write = base.write
def report(name, value):
    if name == 'result.json':
        value.update(scope='Offline unknown-depth probable prior intervention, original hard labels and geometry/appearance thresholds unchanged',
            observer_only_wraps_original_appearance_call=False,
            all_original_arguments_returns_and_exceptions_preserved=False,
            original_clock_replayed=False, runtime_deployed=False)
    return write(name, value)
base.write = report
base.main()
