"""Diagnostic only: reliable near-parallel measured interior, not a production rule."""
from pathlib import Path
import hashlib
import importlib.util
import inspect
import os
import sys
sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = os.environ['OMP_NUM_THREADS'] = '1'
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
BASE = O / 'replay_masks_r2.py'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == '76ac1bd14f91c6b2c31fa4d7a1aea1a16d2a92a3eed09b1b7f65247713984482'
spec = importlib.util.spec_from_file_location('upper_band_base', BASE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
base.D = O / 'reliable_interior_counterfactual_r1'
import alicia_grasp_modes.measured_mask_seeds as seeds
import alicia_grasp_modes.rgb_instance as rgb_instance
original = seeds._confidence_labels
source = inspect.getsource(original)
source = source.replace('if np.count_nonzero(foreground) < 25 and original_top_pixels < 25:', 'if True:')
source = source.replace('top = (core & reliable', 'top = top | (core & reliable')
source = source.replace(' & (heights >= upper_height - LOCAL_SURFACE_MAX_RMS_M)', '')
source = source.replace(' & (heights <= upper_height + LOCAL_SURFACE_MAX_RMS_M)', '')
scope = dict(vars(seeds))
exec(compile(source, '<offline_existing_upper_band_all_sizes>', 'exec'), scope)
counterfactual = scope['_confidence_labels']
calls = []


def labels(*args, **kwargs):
    # Preserve the historically frozen reference reconstruction. The causal
    # intervention applies to arrival inputs only, never changes old evidence.
    if not (base.D / 'reference_reconstruction.json').exists():
        return original(*args, **kwargs)
    out = counterfactual(*args, **kwargs)
    calls.append(out[2])
    return out


seeds._confidence_labels = rgb_instance._confidence_labels = labels
write = base.write
def report(name, value):
    if name == 'result.json':
        value.update(scope='Diagnostic only reliable-interior intervention on two arrival frames, not deployable ownership or qualification',
            observer_only_wraps_original_appearance_call=False,
            all_original_arguments_returns_and_exceptions_preserved=False,
            counterfactual_seed_reports=calls, counterfactual_function_source=source,
            original_clock_replayed=False, runtime_deployed=False)
    return write(name, value)
base.write = report
base.main()
