"""Offline counterfactual, NOT a proposed admission rule or deployment.

Keep exactly the native members intersecting the original spatial domain,
but retain their full current valid depth as the proposed seed. This isolates
seed clipping from all unchanged geometry/refinement/appearance checks.
"""
from pathlib import Path
import hashlib
import importlib.util
import inspect
import os
import sys

sys.dont_write_bytecode = True
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
O = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen57_resume_r3/actual_attempt_r1_geometry')
BASE = O / 'replay_masks_r2.py'
BASE_SHA = '76ac1bd14f91c6b2c31fa4d7a1aea1a16d2a92a3eed09b1b7f65247713984482'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert sha(BASE) == BASE_SHA
    spec = importlib.util.spec_from_file_location('gen57_native_seed_probe_base', BASE)
    base = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(base)
    base.D = O / 'native_seed_counterfactual_r1'
    import alicia_grasp_modes.arrival_fragment_group as fragment
    import alicia_grasp_modes.observation_arrival_confirmation as confirmation
    original = inspect.getsource(fragment.recover_arrival_fragment_group)
    old = "        current = native & valid & spatial['domain_mask']"
    new = """        # COUNTERFACTUAL ONLY: same members, full current measured seed.
        intersecting = native & valid & spatial['domain_mask']
        current = (native & valid) if np.any(intersecting) else intersecting"""
    assert original.count(old) == 1
    counterfactual = original.replace(old, new)
    exec(compile(counterfactual, '<offline_full_native_seed_counterfactual>', 'exec'), vars(fragment))
    confirmation.recover_arrival_fragment_group = fragment.recover_arrival_fragment_group
    original_write = base.write

    def write(name, value):
        if name == 'result.json':
            value.update(
                scope='Offline seed-clipping counterfactual on two exact recorded frames; no qualified runtime or deployable rule',
                baseline_harness=dict(path=str(BASE), sha256=BASE_SHA),
                counterfactual_helper=dict(path=str(Path(__file__).resolve()), sha256=sha(Path(__file__))),
                original_function_sha256=hashlib.sha256(original.encode()).hexdigest(),
                counterfactual_function_sha256=hashlib.sha256(counterfactual.encode()).hexdigest(),
                counterfactual_change=dict(before=old, after=new),
                all_original_arguments_returns_and_exceptions_preserved=False,
                original_geometry_appearance_thresholds_unchanged=True,
                runtime_clock_or_freshness_replayed=False,
                generalization_or_neighbour_ownership_proved=False,
                deployable=False)
            value['limitations'] = [
                'Full native seeds are a diagnostic intervention, not validated object ownership.',
                'Current source connectivity, competing objects and generalization remain unproved.',
                'No deadline, measured-face fallback, three-owner join, or motion is validated.',
                'Online comparisons intentionally measure a changed seed against the original rejected masks.',
                'Neither old masks nor colour/shape scores select a counterfactual member or tune a threshold.']
        return original_write(name, value)

    base.write = write
    base.main()
    assert sha(BASE) == BASE_SHA


if __name__ == '__main__':
    main()
