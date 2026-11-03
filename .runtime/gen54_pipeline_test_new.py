"""Run the new pure geometry regression under a process-local offline guard."""
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import struct
import sys
import time

import extract_gen50_retry_r30_rgb_20261018 as boundary

W = Path(__file__).resolve().parents[1]
C = W / '.runtime/gen54_integrated_candidate'
MODE = sys.argv[1] if len(sys.argv) > 1 else 'new'
assert MODE in ('new', 'recorded', 'height', 'height_focus')
OUT = boundary.N / 'verification/gen54_geometry_pipeline_r1' / (
    {'new':'new_tests_r2','recorded':'recorded_contract_test_r1','height':'height_tests_r2',
     'height_focus':'height_related_focus_r2'}[MODE])


def main():
    assert boundary.E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', TMPDIR=str(OUT),
                      ROS_PACKAGE_PATH=str(C/'src')+':/opt/ros/noetic/share',
                      ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(OUT), ROS_HOME=str(OUT))
    sys.dont_write_bytecode = True
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native) + (native.machine,)))
    platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    boundary.OUT = OUT
    sys.addaudithook(boundary.audit)
    sys.path[:0] = list(map(str, (C/'src/alicia_flexible_grasp_supervisor/src',
        C/'src/alicia_grasp_modes/src', W/'devel/lib/python3/dist-packages',
        Path('/opt/ros/noetic/lib/python3/dist-packages'))))
    test = C / 'src/alicia_flexible_grasp_supervisor/tests' / {
        'new':'test_gen54_open_aperture_refinement.py','recorded':'test_remote_grasp6d_streaming.py',
        'height':'test_gen54_height_window.py','height_focus':'test_gen54_height_window.py'}[MODE]
    fixture = test.parent / 'fixtures' / (
        {'new':'gen54_g53_measured_aperture.json.gz','recorded':'measured_contact_rgbd_20260910.npz',
         'height':'gen54_g53_height_window.json.gz','height_focus':'gen54_g53_height_window.json.gz'}[MODE])
    selector = str(test) + ('::test_configured_resolution_preserves_recorded_bilateral_contact' if MODE == 'recorded' else '')
    scopes = [selector]
    if MODE == 'height_focus':
        scopes += [str(test.parent / name) for name in (
            'test_contact_height_placement.py', 'test_contact_height_window_proposals.py',
            'test_selected_contact_height_recipe.py', 'test_contact_height_retry_diagnostic.py',
            'test_gen9_height_three_build_independent.py')]
    source_paths = [C / 'src/alicia_flexible_grasp_supervisor' / name for name in (
        'src/alicia_flexible_grasp/grasp/gripper_geometry.py',
        'src/alicia_flexible_grasp/grasp/tabletop_geometry_candidates.py',
        'src/alicia_flexible_grasp/grasp/contact_height_placement.py',
        'scripts/remote_grasp6d_node.py')]
    before = {str(p): boundary.sha(p) for p in [fixture] + source_paths
              + [Path(s.split('::')[0]) for s in scopes]}
    import pytest
    begin = time.monotonic()
    with (OUT / 'pytest.log').open('w') as log:
        oldout, olderr = sys.stdout, sys.stderr
        sys.stdout = sys.stderr = log
        try:
            code = pytest.main([*scopes, '-q', '-p', 'no:cacheprovider',
                                '--basetemp=' + str(OUT / 'tmp'),
                                '--junitxml=' + str(OUT / 'pytest.xml')])
        finally:
            sys.stdout, sys.stderr = oldout, olderr
    provenance = {name: dict(path=str(Path(module.__file__).resolve()),
                    sha256=boundary.sha(module.__file__)) for name, module in list(sys.modules.items())
                  if (name.startswith('alicia_') or name == 'remote_grasp6d_streaming_node')
                  and getattr(module, '__file__', None)}
    result = dict(exit_code=int(code), elapsed_sec=time.monotonic() - begin,
                  cpu_user_sec=resource.getrusage(resource.RUSAGE_SELF).ru_utime,
                  cpu_system_sec=resource.getrusage(resource.RUSAGE_SELF).ru_stime,
                  max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                  boundary_attempts=boundary.BOUNDARY, frozen_inputs=before,
                  test_scope=scopes, full_regression=False, module_provenance=provenance,
                  frozen_inputs_unchanged=all(boundary.sha(p) == s for p, s in before.items()),
                  ROS_initialized=False, hardware_contacted=False, offline_only=True)
    (OUT / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('exit_code','elapsed_sec','boundary_attempts',
        'frozen_inputs_unchanged','full_regression')}))
    return int(code)


if __name__ == '__main__':
    raise SystemExit(main())
