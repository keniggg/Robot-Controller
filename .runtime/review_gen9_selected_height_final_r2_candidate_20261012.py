#!/usr/bin/env python3
"""Independent fixed-candidate offline tests; ROS master is unreachable."""
import hashlib
import json
import os
from pathlib import Path
import sys
import pytest

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE = W / '.runtime/gen9_contact_selected_height_20261012'
OUT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/verification/gen9_selected_height_independent_final_r2_tests')
OUT.mkdir(parents=True, exist_ok=False)
assert os.environ['ROS_MASTER_URI'] == 'http://127.0.0.1:1'
assert os.environ['PYTHONNOUSERSITE'] == '1'
os.environ['ROS_LOG_DIR'] = str(OUT/'ros')
os.environ['ROS_PACKAGE_PATH'] = str(STAGE/'src')+':'+os.environ.get('ROS_PACKAGE_PATH','')
for rel in reversed(['src/alicia_flexible_grasp_supervisor/src', 'src/alicia_grasp_modes/src',
                     'src/alicia_flexible_grasp_supervisor/tests', 'src/alicia_grasp_modes/tests']):
    sys.path.insert(0, str(STAGE/rel))
import test_remote_grasp6d_streaming as fixtures
import numpy
import scipy
source = STAGE/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
assert Path(fixtures.remote_node.__file__).resolve() == source
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(source) == 'df7e874b3ce491d18f785773df81df5f86357e3a493db0b02d97892f567d137a'
tests = ['src/alicia_flexible_grasp_supervisor/tests/'+name for name in (
    'test_selected_contact_height_recipe.py', 'test_contact_height_window_proposals.py',
    'test_contact_height_retry_diagnostic.py', 'test_final_registered_orientation_resolution.py',
    'test_contact_goal_necessary_rejection.py', 'test_contact_height_margin.py',
    'test_contact_height_placement.py', 'test_contact_height_audit_binding.py',
    'test_contact_sequence_precheck.py')]
tests.append('src/alicia_grasp_modes/tests/test_remote_modes.py')
tests.append('src/alicia_flexible_grasp_supervisor/tests/test_gen9_height_three_build_independent.py')
for name in ('test_contact_search_yield.py', 'test_contact_search_yield_remote.py',
             'test_contact_search_yield_receipts.py', 'test_contact_search_yield_accept.py',
             'test_contact_search_yield_reached_gain.py', 'test_gen7_contact_yield_completed_budget.py'):
    tests.append('src/alicia_grasp_modes/tests/'+name)
import alicia_grasp_modes.observe_return_remote as mixin
mixin_source = STAGE/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py'
assert Path(mixin.__file__).resolve() == mixin_source
assert sha(mixin_source) == '97816333efd6772b96f61cf0c6a25c2b9b69c09ea3fd2254fab07608e23bc9da'
import test_remote_modes as mode_fixtures
assert Path(mode_fixtures.base.__file__).resolve() == source
assert sha(mode_fixtures.base.__file__) == sha(source)
bindings = {name: sha(STAGE/name) for name in tests}
report = dict(candidate_source=str(source), candidate_sha256=sha(source),
    actual_import=str(fixtures.remote_node.__file__), mode_original_import=str(mode_fixtures.base.__file__), mode_original_sha256=sha(mode_fixtures.base.__file__), mixin_import=str(mixin.__file__), mixin_sha256=sha(mixin_source), python=sys.executable,
    numpy=dict(path=numpy.__file__, version=numpy.__version__),
    scipy=dict(path=scipy.__file__, version=scipy.__version__),
    environment={k: os.environ.get(k) for k in ('ROS_MASTER_URI','ROS_LOG_DIR','ROS_PACKAGE_PATH','PYTHONNOUSERSITE',
        'PYTHONDONTWRITEBYTECODE','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS')},
    tests_sha256=bindings, hardware_actions=False, rpc=False)
(OUT/'imports.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
result = pytest.main(['-q','-p','no:cacheprovider','--junitxml='+str(OUT/'pytest.xml')]+
                     [str(STAGE/name) for name in tests])
report.update(returncode=int(result), source_unchanged=sha(source)==report['candidate_sha256'],
    mixin_source_unchanged=sha(mixin_source)==report['mixin_sha256'],
    test_sources_unchanged=all(sha(STAGE/name)==digest for name,digest in bindings.items()))
(OUT/'result.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
raise SystemExit(result)
