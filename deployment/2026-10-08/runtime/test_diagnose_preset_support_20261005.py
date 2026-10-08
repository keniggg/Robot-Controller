import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('support_diagnostic',Path(__file__).with_name('diagnose_preset_support_20261005.py'))
diagnostic=importlib.util.module_from_spec(spec);spec.loader.exec_module(diagnostic)


@pytest.mark.parametrize('kind',['plane','nonplanar','too_few'])
def test_trace_preserves_original_plane_admission_and_python_trace(kind):
    rng=np.random.RandomState(31)
    points=np.c_[rng.uniform(-.1,.1,(900,2)),np.full(900,.3)]
    if kind=='nonplanar':points[:,2]+=rng.normal(0,.006,900)
    elif kind=='too_few':points=points[:20]
    cfg=diagnostic.tt.Config()
    try:
        expected=diagnostic.tt._fit_plane(points,cfg);error=None
    except ValueError as exc:expected=None;error=str(exc)
    old=sys.gettrace()
    plane,audit=diagnostic.fit_audit(points,cfg)
    assert sys.gettrace() is old
    assert audit['passed_original_gate']==(expected is not None)
    assert audit['reason']==error
    assert audit['full_support_admission_evaluated'] is False
    assert audit['grasp_authority'] is False
    if expected is not None:
        np.testing.assert_array_equal(plane,expected[0])
        assert audit['sampled_inlier_ratio']==expected[1]
    elif kind=='nonplanar':
        assert error=='support_plane_inlier_ratio_low'
        assert plane is not None and audit['sampled_inlier_ratio']<cfg.support_min_ratio
        assert audit['all_point_abs_residual_mm']['p90']>2.
    else:
        assert plane is None and 'model' not in audit
