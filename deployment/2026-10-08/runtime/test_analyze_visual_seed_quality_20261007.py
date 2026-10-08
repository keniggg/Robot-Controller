"""Pure seed selection and known-motion controls; no physical camera access."""
import importlib.util
from pathlib import Path
import cv2
import numpy as np
import pytest
p=Path(__file__).with_name('analyze_visual_seed_quality_20261007.py')
s=importlib.util.spec_from_file_location('seed_quality_test',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)

@pytest.fixture(autouse=True)
def threads():
    old=cv2.getNumThreads();cv2.setNumThreads(1);yield;cv2.setNumThreads(old)


def reference():
    raw=np.random.RandomState(141).randint(0,256,(480,640,3),np.uint8)
    raw=cv2.GaussianBlur(raw,(5,5),1.)
    bounds=[128,48,512,432];_,gray,scale=m.r.gray_for(raw,bounds)
    points,_=m.seed(gray,.02)
    return dict(anchor_raw_color=raw,anchor_gray=gray,seed_points=points),dict(bounds_xyxy=bounds),gray,scale


def test_original_seed_algorithm_reproduces_actual_visual_window_seed():
    arrays,meta,gray,scale=reference();v=m.r.V();assert v._seed(gray,1000.,1.)
    np.testing.assert_array_equal(arrays['seed_points'],v._points)
    assert m.QUALITIES==(.04,.08,.12)


@pytest.mark.parametrize('quality',[.04,.08,.12])
def test_fixed_quality_keeps_motion_controls_and_reports_count_without_mutating_anchor(quality):
    arrays,meta,gray,scale=reference();before={k:v.copy() for k,v in arrays.items()}
    row=m.candidate(arrays,meta,gray,scale,quality)
    assert row['seed_eligible'] and row['seed_points_count']>=24 and row['covered_quadrants']>=3
    assert row['actual_pair']['p90_motion_gate_passed']
    assert len(row['synthetic_controls'])==9 and row['all_synthetic_motion_detected']
    assert all(x['tracks_eligible'] and x['motion_p90_px']>.25 for x in row['synthetic_controls'])
    assert all(np.array_equal(v,before[k]) for k,v in arrays.items())


def test_low_texture_is_rejected_not_silently_called_stationary():
    arrays,meta,gray,scale=reference();arrays['anchor_gray']=np.zeros_like(gray)
    row=m.candidate(arrays,meta,gray,scale,.12)
    assert row['seed_eligible'] is False and row['reason']=='insufficient_texture'
    assert row['actual_pair'] is None and row['synthetic_controls']==[]


def test_unknown_quality_cannot_be_chosen_after_seeing_outcome():
    with pytest.raises(ValueError,match='unregistered_quality_candidate'):
        m.seed(np.zeros((192,192),np.uint8),.2)
