"""Within-call reuse must preserve independent labels and measured geometry."""
from dataclasses import fields
import numpy as np
import pytest

from alicia_grasp_modes import rgb_instance as rgb
from alicia_grasp_modes import arrival_fragment_group as group
from alicia_grasp_modes import measured_mask_seeds as seeds
from test_arrival_fragment_recovery_guards import fragment_scene


class UnsharedFrame:
    """Reference composition using the public, uncached operators each time."""
    def __init__(self, depth, intrinsics, transform, reference, config):
        self.depth, self.intrinsics, self.transform = depth, intrinsics, transform
        self.reference, self.config = reference, config
        self.points = rgb._project(depth, intrinsics)
        self.spatial = rgb._observation_spatial_support(
            self.points, transform, intrinsics, reference, config)

    def refine(self, bgr, candidate, *, rgb_valid=None):
        return rgb.refine_candidate(bgr, self.depth, self.intrinsics,
            self.transform, candidate, self.config, rgb_valid=rgb_valid,
            observation_arrival_retry=True, observation_spatial_reference=self.reference)

    confidence_labels = staticmethod(seeds.confidence_labels)


def assert_candidate_equal(left, right):
    for field in fields(left):
        a, b = getattr(left, field.name), getattr(right, field.name)
        if isinstance(a, np.ndarray):
            assert a.dtype == b.dtype
            assert a.tobytes() == b.tobytes(), field.name
        else:
            assert a == b, field.name


def test_original_fragment_outputs_and_audit_are_bitexact_with_one_projection_and_fit(monkeypatch):
    inputs = fragment_scene()
    with monkeypatch.context() as original:
        original.setattr(group, '_FrameRefinement', UnsharedFrame)
        previous, before = group.recover_arrival_fragment_group(**inputs)
    calls = dict(project=0, domain=0, normals=0)
    for name, key in (('_project','project'),('_observation_spatial_support','domain'),('local_surface_normals','normals')):
        function = getattr(rgb, name)
        def count(*a, _fn=function, _key=key, **kw):
            calls[_key] += 1
            return _fn(*a, **kw)
        monkeypatch.setattr(rgb, name, count)
    result, after = group.recover_arrival_fragment_group(**inputs)
    assert calls == dict(project=1, domain=1, normals=1)
    for old, new in zip(before, after):
        assert_candidate_equal(old, new)
    assert result.metrics == previous.metrics


@pytest.mark.parametrize('difference', ['height', 'normal', 'rgb', 'points', 'depth', 'valid'])
def test_local_fit_reuse_does_not_reuse_core_or_accept_changed_inputs(monkeypatch, difference):
    inputs = fragment_scene()
    frame = rgb._FrameRefinement(inputs['depth'], inputs['intrinsics'], inputs['transform'], inputs['spatial_reference'], inputs['config'])
    seed = np.logical_or.reduce([v.mask > 0 for v in inputs['result'].candidates])
    yy, xx = np.nonzero(seed)
    sl = np.s_[yy.min()-16:yy.max()+17, xx.min()-16:xx.max()+17]
    points = frame.points[sl].copy(); depth = inputs['depth'][sl].copy()
    height = .3-points[..., 0]; normal = np.array([-1.,0.,0.]); color=inputs['rgb'][sl].copy()
    calls=[];original=rgb.local_surface_normals
    def counted(*a):calls.append(1);return original(*a)
    monkeypatch.setattr(rgb,'local_surface_normals',counted)
    frame.confidence_labels(seed[sl],points,depth,height,normal,inputs['config'],bgr=color)
    if difference=='height':height=np.nextafter(height,np.inf)
    elif difference=='normal':normal=np.array([-.999999,0.,.001])
    elif difference=='rgb':color=color.copy();color[...,1]=np.minimum(color[...,1]+1,254)
    elif difference=='points':points=points.copy();points[0,0,0]+=1e-5
    elif difference=='depth':depth=depth.copy();depth[0,0]+=1e-5
    else:depth=depth.copy();depth[0,0]=0.
    expected=seeds.confidence_labels(seed[sl],points,depth,height,normal,inputs['config'],bgr=color)
    actual=frame.confidence_labels(seed[sl],points,depth,height,normal,inputs['config'],bgr=color)
    np.testing.assert_array_equal(actual[0],expected[0]);np.testing.assert_array_equal(actual[1],expected[1]);assert actual[2]==expected[2]
    assert len(calls)==(1 if difference in ('height','normal','rgb') else 2)
    assert not frame.points.flags.writeable and not frame.spatial['domain_mask'].flags.writeable
    assert not frame._fit[3][0].flags.writeable and not frame._fit[3][1].flags.writeable


def test_public_refinement_still_computes_each_frame_and_preserves_default_outputs(monkeypatch):
    inputs=fragment_scene();candidate=inputs['anchor'];calls=[];project=rgb._project
    kwargs=dict(rgb_valid=None,observation_arrival_retry=False)
    old=rgb.refine_candidate(inputs['rgb'],inputs['depth'],inputs['intrinsics'],inputs['transform'],candidate,inputs['config'],**kwargs)
    def counted(*a):calls.append(1);return project(*a)
    monkeypatch.setattr(rgb,'_project',counted)
    for _ in range(2):
        new=rgb.refine_candidate(inputs['rgb'],inputs['depth'],inputs['intrinsics'],inputs['transform'],candidate,inputs['config'],**kwargs)
        assert_candidate_equal(old,new)
    assert len(calls)==2
