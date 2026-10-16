"""Frozen measured footprints propose crops, never historical depth authority."""
from types import SimpleNamespace
import numpy as np
import pytest
from alicia_grasp_modes.rgb_instance import freeze_observation_spatial_reference, _observation_spatial_support
from alicia_grasp_modes.tabletop import Config,_project


def inputs():
    plane=np.array([-1.,0.,0.,.3]);footprint=np.array([[.3,-.02,-.01],[.3,.02,-.01],[.3,.02,.01],[.3,-.02,.01]])
    anchor=SimpleNamespace(support_footprint_base=footprint,metrics={'median_height_m':.02})
    return anchor,plane


def test_reference_is_immutable_copy_and_projects_current_points_only():
    anchor,plane=inputs();ref=freeze_observation_spatial_reference(anchor,plane)
    saved=ref['footprint_base'].copy();anchor.support_footprint_base[:]=99;plane[:]=99
    np.testing.assert_array_equal(ref['footprint_base'],saved)
    with pytest.raises(ValueError):ref['footprint_base'][0,0]=0
    with pytest.raises(ValueError):ref['support_plane_base'][0]=0
    points=np.array([[[.28,0.,0.],[.28,.03,0.],[np.nan,0.,0.]]])
    output=_observation_spatial_support(points,np.eye(4),[400.,400.,320.,240.],ref,Config())
    assert output['domain_mask'].tolist()==[[True,False,False]]
    assert output['provenance']['historical_pixels_counted']==0
    assert output['provenance']['footprint_expansion_m']==0.


@pytest.mark.parametrize('fault',['nan_plane','no_normal','off_plane','degenerate','oversize','nan_height','boolean_height','invalid_transform'])
def test_invalid_spatial_proposal_is_rejected(fault):
    anchor,plane=inputs();ref=freeze_observation_spatial_reference(anchor,plane)
    ref={k:v.copy() if isinstance(v,np.ndarray) else v for k,v in ref.items()};T=np.eye(4)
    if fault=='nan_plane':ref['support_plane_base'][0]=np.nan
    elif fault=='no_normal':ref['support_plane_base'][:3]=0
    elif fault=='off_plane':ref['footprint_base'][:,0]+=.01
    elif fault=='degenerate':ref['footprint_base'][:]=ref['footprint_base'][0]
    elif fault=='oversize':ref['footprint_base'][:,1:]*=20
    elif fault=='nan_height':ref['height_m']=float('nan')
    elif fault=='boolean_height':ref['height_m']=True
    elif fault=='invalid_transform':T[0,0]=float('nan')
    with pytest.raises(ValueError):_observation_spatial_support(np.zeros((2,3,3)),T,[400.,400.,320.,240.],ref,Config())


def test_qualified_station_passes_same_fixed_reference_after_three_frames(monkeypatch):
    from test_observation_arrival_station import completed,next_frame,latest_status,module
    n,clock,feed,joint,params,marker=completed(monkeypatch)
    reference=marker['spatial_reference'];footprint=reference['footprint_base'].copy();plane=reference['support_plane_base'].copy()
    calls=[];metrics=[];original=module.refine_candidate
    def observed(*args,**kwargs):
        calls.append(kwargs.get('observation_spatial_reference'))
        result=original(*args,**kwargs);metrics.append(result.metrics)
        return result
    monkeypatch.setattr(module,'refine_candidate',observed)
    for _ in range(3):
        next_frame(n,clock,joint)
        assert latest_status(n)['state']=='ready',latest_status(n)
        assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert len(calls)==3 and all(value is reference for value in calls)
    np.testing.assert_array_equal(reference['footprint_base'],footprint)
    np.testing.assert_array_equal(reference['support_plane_base'],plane)
    assert len(metrics)==3 and all('rgb_to_current_measured_support_area_ratio' in row for row in metrics)


def test_multiple_metric_candidates_use_original_hold_then_unique_spatial_recovery(monkeypatch):
    from test_observation_arrival_station import completed,latest_status,module
    from test_metric_ambiguity_hold import frame
    n,clock,feed,joint,params,marker=completed(monkeypatch)
    spatial=marker['spatial_reference'];calls=[];original=module.refine_candidate
    def inspect(*args,**kwargs):
        calls.append((kwargs.get('observation_arrival_retry'),kwargs.get('observation_spatial_reference')))
        return original(*args,**kwargs)
    monkeypatch.setattr(module,'refine_candidate',inspect)
    clock[0]+=.1;frame(n,clock,joint)
    assert len(calls)==2 and all(flag is True and ref is None for flag,ref in calls)
    assert not n.tracker.lost and n._metric_ambiguity_pending is not None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    clock[0]+=.2;frame(n,clock,joint,extra=False)
    assert calls[-1][0] is True and calls[-1][1] is spatial
    assert latest_status(n)['state']=='ready' and n._metric_ambiguity_pending is None
