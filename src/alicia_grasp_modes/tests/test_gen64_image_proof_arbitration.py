"""Bounded image-proof arbitration checks; no ROS initialization or hardware."""
from dataclasses import replace
import hashlib
import importlib.util
from pathlib import Path
from types import FunctionType

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.arrival_fragment_group import recover_arrival_matching_fragments
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.tabletop import _project
from test_gen61_seed_insufficiency import raw_thin_region_scene
from test_gen57_source_component_measurement import _foreign


def bound_scene():
    scene, thin = raw_thin_region_scene()
    a = scene['arguments']
    # The independent CURRENT image excludes this support-coloured depth
    # strip. Its measured pixels remain untouched and enter the source graph.
    a['rgb'][thin] = (55, 100, 185)
    histogram, contour = _appearance(a['rgb'], scene['truth'].astype(np.uint8)*255)
    calls = []
    def independent():
        calls.append(True)
        kw = {k:v for k,v in a.items() if k in ('result','rgb','depth','intrinsics',
            'transform','config','anchor','last','spatial_reference','minimum_quality','rgb_valid')}
        return recover_arrival_matching_fragments(**kw, histogram=histogram, contour=contour)
    a['independent_measurement'] = independent
    return scene, thin, calls


def add_neighbor(scene, *, gap=4, width=12, missing=False, bridge=True):
    a = scene['arguments']
    y = int(np.median(np.nonzero(scene['top'])[0]))+12
    edge = int(np.flatnonzero(scene['truth'][y])[-1])+1
    neighbor = np.zeros(a['depth'].shape, bool)
    neighbor[y-width//2:y+width//2, edge+gap:edge+gap+width] = True
    join = np.zeros_like(neighbor)
    if bridge:
        join[y, edge:edge+gap] = True
    assert not np.any((neighbor|join) & scene['truth'])
    rays = _project(np.ones(neighbor.shape,np.float32), a['intrinsics'])
    plane_camera = a['transform'].T @ scene['plane']
    measured = ((.024-plane_camera[3]) /
        np.einsum('ijk,k->ij', rays, plane_camera[:3])).astype(np.float32)
    a['depth'][neighbor|join] = measured[neighbor|join]
    a['rgb'][neighbor|join] = (215,225,205)
    if missing:
        a['depth'][y,edge+gap//2] = 0.
    return neighbor, join


def test_full_raw_graph_and_negative_only_bound_survive_independent_success(monkeypatch):
    scene, thin, calls = bound_scene(); a = scene['arguments']
    depth = a['depth'].copy(); originals = [c.mask.copy() for c in a['result'].candidates]
    checked = []; original = owner._reject_single_pixel_bridge
    def graph(mask, depth, cores, edge, current, **kw):
        bounds = kw['possible_owners']
        assert len(bounds) == 1
        np.testing.assert_array_equal(bounds[0], a['source_topology'].source_array(thin))
        assert np.all(mask[bounds[0]]) and int(bounds[0].sum()) == 60
        assert all(not np.any(core & bounds[0]) for core in cores)
        checked.append((int(mask.sum()), [int(c.sum()) for c in cores]))
        return original(mask, depth, cores, edge, current, **kw)
    monkeypatch.setattr(owner, '_reject_single_pixel_bridge', graph)
    value = owner.recover_source_connected_measurement(**a)
    assert value is not None and calls == [True] and len(checked) == 1
    assert not np.any(value.refined.mask[thin])
    audit = value.audit
    assert audit['raw_foreground_component_points'] == checked[0][0]
    assert audit['articulation_guard_passed'] and not audit['native_union_used_as_rgb_seed']
    assert audit['independent_current_native_proof_used']
    region = audit['raw_only_unqualified_regions'][0]
    assert region['foreground_seed_pixels'] == 0
    assert region['role'] == 'negative_only_possible_owner_bound'
    assert region['region_mask_sha256'] == hashlib.sha256(thin.astype(np.uint8).tobytes()).hexdigest()
    assert region['minimum_core_pixels'] == 25 and region['minimum_measured_points'] == 60
    assert region['trusted_core_pixels'] < 25
    assert all(row['retained_fraction'] >= .85 for row in audit['native_members'])
    np.testing.assert_array_equal(a['depth'], depth)
    for c, old in zip(a['result'].candidates, originals):np.testing.assert_array_equal(c.mask, old)
    value.require_unchanged()


@pytest.mark.parametrize('gap,width', [(2,8),(4,12)])
def test_shortage_then_real_single_pixel_neighbor_bridge_still_rejects(gap,width):
    scene, thin, calls = bound_scene(); add_neighbor(scene,gap=gap,width=width)
    with pytest.raises(ValueError,match='current_fragment_single_pixel_measured_bridge'):
        owner.recover_source_connected_measurement(**scene['arguments'])
    assert calls == [True]  # Complete independent proof cannot overrule source veto.


@pytest.mark.parametrize('missing,bridge', [(True,True),(False,False)])
def test_shortage_never_merges_neighbor_over_missing_depth_or_measured_table(missing,bridge):
    scene, thin, calls = bound_scene(); neighbor,join = add_neighbor(scene,missing=missing,bridge=bridge)
    a=scene['arguments'];before=a['depth'].copy()
    value=owner.recover_source_connected_measurement(**a)
    assert calls == [True] and value is not None
    assert not np.any((value.refined.mask>0)&neighbor)
    assert not np.any((value.raw.mask>0)&neighbor)
    assert not np.any((value.raw.mask>0)&(a['depth']==0))
    assert value.audit['foreign_raw_foreground_components_checked'] >= 1
    np.testing.assert_array_equal(a['depth'],before)


@pytest.mark.parametrize('native,overlap', [(False,1),(False,144),(True,1)])
def test_shortage_cannot_hide_foreign_overlap_in_independent_proposal(native,overlap):
    scene,thin,calls=bound_scene();a=scene['arguments'];foreign,candidate=_foreign(scene,20,20)
    if native:a['result'].candidates.append(candidate)
    original=a['independent_measurement']
    def overlapping():
        result,pair=original();raw,refined=pair
        mask=refined.mask.copy();mask.ravel()[np.flatnonzero(foreign)[:overlap]]=255
        return result,(raw,replace(refined,mask=mask))
    a['independent_measurement']=overlapping
    with pytest.raises(owner.CurrentSourceOwnershipConflict) as caught:
        owner.recover_source_connected_measurement(**a)
    assert calls == [True] and caught.value.evidence['overlap_pixels']==overlap
    assert caught.value.evidence['component_basis']==('native_measured' if native else 'raw_source')


def test_shortage_keeps_original_typed_rejection_when_full_independent_proof_fails():
    scene,thin,calls=bound_scene();a=scene['arguments'];a['rgb'][scene['truth']]=(55,100,185)
    with pytest.raises(owner.RawOnlySeedInsufficient) as caught:
        owner.recover_source_connected_measurement(**a)
    assert calls == [True]
    assert isinstance(caught.value.__cause__,ValueError)
    assert str(caught.value.__cause__).startswith('matching_fragment_')


@pytest.mark.parametrize('field',['depth','config','camera','candidate_identity'])
def test_inputs_changed_during_independent_proof_cannot_be_committed(field):
    scene,thin,calls=bound_scene();a=scene['arguments'];original=a['independent_measurement']
    def mutate():
        value=original()
        if field=='depth':a['depth'][0,0]+=.0001
        elif field=='config':a['config'].association_distance_m+=.0001
        elif field=='camera':a['camera']['fx']+=1.
        else:a['result'].candidates[0]=replace(a['result'].candidates[0])
        return value
    a['independent_measurement']=mutate
    with pytest.raises(ValueError,match='original_source_changed'):
        owner.recover_source_connected_measurement(**a)


def test_typed_source_expiry_after_independent_proof_propagates():
    scene,thin,calls=bound_scene();a=scene['arguments'];original=a['independent_measurement'];expired=[False]
    class Expired(ValueError):pass
    def proof():
        value=original();expired[0]=True;return value
    def check():
        if expired[0]:raise Expired('source expired')
    a.update(independent_measurement=proof,check_current=check)
    with pytest.raises(Expired,match='source expired'):owner.recover_source_connected_measurement(**a)


@pytest.mark.parametrize('fault',['unsupported','intrinsics','activation'])
def test_unproved_topology_cannot_reach_independent_fallback(fault):
    scene,thin,calls=bound_scene();a=scene['arguments']
    if fault=='unsupported':a['source_topology']=None
    elif fault=='intrinsics':a['intrinsics']=a['intrinsics']+[1.,0.,0.,0.]
    else:a['camera']['runtime_profile']['geometry_activation_stamp_ns']='100'
    with pytest.raises(ValueError):owner.recover_source_connected_measurement(**a)
    assert calls==[]


@pytest.mark.parametrize('failure',['matching','source_after_matching'])
def test_real_process_callsite_never_uses_one_fragment_after_proof_failure(monkeypatch,failure):
    # Transport fixture executes the real process method. Only the component
    # helper boundary supplies a controlled proof failure; its callback is
    # the exact candidate callsite's original complete image proof.
    from test_gen57_source_component_pipeline import pipeline,assert_no_new_geometry,module
    path=Path(owner.__file__).resolve().parents[2]/'scripts/unknown_tabletop_perception.py'
    spec=importlib.util.spec_from_file_location('image_candidate_callsite',path)
    candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
    code=candidate.UnknownTabletopPerception._recover_station_source_component_measurement
    method=FunctionType(code.__code__,module.__dict__,code.__name__,code.__defaults__,code.__closure__)
    monkeypatch.setattr(module.UnknownTabletopPerception,'_recover_station_source_component_measurement',method)
    env=pipeline(monkeypatch);seen=[]
    original_matching=module.recover_arrival_matching_fragments
    def matching(*args,**kwargs):
        seen.append('complete_independent_proof')
        if failure=='matching':raise ValueError('matching_fragment_competitor_not_owned')
        return original_matching(*args,**kwargs)
    monkeypatch.setattr(module,'recover_arrival_matching_fragments',matching)
    def component(*args,**kwargs):
        seen.append('source_proof')
        kwargs['independent_measurement']()
        raise ValueError('current_fragment_native_core_not_owned')
    monkeypatch.setattr(module,'recover_source_connected_measurement',component)
    env.run()
    assert seen==['source_proof','complete_independent_proof']
    assert_no_new_geometry(env)
