from dataclasses import replace
import numpy as np
import pytest

from alicia_grasp_modes.metric_ambiguity_identity import metric_seeded_identity_evidence
from alicia_grasp_modes.tabletop import Candidate


def scene():
    rgb=np.full((100,120,3),100,np.uint8);mask=np.zeros(rgb.shape[:2],np.uint8)
    mask[30:55,40:65]=255;rgb[mask>0]=(20,40,60)
    other=np.zeros_like(mask);other[65:85,45:65]=255;rgb[other>0]=(230,210,30)
    candidates=[Candidate(m,None,None,None,None,None,0.,.8) for m in (mask,other)]
    return rgb,candidates,mask


def evaluate(rgb,candidates,mask,**kwargs):
    return metric_seeded_identity_evidence(rgb,candidates,rgb.copy(),mask,
        minimum_quality=.5,refine=kwargs.pop('refine',lambda c:c),**kwargs)


def test_all_candidates_are_refined_without_metric_selection():
    rgb,candidates,mask=scene();called=[]
    def refine(candidate):called.append(candidate);return candidate
    refined,evidence=evaluate(rgb,candidates,mask,refine=refine)
    assert len(called)==len(refined)==2
    assert evidence['evidence_type']=='metric_seeded_full_outline_rgb_identity'
    assert evidence['candidates'][0]['identity_compatible']
    assert evidence['candidates'][1]['explicit_nonownership']
    for key in ('metric_geometry_authority','motion_authorized','candidate_selected',
                'geometry_merged','independent_mser_contract_claimed'):
        assert evidence[key] is False


@pytest.mark.parametrize('change', ['same_colour_alternative','partial_overlap','overlapping_components',
    'full_outline_changed','alternative_refinement_fails','budget','bad_quality','invalid_rgb_domain'])
def test_unproven_alternative_or_source_never_becomes_identity_evidence(change):
    rgb,candidates,mask=scene();kwargs={}
    if change=='same_colour_alternative':rgb[candidates[1].mask>0]=rgb[mask>0][0]
    elif change=='partial_overlap':
        overlap=candidates[1].mask.copy();overlap[54:66,63:65]=255
        kwargs['refine']=lambda c:replace(c,mask=overlap) if c is candidates[1] else c
    elif change=='overlapping_components':candidates[1].mask[54:66,63:65]=255
    elif change=='full_outline_changed':
        small=mask.copy();small[30:40]=0
        kwargs['refine']=lambda c:replace(c,mask=small) if c is candidates[0] else c
    elif change=='alternative_refinement_fails':
        def refine(c):
            if c is candidates[1]:raise ValueError('original_refinement_failed')
            return c
        kwargs['refine']=refine
    elif change=='budget':candidates=candidates*9
    elif change=='bad_quality':candidates[1].quality=.49
    elif change=='invalid_rgb_domain':
        valid=np.ones(mask.shape,bool);valid[40,50]=False;kwargs['rgb_valid']=valid
    expected = ('alternative_ownership_unresolved' if change=='partial_overlap' else
                'native_components_overlap' if change=='overlapping_components' else None)
    with pytest.raises(ValueError, match=expected):evaluate(rgb,candidates,mask,**kwargs)


def test_later_single_metric_candidate_still_requires_complete_rgb_identity():
    rgb,candidates,mask=scene()
    _,evidence=evaluate(rgb,candidates[:1],mask)
    assert evidence['candidate_count']==1
    with pytest.raises(ValueError,match='no_unique_complete_rgb_identity'):
        evaluate(rgb,candidates[1:],mask)
