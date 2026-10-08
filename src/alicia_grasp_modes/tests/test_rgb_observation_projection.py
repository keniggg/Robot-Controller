"""Canonical projection must survive exact protocol digests at real intrinsics."""
from dataclasses import asdict, replace

import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_projection import projected_rgb_search_policy
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy
from alicia_grasp_modes.rgb_observation_source import digest
from alicia_grasp_modes.rgb_observation_volume import project_rgb_observation_vertices

# Preserved original reference vertices from the 2026-10-04 04:42:13 source
# artifact. These are a regression fixture, never a production ROI or offset.
VERTICES = np.array([
    [-0.09933372274034186, -0.385741733288983, 0.0020832826529188497],
    [-0.1251354570914543, -0.37721971301603974, 0.0007410403262448351],
    [-0.11551794509269031, -0.35168529734036985, 5.057120646281832e-05],
    [-0.09095040051373139, -0.359871378216288, 0.0013315829855437],
    [-0.08108144581124471, -0.30259362649108895, 0.11952537280884215],
    [-0.0930607819454836, -0.29863698760770385, 0.1189021909798215],
    [-0.08859552362445519, -0.28678176329573657, 0.11858161711592449],
    [-0.07718920223150749, -0.29058243084390317, 0.1191763705725993]])
T = np.array([
    [-0.13660845850725178, 0.9442777673613684, -0.2994622298910196, -0.06526290570566784],
    [-0.46826351609141803, -0.32794429192357155, -0.820476581562478, -0.23053238902495676],
    [-0.8729647235495686, 0.028143195656795905, 0.48697079170751967, 0.22130787481836772],
    [0., 0., 0., 1.]])
K = np.array([[395.7682800292969, 0., 317.9961242675781],
              [0., 395.6027526855469, 244.833984375], [0., 0., 1.]])


def projected(T=T, vertices=VERTICES, K=K):
    return np.asarray(project_rgb_observation_vertices(vertices,T_base_camera=T,
        camera_matrix=K,image_shape=(480,640),working_depth_range_m=(.07,.5)))


def test_real_noninteger_projection_reproduces_scalar_formula_digest_failure():
    pixels=projected()
    camera=(VERTICES-T[:3,3])@T[:3,:3]
    obsolete=np.c_[K[0,2]-K[0,0]*camera[:,1]/camera[:,0],
                   K[1,2]-K[1,1]*camera[:,2]/camera[:,0]]
    policy=CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.125)
    correct=projected_rgb_search_policy(policy,pixels,(480,640))
    wrong=projected_rgb_search_policy(policy,obsolete,(480,640))
    assert np.max(np.abs(pixels-obsolete))<1e-12
    assert digest(asdict(correct)) != digest(asdict(wrong))
    assert correct.center_xy[0]==0.4992187500000001
    assert correct.radius_fraction==0.055365342774659675
    assert correct.min_local_contrast==policy.min_local_contrast
    assert correct.ambiguity_score_margin==policy.ambiguity_score_margin


def test_saved_accepted_postplan_policy_matches_bit_for_bit_without_volume_rebuild():
    transform=T.copy();transform[2,3]=0.2213078748183677
    policy=projected_rgb_search_policy(CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.125),
        projected(transform),(480,640))
    expected=replace(CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.125),
        center_xy=(0.49921875,0.4520833333333332),radius_fraction=0.05536534277465972)
    assert digest(asdict(policy))==digest(asdict(expected))


@pytest.mark.parametrize('change',['behind','outside_image','outside_depth','invalid_shape','nonfinite'])
def test_shared_projection_keeps_all_original_frustum_checks(change):
    transform=T.copy();vertices=VERTICES.copy()
    if change=='behind': transform[:3,3]+=T[:3,0]
    elif change=='outside_image': transform[:3,3]+=T[:3,1]
    elif change=='outside_depth': transform[:3,3]-=T[:3,0]
    elif change=='invalid_shape': vertices=vertices[:7]
    elif change=='nonfinite': vertices[0,0]=np.nan
    with pytest.raises(ValueError): projected(transform,vertices)


@pytest.mark.parametrize('pixels,shape',[(np.zeros((7,2)),(480,640)),
    (np.full((8,2),np.nan),(480,640)),(np.zeros((8,2)),(True,640)),
    (np.zeros((8,2)),(480,0))])
def test_projected_policy_rejects_malformed_input(pixels,shape):
    with pytest.raises(ValueError):
        projected_rgb_search_policy(CenterSearchPolicy(center_xy=(.5,.5)),pixels,shape)
