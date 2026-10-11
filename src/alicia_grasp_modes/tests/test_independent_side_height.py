"""Missing-side diagnosis retains side heights before the contact intersection.

The synthetic asymmetry is an exact software counterexample. GEN9 replay below
uses the recorded current frame only, not the unavailable complete fused cloud.
"""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from tf.transformations import euler_matrix

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _bilateral_surface_measurement,
    evaluate_bilateral_surface_evidence_and_bounds, insufficient_measured_common_height)
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_grasp_modes.observe_return_visual import measured_bilateral_view_gaps

FIXTURES = Path(__file__).parent / 'fixtures'
FINGER = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                        np.array([.1175, .155, .0774]), .003)
CASES = ('negative_thin', 'positive_thin', 'both_thin', 'both_sufficient', 'disjoint', 'outside_footprint', 'common_sparse')
ROTATIONS = ((0., 0., 0.), (.23, -.31, .68))


def sample(case, rotation=ROTATIONS[0]):
    thin = np.linspace(.009, .0096, 101)
    thick = np.linspace(-.009, .014, 231)
    negative, positive = {
        'negative_thin': (thin, thick), 'positive_thin': (thick, thin),
        'both_thin': (thin, thin), 'both_sufficient': (thick, thick),
        'disjoint': (np.linspace(-.014, -.004, 101), np.linspace(.004, .014, 101)),
        'outside_footprint': (thin, thick),
        'common_sparse': (thin, np.linspace(-.009, .014, 24)),
    }[case]
    points = np.array([(sign*.02, y, z) for sign,zs in ((-1,negative),(1,positive))
                       for y in np.linspace(-.018, .018, 11) for z in zs])
    if case == 'outside_footprint': points[:,1] += .1
    r = euler_matrix(*rotation)[:3,:3]
    center = np.array([.1, -.2, .3]) if any(rotation) else np.zeros(3)
    points = points @ r.T + center
    surface = FusedTargetSurface(TargetTrackIdentity.from_stream(1, 1), points,
                                 np.zeros(len(points), dtype=np.int64), (1,))
    g = NS(center_base=center, axes_base=r, size_xyz_m=np.array([.04,.04,.028]),
           support_normal_base=r[:,2])
    return surface, g, r[:,0]


def measurement(surface, g, jaw, normal=True):
    return _bilateral_surface_measurement(surface, g.center_base, jaw,
        -g.support_normal_base, FINGER, 12, g.support_normal_base if normal else None)


def signature(result):
    counts, low, high, masks = result
    mask_rows = list(masks) if isinstance(masks, tuple) else [masks]
    return dict(counts=list(counts) if counts else None,
                low_hex=None if low is None else float(low).hex(),
                high_hex=None if high is None else float(high).hex(),
                mask_container='tuple' if isinstance(masks, tuple) else type(masks).__name__,
                mask_sha256=[None if m is None else hashlib.sha256(m.tobytes()).hexdigest() for m in mask_rows])


def deficit(surface, g, jaw):
    counts, low, high, _ = measurement(surface,g,jaw)
    normal=g.support_normal_base
    basis=np.array([1.,0.,0.]) if abs(normal[0])<=.9 else np.array([0.,1.,0.])
    basis-=normal*(basis@normal);basis/=np.linalg.norm(basis)
    angle=float(np.degrees(np.arctan2(jaw@np.cross(normal,basis),jaw@basis))%180.)
    row=dict(angle_deg=angle, negative_reach_m=.0205, positive_reach_m=.0205,
             maximum_side_reach_m=.025, failure_code='BILATERAL_SURFACE_EVIDENCE_MISSING')
    table=dict(plan_phase='CONTACT_EXECUTION_PLAN', contact_execution_gate_deferred=False,
               required_contact_patch_overlap_m=.002, surface_evidence_rejections=[])
    if low is not None and high is not None:
        table['contact_boundary_profiles']=[dict(row, contact_height_bounds_source='fused_measured_bilateral_surface',
                                                 contact_height_bounds_m=[-high,-low])]
    else:
        table['surface_evidence_rejections']=[row]
    return measured_bilateral_view_gaps(dict(tabletop_geometry=table),g,
        TabletopGeometryConfig(opening_fit_clearance_each_side_m=.0005),surface,FINGER,
        g.center_base+g.axes_base@np.array([0.,-.1,.1]),include_thin=True)


@pytest.mark.parametrize('case', CASES)
@pytest.mark.parametrize('rotation_index', [0,1])
def test_original_contact_outputs_exactly_match_frozen_baseline(case, rotation_index):
    surface,g,jaw=sample(case,ROTATIONS[rotation_index])
    expected=json.loads((FIXTURES/'independent_side_contact_baseline.json').read_text())
    assert signature(measurement(surface,g,jaw))==expected['cases'][case+':'+str(rotation_index)]
    # Public contact consumer must still return the same measured bounds.
    _,low,high,_=measurement(surface,g,jaw)
    _,bounds=evaluate_bilateral_surface_evidence_and_bounds(surface,g.center_base,jaw,
        -g.support_normal_base,FINGER,12,support_normal_base=g.support_normal_base)
    assert bounds == (None if low is None else (low,high))


@pytest.mark.parametrize('case,missing', [('negative_thin',[-1]),('positive_thin',[1]),('both_thin',[-1,1])])
@pytest.mark.parametrize('rotation_index', [0,1])
def test_missing_side_uses_independent_height_and_contact_stays_rejected(case, missing, rotation_index):
    surface,g,jaw=sample(case,ROTATIONS[rotation_index]);before=surface.points_base.tobytes()
    gaps=deficit(surface,g,jaw)
    assert len(gaps)==1
    gap=gaps[0]
    assert gap['missing_side_signs']==missing
    assert gap['side_measurement_basis']=='before_common_support_intersection'
    assert gap['contact_or_motion_authorized'] is False
    assert 0<gap['measured_common_height_m']<.002
    assert insufficient_measured_common_height((0,gap['measured_common_height_m']),.002)
    if case!='both_thin':
        weak='negative' if case=='negative_thin' else 'positive'
        strong='positive' if case=='negative_thin' else 'negative'
        assert gap[weak+'_measured_height_m']==pytest.approx(.00054)
        assert gap[strong+'_measured_height_m']==pytest.approx(.0208)
    assert surface.points_base.tobytes()==before


def test_sufficient_bilateral_contact_does_not_create_observation_gap():
    surface,g,jaw=sample('both_sufficient')
    assert deficit(surface,g,jaw)==()


def test_unavailable_support_normal_has_identical_no_evidence_result():
    surface,g,jaw=sample('negative_thin')
    assert measurement(surface,g,jaw,False)==(None,None,None,None)


def test_gen9_saved_current_frame_keeps_actual_side_height_before_intersection():
    d=json.loads((FIXTURES/'gen9_saved_current_frame_side_height.json').read_text())
    assert d['scope']=='recorded_current_frame_only_not_complete_fused_surface'
    raw=d['replay_geometry'];points=np.array(raw['object_points_base_m'])
    surface=FusedTargetSurface(TargetTrackIdentity.from_stream(1,4),points,
                               np.zeros(len(points),dtype=np.int64),(d['snapshot_stamp_ns'],))
    g=NS(center_base=np.array(raw['obb_center_base_m']),axes_base=np.array(raw['R_base_obb']),
         size_xyz_m=np.array(raw['obb_size_xyz_m']),support_normal_base=np.array(raw['support_normal_base']))
    gaps=measured_bilateral_view_gaps(d['remote_diagnostics'],g,
        TabletopGeometryConfig(opening_fit_clearance_each_side_m=.0005),surface,FINGER,
        np.array(d['camera_origin']),include_thin=True)
    gap=min(gaps,key=lambda x:x['required_open_width_m'])
    assert gap['angle_deg']==pytest.approx(48.96698211211259)
    assert gap['negative_measured_points']==156 and gap['positive_measured_points']==354
    assert gap['negative_measured_height_m']==pytest.approx(.000898257931337845)
    assert gap['positive_measured_height_m']==pytest.approx(.013471463164848559)
    assert gap['missing_side_signs']==[-1]
    assert gap['measured_common_height_m']<.002
    assert not gap['contact_or_motion_authorized']


@pytest.mark.parametrize('rotation_index', [0,1])
def test_independent_side_points_survive_insufficient_common_band(rotation_index):
    surface,g,jaw=sample('common_sparse',ROTATIONS[rotation_index])
    counts,lower,upper,_=measurement(surface,g,jaw)
    assert min(counts[:2])<12
    assert lower is None and upper is None
    gaps=deficit(surface,g,jaw)
    assert len(gaps)==1 and gaps[0]['missing_side_signs']==[-1]
    assert gaps[0]['negative_measured_points']>=12
    assert gaps[0]['positive_measured_points']>=12
    assert gaps[0]['negative_measured_height_m']<.002
    assert gaps[0]['positive_measured_height_m']>.01
    assert gaps[0]['measured_common_height_m'] is None
    assert not gaps[0]['contact_or_motion_authorized']
