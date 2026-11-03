"""The frozen failed task needs an independently measured interior direction."""
from dataclasses import replace
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))

from alicia_flexible_grasp.grasp import gripper_geometry as gg
from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


@pytest.fixture
def frozen():
    path=ROOT/'tests/fixtures/gen54_g53_measured_aperture.json.gz'
    data=json.loads(gzip.decompress(path.read_bytes()))
    assert data['source_audit_sha256']=='a3a4b24421979d13d6476f94154d120580ae6efa04e22a313953a8774baad38e'
    assert data['surface_reference']['object_sha256']=='53eaadaeefdf11b19157de2413446bbfe755cc6d7f4e94e16ec39ec82674c023'
    g=data['geometry'];s=data['surface']
    geometry=GeometryEstimate(True,'','',np.array(g['obb_center_base_m']),np.array(g['R_base_obb']),
        np.array(g['obb_size_xyz_m']),np.array(g['support_normal_base']),g['support_offset_m'],
        g['support_inlier_ratio'],np.array(g['object_points_base_m']),g['source_mode'])
    surface=FusedTargetSurface(TargetTrackIdentity(**s['identity']),np.array(s['points_base']),
        np.array(s['view_indices']),tuple(s['view_stamps_ns']),tuple(tuple(row)for row in s['view_sources']))
    gripper=gg.GripperGeometry(.05,.002,np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003)
    config=tg.TabletopGeometryConfig(angle_step_deg=5.,angle_dedup_deg=1.,
        opening_fit_clearance_each_side_m=.0005,width_projection_trim_fraction=.01,
        max_candidates=32)
    return geometry,surface,gripper,config


def generate(frozen,refine=True):
    g,s,gripper,config=frozen
    return tg.generate_tabletop_proposals(g.object_points_base,g.center_base,g.axes_base,g.size_xyz_m,
        -g.support_offset_m*g.support_normal_base,g.support_normal_base,config,
        fused_surface=s,finger_geometry=gripper,refine_obb_neighbor_angles=refine)


def test_frozen_aperture_gap_gets_fresh_measured_direction_without_replacing_originals(frozen):
    g,s,gripper,config=frozen
    before=hashlib.sha256(s.points_base.tobytes()).hexdigest()
    result=generate(frozen)
    added=[p for p in result.proposals if 'open_aperture_refinement' in p.audit]
    assert len(result.proposals)==26 and len(added)==1
    p=added[0];proof=p.audit['open_aperture_refinement']
    assert proof['original_bracket_angles_deg']==[65.,70.]
    assert p.angle_deg==pytest.approx(sum(proof['original_bracket_angles_deg'])/2.)
    assert proof['original_bracket_bilateral_ready']==[False,True]
    assert proof['motion_authorized'] is False and proof['collision_verified'] is False
    evidence,bounds=gg.evaluate_bilateral_surface_evidence_and_bounds(s,g.center_base,
        p.jaw_axis_base,-g.support_normal_base,gripper,12,support_normal_base=g.support_normal_base)
    assert evidence.ok and bounds is not None
    assert min(evidence.negative_jaw_points,evidence.positive_jaw_points)>=12
    assert p.required_open_width_m==pytest.approx(evidence.measured_width_m+2*config.opening_fit_clearance_each_side_m)
    assert p.audit['open_envelope_aperture_margin_m']>0.
    # Finite fingers may safely grasp part of a longer object. A negative
    # whole-projection margin must not become a new hard collision gate.
    wide=next(p for p in result.proposals if p.angle_deg==pytest.approx(70.))
    assert wide.audit['open_envelope_aperture_margin_m']<0.
    assert wide.required_open_width_m<wide.audit['open_obb_projection_width_m']
    assert hashlib.sha256(s.points_base.tobytes()).hexdigest()==before
    assert s.points_base.flags.writeable is False


def test_refinement_requires_the_existing_qualified_supplementary_path(frozen):
    result=generate(frozen,refine=False)
    assert result.ok
    assert all('open_aperture_refinement' not in p.audit for p in result.proposals)
    assert 67.5 not in result.sampled_angles_deg


def test_repeated_top_pixels_cannot_gain_bilateral_height_from_refinement(frozen):
    g,s,gripper,config=frozen
    n=g.support_normal_base
    points=s.points_base.copy()
    points+=(.02-g.support_offset_m-points@n)[:,None]*n
    surface=replace(s,points_base=points)
    result=generate((g,surface,gripper,config))
    assert not result.ok
    assert result.failure_code=='BILATERAL_SURFACE_EVIDENCE_MISSING'
    assert 67.5 not in result.sampled_angles_deg


def test_refinement_uses_relative_geometry_not_the_recorded_world_position(frozen):
    g,s,gripper,config=frozen
    delta=np.array([.142,-.071,.023])
    translated=replace(g,center_base=g.center_base+delta,
        object_points_base=g.object_points_base+delta,
        support_offset_m=g.support_offset_m-float(delta@g.support_normal_base))
    shifted=replace(s,points_base=s.points_base+delta)
    before=generate(frozen);after=generate((translated,shifted,gripper,config))
    assert after.sampled_angles_deg==pytest.approx(before.sampled_angles_deg)
    old=next(p for p in before.proposals if 'open_aperture_refinement' in p.audit)
    new=next(p for p in after.proposals if 'open_aperture_refinement' in p.audit)
    assert new.required_open_width_m==pytest.approx(old.required_open_width_m,abs=1e-12)
    assert new.audit['open_envelope_aperture_margin_m']==pytest.approx(old.audit['open_envelope_aperture_margin_m'],abs=1e-12)
    np.testing.assert_allclose(new.contact_center_base,old.contact_center_base+delta,atol=1e-12,rtol=0.)


def test_local_bracket_crosses_the_unoriented_jaw_angle_seam():
    normal=np.array([0.,0.,1.])
    angles=[(a,np.array([np.cos(np.deg2rad(a)),np.sin(np.deg2rad(a)),0.]))
            for a in (0.,175.)]
    facts=[dict(angle_deg=0.,open_envelope_aperture_margin_m=.001,bilateral_ready=False),
           dict(angle_deg=175.,open_envelope_aperture_margin_m=-.001,bilateral_ready=True)]
    result=list(tg._measured_aperture_directions(angles,facts,normal,tg.TabletopGeometryConfig(),True))
    assert len(result)==3
    angle,axis,audit=result[-1]
    assert angle==pytest.approx(177.5)
    assert audit['original_bracket_angles_deg']==[175.,0.]
    assert abs(float(axis@normal))<1e-12
    assert np.linalg.norm(axis)==pytest.approx(1.)


def test_multiple_local_brackets_keep_a_fixed_fresh_measurement_budget():
    normal=np.array([0.,0.,1.])
    angles=[(float(a),np.array([np.cos(np.deg2rad(a)),np.sin(np.deg2rad(a)),0.]))
            for a in range(0,180,5)]
    facts=[dict(angle_deg=angle,open_envelope_aperture_margin_m=.001 if i%2==0 else -.001,
                bilateral_ready=bool(i%2)) for i,(angle,_) in enumerate(angles)]
    result=list(tg._measured_aperture_directions(angles,facts,normal,tg.TabletopGeometryConfig(),True))
    added=[row for row in result if row[2] is not None]
    assert len(added)==4
    assert len(result)==len(angles)+4
    assert len({round(row[0],12) for row in result})==len(result)
    assert all(row[2]['independent_measurement_required'] for row in added)
