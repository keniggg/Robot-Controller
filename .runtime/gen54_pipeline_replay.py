"""Exact GEN53 pixel qualification and measured candidate evidence; offline only."""
import gzip, hashlib, json, os, platform, resource, signal, struct, sys, time
from pathlib import Path
from dataclasses import asdict, replace
from types import SimpleNamespace
import extract_gen50_retry_r30_rgb_20261018 as boundary

W = Path(__file__).resolve().parents[1]
ROOT = boundary.N/'verification/gen54_geometry_pipeline_r1'
INPUT = ROOT/'exact_inputs'
OUT = ROOT/'pure_replay_r3'

def put(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')

def main():
    assert boundary.E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
        OMP_NUM_THREADS='1', TMPDIR=str(OUT))
    sys.dont_write_bytecode = True
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native)+(native.machine,)))
    platform.architecture = lambda *a, **kw: (str(8*struct.calcsize('P'))+'bit', 'ELF')
    boundary.OUT = OUT
    sys.addaudithook(boundary.audit)
    signal.signal(signal.SIGALRM, lambda *a: (_ for _ in ()).throw(TimeoutError('90-second offline budget')))
    signal.alarm(90)
    import numpy as np, cv2, yaml
    cv2.setNumThreads(1)
    sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.vision.multiview_surface import FusedTargetSurface, SurfaceView
    from alicia_flexible_grasp.vision.fused_scene_geometry import _array_digest
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
    receipt = json.loads((INPUT/'result.json').read_text())
    ap = Path(receipt['audit']['path'])
    assert boundary.sha(ap) == receipt['audit']['sha256']
    a = json.loads(ap.read_text())
    g = a['replay_geometry']; f = a['multiview_surface']['fused_scene_geometry']
    geometry = GeometryEstimate(True, '', '', np.array(g['obb_center_base_m']),
        np.array(g['R_base_obb']), np.array(g['obb_size_xyz_m']),
        np.array(g['support_normal_base']), g['support_offset_m'], g['support_inlier_ratio'],
        np.array(g['object_points_base_m']), g['source_mode'])
    raw = replace(geometry, center_base=np.array(f['old_center_base_m']),
        size_xyz_m=np.array(f['old_size_xyz_m']),
        object_points_base=geometry.object_points_base[:f['current_raw_point_count']])
    assert _array_digest(raw.object_points_base) == f['current_raw_points_sha256']
    context = json.loads((INPUT/'original_nonimage_context.json').read_text())
    camera = next(r['parsed_json']['parameters'] for r in context
                  if r['topic']=='/grasp_record/camera_runtime_metadata')
    cam = SimpleNamespace(**{k: camera[k] for k in ('width','height','fx','fy','cx','cy','depth_scale')})
    stamp = a['snapshot_stamp_ns']
    identity = TargetTrackIdentity(a['target_epoch'], a['multiview_surface']['target_track_id'])
    arrays = {k: np.load(INPUT/(str(stamp)+'_'+k+'.npy'), allow_pickle=False)
              for k in ('color','depth','mask')}
    for k, v in arrays.items():
        assert hashlib.sha256(v.tobytes()).hexdigest() == receipt['final_retained_pixels'][k]['expected_retained_sha256']
    results = {}
    start = time.monotonic()
    hole_rows=[]
    box_source=Path(box.__file__).read_text().splitlines()
    occupancy_line=next(i+1 for i,s in enumerate(box_source) if s.strip().startswith('if occupancy <'))
    def trace(frame,event,arg):
        if (event=='line' and frame.f_code.co_filename==box.__file__
                and frame.f_code.co_name=='_qualify_regular_box_top'
                and frame.f_lineno==occupancy_line):
            z=frame.f_locals
            if z.get('cuboid') is not None:
                regions=[]
                for index in range(1,z['count']):
                    component=z['labels']==index
                    if np.any(component & z['dilated_boundary']):continue
                    yy,xx=np.where(component)
                    depth=z['depth'][yy,xx].astype(float)*z['scale']
                    valid=np.isfinite(depth)&(depth>0.)
                    points=np.column_stack(((xx-z['K'][2])*depth/z['K'][0],
                        (yy-z['K'][3])*depth/z['K'][1],depth))@z['T'][:3,:3].T+z['T'][:3,3]
                    distance=(points-z['origin'])@z['tn']
                    uncertainty=float(z['cuboid']['height_uncertainty'])
                    compatible=valid&(np.abs(distance)<=uncertainty)
                    contrad=valid&~compatible
                    regions.append(dict(pixel_count=len(yy),
                        unknown_count=int((~valid).sum()),compatible_count=int(compatible.sum()),
                        measured_recess_or_occluder_count=int(contrad.sum()),
                        compatible_height_uncertainty_m=uncertainty,
                        measured_plane_distance_min_max_m=[float(distance[valid].min()),float(distance[valid].max())] if valid.any() else None,
                        raw_unknown_pixels_yx=np.column_stack((yy[~valid],xx[~valid])).tolist(),
                        measured_compatible_pixels_yx=np.column_stack((yy[compatible],xx[compatible])).tolist(),
                        measured_recess_or_occluder_pixels_yx=np.column_stack((yy[contrad],xx[contrad])).tolist()))
                hole_rows.append(dict(geometry=label,interior_pixels=int(z['interior'].sum()),
                    occupancy=z['occupancy'],maximum_hole_fraction=z['hole_fraction'],
                    regions=sorted(regions,key=lambda x:-x['pixel_count']),
                    gate_thresholds_unmodified=True,
                    classification_scope='Original unmodified qualifier local values, before occupancy guard; existing outer-branch measured/unknown classification using current cuboid empirical height uncertainty. Diagnostic only.'))
        return trace
    sys.settrace(trace)
    for label, geo in [('raw',raw),('fused',geometry)]:
        began = time.monotonic()
        q = box.qualify_regular_box_top(arrays['depth'], arrays['mask'], cam, cam.depth_scale,
            np.array(a['snapshot_transform']['T_base_optical']), geo, identity, stamp,
            'instance_mask', color_bgr=arrays['color'])
        p = OUT/(label+'_qualification.json')
        put(p, dict(ok=q.ok,code=q.code,audit=_plain(dict(q.audit)),
            model=q.model.to_evidence_dict() if q.model else None))
        results[label] = dict(ok=q.ok, code=q.code, reason=q.audit.get('reason'),
            occupancy=q.audit.get('top_occupancy_fraction'),
            hole_fraction=q.audit.get('maximum_internal_hole_fraction'),
            model_available=q.model is not None, geometry_sha256=box.regular_box_geometry_sha256(geo),
            result_path=str(p), result_sha256=boundary.sha(p), elapsed_sec=time.monotonic()-began)
    sys.settrace(None)
    put(OUT/'interior_hole_classification.json',hole_rows)
    ref = f['surface_replay']['references']['snapshot']
    blob = (Path(ref['store_root'])/ref['relative_path']).read_bytes()
    data = gzip.decompress(blob)
    assert hashlib.sha256(blob).hexdigest()==ref['gzip_sha256']
    assert hashlib.sha256(data).hexdigest()==ref['object_sha256']
    saved = json.loads(data)
    surface = FusedTargetSurface(identity, np.array(saved['points_base']),
        np.array(saved['view_indices'],dtype=np.int64),tuple(saved['view_stamps_ns']),
        tuple(tuple(row) for row in saved['view_sources']))
    reference = SurfaceView(identity=identity,
        **{k:v for k,v in saved['reference'].items() if k!='identity'})
    assert _array_digest(surface.points_base)==f['fused_points_sha256']
    cfgpath = W/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
    remote = yaml.safe_load(cfgpath.read_text())['grasp_6d']['remote']
    gc=remote['gripper_geometry']
    tc = remote['tabletop_geometry_candidates']
    gripper = gg.GripperGeometry(gc['max_inner_gap_m'],gc['width_safety_margin_per_side_m'],
        np.array(gc['finger_box_xyz_m']),np.array(gc['palm_box_xyz_m']),tc['min_finger_support_clearance_m'])
    config = tg.TabletopGeometryConfig(**{k:v for k,v in tc.items()
        if k in tg.TabletopGeometryConfig.__dataclass_fields__})
    fit = remote['gripper_geometry']['opening_fit_clearance_each_side_m']
    config = replace(config, opening_fit_clearance_each_side_m=fit)
    # Match the producer's exact frozen current support and axes.
    n = geometry.support_normal_base
    gen = tg.generate_tabletop_proposals(geometry.object_points_base, geometry.center_base,
        geometry.axes_base,geometry.size_xyz_m,-geometry.support_offset_m*n,n,config,
        fused_surface=surface,finger_geometry=gripper,refine_obb_neighbor_angles=True)
    archive = a['remote_diagnostics']['tabletop_geometry']
    angles = tg._sample_angles(geometry.axes_base,n,config,refine_neighbors=True)
    evidence_rows=[]
    for angle,jaw in angles:
        measured,bounds = gg.evaluate_bilateral_surface_evidence_and_bounds(surface,
            geometry.center_base,jaw,-n,gripper,minimum_points_per_side=12,support_normal_base=n)
        details = gg._bilateral_surface_measurement_details(surface,
            geometry.center_base,jaw,-n,gripper,12,n)
        row = dict(angle_deg=angle,evidence=asdict(measured),bounds_m=bounds,
            full_obb_projection_width_m=float(np.abs(geometry.axes_base.T@jaw)@geometry.size_xyz_m),
            center_line_interval_m=gg._obb_line_interval(geometry.center_base,jaw,
                geometry.center_base,geometry.axes_base,geometry.size_xyz_m),
            proposal_created=any(abs(p.angle_deg-angle)<1e-8 for p in gen.proposals))
        side_masks=details[4]
        if side_masks is not None:
            delta=surface.points_base-geometry.center_base
            row['independent_sides']=[]
            for mask in side_masks:
                pts=delta[mask]@n
                row['independent_sides'].append(dict(count=int(mask.sum()),
                    support_bounds_m=np.quantile(pts,[.02,.98]).tolist() if len(pts) else None,
                    point_source_counts={str(k):int(np.count_nonzero(surface.view_indices[mask]==k)) for k in range(len(surface.view_stamps_ns))}))
        evidence_rows.append(row)
    put(OUT/'measured_directions.json', dict(rows=evidence_rows, config=asdict(config),
        gripper=_plain(asdict(gripper)),source_surface_reference=ref,
        generation=dict(proposal_count=len(gen.proposals),sampled_angles_deg=gen.sampled_angles_deg,
            proposals=[dict(source_index=p.source_index,angle_deg=p.angle_deg,
                required_open_width_m=p.required_open_width_m,audit=dict(p.audit))for p in gen.proposals],
            surface_evidence_rejections=list(gen.surface_evidence_rejections)),
        archived_proposal_count=archive['proposal_count'],
        configuration_scope='Frozen deployed YAML; identical sampled angles, proposal count and missing-evidence directions checked against original audit'))
    assert len(gen.proposals)==archive['proposal_count']
    assert np.allclose(gen.sampled_angles_deg,archive['sampled_angles_deg'],atol=1e-12,rtol=0.)
    assert [r['angle_deg']for r in gen.surface_evidence_rejections] == [r['angle_deg']for r in archive['surface_evidence_rejections']]
    provenance={}
    for name,module in list(sys.modules.items()):
        if name.startswith('alicia_flexible_grasp') and getattr(module,'__file__',None):
            p=Path(module.__file__).resolve();assert W in p.parents
            provenance[name]=dict(path=str(p),sha256=boundary.sha(p))
    result=dict(passed=True,diagnostic_only=True,execution_authorized=False,
        real_grasp_success=False,qualification=results,raw_registration_box_unavailable=True,
        generation_replayed=True,model_thresholds_unmodified=True,
        recorded_fused_surface_replayed=True,full_pipeline_or_execution_replayed=False,
        source_provenance=provenance,config_sha256=boundary.sha(cfgpath),
        source_audit_sha256=boundary.sha(ap),inputs_receipt_sha256=boundary.sha(INPUT/'result.json'),
        helper_sha256=boundary.sha(__file__),elapsed_sec=time.monotonic()-start,
        maximum_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        cpu_user_sec=resource.getrusage(resource.RUSAGE_SELF).ru_utime,
        cpu_system_sec=resource.getrusage(resource.RUSAGE_SELF).ru_stime,
        external_boundary_attempts=boundary.BOUNDARY,ROS_initialized=False,
        hardware_contacted=False,production_modified=False)
    assert not boundary.BOUNDARY
    put(OUT/'result.json',result)
    signal.alarm(0)
    print(json.dumps(dict(result=str(OUT/'result.json'),qualification=results,
        proposal_count=len(gen.proposals),elapsed_sec=result['elapsed_sec']),ensure_ascii=False))

if __name__=='__main__':main()
